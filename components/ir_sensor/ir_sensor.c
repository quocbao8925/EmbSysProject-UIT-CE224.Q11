#include "ir_sensor.h"
#include "sdkconfig.h"
#include "esp_log.h"

static const char *TAG = "IR_SENSOR";

static QueueHandle_t ir_sensor_queue = NULL;
static gpio_num_t s_gpio = -1;
static uint32_t last_isr_time = 0;
static uint8_t last_level = 0xFF;

// ISR
void IRAM_ATTR ir_sensor_isr(void *arg) {
    uint32_t pin = (uint32_t)arg;
    ir_sensor_event_t evt = {
        .gpio_num = pin,
        .level = gpio_get_level(pin)
    };
    xQueueSendFromISR(ir_sensor_queue, &evt, NULL);
}
// init 
void ir_sensor_init(ir_sensor_handler* handler, gpio_num_t gpio_num, bool use_pullup) {
    // init queue
    ir_sensor_queue = xQueueCreate(IR_SENSOR_QUEUE_SIZE, sizeof(ir_sensor_event_t));
    // init GPIO
    s_gpio = gpio_num;
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << s_gpio,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = use_pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE //
    };
    gpio_config(&io);
    // init handler
    handler->gpio_num = s_gpio;
    handler->current_level = gpio_get_level(s_gpio);
    handler->event_count = 0;
    // add ISR handler
    gpio_isr_handler_add(s_gpio, ir_sensor_isr, (void*)(uint32_t)s_gpio);

    ESP_LOGI(TAG, "IR sensor initialized on GPIO %d", s_gpio);
}

static inline void ir_sensor_inc_count(ir_sensor_handler* handler){
    handler->event_count++;
}

bool ir_sensor_wait_event(ir_sensor_event_t *evt) {
    if (xQueueReceive(ir_sensor_queue, evt, portMAX_DELAY)) {
        uint32_t now = xTaskGetTickCount();
        if (now - last_isr_time > pdMS_TO_TICKS(MS_PER_OBJ) && (evt->level != last_level)) {
            // >100ms
            last_isr_time = now;
            last_level = evt->level;
            if (evt->level == COND_TO_COUNT) {
                return true;
            }
        }
    }
    return false;
}

void ir_sensor_deinit(void)
{
    if (s_gpio >= 0) {
        gpio_isr_handler_remove(s_gpio);
        vQueueDelete(ir_sensor_queue);
        s_gpio = -1;
    }
}

void ir_sensor_counter(ir_sensor_handler* handler, ir_sensor_event_t *evt)
{
    if (ir_sensor_wait_event(evt)) {
        ir_sensor_inc_count(handler);
        ESP_LOGI(TAG, "[OBJECTS count: %d]", handler->event_count);
    }
}