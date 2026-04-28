// main.c (ESP-IDF)
#include <string.h>
#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_log.h"

#include "ir_sensor.h"
#include "tcs34725.h"
#include "pca9685.h"
#include "motor_l298n.h"
#include "encoder.h"

#include "motor_speed_control.h"
#include "i2c_lcd.h"

#include "net_mqtt.h"


#define WIFI_SSID      ""
#define WIFI_PASS      ""
#define MQTT_BROKER_URI "mqtts://132401d4e07649638b5a848c17540a65.s1.eu.hivemq.cloud:8883" // Broker public để test
#define MQTT_TOPIC     "esp32/conveyor/data"


#define IRSENSOR_GPIO   GPIO_NUM_16
#define IRSENSOR_GPIO1   GPIO_NUM_17
#define IRSENSOR_GPIO2   GPIO_NUM_23
#define BTN_GPIO        GPIO_NUM_33
#define BTN_GPIO1       GPIO_NUM_32
#define USE_PULLUP      1


#define I2C_PORT        I2C_NUM_1
#define I2C_SDA         GPIO_NUM_18
#define I2C_SCL         GPIO_NUM_19
#define I2C_FREQ        100000


#define CONVEYOR_SPEED_PERCENT  20
#define CONVEYOR_TIMEOUT_MS     5000
#define IR_ACTIVE_LEVEL         0     
#define NUM_SPEED_SAMPLES     6

static const char *TAG = "main";

static TaskHandle_t button_task_handler = NULL;

static const float auto_speed_table[] = {0.0f, 4.0f, 6.0f, 8.0f, 10.0f};
#define AUTO_SPEED_COUNT (sizeof(auto_speed_table)/sizeof(auto_speed_table[0]))

volatile uint8_t g_auto_speed_idx = 2;   
volatile float   g_target_rps     = 0.0f; 

bool is_running = false;
uint8_t counter[3] = {0}; 
float speed_buffer[] = {0, 6.0, 10.0, 15.0};
uint8_t speed_idx = 0; 
uint8_t target_idx = 0;
volatile float g_measured_rps = 0.0f; 
bool was_stopped = true; 

ir_sensor_handler ir_handler;
ir_sensor_handler ir_handler1;
ir_sensor_handler ir_handler2;
ir_sensor_event_t ir_event;
tcs34725_handler  tcs_handler;

encoder_t wheel_encoder = {
    .pin = GPIO_NUM_34,
    .pulses_per_rev = 20
};

motor_t motorA = {
    .in1_pin      = GPIO_NUM_25,
    .in2_pin      = GPIO_NUM_26,
    .en_pin       = GPIO_NUM_27,
    .pwm_channel  = LEDC_CHANNEL_0,
    .pwm_timer    = LEDC_TIMER_0,
    .pwm_freq     = 20000,
    .pwm_mode     = LEDC_LOW_SPEED_MODE
};

pid_speed_t pid ;

pca9685_t pca = {
    .i2c_port = I2C_PORT,
    .address = 0x40,
    .i2c_freq = I2C_FREQ
};
typedef enum { MODE_MANUAL=0, MODE_AUTO=1} system_mode_t;

volatile system_mode_t g_mode = MODE_MANUAL;
volatile bool g_start_request = false; 
typedef enum { COL_RED=0, COL_GREEN=1, COL_BLUE=2, COL_UNKNOWN=3 } color_t;

static void set_servos_for_color(color_t c);
static void servos_default_open_all(void);
static uint8_t speed_index_for_manual_color(color_t c);
// i2c
static void i2c_bus_init(void);
// button
static void button_init(void);
static void button_task(void *arg);
// ir sensor
static void wait_ir_release(void);
static bool check_ir_and_count(void);
// lcd task
static void lcd_task(void *arg);
// 2 types of detecting color
static color_t detect_color_once(void);
static color_t detect_color_majority(uint8_t samples, uint32_t delay_ms);
// main function to run the conveyor
static void main_task(void *arg);

// pid control for motor
void motor_control_task(void *arg);

void app_main(void)
{

    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(isr_err);
    }

    i2c_bus_init();
    lcd_init(I2C_PORT); 
    lcd_clear();


    // init devices
    ESP_ERROR_CHECK(pca9685_init(&pca));     
    ir_sensor_init(&ir_handler, IRSENSOR_GPIO, USE_PULLUP);
    ir_sensor_init(&ir_handler1, IRSENSOR_GPIO1, USE_PULLUP);
    ir_sensor_init(&ir_handler2, IRSENSOR_GPIO2, USE_PULLUP);
    encoder_init(&wheel_encoder);
    motor_init(&motorA);
    tcs34725_init(&tcs_handler);

    motor_set_direction(&motorA, MOTOR_DIR_FORWARD);
    motor_set_speed(&motorA, 0);
    pid_speed_init(&pid, 0.01, 0.002, 0.005, -1.0f, 1.0f);
    //testing push data to MQTT
    wifi_init_sta(WIFI_SSID, WIFI_PASS);
    mqtt_app_start(
        "mqtts://132401d4e07649638b5a848c17540a65.s1.eu.hivemq.cloud:8883",
        "conveyor",
        "Aa123456",
        "esp32/conveyor/data"
    );

    xTaskCreate(lcd_task, "lcd_task", 2048, NULL, 5, NULL);
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, &button_task_handler);

    button_init();
    lcd_clear();

    xTaskCreatePinnedToCore(main_task, "main_task", 4096, NULL, 8, NULL, 1);
    xTaskCreatePinnedToCore(motor_control_task, "motor_ctrl", 4096, NULL, 5, NULL, 0);
    
}
static void set_servos_for_color(color_t c)
{
    switch (c) {
        case COL_RED:
            servo_close(&pca, 0);
            servo_open(&pca, 1);
            servo_open(&pca, 2);
            break;
        case COL_GREEN:
            servo_close(&pca, 1);
            servo_open(&pca, 0);
            servo_open(&pca, 2);
            break;
        case COL_BLUE:
            servo_close(&pca, 2);
            servo_open(&pca, 0);
            servo_open(&pca, 1);
            break;
        default: // unknown
            servo_open(&pca, 0);
            servo_open(&pca, 1);
            servo_open(&pca, 2);
            break;
    }
}
static void servos_default_open_all(void)
{
    servo_open(&pca, 0);
    servo_open(&pca, 1);
    servo_open(&pca, 2);
}
static uint8_t speed_index_for_manual_color(color_t c)
{
    switch (c) {
        case COL_RED:   return 1; // 6
        case COL_GREEN: return 2; // 10
        case COL_BLUE:  return 2; // 10
        default:        return 3; // 15
    }
}

static void i2c_bus_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SCL,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ,
        .clk_flags = 0
    };

    ESP_ERROR_CHECK(i2c_param_config(I2C_PORT, &conf));

    i2c_driver_delete(I2C_PORT);

    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));
    ESP_LOGI(TAG, "I2C initialized on SDA=%d SCL=%d port=%d", I2C_SDA, I2C_SCL, I2C_PORT);
}
static void IRAM_ATTR button_isr(void *arg)
{

    if (button_task_handler == NULL) return;

    BaseType_t hp = pdFALSE;
    xTaskNotifyFromISR(button_task_handler, 0x01, eSetBits, &hp);
    if (hp) portYIELD_FROM_ISR();
}

static void button_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BTN_GPIO) | (1ULL << BTN_GPIO1),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE  
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_ERROR_CHECK(gpio_isr_handler_add(BTN_GPIO, button_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BTN_GPIO1, button_isr, NULL));
}

static void wait_ir_release(void)
{

    while (gpio_get_level(IRSENSOR_GPIO)  == IR_ACTIVE_LEVEL ||
           gpio_get_level(IRSENSOR_GPIO1) == IR_ACTIVE_LEVEL ||
           gpio_get_level(IRSENSOR_GPIO2) == IR_ACTIVE_LEVEL) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}  
static void button_task(void *arg)
{
    uint32_t bits = 0;

    while (1) {
        xTaskNotifyWait(0, UINT32_MAX, &bits, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(30)); // debounce


        if (gpio_get_level(BTN_GPIO1) == 0) {
            g_mode = (g_mode == MODE_MANUAL) ? MODE_AUTO : MODE_MANUAL;
            if (g_mode == MODE_AUTO) {
                g_target_rps = auto_speed_table[g_auto_speed_idx]; // default auto speed
            } else {
                g_target_rps = 0.0f;
            }
            ESP_LOGI(TAG, "MODE -> %s", (g_mode==MODE_MANUAL)?"MANUAL":"AUTO");

            g_start_request = false;
            is_running = false;
            target_idx = 0;
            servos_default_open_all();
            wait_ir_release();
            continue;
        }

        // START manual
        if (gpio_get_level(BTN_GPIO) == 0) {

            if (g_mode == MODE_MANUAL) {
                if (!is_running) {
                    g_start_request = true;
                    ESP_LOGI(TAG, "START requested (MANUAL)");
                } else {
                    ESP_LOGI(TAG, "START ignored (running)");
                }
            } else {
                // AUTO: BTN0 = cycle speed
                g_auto_speed_idx = (g_auto_speed_idx + 1) % AUTO_SPEED_COUNT;
                g_target_rps = auto_speed_table[g_auto_speed_idx];
                ESP_LOGI(TAG, "AUTO speed -> %.1f rps", g_target_rps);
            }

        }
    }
}
static bool check_ir_and_count(void)
{
    if (gpio_get_level(IRSENSOR_GPIO) == IR_ACTIVE_LEVEL) {
        counter[2]++; // BLUE
        send_data_to_mqtt(counter[0], counter[1], counter[2]);
        ESP_LOGI(TAG, "IR BLUE hit");
        return true;
    }
    if (gpio_get_level(IRSENSOR_GPIO1) == IR_ACTIVE_LEVEL) {
        counter[1]++; // GREEN
        send_data_to_mqtt(counter[0], counter[1], counter[2]);
        ESP_LOGI(TAG, "IR GREEN hit");
        return true;
    }
    if (gpio_get_level(IRSENSOR_GPIO2) == IR_ACTIVE_LEVEL) {
        counter[0]++; // RED
        send_data_to_mqtt(counter[0], counter[1], counter[2]);
        ESP_LOGI(TAG, "IR RED hit");
        return true;
    }
    return false;
}

static color_t detect_color_once(void)
{
    int8_t offsetr = 53, offsetg = 97, offsetb = 87;
    uint8_t threshold = 3;

    uint8_t r, g, b;
    tcs34725_reader(&tcs_handler);
    get_rgb_values(&tcs_handler, &r, &g, &b);

    // adjust
    r = (r > offsetr) ? (r - offsetr) : 0;
    g = (g > offsetg) ? (g - offsetg) : 0;
    b = (b > offsetb) ? (b - offsetb) : 0;

    if (r > g && r > b && r > threshold) return COL_RED;
    if (g > r && g > b && g > threshold) return COL_GREEN;
    if (b > r && b > g && b > threshold) return COL_BLUE;
    return COL_UNKNOWN;
}
static color_t detect_color_majority(uint8_t samples, uint32_t delay_ms)
{
    uint8_t buf[4] = {0};

    for (uint8_t i = 0; i < samples; i++) {
        color_t c = detect_color_once();
        buf[(int)c]++;

        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    int max_i = 0;
    for (int i = 1; i < 4; i++) {
        if (buf[i] > buf[max_i]) max_i = i;
    }

    ESP_LOGI(TAG, "Color vote: R=%u G=%u B=%u U=%u -> %d",
             buf[0], buf[1], buf[2], buf[3], max_i);

    return (color_t)max_i;
}

static void main_task(void *arg)
{
    bool auto_gate_active = false; 
    servos_default_open_all();

    while (1) {
        if (g_mode == MODE_MANUAL) {

            target_idx = 0;
            is_running = false;
            auto_gate_active = false;
            servos_default_open_all();

            if (!g_start_request) {
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            g_start_request = false;

            color_t c = detect_color_majority(10, 50);

            set_servos_for_color(c);

            uint8_t idx = speed_index_for_manual_color(c);
            g_target_rps = speed_buffer[idx];
            is_running = true;

            int64_t start = esp_timer_get_time();

            while (1) {
                if (check_ir_and_count()) {
                    wait_ir_release();
                    break;
                }
                int64_t now = esp_timer_get_time();
                if ((now - start) >= (int64_t)CONVEYOR_TIMEOUT_MS * 1000) {
                    ESP_LOGW(TAG, "MANUAL timeout");
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(5));
            }

            is_running = false;
            g_target_rps = 0.0f;
            servos_default_open_all();

        } else {

            if (g_target_rps < 0.01f) {
                g_target_rps = auto_speed_table[g_auto_speed_idx]; // default 6 rps
            }
            is_running = (g_target_rps >= 0.05f);

            if (!auto_gate_active) servos_default_open_all();

            if (!auto_gate_active) {
                color_t c = detect_color_majority(1, 5);
                if (c != COL_UNKNOWN) {
                    set_servos_for_color(c);
                    auto_gate_active = true;
                }
            }

            if (check_ir_and_count()) {
                wait_ir_release();
                servos_default_open_all();
                auto_gate_active = false;
            }

            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}
static uint8_t i_measured = NUM_SPEED_SAMPLES;
static float sum_measured = 0;
void motor_control_task(void *arg)
{
    while (1) {
        float target_rps = g_target_rps;

        float measured = encoder_get_rps(&wheel_encoder);
        if (i_measured){
            i_measured--;
            sum_measured += measured;
        } else {
            g_measured_rps = sum_measured / NUM_SPEED_SAMPLES;
            i_measured = NUM_SPEED_SAMPLES;
            sum_measured = 0;
            ESP_LOGI(TAG, "Target RPS: %.2f, Measured RPS: %.2f", target_rps, g_measured_rps);
        }
        was_stopped = (measured < 0.1f);

        if (target_rps < 0.05f) {
            pid_speed_reset(&pid);
            motor_set_speed(&motorA, 0);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        motor_speed_pid_step(&motorA, &wheel_encoder, &pid, target_rps,
                            CONTROL_DT, was_stopped, measured);

        vTaskDelay(pdMS_TO_TICKS(CONTROL_DT_MS));
    }
}
static void lcd_task(void *arg)
{
    char line0[32], line1[32];

    while (1) {

        snprintf(line0, sizeof(line0),
         "SP:%5.2f/%5.2f   ",
         g_measured_rps, g_target_rps);

        snprintf(line1, sizeof(line1),
                 "R:%d G:%d B:%d %c   ",
                 counter[0], counter[1], counter[2],
                 (g_mode == MODE_MANUAL) ? 'M' : 'A');

        lcd_put_cur(0, 0);
        lcd_send_string(line0);

        lcd_put_cur(1, 0);
        lcd_send_string(line1);

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}