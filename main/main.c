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


// Cấu hình Wifi và MQTT Broker
#define WIFI_SSID      "Hp"
#define WIFI_PASS      "11111111"
#define MQTT_BROKER_URI "mqtts://132401d4e07649638b5a848c17540a65.s1.eu.hivemq.cloud:8883" // Broker public để test
#define MQTT_TOPIC     "esp32/conveyor/data"

// ================== PIN MAP ==================
#define IRSENSOR_GPIO   GPIO_NUM_16
#define IRSENSOR_GPIO1   GPIO_NUM_17
#define IRSENSOR_GPIO2   GPIO_NUM_23
#define BTN_GPIO        GPIO_NUM_33
#define BTN_GPIO1       GPIO_NUM_32
#define USE_PULLUP      1

// I2C for TCS34725 + PCA9685
#define I2C_PORT        I2C_NUM_1
#define I2C_SDA         GPIO_NUM_18
#define I2C_SCL         GPIO_NUM_19
#define I2C_FREQ        100000

// Conveyor settingsg
#define CONVEYOR_SPEED_PERCENT  20
#define CONVEYOR_TIMEOUT_MS     5000
#define IR_ACTIVE_LEVEL         0      // IR module thường active-low

static const char *TAG = "main";

// ================== GLOBAL HANDLERS ==================
static TaskHandle_t button_task_handler = NULL;

bool is_running = false;
uint8_t counter[3] = {0}; // 0=RED,1=GREEN,2=BLUE
float speed_buffer[] = {0, 6.0, 10.0, 15.0}; // RPS // PID speed buffer
uint8_t speed_idx = 0; // index trong speed_buffer
uint8_t target_idx = 0;
volatile float g_measured_rps = 0.0f; // tốc độ hiện tại đo từ encoder
bool was_stopped = true; // trạng thái trước đó của băng tải

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
volatile bool g_start_request = false;   // manual: bấm START để chạy 1 chu kỳ
typedef enum { COL_RED=0, COL_GREEN=1, COL_BLUE=2, COL_UNKNOWN=3 } color_t;
// ================== PROTOTYPES ==================
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

// ================== APP MAIN ==================
void app_main(void)
{
    // ISR service: chỉ install 1 lần.
    esp_err_t isr_err = gpio_install_isr_service(0);
    if (isr_err != ESP_OK && isr_err != ESP_ERR_INVALID_STATE) {
        ESP_ERROR_CHECK(isr_err);
    }

    // I2C bus init (1 lần duy nhất)
    i2c_bus_init();
    lcd_init(I2C_PORT); // khởi tạo LCD trên bus đã có
    lcd_clear();


    // init devices
    ESP_ERROR_CHECK(pca9685_init(&pca));        // PCA init (KHÔNG install i2c bên trong)
    ir_sensor_init(&ir_handler, IRSENSOR_GPIO, USE_PULLUP);
    ir_sensor_init(&ir_handler1, IRSENSOR_GPIO1, USE_PULLUP);
    ir_sensor_init(&ir_handler2, IRSENSOR_GPIO2, USE_PULLUP);
    encoder_init(&wheel_encoder);
    motor_init(&motorA);
    tcs34725_init(&tcs_handler);

    motor_set_direction(&motorA, MOTOR_DIR_FORWARD);
    motor_set_speed(&motorA, 0);
    pid_speed_init(&pid,
        1.0f,   // Kp
        0.1f,    // Ki
        0.0f,    // Kd (0)
       -3.0f,
        1.0f
    );
    // testing push data to MQTT
    wifi_init_sta(WIFI_SSID, WIFI_PASS);
    mqtt_app_start(
        "mqtts://132401d4e07649638b5a848c17540a65.s1.eu.hivemq.cloud:8883",
        "conveyor",
        "Aa123456",
        "esp32/conveyor/data"
    );

    // create tasks (TẠO TASK TRƯỚC, RỒI mới gắn ISR)
    xTaskCreate(lcd_task, "lcd_task", 2048, NULL, 5, NULL);
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, &button_task_handler);
    // Init button sau khi có task handle
    button_init();
    lcd_clear();
    // Tạo các task nặng/quan trọng và ghim vào Core (Tùy chọn, nhưng tốt cho hiệu năng)
    // Core 0 thường chạy Wifi/BT, Core 1 chạy App. 
    // Nếu bạn dùng Wifi nhiều, nên để rgb_task sang Core 1 cùng với LCD để tránh xung đột ngắt Wifi.
    xTaskCreatePinnedToCore(main_task, "main_task", 4096, NULL, 8, NULL, 1);
    xTaskCreatePinnedToCore(motor_control_task, "motor_ctrl", 4096, NULL, 5, NULL, 0);
    

    ESP_LOGI(TAG, "System ready. Press button to start classification.");
    
}
static void set_servos_for_color(color_t c)
{
    switch (c) {
        case COL_RED:
            servo_close(&pca, 0);
            servo_close(&pca, 1);
            servo_close(&pca, 2);
            break;
        case COL_GREEN:
            servo_open(&pca, 0);
            servo_close(&pca, 1);
            servo_close(&pca, 2);
            break;
        case COL_BLUE:
            servo_open(&pca, 0);
            servo_open(&pca, 1);
            servo_close(&pca, 2);
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
        case COL_RED:   return 1; // 5
        case COL_GREEN: return 2; // 10
        case COL_BLUE:  return 2; // 10
        default:        return 3; // 15
    }
}

// ================== I2C INIT (CHỈ 1 LẦN) ==================
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

    // Nếu đã install rồi thì delete trước (tránh “install error”)
    // (delete fail cũng kệ, vì có thể chưa install)
    i2c_driver_delete(I2C_PORT);

    ESP_ERROR_CHECK(i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0));
    ESP_LOGI(TAG, "I2C initialized on SDA=%d SCL=%d port=%d", I2C_SDA, I2C_SCL, I2C_PORT);
}
static void IRAM_ATTR button_isr(void *arg)
{
    // ISR gọi trước khi task handle có thể tồn tại -> CHẶN NULL để khỏi assert/reset
    if (button_task_handler == NULL) return;

    BaseType_t hp = pdFALSE;
    xTaskNotifyFromISR(button_task_handler, 0x01, eSetBits, &hp);
    if (hp) portYIELD_FROM_ISR();
}
// ================== BUTTON ==================
static void button_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BTN_GPIO) | (1ULL << BTN_GPIO1),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE  // nhấn kéo xuống GND
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_ERROR_CHECK(gpio_isr_handler_add(BTN_GPIO, button_isr, NULL));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BTN_GPIO1, button_isr, NULL));
}

static void wait_ir_release(void)
{
    // chờ tất cả IR về inactive
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

        // MODE toggle
        if (gpio_get_level(BTN_GPIO1) == 0) {
            g_mode = (g_mode == MODE_MANUAL) ? MODE_AUTO : MODE_MANUAL;
            ESP_LOGI(TAG, "MODE -> %s", (g_mode==MODE_MANUAL)?"MANUAL":"AUTO");

            // khi đổi mode: reset mọi thứ về safe
            g_start_request = false;
            is_running = false;
            target_idx = 0;
            servos_default_open_all();
            wait_ir_release();
            continue;
        }

        // START manual
        if (gpio_get_level(BTN_GPIO) == 0) {
            if (g_mode == MODE_MANUAL && !is_running) {
                g_start_request = true;
                ESP_LOGI(TAG, "START requested (MANUAL)");
            } else {
                ESP_LOGI(TAG, "START ignored (AUTO or running)");
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
// ================== COLOR DETECT WITH BUFFER (GIỮ KIỂU CŨ) ==================
// return: 0=RED, 1=GREEN, 2=BLUE, 3=UNKNOWN
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
    bool auto_gate_active = false; // AUTO: đang đóng/mở theo màu hay đang default?
    servos_default_open_all();

    while (1) {
        if (g_mode == MODE_MANUAL) {

            // MANUAL: motor dừng chờ start
            target_idx = 0;
            is_running = false;
            auto_gate_active = false;
            servos_default_open_all();

            if (!g_start_request) {
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            g_start_request = false;

            // 1) nhìn màu nhiều lần khi motor đang dừng
            color_t c = detect_color_majority(10, 50);

            // 2) set servo theo màu
            set_servos_for_color(c);

            // 3) set tốc độ theo màu
            target_idx = speed_index_for_manual_color(c);

            // 4) chạy băng tải (PID task sẽ tự chạy theo target_idx)
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

            // 5) stop + reset
            is_running = false;
            target_idx = 0;
            servos_default_open_all();

        } else {
            // AUTO MODE
            is_running = true;
            target_idx = 1; // 5 rps luôn chạy

            // default servo = open all cho unknown
            if (!auto_gate_active) servos_default_open_all();

            // Nếu đang default -> nhìn nhanh 1-2 lần để quyết định gate cho vật mới
            if (!auto_gate_active) {
                color_t c = detect_color_majority(2, 5);
                if (c != COL_UNKNOWN) {
                    set_servos_for_color(c);
                    auto_gate_active = true;
                }
            }

            // Nếu IR hit -> trả servo về default open all
            if (check_ir_and_count()) {
                wait_ir_release();
                servos_default_open_all();
                auto_gate_active = false;
            }

            vTaskDelay(pdMS_TO_TICKS(20));
        }
    }
}


void motor_control_task(void *arg)
{
    while (1) {
        float target_rps = speed_buffer[target_idx];

        // đo tốc độ 1 nơi duy nhất
        float measured = encoder_get_rps(&wheel_encoder);
        g_measured_rps = measured;
        was_stopped = (measured < 0.1f);

        if (!is_running && target_rps < 0.05f) {
            pid_speed_reset(&pid);
            motor_set_speed(&motorA, 0);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        // NOTE: motor_speed_pid_step của bri đang đọc encoder bên trong.
        // Khuyến nghị: sửa motor_speed_pid_step để nhận measured (đỡ đọc 2 lần).
        // Nếu chưa sửa, vẫn chạy được, nhưng tốt nhất là sửa như ghi chú phía dưới.
        ESP_LOGI(TAG, "Target RPS: %.2f, Measured RPS: %.2f", target_rps, measured);
        motor_speed_pid_step(&motorA, &wheel_encoder, &pid, target_rps, CONTROL_DT, was_stopped, measured);

        vTaskDelay(pdMS_TO_TICKS(CONTROL_DT_MS));
    }
}
static void lcd_task(void *arg)
{
    char line0[32], line1[32];

    while (1) {
        // Line 0
        // NOTE: dùng target_idx cho target, speed_idx của bri không update
        snprintf(line0, sizeof(line0),
                 "SP:%5.2f/%5.2f   ",  // thêm spaces để clear phần dư
                 g_measured_rps, speed_buffer[target_idx]);

        // Line 1
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