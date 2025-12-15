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

// ================== PIN MAP ==================
#define IRSENSOR_GPIO   GPIO_NUM_16
#define BTN_GPIO        GPIO_NUM_33
#define USE_PULLUP      1

// I2C for TCS34725 + PCA9685
#define I2C_PORT        I2C_NUM_1
#define I2C_SDA         GPIO_NUM_18
#define I2C_SCL         GPIO_NUM_19
#define I2C_FREQ        400000

// Conveyor settings
#define CONVEYOR_SPEED_PERCENT  50
#define CONVEYOR_TIMEOUT_MS     5000
#define IR_ACTIVE_LEVEL         0      // IR module thường active-low

static const char *TAG = "main";

// ================== GLOBAL HANDLERS ==================
static TaskHandle_t button_task_handle = NULL;
static TaskHandle_t rgb_task_handle    = NULL;

ir_sensor_handler ir_handler;
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

pca9685_t pca = {
    .i2c_port = I2C_PORT,
    .address = 0x40,
    .i2c_freq = I2C_FREQ
};

// ================== PROTOTYPES ==================
static void i2c_bus_init(void);
static void button_init(void);
static void rgb_task(void *arg);
static void button_task(void *arg);
static void IRAM_ATTR button_isr(void *arg);

static void conveyor_run_until_ir_or_timeout(uint32_t timeout_ms);
static int  detect_majority_color_with_buffer(void);

// Nếu bạn đã có hàm servo_open/servo_close ở nơi khác thì cứ giữ.
// Ở đây mình chỉ gọi, không định nghĩa lại.
extern void servo_open(pca9685_t *dev, uint8_t ch);
extern void servo_close(pca9685_t *dev, uint8_t ch);

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

// ================== BUTTON ==================
static void button_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BTN_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE  // nhấn kéo xuống GND
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    ESP_ERROR_CHECK(gpio_isr_handler_add(BTN_GPIO, button_isr, NULL));
}

static void IRAM_ATTR button_isr(void *arg)
{
    // ISR gọi trước khi task handle có thể tồn tại -> CHẶN NULL để khỏi assert/reset
    if (button_task_handle == NULL) return;

    BaseType_t hp = pdFALSE;
    xTaskNotifyFromISR(button_task_handle, 0x01, eSetBits, &hp);
    if (hp) portYIELD_FROM_ISR();
}

static void button_task(void *arg)
{
    uint32_t bits = 0;

    while (1) {
        // chờ ISR notify
        xTaskNotifyWait(0, UINT32_MAX, &bits, portMAX_DELAY);

        // debounce mềm
        vTaskDelay(pdMS_TO_TICKS(30));
        if (gpio_get_level(BTN_GPIO) == 0) {
            ESP_LOGI(TAG, "Button pressed -> start classification");

            // notify rgb_task bắt đầu 1 lượt phân loại
            if (rgb_task_handle) {
                xTaskNotify(rgb_task_handle, 0x01, eSetBits);
            }
        }
    }
}

// ================== CONVEYOR CONTROL ==================
static void conveyor_run_until_ir_or_timeout(uint32_t timeout_ms)
{
    // chạy băng tải
    motor_set_direction(&motorA, MOTOR_DIR_FORWARD);
    motor_set_speed(&motorA, CONVEYOR_SPEED_PERCENT);

    int64_t start = esp_timer_get_time(); // us
    while (1) {
        // IR detect -> dừng
        if (gpio_get_level(IRSENSOR_GPIO) == IR_ACTIVE_LEVEL) {
            ESP_LOGI(TAG, "IR detected -> stop conveyor");
            break;
        }

        // timeout -> dừng
        int64_t now = esp_timer_get_time();
        if ((now - start) >= (int64_t)timeout_ms * 1000) {
            ESP_LOGW(TAG, "Timeout %ums -> stop conveyor", timeout_ms);
            break;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // chắc chắn dừng
    motor_set_speed(&motorA, 0);
}

// ================== COLOR DETECT WITH BUFFER (GIỮ KIỂU CŨ) ==================
// return: 0=RED, 1=GREEN, 2=BLUE, 3=UNKNOWN
static int detect_majority_color_with_buffer(void)
{
    int8_t offsetr = 53, offsetg = 97, offsetb = 87;
    uint8_t threshold = 3;

    uint8_t r, g, b;
    uint8_t buffer[4] = {0};
    uint8_t idx = 0;

    while (1) {
        tcs34725_reader(&tcs_handler);
        get_rgb_values(&tcs_handler, &r, &g, &b);

        ESP_LOGI(TAG, "Raw RGB: R=%u G=%u B=%u", r, g, b);

        // adjust
        r = (r > offsetr) ? (r - offsetr) : 0;
        g = (g > offsetg) ? (g - offsetg) : 0;
        b = (b > offsetb) ? (b - offsetb) : 0;

        // vote
        if (r > g && r > b && r > threshold) {
            buffer[0]++;
        } else if (g > r && g > b && g > threshold) {
            buffer[1]++;
        } else if (b > r && b > g && b > threshold) {
            buffer[2]++;
        } else {
            buffer[3]++;
        }

        idx++;
        if (idx >= 10) {
            // majority
            int max_idx = 0;
            for (int i = 1; i < 4; i++) {
                if (buffer[i] > buffer[max_idx]) max_idx = i;
            }

            ESP_LOGI(TAG, "Majority color: %s (R=%u G=%u B=%u U=%u)",
                     (max_idx==0)?"RED":(max_idx==1)?"GREEN":(max_idx==2)?"BLUE":"UNKNOWN",
                     buffer[0], buffer[1], buffer[2], buffer[3]);

            return max_idx;
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ================== RGB TASK (BẤM NÚT MỚI CHẠY) ==================
static void rgb_task(void *arg)
{
    uint32_t bits = 0;

    while (1) {
        // chờ “start classification” từ button_task
        xTaskNotifyWait(0, UINT32_MAX, &bits, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(100)); //

        // 1) detect màu (buffer)
        int color = detect_majority_color_with_buffer();

        // 2) mở/đóng cổng theo màu (bạn đã có code sẵn servo_open/close)
        switch (color) {
            case 0: // RED
                servo_close(&pca, 0);
                servo_close(&pca, 1);
                servo_close(&pca, 2);
                break;

            case 1: // GREEN
                servo_open(&pca, 2);
                servo_close(&pca, 1);
                servo_close(&pca, 0);
                break;

            case 2: // BLUE
                servo_open(&pca, 2);
                servo_open(&pca, 1);
                servo_close(&pca, 0);
                break;

            default: // UNKNOWN
                servo_open(&pca, 0);
                servo_open(&pca, 1);
                servo_open(&pca, 2);
                break;
        }

        // 3) chạy băng tải, tối đa 5s hoặc IR detect thì dừng
        conveyor_run_until_ir_or_timeout(CONVEYOR_TIMEOUT_MS);

        // 4) đảm bảo dừng băng tải (double safety)
        motor_set_speed(&motorA, 0);

        ESP_LOGI(TAG, "Classification cycle finished.\n");
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

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

    // init devices
    ESP_ERROR_CHECK(pca9685_init(&pca));        // PCA init (KHÔNG install i2c bên trong)
    ir_sensor_init(&ir_handler, IRSENSOR_GPIO, USE_PULLUP);
    encoder_init(&wheel_encoder);
    motor_init(&motorA);
    tcs34725_init(&tcs_handler);

    motor_set_direction(&motorA, MOTOR_DIR_FORWARD);
    motor_set_speed(&motorA, 0);

    // create tasks (TẠO TASK TRƯỚC, RỒI mới gắn ISR)
    xTaskCreate(button_task, "button_task", 2048, NULL, 10, &button_task_handle);
    xTaskCreate(rgb_task,    "rgb_task",    4096, NULL, 9,  &rgb_task_handle);

    // init button after task handles are valid
    button_init();

    ESP_LOGI(TAG, "System ready. Press button to start classification.");
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
