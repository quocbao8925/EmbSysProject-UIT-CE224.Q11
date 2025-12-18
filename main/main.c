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

#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "mqtt_client.h"
#include "cJSON.h"
#include "esp_crt_bundle.h"

// Cấu hình Wifi và MQTT Broker
#define WIFI_SSID      "B17.17"
#define WIFI_PASS      "12345678"
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

// Conveyor settings
#define CONVEYOR_SPEED_PERCENT  50
#define CONVEYOR_TIMEOUT_MS     5000
#define IR_ACTIVE_LEVEL         0      // IR module thường active-low

static const char *TAG = "main";

// ================== GLOBAL HANDLERS ==================
static TaskHandle_t button_task_handle = NULL;
static TaskHandle_t rgb_task_handle    = NULL;
static esp_mqtt_client_handle_t mqtt_client = NULL;
bool is_running = false;
uint8_t counter[3] = {0}; // 0=RED,1=GREEN,2=BLUE
float speed_buffer[] = {0.8, 1.2, 1.6}; // RPS // PID speed buffer
uint8_t speed_idx = 1; // index trong speed_buffer
volatile float g_measured_rps = 0.0f; // tốc độ hiện tại đo từ encoder

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

// ================== PROTOTYPES ==================
static void i2c_bus_init(void);
static void button_init(void);
static void rgb_task(void *arg);
static void button_task(void *arg);
static void button_isr(void *arg);

static void conveyor_run_until_ir_or_timeout(uint32_t timeout_ms);
static int  detect_majority_color_with_buffer(void);

// Nếu bạn đã có hàm servo_open/servo_close ở nơi khác thì cứ giữ.
// Ở đây mình chỉ gọi, không định nghĩa lại.
extern void servo_open(pca9685_t *dev, uint8_t ch);
extern void servo_close(pca9685_t *dev, uint8_t ch);

// MQTT client handle
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGI(TAG, "Retrying to connect to the AP");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

static void wifi_init_sta(void)
{
    nvs_flash_init(); // Khởi tạo NVS để lưu config wifi
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                        &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                        &wifi_event_handler, NULL, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    esp_wifi_start();
}

// --- 2. XỬ LÝ SỰ KIỆN MQTT ---
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT Connected");
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG, "MQTT Disconnected");
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT Error");
        break;
    default:
        break;
    }
}
static void mqtt_app_start(void)
{
    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = MQTT_BROKER_URI, // Nhớ có 's' và port 8883
        .credentials.username = "conveyor",  // User bạn vừa tạo trong Access Management
        .credentials.authentication.password = "Aa123456", // Pass bạn vừa tạo
        
        // Cấu hình chứng chỉ SSL (Quan trọng để không bị lỗi kết nối)
        .broker.verification.crt_bundle_attach = esp_crt_bundle_attach, 
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, mqtt_client);
    esp_mqtt_client_start(mqtt_client);
}
void send_data_to_mqtt(int r, int g, int b, float speed)
{
    if (mqtt_client == NULL) return;

    // Tạo JSON object: {"red": 10, "green": 5, "blue": 2, "speed": 2.5}
    cJSON *root = cJSON_CreateObject();
    cJSON_AddNumberToObject(root, "red", r);
    cJSON_AddNumberToObject(root, "green", g);
    cJSON_AddNumberToObject(root, "blue", b);
    cJSON_AddNumberToObject(root, "speed", speed);

    // Chuyển JSON object thành chuỗi (string)
    char *post_data = cJSON_PrintUnformatted(root);

    // Publish lên topic
    // tham số: client, topic, data, len, qos, retain
    esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC, post_data, 0, 1, 0);

    // Giải phóng bộ nhớ (RẤT QUAN TRỌNG ĐỂ KHÔNG BỊ MEMORY LEAK)
    cJSON_Delete(root);
    free(post_data);
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
static void IRAM_ATTR ir_sensor_isr(void *arg)
{
    // Lấy số chân GPIO đã kích hoạt ngắt
    uint32_t gpio_num = (uint32_t)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint32_t notify_value = 0;

    // Xác định xem IR nào được kích hoạt và gán cờ bit tương ứng
    // Bit 0: Blue (GPIO 16), Bit 1: Green (GPIO 17), Bit 2: Red (GPIO 23)
    if (gpio_num == IRSENSOR_GPIO) {
        notify_value = 0x01; 
    } else if (gpio_num == IRSENSOR_GPIO1) {
        notify_value = 0x02;
    } else if (gpio_num == IRSENSOR_GPIO2) {
        notify_value = 0x04;
    }

    // Gửi thông báo đến rgb_task để đánh thức nó dậy
    if (rgb_task_handle != NULL && notify_value != 0) {
        xTaskNotifyFromISR(rgb_task_handle, notify_value, eSetBits, &xHigherPriorityTaskWoken);
    }

    // Nếu task được đánh thức có độ ưu tiên cao hơn task hiện tại -> chuyển context ngay
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
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
        if (gpio_get_level(BTN_GPIO) == 0  && !is_running) {
            ESP_LOGI(TAG, "Button pressed -> start classification");

            // notify rgb_task bắt đầu 1 lượt phân loại
            if (rgb_task_handle) {
                xTaskNotify(rgb_task_handle, 0x01, eSetBits);
                is_running = true;
            }
        }
        if (gpio_get_level(BTN_GPIO1) == 0) {
            speed_idx = (speed_idx + 1) % 4;
            // hiển thị tốc độ hiện tại
            ESP_LOGI(TAG, "Speed button pressed -> speed set to %f RPS", speed_buffer[speed_idx]);}
    }
}
// ================== CONVEYOR CONTROL ==================
static void conveyor_run_until_ir_or_timeout(uint32_t timeout_ms)
{
    // chạy băng tải
    motor_set_direction(&motorA, MOTOR_DIR_FORWARD);
    motor_set_speed(&motorA, speed_buffer[speed_idx]);

    int64_t start = esp_timer_get_time(); // us
    while (1) {

        // IR detect -> dừng
        if (gpio_get_level(IRSENSOR_GPIO) == IR_ACTIVE_LEVEL) {
            ESP_LOGI(TAG, "IR detected -> stop conveyor");
            counter[2]++; // BLUE
            break;
        }
        if (gpio_get_level(IRSENSOR_GPIO1) == IR_ACTIVE_LEVEL) {
            ESP_LOGI(TAG, "IR1 detected -> stop conveyor");
            counter[1]++; // GREEN
            break;
        }
        if (gpio_get_level(IRSENSOR_GPIO2) == IR_ACTIVE_LEVEL) {
            ESP_LOGI(TAG, "IR2 detected -> stop conveyor");
            counter[0]++; // RED
            break;
        }


        // timeout -> dừng
        int64_t now = esp_timer_get_time();
        if ((now - start) >= (int64_t)timeout_ms * 1000) {
            ESP_LOGW(TAG, "Timeout %ums -> stop conveyor", timeout_ms);
            break;
        }

        //vTaskDelay(period);
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
                servo_open(&pca, 0);
                servo_close(&pca, 1);
                servo_close(&pca, 2);
                break;

            case 2: // BLUE
                servo_open(&pca, 0);
                servo_open(&pca, 1);
                servo_close(&pca, 2);
                break;

            default: // UNKNOWN
                servo_open(&pca, 2);
                servo_open(&pca, 1);
                servo_open(&pca, 0);
                break;
        }

        // 3) chạy băng tải, tối đa 5s hoặc IR detect thì dừng
        conveyor_run_until_ir_or_timeout(CONVEYOR_TIMEOUT_MS);
        
        // 4) đảm bảo dừng băng tải (double safety)
        motor_set_speed(&motorA, 0);
        is_running = false;
        ESP_LOGI(TAG, "Classification cycle finished.\n");
        ESP_LOGI(TAG, "Counters: RED=%d GREEN=%d BLUE=%d", counter[0], counter[1], counter[2]);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void lcd_task(void *arg)
{
    char buffer[32];
    char buffer1[32];
    while (1) {
        // Hiển thị thông tin lên LCD
        lcd_put_cur(0, 0);
        sprintf(buffer, "SPD: %.2f", encoder_get_rps(&wheel_encoder));
        lcd_send_string(buffer);
        lcd_put_cur(1, 0);
        sprintf(buffer1, "R: %d, G: %d, B: %d", counter[0], counter[1], counter[2]);
        lcd_send_string(buffer1);
        vTaskDelay(pdMS_TO_TICKS(500));
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
    pid_speed_init(&pid, 15.0f, 3.0f, 0.0f, 0.0f, 100.0f);
    // create tasks (TẠO TASK TRƯỚC, RỒI mới gắn ISR)

    xTaskCreate(button_task, "button_task", 2048, NULL, 10, &button_task_handle);
    xTaskCreate(rgb_task,    "rgb_task",    4096, NULL, 9,  &rgb_task_handle);
    xTaskCreate(lcd_task,    "lcd_task",    2048, NULL, 8,  NULL);
    // init button after task handles are valid
    button_init();

    ESP_LOGI(TAG, "System ready. Press button to start classification.");

    // ESP_LOGI(TAG, "Connecting to WiFi...");
    // wifi_init_sta(); 
    
    // ESP_LOGI(TAG, "Starting MQTT...");
    // mqtt_app_start();

    // // ... (Phần khởi tạo Task giữ nguyên) ...

    // char buffer[32];
    // int64_t last_mqtt_time = 0;
    // while (1) {
    //     float current_speed = encoder_get_rps(&wheel_encoder);
    //     if (esp_timer_get_time() - last_mqtt_time > 2000000) {
    //         send_data_to_mqtt(counter[0], counter[1], counter[2], current_speed);
    //         last_mqtt_time = esp_timer_get_time();
    //         ESP_LOGI(TAG, "Data pushed to MQTT");
    //     }
    //     vTaskDelay(1000 / portTICK_PERIOD_MS);
    // }
}
