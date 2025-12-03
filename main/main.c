// main.c (ESP-IDF)

#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "ir_sensor.h"
#include "tcs34725.h"
#include "driver/i2c.h"
#include "driver/ledc.h"

// begin defines
#define IRSENSOR_GPIO 32
#define USE_PULLUP 1



// example
// end defines
// begin structs

// end structs 

// begin global variables, constants
ir_sensor_handler ir_handler;
ir_sensor_event_t ir_event;
tcs34725_handler tcs_handler;

static const char *TAG = "main";
// end global variables

// begin function prototypes
void ir_sensor_task(void *arg);
void tcs34725_task(void *arg);
uint32_t angle_to_duty(int angle) {
    // Map angle (0-180) to duty (102 to 409)
    if (angle > 180) angle = 180;
    if (angle < 0) angle = 0;
    float pulse_width = 1 + (angle / 180.0) * 2.0; // in ms
    uint32_t duty = (pulse_width / 20.0) * 4096; // 20ms period, 12-bit resolution
    return duty;
}
void servo_rotate_task(void *arg){
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << 25),
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE
    };
    // gpio_config(&io_conf); // apply GPIO configuration
    // ledc_timer_config_t timer_conf = {
    //     .duty_resolution = LEDC_TIMER_12_BIT,
    //     .freq_hz = 50,
    //     .speed_mode = LEDC_HIGH_SPEED_MODE,
    //     .timer_num = LEDC_TIMER_0,
    //     .clk_cfg = LEDC_AUTO_CLK
    // };
    // ledc_timer_config(&timer_conf); // apply timer configuration

    ledc_channel_config_t ch_conf = {
        .channel = LEDC_CHANNEL_0,
        .duty = 0,
        .gpio_num = 25,
        .intr_type = LEDC_INTR_DISABLE,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .hpoint = 0,
        .timer_sel = LEDC_TIMER_0
    };
    ledc_channel_config(&ch_conf); // apply channel configuration

    while (1) {
        for (int angle = 0; angle <= 180; angle += 10) {
            ESP_LOGI(TAG, "Rotating to angle: %d", angle);
            uint32_t duty = angle_to_duty(angle);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
        for (int angle = 170; angle >= 10; angle -= 10) {
            ESP_LOGI(TAG, "Rotating to angle: %d", angle);
            uint32_t duty = angle_to_duty(angle);
            ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, duty);
            ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

// end function prototypes
void app_main(void)
{
    // Initialize 
    ir_sensor_init(&ir_handler, IRSENSOR_GPIO, USE_PULLUP);
    //tcs34725_init(&tcs_handler);
    // Create tasks
    // xTaskCreate(servo_rotate_task, "Servo_Rotate_Task", 2048, NULL, 5, NULL);
    xTaskCreate(ir_sensor_task, "IR_Sensor_Task", 2048, NULL, 5, NULL);
    //xTaskCreate(tcs34725_task, "TCS34725_Task", 2048, NULL, 5, NULL);
    // Bind tasks to cores if needed (optional)
    xTaskCreatePinnedToCore(ir_sensor_task, "IR_Sensor_Task", 2048, NULL, 5, NULL, 0);
    //xTaskCreatePinnedToCore(tcs34725_task, "TCS34725_Task", 2048, NULL, 5, NULL, 1);
    // ir_sensor_counter(&ir_handler, &ir_event);
    while (1) {

    }
}
// end main function
// begin function definitions
void ir_sensor_task(void *arg) {
    while (1) {
        ir_sensor_counter(&ir_handler, &ir_event);
    }
}
void tcs34725_task(void *arg) {
    while (1) {
        tcs34725_reader(&tcs_handler);
    }
}
/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
// #include <stdio.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/gpio.h"
// #include "esp_log.h"
// #include "led_strip.h"
// #include "sdkconfig.h"

// static const char *TAG = "example";

// /* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
//    or you can edit the following line and set a number here.
// */
// #define BLINK_GPIO 2

// static uint8_t s_led_state = 0;

// #ifdef CONFIG_BLINK_LED_STRIP

// static led_strip_handle_t led_strip;

// static void blink_led(void)
// {
//     /* If the addressable LED is enabled */
//     if (s_led_state) {
//         /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
//         led_strip_set_pixel(led_strip, 0, 16, 16, 16);
//         /* Refresh the strip to send data */
//         led_strip_refresh(led_strip);
//     } else {
//         /* Set all LED off to clear all pixels */
//         led_strip_clear(led_strip);
//     }
// }

// static void configure_led(void)
// {
//     ESP_LOGI(TAG, "Example configured to blink addressable LED!");
//     /* LED strip initialization with the GPIO and pixels number*/
//     led_strip_config_t strip_config = {
//         .strip_gpio_num = BLINK_GPIO,
//         .max_leds = 1, // at least one LED on board
//     };
// #if CONFIG_BLINK_LED_STRIP_BACKEND_RMT
//     led_strip_rmt_config_t rmt_config = {
//         .resolution_hz = 10 * 1000 * 1000, // 10MHz
//         .flags.with_dma = false,
//     };
//     ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
// #elif CONFIG_BLINK_LED_STRIP_BACKEND_SPI
//     led_strip_spi_config_t spi_config = {
//         .spi_bus = SPI2_HOST,
//         .flags.with_dma = true,
//     };
//     ESP_ERROR_CHECK(led_strip_new_spi_device(&strip_config, &spi_config, &led_strip));
// #else
// #error "unsupported LED strip backend"
// #endif
//     /* Set all LED off to clear all pixels */
//     led_strip_clear(led_strip);
// }

// #elif CONFIG_BLINK_LED_GPIO

// static void blink_led(void)
// {
//     /* Set the GPIO level according to the state (LOW or HIGH)*/
//     gpio_set_level(BLINK_GPIO, s_led_state);
// }

// static void configure_led(void)
// {
//     ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
//     gpio_reset_pin(BLINK_GPIO);
//     /* Set the GPIO as a push/pull output */
//     gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
// }

// #else
// #error "unsupported LED type"
// #endif

// void app_main(void)
// {

//     /* Configure the peripheral according to the LED type */
//     configure_led();

//     while (1) {
//         ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
//         blink_led();
//         /* Toggle the LED state */
//         s_led_state = !s_led_state;
//         vTaskDelay(2000 / portTICK_PERIOD_MS);
//     }
// }

