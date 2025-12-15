#include "pca9685.h"
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MODE1_REG     0x00
#define PRE_SCALE     0xFE

#define MODE1_AI      0x20
#define MODE1_SLEEP   0x10
#define MODE1_ALLCALL 0x01
#define MODE1_RESTART 0x80

static esp_err_t write_reg(pca9685_t *dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_write_to_device(dev->i2c_port, dev->address, buf, 2, pdMS_TO_TICKS(50));
}

static esp_err_t write_pwm(pca9685_t *dev, uint8_t ch, uint16_t on, uint16_t off)
{
    uint8_t reg = 0x06 + 4 * ch;
    uint8_t buf[5] = {
        reg,
        on & 0xFF,
        (on >> 8) & 0xFF,
        off & 0xFF,
        (off >> 8) & 0xFF
    };
    return i2c_master_write_to_device(dev->i2c_port, dev->address, buf, 5, pdMS_TO_TICKS(50));
}

esp_err_t pca9685_set_pwm_freq(pca9685_t *dev, float freq)
{
    // prescale = round(25MHz/(4096*freq) - 1)
    float prescaleval = 25000000.0f / (4096.0f * freq) - 1.0f;
    uint8_t prescale = (uint8_t)floorf(prescaleval + 0.5f);

    // sleep
    ESP_ERROR_CHECK(write_reg(dev, MODE1_REG, MODE1_SLEEP | MODE1_ALLCALL));
    vTaskDelay(pdMS_TO_TICKS(5));

    // prescale
    ESP_ERROR_CHECK(write_reg(dev, PRE_SCALE, prescale));

    // wake
    ESP_ERROR_CHECK(write_reg(dev, MODE1_REG, MODE1_AI | MODE1_ALLCALL));
    vTaskDelay(pdMS_TO_TICKS(5));

    // restart
    ESP_ERROR_CHECK(write_reg(dev, MODE1_REG, MODE1_RESTART | MODE1_AI | MODE1_ALLCALL));
    vTaskDelay(pdMS_TO_TICKS(5));

    return ESP_OK;
}

esp_err_t pca9685_init(pca9685_t *dev)
{
    // IMPORTANT: I2C driver must be installed in main already.
    // Just init PCA9685 registers here.

    // reset mode
    ESP_ERROR_CHECK(write_reg(dev, MODE1_REG, MODE1_ALLCALL));
    vTaskDelay(pdMS_TO_TICKS(5));

    // 50Hz for servo
    ESP_ERROR_CHECK(pca9685_set_pwm_freq(dev, 50.0f));
    return ESP_OK;
}

esp_err_t pca9685_set_pwm(pca9685_t *dev, uint8_t channel, uint16_t on, uint16_t off)
{
    return write_pwm(dev, channel, on, off);
}

esp_err_t pca9685_set_servo_angle(pca9685_t *dev, uint8_t channel, float angle)
{
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    // typical 0.5ms..2.5ms at 50Hz
    float pulse_us = 500.0f + (angle / 180.0f) * 2000.0f; // 500..2500 us
    uint16_t ticks = (uint16_t)(pulse_us * 4096.0f / 20000.0f); // 20ms period

    return pca9685_set_pwm(dev, channel, 0, ticks);
}
void servo_open(pca9685_t *dev, uint8_t channel)
{
    pca9685_set_servo_angle(dev, channel, 95); // Open position
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for servo to move
}
void servo_close(pca9685_t *dev, uint8_t channel)
{
    pca9685_set_servo_angle(dev, channel, 2); // Close position
    vTaskDelay(pdMS_TO_TICKS(100)); // Wait for servo to move
}   