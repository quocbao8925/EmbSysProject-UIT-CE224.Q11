#ifndef PCA9685_H
#define PCA9685_H

#include <stdint.h>
#include "driver/i2c.h"
#include "esp_err.h"

typedef struct {
    i2c_port_t i2c_port;   // I2C_NUM_0 / I2C_NUM_1
    uint8_t address;       // 0x40 default
    uint32_t i2c_freq;     // not used for install anymore, but keep if you want
} pca9685_t;

esp_err_t pca9685_init(pca9685_t *dev);
esp_err_t pca9685_set_pwm_freq(pca9685_t *dev, float freq);
esp_err_t pca9685_set_pwm(pca9685_t *dev, uint8_t channel, uint16_t on, uint16_t off);
esp_err_t pca9685_set_servo_angle(pca9685_t *dev, uint8_t channel, float angle);
void servo_open(pca9685_t *dev, uint8_t channel);
void servo_close(pca9685_t *dev, uint8_t channel);

#endif
