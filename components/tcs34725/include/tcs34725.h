#ifndef TCS34725_H
#define TCS34725_H

#pragma once
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"


// defines i2c
#define TCS34725_ADDR           0x29 // SDA,SCL, S1->G, VCC->V, GND->GND
#define I2C_MASTER_NUM          I2C_NUM_0
#define I2C_MASTER_SDA_IO       21
#define I2C_MASTER_SCL_IO       22
#define I2C_MASTER_FREQ_HZ      400000
#define I2C_MASTER_TIMEOUT_MS   1000

// defines command bits & registers
// bit CMD
#define TCS_CMD_BIT  0x80
#define TCS_CMD_AUTO_INC   0x20
// registers' addresses
#define REG_ENABLE   0x00
#define REG_STATUS   0x13
#define REG_ATIME    0x01 
#define REG_CONTROL  0x0F
#define REG_ID       0x12
#define REG_CDATAL   0x14
#define REG_RDATAL   0x16
#define REG_GDATAL   0x18
#define REG_BDATAL   0x1A
// config values
#define ENABLE_PON   0x01  // Power ON
#define ENABLE_AEN   0x02  // ADC Enable
// handler struct
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} tcs34725_handler;
// function prototypes
void i2c_address_scan(void);
void tcs34725_init(tcs34725_handler *handler);
void tcs34725_deinit(void);
esp_err_t tcs_write(uint8_t reg, uint8_t value);
esp_err_t tcs_read(tcs34725_handler *handler, uint8_t reg, uint8_t *data, size_t len);
void tcs34725_reader(tcs34725_handler *handler);
void get_rgb_values(tcs34725_handler *handler, uint8_t *r, uint8_t *g, uint8_t *b);
#endif