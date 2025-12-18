#ifndef ENCODER_H
#define ENCODER_H

#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_err.h"

typedef struct {
    gpio_num_t pin;
    int32_t pulses_per_rev;

    volatile int32_t pulses;
    int64_t last_time_us;

    portMUX_TYPE mux;
} encoder_t;

esp_err_t encoder_init(encoder_t *enc);
void encoder_isr_handler(void *arg);

// gọi định kỳ để tính tốc độ
float encoder_get_rps(encoder_t *enc);  // vòng / giây
float encoder_get_rpm(encoder_t *enc);  // vòng / phút

#endif