#include "encoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"

static encoder_t *s_encoder_ref = NULL;

static void IRAM_ATTR encoder_isr_handler(void *arg)
{
    encoder_t *enc = (encoder_t *)arg;
    portENTER_CRITICAL_ISR(&enc->mux);
    enc->pulses++;
    portEXIT_CRITICAL_ISR(&enc->mux);
}

esp_err_t encoder_init(encoder_t *enc)
{
    enc->pulses = 0;
    enc->last_time_us = esp_timer_get_time();
    enc->mux = (portMUX_TYPE)portMUX_INITIALIZER_UNLOCKED;

    gpio_config_t io_conf = {
        .pin_bit_mask = 1ULL << enc->pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_POSEDGE
    };
    gpio_config(&io_conf);

    gpio_isr_handler_add(enc->pin, encoder_isr_handler, enc);
    return ESP_OK;
}

float encoder_get_rps(encoder_t *enc)
{
    int64_t now = esp_timer_get_time();
    float dt = (now - enc->last_time_us) / 1e6f;
    if (dt <= 0.0001f) dt = 0.0001f;
    enc->last_time_us = now;

    int32_t pulses;
    portENTER_CRITICAL(&enc->mux);
    pulses = enc->pulses;
    enc->pulses = 0;
    portEXIT_CRITICAL(&enc->mux);

    float rev = (float)pulses / (float)enc->pulses_per_rev;
    return rev / dt;
}

float encoder_get_rpm(encoder_t *enc)
{
    return encoder_get_rps(enc) * 60.0f;
}
/*
encoder_t wheel_encoder = {
    .pin = GPIO_NUM_34,        // OUT từ LM393 to gpio
    .pulses_per_rev = 20       // number of holes on the wheel
};
*/