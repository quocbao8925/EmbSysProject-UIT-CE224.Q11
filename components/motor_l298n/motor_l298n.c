#include "motor_l298n.h"

esp_err_t motor_init(motor_t *motor)
{
    // Configure IN1, IN2 as output
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << motor->in1_pin) | (1ULL << motor->in2_pin),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = 0,
        .pull_up_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_set_level(motor->in1_pin, 0);
    gpio_set_level(motor->in2_pin, 0);

    // LEDC timer for PWM
    ledc_timer_config_t timer_conf = {
        .speed_mode       = motor->pwm_mode,
        .timer_num        = motor->pwm_timer,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .freq_hz          = motor->pwm_freq,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    // LEDC channel
    ledc_channel_config_t ch_conf = {
        .gpio_num       = motor->en_pin,
        .speed_mode     = motor->pwm_mode,
        .channel        = motor->pwm_channel,
        .timer_sel      = motor->pwm_timer,
        .duty           = 0,
        .hpoint         = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_conf));

    return ESP_OK;
}

esp_err_t motor_set_direction(motor_t *motor, motor_dir_t dir)
{
    switch (dir)
    {
        case MOTOR_DIR_FORWARD:
            gpio_set_level(motor->in1_pin, 1);
            gpio_set_level(motor->in2_pin, 0);
            break;

        case MOTOR_DIR_BACKWARD:
            gpio_set_level(motor->in1_pin, 0);
            gpio_set_level(motor->in2_pin, 1);
            break;

        case MOTOR_DIR_STOP:
        default:
            gpio_set_level(motor->in1_pin, 0);
            gpio_set_level(motor->in2_pin, 0);
            break;
    }
    return ESP_OK;
}

esp_err_t motor_set_speed(motor_t *motor, uint8_t speed_percent)
{
    if (speed_percent > 100) speed_percent = 100;

    uint32_t duty = (speed_percent * ((1 << 10) - 1)) / 100;

    ESP_ERROR_CHECK(ledc_set_duty(motor->pwm_mode, motor->pwm_channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(motor->pwm_mode, motor->pwm_channel));

    return ESP_OK;
}

esp_err_t motor_stop(motor_t *motor)
{
    motor_set_direction(motor, MOTOR_DIR_STOP);
    motor_set_speed(motor, 0);
    return ESP_OK;
}
// init example
/*
motor_t motorA = {
    .in1_pin      = GPIO_NUM_25,
    .in2_pin      = GPIO_NUM_26,
    .en_pin       = GPIO_NUM_27,
    .pwm_channel  = LEDC_CHANNEL_0,
    .pwm_timer    = LEDC_TIMER_0,
    .pwm_freq     = 20000,       // 20 kHz (silent for motor)
    .pwm_mode     = LEDC_LOW_SPEED_MODE
};*/