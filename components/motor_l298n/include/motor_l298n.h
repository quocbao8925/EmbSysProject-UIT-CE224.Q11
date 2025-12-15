#ifndef MOTOR_L298N_H
#define MOTOR_L298N_H

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_err.h"

typedef enum {
    MOTOR_DIR_STOP = 0,
    MOTOR_DIR_FORWARD,
    MOTOR_DIR_BACKWARD
} motor_dir_t;

typedef struct {
    gpio_num_t in1_pin;
    gpio_num_t in2_pin;
    gpio_num_t en_pin;       // PWM pin
    ledc_channel_t pwm_channel;
    ledc_timer_t pwm_timer;
    uint32_t pwm_freq;
    ledc_mode_t pwm_mode;
} motor_t;

// Init motor
esp_err_t motor_init(motor_t *motor);

// Set direction
esp_err_t motor_set_direction(motor_t *motor, motor_dir_t dir);

// Set speed (0–100%)
esp_err_t motor_set_speed(motor_t *motor, uint8_t speed_percent);

// Shortcut
esp_err_t motor_stop(motor_t *motor);

#endif