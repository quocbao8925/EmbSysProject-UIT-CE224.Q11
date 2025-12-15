#ifndef PID_SPEED_H
#define PID_SPEED_H

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_error;

    float output_min;
    float output_max;
} pid_speed_t;

void pid_speed_init(pid_speed_t *pid,
                    float kp, float ki, float kd,
                    float out_min, float out_max);

float pid_speed_update(pid_speed_t *pid,
                       float target,
                       float measured,
                       float dt);

#endif