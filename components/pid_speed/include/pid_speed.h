#ifndef PID_SPEED_H
#define PID_SPEED_H

#include <stdint.h>

typedef struct {
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_measured;

    float output_min;
    float output_max;
} pid_speed_t;

void pid_speed_init(pid_speed_t *pid, float kp, float ki, float kd, float out_min, float out_max);

float pid_speed_update(pid_speed_t *pid, float target, float measured, float dt);
void pid_speed_reset(pid_speed_t *pid);

#endif
/*pid_speed_init(&pid,
    15.0f,   // Kp
    3.0f,    // Ki
    0.0f,    // Kd (để 0 cho đỡ nhiễu)
    0.0f,    // PWM min
    100.0f   // PWM max
);*/