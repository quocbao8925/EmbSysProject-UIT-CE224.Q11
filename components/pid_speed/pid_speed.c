#include "pid_speed.h"

void pid_speed_init(pid_speed_t *pid,
                    float kp, float ki, float kd,
                    float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->integral = 0.0f;
    pid->prev_error = 0.0f;

    pid->output_min = out_min;
    pid->output_max = out_max;
}

float pid_speed_update(pid_speed_t *pid,
                       float target,
                       float measured,
                       float dt)
{
    float error = target - measured;

    // Integral
    pid->integral += error * dt;

    // Anti-windup
    if (pid->integral > pid->output_max)
        pid->integral = pid->output_max;
    else if (pid->integral < pid->output_min)
        pid->integral = pid->output_min;

    // Derivative
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    // PID output = delta PWM
    float output = pid->kp * error
                 + pid->ki * pid->integral
                 + pid->kd * derivative;

    // Clamp output
    if (output > pid->output_max)
        output = pid->output_max;
    else if (output < pid->output_min)
        output = pid->output_min;

    return output;
}
