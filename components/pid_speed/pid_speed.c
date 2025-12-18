#include "pid_speed.h"


void pid_speed_reset(pid_speed_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
}
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

    // Clamp integral (đơn vị: error·time)
    const float I_LIMIT = 20.0f;   // tune được
    if (pid->integral > I_LIMIT)  pid->integral = I_LIMIT;
    if (pid->integral < -I_LIMIT) pid->integral = -I_LIMIT;

    // Derivative (chỉ dùng nếu encoder đủ mượt)
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    float output =
          pid->kp * error
        + pid->ki * pid->integral
        + pid->kd * derivative;

    // Clamp OUTPUT (PWM %)
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;

    return output;
}
