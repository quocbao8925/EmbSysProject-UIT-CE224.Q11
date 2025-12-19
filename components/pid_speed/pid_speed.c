#include "pid_speed.h"

static float last_delta = 1;
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

    pid->integral += error * dt;
    if (pid->integral > 20.0f) pid->integral = 20.0f;
    if (pid->integral < -20.0f) pid->integral = -20.0f;

    float output =
          pid->kp * error
        + pid->ki * pid->integral;
    output = (output*0.8 + last_delta*0.2);
    last_delta = output;
    return output;   // ΔPWM
}
