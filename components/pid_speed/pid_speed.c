#include "pid_speed.h"
#include <math.h>

static float last_delta = 1;
void pid_speed_reset(pid_speed_t *pid)
{
    pid->integral = 0.0f;
    pid->prev_measured = 0.0f;
}
void pid_speed_init(pid_speed_t *pid,
                    float kp, float ki, float kd,
                    float out_min, float out_max)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->integral = 0.0f;
    pid->prev_measured = 0.0f;

    pid->output_min = out_min;
    pid->output_max = out_max;
}

float pid_speed_update(pid_speed_t *pid,
                       float target,
                       float measured,
                       float dt)
{
    float remove_p = 0;
    float error = target - measured;
    if (error > -1.0f && error < 1.0f) {
        error = 0.0f; 
        // Khi error = 0, thành phần P sẽ mất, I ngừng tăng -> Motor giữ nguyên trạng thái
    }
    if (fabs(error) <= 0.5f) 
        remove_p = 0;
    else 
        remove_p = 1;
    pid->integral += error * dt;
    if (pid->integral > 20.0f) pid->integral = 20.0f;
    if (pid->integral < -20.0f) pid->integral = -20.0f;

    float output =
          pid->kp * error * remove_p
        + pid->ki * pid->integral
        - pid->kd * (measured - pid->prev_measured) / dt;
    output = (output*0.6 + last_delta*0.4);
    last_delta = output;
    if (output > pid->output_max) output = pid->output_max;
    if (output < pid->output_min) output = pid->output_min;
    pid->prev_measured = measured;
    return output;   // ΔPWM
}
