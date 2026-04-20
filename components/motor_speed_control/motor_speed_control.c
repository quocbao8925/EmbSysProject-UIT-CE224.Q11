#include "motor_speed_control.h"
#include <math.h>

static float current_pwm = 0.0f;
static bool kick_armed = true;

#define MIN_PWM        30.0f   
#define MAX_PWM        100.0f
#define KICK_PWM       47.0f
#define KICK_TIME_MS   50
#define ACCEL_STEP     0.25f   
#define DECEL_STEP     0.25f    

static int64_t kick_end_time = 0;

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}
void motor_speed_pid_step(motor_t *motor,
                          encoder_t *encoder,
                          pid_speed_t *pid,
                          float target_rps,
                          float dt, bool was_stopped, float measured)
{
    int64_t now = esp_timer_get_time();

    // STOP
    if (target_rps < 0.05f) {
        kick_armed = true;
        pid_speed_reset(pid);
        current_pwm = 0;
        motor_set_speed(motor, 0);
        kick_end_time = 0;
        return;
    }
    // KICK START
    if (kick_armed && was_stopped && target_rps > 0.05f) {
        kick_end_time = now + KICK_TIME_MS * 1000;
        was_stopped = false;
        kick_armed = false;
    }

    if (kick_end_time > now) {
        current_pwm = KICK_PWM;
    } else {
        kick_end_time = 0;

        float delta = pid_speed_update(pid, target_rps, measured, dt);

        if (delta > 0) {
            if (delta > ACCEL_STEP) delta = ACCEL_STEP;
        } else {
            if (delta < -DECEL_STEP) delta = -DECEL_STEP;
        }

        current_pwm += delta * ((was_stopped && target_rps > 0.05f) ? 2.0f : 1.0f);
        /* Deadzone */
        if (current_pwm > 0 && current_pwm < MIN_PWM)
            current_pwm = MIN_PWM;

        current_pwm = clampf(current_pwm, 0, MAX_PWM);
    }

    motor_set_speed(motor, (uint8_t)current_pwm);
}
