#include "motor_speed_control.h"
#include <math.h>

static float current_pwm = 0.0f;

#define MIN_PWM        20.0f   // deadzone thực tế
#define MAX_PWM        100.0f
#define KICK_PWM       45.0f
#define KICK_TIME_MS   80
#define PWM_SLEW_STEP  2.0f    // % mỗi dt (rất quan trọng)

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
                          float dt, bool was_stopped)
{
    float measured = encoder_get_rps(encoder);

    int64_t now = esp_timer_get_time();

    /* ===== STOP ===== */
    if (target_rps < 0.05f) {
        pid_speed_reset(pid);
        current_pwm = 0;
        motor_set_speed(motor, 0);
        kick_end_time = 0;
        return;
    }

    /* ===== KICK START ===== */
    if (was_stopped && target_rps > 0.05f) {
        kick_end_time = now + KICK_TIME_MS * 1000;
        was_stopped = false;
    }

    if (kick_end_time > now) {
        current_pwm = KICK_PWM;
    } else {
        kick_end_time = 0;

        /* ===== PI: delta PWM ===== */
        float delta = pid_speed_update(pid, target_rps, measured, dt);
        /* Slew rate limit */
        delta = clampf(delta, -PWM_SLEW_STEP, PWM_SLEW_STEP);

        current_pwm += delta;

        /* Deadzone */
        if (current_pwm > 0 && current_pwm < MIN_PWM)
            current_pwm = MIN_PWM;

        current_pwm = clampf(current_pwm, 0, MAX_PWM);
    }

    motor_set_speed(motor, (uint8_t)current_pwm);
}
