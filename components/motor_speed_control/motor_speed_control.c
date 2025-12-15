#include "motor_speed_control.h"

static float current_pwm = 0.0f; // 0–100 %

void motor_speed_pid_step(motor_t *motor,
                          encoder_t *encoder,
                          pid_speed_t *pid,
                          float target_rps,
                          float dt)
{
    // 1. Đo tốc độ hiện tại
    float measured_rps = encoder_get_rps(encoder);

    // 2. PID tính delta PWM
    float delta = pid_speed_update(pid,
                                   target_rps,
                                   measured_rps,
                                   dt);

    // 3. Cập nhật PWM hiện tại
    current_pwm += delta;

    // Clamp PWM
    if (current_pwm > 100.0f) current_pwm = 100.0f;
    if (current_pwm < 0.0f)   current_pwm = 0.0f;

    // 4. Áp PWM cho motor
    motor_set_speed(motor, (uint8_t)current_pwm);
}
