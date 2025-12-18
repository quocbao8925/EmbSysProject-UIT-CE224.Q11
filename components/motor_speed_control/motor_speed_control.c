#include "motor_speed_control.h"
#include <math.h> // Để dùng hàm fabs (trị tuyệt đối số thực)

static float current_pwm = 0.0f;       // Giá trị tính toán nội bộ của PID
static uint8_t last_applied_pwm = 0;   // Giá trị thực tế đã nạp xuống motor
#define MIN_PWM      55.0f   // deadzone L298N (tune)
#define KICK_PWM    100.0f
#define KICK_TIME_MS 150
static int64_t kick_end_time = 0;
// Cấu hình ngưỡng thay đổi:
// Chỉ nạp PWM mới xuống motor nếu nó lệch quá 2% so với giá trị cũ
#define PWM_UPDATE_THRESHOLD  2.0f 

void motor_speed_pid_step(motor_t *motor,
                          encoder_t *encoder,
                          pid_speed_t *pid,
                          float target_rps,
                          float dt)
{
    float measured = encoder_get_rps(encoder);
    float pwm;

    int64_t now = esp_timer_get_time();

    // Stop condition
    if (target_rps <= 0.01f) {
        pid_speed_reset(pid);
        motor_set_speed(motor, 0);
        kick_end_time = 0;
        return;
    }

    // Kick-start nếu motor đứng
    if (measured < 0.05f && kick_end_time == 0) {
        kick_end_time = now + KICK_TIME_MS * 1000;
    }

    if (kick_end_time > now) {
        pwm = KICK_PWM;
    } else {
        kick_end_time = 0;
        pwm = pid_speed_update(pid, target_rps, measured, dt);

        // Deadzone compensation
        if (pwm > 0 && pwm < MIN_PWM)
            pwm = MIN_PWM;
    }

    motor_set_speed(motor, (uint8_t)pwm);
}