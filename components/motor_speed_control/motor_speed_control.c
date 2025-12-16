#include "motor_speed_control.h"
#include <math.h> // Để dùng hàm fabs (trị tuyệt đối số thực)

static float current_pwm = 0.0f;       // Giá trị tính toán nội bộ của PID
static uint8_t last_applied_pwm = 0;   // Giá trị thực tế đã nạp xuống motor

// Cấu hình ngưỡng thay đổi:
// Chỉ nạp PWM mới xuống motor nếu nó lệch quá 2% so với giá trị cũ
#define PWM_UPDATE_THRESHOLD  2.0f 

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

    // 3. Cập nhật biến tính toán (Internal State)
    current_pwm += delta;

    // Clamp giá trị tính toán (Giới hạn logic)
    // Bạn đang giới hạn max 50%, oke.
    if (current_pwm > 50.0f) current_pwm = 50.0f;
    if (current_pwm < 0.0f)  current_pwm = 0.0f;

    // 4. LOGIC MỚI: Chỉ apply khi thay đổi đủ lớn
    // So sánh giá trị muốn đặt (current_pwm) với giá trị đang chạy (last_applied_pwm)
    if (fabs(current_pwm - (float)last_applied_pwm) >= PWM_UPDATE_THRESHOLD) 
    {
        // Chuyển sang số nguyên
        uint8_t new_pwm_int = (uint8_t)current_pwm;
        
        // Gọi hàm điều khiển phần cứng
        motor_set_speed(motor, new_pwm_int);
        
        // Lưu lại trạng thái
        last_applied_pwm = new_pwm_int;

        printf("UPDATE MOTOR: Target: %.2f, Meas: %.2f, PWM: %d\n", 
                target_rps, measured_rps, new_pwm_int);
    }
    else 
    {
        // Không làm gì cả -> Motor chạy mượt hơn, không bị spam lệnh set_speed liên tục
        // printf("STABLE: Target: %.2f, Meas: %.2f, Holding PWM: %d\n", target_rps, measured_rps, last_applied_pwm);
    }
}