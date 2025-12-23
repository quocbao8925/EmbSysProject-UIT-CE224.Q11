#ifndef MOTOR_SPEED_CONTROL_H
#define MOTOR_SPEED_CONTROL_H

#include <stdint.h>
#include <stdbool.h>
#include "motor_l298n.h"  // Driver motor của bạn
#include "encoder.h"      // Driver encoder của bạn
#include "pid_speed.h"    // Thư viện PID của bạn

// Hàm điều khiển chính
void motor_speed_pid_step(motor_t *motor,
                          encoder_t *encoder,
                          pid_speed_t *pid,
                          float target_rps,
                          float dt, 
                          bool was_stopped, 
                          float measured);

#endif