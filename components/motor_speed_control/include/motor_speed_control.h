#ifndef MOTOR_SPEED_CONTROL_H
#define MOTOR_SPEED_CONTROL_H

#include "motor_l298n.h"
#include "encoder.h"
#include "pid_speed.h"

void motor_speed_pid_step(motor_t *motor,
                          encoder_t *encoder,
                          pid_speed_t *pid,
                          float target_rps,
                          float dt, bool was_stopped, float measured);

#endif
