// stepper_control.h

#ifndef STEPPER_CONTROL_H
#define STEPPER_CONTROL_H

#include "globals.h"

// Declaração de funções de controle dos motores
int64_t alarm_irq_handler(alarm_id_t id, void *user_data);
void init_stepper_motor(stepper_motor_t *motor);
void move_n_steps(stepper_motor_t *motor, int32_t steps);
void move_to_position(stepper_motor_t *motor, int32_t target, bool wait);
void stop_all_motors(void);
void start_motor_continuous(stepper_motor_t *motor);
void stop_motor(stepper_motor_t *motor);

#endif // STEPPER_CONTROL_H