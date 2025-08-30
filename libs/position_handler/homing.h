#ifndef HOMING_H
#define HOMING_H

#include "globals.h"

bool home_single_motor(stepper_motor_t *motor, int motor_id);
bool home_all_motors(void);
bool read_limit_switch(stepper_motor_t *motor);
void init_limit_switches(void);

#endif