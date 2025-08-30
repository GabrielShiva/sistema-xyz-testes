// joystick_control.h

#ifndef JOYSTICK_CONTROL_H
#define JOYSTICK_CONTROL_H

#include "globals.h"

// Declaração de funções do joystick
uint16_t read_joystick_average(int adc_input, int samples, int delay_ms);
void control_motor_from_joystick(stepper_motor_t *motor, uint16_t reading, uint16_t center);

#endif // JOYSTICK_CONTROL_H