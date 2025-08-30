#ifndef TASKSRTOS_H
#define TASKSRTOS_H

#include "FreeRTOS.h"
#include "task.h"

#include "globals.h"
#include "stepper_control.h"
#include "joystick_control.h"
#include "command_handler.h"
#include "position_handler.h"
#include "homing.h"

void vSerialTask();

void vJoystickTask();

#endif