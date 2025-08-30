// command_handler.h

#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include "globals.h"

// Declaração de funções de comando
void process_serial_input(void);
void handle_command(const char* command);
void send_state_update(void);
void send_position_update(void);
void send_homing_status(void);

#endif // COMMAND_HANDLER_H