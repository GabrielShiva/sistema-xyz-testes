
#ifndef POSITION_HANDLER_H
#define POSITION_HANDLER_H

#include "globals.h"

void init_saved_positions(void);
void send_saved_positions_update(void);
int find_saved_position_index(char character);

#endif