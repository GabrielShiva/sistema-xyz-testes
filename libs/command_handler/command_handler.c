// command_handler.c

#include "command_handler.h"
#include "stepper_control.h" // Precisa das funções de controle do motor

// Funções de parsing (internas a este módulo)
static void parse_move_command(const char* params);
static void parse_speed_command(const char* params);
static void parse_mode_command(const char* params);
static void parse_motors_command(const char* params);
static void parse_moveto_command(const char* params);
static void parse_setzero_command(const char* params);

void process_serial_input(void) {
    int c = getchar_timeout_us(0);
    if (c != PICO_ERROR_TIMEOUT) {
        if (c == '\n' || c == '\r') {
            if (command_buffer_pos > 0) {
                command_buffer[command_buffer_pos] = '\0';
                command_ready = true;
            }
        } else if (c >= 32 && c <= 126) {
            if (command_buffer_pos < COMMAND_BUFFER_SIZE - 1) {
                command_buffer[command_buffer_pos++] = (char)c;
            }
        }
    }
}

void handle_command(const char* command) {
    printf("COMANDO RECEBIDO: %s\n", command);
    if (strncmp(command, "MOVE,", 5) == 0) parse_move_command(command + 5);
    else if (strncmp(command, "SPEED,", 6) == 0) parse_speed_command(command + 6);
    else if (strcmp(command, "STOP") == 0) {
        stop_all_motors();
        printf("ACK,Parada de emergencia executada\n");
    }
    else if (strncmp(command, "MODE,", 5) == 0) parse_mode_command(command + 5);
    else if (strncmp(command, "MOTORS,", 7) == 0) parse_motors_command(command + 7);
    else if (strncmp(command, "MOVETO,", 7) == 0) parse_moveto_command(command + 7);
    else if (strncmp(command, "SETZERO", 7) == 0) {
        if (strlen(command) > 7 && command[7] == ',') parse_setzero_command(command + 8);
        else parse_setzero_command(NULL);
    }
    else if (strcmp(command, "STATUS") == 0) {
        printf("STATUS");
        for (uint i = 0; i < active_motor_count && i < 3; i++) {
            stepper_motor_t *motor = &steppers[i];
            printf(",%ld,%d,%d,%.1f", motor->step_position, motor->alarm_active ? 1 : 0, motor->dir, motor->actual_step_interval);
        }
        printf("\n");
        send_state_update();
    }
    else { printf("ERROR,Comando desconhecido: %s\n", command); }
}

void send_state_update(void) {
    const char* state_str = (current_state == STATE_JOYSTICK) ? "JOYSTICK" : "COMMAND";
    printf("STATE,%s,%d\n", state_str, active_motor_count);
}

void send_position_update(void) {
    printf("POSITION");
    for (uint i = 0; i < active_motor_count && i < 3; i++) {
        printf(",%ld", steppers[i].step_position);
    }
    printf("\n");
}

// // Implementações das funções de parsing (exemplo de uma, as outras são análogas)
// static void parse_move_command(const char* params) {
//     if (current_state != STATE_COMMAND) {
//         printf("ERROR,Nao pode mover no modo joystick\n");
//         return;
//     }
//     char* param_copy = strdup(params); // Use strdup para evitar modificar o buffer original
//     char* first_param = strtok(param_copy, ",");
//     char* second_param = strtok(NULL, ",");
//     int motor_id = 0;
//     int steps = 0;
//     if (second_param) {
//         motor_id = atoi(first_param);
//         steps = atoi(second_param);
//     } else {
//         steps = atoi(first_param);
//     }
//     if (motor_id < 0 || motor_id >= (int)active_motor_count) {
//         printf("ERROR,ID de motor invalido: %d\n", motor_id);
//     } else {
//         stepper_motor_t *motor = &steppers[motor_id];
//         if (motor->alarm_active) {
//             stop_motor(motor);
//             sleep_ms(10);
//         }
//         printf("ACK,Motor %d movendo %d passos\n", motor_id, steps);
//         if (steps != 0) {
//             move_n_steps(motor, steps);
//             while (!motor->movement_done && motor->alarm_active) { sleep_ms(1); }
//             printf("ACK,Motor %d realizou movimento para posicao %ld\n", motor_id, motor->step_position);
//         }
//         send_position_update();
//     }
//     free(param_copy);
// }

// ... Coloque aqui as outras funções parse_* de forma similar
// (parse_speed_command, parse_mode_command, etc.)
// ... (O código completo para elas está no seu arquivo original)






// Replace your existing parse_move_command function with this:
void parse_move_command(const char* params) {
    char* param_copy = malloc(strlen(params) + 1);
    strcpy(param_copy, params);

    char* first_param = strtok(param_copy, ",");
    char* second_param = strtok(NULL, ",");

    int motor_id = 0;
    int steps = 0;

    if (second_param) {
        // Two parameters: motor_id, steps
        motor_id = atoi(first_param);
        steps = atoi(second_param);
    } else {
        // One parameter: steps (default to motor 0)
        steps = atoi(first_param);
    }

    if (motor_id < 0 || motor_id >= (int)active_motor_count || motor_id >= 3) {
        printf("ERROR,ID de motor invalido: %d\n", motor_id);
        free(param_copy);
        return;
    }

    stepper_motor_t *motor = &steppers[motor_id];

    // Stop current movement if any
    if (motor->alarm_active) {
        stop_motor(motor);
        sleep_ms(10);
    }

    printf("ACK,Motor %d movendo %d passos\n", motor_id, steps);

    if (steps != 0) {
        move_n_steps(motor, steps);

        // Wait for movement to complete
        while (!motor->movement_done && motor->alarm_active) {
            sleep_ms(1);
        }
        printf("ACK,Motor %d realizou movimento para posicao %d\n",
               motor_id, (int)motor->step_position);
    }

    send_position_update();
    free(param_copy);
}

// Parse SPEED command: SPEED,<interval_us>
void parse_speed_command(const char* params) {
    int speed = atoi(params);

    if (speed > 0 && speed <= 10000) { // Reasonable range: 100Hz to 0.1Hz
        // Update speed for all active motors
        for (uint i = 0; i < active_motor_count; i++) {
            steppers[i].initial_step_interval = (float)speed;
            steppers[i].max_speed = speed / 10; // Max speed is 10x faster than initial
            if (steppers[i].max_speed < 100) steppers[i].max_speed = 100; // Minimum 100μs
        }

        printf("ACK,Velocidade atualizada: inicial=%dμs, max=%dμs\n",
               speed, speed / 10);
    } else {
        printf("ERROR,Velocidade invalida: %d (deve estar entre 1-10000μs)\n", speed);
    }
}

// Parse MODE command: MODE,JOYSTICK or MODE,COMMAND
void parse_mode_command(const char* params) {
    if (strcmp(params, "JOYSTICK") == 0) {
        if (current_state != STATE_JOYSTICK) {
            // Stop all motors before switching to joystick mode
            stop_all_motors();
            current_state = STATE_JOYSTICK;
            printf("ACK,Alterado para o modo JOYSTICK\n");
            send_state_update();
        }
    }
    else if (strcmp(params, "COMMAND") == 0) {
        if (current_state != STATE_COMMAND) {
            // Stop all motors before switching to command mode
            stop_all_motors();
            current_state = STATE_COMMAND;
            printf("ACK,Alterado para o modo de COMANDO\n");
            send_state_update();
        }
    }
    else {
        printf("ERROR,Modo invalido: %s (use JOYSTICK ou COMMAND)\n", params);
    }
}

// Parse MOTORS command: MOTORS,<count>
void parse_motors_command(const char* params) {
    int count = atoi(params);

    if (count >= 1 && count <= 3) {
        // Stop all motors before changing count
        stop_all_motors();

        active_motor_count = (uint8_t)count;
        printf("ACK,Numero de motores definido para %d\n", count);
        send_state_update();
    } else {
        printf("ERROR,Numero de motores invalido: %d (deve estar entre 1-3)\n", count);
    }
}

// Replace your existing parse_moveto_command function with this:
void parse_moveto_command(const char* params) {
    char* param_copy = malloc(strlen(params) + 1);
    strcpy(param_copy, params);

    // Parse parameters in pairs: motor_id, position, motor_id, position, ..., wait_flag
    char* tokens[10]; // Max 4 motors * 2 params + wait flag
    int token_count = 0;

    char* token = strtok(param_copy, ",");
    while (token && token_count < 10) {
        tokens[token_count++] = token;
        token = strtok(NULL, ",");
    }

    if (token_count < 3 || token_count % 2 == 0) {
        printf("ERROR,Parametros de MOVETO invalidos. Formato: motor_id,position[,motor_id,position],wait\n");
        free(param_copy);
        return;
    }

    // Last token is wait flag
    bool wait_for_completion = (atoi(tokens[token_count - 1]) == 1);
    int motor_move_count = (token_count - 1) / 2;

    // Validate motor IDs and prepare movements
    typedef struct {
        int motor_id;
        int32_t target_position;
    } motor_move_t;

    motor_move_t moves[3];
    int valid_moves = 0;

    for (int i = 0; i < motor_move_count; i++) {
        int motor_id = atoi(tokens[i * 2]);
        int32_t position = atoi(tokens[i * 2 + 1]);

        if (motor_id < 0 || motor_id >= (int)active_motor_count || motor_id >= 3) {
            printf("ERROR,ID de motor invalido: %d (faixa entre: 0-%d)\n",
                   motor_id, active_motor_count - 1);
            free(param_copy);
            return;
        }

        moves[valid_moves].motor_id = motor_id;
        moves[valid_moves].target_position = position;
        valid_moves++;
    }

    // Stop all motors that will be moved
    for (int i = 0; i < valid_moves; i++) {
        stepper_motor_t *motor = &steppers[moves[i].motor_id];
        if (motor->alarm_active) {
            stop_motor(motor);
        }
    }

    sleep_ms(10); // Small delay to ensure all motors stop

    printf("ACK,Movendo %d motores para as posicoes especificadas (esperar=%s)\n",
           valid_moves, wait_for_completion ? "true" : "false");

    // Start all movements simultaneously
    for (int i = 0; i < valid_moves; i++) {
        stepper_motor_t *motor = &steppers[moves[i].motor_id];
        int32_t steps = moves[i].target_position - (int32_t)motor->step_position;

        printf("ACK,Motor %d: %d -> %d (%d passos)\n",
               moves[i].motor_id,
               (int)motor->step_position,
               moves[i].target_position,
               steps);

        if (steps != 0) {
            move_n_steps(motor, steps);
        }
    }

    // Wait for all movements to complete if requested
    if (wait_for_completion) {
        bool all_done = false;
        while (!all_done) {
            all_done = true;
            for (int i = 0; i < valid_moves; i++) {
                stepper_motor_t *motor = &steppers[moves[i].motor_id];
                if (motor->alarm_active && !motor->movement_done) {
                    all_done = false;
                    break;
                }
            }
            if (!all_done) {
                sleep_ms(1);
            }
        }

        printf("ACK,Todos os movimentos foram completados\n");
        for (int i = 0; i < valid_moves; i++) {
            printf("ACK,Posicao final do motor %d: %d\n",
                   moves[i].motor_id,
                   (int)steppers[moves[i].motor_id].step_position);
        }
    }

    // Send position update for all motors
    send_position_update();

    free(param_copy);
}

// Replace your existing parse_setzero_command function with this:
void parse_setzero_command(const char* params) {
    if (params && strlen(params) > 0) {
        // SETZERO,<motor_id> - Set specific motor to zero
        int motor_id = atoi(params);
        if (motor_id >= 0 && motor_id < (int)active_motor_count && motor_id < 3) {
            steppers[motor_id].step_position = 0;
            printf("ACK,Motor %d teve posicao zerada\n", motor_id);
        } else {
            printf("ERROR,ID de motor invalido: %d\n", motor_id);
            return;
        }
    } else {
        // SETZERO - Set all motors to zero
        for (uint i = 0; i < active_motor_count && i < 3; i++) {
            steppers[i].step_position = 0;
        }
        printf("ACK,Todos os motores tiveram as posicoes zeradas\n");
    }

    send_position_update();
}