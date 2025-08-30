// globals.h

#ifndef GLOBALS_H
#define GLOBALS_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/adc.h"

// --- Definições de Pinos e Constantes ---
#define LED_PIN              25
#define JOYSTICK_X_PIN       26
#define JOYSTICK_Y_PIN       27

#define LIMIT_SWITCH_X_PIN 9
#define LIMIT_SWITCH_Y_PIN 8

#define COMMAND_BUFFER_SIZE  64
#define NUM_CAL_SAMPLES      100
#define CAL_SAMPLE_DELAY_MS  5
#define DEADZONE             200
#define MAX_INTERVAL_JOYSTICK 1750

#define MAX_SAVED_POSITIONS 26

// Define os parâmetros de homing
#define HOMING_SPEED_SLOW 2000  // μs - slow speed for final approach
#define HOMING_SPEED_FAST 1000  // μs - fast speed for initial movement
#define HOMING_BACKOFF_STEPS 130 // steps to back off from limit switch

// --- Tipos de Dados Globais ---

// Define os estados do sistema
typedef enum {
    STATE_JOYSTICK = 0,
    STATE_COMMAND = 1,
    STATE_HOMING = 2
} system_state_t;

// Estrutura que representa o motor de passo
typedef struct {
    char character;
    int32_t x_position;
    int32_t y_position;
    bool is_used;
} saved_position_t;

// Estrutura que representa o motor de passo
typedef struct {
    // pinos de controle
    uint dir_pin;
    uint step_pin;

    // pinos de resolução
    uint ms1_pin;
    uint ms2_pin;
    uint ms3_pin;

    // variáveis de controle de velocidade
    volatile float initial_step_interval;
    volatile float actual_step_interval;
    volatile float half_period_interval;

    volatile uint32_t total_steps;
    volatile uint32_t step_count;
    volatile uint32_t ramp_up_count;
    volatile int32_t  step_position; // Alterado para int32_t para posições negativas
    volatile uint32_t acceleration_counter;
    volatile uint32_t max_speed;
    volatile int      dir;

    volatile bool movement_done;
    volatile bool step_state;

    // novos campos para controle contínuo via joystick
    volatile bool continuous_mode;
    volatile bool alarm_active;
    alarm_id_t alarm_id;

    // Controle de homing (definição de referência para os eixos x e y))
    volatile bool homing_mode;
    volatile bool is_homed;
    volatile bool limit_switch_triggered;

    // Pino do switch de limite
    uint limit_switch_pin;
} stepper_motor_t;


// --- Declarações de Variáveis Globais (extern) ---

// Declara os motores de passo
extern stepper_motor_t steppers[3];

// Variáveis relacionadas aos comandos
extern char command_buffer[COMMAND_BUFFER_SIZE];
extern int command_buffer_pos;
extern bool command_ready;

// Variáveis do joystick
extern uint16_t joystick_x_center;
extern uint16_t joystick_y_center;

// Variáveis de estado do sistema
extern system_state_t current_state;
extern uint8_t active_motor_count;

extern saved_position_t saved_positions[MAX_SAVED_POSITIONS];

#endif // GLOBALS_H