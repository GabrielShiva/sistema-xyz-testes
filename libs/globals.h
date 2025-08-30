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

#define COMMAND_BUFFER_SIZE  64
#define NUM_CAL_SAMPLES      100
#define CAL_SAMPLE_DELAY_MS  5
#define DEADZONE             200
#define MAX_INTERVAL_JOYSTICK 1750

// --- Tipos de Dados Globais ---

// Define os estados do sistema
typedef enum {
    STATE_JOYSTICK = 0,
    STATE_COMMAND = 1
} system_state_t;

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

#endif // GLOBALS_H