#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/irq.h"

#define STEP_PIN  1  // pino STEP
#define DIR_PIN   0  // pino DIR
// #define STEP_PIN  16  // pino STEP
// #define DIR_PIN   17  // pino DIR
#define LED_PIN   25

#define MS1_PIN 13
#define MS2_PIN 14
#define MS3_PIN 15

// Use alarm 0
#define ALARM_NUM 0
#define ALARM_IRQ TIMER_IRQ_0

typedef enum {
    FULL_STEP = 0,
    HALF_STEP,
    QUARTER_STEP,
    EIGHTH_STEP,
    SIXTEENTH_STEP,
    NUM_MICROSTEP_MODES
} microstep_t;

static const uint8_t microstep_table[NUM_MICROSTEP_MODES][3] = {
    [FULL_STEP]      = {0, 0, 0},
    [HALF_STEP]      = {1, 0, 0},
    [QUARTER_STEP]   = {0, 1, 0},
    [EIGHTH_STEP]    = {1, 1, 0},
    [SIXTEENTH_STEP] = {1, 1, 1},
};

void set_resolution(microstep_t resolution) {
    if (resolution >= NUM_MICROSTEP_MODES) {
        resolution = FULL_STEP;
    }

    gpio_put(MS1_PIN, microstep_table[resolution][0]);
    gpio_put(MS2_PIN, microstep_table[resolution][1]);
    gpio_put(MS3_PIN, microstep_table[resolution][2]);
}

static volatile uint c0; // valor inicial de tempo entre pulsos
static volatile int dir = 0; // Indica a direção de rotação
static volatile uint max_speed = 100; // maior velocidade possível
static volatile uint32_t n = 0; // contador durante aceleração
static volatile float d; // tempo entre os passos atual
static volatile uint32_t step_count = 0; // total de passos dados
static volatile uint32_t ramp_up_step_count = 0; // numero de passos para atingir max_speed
static volatile uint32_t total_steps = 0; // total de passos do movimento
static volatile uint32_t step_position = 0; // posição atual
static volatile bool movement_done = false; // flag que indica se o movimento terminou
static volatile bool step_state = false;
static volatile float half_period_us = 0;
static volatile uint counter = 0;

int64_t alarm_irq_handler(alarm_id_t id, void *user_data) {
    if (step_count >= total_steps) {
        movement_done = true;
        return 0;
    }

    if (!step_state) {
        // subida do pulso
        gpio_put(STEP_PIN, 1);
        step_state = true;

        // agendamos o “fim” do pulso (meio período)
        return (int64_t)half_period_us;
    } else {
        // descida do pulso
        gpio_put(STEP_PIN, 0);
        step_state = false;

        step_count++;
        step_position += dir;

        if (ramp_up_step_count == 0) {
            n++;
            d = d - (2.0f * d) / (4.0f * n + 1.0f);

            if (d <= max_speed) {
                d = max_speed;
                ramp_up_step_count = step_count;
            }
            if (step_count >= total_steps / 2) {
                ramp_up_step_count = step_count;
            }
        } else if (step_count >= total_steps - ramp_up_step_count) {
            n--;
            d = (d * (4.0f * n + 1.0f)) / (4.0f * n + 1.0f - 2.0f);
        }
        // Recalcula metade do período
        half_period_us = d * 0.5f;

        counter++;

        // Reagenda próximo alarme em half_period_us
        return (int64_t)half_period_us;
    }
}

void move_n_steps (int32_t steps) {
    gpio_put(DIR_PIN, steps < 0 ? 1 : 0);
    dir = steps > 0 ? 1 : -1;
    total_steps = abs(steps);
    d = (float)c0;
    step_count = 0;
    n = 0;
    ramp_up_step_count = 0;
    movement_done = false;
    half_period_us = d * 0.5f;
    step_state = false;

    add_alarm_in_us((int64_t)half_period_us, alarm_irq_handler, NULL, false);
}

void move_to_position(int32_t p, bool wait) {
    move_n_steps(p - step_position);
    while(wait && !movement_done);
}

int main() {
    stdio_init_all();

    // Inicializa pinos STEP e DIR
    gpio_init(STEP_PIN); gpio_set_dir(STEP_PIN, GPIO_OUT); gpio_put(STEP_PIN, 0);
    gpio_init(DIR_PIN);  gpio_set_dir(DIR_PIN, GPIO_OUT);

    // Inicializa os pinos de microstep
    gpio_init(MS1_PIN); gpio_set_dir(MS1_PIN, GPIO_OUT);
    gpio_init(MS2_PIN); gpio_set_dir(MS2_PIN, GPIO_OUT);
    gpio_init(MS3_PIN); gpio_set_dir(MS3_PIN, GPIO_OUT);

    // Configura microstep 1/16
    set_resolution(SIXTEENTH_STEP);

    // Inicial valores de rampa
    c0 = 6200;          // 6400 us

    sleep_ms(4000);
    printf("Comecou !!!\n");

    while (true) {
        move_to_position(1600, true);
        move_to_position(-1600, true);
        move_to_position(0, true);

        move_to_position(200, true);
        move_to_position(400, true);
        move_to_position(600, true);
        move_to_position(800, true);

        move_to_position(400, true);
        move_to_position(600, true);
        move_to_position(200, true);
        move_to_position(400, true);
        move_to_position(0, true);

        max_speed = 600;
        move_to_position(200, true);
        move_to_position(400, true);

        max_speed = 400;
        move_to_position(600, true);
        move_to_position(800, true);

        max_speed = 200;
        move_to_position(1000, true);
        move_to_position(1200, true);

        max_speed = 10;
        move_to_position(0, true);

        printf("Fim\n");

        while(true);
    }

    return 0;
}
