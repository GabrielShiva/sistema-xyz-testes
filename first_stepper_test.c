#include <stdio.h>
#include "pico/stdlib.h"

#define STEP_PIN  21  // pino STEP
#define DIR_PIN   20  // pino DIR

#define CW 1
#define CCW 0

#define SPR 48 // passos por revolução (360 / 7.5)

#define MS1_PIN 13
#define MS2_PIN 14
#define MS3_PIN 15

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

int main() {
    stdio_init_all();

    // Inicializa pinos STEP e DIR
    gpio_init(STEP_PIN); gpio_set_dir(STEP_PIN, GPIO_OUT); gpio_put(STEP_PIN, 0);
    gpio_init(DIR_PIN);  gpio_set_dir(DIR_PIN, GPIO_OUT); gpio_put(DIR_PIN, CW);

    // Inicializa os pinos de microstep
    gpio_init(MS1_PIN); gpio_set_dir(MS1_PIN, GPIO_OUT);
    gpio_init(MS2_PIN); gpio_set_dir(MS2_PIN, GPIO_OUT);
    gpio_init(MS3_PIN); gpio_set_dir(MS3_PIN, GPIO_OUT);

    // Configura microstep 1/16
    set_resolution(SIXTEENTH_STEP);

    // Define o número de passos por revolução com o micropasso de 1/16
    const uint steps_per_rev = SPR * 16; // 768 passos
    const uint64_t pulse_delay_us = (uint64_t)(((1.0f / 48.0f) / 16.0f) / 2.0f * 1e6f); // 651 us

    for (int x = 0; x < steps_per_rev * 3; x++) {
        gpio_put(STEP_PIN, 1);
        sleep_us(pulse_delay_us);
        gpio_put(STEP_PIN, 0);
        sleep_us(pulse_delay_us);
    }

    sleep_ms(1000);
    gpio_put(DIR_PIN, CCW);  // muda direção inicial

    for (int x = 0; x < steps_per_rev * 2; x++) {
        gpio_put(STEP_PIN, 1);
        sleep_us(pulse_delay_us);
        gpio_put(STEP_PIN, 0);
        sleep_us(pulse_delay_us);
    }

    sleep_ms(1000);
    gpio_put(DIR_PIN, CW);  // muda direção inicial

    for (int x = 0; x < steps_per_rev; x++) {
        gpio_put(STEP_PIN, 1);
        sleep_us(pulse_delay_us);
        gpio_put(STEP_PIN, 0);
        sleep_us(pulse_delay_us);
    }

    return 0;
}
