#include <stdio.h>
#include "pico/stdlib.h"

#define STEP_PIN   21  // pino STEP
#define DIR_PIN    20  // pino DIR

#define CW   1
#define CCW  0

#define SPR        48      // full-steps per revolution (360°/7.5°)
#define RES        16      // microsteps per full-step
#define MICROSTEPS (SPR * RES)

#define MS1_PIN    13
#define MS2_PIN    14
#define MS3_PIN    15

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

void set_resolution(microstep_t mode) {
    if (mode >= NUM_MICROSTEP_MODES) mode = FULL_STEP;
    gpio_put(MS1_PIN, microstep_table[mode][0]);
    gpio_put(MS2_PIN, microstep_table[mode][1]);
    gpio_put(MS3_PIN, microstep_table[mode][2]);
}

///
/// Rotate the motor `revs` revolutions at `rpm` in `dir` direction.
///
void run_revs(float rpm, uint8_t dir, uint32_t revs) {
    // total pulses to generate:
    uint32_t total_steps = MICROSTEPS * revs;

    // pulses per second = (rpm/60) * MICROSTEPS
    float pps = (rpm / 60.0f) * (float)MICROSTEPS;
    if (pps < 1.0f) pps = 1.0f;

    // full-period in µs for one pulse: Tp = 1/pps seconds = 1e6/pps µs
    uint64_t period_us = (uint64_t)(1e6f / pps);
    uint64_t half_us   = period_us >> 1;

    // set direction:
    gpio_put(DIR_PIN, dir);

    for (uint64_t i = 0; i < total_steps; i++) {
        gpio_put(STEP_PIN, 1);
        sleep_us(half_us);
        gpio_put(STEP_PIN, 0);
        sleep_us(half_us);
    }
}

int main() {
    stdio_init_all();

    // init STEP/DIR pins
    gpio_init(STEP_PIN); gpio_set_dir(STEP_PIN, GPIO_OUT); gpio_put(STEP_PIN, 0);
    gpio_init(DIR_PIN);  gpio_set_dir(DIR_PIN,  GPIO_OUT);

    // init microstep pins
    gpio_init(MS1_PIN);  gpio_set_dir(MS1_PIN, GPIO_OUT);
    gpio_init(MS2_PIN);  gpio_set_dir(MS2_PIN, GPIO_OUT);
    gpio_init(MS3_PIN);  gpio_set_dir(MS3_PIN, GPIO_OUT);

    // set 1/16 microstep
    set_resolution(SIXTEENTH_STEP);

    // --- now do your sequence:
    // 3 rev CW at 120 rpm,
    // 2 rev CCW at  90 rpm,
    // 1 rev CW at  60 rpm
    // run_revs(60.0f, CW, 3);
    // sleep_ms(500);
    // run_revs(60.0f, CCW, 2);
    // sleep_ms(500);
    // run_revs( 60.0f, CW, 1);
    run_revs(150.0f, CW, 1);
    return 0;
}
