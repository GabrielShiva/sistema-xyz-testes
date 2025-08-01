#include <stdio.h>
#include "pico/stdlib.h"

#define STEP_PIN   21
#define DIR_PIN    20

#define CW   1
#define CCW  0

#define SPR        48      // full-steps/rev (360°/7.5°)
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

// Run one revolution while ramping up to `target_hz`
void run_rev_with_ramp(uint dir, float target_hz) {
    gpio_put(DIR_PIN, dir);

    for (uint i = 0; i < MICROSTEPS; i++) {
        // fraction of the way through the rev (0.0 → 1.0)
        float frac = (float)i / (float)MICROSTEPS;
        // current frequency ramps from 0 → target_hz
        float freq = frac * target_hz;
        if (freq < 1.0f) freq = 1.0f;  // avoid divide-by-zero

        // half-period in microseconds = 1e6 / (2 * freq)
        uint64_t half_period = (uint64_t)(1e6f / (2.0f * freq));

        gpio_put(STEP_PIN, 1);
        sleep_us(half_period);
        gpio_put(STEP_PIN, 0);
        sleep_us(half_period);
    }
}

int main() {
    stdio_init_all();

    // init STEP/DIR
    gpio_init(STEP_PIN); gpio_set_dir(STEP_PIN, GPIO_OUT);
    gpio_init(DIR_PIN);  gpio_set_dir(DIR_PIN,  GPIO_OUT);
    gpio_put(STEP_PIN, 0);

    // init microstep pins
    gpio_init(MS1_PIN);  gpio_set_dir(MS1_PIN, GPIO_OUT);
    gpio_init(MS2_PIN);  gpio_set_dir(MS2_PIN, GPIO_OUT);
    gpio_init(MS3_PIN);  gpio_set_dir(MS3_PIN, GPIO_OUT);

    // 1/16 resolution
    set_resolution(SIXTEENTH_STEP);

    // Ramp up one CW rev to 800 Hz
    run_rev_with_ramp(CW, 800.0f);

    sleep_ms(500);

    // Ramp up one CCW rev to 800 Hz
    run_rev_with_ramp(CCW, 800.0f);

    return 0;
}
