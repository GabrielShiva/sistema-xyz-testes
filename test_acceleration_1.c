#include <stdio.h>
#include "pico/stdlib.h"

#define STEP_PIN    21
#define DIR_PIN     20

#define CW   1
#define CCW  0

#define SPR         48       // full-steps per rev (360°/7.5°)
#define RES         16       // microsteps per full-step
#define MICROSTEPS  (SPR * RES)

#define MS1_PIN     13
#define MS2_PIN     14
#define MS3_PIN     15

// Microstep resolution table…
static const uint8_t microstep_table[][3] = {
    {0,0,0},  // FULL
    {1,0,0},  // HALF
    {0,1,0},  // QUARTER
    {1,1,0},  // EIGHTH
    {1,1,1},  // SIXTEENTH
};

void set_resolution(int mode) {
    if (mode < 0 || mode > 4) mode = 0;
    gpio_put(MS1_PIN, microstep_table[mode][0]);
    gpio_put(MS2_PIN, microstep_table[mode][1]);
    gpio_put(MS3_PIN, microstep_table[mode][2]);
}

/// Generate exactly `steps` pulses at a fixed half-period `delay_us`
void pulse_steps(uint32_t steps, uint32_t delay_us) {
    for (uint32_t i = 0; i < steps; i++) {
        gpio_put(STEP_PIN, 1);
        sleep_us(delay_us);
        gpio_put(STEP_PIN, 0);
        sleep_us(delay_us);
    }
}

/// Ramp the half-period delay linearly from `start_us` → `end_us`
/// over exactly `MICROSTEPS` pulses (one full rev) in direction `dir`.
void ramp_revolution(uint32_t start_us, uint32_t end_us, uint8_t dir) {
    gpio_put(DIR_PIN, dir);
    int32_t delta = (int32_t)end_us - (int32_t)start_us;
    for (uint32_t i = 0; i < MICROSTEPS; i++) {
        // linear interpolation
        uint32_t delay = start_us + ((delta * (int32_t)i) / (int32_t)MICROSTEPS);
        gpio_put(STEP_PIN, 1);
        sleep_us(delay);
        gpio_put(STEP_PIN, 0);
        sleep_us(delay);
    }
}

int main() {
    stdio_init_all();

    // init STEP/DIR
    gpio_init(STEP_PIN); gpio_set_dir(STEP_PIN, GPIO_OUT);
    gpio_init(DIR_PIN);  gpio_set_dir(DIR_PIN,  GPIO_OUT);

    // init microstep pins
    gpio_init(MS1_PIN);  gpio_set_dir(MS1_PIN, GPIO_OUT);
    gpio_init(MS2_PIN);  gpio_set_dir(MS2_PIN, GPIO_OUT);
    gpio_init(MS3_PIN);  gpio_set_dir(MS3_PIN, GPIO_OUT);

    // set 1/16-step
    set_resolution(4);

    while (1) {
        sleep_ms(1000);

        // 1) Ramp from slow (3000µs half-period) down to fast (700µs)
        ramp_revolution(5000, 300, CW);

        // 2) Hold constant at 700µs half-period for 3 full revs
        gpio_put(DIR_PIN, CW);
        pulse_steps(MICROSTEPS * 3, 300);

        sleep_ms(2000);
    }

    return 0;
}
