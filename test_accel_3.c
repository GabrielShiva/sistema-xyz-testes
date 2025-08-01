#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"

#define STEP_PIN    21
#define DIR_PIN     20
float waitTime = 0.003f;
int acceleration_term = 4000;
int roundedWaitTime;
uint32_t timer1;
uint32_t timer2;
uint32_t difference;

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

// classic step-delay recurrence
static inline uint32_t next_delay(uint32_t delay, uint32_t step_index) {
    return delay - ((2 * delay) / (4 * step_index + 1));
}

void accelerate_one_rev(float start_rpm, float end_rpm) {
    // convert RPM → delay in µs
    uint32_t delay_start = (uint32_t)(1e6f / ( (start_rpm/60)*MICROSTEPS ));
    uint32_t delay_end   = (uint32_t)(1e6f / ( (end_rpm/60)*MICROSTEPS ));

    uint32_t delay = delay_start;
    for (uint32_t i = 1; i <= MICROSTEPS; ++i) {
        // linearly step the delay toward delay_end
        delay = delay - (delay - delay_end) * i / MICROSTEPS;

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

        gpio_put(DIR_PIN, CW);
        accelerate_one_rev(60.0f, 120.0f);

        sleep_ms(2000);
    }

    return 0;
}
