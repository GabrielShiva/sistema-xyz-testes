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

float positiveAccel(float wait_time) {
    float dVelocity = wait_time * acceleration_term;
    wait_time = 1/(dVelocity+1/wait_time);
    if (wait_time < 0.00025f) {
        wait_time = 0.00025f;
    }

    return wait_time;
}

float negativeAccel(float wait_time) {
    float dVelocity = wait_time * -1 * acceleration_term;
    wait_time = 1/(dVelocity+1/wait_time);
    if (wait_time > 0.003f) {
        wait_time = 0.003f;
    }

    return wait_time;
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

        for(int x = 0; x < MICROSTEPS * 7; x++) {
            waitTime = positiveAccel(waitTime);
            roundedWaitTime = round(waitTime * 1000000);
            gpio_put(STEP_PIN, 1);
            sleep_us(roundedWaitTime);
            gpio_put(STEP_PIN, 0);
            sleep_us(roundedWaitTime);
        }

        for(int x = 0; x < MICROSTEPS * 2; x++) {
            waitTime = negativeAccel(waitTime);
            roundedWaitTime = round(waitTime * 1000000);
            gpio_put(STEP_PIN, 1);
            sleep_us(roundedWaitTime);
            gpio_put(STEP_PIN, 0);
            sleep_us(roundedWaitTime);
        }

        sleep_ms(2000);
    }

    return 0;
}
