#include <stdio.h>
#include "pico/stdlib.h"
#include <math.h>

#define STEP_PIN   1  // STEP pin to A4988
#define DIR_PIN    0  // DIR pin to A4988
#define LIMIT_PIN  7  // Limit switch input (normally open, active LOW)

#define CW  1
#define CCW 0

#define MS1_PIN 13
#define MS2_PIN 14
#define MS3_PIN 15

#define STEP_DELAY_US       4000     // Normal stepping delay
#define BACKOFF_STEPS       200     // Steps to back off after hitting switch
#define SLOW_STEP_DELAY_US  5000    // Slower approach after backoff

#define DEBOUNCE_MS         260     // Debounce time for limit switch (ms)

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

// Global debounce variables
volatile bool limit_pressed = false;
absolute_time_t last_interrupt_time;

void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == LIMIT_PIN && (events & GPIO_IRQ_EDGE_FALL)) {
        absolute_time_t now = get_absolute_time();
        if (absolute_time_diff_us(last_interrupt_time, now) > DEBOUNCE_MS * 1000) {
            last_interrupt_time = now;
            limit_pressed = true;
            printf("LIMIT switch pressed (debounced)\n");
        }
    }
}

void set_resolution(microstep_t resolution) {
    if (resolution >= NUM_MICROSTEP_MODES) {
        resolution = FULL_STEP;
    }

    gpio_put(MS1_PIN, microstep_table[resolution][0]);
    gpio_put(MS2_PIN, microstep_table[resolution][1]);
    gpio_put(MS3_PIN, microstep_table[resolution][2]);
}

void step_once(uint delay_us) {
    gpio_put(STEP_PIN, 1);
    sleep_us(delay_us);
    gpio_put(STEP_PIN, 0);
    sleep_us(delay_us);
}

void move_steps(int steps, bool dir, uint delay_us) {
    gpio_put(DIR_PIN, dir);
    for (int i = 0; i < steps; i++) {
        step_once(delay_us);
    }
}

void home_axis() {
    // Move toward switch
    printf("Homing start...\n");
    gpio_put(DIR_PIN, CCW);  // Assume CW moves toward the switch
    limit_pressed = false;
    while (!limit_pressed) {
        step_once(STEP_DELAY_US);
    }

    // Back off a bit
    gpio_put(DIR_PIN, CW);
    move_steps(BACKOFF_STEPS, CW, STEP_DELAY_US);

    // Slowly re-approach
    limit_pressed = false;
    gpio_put(DIR_PIN, CCW);
    while (!limit_pressed) {
        step_once(SLOW_STEP_DELAY_US);
    }

    printf("Homing complete.\n");
}

int main() {
    stdio_init_all();

    // Initialize STEP and DIR pins
    gpio_init(STEP_PIN);
    gpio_set_dir(STEP_PIN, GPIO_OUT);
    gpio_put(STEP_PIN, 0);

    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_put(DIR_PIN, 0);

    // Initialize limit pin with pull-up
    gpio_init(LIMIT_PIN);
    gpio_set_dir(LIMIT_PIN, GPIO_IN);
    gpio_pull_up(LIMIT_PIN);

    // Enable interrupt on falling edge for limit pin
    gpio_set_irq_enabled_with_callback(LIMIT_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    last_interrupt_time = get_absolute_time();

    // Setup microstep resolution pins
    gpio_init(MS1_PIN); gpio_set_dir(MS1_PIN, GPIO_OUT);
    gpio_init(MS2_PIN); gpio_set_dir(MS2_PIN, GPIO_OUT);
    gpio_init(MS3_PIN); gpio_set_dir(MS3_PIN, GPIO_OUT);

    set_resolution(SIXTEENTH_STEP);  // 1/16 step resolution

    sleep_ms(5000);  // Wait before starting

    home_axis();  // Start homing

    // Main idle loop
    while (1) {
        sleep_ms(1000);
    }

    return 0;
}
