#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/time.h"

#define STEP_PIN 21
#define DIR_PIN 20
#define MS1_PIN 13
#define MS2_PIN 14
#define MS3_PIN 15
#define LED_PIN 25

#define CW 1
#define CCW 0
#define SPR 48

typedef enum {
    FULL_STEP,
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

typedef struct {
    volatile uint32_t steps_remaining;
    volatile bool step_state;
} step_state_t;

bool step_callback(repeating_timer_t *rt) {
    step_state_t *s = (step_state_t *)rt->user_data;
    s->step_state = !s->step_state;
    gpio_put(STEP_PIN, s->step_state);
    if (!s->step_state) {
        if (s->steps_remaining > 0) s->steps_remaining--;
    }
    return s->steps_remaining > 0;
}

void set_resolution(microstep_t res) {
    if (res >= NUM_MICROSTEP_MODES) res = FULL_STEP;
    gpio_put(MS1_PIN, microstep_table[res][0]);
    gpio_put(MS2_PIN, microstep_table[res][1]);
    gpio_put(MS3_PIN, microstep_table[res][2]);
}

bool led_callback(repeating_timer_t *rt) {
    gpio_xor_mask(1u << LED_PIN);
    return true;
}

int main() {
    stdio_init_all();
    gpio_init(LED_PIN); gpio_set_dir(LED_PIN, GPIO_OUT); gpio_put(LED_PIN, 0);

    gpio_init_mask((1u<<STEP_PIN)|(1u<<DIR_PIN)|(1u<<MS1_PIN)|(1u<<MS2_PIN)|(1u<<MS3_PIN));
    gpio_set_dir(STEP_PIN, GPIO_OUT);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_set_dir(MS1_PIN, GPIO_OUT);
    gpio_set_dir(MS2_PIN, GPIO_OUT);
    gpio_set_dir(MS3_PIN, GPIO_OUT);
    gpio_put(STEP_PIN, 0);

    set_resolution(SIXTEENTH_STEP);
    const uint steps_per_rev = SPR * 16; // 768
    const uint64_t pulse_delay_us = (uint64_t)(((1.0f/48.0f)/16.0f)/2.0f * 1e6f); // ~651 µs
    int64_t interval = -(int64_t)pulse_delay_us;

    step_state_t state = { .steps_remaining = 0, .step_state = false };
    repeating_timer_t timer, led_timer;

    // LED blinks at, say, 2 Hz (every 500 ms)
    add_repeating_timer_ms(500, led_callback, NULL, &led_timer);

    while (true) {
        // Phase 1: 3 rev CW
        gpio_put(DIR_PIN, CW);
        state.steps_remaining = steps_per_rev * 5;
        add_repeating_timer_us(interval, step_callback, &state, &timer);
        while (state.steps_remaining) tight_loop_contents();
        cancel_repeating_timer(&timer);

        sleep_ms(2000);
    }

    return 0;
}
