#include <stdio.h>
#include "pico/stdlib.h"
#include <math.h>

#define STEP_PIN  21  // pino STEP
#define DIR_PIN   20  // pino DIR
#define BTN_CONTROL 16 // Inicia/Para movimento

#define CW 1
#define CCW 0

#define SPR 48 // passos por revolução (360 / 7.5)
#define MICROSTEPS 768

#define MS1_PIN 13
#define MS2_PIN 14
#define MS3_PIN 15

bool mov_state = false;
bool mov_process = false;

absolute_time_t last_btn_press = 0;

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

void btn_irq(uint gpio, uint32_t events) {
    absolute_time_t curr = to_ms_since_boot(get_absolute_time());

    if (gpio == BTN_CONTROL && !mov_process && curr - last_btn_press > 300) {
        curr = last_btn_press;

        mov_state = !mov_state;
    }
}

void const_accel(bool is_accel) {
    uint64_t delays[MICROSTEPS];
    float angle = 1.0f;
    float accel = 0.05f;
    float c0 = 2000.0f * sqrtf(2.0f * angle / accel) * 0.67703f; //19149
    float lastDelay = 0.0f;
    int highSpeed = 300;

    for (int i = 0; i < MICROSTEPS; i++) {
        float d = c0;

        if (i > 0) {
            d = lastDelay - (2 * lastDelay) / (4 * i + 1);
        }

        if (d < highSpeed) {
            d = highSpeed;
        }

        delays[i] = (uint64_t)d;
        lastDelay = d;
    }

    if (is_accel) {
        for (int i = 0; i < MICROSTEPS; i++) {
            gpio_put(STEP_PIN, 1);
            sleep_us(delays[i]);
            gpio_put(STEP_PIN, 0);
            sleep_us(delays[i]);
        }
    } else {
        for (int i = 0; i < MICROSTEPS; i++) {
            gpio_put(STEP_PIN, 1);
            sleep_us(delays[MICROSTEPS - i - 1]);
            gpio_put(STEP_PIN, 0);
            sleep_us(delays[MICROSTEPS - i - 1]);
        }
    }
}

int main() {
    stdio_init_all();

    // Inicializa Botão
    gpio_init(BTN_CONTROL); gpio_set_dir(BTN_CONTROL, GPIO_IN); gpio_pull_up(BTN_CONTROL);
    gpio_set_irq_enabled_with_callback(BTN_CONTROL, GPIO_IRQ_EDGE_FALL, true, &btn_irq);

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

    mov_state = false;
    gpio_put(DIR_PIN, 1);

    while(true) {
        if (mov_state) {
            mov_process = true;

            // acelera de 0 até 300 us
            const_accel(true);

            //mantém em 300 us por duas revoluções no sentido horário
            for (int i = 0; i < MICROSTEPS * 4; i++) {
                gpio_put(STEP_PIN, 1);
                sleep_us(300);
                gpio_put(STEP_PIN, 0);
                sleep_us(300);
            }

            // desacelerá de 300 us até 0
            const_accel(false);
            sleep_ms(500);

            mov_process = false;
            mov_state = false;
        } else {
            sleep_ms(60);
        }
    }

    return 0;
}
