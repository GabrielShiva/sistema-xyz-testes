/*
* TESTE 01 - MOTOR DE PASSO M49SP-2K (7.5º/step)
* DESC: Esse código testa o funcionamento do motor de passo através da utilização de micropassos (1/16).
*   O motor de passo deve executar revoluções completas no sentido horário e anti-horário em velocidade
*   constante. O desenvolvimento do controle é feita pela geração de um trem de pulsos enviado ao pino
*   STEP. Esses pulsos são gerados pela utilização da função bloqueante sleep_ms(), o que impede a execu-
*   ção de outras tarefas em paralelo.
*/

#include <stdio.h>
#include "pico/stdlib.h"

#define STEP_PIN  1  // pino STEP
#define DIR_PIN   0  // pino DIR
#define BTN_CONTROL 7 // Inicia/Para movimento

// #define CW 1
// #define CCW 0
#define CW 0
#define CCW 1

#define SPR 96 // passos por revolução (360 / 3.75º)

#define MS1_PIN 13
#define MS2_PIN 14
#define MS3_PIN 15

bool mov_state = false;
absolute_time_t last_btn_press = 0;
bool mov_process = false;

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
    const uint steps_per_rev = SPR * 16; // 1536 passos
    const uint64_t pulse_delay_us = (uint64_t)(((1.0f / 96.0f) / 16.0f / 16.0f) * 1e6f); // 651 us

    mov_state = false;

    while(true) {
        if (mov_state) {
            mov_process = true;

            for (int x = 0; x < steps_per_rev * 10; x++) {
                gpio_put(STEP_PIN, 1);
                sleep_us(pulse_delay_us);
                gpio_put(STEP_PIN, 0);
                sleep_us(pulse_delay_us);
            }

            mov_process = false;
            mov_state = false;
        }

        sleep_ms(60);
    }

    return 0;
}
