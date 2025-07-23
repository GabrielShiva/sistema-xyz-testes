#include <stdio.h>
#include "pico/stdlib.h"

#define STEP_PIN  18  // pino STEP
#define DIR_PIN   20  // pino DIR
#define BTN_PIN   5  // botão de controle

#define CW 1
#define CCW 0

#define STEPS_PER_REV 48

// 7.5º por passo -> 48 passos por revolução

int main() {
    stdio_init_all();

    gpio_init(STEP_PIN);
    gpio_set_dir(STEP_PIN, GPIO_OUT);
    gpio_put(STEP_PIN, 0);

    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_put(DIR_PIN, CW);  // direção inicial

    uint step_count = STEPS_PER_REV;
    uint delay = 21; //1s dividido por 48

    while (true) {
        for (int x = 0; x < step_count; x++) {
            gpio_put(STEP_PIN, 1);
            sleep_ms(delay);
            gpio_put(STEP_PIN, 0);
            sleep_ms(delay);
        }

        sleep_ms(500);
        gpio_put(DIR_PIN, CCW);  // muda direção inicial

        for (int x = 0; x < step_count; x++) {
            gpio_put(STEP_PIN, 1);
            sleep_ms(delay);
            gpio_put(STEP_PIN, 0);
            sleep_ms(delay);
        }

        sleep_ms(500);
        gpio_put(DIR_PIN, CW);  // muda direção inicial
    }

    return 0;
}
