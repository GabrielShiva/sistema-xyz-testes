#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"

#define STEP_PIN        18
#define DIR_PIN         20
#define POT_ADC_PIN     26  // ADC0

#define STEPS_PER_REV   48
#define DEG_PER_STEP    (360.0f / STEPS_PER_REV)

void step_motor(int steps, uint delay_us) {
    for (int i = 0; i < steps; i++) {
        gpio_put(STEP_PIN, 1);
        sleep_us(delay_us);
        gpio_put(STEP_PIN, 0);
        sleep_us(delay_us);
    }
}

int main() {
    stdio_init_all();

    gpio_init(STEP_PIN);
    gpio_set_dir(STEP_PIN, GPIO_OUT);
    gpio_put(STEP_PIN, 0);

    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);

    adc_init();
    adc_gpio_init(POT_ADC_PIN);
    adc_select_input(0); // ADC 0 -> GP26

    int last_steps = -1;

    while (1) {
        uint16_t raw = adc_read(); // 12-bit valor (0–4095)
        float angle = (raw / 4095.0f) * 360.0f;

        int target_steps = (int)(angle / DEG_PER_STEP + 0.5f);

        if (target_steps != last_steps) {
            last_steps = target_steps;
            gpio_put(DIR_PIN, 0);  // você pode mapear sentido conforme desejar
            step_motor(target_steps, 500); // ajuste velocidade conforme desejar
        }

        sleep_ms(100);
    }
}
