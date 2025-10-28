#include <stdio.h>
#include "pico/stdlib.h"

#define LED_PIN    25
#define SWITCH_PIN 10

int main (void) {
    stdio_init_all();

    gpio_init(SWITCH_PIN);
    gpio_set_dir(SWITCH_PIN, GPIO_IN);
    gpio_pull_up(SWITCH_PIN);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    while (true) {
        bool sensor_state = gpio_get(SWITCH_PIN);

        // Depending on the sensor logic:
        // If output LOW means beam interrupted, use !sensor_state
        if (!sensor_state) {  // Beam interrupted
            gpio_put(LED_PIN, 1);  // Turn LED ON
        } else {
            gpio_put(LED_PIN, 0);  // Turn LED OFF
        }

        sleep_ms(10);  // small debounce/delay
    }


    return 0;
}
