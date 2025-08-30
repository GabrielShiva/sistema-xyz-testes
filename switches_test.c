#include <stdio.h>
#include "pico/stdlib.h"

#define LIMIT_X_PIN 9
#define LIMIT_Y_PIN 8

// Interrupt handler
void gpio_callback(uint gpio, uint32_t events) {
    if (gpio == LIMIT_X_PIN) {
        printf("Limit switch X triggered!\n");
    }
    else if (gpio == LIMIT_Y_PIN) {
        printf("Limit switch Y triggered!\n");
    }
}

int main() {
    stdio_init_all();

    // Configure GPIOs as input with pull-down
    gpio_init(LIMIT_X_PIN);
    gpio_set_dir(LIMIT_X_PIN, GPIO_IN);
    gpio_pull_down(LIMIT_X_PIN);

    gpio_init(LIMIT_Y_PIN);
    gpio_set_dir(LIMIT_Y_PIN, GPIO_IN);
    gpio_pull_down(LIMIT_Y_PIN);

    // Add interrupts for rising edge (NO -> goes HIGH when pressed)
    gpio_set_irq_enabled_with_callback(LIMIT_X_PIN, GPIO_IRQ_EDGE_FALL, true, &gpio_callback);
    gpio_set_irq_enabled(LIMIT_Y_PIN, GPIO_IRQ_EDGE_FALL, true);

    printf("Waiting for limit switch presses...\n");

    // Main loop does nothing; everything handled by interrupt
    while (true) {
        tight_loop_contents();
    }
}
