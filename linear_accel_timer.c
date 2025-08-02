#include "pico/stdlib.h"
#include "hardware/timer.h"
#include <stdlib.h>  // Include for labs()

#define DIR_PIN          2
#define STEP_PIN         3

#define PULSE_WIDTH_US   60  // Step pulse width in microseconds

const unsigned int c0 = 1600 * 4;  // Initial step delay in microseconds

// Global state variables
volatile int dir = 0;
volatile unsigned int maxSpeedMicros;  // Max step interval in microseconds
volatile unsigned long n = 0;
volatile unsigned long stepCount = 0;
volatile unsigned long rampUpStepCount = 0;
volatile unsigned long totalSteps = 0;
volatile long stepPosition = 0;
volatile bool movementDone = true;  // Start with no movement

volatile unsigned long d = c0;  // Current step delay in microseconds
static alarm_id_t step_alarm_id = 0; // Alarm ID for step generation

// Alarm callback function for step generation
int64_t step_alarm_callback(alarm_id_t id, void *user_data) {
    // Generate step pulse
    gpio_put(STEP_PIN, 1);
    busy_wait_us_32(PULSE_WIDTH_US);
    gpio_put(STEP_PIN, 0);

    stepCount++;
    stepPosition += dir;

    // Check if all steps are completed
    if (stepCount >= totalSteps) {
        movementDone = true;
        step_alarm_id = 0;
        return 0; // Stop the timer
    }

    // Update acceleration profile
    if (rampUpStepCount == 0) { // Ramp-up phase
        n++;
        d = d - (2 * d) / (4 * n + 1);
        if (d <= maxSpeedMicros) {
            d = maxSpeedMicros;
            rampUpStepCount = stepCount;
        }
        if (stepCount >= totalSteps / 2) {
            rampUpStepCount = stepCount;
        }
    } else if (stepCount >= totalSteps - rampUpStepCount) { // Ramp-down phase
        if (n > 0) { // Avoid division by zero
            n--;
            d = (d * (4 * n + 1)) / (4 * n + 1 - 2);
        }
    }

    // Return the delay until the next step in microseconds
    return (int64_t)d;
}

void moveNSteps(long steps) {
    // Cancel any ongoing movement
    if (step_alarm_id) {
        cancel_alarm(step_alarm_id);
        step_alarm_id = 0;
    }

    // Configure direction and step parameters
    gpio_put(DIR_PIN, (steps < 0) ? 1 : 0);
    dir = (steps > 0) ? 1 : -1;
    totalSteps = labs(steps);  // Use labs for long integers
    d = c0;
    stepCount = 0;
    n = 0;
    rampUpStepCount = 0;
    movementDone = false;

    // Schedule the first step
    step_alarm_id = add_alarm_in_us(d, step_alarm_callback, NULL, true);
}

void moveToPosition(long position, bool wait) {
    long steps = position - stepPosition;
    moveNSteps(steps);
    while (wait && !movementDone) {
        tight_loop_contents();
    }
}

int main() {
    stdio_init_all();

    // Initialize GPIO pins
    gpio_init(DIR_PIN);
    gpio_set_dir(DIR_PIN, GPIO_OUT);
    gpio_init(STEP_PIN);
    gpio_set_dir(STEP_PIN, GPIO_OUT);

    // Main loop with movement sequences
    while (1) {
        maxSpeedMicros = 10 * 4; // 40 us
        moveToPosition(1600, true);
        moveToPosition(-1600, true);
        moveToPosition(0, true);

        moveToPosition(200, true);
        moveToPosition(400, true);
        moveToPosition(600, true);
        moveToPosition(800, true);

        moveToPosition(400, true);
        moveToPosition(600, true);
        moveToPosition(200, true);
        moveToPosition(400, true);
        moveToPosition(0, true);

        maxSpeedMicros = 600 * 4; // 2400 us
        moveToPosition(200, true);
        moveToPosition(400, true);

        maxSpeedMicros = 400 * 4; // 1600 us
        moveToPosition(600, true);
        moveToPosition(800, true);

        maxSpeedMicros = 200 * 4; // 800 us
        moveToPosition(1000, true);
        moveToPosition(1200, true);

        maxSpeedMicros = 10 * 4; // 40 us
        moveToPosition(0, true);

        while (true) {}  // Halt after sequence
    }
}
