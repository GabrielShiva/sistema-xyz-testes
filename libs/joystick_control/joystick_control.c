// joystick_control.c

#include "joystick_control.h"
#include "stepper_control.h" // Precisa de start/stop_motor

uint16_t read_joystick_average(int adc_input, int samples, int delay_ms) {
    uint32_t sum = 0;
    for (int i = 0; i < samples; i++) {
        adc_select_input(adc_input);
        sum += adc_read();
        sleep_ms(delay_ms);
    }
    return (uint16_t)(sum / (uint32_t)samples);
}

void control_motor_from_joystick(stepper_motor_t *motor, uint16_t reading, uint16_t center) {
    int delta = (int)reading - (int)center;
    const uint16_t ADC_MAX = (1 << 12) - 1;

    if (abs(delta) <= (int)DEADZONE) {
        if (motor->alarm_active) {
            stop_motor(motor);
        }
    } else {
        motor->dir = (delta > 0) ? 1 : -1;
        gpio_put(motor->dir_pin, (delta < 0) ? 1 : 0);

        uint16_t max_pos_dev = ADC_MAX - center;
        uint16_t max_neg_dev = center;
        float max_dev = (delta > 0) ? (float)max_pos_dev : (float)max_neg_dev;
        if (max_dev <= 0.0f) max_dev = (float)ADC_MAX / 2.0f;

        float norm = (float)abs(delta) / max_dev;
        if (norm > 1.0f) norm = 1.0f;

        float min_int = (float)motor->max_speed;
        float max_int = motor->initial_step_interval;
        float desired_interval = max_int - norm * (max_int - min_int);

        if (desired_interval < MAX_INTERVAL_JOYSTICK) desired_interval = MAX_INTERVAL_JOYSTICK;

        motor->actual_step_interval = desired_interval;
        motor->half_period_interval = desired_interval * 0.5f;

        if (!motor->alarm_active) {
            start_motor_continuous(motor);
        }
    }
}