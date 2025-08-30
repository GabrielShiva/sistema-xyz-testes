// stepper_control.c

#include "stepper_control.h"

int64_t alarm_irq_handler(alarm_id_t id, void *user_data) {
    // Obtém a instância do motor
    stepper_motor_t *motor = (stepper_motor_t*)user_data;

    // Se em modo contínuo, gerencia pulso sem a lógica de rampa por passos finitos
    if (motor->continuous_mode) {
        if (!motor->step_state) {
            gpio_put(motor->step_pin, 1);
            motor->step_state = true;
            return (int64_t)motor->half_period_interval;
        } else {
            gpio_put(motor->step_pin, 0);
            motor->step_state = false;
            motor->step_count++;
            motor->step_position += motor->dir;
            motor->half_period_interval = motor->actual_step_interval * 0.5f;

            if (motor->movement_done) {
                motor->alarm_active = false;
                motor->continuous_mode = false;
                return 0;
            }
            return (int64_t)motor->half_period_interval;
        }
    }

    // Caso o movimento tenha acabado
    if (motor->step_count >= motor->total_steps) {
        motor->movement_done = true;
        motor->alarm_active = false;
        return 0;
    }

    // Lógica de pulso e rampa para movimento discreto
    if (!motor->step_state) {
        gpio_put(motor->step_pin, 1);
        motor->step_state = true;
        return (int64_t)motor->half_period_interval;
    } else {
        gpio_put(motor->step_pin, 0);
        motor->step_state = false;
        motor->step_count++;
        motor->step_position += motor->dir;

        // Implementa a aceleração/desaceleração
        if (motor->ramp_up_count == 0) {
            motor->acceleration_counter++;
            motor->actual_step_interval = motor->actual_step_interval - (2.0f * motor->actual_step_interval) / (4.0f * motor->acceleration_counter + 1.0f);
            if (motor->actual_step_interval <= motor->max_speed) {
                motor->actual_step_interval = motor->max_speed;
                motor->ramp_up_count = motor->step_count;
            }
            if (motor->step_count >= motor->total_steps / 2) {
                motor->ramp_up_count = motor->step_count;
            }
        } else if (motor->step_count >= motor->total_steps - motor->ramp_up_count) {
            motor->acceleration_counter--;
            motor->actual_step_interval = (motor->actual_step_interval * (4.0f * motor->acceleration_counter + 1.0f)) / (4.0f * motor->acceleration_counter + 1.0f - 2.0f);
        }

        motor->half_period_interval = motor->actual_step_interval * 0.5f;
        return (int64_t)motor->half_period_interval;
    }
}

void init_stepper_motor(stepper_motor_t *motor) {
    gpio_init(motor->step_pin); gpio_set_dir(motor->step_pin, GPIO_OUT); gpio_put(motor->step_pin, 0);
    gpio_init(motor->dir_pin); gpio_set_dir(motor->dir_pin, GPIO_OUT);
    gpio_init(motor->ms1_pin); gpio_set_dir(motor->ms1_pin, GPIO_OUT);
    gpio_init(motor->ms2_pin); gpio_set_dir(motor->ms2_pin, GPIO_OUT);
    gpio_init(motor->ms3_pin); gpio_set_dir(motor->ms3_pin, GPIO_OUT);

    gpio_put(motor->ms1_pin, 1); gpio_put(motor->ms2_pin, 1); gpio_put(motor->ms3_pin, 1);

    motor->initial_step_interval = 6200.0f;
    motor->actual_step_interval = motor->initial_step_interval;
    motor->half_period_interval = motor->actual_step_interval * 0.5f;
    motor->max_speed = 100;
    motor->movement_done = false;
    motor->step_state = false;
    motor->step_position = 0;
    motor->total_steps = 0;
    motor->step_count = 0;
    motor->ramp_up_count = 0;
    motor->acceleration_counter = 0;
    motor->dir = 0;
    motor->continuous_mode = false;
    motor->alarm_active = false;
    motor->alarm_id = 0;
}

void move_n_steps(stepper_motor_t *motor, int32_t steps) {
    if (steps == 0) return;
    motor->total_steps = abs(steps);
    motor->step_count = 0;
    motor->ramp_up_count = 0;
    motor->acceleration_counter = 0;
    motor->movement_done = false;
    motor->step_state = false;
    motor->actual_step_interval = motor->initial_step_interval;
    motor->half_period_interval = motor->actual_step_interval * 0.5f;
    motor->dir = steps > 0 ? 1 : -1;
    gpio_put(motor->dir_pin, steps < 0 ? 1 : 0);
    motor->alarm_id = add_alarm_in_us((int64_t)motor->half_period_interval, alarm_irq_handler, motor, false);
    motor->alarm_active = true;
}

void move_to_position(stepper_motor_t *motor, int32_t target, bool wait) {
    move_n_steps(motor, target - motor->step_position);
    if (wait) {
        while (!motor->movement_done);
    }
}

void start_motor_continuous(stepper_motor_t *motor) {
    if (motor->alarm_active) return;
    motor->continuous_mode = true;
    motor->movement_done = false;
    motor->step_state = false;
    motor->step_count = 0;
    motor->half_period_interval = motor->actual_step_interval * 0.5f;
    motor->alarm_id = add_alarm_in_us((int64_t)motor->half_period_interval, alarm_irq_handler, motor, false);
    motor->alarm_active = true;
}

void stop_motor(stepper_motor_t *motor) {
    if (!motor->alarm_active) return;
    cancel_alarm(motor->alarm_id);
    motor->alarm_active = false;
    motor->continuous_mode = false;
    motor->movement_done = true;
    motor->step_state = false;
    gpio_put(motor->step_pin, 0);
}

void stop_all_motors(void) {
    for (uint i = 0; i < active_motor_count && i < 3; i++) {
        if (steppers[i].alarm_active) {
            stop_motor(&steppers[i]);
        }
    }
}