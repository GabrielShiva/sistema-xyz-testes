#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/adc.h"

// Pin definitions remain the same
#define LED_PIN        25
#define JOYSTICK_X_PIN 26
#define JOYSTICK_Y_PIN 28
#define LIMIT_SWITCH_X_PIN 9
#define LIMIT_SWITCH_Y_PIN 8
#define LIMIT_SWITCH_Z_PIN 10

// Define system states
typedef enum {
    STATE_JOYSTICK = 0,
    STATE_COMMAND = 1,
    STATE_HOMING = 2
} system_state_t;

// Saved position structure
typedef struct {
    char character;
    int32_t x_position;
    int32_t y_position;
    bool is_used;
} saved_position_t;

#define MAX_SAVED_POSITIONS 26
saved_position_t saved_positions[MAX_SAVED_POSITIONS];

// Stepper motor structure - modified for PWM
typedef struct {
    // Control pins
    uint dir_pin;
    uint step_pin;
    uint ms1_pin;
    uint ms2_pin;
    uint ms3_pin;
    uint limit_switch_pin;

    // PWM control
    uint pwm_slice;
    uint pwm_channel;

    // Speed and position control
    volatile float target_frequency;  // Target step frequency in Hz
    volatile float current_frequency; // Current step frequency in Hz
    volatile int32_t step_position;
    volatile int32_t target_position;
    volatile int      dir;
    volatile bool movement_active;

    // Acceleration parameters
    float max_frequency;        // Maximum speed in Hz (steps/sec)
    float min_frequency;        // Starting frequency in Hz
    float acceleration;         // Hz/s^2
    uint32_t last_update_time;  // For acceleration tracking

    // Joystick mode
    volatile bool continuous_mode;

    // Homing
    volatile bool homing_mode;
    volatile bool is_homed;
    volatile bool limit_switch_triggered;

    // Step counting via interrupt
    volatile uint32_t step_count;
    volatile uint32_t target_steps;
} stepper_motor_t;

// Motor instances
stepper_motor_t steppers[3] = {
    { .dir_pin = 16, .step_pin = 17, .ms1_pin = 13, .ms2_pin = 14, .ms3_pin = 15, .limit_switch_pin = LIMIT_SWITCH_X_PIN },
    { .dir_pin = 0, .step_pin = 3, .ms1_pin = 13, .ms2_pin = 14, .ms3_pin = 15, .limit_switch_pin = LIMIT_SWITCH_Y_PIN },
    { .dir_pin = 19, .step_pin = 18, .ms1_pin = 13, .ms2_pin = 14, .ms3_pin = 15, .limit_switch_pin = LIMIT_SWITCH_Z_PIN }
};

// Command buffer
#define COMMAND_BUFFER_SIZE 64
char command_buffer[COMMAND_BUFFER_SIZE];
int command_buffer_pos = 0;
bool command_ready = false;

// Joystick calibration
#define NUM_CAL_SAMPLES 100
#define CAL_SAMPLE_DELAY_MS 5
const uint16_t DEADZONE = 200;
uint16_t joystick_x_center = 2048;
uint16_t joystick_y_center = 2048;

// System state
system_state_t current_state = STATE_JOYSTICK;
uint8_t active_motor_count = 3;

// Homing parameters
#define HOMING_SPEED_SLOW 200.0f   // Hz - slow speed for final approach
#define HOMING_SPEED_FAST 500.0f   // Hz - fast speed for initial movement
#define HOMING_BACKOFF_STEPS 130

// Function declarations
void init_stepper_motor(stepper_motor_t *motor);
void set_motor_frequency(stepper_motor_t *motor, float frequency);
void start_motor_pwm(stepper_motor_t *motor);
void stop_motor_pwm(stepper_motor_t *motor);
void update_motor_acceleration(stepper_motor_t *motor);
void move_n_steps(stepper_motor_t *motor, int32_t steps);
void move_to_position(stepper_motor_t *motor, int32_t target, bool wait);
void stop_all_motors(void);

bool led_callback(repeating_timer_t *rt);
void process_serial_input(void);
void handle_command(const char* command);
void parse_move_command(const char* params);
void parse_speed_command(const char* params);
void parse_mode_command(const char* params);
void parse_motors_command(const char* params);
void parse_moveto_command(const char* params);
void parse_setzero_command(const char* params);
void parse_savepos_command(const char* params);
void parse_recallpos_command(const char* params);
void parse_recallstring_command(const char* params);
void parse_home_command(const char* params);
void parse_clearpos_command(void);
// void parse_testallpos_command(void);
void send_state_update(void);
void send_position_update(void);
void send_homing_status(void);

uint16_t read_joystick_average(int adc_input, int samples, int delay_ms);
void control_motor_from_joystick(stepper_motor_t *motor, uint16_t reading, uint16_t center);

void init_saved_positions(void);
void send_saved_positions_update(void);
int find_saved_position_index(char character);
bool saved_positions_is_empty(void);

bool home_single_motor(stepper_motor_t *motor, int motor_id);
bool home_all_motors(void);
bool read_limit_switch(stepper_motor_t *motor);
void init_limit_switches(void);

// PWM interrupt handlers for step counting
// void pwm_irq_handler_motor0(void);
// void pwm_irq_handler_motor1(void);
// void pwm_irq_handler_motor2(void);

// Initialize stepper motor with PWM
void init_stepper_motor(stepper_motor_t *motor) {
    // Initialize direction pin
    gpio_init(motor->dir_pin);
    gpio_set_dir(motor->dir_pin, GPIO_OUT);

    // Initialize step pin for PWM
    gpio_set_function(motor->step_pin, GPIO_FUNC_PWM);
    motor->pwm_slice = pwm_gpio_to_slice_num(motor->step_pin);
    motor->pwm_channel = pwm_gpio_to_channel(motor->step_pin);

    // Initialize microstepping pins
    gpio_init(motor->ms1_pin);
    gpio_set_dir(motor->ms1_pin, GPIO_OUT);
    gpio_init(motor->ms2_pin);
    gpio_set_dir(motor->ms2_pin, GPIO_OUT);
    gpio_init(motor->ms3_pin);
    gpio_set_dir(motor->ms3_pin, GPIO_OUT);

    // Set 1/16 microstepping
    gpio_put(motor->ms1_pin, 1);
    gpio_put(motor->ms2_pin, 1);
    gpio_put(motor->ms3_pin, 1);

    // Initialize motor parameters
    motor->min_frequency = 300.0f;        // 100 Hz minimum
    motor->max_frequency = 10000.0f;       // 2000 Hz maximum
    motor->acceleration = 10000.0f;        // 5000 Hz/s^2
    motor->current_frequency = 0.0f;
    motor->target_frequency = 0.0f;
    motor->step_position = 0;
    motor->target_position = 0;
    motor->dir = 0;
    motor->movement_active = false;
    motor->continuous_mode = false;
    motor->homing_mode = false;
    motor->is_homed = false;
    motor->limit_switch_triggered = false;
    motor->step_count = 0;
    motor->target_steps = 0;
    motor->last_update_time = to_ms_since_boot(get_absolute_time());

    // Configure PWM but don't start yet
    pwm_config config = pwm_get_default_config();
    pwm_init(motor->pwm_slice, &config, false);

    // Set up PWM interrupt for step counting
    pwm_clear_irq(motor->pwm_slice);
    pwm_set_irq_enabled(motor->pwm_slice, true);
}

// Set motor frequency (speed)
void set_motor_frequency(stepper_motor_t *motor, float frequency) {
    if (frequency < 0.1f) {
        stop_motor_pwm(motor);
        return;
    }

    // Clamp frequency to valid range
    if (frequency > motor->max_frequency) {
        frequency = motor->max_frequency;
    }
    if (frequency < motor->min_frequency && frequency > 0.1f) {
        frequency = motor->min_frequency;
    }

    motor->target_frequency = frequency;

    // Calculate PWM parameters
    // System clock is 125 MHz
    uint32_t sys_clock = 125000000;

    // Calculate divisor and wrap values
    // PWM frequency = sys_clock / (divisor * (wrap + 1))
    // We want 2x the step frequency (for 50% duty cycle square wave)
    float pwm_freq = frequency * 2.0f;

    // Choose divisor to get good resolution
    uint32_t divisor = 1;
    uint32_t wrap;

    if (pwm_freq < 1000.0f) {
        divisor = 125;  // 1 MHz PWM clock
        wrap = (uint32_t)(1000000.0f / pwm_freq) - 1;
    } else {
        divisor = 25;   // 5 MHz PWM clock
        wrap = (uint32_t)(5000000.0f / pwm_freq) - 1;
    }

    if (wrap > 65535) wrap = 65535;
    if (wrap < 1) wrap = 1;

    // Configure PWM
    pwm_set_clkdiv_int_frac(motor->pwm_slice, divisor, 0);
    pwm_set_wrap(motor->pwm_slice, wrap);
    pwm_set_chan_level(motor->pwm_slice, motor->pwm_channel, wrap / 2); // 50% duty cycle
}

// Start motor PWM
void start_motor_pwm(stepper_motor_t *motor) {
    if (!motor->movement_active) {
        motor->movement_active = true;
        motor->step_count = 0;
        set_motor_frequency(motor, motor->min_frequency);
        motor->current_frequency = motor->min_frequency;
        motor->last_update_time = to_ms_since_boot(get_absolute_time());
        pwm_set_enabled(motor->pwm_slice, true);
    }
}

// Stop motor PWM
void stop_motor_pwm(stepper_motor_t *motor) {
    pwm_set_enabled(motor->pwm_slice, false);
    motor->movement_active = false;
    motor->current_frequency = 0.0f;
    motor->target_frequency = 0.0f;
    gpio_put(motor->step_pin, 0);  // Ensure step pin is low
}

// Update motor acceleration (call regularly in main loop)
void update_motor_acceleration(stepper_motor_t *motor) {
    if (!motor->movement_active) return;

    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    float dt = (current_time - motor->last_update_time) / 1000.0f; // Convert to seconds

    if (dt < 0.001f) return; // Update at most every 1ms

    motor->last_update_time = current_time;

    // Check limit switch during homing
    if (motor->homing_mode && read_limit_switch(motor)) {
        motor->limit_switch_triggered = true;
        stop_motor_pwm(motor);
        return;
    }

    // In continuous mode, just maintain target frequency
    if (motor->continuous_mode) {
        motor->current_frequency = motor->target_frequency;
        set_motor_frequency(motor, motor->current_frequency);
        return;
    }

    // Calculate remaining steps
    int32_t remaining_steps = motor->target_steps - motor->step_count;

    if (remaining_steps <= 0) {
        stop_motor_pwm(motor);
        return;
    }

    // Calculate deceleration distance (steps needed to stop from current speed)
    float decel_steps = (motor->current_frequency * motor->current_frequency) / (2.0f * motor->acceleration);

    // Accelerate or decelerate
    if (remaining_steps > decel_steps && motor->current_frequency < motor->max_frequency) {
        // Accelerate
        motor->current_frequency += motor->acceleration * dt;
        if (motor->current_frequency > motor->max_frequency) {
            motor->current_frequency = motor->max_frequency;
        }
    } else {
        // Decelerate
        float target_freq = sqrtf(2.0f * motor->acceleration * remaining_steps);
        if (target_freq < motor->current_frequency) {
            motor->current_frequency -= motor->acceleration * dt;
            if (motor->current_frequency < motor->min_frequency) {
                motor->current_frequency = motor->min_frequency;
            }
        }
    }

    set_motor_frequency(motor, motor->current_frequency);
}

// Unified PWM interrupt handler for all motors
void pwm_irq_handler(void) {
    // Check which PWM slice(s) triggered the interrupt
    uint32_t irq_status = pwm_get_irq_status_mask();

    // Handle motor 0
    if (irq_status & (1u << steppers[0].pwm_slice)) {
        pwm_clear_irq(steppers[0].pwm_slice);
        steppers[0].step_count++;
        steppers[0].step_position += steppers[0].dir;
    }

    // Handle motor 1
    if (irq_status & (1u << steppers[1].pwm_slice)) {
        pwm_clear_irq(steppers[1].pwm_slice);
        steppers[1].step_count++;
        steppers[1].step_position += steppers[1].dir;
    }

    // Handle motor 2
    if (irq_status & (1u << steppers[2].pwm_slice)) {
        pwm_clear_irq(steppers[2].pwm_slice);
        steppers[2].step_count++;
        steppers[2].step_position += steppers[2].dir;
    }
}

// Move motor N steps
void move_n_steps(stepper_motor_t *motor, int32_t steps) {
    if (steps == 0) return;

    // Set direction
    motor->dir = steps > 0 ? 1 : -1;
    gpio_put(motor->dir_pin, steps < 0 ? 1 : 0);

    motor->target_steps = abs(steps);
    motor->step_count = 0;
    motor->continuous_mode = false;

    start_motor_pwm(motor);
}

// Move to absolute position
void move_to_position(stepper_motor_t *motor, int32_t target, bool wait) {
    int32_t steps = target - motor->step_position;
    move_n_steps(motor, steps);

    if (wait) {
        while (motor->movement_active) {
            update_motor_acceleration(motor);
            sleep_ms(1);
        }
    }
}

// Stop all motors
void stop_all_motors(void) {
    for (uint i = 0; i < active_motor_count && i < 3; i++) {
        stop_motor_pwm(&steppers[i]);
    }
}

// Control motor from joystick
void control_motor_from_joystick(stepper_motor_t *motor, uint16_t reading, uint16_t center) {
    int delta = (int)reading - (int)center;
    const uint16_t ADC_MAX = 4095;

    if (abs(delta) <= (int)DEADZONE) {
        if (motor->movement_active) {
            stop_motor_pwm(motor);
        }
        return;
    }

    // Set direction
    motor->dir = delta > 0 ? 1 : -1;
    gpio_put(motor->dir_pin, delta > 0 ? 0 : 1);

    // Calculate normalized position
    uint16_t max_dev = (delta > 0) ? (ADC_MAX - center) : center;
    if (max_dev == 0) max_dev = ADC_MAX / 2;

    float norm = (float)abs(delta) / (float)max_dev;
    if (norm > 1.0f) norm = 1.0f;

    // Map to frequency range
    float freq = motor->min_frequency + norm * (motor->max_frequency - motor->min_frequency);

    motor->continuous_mode = true;
    motor->target_frequency = freq;

    if (!motor->movement_active) {
        start_motor_pwm(motor);
    }
}

// Initialize limit switches
void init_limit_switches(void) {
    gpio_init(LIMIT_SWITCH_X_PIN);
    gpio_set_dir(LIMIT_SWITCH_X_PIN, GPIO_IN);
    gpio_pull_down(LIMIT_SWITCH_X_PIN);

    gpio_init(LIMIT_SWITCH_Y_PIN);
    gpio_set_dir(LIMIT_SWITCH_Y_PIN, GPIO_IN);
    gpio_pull_down(LIMIT_SWITCH_Y_PIN);

    gpio_init(LIMIT_SWITCH_Z_PIN);
    gpio_set_dir(LIMIT_SWITCH_Z_PIN, GPIO_IN);
    gpio_pull_down(LIMIT_SWITCH_Z_PIN);
}

// Read limit switch
bool read_limit_switch(stepper_motor_t *motor) {
    return gpio_get(motor->limit_switch_pin);
}

// Home single motor
bool home_single_motor(stepper_motor_t *motor, int motor_id) {
    // Phase 1: Fast approach
    if (read_limit_switch(motor)) {
        printf("ACK,Motor %d already at limit - backing off\n", motor_id);
        motor->dir = 1;
        gpio_put(motor->dir_pin, 0);
        move_n_steps(motor, HOMING_BACKOFF_STEPS);
        while (motor->movement_active) {
            update_motor_acceleration(motor);
            sleep_ms(1);
        }
        sleep_ms(100);
    }

    printf("ACK,Motor %d: Phase 1 - fast approach\n", motor_id);
    motor->homing_mode = true;
    motor->limit_switch_triggered = false;
    motor->dir = -1;
    gpio_put(motor->dir_pin, 1);
    motor->continuous_mode = true;
    motor->target_frequency = HOMING_SPEED_FAST;
    start_motor_pwm(motor);

    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    while (!motor->limit_switch_triggered && motor->movement_active) {
        update_motor_acceleration(motor);
        if (to_ms_since_boot(get_absolute_time()) - start_time > 50000) {
            stop_motor_pwm(motor);
            printf("ERROR,Timeout homing motor %d\n", motor_id);
            return false;
        }
        sleep_ms(1);
    }

    if (!motor->limit_switch_triggered) {
        printf("ERROR,Motor %d did not reach limit switch\n", motor_id);
        return false;
    }

    printf("ACK,Motor %d: Limit switch reached\n", motor_id);
    sleep_ms(100);

    // Phase 2: Back off
    printf("ACK,Motor %d: Phase 2 - backing off\n", motor_id);
    motor->homing_mode = false;
    motor->dir = 1;
    gpio_put(motor->dir_pin, 0);
    move_n_steps(motor, HOMING_BACKOFF_STEPS);
    while (motor->movement_active) {
        update_motor_acceleration(motor);
        sleep_ms(1);
    }
    sleep_ms(100);

    // Phase 3: Slow final approach
    printf("ACK,Motor %d: Phase 3 - slow final approach\n", motor_id);
    motor->homing_mode = true;
    motor->limit_switch_triggered = false;
    motor->dir = -1;
    gpio_put(motor->dir_pin, 1);
    motor->continuous_mode = true;
    motor->target_frequency = HOMING_SPEED_SLOW;
    start_motor_pwm(motor);

    start_time = to_ms_since_boot(get_absolute_time());
    while (!motor->limit_switch_triggered && motor->movement_active) {
        update_motor_acceleration(motor);
        if (to_ms_since_boot(get_absolute_time()) - start_time > 10000) {
            stop_motor_pwm(motor);
            printf("ERROR,Timeout on final approach motor %d\n", motor_id);
            return false;
        }
        sleep_ms(1);
    }

    motor->step_position = 0;
    motor->is_homed = true;
    motor->homing_mode = false;

    printf("ACK,Motor %d: Home position set\n", motor_id);
    send_position_update();

    return true;
}

// Home all motors
bool home_all_motors(void) {
    bool all_success = true;

    for (uint i = 0; i < active_motor_count && i < 3; i++) {
        printf("ACK,Starting homing for motor %d\n", i);
        if (!home_single_motor(&steppers[i], i)) {
            all_success = false;
            printf("ERROR,Homing failed for motor %d\n", i);
        }
        sleep_ms(500);
    }

    return all_success;
}

// (Continue with remaining functions - process_serial_input, handle_command, etc.)
// These remain largely the same as the original code

int main(void) {
    stdio_init_all();

    // Initialize ADC for joystick
    adc_init();
    adc_gpio_init(JOYSTICK_X_PIN);
    adc_gpio_init(JOYSTICK_Y_PIN);

    // Initialize LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);

    // LED timer
    repeating_timer_t led_timer;
    add_repeating_timer_ms(500, led_callback, NULL, &led_timer);

    // Initialize limit switches
    init_limit_switches();

    // Initialize saved positions
    init_saved_positions();

    // Initialize motors
    for (uint i = 0; i < 3; i++) {
        init_stepper_motor(&steppers[i]);
    }

    // Set up PWM interrupts
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_irq_handler);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    sleep_ms(5000);

    // Calibrate joystick
    joystick_x_center = read_joystick_average(0, NUM_CAL_SAMPLES, CAL_SAMPLE_DELAY_MS);
    printf("Joystick X center (avg of %d samples) = %u\n", NUM_CAL_SAMPLES, joystick_x_center);
    joystick_y_center = read_joystick_average(1, NUM_CAL_SAMPLES, CAL_SAMPLE_DELAY_MS);
    printf("Joystick Y center (avg of %d samples) = %u\n", NUM_CAL_SAMPLES, joystick_y_center);

    printf("Starting motors...\n");

    // Initialize some saved positions (example)
    saved_positions[0] = (saved_position_t){ .character = 'a', .x_position = 25003, .y_position = 2287, .is_used = true };
    saved_positions[1] = (saved_position_t){ .character = 'b', .x_position = 1051, .y_position = 1579, .is_used = true };
    saved_positions[2] = (saved_position_t){ .character = 'c', .x_position = 2818, .y_position = 2109, .is_used = true };
    saved_positions[3] = (saved_position_t){ .character = 'd', .x_position = 0, .y_position = 2195, .is_used = true };
    saved_positions[4] = (saved_position_t){ .character = 'e', .x_position = 4147, .y_position = 955, .is_used = true };
    saved_positions[5] = (saved_position_t){ .character = 'n', .x_position = 14400, .y_position = 670, .is_used = true };
    saved_positions[6] = (saved_position_t){ .character = 'i', .x_position = 12318, .y_position = 4409, .is_used = true };


    // Send initial status
    send_state_update();
    send_position_update();
    send_saved_positions_update();
    send_homing_status();

    while (true) {
        // Update motor acceleration
        for (uint i = 0; i < active_motor_count && i < 3; i++) {
            update_motor_acceleration(&steppers[i]);
        }

        // Process serial input
        process_serial_input();

        if (command_ready) {
            handle_command(command_buffer);
            command_buffer_pos = 0;
            command_ready = false;
        }

        // Joystick control
        if (current_state == STATE_JOYSTICK) {
            adc_select_input(0);
            uint16_t x_reading = adc_read();
            adc_select_input(1);
            uint16_t y_reading = adc_read();

            control_motor_from_joystick(&steppers[0], x_reading, joystick_x_center);

            if (active_motor_count >= 2) {
                control_motor_from_joystick(&steppers[1], y_reading, joystick_y_center);
            }

            // Send joystick data (simplified)
            static int update_counter = 0;
            if (++update_counter >= 10) {
                printf("DATA,%u,%u,%d,%d,%.1f,%.1f\n",
                       x_reading, y_reading,
                       steppers[0].dir,
                       active_motor_count >= 2 ? steppers[1].dir : 0,
                       steppers[0].current_frequency,
                       active_motor_count >= 2 ? steppers[1].current_frequency : 0.0f);
                update_counter = 0;
            }
        }

        // Periodic position updates
        static int position_update_counter = 0;
        if (++position_update_counter >= 60) {
            send_position_update();
            send_homing_status();
            position_update_counter = 0;
        }

        sleep_ms(10);  // Reduced from 25ms for better acceleration updates
    }

    return 0;
}

// LED callback
bool led_callback(repeating_timer_t *rt) {
    gpio_put(LED_PIN, !gpio_get(LED_PIN));
    return true;
}

// Read joystick average
uint16_t read_joystick_average(int adc_input, int samples, int delay_ms) {
    uint32_t sum = 0;
    for (int i = 0; i < samples; i++) {
        adc_select_input(adc_input);
        sum += adc_read();
        sleep_ms(delay_ms);
    }
    return (uint16_t)(sum / samples);
}

// Remaining helper functions (send_state_update, etc.) stay the same...
// Add all parse_* functions and other helper functions from original code here
void send_homing_status(void) {
    printf("HOMING_STATUS");
    for (uint i = 0; i < active_motor_count && i < 3; i++) {
        stepper_motor_t *motor = &steppers[i];
        bool limit_state = read_limit_switch(motor);
        printf(",%d,%d", motor->is_homed ? 1 : 0, limit_state ? 1 : 0);
    }
    printf("\n");
}

// Realiza o processamento dos dados vindos via serial
void process_serial_input(void) {
    int c = getchar_timeout_us(0); // Define uma leitura não bloqueante

    if (c != PICO_ERROR_TIMEOUT) {
        if (c == '\n' || c == '\r') {
            if (command_buffer_pos > 0) {
                command_buffer[command_buffer_pos] = '\0';
                command_ready = true;
            }
        } else if (c >= 32 && c <= 126) {
            if (command_buffer_pos < COMMAND_BUFFER_SIZE - 1) {
                command_buffer[command_buffer_pos++] = (char)c;
            }
        }
    }
}

// Direciona para as funções de acordo com o comando enviado via serial
void handle_command(const char* command) {
    printf("COMANDO RECEBIDO: %s\n", command);

    if (strncmp(command, "MOVE,", 5) == 0) {
        if (current_state != STATE_COMMAND) {
            printf("ERROR,Nao pode mover no modo joystick\n");
            return;
        }
        parse_move_command(command + 5);
    }
    else if (strncmp(command, "SPEED,", 6) == 0) {
        if (current_state != STATE_COMMAND) {
            printf("ERROR,Nao pode definir velocidade no modo joystick\n");
            return;
        }
        parse_speed_command(command + 6);
    }
    else if (strcmp(command, "STOP") == 0) {
        stop_all_motors();
        printf("ACK,Parada de emergencia executada\n");
    }
    else if (strncmp(command, "MODE,", 5) == 0) {
        parse_mode_command(command + 5);
    }
    else if (strncmp(command, "MOTORS,", 7) == 0) {
        parse_motors_command(command + 7);
    }
    else if (strncmp(command, "MOVETO,", 7) == 0) {
        if (current_state != STATE_COMMAND) {
            printf("ERROR,Nao pode posicionar no modo joystick\n");
            return;
        }
        parse_moveto_command(command + 7);
    }
    else if (strncmp(command, "HOME", 4) == 0) {
        if (current_state == STATE_JOYSTICK) {
            printf("ERROR,Nao pode executar homing no modo joystick\n");
            return;
        }
        if (strlen(command) > 4 && command[4] == ',') {
            parse_home_command(command + 5);
        } else {
            parse_home_command(NULL);
        }
    }
    else if (strncmp(command, "SAVEPOS,", 8) == 0) {
        parse_savepos_command(command + 8);
    }
    else if (strncmp(command, "RECALLPOS,", 10) == 0) {
        if (current_state != STATE_COMMAND) {
            printf("ERROR,Nao pode recuperar posicao no modo joystick\n");
            return;
        }
        parse_recallpos_command(command + 10);
    }
    else if (strcmp(command, "CLEARPOS") == 0) {
        parse_clearpos_command();
    }
    else if (strcmp(command, "TESTALLPOS") == 0) {
        bool is_empty = saved_positions_is_empty();

        if (is_empty) {
            printf("ERROR,nenhuma posicao foi encontrada\n");
        } else {
            // parse_testallpos_command();
        }
    }
    else if (strncmp(command, "RECALLSTRING,", 13) == 0) {
        if (current_state != STATE_COMMAND) {
            printf("ERROR,Nao pode executar RECALLSTRING no modo joystick\n");
            return;
        }
        parse_recallstring_command(command + 13);
    }
    else if (strncmp(command, "SETZERO", 7) == 0) {
        if (current_state != STATE_COMMAND) {
            printf("ERROR,Nao pode definir origem no modo joystick\n");
            return;
        }
        if (strlen(command) > 7 && command[7] == ',') {
            parse_setzero_command(command + 8);
        } else {
            parse_setzero_command(NULL);
        }
    }
    else if (strcmp(command, "STATUS") == 0) {
        // Envia o estado de todos os motores ativos
        printf("STATUS");
        // for (uint i = 0; i < active_motor_count && i < 3; i++) {
        //     stepper_motor_t *motor = &steppers[i];
        //     printf(",%d,%d,%d,%.1f",
        //            (int)motor->step_position,
        //            motor->alarm_active ? 1 : 0,
        //            motor->dir,
        //            motor->actual_step_interval);
        // }
        printf("\n");
        send_state_update();
    }
    else {
        printf("ERROR,Comando desconhecido: %s\n", command);
    }
}

// Faz com que o motor execute um movimento
void parse_move_command(const char* params) {
    char* param_copy = malloc(strlen(params) + 1);
    strcpy(param_copy, params);

    char* first_param = strtok(param_copy, ",");
    char* second_param = strtok(NULL, ",");

    int motor_id = 0;
    int steps = 0;

    if (second_param) {
        // Dois parâmetros: motor_id, steps
        motor_id = atoi(first_param);
        steps = atoi(second_param);
    } else {
        steps = atoi(first_param);
    }

    if (motor_id < 0 || motor_id >= (int)active_motor_count || motor_id >= 3) {
        printf("ERROR,ID de motor invalido: %d\n", motor_id);
        free(param_copy);
        return;
    }

    stepper_motor_t *motor = &steppers[motor_id];

    // Para o movimento do motor caso esteja ocorrendo
    if (motor->movement_active) {
        stop_motor_pwm(motor);
        sleep_ms(10);
    }

    printf("ACK,Motor %d movendo %d passos\n", motor_id, steps);

    if (steps != 0) {
        move_n_steps(motor, steps);

        while (motor->movement_active) {
            update_motor_acceleration(motor);
            sleep_ms(1);
        }
        printf("ACK,Motor %d realizou movimento para posicao %d\n",
               motor_id, (int)motor->step_position);
    }

    send_position_update();
    free(param_copy);
}

// Define a velocidade de atuação do motor: SPEED,<interval_us>
void parse_speed_command(const char* params) {
    int frequency = atoi(params);

    if (frequency > 0 && frequency <= 5000) {
        // Atualiza a velocidade de todos os motores ativos
        for (uint i = 0; i < active_motor_count; i++) {
            steppers[i].max_frequency = (float)frequency;
            steppers[i].min_frequency = frequency / 20.0f;
            if (steppers[i].min_frequency < 50.0f) {
                steppers[i].min_frequency = 50.0f;
            }
        }

        printf("ACK,Velocidade atualizada: inicial=%dμs, max=%dμs\n",
               frequency, (int)(frequency / 20.0f));
    } else {
        printf("ERROR,Velocidade invalida: %d (deve estar entre 1-10000μs)\n", frequency);
    }
}

// Define o modo do sistema: MODE,JOYSTICK ou MODE,COMMAND
void parse_mode_command(const char* params) {
    if (strcmp(params, "JOYSTICK") == 0) {
        if (current_state != STATE_JOYSTICK) {
            stop_all_motors();
            current_state = STATE_JOYSTICK;
            printf("ACK,Alterado para o modo JOYSTICK\n");
            send_state_update();
        }
    }
    else if (strcmp(params, "COMMAND") == 0) {
        if (current_state != STATE_COMMAND) {
            stop_all_motors();
            current_state = STATE_COMMAND;
            printf("ACK,Alterado para o modo de COMANDO\n");
            send_state_update();
        }
    }
    else {
        printf("ERROR,Modo invalido: %s (use JOYSTICK ou COMMAND)\n", params);
    }
}

// Define a quantidade de motores ativos: MOTORS,<count>
void parse_motors_command(const char* params) {
    int count = atoi(params);

    if (count >= 1 && count <= 3) {
        stop_all_motors();

        active_motor_count = (uint8_t)count;
        printf("ACK,Numero de motores definido para %d\n", count);
        send_state_update();
    } else {
        printf("ERROR,Numero de motores invalido: %d (deve estar entre 1-3)\n", count);
    }
}

// Faz o motor executar o movimento para uma determinada posição
void parse_moveto_command(const char* params) {
    char* param_copy = malloc(strlen(params) + 1);
    strcpy(param_copy, params);

    char* tokens[10];
    int token_count = 0;

    char* token = strtok(param_copy, ",");
    while (token && token_count < 10) {
        tokens[token_count++] = token;
        token = strtok(NULL, ",");
    }

    if (token_count < 3 || token_count % 2 == 0) {
        printf("ERROR,Parametros de MOVETO invalidos. Formato: motor_id,position[,motor_id,position],wait\n");
        free(param_copy);
        return;
    }

    bool wait_for_completion = (atoi(tokens[token_count - 1]) == 1);
    int motor_move_count = (token_count - 1) / 2;

    typedef struct {
        int motor_id;
        int32_t target_position;
    } motor_move_t;

    motor_move_t moves[3];
    int valid_moves = 0;

    for (int i = 0; i < motor_move_count; i++) {
        int motor_id = atoi(tokens[i * 2]);
        int32_t position = atoi(tokens[i * 2 + 1]);

        if (motor_id < 0 || motor_id >= (int)active_motor_count || motor_id >= 3) {
            printf("ERROR,ID de motor invalido: %d (faixa entre: 0-%d)\n",
                   motor_id, active_motor_count - 1);
            free(param_copy);
            return;
        }

        moves[valid_moves].motor_id = motor_id;
        moves[valid_moves].target_position = position;
        valid_moves++;
    }

    for (int i = 0; i < valid_moves; i++) {
        stepper_motor_t *motor = &steppers[moves[i].motor_id];
        if (motor->movement_active) {
            stop_motor_pwm(motor);
        }
    }

    sleep_ms(10);

    printf("ACK,Movendo %d motores para as posicoes especificadas (esperar=%s)\n",
           valid_moves, wait_for_completion ? "true" : "false");

    // Inicia os movimentos de maneira simultânea
    for (int i = 0; i < valid_moves; i++) {
        stepper_motor_t *motor = &steppers[moves[i].motor_id];
        int32_t steps = moves[i].target_position - (int32_t)motor->step_position;

        printf("ACK,Motor %d: %d -> %d (%d passos)\n",
               moves[i].motor_id,
               (int)motor->step_position,
               moves[i].target_position,
               steps);

        if (steps != 0) {
            move_n_steps(motor, steps);
        }
    }

    // Espera pelo termino do movimento de todos os motores
    if (wait_for_completion) {
        bool all_done = false;
        while (!all_done) {
            all_done = true;
            for (int i = 0; i < valid_moves; i++) {
                stepper_motor_t *motor = &steppers[moves[i].motor_id];
                update_motor_acceleration(motor);
                if (motor->movement_active) {
                    all_done = false;
                    break;
                }
            }
            if (!all_done) {
                sleep_ms(1);
            }
        }

        printf("ACK,Todos os movimentos foram completados\n");
        for (int i = 0; i < valid_moves; i++) {
            printf("ACK,Posicao final do motor %d: %d\n",
                   moves[i].motor_id,
                   (int)steppers[moves[i].motor_id].step_position);
        }
    }

    // Envia o status da posição dos motores
    send_position_update();

    free(param_copy);
}

// Redefine a posição zero para o motor
void parse_setzero_command(const char* params) {
    if (params && strlen(params) > 0) {
        // SETZERO,<motor_id>
        int motor_id = atoi(params);
        if (motor_id >= 0 && motor_id < (int)active_motor_count && motor_id < 3) {
            steppers[motor_id].step_position = 0;
            printf("ACK,Motor %d teve posicao zerada\n", motor_id);
        } else {
            printf("ERROR,ID de motor invalido: %d\n", motor_id);
            return;
        }
    } else {
        // SETZERO - aplica para todos os motores
        for (uint i = 0; i < active_motor_count && i < 3; i++) {
            steppers[i].step_position = 0;
        }
        printf("ACK,Todos os motores tiveram as posicoes zeradas\n");
    }

    send_position_update();
}

// Envia a posição atual dos motores via serial
void send_position_update(void) {
    printf("POSITION");
    for (uint i = 0; i < active_motor_count && i < 3; i++) {
        printf(",%d", (int)steppers[i].step_position);
    }
    printf("\n");
}

// Envia o estado atual do sistema via serial (JOYSTICK, COMMAND, HOMING)
void send_state_update(void) {
    const char* state_str;
    if (current_state == STATE_JOYSTICK) {
        state_str = "JOYSTICK";
    } else if (current_state == STATE_COMMAND) {
        state_str = "COMMAND";
    } else {
        state_str = "HOMING";
    }
    printf("STATE,%s,%d\n", state_str, active_motor_count);
}

// Inicializa o array que salva as posições da letra especificada
void init_saved_positions(void) {
    for (int i = 0; i < MAX_SAVED_POSITIONS; i++) {
        saved_positions[i].character = '\0';
        saved_positions[i].x_position = 0;
        saved_positions[i].y_position = 0;
        saved_positions[i].is_used = false;
    }
}

// Encontra o item do array que contém a letra especificada via interface (serial)
int find_saved_position_index(char character) {
    for (int i = 0; i < MAX_SAVED_POSITIONS; i++) {
        if (saved_positions[i].is_used && saved_positions[i].character == character) {
            return i;
        }
    }

    // Caso a letra não seja encontrada, retorna -1
    return -1;
}

bool saved_positions_is_empty(void) {
    for (int i = 0; i < MAX_SAVED_POSITIONS; i++) {
        if (saved_positions[i].is_used) {
            return false; // Encontrou ao menos um em uso → não está vazio
        }
    }
    return true; // Nenhum em uso → está vazio
}

// Salva a posição atual do atuador e associa a uma letra: SAVEPOS,<character>,<x_pos>,<y_pos>
void parse_savepos_command(const char* params) {
    char* param_copy = malloc(strlen(params) + 1);
    strcpy(param_copy, params);

    char* char_param = strtok(param_copy, ",");
    char* x_param = strtok(NULL, ",");
    char* y_param = strtok(NULL, ",");

    if (!char_param || !x_param || !y_param) {
        printf("ERROR,Parametros insuficientes para SAVEPOS. Formato: character,x_pos,y_pos\n");
        free(param_copy);
        return;
    }

    char character = char_param[0];
    int32_t x_pos = atoi(x_param);
    int32_t y_pos = atoi(y_param);

    // Faz a validação do caractere
    if (!((character >= 'a' && character <= 'z') ||
          (character >= '0' && character <= '9'))) {
        printf("ERROR,Caractere invalido: %c (use a-z ou 0-9)\n", character);
        free(param_copy);
        return;
    }

    int index = find_saved_position_index(character);

    if (index == -1) {
        for (int i = 0; i < MAX_SAVED_POSITIONS; i++) {
            if (!saved_positions[i].is_used) {
                index = i;
                break;
            }
        }
    }

    if (index == -1) {
        printf("ERROR,Memoria de posicoes cheia (maximo %d posicoes)\n", MAX_SAVED_POSITIONS);
        free(param_copy);
        return;
    }

    // Salava as coordenadas no array
    saved_positions[index].character = character;
    saved_positions[index].x_position = x_pos;
    saved_positions[index].y_position = y_pos;
    saved_positions[index].is_used = true;

    printf("ACK,Posicao '%c' salva: X=%d, Y=%d\n", character, x_pos, y_pos);
    send_saved_positions_update();

    free(param_copy);
}

// Recupera a posição da letra especificada: RECALLPOS,<character>
void parse_recallpos_command(const char* params) {
    if (strlen(params) != 1) {
        printf("ERROR,RECALLPOS requer exatamente um caractere\n");
        return;
    }

    // Obtém o caractere especificado no comando
    // e recupera as posições x e y associados ao mesmo
    char character = params[0];
    int index = find_saved_position_index(character);

    if (index == -1) {
        printf("ERROR,Posicao '%c' nao encontrada\n", character);
        return;
    }

    // Define a posição x e y alvo para os motores
    int32_t target_x = saved_positions[index].x_position;
    int32_t target_y = saved_positions[index].y_position;

    printf("ACK,Movendo para posicao salva '%c': X=%d, Y=%d\n",
           character, target_x, target_y);

    // Para todos os motores, caso estejam se movimentando
    stop_all_motors();
    sleep_ms(10);

    // Move motor 0 (eixo x)
    // Realiza a diferença entre a posição atual do motor e a desejada
    stepper_motor_t *motor0 = &steppers[0];
    int32_t steps_x = target_x - (int32_t)motor0->step_position;
    if (steps_x != 0) {
        move_n_steps(motor0, steps_x);
    }

    // Move motor 1 (eixo y)
    if (active_motor_count >= 2) {
        stepper_motor_t *motor1 = &steppers[1];
        int32_t steps_y = target_y - (int32_t)motor1->step_position;
        if (steps_y != 0) {
            move_n_steps(motor1, steps_y);
        }
    }

    // Espera pelo termino dos movimentos
    bool all_done = false;
    while (!all_done) {
        all_done = true;

        update_motor_acceleration(motor0);
        if (motor0->movement_active) {
            all_done = false;
        }

        if (active_motor_count >= 2) {
            stepper_motor_t *motor1 = &steppers[1];
            update_motor_acceleration(motor1);
            if (motor1->movement_active) {
                all_done = false;
            }
        }

        if (!all_done) {
            sleep_ms(1);
        }
    }

     // move o motor do eixo z
    stepper_motor_t *motor2 = &steppers[2];
    move_n_steps(motor2, 2500);
    while (motor2->movement_active) {
        update_motor_acceleration(motor2);
        sleep_ms(1);
    }
    sleep_ms(5);
    move_n_steps(motor2, -2500);
    while (motor2->movement_active) {
        update_motor_acceleration(motor2);
        sleep_ms(1);
    }
    // Delay opcional entre letras
    sleep_ms(300);

    // Espera a tecla ser pressionada
    // char caracter = uart_getc(uart1);

    // while (caracter != character) {
    //     caracter = uart_getc(uart1);
    // }

    // Envia a atualização via serial sobre a posição dos motores
    printf("ACK,Finalizaou movimento\n");
    // printf("ACK,valor recebido = %c\n", caracter);
    send_position_update();
}

// Recupera os caracteres que compõem a string: RECALLSTRING,<texto>
void parse_recallstring_command(const char* params) {
    // Verifica se uma string foi passada, se não, retorna mensagem de erro
    if (!params || strlen(params) == 0) {
        printf("ERROR,RECALLSTRING requer uma string nao vazia\n");
        return;
    }

    // Obtém o tamanho da string
    size_t len = strlen(params);

    for (size_t i = 0; i < len; i++) {
        char character = params[i];
        int index = find_saved_position_index(character);

        // Se o caractere não for encontrado no array, então pula a iteração
        if (index == -1) {
            printf("ERROR,Posicao '%c' nao encontrada (pulando)\n", character);
            continue;
        }

        // Caso o caractere tenha sido encontrado, recupera as posições x e y
        int32_t target_x = saved_positions[index].x_position;
        int32_t target_y = saved_positions[index].y_position;

        // Envia mensagem indicando: movimento iniciado
        printf("ACK,Palavra: '%s' --- Movendo para '%c': X=%d, Y=%d\n",
               params, character, target_x, target_y);

        // Para todos os motores antes de mover
        stop_all_motors();
        sleep_ms(10);

        // Move motor 0 (X)
        stepper_motor_t *motor0 = &steppers[0];
        int32_t steps_x = target_x - (int32_t)motor0->step_position;
        if (steps_x != 0) {
            move_n_steps(motor0, steps_x);
        }

        // Move motor 1 (Y)
        if (active_motor_count >= 2) {
            stepper_motor_t *motor1 = &steppers[1];
            int32_t steps_y = target_y - (int32_t)motor1->step_position;
            if (steps_y != 0) {
                move_n_steps(motor1, steps_y);
            }
        }

        // Espera concluir o movimento
        bool all_done = false;
        while (!all_done) {
            all_done = true;

            update_motor_acceleration(motor0);
            if (motor0->movement_active) {
                all_done = false;
            }

            if (active_motor_count >= 2) {
                stepper_motor_t *motor1 = &steppers[1];
                update_motor_acceleration(motor1);
                if (motor1->movement_active) {
                    all_done = false;
                }
            }
            if (!all_done) {
                sleep_ms(1);
            }
        }

        // Atualiza interface
        send_position_update();

        // move o motor do eixo z
        stepper_motor_t *motor2 = &steppers[2];
        move_n_steps(motor2, 2600);
        while (motor2->movement_active) {
            update_motor_acceleration(motor2);
            sleep_ms(1);
        }
        sleep_ms(50);
        move_n_steps(motor2, -2600);
        while (motor2->movement_active) {
            update_motor_acceleration(motor2);
            sleep_ms(1);
        }
        // Delay opcional entre letras
        sleep_ms(300);
    }

    printf("ACK,Palavra '%s' foi percorrida com sucesso\n", params);
}

// Envia as posições que estão salvas no array para a GUI
void send_saved_positions_update(void) {
    printf("SAVED_POSITIONS");

    for (int i = 0; i < MAX_SAVED_POSITIONS; i++) {
        if (saved_positions[i].is_used) {
            printf(",%c,%d,%d",
                   saved_positions[i].character,
                   saved_positions[i].x_position,
                   saved_positions[i].y_position);
        }
    }

    printf("\n");
}

// Limpa o array de posições: CLEARPOS
void parse_clearpos_command(void) {
    init_saved_positions();
    printf("ACK,Todas as posicoes salvas foram apagadas\n");
    send_saved_positions_update();
}

// Realiza o homing para os motores: HOME,<motor_id> ou somente HOME para todos os motores
void parse_home_command(const char* params) {
    stop_all_motors();

    current_state = STATE_HOMING;
    send_state_update();

    if (params && strlen(params) > 0) {
        // HOME,<motor_id> - um motor em específico
        int motor_id = atoi(params);
        if (motor_id >= 0 && motor_id < (int)active_motor_count && motor_id < 3) {
            printf("ACK,Iniciando homing do motor %d\n", motor_id);

            if (home_single_motor(&steppers[motor_id], motor_id)) {
                printf("ACK,Homing do motor %d concluido\n", motor_id);
            } else {
                printf("ERROR,Falha no homing do motor %d\n", motor_id);
            }
        } else {
            printf("ERROR,ID de motor invalido para homing: %d\n", motor_id);
        }
    } else {
        // HOME - todos os motores
        printf("ACK,Iniciando homing de todos os motores\n");

        if (home_all_motors()) {
            printf("ACK,Homing de todos os motores concluido\n");
        } else {
            printf("ERROR,Falha no homing de alguns motores\n");
        }
    }

    current_state = STATE_COMMAND;
    send_state_update();
    send_homing_status();
}
