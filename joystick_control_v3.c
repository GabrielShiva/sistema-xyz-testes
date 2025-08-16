#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"

#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/adc.h"

#define LED_PIN        25
#define JOYSTICK_X_PIN 26

// System state enumeration
typedef enum {
    STATE_JOYSTICK = 0,
    STATE_COMMAND = 1
} system_state_t;

// Estrutura que representa o motor de passo
typedef struct {
    // pinos de controle
    uint dir_pin;
    uint step_pin;

    // pinos de resolução
    uint ms1_pin;
    uint ms2_pin;
    uint ms3_pin;

    // variáveis de controle de velocidade
    volatile float initial_step_interval;
    volatile float actual_step_interval;
    volatile float half_period_interval;

    volatile uint32_t total_steps;
    volatile uint32_t step_count;
    volatile uint32_t ramp_up_count;
    volatile uint32_t step_position;
    volatile uint32_t acceleration_counter;
    volatile uint32_t max_speed;
    volatile int      dir;

    volatile bool movement_done;
    volatile bool step_state;

    // novos campos para controle contínuo via joystick
    volatile bool continuous_mode;
    volatile bool alarm_active;
    alarm_id_t alarm_id;
} stepper_motor_t;

stepper_motor_t steppers[3] = {
    { .dir_pin = 0, .step_pin = 1, .ms1_pin = 13, .ms2_pin = 14, .ms3_pin = 15 },
    { .dir_pin = 17, .step_pin = 16, .ms1_pin = 13, .ms2_pin = 14, .ms3_pin = 15 }
};

// Add these global variables after your stepper_motor_t definitions
#define COMMAND_BUFFER_SIZE 64
char command_buffer[COMMAND_BUFFER_SIZE];
int command_buffer_pos = 0;
bool command_ready = false;

// System state variables
system_state_t current_state = STATE_JOYSTICK;
uint8_t active_motor_count = 1;

// Declaração de funções
int64_t alarm_irq_handler(alarm_id_t id, void *user_data);
void init_stepper_motor(stepper_motor_t *motor);
void move_n_steps(stepper_motor_t *motor, int32_t steps);
void move_to_position(stepper_motor_t *motor, int32_t target, bool wait);
bool led_callback(repeating_timer_t *rt);
// Add these function prototypes after your existing ones
void process_serial_input(void);
void handle_command(const char* command);
void parse_move_command(const char* params);
void parse_speed_command(const char* params);
void parse_mode_command(const char* params);
void parse_motors_command(const char* params);
void send_state_update(void);
void stop_all_motors(void);

// novos protótipos
void start_motor_continuous(stepper_motor_t *motor);
void stop_motor(stepper_motor_t *motor);
uint16_t read_joystick_average(int samples, int delay_ms);

// Função principal
int main (void) {
    stdio_init_all();

    adc_init();
    adc_gpio_init(JOYSTICK_X_PIN);
    adc_select_input(0);

    gpio_init(LED_PIN); gpio_set_dir(LED_PIN, GPIO_OUT); gpio_put(LED_PIN, 0);

    repeating_timer_t led_timer;

    // Inicializa os motores
    for (uint i = 0; i < 2; i++) {
        init_stepper_motor(&steppers[i]);
    }

    const int MOTOR_STEPS = 48;
    int32_t steps_per_rev = MOTOR_STEPS * 16; // 768 passos/rev

    // steppers[0].initial_step_interval = 4000.0f; // 4 ms entre passos no início
    // steppers[0].max_speed             = 1000;    // não acelera além de 1 ms por passo

    // steppers[1].initial_step_interval = 7000.0f; // mais lento no início
    // steppers[1].max_speed             = 500;    // limite mais lento para este motor

    // Cria instância do timer para o LED
    add_repeating_timer_ms(500, led_callback, NULL, &led_timer);

    sleep_ms(3000);

    // =========================
    // Calibração inicial do joystick: lê 100 amostras e define o centro
    // =========================
    const int NUM_CAL_SAMPLES = 100;
    const int CAL_SAMPLE_DELAY_MS = 5;
    uint16_t joystick_center = read_joystick_average(NUM_CAL_SAMPLES, CAL_SAMPLE_DELAY_MS);
    printf("Joystick center (avg %d samples) = %u\n", NUM_CAL_SAMPLES, joystick_center);
    sleep_ms(2000);

    // margem morta (deadzone) em contagens ADC, ajuste se necessário
    const uint16_t DEADZONE = 200; // ~30/4095 ~ small tolerance. Ajuste conforme ruído do seu joystick

    const uint16_t MAX_INTERVAL_JOYSTICK = 1750; // us

    // referência para normalização
    const uint16_t ADC_MAX = (1 << 12) - 1; // 4095

    // Configurações do motor 0 (já definidas em init, mas deixo claro aqui caso queira alterar)
    stepper_motor_t *m0 = &steppers[0];

    printf("Iniciando demo para 3 motores...\n");

    // Send initial state after initialization
    send_state_update();

    while (true) {
        // Process any incoming serial commands
        process_serial_input();

        if (command_ready) {
            handle_command(command_buffer);
            command_buffer_pos = 0;
            command_ready = false;
        }

        // Only process joystick input in JOYSTICK mode
        if (current_state == STATE_JOYSTICK) {
            uint16_t reading = adc_read();
            int delta = (int)reading - (int)joystick_center;

            if (abs(delta) <= (int)DEADZONE) {
                // within deadzone -> ensure motor is stopped
                if (m0->alarm_active) {
                    stop_motor(m0);
                }
            } else {
                // outside deadzone -> direction and speed proportional to distance from center
                if (delta > 0) {
                    m0->dir = 1;
                    gpio_put(m0->dir_pin, 0);
                } else {
                    m0->dir = -1;
                    gpio_put(m0->dir_pin, 1);
                }

                // Calculate normalization using max distance from center
                uint16_t max_pos_dev = ADC_MAX - joystick_center;
                uint16_t max_neg_dev = joystick_center;
                float max_dev = (delta > 0) ? (float)max_pos_dev : (float)max_neg_dev;
                if (max_dev <= 0.0f) max_dev = (float)ADC_MAX / 2.0f;

                float norm = (float)abs(delta) / max_dev;
                if (norm > 1.0f) norm = 1.0f;

                // Map norm (0..1) to step interval
                float min_int = (float)m0->max_speed;
                float max_int = m0->initial_step_interval;
                float desired_interval = max_int - norm * (max_int - min_int);

                if (desired_interval < MAX_INTERVAL_JOYSTICK) desired_interval = MAX_INTERVAL_JOYSTICK;

                m0->actual_step_interval = desired_interval;
                m0->half_period_interval = desired_interval * 0.5f;

                // Start motor in continuous mode if not already running
                if (!m0->alarm_active) {
                    start_motor_continuous(m0);
                }
            }

            // Send data only in joystick mode
            printf("DATA,%u,%d,%.1f\n", reading, m0->dir, m0->actual_step_interval);
        } else {
            // In command mode, send minimal data or status
            // You can modify this based on what data you want to send in command mode
            printf("DATA,0,0,0.0\n"); // Or comment this out if no data needed
        }

        sleep_ms(10); // 50 Hz loop rate
    }

    return 0;
}

int64_t alarm_irq_handler(alarm_id_t id, void *user_data) {
    // Obtém a instância do motor
    stepper_motor_t *motor = (stepper_motor_t*)user_data;

    // Se em modo contínuo, gerencia pulso sem a lógica de rampa por passos finitos
    if (motor->continuous_mode) {
        if (!motor->step_state) {
            // pulso alto
            gpio_put(motor->step_pin, 1);
            motor->step_state = true;
            // Retorna metade do período (us) até próximo toggle
            return (int64_t)motor->half_period_interval;
        } else {
            // pulso baixo
            gpio_put(motor->step_pin, 0);
            motor->step_state = false;

            // Atualiza contadores de posição
            motor->step_count++;
            motor->step_position += motor->dir;

            // Atualiza half_period de acordo com actual_step_interval (que é atualizada pelo loop principal)
            motor->half_period_interval = motor->actual_step_interval * 0.5f;

            // Se por alguma razão movement_done foi solicitado, interrompe
            if (motor->movement_done) {
                motor->alarm_active = false;
                motor->continuous_mode = false;
                return 0; // cancela agendamento
            }

            return (int64_t)motor->half_period_interval;
        }
    }

    // Caso o movimento tenha acabado, interrompe o agendamento de alarmes
    if (motor->step_count >= motor->total_steps) {
        motor->movement_done = true;
        motor->alarm_active = false;
        return 0;
    }

    // Verifica o estado do pulso enviado ao pino STEP
    if (!motor->step_state) {
        // Produz um nível lógico alto
        gpio_put(motor->step_pin, 1);
        motor->step_state = true;

        return (int64_t)motor->half_period_interval;
    } else {
        // Produz um nível lógico baixo
        gpio_put(motor->step_pin, 0);
        motor->step_state = false;

        // Incrementa o contador de passos e atualiza a posição do motor
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
            motor->actual_step_interval = (motor->actual_step_interval * (4.0f * motor->acceleration_counter + 1)) / (4.0f * motor->acceleration_counter + 1 - 2);
        }

        // Computa o próximo meio-período
        motor->half_period_interval = motor->actual_step_interval * 0.5f;
        return (int64_t)motor->half_period_interval;
    }
}

// função de callback para o LED ficar piscando
bool led_callback(repeating_timer_t *rt) {
    gpio_put(LED_PIN, !gpio_get(LED_PIN));
    return true;
}

void init_stepper_motor(stepper_motor_t *motor) {
    gpio_init(motor->step_pin);
    gpio_set_dir(motor->step_pin, GPIO_OUT);
    gpio_put(motor->step_pin, 0);

    gpio_init(motor->dir_pin);
    gpio_set_dir(motor->dir_pin, GPIO_OUT);

    gpio_init(motor->ms1_pin);
    gpio_set_dir(motor->ms1_pin, GPIO_OUT);
    gpio_init(motor->ms2_pin);
    gpio_set_dir(motor->ms2_pin, GPIO_OUT);
    gpio_init(motor->ms3_pin);
    gpio_set_dir(motor->ms3_pin, GPIO_OUT);

    // Define a resolução de 1/16
    gpio_put(motor->ms1_pin, 1);
    gpio_put(motor->ms2_pin, 1);
    gpio_put(motor->ms3_pin, 1);

    // Valores iniciais para a rampa
    motor->initial_step_interval = 6200.0f;
    motor->actual_step_interval  = motor->initial_step_interval;
    motor->half_period_interval  = motor->actual_step_interval * 0.5f;
    motor->max_speed             = 100;
    motor->movement_done         = false;
    motor->step_state            = false;
    motor->step_position         = 0;
    motor->total_steps           = 0;
    motor->step_count            = 0;
    motor->ramp_up_count         = 0;
    motor->acceleration_counter  = 0;
    motor->dir                   = 0;
    motor->continuous_mode       = false;
    motor->alarm_active          = false;
    motor->alarm_id              = 0;
}

void move_n_steps(stepper_motor_t *motor, int32_t steps) {
    // Reseta os valores da rampa
    motor->total_steps          = abs(steps);
    motor->step_count           = 0;
    motor->ramp_up_count        = 0;
    motor->acceleration_counter = 0;
    motor->movement_done        = false;
    motor->step_position        = motor->step_position;
    motor->step_state           = false;
    motor->actual_step_interval = motor->initial_step_interval;
    motor->half_period_interval = motor->actual_step_interval * 0.5f;

    // Define a direção de rotação
    motor->dir = steps > 0 ? 1 : -1;
    gpio_put(motor->dir_pin, steps < 0 ? 1 : 0);

    // Agenda o alarme para o primeiro pulso
    motor->alarm_id = add_alarm_in_us((int64_t)motor->half_period_interval, alarm_irq_handler, motor, false);
    motor->alarm_active = true;
}

void move_to_position(stepper_motor_t *motor, int32_t target, bool wait) {
    move_n_steps(motor, target - (int32_t)motor->step_position);
    if (wait) {
        while (!motor->movement_done);
    }
}

// Inicia o motor em modo contínuo (usado para joystick)
void start_motor_continuous(stepper_motor_t *motor) {
    if (motor->alarm_active) return; // já rodando

    motor->continuous_mode = true;
    motor->movement_done = false;
    motor->step_state = false;
    motor->step_count = 0; // pode manter contagem se quiser
    motor->half_period_interval = motor->actual_step_interval * 0.5f;
    motor->alarm_id = add_alarm_in_us((int64_t)motor->half_period_interval, alarm_irq_handler, motor, false);
    motor->alarm_active = true;
}

// Para o motor (modo contínuo)
void stop_motor(stepper_motor_t *motor) {
    if (!motor->alarm_active) return;
    cancel_alarm(motor->alarm_id);
    motor->alarm_active = false;
    motor->continuous_mode = false;
    motor->movement_done = true;
    motor->step_state = false;
    gpio_put(motor->step_pin, 0);
}

// Lê N amostras do ADC e retorna a média
uint16_t read_joystick_average(int samples, int delay_ms) {
    uint32_t sum = 0;
    for (int i = 0; i < samples; i++) {
        uint16_t r = adc_read();
        sum += r;
        sleep_ms(delay_ms);
    }
    return (uint16_t)(sum / (uint32_t)samples);
}

// Add this function to process incoming serial data
void process_serial_input(void) {
    int c = getchar_timeout_us(0); // Non-blocking read

    if (c != PICO_ERROR_TIMEOUT) {
        if (c == '\n' || c == '\r') {
            if (command_buffer_pos > 0) {
                command_buffer[command_buffer_pos] = '\0';
                command_ready = true;
            }
        } else if (c >= 32 && c <= 126) { // Printable characters only
            if (command_buffer_pos < COMMAND_BUFFER_SIZE - 1) {
                command_buffer[command_buffer_pos++] = (char)c;
            }
        }
    }
}

// Main command handler
void handle_command(const char* command) {
    printf("Received command: %s\n", command);

    if (strncmp(command, "MOVE,", 5) == 0) {
        if (current_state != STATE_COMMAND) {
            printf("ERROR,Cannot move in joystick mode\n");
            return;
        }
        parse_move_command(command + 5);
    }
    else if (strncmp(command, "SPEED,", 6) == 0) {
        if (current_state != STATE_COMMAND) {
            printf("ERROR,Cannot set speed in joystick mode\n");
            return;
        }
        parse_speed_command(command + 6);
    }
    else if (strcmp(command, "STOP") == 0) {
        // Emergency stop works in any mode
        stop_all_motors();
        printf("ACK,Emergency stop executed\n");
    }
    else if (strncmp(command, "MODE,", 5) == 0) {
        parse_mode_command(command + 5);
    }
    else if (strncmp(command, "MOTORS,", 7) == 0) {
        parse_motors_command(command + 7);
    }
    else if (strcmp(command, "STATUS") == 0) {
        // Send status information
        stepper_motor_t *m0 = &steppers[0];
        printf("STATUS,%d,%d,%d,%.1f\n",
               (int)m0->step_position,
               m0->alarm_active ? 1 : 0,
               m0->dir,
               m0->actual_step_interval);
        send_state_update();
    }
    else {
        printf("ERROR,Unknown command: %s\n", command);
    }
}

// Parse MOVE command: MOVE,<steps>
void parse_move_command(const char* params) {
    int steps = atoi(params);
    stepper_motor_t *m0 = &steppers[0]; // For now, only control motor 0

    // Stop current movement if any
    if (m0->alarm_active) {
        stop_motor(m0);
        sleep_ms(10); // Small delay to ensure stop
    }

    printf("ACK,Moving %d steps\n", steps);

    if (steps != 0) {
        move_n_steps(m0, steps);

        // Wait for movement to complete
        while (!m0->movement_done && m0->alarm_active) {
            sleep_ms(1);
        }
        printf("ACK,Movement completed\n");
    }
}

// Parse MODE command: MODE,JOYSTICK or MODE,COMMAND
void parse_mode_command(const char* params) {
    if (strcmp(params, "JOYSTICK") == 0) {
        if (current_state != STATE_JOYSTICK) {
            // Stop all motors before switching to joystick mode
            stop_all_motors();
            current_state = STATE_JOYSTICK;
            printf("ACK,Switched to joystick mode\n");
            send_state_update();
        }
    }
    else if (strcmp(params, "COMMAND") == 0) {
        if (current_state != STATE_COMMAND) {
            // Stop all motors before switching to command mode
            stop_all_motors();
            current_state = STATE_COMMAND;
            printf("ACK,Switched to command mode\n");
            send_state_update();
        }
    }
    else {
        printf("ERROR,Invalid mode: %s (use JOYSTICK or COMMAND)\n", params);
    }
}

// Parse MOTORS command: MOTORS,<count>
void parse_motors_command(const char* params) {
    int count = atoi(params);

    if (count >= 1 && count <= 3) {
        // Stop all motors before changing count
        stop_all_motors();

        active_motor_count = (uint8_t)count;
        printf("ACK,Active motor count set to %d\n", count);
        send_state_update();
    } else {
        printf("ERROR,Invalid motor count: %d (must be 1-3)\n", count);
    }
}

// Send current state to GUI
void send_state_update(void) {
    const char* state_str = (current_state == STATE_JOYSTICK) ? "JOYSTICK" : "COMMAND";
    printf("STATE,%s,%d\n", state_str, active_motor_count);
}

// Stop all active motors
void stop_all_motors(void) {
    for (uint i = 0; i < active_motor_count && i < 3; i++) {
        if (steppers[i].alarm_active) {
            stop_motor(&steppers[i]);
        }
    }
}

// Parse SPEED command: SPEED,<interval_us>
void parse_speed_command(const char* params) {
    int speed = atoi(params);

    if (speed > 0 && speed <= 10000) { // Reasonable range: 100Hz to 0.1Hz
        // Update speed for all active motors
        for (uint i = 0; i < active_motor_count; i++) {
            steppers[i].initial_step_interval = (float)speed;
            steppers[i].max_speed = speed / 10; // Max speed is 10x faster than initial
            if (steppers[i].max_speed < 100) steppers[i].max_speed = 100; // Minimum 100μs
        }

        printf("ACK,Speed updated: initial=%dμs, max=%dμs\n",
               speed, speed / 10);
    } else {
        printf("ERROR,Invalid speed: %d (must be 1-10000μs)\n", speed);
    }
}
