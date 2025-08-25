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
#define JOYSTICK_Y_PIN 27

// Define os estados do sistema
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

// Declara os motores de passo
stepper_motor_t steppers[3] = {
    { .dir_pin = 0, .step_pin = 1, .ms1_pin = 13, .ms2_pin = 14, .ms3_pin = 15 },
    { .dir_pin = 17, .step_pin = 16, .ms1_pin = 13, .ms2_pin = 14, .ms3_pin = 15 }
};

// Define variáveis realacionadas aos comandos enviados da interface
#define COMMAND_BUFFER_SIZE 64
char command_buffer[COMMAND_BUFFER_SIZE];
int command_buffer_pos = 0;
bool command_ready = false;

// Constantes para leitura do joystick
#define NUM_CAL_SAMPLES 100
#define CAL_SAMPLE_DELAY_MS 5
const uint16_t DEADZONE = 200;
const uint16_t MAX_INTERVAL_JOYSTICK = 1750; // Máxima velocidade imposta sobre o comando
uint16_t joystick_x_center = 2048;
uint16_t joystick_y_center = 2048;

// Define o estado atual do sistema e a quantidade de motores ativos no momento
system_state_t current_state = STATE_JOYSTICK;
uint8_t active_motor_count = 2;

// Declaração de funções
// Controle dos motores
int64_t alarm_irq_handler(alarm_id_t id, void *user_data);
void init_stepper_motor(stepper_motor_t *motor);
void move_n_steps(stepper_motor_t *motor, int32_t steps);
void move_to_position(stepper_motor_t *motor, int32_t target, bool wait);
void stop_all_motors(void);
void start_motor_continuous(stepper_motor_t *motor);
void stop_motor(stepper_motor_t *motor);

// Declaração de funções
// Definição de outras tarefas
bool led_callback(repeating_timer_t *rt);

// Declaração de funções
// Processamento de comandos vindos da interface
void process_serial_input(void);
void handle_command(const char* command);
void parse_move_command(const char* params);
void parse_speed_command(const char* params);
void parse_mode_command(const char* params);
void parse_motors_command(const char* params);
void parse_moveto_command(const char* params);
void parse_setzero_command(const char* params);
void send_state_update(void);
void send_position_update(void);

// Declaração de funções
// Leitura de dados do joystick
uint16_t read_joystick_average(int adc_input, int samples, int delay_ms);
void control_motor_from_joystick(stepper_motor_t *motor, uint16_t reading, uint16_t center);


int main (void) {
    stdio_init_all();

    // Inicializa o ADC para o joystick
    adc_init();
    adc_gpio_init(JOYSTICK_X_PIN);
    adc_gpio_init(JOYSTICK_Y_PIN);

    // Inicializa o LED do raspberry
    gpio_init(LED_PIN); gpio_set_dir(LED_PIN, GPIO_OUT); gpio_put(LED_PIN, 0);

    // Cria instância do timer para o LED e chama a cada 500 ms
    repeating_timer_t led_timer;
    add_repeating_timer_ms(500, led_callback, NULL, &led_timer);

    // Inicializa os motores
    for (uint i = 0; i < 2; i++) {
        init_stepper_motor(&steppers[i]);
    }

    // Delay para dar tempo de ligar o serial
    sleep_ms(5000);

    // Calibração inicial do joystick: lê 100 amostras e define o centro
    // Nesse momento, o joystick não deve ser movido
    joystick_x_center = read_joystick_average(0, NUM_CAL_SAMPLES, CAL_SAMPLE_DELAY_MS);
    printf("Joystick X center (avg %d samples) = %u\n", NUM_CAL_SAMPLES, joystick_x_center);
    joystick_y_center = read_joystick_average(1, NUM_CAL_SAMPLES, CAL_SAMPLE_DELAY_MS);
    printf("Joystick Y center (avg %d samples) = %u\n", NUM_CAL_SAMPLES, joystick_y_center);

    printf("Iniciando demo para 2 motores...\n");

    // Envia o estado do sistema e a posição atual para a interface (INICIALIZAÇÃO DA INTERFACE)
    send_state_update();
    send_position_update();

    while (true) {
        // Processa os dados vindos via serial (dados enviados pela interface)
        process_serial_input();

        // Se o comando foi processado, lidar com o comando
        if (command_ready) {
            // Executa a função relacionada ao comando
            handle_command(command_buffer);
            command_buffer_pos = 0;
            command_ready = false;
        }

        // Se o estado do sistema for JOYSTICK
        if (current_state == STATE_JOYSTICK) {
            // Realiza a leitura dos eixos x e y
            adc_select_input(0);
            uint16_t x_reading = adc_read();
            adc_select_input(1);
            uint16_t y_reading = adc_read();

            // Controla o motor 0
            control_motor_from_joystick(&steppers[0], x_reading, joystick_x_center);

            // Caso dois motores estiverem ativos, executar o segundo motor
            if (active_motor_count >= 2) {
                control_motor_from_joystick(&steppers[1], y_reading, joystick_y_center);
            }

            // Envia os dados lidos pelo joystick para a interface (Apenas se eles mudaram)
            static uint16_t last_x_reading = 0;
            static uint16_t last_y_reading = 0;
            static float last_x_interval = 0;
            static float last_y_interval = 0;

            if (abs((int)x_reading - (int)last_x_reading) > 10 ||
                abs((int)y_reading - (int)last_y_reading) > 10 ||
                fabs(steppers[0].actual_step_interval - last_x_interval) > 50.0f ||
                (active_motor_count >= 2 && fabs(steppers[1].actual_step_interval - last_y_interval) > 50.0f)) {

                if (active_motor_count >= 2) {
                    // Envia os dados via serial para o caso de dois motores:
                    // DATA,<x_reading>,<y_reading>,<motor_0_dir>,<motor_1_dir>,<motor_0_speed>,<motor_1_speed>
                    printf("DATA,%u,%u,%d,%d,%.1f,%.1f\n",
                           x_reading, y_reading,
                           steppers[0].dir, steppers[1].dir,
                           steppers[0].actual_step_interval, steppers[1].actual_step_interval);
                } else {
                    // Envia os dados via serial para o caso de um motores:
                    // DATA,<x_reading>,<y_reading>,<motor_0_dir>,<motor_0_speed>
                    printf("DATA,%u,%u,%d,0,%.1f,0.0\n",
                           x_reading, y_reading,
                           steppers[0].dir,
                           steppers[0].actual_step_interval);
                }

                last_x_reading = x_reading;
                last_y_reading = y_reading;
                last_x_interval = steppers[0].actual_step_interval;
                if (active_motor_count >= 2) {
                    last_y_interval = steppers[1].actual_step_interval;
                }
            }
        } else {
            // No modo COMMAND, não envia nada
            printf("DATA,0,0,0,0,0.0,0.0\n");
        }

        // Envia atualizações de estado do sistema
        static int position_update_counter = 0;
        if (++position_update_counter >= 200) {
            send_position_update();
            position_update_counter = 0;
        }

        sleep_ms(25);
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
uint16_t read_joystick_average(int adc_input, int samples, int delay_ms) {
    uint32_t sum = 0;
    for (int i = 0; i < samples; i++) {
        adc_select_input(adc_input);
        uint16_t r = adc_read();
        sum += r;
        sleep_ms(delay_ms);
    }

    return (uint16_t)(sum / (uint32_t)samples);
}

// Realiza o controle do motor especificado com o joystick
void control_motor_from_joystick(stepper_motor_t *motor, uint16_t reading, uint16_t center) {
    // Obtém a distância do joystick para o centro
    int delta = (int)reading - (int)center;
    const uint16_t ADC_MAX = (1 << 12) - 1; // 4095

    if (abs(delta) <= (int)DEADZONE) {
        if (motor->alarm_active) {
            stop_motor(motor);
        }
    } else {
        // Define a direção de rotação do motor
        if (delta > 0) {
            motor->dir = 1;
            gpio_put(motor->dir_pin, 0);
        } else {
            motor->dir = -1;
            gpio_put(motor->dir_pin, 1);
        }

        // Calcula a do joystick do centro e faz a normalização de (0 -> 4095) para (0 -> 1)
        uint16_t max_pos_dev = ADC_MAX - center;
        uint16_t max_neg_dev = center;
        float max_dev = (delta > 0) ? (float)max_pos_dev : (float)max_neg_dev;
        if (max_dev <= 0.0f) max_dev = (float)ADC_MAX / 2.0f;

        float norm = (float)abs(delta) / max_dev;
        if (norm > 1.0f) norm = 1.0f;

        // Mapeia a normalização de (0 -> 1) para o intervalo do pulso
        float min_int = (float)motor->max_speed;
        float max_int = motor->initial_step_interval;
        float desired_interval = max_int - norm * (max_int - min_int);

        if (desired_interval < MAX_INTERVAL_JOYSTICK) desired_interval = MAX_INTERVAL_JOYSTICK;

        motor->actual_step_interval = desired_interval;
        motor->half_period_interval = desired_interval * 0.5f;

        // Inicia a movimentação do motor caso não esteja ativado
        if (!motor->alarm_active) {
            start_motor_continuous(motor);
        }
    }

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
    else if (strncmp(command, "SETZERO", 7) == 0) {
        if (current_state != STATE_COMMAND) {
            printf("ERROR,Nao pode definir origem no modo joystick\n");
            return;
        }
        // Check if there are parameters after SETZERO
        if (strlen(command) > 7 && command[7] == ',') {
            parse_setzero_command(command + 8);
        } else {
            parse_setzero_command(NULL);
        }
    }
    else if (strcmp(command, "STATUS") == 0) {
        // Send status information for all active motors
        printf("STATUS");
        for (uint i = 0; i < active_motor_count && i < 3; i++) {
            stepper_motor_t *motor = &steppers[i];
            printf(",%d,%d,%d,%.1f",
                   (int)motor->step_position,
                   motor->alarm_active ? 1 : 0,
                   motor->dir,
                   motor->actual_step_interval);
        }
        printf("\n");
        send_state_update();
    }
    else {
        printf("ERROR,Comando desconhecido: %s\n", command);
    }
}

// Replace your existing parse_move_command function with this:
void parse_move_command(const char* params) {
    char* param_copy = malloc(strlen(params) + 1);
    strcpy(param_copy, params);

    char* first_param = strtok(param_copy, ",");
    char* second_param = strtok(NULL, ",");

    int motor_id = 0;
    int steps = 0;

    if (second_param) {
        // Two parameters: motor_id, steps
        motor_id = atoi(first_param);
        steps = atoi(second_param);
    } else {
        // One parameter: steps (default to motor 0)
        steps = atoi(first_param);
    }

    if (motor_id < 0 || motor_id >= (int)active_motor_count || motor_id >= 3) {
        printf("ERROR,ID de motor invalido: %d\n", motor_id);
        free(param_copy);
        return;
    }

    stepper_motor_t *motor = &steppers[motor_id];

    // Stop current movement if any
    if (motor->alarm_active) {
        stop_motor(motor);
        sleep_ms(10);
    }

    printf("ACK,Motor %d movendo %d passos\n", motor_id, steps);

    if (steps != 0) {
        move_n_steps(motor, steps);

        // Wait for movement to complete
        while (!motor->movement_done && motor->alarm_active) {
            sleep_ms(1);
        }
        printf("ACK,Motor %d realizou movimento para posicao %d\n",
               motor_id, (int)motor->step_position);
    }

    send_position_update();
    free(param_copy);
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

        printf("ACK,Velocidade atualizada: inicial=%dμs, max=%dμs\n",
               speed, speed / 10);
    } else {
        printf("ERROR,Velocidade invalida: %d (deve estar entre 1-10000μs)\n", speed);
    }
}

// Parse MODE command: MODE,JOYSTICK or MODE,COMMAND
void parse_mode_command(const char* params) {
    if (strcmp(params, "JOYSTICK") == 0) {
        if (current_state != STATE_JOYSTICK) {
            // Stop all motors before switching to joystick mode
            stop_all_motors();
            current_state = STATE_JOYSTICK;
            printf("ACK,Alterado para o modo JOYSTICK\n");
            send_state_update();
        }
    }
    else if (strcmp(params, "COMMAND") == 0) {
        if (current_state != STATE_COMMAND) {
            // Stop all motors before switching to command mode
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

// Parse MOTORS command: MOTORS,<count>
void parse_motors_command(const char* params) {
    int count = atoi(params);

    if (count >= 1 && count <= 3) {
        // Stop all motors before changing count
        stop_all_motors();

        active_motor_count = (uint8_t)count;
        printf("ACK,Numero de motores definido para %d\n", count);
        send_state_update();
    } else {
        printf("ERROR,Numero de motores invalido: %d (deve estar entre 1-3)\n", count);
    }
}

// Replace your existing parse_moveto_command function with this:
void parse_moveto_command(const char* params) {
    char* param_copy = malloc(strlen(params) + 1);
    strcpy(param_copy, params);

    // Parse parameters in pairs: motor_id, position, motor_id, position, ..., wait_flag
    char* tokens[10]; // Max 4 motors * 2 params + wait flag
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

    // Last token is wait flag
    bool wait_for_completion = (atoi(tokens[token_count - 1]) == 1);
    int motor_move_count = (token_count - 1) / 2;

    // Validate motor IDs and prepare movements
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

    // Stop all motors that will be moved
    for (int i = 0; i < valid_moves; i++) {
        stepper_motor_t *motor = &steppers[moves[i].motor_id];
        if (motor->alarm_active) {
            stop_motor(motor);
        }
    }

    sleep_ms(10); // Small delay to ensure all motors stop

    printf("ACK,Movendo %d motores para as posicoes especificadas (esperar=%s)\n",
           valid_moves, wait_for_completion ? "true" : "false");

    // Start all movements simultaneously
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

    // Wait for all movements to complete if requested
    if (wait_for_completion) {
        bool all_done = false;
        while (!all_done) {
            all_done = true;
            for (int i = 0; i < valid_moves; i++) {
                stepper_motor_t *motor = &steppers[moves[i].motor_id];
                if (motor->alarm_active && !motor->movement_done) {
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

    // Send position update for all motors
    send_position_update();

    free(param_copy);
}

// Replace your existing parse_setzero_command function with this:
void parse_setzero_command(const char* params) {
    if (params && strlen(params) > 0) {
        // SETZERO,<motor_id> - Set specific motor to zero
        int motor_id = atoi(params);
        if (motor_id >= 0 && motor_id < (int)active_motor_count && motor_id < 3) {
            steppers[motor_id].step_position = 0;
            printf("ACK,Motor %d teve posicao zerada\n", motor_id);
        } else {
            printf("ERROR,ID de motor invalido: %d\n", motor_id);
            return;
        }
    } else {
        // SETZERO - Set all motors to zero
        for (uint i = 0; i < active_motor_count && i < 3; i++) {
            steppers[i].step_position = 0;
        }
        printf("ACK,Todos os motores tiveram as posicoes zeradas\n");
    }

    send_position_update();
}

// Replace your existing send_position_update function with this:
void send_position_update(void) {
    // Send position data for all active motors
    printf("POSITION");
    for (uint i = 0; i < active_motor_count && i < 3; i++) {
        printf(",%d", (int)steppers[i].step_position);
    }
    printf("\n");
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

