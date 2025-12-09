#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "pico/stdlib.h"
#include "hardware/uart.h"

#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/adc.h"

#define LED_PIN              25

// Parâmetros da comunicação UART
#define UART_ID              uart1
#define UART_TX              4
#define UART_RX              5
#define UART_BAUD_RATE       115200
#define UART_BUFFER_SIZE     256

// GPIOs do joystick
#define JOYSTICK_X_PIN 27 // channel 1
#define JOYSTICK_Y_PIN 28 // channel 2
#define JOYSTICK_X_CHANNEL 1
#define JOYSTICK_Y_CHANNEL 2

// Parâmetros para amostragem do centro do joystick
#define NUM_CAL_SAMPLES      100
#define CAL_SAMPLE_DELAY_MS  5

// GPIOs do fim de curso
#define LIMIT_SWITCH_X_PIN 10
#define LIMIT_SWITCH_Y_PIN 11
#define LIMIT_SWITCH_Z_PIN 12

// Parâmetros de comando
#define MAX_SAVED_POSITIONS  26
#define COMMAND_BUFFER_SIZE  256

// Parâmetros de homin
#define HOMING_SPEED_SLOW    1000  // μs - slow speed for final approach
#define HOMING_SPEED_FAST    950  // μs - fast speed for initial movement
#define HOMING_BACKOFF_STEPS 50 // steps to back off from limit switch

// Define os estados do sistema
typedef enum {
    STATE_JOYSTICK = 0,
    STATE_COMMAND = 1,
    STATE_HOMING = 2
} system_state_t;

// Estrutura que representa o motor de passo
typedef struct {
    char character;
    int32_t x_position;
    int32_t y_position;
    bool is_used;
} saved_position_t;

saved_position_t saved_positions[MAX_SAVED_POSITIONS];

// Estrutura que representa o motor de passo
typedef struct {
    // Pinos de controle
    uint dir_pin;
    uint step_pin;
    // Pino do switch de limite
    uint limit_switch_pin;
    // Variáveis de controle de velocidade
    volatile float initial_step_interval;
    volatile float actual_step_interval;
    volatile float half_period_interval;
    volatile uint32_t total_steps;
    volatile uint32_t step_count;
    volatile uint32_t ramp_up_count;
    volatile int32_t step_position;
    volatile uint32_t acceleration_counter;
    volatile uint32_t max_speed;
    volatile int      dir;
    volatile bool movement_done;
    volatile bool step_state;
    // Controle via joystick
    volatile bool continuous_mode;
    volatile bool alarm_active;
    alarm_id_t alarm_id;
    // Controle de homing (definição de referência para os eixos x e y))
    volatile bool homing_mode;
    volatile bool is_homed;
    volatile bool limit_switch_triggered;
} stepper_motor_t;

// Declara os motores de passo
stepper_motor_t steppers[3] = {
    { .dir_pin = 16, .step_pin = 17, .limit_switch_pin = LIMIT_SWITCH_X_PIN }, // mot_1_x
    { .dir_pin = 0, .step_pin = 1, .limit_switch_pin = LIMIT_SWITCH_Y_PIN }, // mot_2_y
    { .dir_pin = 22, .step_pin = 26, .limit_switch_pin = LIMIT_SWITCH_Z_PIN } // mot_3_z
};

// Define variáveis realacionadas aos comandos enviados da interface
char command_buffer[COMMAND_BUFFER_SIZE];
int command_buffer_pos = 0;
bool command_ready = false;

// Constantes para leitura do joystick
const uint16_t DEADZONE = 200;
const uint16_t MAX_INTERVAL_JOYSTICK = 400; // Máxima velocidade imposta sobre o comando
uint16_t joystick_x_center = 2048;
uint16_t joystick_y_center = 2048;

// Define o estado atual do sistema e a quantidade de motores ativos no momento
system_state_t current_state = STATE_COMMAND;
uint8_t active_motor_count = 3; // define o numero de motores ligados

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
// void process_serial_input(void);
void process_uart_input(void);
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
void parse_testallpos_command(void);
void send_state_update(void);
void send_position_update(void);
void send_homing_status(void);

// Declaração de funções
// Leitura de dados do joystick
uint16_t read_joystick_average(int adc_input, int samples, int delay_ms);
void control_motor_from_joystick(stepper_motor_t *motor, uint16_t reading, uint16_t center);
void handle_joystick_state();

// Declaração de funções
// Salva as posições
void init_saved_positions(void);
void send_saved_positions_update(void);
int find_saved_position_index(char character);
bool saved_positions_is_empty(void);

// Declaração de funções
// Funções de homing
bool home_single_motor(stepper_motor_t *motor, int motor_id);
bool home_all_motors(void);
bool read_limit_switch(stepper_motor_t *motor);
void init_limit_switches(void);

int main (void) {
    stdio_init_all();
    sleep_ms(2000);

    // Inicializa o ADC para o joystick
    adc_init();
    adc_gpio_init(JOYSTICK_X_PIN);
    adc_gpio_init(JOYSTICK_Y_PIN);

    // Calibração inicial do joystick: lê 100 amostras e define o centro
    // Nesse momento, o joystick não deve ser movido    joystick_x_center = read_joystick_average(0, NUM_CAL_SAMPLES, CAL_SAMPLE_DELAY_MS);
    joystick_x_center = read_joystick_average(JOYSTICK_X_CHANNEL, NUM_CAL_SAMPLES, CAL_SAMPLE_DELAY_MS);
    printf("Centro - Joystick X  (media para %d amostras) = %u\n", NUM_CAL_SAMPLES, joystick_x_center);
    joystick_y_center = read_joystick_average(JOYSTICK_Y_CHANNEL, NUM_CAL_SAMPLES, CAL_SAMPLE_DELAY_MS);
    printf("Centro - Joystick Y (media para %d amostras) = %u\n", NUM_CAL_SAMPLES, joystick_y_center);

    // Inicializa o LED do raspberry
    gpio_init(LED_PIN); gpio_set_dir(LED_PIN, GPIO_OUT); gpio_put(LED_PIN, 0);

    // Cria instância do timer para o LED e chama a cada 500 ms
    repeating_timer_t led_timer;
    add_repeating_timer_ms(500, led_callback, NULL, &led_timer);
    // Initialize limit switches
    init_limit_switches();

    // Initialize saved positions array
    init_saved_positions();

    // Inicializa os motores
    for (uint i = 0; i < 3; i++) {
        init_stepper_motor(&steppers[i]);
    }

    // Envia o estado do sistema e a posição atual para a interface (INICIALIZAÇÃO DA INTERFACE)
    // Inicializa a UART para comunicar com a outra placa
    uart_init(UART_ID, UART_BAUD_RATE);
    gpio_set_function(UART_TX, GPIO_FUNC_UART);
    gpio_set_function(UART_RX, GPIO_FUNC_UART);

    // Espera por mensagem de conexão
    // START_CONN\n
    // while (!uart_is_readable(UART_ID))
    // {
    //     tight_loop_contents();
    // }

    // while (uart_is_readable(UART_ID))
    // {
    //     char c = uart_getc(UART_ID);

    //     if (command_buffer_pos < 255) {
    //         command_buffer[command_buffer_pos++] = c;
    //     }

    //     if (c == '\n' || c == '\r') {
    //         command_buffer[command_buffer_pos] = '\0';
    //         command_buffer_pos = 0;
    //     }
    // }

    // if (!(strcmp(command_buffer, "START_CONN") == 0)) {
    //     command_buffer_pos = 0;
    //     command_buffer[0] = '\0';
    //     strncpy(command_buffer, "ERROR, Falha ao conectar\n", 256);
    //     uart_puts(UART_ID, command_buffer);
    // }

    // send_state_update();
    // send_position_update();
    // send_saved_positions_update();
    // send_homing_status();

    
    uint32_t last_time = 0;

    while (true) {
        // Processa os dados vindos via serial (dados enviados pela interface)
        process_uart_input();
        switch (current_state)
        {
            case STATE_COMMAND:
                // Se o comando foi processado, lidar com o comando
                if (command_ready) {
                    // Executa a função relacionada ao comando
                    handle_command(command_buffer);
                    command_buffer_pos = 0;
                    command_ready = false;
                    command_buffer[0] = '\0';
                }
                break;
            case STATE_JOYSTICK:
                handle_joystick_state();
                break;
        }

        uint32_t time_now = to_ms_since_boot(get_absolute_time());
        if (time_now - last_time >= 1000) {
            last_time = time_now;

            send_state_update();
            send_position_update();
            if (current_state == STATE_HOMING) {
                send_homing_status();
            }
        }

        sleep_ms(25);
    }

    return 0;
}

void process_uart_input(void) {
    while (uart_is_readable(UART_ID)) {
        char c = uart_getc(UART_ID);

        if (command_buffer_pos < COMMAND_BUFFER_SIZE - 1) {        
            command_buffer[command_buffer_pos++] = c;
        }

        if (c == '\r' || c == '\n') {
            command_buffer[command_buffer_pos] = '\0';
            command_buffer_pos = 0;
            command_ready = true;
        }
    }

}

void handle_joystick_state() {
    // Realiza a leitura dos eixos x e y
    adc_select_input(0);
    uint16_t x_reading = adc_read();
    adc_select_input(1);
    uint16_t y_reading = adc_read();

    // Controle dos motores x e y
    control_motor_from_joystick(&steppers[0], x_reading, joystick_x_center);
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
            last_x_reading = x_reading;
            last_y_reading = y_reading;
            last_x_interval = steppers[0].actual_step_interval;

            if (active_motor_count >= 2) {
                last_y_interval = steppers[1].actual_step_interval;
            }
    }  
}

// Callback para computar a largura do pulso enviado para o pino STEP do driver do motor
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

            // Caso o motor esteja em modo HOMING
            if (motor->homing_mode) {

                // Verifica o estado da chave de fim de curso
                if (read_limit_switch(motor)) {
                    motor->limit_switch_triggered = true;
                    motor->movement_done = true;
                    motor->alarm_active = false;
                    motor->continuous_mode = false;
                    gpio_put(motor->step_pin, 0);
                    return 0;
                }
            }

            // Atualiza half_period de acordo com actual_step_interval (que é atualizada pelo loop principal)
            motor->half_period_interval = motor->actual_step_interval * 0.5f;

            // Se por alguma razão movement_done foi solicitado, interrompe
            if (motor->movement_done) {
                motor->alarm_active = false;
                motor->continuous_mode = false;
                return 0;
            }

            // Retorna a nova largura de pulso
            return (int64_t)motor->half_period_interval;
        }
    }

    // Caso o movimento tenha acabado, interrompe o agendamento de alarmes
    if (motor->step_count >= motor->total_steps) {
        motor->movement_done = true;
        motor->alarm_active = false;
        return 0;
    }

    // Verifica o estado da chave durante o processo de homing
    // Caso a chave tenha sido pressionada, para o movimento do motor e encerra o alarme
    if (read_limit_switch(motor) && motor->dir < 0) {
        motor->movement_done = true;
        motor->alarm_active = false;
        gpio_put(motor->step_pin, 0);
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

        // Computa a nova largura do pulso enviado pelo pino STEP
        motor->half_period_interval = motor->actual_step_interval * 0.5f;
        return (int64_t)motor->half_period_interval;
    }
}

// Callback para o LED do raspberry ficar piscando
bool led_callback(repeating_timer_t *rt) {
    gpio_put(LED_PIN, !gpio_get(LED_PIN));
    return true;
}

// Inicializa o motor de passo
void init_stepper_motor(stepper_motor_t *motor) {
    gpio_init(motor->step_pin);
    gpio_set_dir(motor->step_pin, GPIO_OUT);
    gpio_put(motor->step_pin, 0);

    gpio_init(motor->dir_pin);
    gpio_set_dir(motor->dir_pin, GPIO_OUT);

    // Valores iniciais para a rampa
    motor->initial_step_interval  = 6200.0f;
    motor->actual_step_interval   = motor->initial_step_interval;
    motor->half_period_interval   = motor->actual_step_interval * 0.5f;
    motor->max_speed              = 500;
    motor->movement_done          = false;
    motor->step_state             = false;
    motor->step_position          = 0;
    motor->total_steps            = 0;
    motor->step_count             = 0;
    motor->ramp_up_count          = 0;
    motor->acceleration_counter   = 0;
    motor->dir                    = 0;
    motor->continuous_mode        = false;
    motor->alarm_active           = false;
    motor->alarm_id               = 0;
    motor->homing_mode            = false;
    motor->is_homed               = false;
    motor->limit_switch_triggered = false;
}

// Inicializa as chaves de fim de curso e ativa os resistores de pull-down
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

// Retorna o estado da chave de fim de curso (pressionado ou não)
bool read_limit_switch(stepper_motor_t *motor) {
    return gpio_get(motor->limit_switch_pin);
}

// Executa o movimento relativo
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

    // Agenda o alarme para o primeiro pulso (largura do pulso enviado ao pino STEP)
    motor->alarm_id = add_alarm_in_us((int64_t)motor->half_period_interval, alarm_irq_handler, motor, false);
    motor->alarm_active = true;
}

// Executa o movimento absoluto (em relação ao ponto zero - referência)
void move_to_position(stepper_motor_t *motor, int32_t target, bool wait) {
    move_n_steps(motor, target - (int32_t)motor->step_position);
    if (wait) {
        while (!motor->movement_done);
    }
}

// Realiza o HOMING de um motor
bool home_single_motor(stepper_motor_t *motor, int motor_id) {
    char rbuffer[256];

    // Caso o motor já se encontre no fim do eixo (x ou y)
    if (read_limit_switch(motor)) {
        snprintf(rbuffer, 256, "ACK,Motor %d ja esta no fim de curso - realizando retorno \n", motor_id);
        uart_puts(UART_ID, rbuffer);

        // Define os paramêtros de velocidade (velocidade inicial, velocidade final e largura de pulso)
        motor->initial_step_interval = HOMING_SPEED_SLOW;
        motor->max_speed = HOMING_SPEED_SLOW;
        motor->actual_step_interval = HOMING_SPEED_SLOW;

        // Define a direção de giro do motor)
        motor->dir = 1;
        gpio_put(motor->dir_pin, 0);

        // Executa o movimento para se afastar da chave de fim de curso
        // E aguarda o término do movimento
        move_n_steps(motor, HOMING_BACKOFF_STEPS);
        while (!motor->movement_done && motor->alarm_active) {
            sleep_ms(1);
        }

        sleep_ms(100);
    }

    memset(rbuffer, 0, sizeof(rbuffer));
    // Parte 1 -> Aproxima até a chave de fim de curso ativar
    snprintf(rbuffer, 256, "ACK,Motor %d: Fase 1 - aproximacao rapida\n", motor_id);
    uart_puts(UART_ID, rbuffer);

    motor->homing_mode = true;
    motor->limit_switch_triggered = false;
    motor->initial_step_interval = HOMING_SPEED_FAST;
    motor->max_speed = HOMING_SPEED_FAST;
    motor->actual_step_interval = HOMING_SPEED_FAST;

    motor->dir = -1;
    gpio_put(motor->dir_pin, 1);

    start_motor_continuous(motor);

    memset(rbuffer, 0, sizeof(rbuffer));
    // Espera o motor chegar na chave (máximo de 50 segundos)
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    while (!motor->limit_switch_triggered && motor->alarm_active) {
        if (to_ms_since_boot(get_absolute_time()) - start_time > 50000) {
            stop_motor(motor);
            snprintf(rbuffer, 256, "ERROR,Timeout no homing do motor %d - limit switch nao encontrado\n", motor_id);
            uart_puts(UART_ID, rbuffer);

            return false;
        }
        sleep_ms(1);
    }

    memset(rbuffer, 0, sizeof(rbuffer));
    if (!motor->limit_switch_triggered) {
        snprintf(rbuffer, 256, "ERROR,Motor %d nao atingiu o limit switch\n", motor_id);
        uart_puts(UART_ID, rbuffer);
        return false;
    }

    snprintf(rbuffer, 256, "ACK,Motor %d: Limit switch atingido\n", motor_id);
    uart_puts(UART_ID, rbuffer);
    sleep_ms(100);

    memset(rbuffer, 0, sizeof(rbuffer));
    // Parte 2 -> Recua da chave
    snprintf(rbuffer, 256, "ACK,Motor %d: Fase 2 - recuo do limit switch\n", motor_id);
    uart_puts(UART_ID, rbuffer);

    motor->homing_mode = false;
    motor->limit_switch_triggered = false;
    motor->initial_step_interval = HOMING_SPEED_SLOW;
    motor->max_speed = HOMING_SPEED_SLOW;
    motor->actual_step_interval = HOMING_SPEED_SLOW;

    motor->dir = 1;
    gpio_put(motor->dir_pin, 0);

    move_n_steps(motor, HOMING_BACKOFF_STEPS);
    while (!motor->movement_done && motor->alarm_active) {
        sleep_ms(1);
    }

    sleep_ms(100);

    // Parte -> Aproxima da chave de forma lenta
    // printf("ACK,Motor %d: Fase 3 - aproximacao final lenta\n", motor_id);

    // motor->homing_mode = true;
    // motor->limit_switch_triggered = false;
    // motor->initial_step_interval = HOMING_SPEED_SLOW;
    // motor->max_speed = HOMING_SPEED_SLOW;
    // motor->actual_step_interval = HOMING_SPEED_SLOW;

    // motor->dir = -1;
    // gpio_put(motor->dir_pin, 1);

    // start_motor_continuous(motor);

    // start_time = to_ms_since_boot(get_absolute_time());
    // while (!motor->limit_switch_triggered && motor->alarm_active) {
    //     if (to_ms_since_boot(get_absolute_time()) - start_time > 10000) {
    //         stop_motor(motor);
    //         printf("ERROR,Timeout na aproximacao final do motor %d\n", motor_id);
    //         return false;
    //     }
    //     sleep_ms(1);
    // }

    // Define a posição atual como zero
    motor->step_position = 0;
    motor->is_homed = true;
    motor->homing_mode = false;

    // Reseta os parâmetros para os valores normais
    motor->initial_step_interval = 6200.0f;
    motor->max_speed = 700;
    motor->actual_step_interval = motor->initial_step_interval;

    memset(rbuffer, 0, sizeof(rbuffer));
    snprintf(rbuffer, 256, "ACK,Motor %d: Posicao home definida\n", motor_id);
    uart_puts(UART_ID, rbuffer);

    send_position_update();

    return true;
}

// Realiza o HOMING de todos os motores
bool home_all_motors(void) {
    bool all_success = true;
    char rbuffer[256];

    // Executa o processo de homing (definição do ponto zero) para cada motor
    for (uint i = 0; i < active_motor_count && i < 3; i++) {
        memset(rbuffer, 0, sizeof(rbuffer));
        snprintf(rbuffer, 256, "ACK,Iniciando homing do motor %d\n", i);
        uart_puts(UART_ID, rbuffer);

        if (!home_single_motor(&steppers[i], i)) {
            all_success = false;

            memset(rbuffer, 0, sizeof(rbuffer));
            snprintf(rbuffer, 256, "ERROR,Falha no homing do motor %d\n", i);
            uart_puts(UART_ID, rbuffer);
        }

        sleep_ms(500);
    }

    return all_success;
}

// Envia o status do processo de HOMING dos motores
void send_homing_status(void) {
    char rbuffer[256];
    int pos = 0;

    memset(rbuffer, 0, sizeof(rbuffer));
    pos += snprintf(rbuffer + pos, sizeof(rbuffer) - pos, "HOMING_STATUS");

    for (uint i = 0; i < active_motor_count && i < 3; i++) {
        stepper_motor_t *motor = &steppers[i];
        bool limit_state = read_limit_switch(motor);

        pos += snprintf(rbuffer + pos, sizeof(rbuffer) - pos, ",%d,%d",
                        motor->is_homed ? 1 : 0,
                        limit_state ? 1 : 0);

        if (pos >= sizeof(rbuffer)) {
            pos = sizeof(rbuffer) - 1;
            break;
        }
    }

    snprintf(rbuffer + pos, sizeof(rbuffer) - pos, "\n");
    printf("MSG 2: %s\n\n", rbuffer);
    uart_puts(UART_ID, rbuffer);
}

// Inicia o motor em modo contínuo (usado para joystick)
void start_motor_continuous(stepper_motor_t *motor) {
    if (motor->alarm_active) return; // se já está rodando, só retorna

    // se não estiver rodando define os parametros
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

// Lê N amostras do ADC e retorna a média para cada eixo
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
    const uint16_t ADC_MAX = 4095;

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

// Direciona para as funções de acordo com o comando enviado via serial
void handle_command(const char* command) {
    char rbuffer[256];

    printf("comando: %s\n", command);

    if (strncmp(command, "MOVE,", 5) == 0) {
        if (current_state != STATE_COMMAND) {
            sprintf(rbuffer, "ERROR,Nao pode mover no modo de teste\n");
            uart_puts(UART_ID, rbuffer);
            return;
        }
        parse_move_command(command + 5);
    }
    else if (strncmp(command, "SPEED,", 6) == 0) {
        if (current_state != STATE_COMMAND) {
            sprintf(rbuffer, "ERROR,Nao pode definir velocidade no modo de teste\n");
            uart_puts(UART_ID, rbuffer);
            return;
        }
        parse_speed_command(command + 6);
    }
    else if (strcmp(command, "STOP") == 0) {
        stop_all_motors();
        sprintf(rbuffer, "ACK,Parada de emergencia executada\n");
        uart_puts(UART_ID, rbuffer);
    }
    else if (strncmp(command, "MODE,", 5) == 0) {
        parse_mode_command(command + 5);
    }
    else if (strncmp(command, "MOTORS,", 7) == 0) {
        parse_motors_command(command + 7);
    }
    else if (strncmp(command, "MOVETO,", 7) == 0) {
        if (current_state != STATE_COMMAND) {
            sprintf(rbuffer, "ERROR,Nao pode posicionar no modo de teste\n");
            uart_puts(UART_ID, rbuffer);
            return;
        }
        parse_moveto_command(command + 7);
    }
    else if (strncmp(command, "HOME", 4) == 0) {
        if (current_state == STATE_JOYSTICK) {
            sprintf(rbuffer, "ERROR,Nao pode executar homing no modo de teste\n");
            uart_puts(UART_ID, rbuffer);
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
            sprintf(rbuffer, "ERROR,Nao pode recuperar posicao no modo de teste\n");
            uart_puts(UART_ID, rbuffer);
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
            sprintf(rbuffer, "ERROR,nenhuma posicao foi encontrada\n");
            uart_puts(UART_ID, rbuffer);
        } else {
            parse_testallpos_command();
        }
    }
    else if (strncmp(command, "RECALLSTRING,", 13) == 0) {
        if (current_state != STATE_COMMAND) {
            sprintf(rbuffer, "ERROR,Nao pode executar RECALLSTRING no modo teste\n");
            uart_puts(UART_ID, rbuffer);
            return;
        }
        parse_recallstring_command(command + 13);
    }
    else if (strcmp(command, "GET_HOMING") == 0) {
        if (current_state != STATE_COMMAND) {
            sprintf(rbuffer, "ERROR,Nao pode executar RECALLSTRING no modo teste\n");
            uart_puts(UART_ID, rbuffer);
            return;
        }
        send_homing_status();
    }
    else if (strcmp(command, "GET_STATE") == 0) {
        if (current_state != STATE_COMMAND) {
            sprintf(rbuffer, "ERROR,Nao pode executar RECALLSTRING no modo teste\n");
            uart_puts(UART_ID, rbuffer);
            return;
        }
        send_state_update();
    }
    else if (strcmp(command, "GET_POSITIONS") == 0) {
        if (current_state != STATE_COMMAND) {
            sprintf(rbuffer, "ERROR,Nao pode executar RECALLSTRING no modo teste\n");
            uart_puts(UART_ID, rbuffer);
            return;
        }
        send_position_update();
    }
    else if (strcmp(command, "SETZERO") == 0) {
        if (current_state != STATE_COMMAND) {
            sprintf(rbuffer, "ERROR,Nao pode definir origem no modo teste\n");
            uart_puts(UART_ID, rbuffer);
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
        stepper_motor_t *motor0 = &steppers[0];
        stepper_motor_t *motor1 = &steppers[1];
        stepper_motor_t *motor2 = &steppers[2];

        snprintf(rbuffer, 256,
            "STATUS,%d,%d,%d,%.1f,%d,%d,%d,%.1f,%d,%d,%d,%.1f\n",
            (int)motor0->step_position,
            motor0->alarm_active ? 1 : 0,
            motor0->dir,
            motor0->actual_step_interval,
            (int)motor1->step_position,
            motor1->alarm_active ? 1 : 0,
            motor1->dir,
            motor1->actual_step_interval,
            (int)motor2->step_position,
            motor2->alarm_active ? 1 : 0,
            motor2->dir,
            motor2->actual_step_interval
        );
        uart_puts(UART_ID, rbuffer);

        send_state_update();
    } else {
        snprintf(rbuffer, 256, "ERROR,Comando desconhecido: %s\n", command);
        uart_puts(UART_ID, rbuffer);
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
    if (motor->alarm_active) {
        stop_motor(motor);
        sleep_ms(10);
    }

    printf("ACK,Motor %d movendo %d passos\n", motor_id, steps);

    if (steps != 0) {
        move_n_steps(motor, steps);

        while (!motor->movement_done && motor->alarm_active) {
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
    int speed = atoi(params);

    if (speed > 0 && speed <= 10000) {
        // Atualiza a velocidade de todos os motores ativos
        for (uint i = 0; i < active_motor_count; i++) {
            steppers[i].initial_step_interval = (float)speed;
            steppers[i].max_speed = speed / 10;
            if (steppers[i].max_speed < 100) steppers[i].max_speed = 100;
        }

        printf("ACK,Velocidade atualizada: inicial=%dμs, max=%dμs\n",
               speed, speed / 10);
    } else {
        printf("ERROR,Velocidade invalida: %d (deve estar entre 1-10000μs)\n", speed);
    }
}

// Define o modo do sistema: MODE,JOYSTICK ou MODE,COMMAND
void parse_mode_command(const char* params) {
    char rbuffer[256];

    if (strcmp(params, "JOYSTICK") == 0) {
        if (current_state != STATE_JOYSTICK) {
            stop_all_motors();
            current_state = STATE_JOYSTICK;
            printf("ACK,Alterado para o modo JOYSTICK\n");

            snprintf(rbuffer, 256, "ACK,Alterado para o modo JOYSTICK\n");
            uart_puts(UART_ID, rbuffer);
            send_state_update();
        }
    }
    else if (strcmp(params, "COMMAND") == 0) {
        if (current_state != STATE_COMMAND) {
            stop_all_motors();
            current_state = STATE_COMMAND;
            printf("ACK,Alterado para o modo de COMANDO\n");

            snprintf(rbuffer, 256, "ACK,Alterado para o modo de COMANDO\n");
            uart_puts(UART_ID, rbuffer);
            send_state_update();
        }
    }
    else {
        printf("ERROR,Modo invalido: %s (use JOYSTICK ou COMMAND)\n", params);

        snprintf(rbuffer, 256, "ERROR,Modo invalido: %s (use JOYSTICK ou COMMAND)\n", params);
        uart_puts(UART_ID, rbuffer);
    }
}

// Define a quantidade de motores ativos: MOTORS,<count>
void parse_motors_command(const char* params) {
    int count = atoi(params);
    char rbuffer[256];

    if (count >= 1 && count <= 3) {
        stop_all_motors();

        active_motor_count = (uint8_t)count;
        snprintf(rbuffer, 256, "ACK,Numero de motores definido para %d\n", count);
        uart_puts(UART_ID, rbuffer);
        send_state_update();
    } else {
        snprintf(rbuffer, 256, "ERROR,Numero de motores invalido: %d (deve estar entre 1-3)\n", count);
        uart_puts(UART_ID, rbuffer);
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
        if (motor->alarm_active) {
            stop_motor(motor);
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
    char rbuffer[256];
    uint pos = 0;

    pos += snprintf(rbuffer + pos, sizeof(rbuffer) - pos, "POSITION");
    for (uint i = 0; i < 2; i++) {
        pos += snprintf(rbuffer + pos, sizeof(rbuffer) - pos, ",%d", (int)steppers[i].step_position);

        if (pos >= sizeof(rbuffer)) break;
    }

    snprintf(rbuffer + pos, sizeof(rbuffer) - pos, "\n");

    printf("MSG: %s\n", rbuffer);
    uart_puts(UART_ID, rbuffer);
}

// Envia o estado atual do sistema via serial (JOYSTICK, COMMAND, HOMING)
void send_state_update(void) {
    const char* state_str;
    char rbuffer[256];

    if (current_state == STATE_JOYSTICK) {
        state_str = "JOYSTICK";
    } else if (current_state == STATE_COMMAND) {
        state_str = "COMMAND";
    } else {
        state_str = "HOMING";
    }

    snprintf(rbuffer, 256, "STATE,%s,%d\n", state_str, active_motor_count);
    printf("MSG: %s\n", rbuffer);
    uart_puts(UART_ID, rbuffer);
}

// Para o movimento de todos os motores
void stop_all_motors(void) {
    for (uint i = 0; i < active_motor_count && i < 3; i++) {
        if (steppers[i].alarm_active) {
            stop_motor(&steppers[i]);
        }
    }
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

// Salva a posição atual do atuador e associa a uma letra: SAVEPOS,<character>,<x_pos>,<y_pos>
void parse_savepos_command(const char* params) {
    char rbuffer[256];
    char* param_copy = malloc(strlen(params) + 1);
    strcpy(param_copy, params);

    char* char_param = strtok(param_copy, ",");
    char* x_param = strtok(NULL, ",");
    char* y_param = strtok(NULL, ",");

    if (!char_param || !x_param || !y_param) {
        sprintf(rbuffer, "ERROR,Parametros insuficientes para SAVEPOS. Formato: character,x_pos,y_pos\n");
        uart_puts(UART_ID, rbuffer);
        free(param_copy);
        return;
    }

    char character = char_param[0];
    int32_t x_pos = atoi(x_param);
    int32_t y_pos = atoi(y_param);

    // Faz a validação do caractere
    if (!((character >= 'a' && character <= 'z') ||
          (character >= '0' && character <= '9'))) {
        memset(rbuffer, 0, sizeof(rbuffer));
        snprintf(rbuffer, 256, "ERROR,Caractere invalido: %c (use a-z ou 0-9)\n", character);
        uart_puts(UART_ID, rbuffer);
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
        memset(rbuffer, 0, sizeof(rbuffer));
        snprintf(rbuffer, 256, "ERROR,Memoria de posicoes cheia (maximo %d posicoes)\n", MAX_SAVED_POSITIONS);
        uart_puts(UART_ID, rbuffer);
        free(param_copy);
        return;
    }

    // Salava as coordenadas no array
    saved_positions[index].character = character;
    saved_positions[index].x_position = x_pos;
    saved_positions[index].y_position = y_pos;
    saved_positions[index].is_used = true;

    memset(rbuffer, 0, sizeof(rbuffer));
    snprintf(rbuffer, 256, "ACK,Posicao '%c' salva: X=%d, Y=%d\n", character, x_pos, y_pos);
    uart_puts(UART_ID, rbuffer);
    send_saved_positions_update();

    free(param_copy);
}

// Recupera a posição da letra especificada: RECALLPOS,<character>
void parse_recallpos_command(const char* params) {
    char rbuffer[256];

    if (strlen(params) != 1) {
        sprintf(rbuffer, "ERROR,RECALLPOS requer exatamente um caractere\n");
        uart_puts(UART_ID, rbuffer);
        return;
    }

    // Obtém o caractere especificado no comando
    // e recupera as posições x e y associados ao mesmo
    char character = params[0];
    int index = find_saved_position_index(character);

    if (index == -1) {
        memset(rbuffer, 0, sizeof(rbuffer));
        snprintf(rbuffer, 256, "ERROR,Posicao '%c' nao encontrada\n", character);
        uart_puts(UART_ID, rbuffer);
        return;
    }

    // Define a posição x e y alvo para os motores
    int32_t target_x = saved_positions[index].x_position;
    int32_t target_y = saved_positions[index].y_position;

    memset(rbuffer, 0, sizeof(rbuffer));
    snprintf(rbuffer, 256, "ACK,Movendo para posicao salva '%c': X=%d, Y=%d\n",
           character, target_x, target_y);
    uart_puts(UART_ID, rbuffer);

    // Para todos os motores, caso estejam se movimentando
    stop_all_motors();
    sleep_ms(10);

    // Move motor 0 (eixo x)
    // Realiza a diferença entre a posição atual do motor e a desejada
    stepper_motor_t *motor0 = &steppers[0];
    int32_t steps_x = target_x - (int32_t)motor0->step_position;
    if (steps_x != 0) {
        // motor0->initial_step_interval  = 5000.0f;
        // motor0->max_speed              = 2000;
        move_n_steps(motor0, steps_x);
    }

    // Move motor 1 (eixo y)
    if (active_motor_count >= 2) {
        stepper_motor_t *motor1 = &steppers[1];
        int32_t steps_y = target_y - (int32_t)motor1->step_position;
        if (steps_y != 0) {
            // motor1->initial_step_interval  = 5000.0f;
            // motor1->max_speed              = 2000;
            move_n_steps(motor1, steps_y);
        }
    }

    // Espera pelo termino dos movimentos
    bool all_done = false;
    while (!all_done) {
        all_done = true;
        if (motor0->alarm_active && !motor0->movement_done) {
            all_done = false;
        }
        if (active_motor_count >= 2) {
            stepper_motor_t *motor1 = &steppers[1];
            if (motor1->alarm_active && !motor1->movement_done) {
                all_done = false;
            }
        }
        if (!all_done) {
            sleep_ms(1);
        }
    }

    // Espera a tecla ser pressionada
    // char caracter = uart_getc(uart1);

    // while (caracter != character) {
    //     caracter = uart_getc(uart1);
    // }

    // Envia a atualização via serial sobre a posição dos motores

    memset(rbuffer, 0, sizeof(rbuffer));
    sprintf(rbuffer, "ACK,Finalizaou movimento\n");
    uart_puts(UART_ID, rbuffer);

    send_position_update();
}

// Recupera os caracteres que compõem a string: RECALLSTRING,<texto>
void parse_recallstring_command(const char* params) {
    char rbuffer[256];

    // Verifica se uma string foi passada, se não, retorna mensagem de erro
    if (!params || strlen(params) == 0) {
        memset(rbuffer, 0, sizeof(rbuffer));
        sprintf(rbuffer, "ERROR,RECALLSTRING requer uma string nao vazia\n");
        uart_puts(UART_ID, rbuffer);
        return;
    }

    // Obtém o tamanho da string
    size_t len = strlen(params);

    for (size_t i = 0; i < len; i++) {
        char character = params[i];
        int index = find_saved_position_index(character);

        // Se o caractere não for encontrado no array, então pula a iteração
        if (index == -1) {
            memset(rbuffer, 0, sizeof(rbuffer));
            // sprintf(rbuffer, "ERROR,Posicao '%c' nao encontrada (pulando)\n", character);
            uart_puts(UART_ID, rbuffer);
            continue;
        }

        // Caso o caractere tenha sido encontrado, recupera as posições x e y
        int32_t target_x = saved_positions[index].x_position;
        int32_t target_y = saved_positions[index].y_position;

        // Envia mensagem indicando: movimento iniciado
        memset(rbuffer, 0, sizeof(rbuffer));
        sprintf(rbuffer, "ACK,Palavra: '%s' --- Movendo para '%c': X=%d, Y=%d\n",
               params, character, target_x, target_y);
        uart_puts(UART_ID, rbuffer);

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
            if (motor0->alarm_active && !motor0->movement_done) {
                all_done = false;
            }
            if (active_motor_count >= 2) {
                stepper_motor_t *motor1 = &steppers[1];
                if (motor1->alarm_active && !motor1->movement_done) {
                    all_done = false;
                }
            }
            if (!all_done) {
                sleep_ms(1);
            }
        }

        // Atualiza interface
        send_position_update();

        // Delay opcional entre letras
        sleep_ms(300);
    }

    memset(rbuffer, 0, sizeof(rbuffer));
    sprintf(rbuffer, "ACK,Palavra '%s' foi percorrida com sucesso\n", params);
    uart_puts(UART_ID, rbuffer);
}

// Envia as posições que estão salvas no array para a GUI
void send_saved_positions_update(void) {
    char rbuffer[256];
    int pos = 0;

    pos += snprintf(rbuffer + pos, sizeof(rbuffer) - pos, "SAVED_POSITIONS");

    for (int i = 0; i < MAX_SAVED_POSITIONS; i++) {
        if (saved_positions[i].is_used) {
            pos += snprintf(rbuffer + pos, sizeof(rbuffer) - pos, ",%c,%d,%d",
                            saved_positions[i].character,
                            saved_positions[i].x_position,
                            saved_positions[i].y_position);

            if (pos >= sizeof(rbuffer)) {
                pos = sizeof(rbuffer) - 1;
                break;
            }
        }
    }

    snprintf(rbuffer + pos, sizeof(rbuffer) - pos, "\n");

    uart_puts(UART_ID, rbuffer);
}

// Limpa o array de posições: CLEARPOS
void parse_clearpos_command(void) {
    init_saved_positions();
    printf("ACK,Todas as posicoes salvas foram apagadas\n");
    send_saved_positions_update();
}

bool saved_positions_is_empty(void) {
    for (int i = 0; i < MAX_SAVED_POSITIONS; i++) {
        if (saved_positions[i].is_used) {
            return false; // Encontrou ao menos um em uso → não está vazio
        }
    }
    return true; // Nenhum em uso → está vazio
}

void parse_testallpos_command(void) {
    char rbuffer[256];

    for (int i = 0; i < MAX_SAVED_POSITIONS; i++) {
        if (saved_positions[i].is_used) {
            memset(rbuffer, 0, sizeof(rbuffer));
            sprintf(rbuffer, "ACK,movendo o atuador para a letra %c (%d,%d)\n",
                   saved_positions[i].character,
                   saved_positions[i].x_position,
                   saved_positions[i].y_position);
            uart_puts(UART_ID, rbuffer);

            // Define a posição x e y alvo para os motores
            int32_t target_x = saved_positions[i].x_position;
            int32_t target_y = saved_positions[i].y_position;

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
                if (motor0->alarm_active && !motor0->movement_done) {
                    all_done = false;
                }
                if (active_motor_count >= 2) {
                    stepper_motor_t *motor1 = &steppers[1];
                    if (motor1->alarm_active && !motor1->movement_done) {
                        all_done = false;
                    }
                }
                if (!all_done) {
                    sleep_ms(1);
                }
            }

            send_position_update();
        }

        sleep_ms(500);
    }

    memset(rbuffer, 0, sizeof(rbuffer));
    sprintf(rbuffer, "ACK,deslocamento por todos os pontos foi executado\n");
    uart_puts(UART_ID, rbuffer);
}

// Realiza o homing para os motores: HOME,<motor_id> ou somente HOME para todos os motores
void parse_home_command(const char* params) {
    char rbuffer[256];
    stop_all_motors();

    current_state = STATE_HOMING;
    send_state_update();

    if (params && strlen(params) > 0) {
        // HOME,<motor_id> - um motor em específico
        int motor_id = atoi(params);
        if (motor_id >= 0 && motor_id < (int)active_motor_count && motor_id < 3) {
            memset(rbuffer, 0, sizeof(rbuffer));
            sprintf(rbuffer, "ACK,Iniciando homing do motor %d\n", motor_id);
            uart_puts(UART_ID, rbuffer);
            uart_puts(UART_ID, rbuffer);

            if (home_single_motor(&steppers[motor_id], motor_id)) {
                memset(rbuffer, 0, sizeof(rbuffer));
                sprintf(rbuffer, "ACK,Homing do motor %d concluido\n", motor_id);
                uart_puts(UART_ID, rbuffer);
            } else {
                memset(rbuffer, 0, sizeof(rbuffer));
                sprintf(rbuffer, "ERROR,Falha no homing do motor %d\n", motor_id);
                uart_puts(UART_ID, rbuffer);
            }
        } else {
            memset(rbuffer, 0, sizeof(rbuffer));
            sprintf(rbuffer, "ERROR,ID de motor invalido para homing: %d\n", motor_id);
            uart_puts(UART_ID, rbuffer);
        }
    } else {
        // HOME - todos os motores
        memset(rbuffer, 0, sizeof(rbuffer));
        sprintf(rbuffer, "ACK,Iniciando homing de todos os motores\n");
        uart_puts(UART_ID, rbuffer);

        if (home_all_motors()) {
            memset(rbuffer, 0, sizeof(rbuffer));
            sprintf(rbuffer, "ACK,Homing de todos os motores concluido\n");
            uart_puts(UART_ID, rbuffer);
        } else {
            memset(rbuffer, 0, sizeof(rbuffer));
            sprintf(rbuffer, "ERROR,Falha no homing de alguns motores\n");
            uart_puts(UART_ID, rbuffer);
        }
    }

    current_state = STATE_COMMAND;
    send_state_update();
    send_homing_status();
}
