/**
 * @file multiple_motors_control.c
 * @brief Demo de controle de motores de passo com alarme/timer do RP2040.
 *
 * Contém definição da estrutura do motor, inicialização, agendamento de
 * pulsos via alarm IRQ e uma API simples (bloqueante e não-bloqueante).
 *
 * ADICIONADO: Sistema de movimento sincronizado usando algoritmo Bresenham.
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "pico/stdlib.h"

#include "hardware/irq.h"
#include "hardware/timer.h"


/**
 * @def LED_PIN
 * @brief GPIO usado para o LED de status.
 *
 * Usado pelo timer repetitivo para piscar um LED enquanto a demo roda.
 */
#define LED_PIN   25

#define BTN_CONTROL 7 // Inicia/Para movimento
volatile absolute_time_t last_btn_press = 0;
volatile bool mov_state = false;

/**
 * @struct stepper_motor_t
 * @brief Estrutura que representa o estado e configuração de um motor de passo.
 */
typedef struct {
    /* --- Pinos de controle --- */

    /** @brief Pino DIR (direção). */
    uint pin_dir;

    /** @brief Pino STEP (pulsos). */
    uint pin_step;

    /* --- Pinos de microstepping (MS1/MS2/MS3) --- */

    /** @brief Pino MS1 (microstepping). */
    uint pin_ms1;

    /** @brief Pino MS2 (microstepping). */
    uint pin_ms2;

    /** @brief Pino MS3 (microstepping). */
    uint pin_ms3;

    /* --- Variáveis de tempo (unidade: microsegundos) --- */

    /**
     * @brief Intervalo inicial entre passos (us) usado ao iniciar movimento.
     *
     * Valor base para iniciar o perfil de rampa.
     */
    volatile float initial_step_interval_us;

    /**
     * @brief Intervalo atual entre passos (us).
     *
     * É atualizado durante as fases de aceleração/desaceleração.
     */
    volatile float current_step_interval_us;

    /**
     * @brief Meio-período do pulso atual (us).
     *
     * Retornado pelo alarm handler para agendar o próximo evento (alto/baixo).
     */
    volatile float half_step_interval_us;

    /* --- Contadores e posição --- */

    /** @brief Total de passos que serão executados na operação atual (absoluto). */
    volatile uint32_t steps_total;

    /** @brief Quantidade de passos já executados na operação atual. */
    volatile uint32_t steps_executed;

    /**
     * @brief Número de passos que definem a rampa de aceleração.
     *
     * Determinado dinamicamente quando a velocidade máxima (menor intervalo) é alcançada.
     */
    volatile uint32_t ramp_steps;

    /**
     * @brief Posição atual em passos.
     *
     * NOTA: este campo é `uint32_t` no código. Se o sistema precisar representar
     * posições negativas (movimento para trás), considere alterá-lo para
     * `int32_t` para evitar underflow ao somar `direction` negativo.
     */
    volatile uint32_t position_steps;

    /** @brief Contador auxiliar usado no cálculo da aceleração. */
    volatile uint32_t accel_counter;

    /**
     * @brief Menor intervalo entre passos (us) permitido — representa a velocidade máxima.
     *
     * É um limite: o algoritmo de rampa não reduzirá `current_step_interval_us` abaixo deste.
     */
    volatile uint32_t min_step_interval_us;

    /* --- Direção e flags --- */

    /**
     * @brief Sinal de direção: +1 para sentido horário (CW), -1 para anti-horário (CCW).
     *
     * Tipo `int8_t` é suficiente (valores -1 ou +1).
     */
    volatile int8_t direction;

    /** @brief Flag indicando que o movimento atual foi concluído. */
    volatile bool movement_complete;

    /**
     * @brief Estado atual do pino STEP (true = nível lógico alto).
     *
     * Usado no IRQ para alternar entre subir/baixar o pino.
     */
    volatile bool step_pin_state;

    /**
     * @brief ID do alarme/timer retornado por add_alarm_in_us().
     *
     * Armazenado caso seja necessário cancelar ou verificar o alarme.
     */
    alarm_id_t timer_alarm_id;
} stepper_motor_t;

/**
 * @brief Array de instâncias de motores usados na demo.
 *
 * A alocação é estática para fácil exemplo; cada elemento deve ser inicializado
 * via init_stepper_motor() antes do uso.
 */
stepper_motor_t steppers[3] = {
    /* Motor 1 */
    { .pin_dir = 0, .pin_step = 1, .pin_ms1 = 13, .pin_ms2 = 14, .pin_ms3 = 15 },
    /* Motor 2 */
    { .pin_dir = 17, .pin_step = 16, .pin_ms1 = 13, .pin_ms2 = 14, .pin_ms3 = 15 }
};

// ========================================================================
// ADICIONADO: SISTEMA DE MOVIMENTO SINCRONIZADO
// ========================================================================

/**
 * @brief Maximum number of coordinated axes
 */
#define MAX_COORDINATED_AXES 3

/**
 * @brief Structure to hold coordinated movement state
 */
typedef struct {
    /** @brief Pointers to the coordinated motors (X, Y, Z) */
    stepper_motor_t *motors[MAX_COORDINATED_AXES];

    /** @brief Current position for each axis */
    int32_t current_position[MAX_COORDINATED_AXES];

    /** @brief Target position for each axis */
    int32_t target_position[MAX_COORDINATED_AXES];

    /** @brief Distance to travel for each axis (absolute) */
    int32_t distance[MAX_COORDINATED_AXES];

    /** @brief Direction for each axis (+1 or -1) */
    int8_t direction[MAX_COORDINATED_AXES];

    /** @brief Bresenham error terms for each axis */
    int32_t error[MAX_COORDINATED_AXES];

    /** @brief Steps executed for each axis */
    int32_t steps_executed[MAX_COORDINATED_AXES];

    /** @brief Total steps for the dominant axis (longest distance) */
    int32_t total_steps;

    /** @brief Index of the dominant axis (axis with most steps) */
    uint8_t dominant_axis;

    /** @brief Number of active axes in this movement */
    uint8_t active_axes;

    /** @brief Movement state flags */
    volatile bool movement_active;
    volatile bool movement_complete;

    /** @brief Timer for coordinated movement */
    alarm_id_t coord_timer_id;

    /** @brief Movement parameters */
    float initial_step_interval_us;
    float current_step_interval_us;
    float min_step_interval_us;

    /** @brief Acceleration parameters */
    uint32_t accel_counter;
    uint32_t ramp_steps;

} coordinated_stepper_t;

/**
 * @brief Global coordinated stepper instance
 */
static coordinated_stepper_t coord_stepper = {0};

/**
 * @brief Movement command queue structure
 */
typedef struct {
    int32_t target[MAX_COORDINATED_AXES];
    bool valid;
} movement_command_t;

#define COMMAND_QUEUE_SIZE 16
static movement_command_t command_queue[COMMAND_QUEUE_SIZE];
static volatile uint8_t queue_head = 0;
static volatile uint8_t queue_tail = 0;
static volatile uint8_t queue_count = 0;

/* -------------------------
   Declaração das funções ORIGINAIS
   ------------------------- */

/**
 * @brief Handler de IRQ/Alarm chamado pelo hardware timer para gerar pulsos.
 *
 * Função executada no contexto de IRQ. Ela alterna o pino STEP (alto/baixo),
 * atualiza contadores e recalcula o próximo atraso (meio-período).
 *
 * @param id Identificador do alarme (alarm_id_t) recebido pelo callback.
 * @param user_data Ponteiro para a instância do motor (stepper_motor_t*).
 * @return Próximo atraso em microsegundos (int64_t) a ser usado pelo agendador.
 *         Retorna 0 para não re-agendar (movimento concluído).
 */
int64_t alarm_irq_handler(alarm_id_t id, void *user_data);

/**
 * @brief Inicializa os pinos e valores padrão da estrutura do motor.
 *
 * Configura os GPIOs (STEP, DIR, MS1/MS2/MS3), define a resolução de 1/16
 * por padrão e inicializa valores de rampa/velocidade.
 *
 * @param motor Ponteiro para a instância do motor a ser inicializada.
 */
void init_stepper_motor(stepper_motor_t *motor);

/* --- API não-bloqueante (agendam movimento e retornam imediatamente) --- */

/**
 * @brief Agenda um movimento relativo em número de passos (não-bloqueante).
 *
 * @param motor Ponteiro para o motor alvo.
 * @param steps Quantidade de passos a mover (positivo = sentido +, negativo = sentido -).
 */
void start_move_steps(stepper_motor_t *motor, int32_t steps);

/**
 * @brief Agenda um movimento absoluto até a posição especificada (não-bloqueante).
 *
 * @param motor Ponteiro para o motor alvo.
 * @param target_steps Posição alvo, em passos (absolute step count).
 */
void start_move_to(stepper_motor_t *motor, int32_t target_steps);

/* --- API bloqueante (wrapper conveniência) --- */

/**
 * @brief Move o motor um número de passos e bloqueia até a conclusão.
 *
 * Wrapper que chama `start_move_steps()` e em seguida `wait_until_stop()`.
 *
 * @param motor Ponteiro para o motor alvo.
 * @param steps Quantidade de passos a mover (positivo/negativo).
 */
void move_steps_blocking(stepper_motor_t *motor, int32_t steps);

/**
 * @brief Move o motor para uma posição absoluta e bloqueia até a conclusão.
 *
 * Wrapper que chama `start_move_to()` e em seguida `wait_until_stop()`.
 *
 * @param motor Ponteiro para o motor alvo.
 * @param target_steps Posição alvo, em passos.
 */
void move_to_position_blocking(stepper_motor_t *motor, int32_t target_steps);

/* --- Funções de controle internas (existentes no código original) --- */

/**
 * @brief Prepara e agenda um movimento relativo (interna, não-bloqueante).
 *
 * Configura contadores, direção e agenda o primeiro alarme para gerar pulsos.
 *
 * @param motor Ponteiro para o motor alvo.
 * @param steps Número relativo de passos (positivo/negativo).
 */
void move_n_steps(stepper_motor_t *motor, int32_t steps);

/**
 * @brief Move até uma posição absoluta.
 *
 * Calcula o deslocamento necessário (target - posição atual) e chama move_n_steps().
 * Se `wait` for true, bloqueia até a conclusão do movimento.
 *
 * @param motor Ponteiro para o motor alvo.
 * @param target Posição alvo em passos (passos absolutos).
 * @param wait Se true, a função bloqueia até o término do movimento.
 */
void move_to_position(stepper_motor_t *motor, int32_t target, bool wait);

/**
 * @brief Callback do timer repetitivo usado para piscar o LED.
 *
 * Alterna o estado do pino LED_PIN. Projetado para ser usado com
 * add_repeating_timer_ms().
 *
 * @param rt Ponteiro para o repeating_timer_t (não utilizado).
 * @return true para manter o timer repetindo.
 */
bool led_callback(repeating_timer_t *rt);

/**
 * @brief Helper que indica se o movimento do motor especificado já terminou.
 *
 * @param motor Ponteiro para a instância do motor desejado.
 * @return true para um movimento finalizado.
 */
bool is_movement_done(stepper_motor_t *motor);

/**
 * @brief Função que bloqueia a execução do código enquanto o movimento do motor não terminar.
 *
 * @param motor Ponteiro para a instância do motor desejado.
 */
void wait_until_stop(stepper_motor_t *motor);

/**
 * @brief Define a posição atual do motor como zero (posição de referência).
 *
 * Esta função desativa temporariamente interrupções para evitar condição de
 * corrida com o alarm IRQ handler, e então escreve 0 no campo de posição.
 *
 * Observações:
 * - Use esta função quando for seguro redefinir a referência (idealmente com o
 *   motor parado), pois não ajusta outros contadores de movimento.
 * - Se quiser também reiniciar contadores de movimento (steps_executed / steps_total),
 *   veja a versão alternativa comentada abaixo.
 *
 * @param motor Ponteiro para a instância do motor.
 */
void set_motor_reference(stepper_motor_t *motor);

/**
 * @brief Para o movimento do motor
 *
 * @param motor Ponteiro para a instância do motor.
 */
void stop_move(stepper_motor_t *motor);

void btn_irq(uint gpio, uint32_t events);

// ========================================================================
// ADICIONADO: DECLARAÇÕES DAS FUNÇÕES DE MOVIMENTO SINCRONIZADO
// ========================================================================

/* Function declarations for coordinated movement */
int64_t coordinated_alarm_handler(alarm_id_t id, void *user_data);
bool enqueue_movement(int32_t target[]);
bool dequeue_movement(int32_t target[]);
void start_next_movement(void);
void calculate_bresenham_parameters(void);

/**
 * @brief Define the coordinated motors for X, Y, Z axes
 *
 * @param motor_x Pointer to X-axis motor (can be NULL)
 * @param motor_y Pointer to Y-axis motor (can be NULL)
 * @param motor_z Pointer to Z-axis motor (can be NULL)
 */
void stepperDefineXYZ(stepper_motor_t *motor_x, stepper_motor_t *motor_y, stepper_motor_t *motor_z);

/**
 * @brief Check if all coordinated motors are idle
 *
 * @return true if all motors are idle and no coordinated movement is active
 */
bool stepperIsIdleXYZ(void);

/**
 * @brief Move to absolute position using coordinated movement
 *
 * @param target Array containing target positions [x, y, z]
 */
void stepperMoveToXYZ(int32_t target[MAX_COORDINATED_AXES]);

/**
 * @brief Process movement queue and execute coordinated movements
 * This function should be called regularly from the main loop
 */
void stepperRun(void);

/**
 * @brief Get current position of coordinated axes
 *
 * @param position Buffer to store current positions [x, y, z]
 */
void stepperGetPositionXYZ(int32_t position[MAX_COORDINATED_AXES]);

/**
 * @brief Set current position as reference (zero) for all axes
 */
void stepperSetReferenceXYZ(void);

/**
 * @brief Stop all coordinated movement immediately
 */
void stepperStopXYZ(void);

/* -------------------------
   Implementações ORIGINAIS
   ------------------------- */

int main (void) {
    stdio_init_all();

    gpio_init(LED_PIN); gpio_set_dir(LED_PIN, GPIO_OUT); gpio_put(LED_PIN, 0);

    // Inicializa Botão
    gpio_init(BTN_CONTROL); gpio_set_dir(BTN_CONTROL, GPIO_IN); gpio_pull_up(BTN_CONTROL);
    gpio_set_irq_enabled_with_callback(BTN_CONTROL, GPIO_IRQ_EDGE_FALL, true, &btn_irq);

    repeating_timer_t led_timer;

    /* Inicializa os motores */
    for (uint i = 0; i < 2; i++) {
        init_stepper_motor(&steppers[i]);
    }

    const int MOTOR_STEPS = 48;
    int32_t steps_per_rev = MOTOR_STEPS * 16; /* 768 passos/rev */

    /* Exemplo de configuração inicial (comentado) */
    /* steppers[0].initial_step_interval_us = 4000.0f; // 4 ms entre passos no início */
    steppers[0].min_step_interval_us = 3000; // não acelera além de 1 ms por passo

    /* Cria instância do timer para o LED */
    add_repeating_timer_ms(500, led_callback, NULL, &led_timer);

    // ========================================================================
    // ADICIONADO: EXEMPLO DE USO DO SISTEMA SINCRONIZADO
    // ========================================================================

    // Define coordinated motors (X = steppers[0], Y = steppers[1])
    stepperDefineXYZ(&steppers[0], &steppers[1], NULL);

    // Draw a 1500 x 2000 steps rectangle
    #define ARRAY_SIZE 1
    uint8_t i = 0;
    bool draw = true;

    // int32_t xy[ARRAY_SIZE][MAX_COORDINATED_AXES] = {
    //     {1500, 0, 0},      // Move to (1500, 0)
    //     {1500, 2000, 0},   // Move to (1500, 2000)
    //     {0, 2000, 0},      // Move to (0, 2000)
    //     {0, 0, 0}          // Move back to (0, 0)
    // };
    int32_t xy[ARRAY_SIZE][MAX_COORDINATED_AXES] = {
        {4608, 6912, 0}
    };

    sleep_ms(5000);
    printf("Iniciando demo para movimento sincronizado...\n");

    mov_state = false;

    while (true) {
        // This processes the movement queue - MUST be called in main loop
        stepperRun();

        // Move to an absolute position in step units
        if(draw && stepperIsIdleXYZ()){
            printf("Movendo para posição: X=%ld, Y=%ld\n", xy[i][0], xy[i][1]);
            stepperMoveToXYZ(xy[i]);
            i++;
            if(i == ARRAY_SIZE) {
                draw = false;
                printf("Desenho do retângulo completo!\n");
            }
        }

        // Add a small delay to prevent busy waiting
        sleep_ms(5);

        // --------- TODOS OS TESTES ORIGINAIS COMENTADOS
        // Você pode descomentar qualquer um deles para testar movimento individual

        // --------- TESTE PARA PARAR O MOTOR
        // start_move_to(&steppers[0], 10000);
        // while (!is_movement_done(&steppers[0])) {
        //     if (mov_state) {
        //         stop_move(&steppers[0]);
        //         while (true);
        //     }
        // }
        // while (true);

        // --------- TESTE PARA DEFINIÇÃO DE NOVA REFERÊNCIA DO MOTOR (ponto zero)
        // start_move_to(&steppers[0], 576);
        // while (!is_movement_done(&steppers[0])) {
        //     tight_loop_contents();
        // }
        // sleep_ms(2000);
        // set_motor_reference(&steppers[0]);
        // start_move_to(&steppers[0], -576);
        // while (!is_movement_done(&steppers[0])) {
        //     tight_loop_contents();
        // }
        // sleep_ms(2000);
        // start_move_to(&steppers[0], 0);
        // while (!is_movement_done(&steppers[0])) {
        //     tight_loop_contents();
        // }
        // while(true);
    }

    return 0;
}

void btn_irq(uint gpio, uint32_t events) {
    absolute_time_t curr = to_ms_since_boot(get_absolute_time());

    if (gpio == BTN_CONTROL && curr - last_btn_press > 310) {
        last_btn_press = curr;

        mov_state = !mov_state;
    }
}

int64_t alarm_irq_handler(alarm_id_t id, void *user_data) {
    /* Obtém a instância do motor */
    stepper_motor_t *motor = (stepper_motor_t*)user_data;

    /* Caso o movimento tenha acabado, interrompe o agendamento de alarmes */
    if (motor->steps_executed >= motor->steps_total) {
        motor->movement_complete = true;
        return 0;
    }

    /* Verifica o estado do pulso enviado ao pino STEP */
    if (!motor->step_pin_state) {
        /* Produz um nível lógico alto */
        gpio_put(motor->pin_step, 1);
        motor->step_pin_state = true;

        return (int64_t)motor->half_step_interval_us;
    } else {
        /* Produz um nível lógico alto */
        gpio_put(motor->pin_step, 0);
        motor->step_pin_state = false;

        /* Incrementa o contador de passos e atualiza a posição do motor */
        motor->steps_executed++;
        motor->position_steps += motor->direction;

        /* Implementa a aceleração/desaceleração */
        if (motor->ramp_steps == 0) {
            motor->accel_counter++;
            motor->current_step_interval_us = motor->current_step_interval_us - (2.0f * motor->current_step_interval_us) / (4.0f * motor->accel_counter + 1.0f);

            if (motor->current_step_interval_us <= motor->min_step_interval_us) {
                motor->current_step_interval_us = motor->min_step_interval_us;
                motor->ramp_steps = motor->steps_executed;
            }
            if (motor->steps_executed >= motor->steps_total / 2) {
                motor->ramp_steps = motor->steps_executed;
            }
        } else if (motor->steps_executed >= motor->steps_total - motor->ramp_steps) {
            motor->accel_counter--;
            motor->current_step_interval_us = (motor->current_step_interval_us * (4.0f * motor->accel_counter + 1)) / (4.0f * motor->accel_counter + 1 - 2);
        }

        /* Computa o próximo meio-período */
        motor->half_step_interval_us = motor->current_step_interval_us * 0.5f;
        return (int64_t)motor->half_step_interval_us;
    }
}

bool led_callback(repeating_timer_t *rt) {
    gpio_put(LED_PIN, !gpio_get(LED_PIN));
    return true;
}

void init_stepper_motor(stepper_motor_t *motor) {
    gpio_init(motor->pin_step);
    gpio_set_dir(motor->pin_step, GPIO_OUT);
    gpio_put(motor->pin_step, 0);

    gpio_init(motor->pin_dir);
    gpio_set_dir(motor->pin_dir, GPIO_OUT);

    gpio_init(motor->pin_ms1);
    gpio_set_dir(motor->pin_ms1, GPIO_OUT);
    gpio_init(motor->pin_ms2);
    gpio_set_dir(motor->pin_ms2, GPIO_OUT);
    gpio_init(motor->pin_ms3);
    gpio_set_dir(motor->pin_ms3, GPIO_OUT);

    /* Define a resolução de 1/16 */
    gpio_put(motor->pin_ms1, 1);
    gpio_put(motor->pin_ms2, 1);
    gpio_put(motor->pin_ms3, 1);

    /* Valores iniciais para a rampa */
    motor->initial_step_interval_us = 6200.0f;
    motor->current_step_interval_us = motor->initial_step_interval_us;
    motor->half_step_interval_us    = motor->current_step_interval_us * 0.5f;
    motor->min_step_interval_us     = 100;
    motor->movement_complete        = true;
    motor->step_pin_state           = false;
    motor->position_steps           = 0;
    motor->steps_total              = 0;
    motor->steps_executed           = 0;
    motor->ramp_steps               = 0;
    motor->accel_counter            = 0;
    motor->direction                = 0;
}

void move_n_steps(stepper_motor_t *motor, int32_t steps) {
    /* Reseta os valores da rampa */
    motor->steps_total              = abs(steps);
    motor->steps_executed           = 0;
    motor->ramp_steps               = 0;
    motor->accel_counter            = 0;
    motor->movement_complete        = false;
    motor->position_steps           = motor->position_steps;
    motor->step_pin_state           = false;
    motor->current_step_interval_us = motor->initial_step_interval_us;
    motor->half_step_interval_us    = motor->current_step_interval_us * 0.5f;

    /* Define a direção de rotação */
    motor->direction = steps > 0 ? 1 : -1;
    gpio_put(motor->pin_dir, steps < 0 ? 1 : 0);

    /* Agenda o alarme para o primeiro pulso */
    motor->timer_alarm_id = add_alarm_in_us((int64_t)motor->half_step_interval_us, alarm_irq_handler, motor, false);
}

void move_to_position(stepper_motor_t *motor, int32_t target, bool wait) {
    move_n_steps(motor, target - (int32_t)motor->position_steps);
    if (wait) {
        while (!motor->movement_complete);
    }
}

void start_move_steps(stepper_motor_t *motor, int32_t steps) {
    if (!motor) return;

    move_n_steps(motor, steps);
}

void start_move_to(stepper_motor_t *motor, int32_t target_steps) {
    if (!motor) return;

    // move_to_position(..., false) agenda o movimento e retorna imediatamente
    move_to_position(motor, target_steps, false);
}

void move_steps_blocking(stepper_motor_t *motor, int32_t steps) {
    if (!motor) return;

    start_move_steps(motor, steps);
    wait_until_stop(motor);
}

void move_to_position_blocking(stepper_motor_t *motor, int32_t target_steps) {
    if (!motor) return;
    start_move_to(motor, target_steps);
    wait_until_stop(motor);
}

bool is_movement_done(stepper_motor_t *motor) {
    if (!motor) return true; // consider "done" for null pointer to avoid blocking
    return motor->movement_complete;
}

void wait_until_stop(stepper_motor_t *motor) {
    if (!motor) return;
    // busy-wait but yield cheaply on RP2040
    while (!is_movement_done(motor)) {
        tight_loop_contents();
    }
}

void set_motor_reference(stepper_motor_t *motor) {
    if (!motor) return;

    motor->position_steps   = 0;
    motor->steps_executed   = 0;
    motor->steps_total      = 0;
    motor->ramp_steps       = 0;
    motor->accel_counter    = 0;
    motor->movement_complete = true;
}

void stop_move(stepper_motor_t *motor) {
    if (!motor) return;
    cancel_alarm(motor->timer_alarm_id);

    motor->movement_complete = true;
    motor->step_pin_state = false;
    gpio_put(motor->pin_step, 0);
}

// ========================================================================
// ADICIONADO: IMPLEMENTAÇÕES DAS FUNÇÕES DE MOVIMENTO SINCRONIZADO
// ========================================================================

/**
 * @brief Define the coordinated motors for X, Y, Z axes
 */
void stepperDefineXYZ(stepper_motor_t *motor_x, stepper_motor_t *motor_y, stepper_motor_t *motor_z) {
    coord_stepper.motors[0] = motor_x;  // X-axis
    coord_stepper.motors[1] = motor_y;  // Y-axis
    coord_stepper.motors[2] = motor_z;  // Z-axis

    // Initialize current positions from motor positions
    for (uint8_t i = 0; i < MAX_COORDINATED_AXES; i++) {
        if (coord_stepper.motors[i] != NULL) {
            coord_stepper.current_position[i] = coord_stepper.motors[i]->position_steps;
        } else {
            coord_stepper.current_position[i] = 0;
        }
    }

    // Initialize movement parameters
    coord_stepper.initial_step_interval_us = 6200.0f;
    coord_stepper.min_step_interval_us = 1000.0f;
    coord_stepper.movement_active = false;
    coord_stepper.movement_complete = true;

    // Initialize command queue
    queue_head = queue_tail = queue_count = 0;
    for (uint8_t i = 0; i < COMMAND_QUEUE_SIZE; i++) {
        command_queue[i].valid = false;
    }
}

/**
 * @brief Check if all coordinated motors are idle
 */
bool stepperIsIdleXYZ(void) {
    if (coord_stepper.movement_active) {
        return false;
    }

    // Check individual motor states
    for (uint8_t i = 0; i < MAX_COORDINATED_AXES; i++) {
        if (coord_stepper.motors[i] != NULL) {
            if (!coord_stepper.motors[i]->movement_complete) {
                return false;
            }
        }
    }

    return queue_count == 0;
}

/**
 * @brief Move to absolute position using coordinated movement
 */
void stepperMoveToXYZ(int32_t target[MAX_COORDINATED_AXES]) {
    // Enqueue the movement command
    enqueue_movement(target);
}

/**
 * @brief Process movement queue and execute coordinated movements
 */
void stepperRun(void) {
    // If no movement is active and there are commands in queue
    if (!coord_stepper.movement_active && queue_count > 0) {
        start_next_movement();
    }
}

/**
 * @brief Enqueue a movement command
 */
bool enqueue_movement(int32_t target[MAX_COORDINATED_AXES]) {
    if (queue_count >= COMMAND_QUEUE_SIZE) {
        return false; // Queue full
    }

    // Copy target positions
    for (uint8_t i = 0; i < MAX_COORDINATED_AXES; i++) {
        command_queue[queue_tail].target[i] = target[i];
    }
    command_queue[queue_tail].valid = true;

    queue_tail = (queue_tail + 1) % COMMAND_QUEUE_SIZE;
    queue_count++;

    return true;
}

/**
 * @brief Dequeue a movement command
 */
bool dequeue_movement(int32_t target[MAX_COORDINATED_AXES]) {
    if (queue_count == 0) {
        return false; // Queue empty
    }

    // Copy target positions
    for (uint8_t i = 0; i < MAX_COORDINATED_AXES; i++) {
        target[i] = command_queue[queue_head].target[i];
    }
    command_queue[queue_head].valid = false;

    queue_head = (queue_head + 1) % COMMAND_QUEUE_SIZE;
    queue_count--;

    return true;
}

/**
 * @brief Start the next movement from the queue
 */
void start_next_movement(void) {
    int32_t target[MAX_COORDINATED_AXES];

    if (!dequeue_movement(target)) {
        return; // No movement in queue
    }

    // Copy current positions and calculate movement parameters
    coord_stepper.active_axes = 0;
    coord_stepper.total_steps = 0;
    coord_stepper.dominant_axis = 0;

    for (uint8_t i = 0; i < MAX_COORDINATED_AXES; i++) {
        if (coord_stepper.motors[i] != NULL) {
            coord_stepper.current_position[i] = coord_stepper.motors[i]->position_steps;
            coord_stepper.target_position[i] = target[i];

            // Calculate distance and direction
            int32_t delta = target[i] - coord_stepper.current_position[i];
            coord_stepper.distance[i] = abs(delta);
            coord_stepper.direction[i] = (delta > 0) ? 1 : -1;

            // Set motor direction
            gpio_put(coord_stepper.motors[i]->pin_dir, delta < 0 ? 1 : 0);

            if (coord_stepper.distance[i] > 0) {
                coord_stepper.active_axes++;

                // Find dominant axis (longest distance)
                if (coord_stepper.distance[i] > coord_stepper.total_steps) {
                    coord_stepper.total_steps = coord_stepper.distance[i];
                    coord_stepper.dominant_axis = i;
                }
            }

            coord_stepper.steps_executed[i] = 0;
        }
    }

    if (coord_stepper.total_steps == 0) {
        return; // No movement needed
    }

    // Calculate Bresenham parameters
    calculate_bresenham_parameters();

    // Initialize movement state
    coord_stepper.movement_active = true;
    coord_stepper.movement_complete = false;
    coord_stepper.current_step_interval_us = coord_stepper.initial_step_interval_us;
    coord_stepper.accel_counter = 0;
    coord_stepper.ramp_steps = 0;

    // Start the coordinated timer
    coord_stepper.coord_timer_id = add_alarm_in_us(
        (int64_t)(coord_stepper.current_step_interval_us * 0.5f),
        coordinated_alarm_handler,
        &coord_stepper,
        false
    );
}

/**
 * @brief Calculate Bresenham algorithm parameters
 */
void calculate_bresenham_parameters(void) {
    for (uint8_t i = 0; i < MAX_COORDINATED_AXES; i++) {
        if (coord_stepper.motors[i] != NULL && coord_stepper.distance[i] > 0) {
            // Initialize Bresenham error term
            coord_stepper.error[i] = coord_stepper.total_steps / 2;
        } else {
            coord_stepper.error[i] = 0;
        }
    }
}

/**
 * @brief Coordinated movement alarm handler
 */
int64_t coordinated_alarm_handler(alarm_id_t id, void *user_data) {
    coordinated_stepper_t *coord = (coordinated_stepper_t*)user_data;

    // Check if movement is complete
    bool all_axes_complete = true;
    for (uint8_t i = 0; i < MAX_COORDINATED_AXES; i++) {
        if (coord->motors[i] != NULL && coord->distance[i] > 0) {
            if (coord->steps_executed[i] < coord->distance[i]) {
                all_axes_complete = false;
                break;
            }
        }
    }

    if (all_axes_complete) {
        coord->movement_active = false;
        coord->movement_complete = true;
        return 0; // Stop the alarm
    }

    // Execute Bresenham algorithm for each axis
    for (uint8_t i = 0; i < MAX_COORDINATED_AXES; i++) {
        if (coord->motors[i] != NULL && coord->distance[i] > 0 &&
            coord->steps_executed[i] < coord->distance[i]) {

            coord->error[i] += coord->distance[i];

            if (coord->error[i] >= coord->total_steps) {
                coord->error[i] -= coord->total_steps;

                // Execute step for this axis
                gpio_put(coord->motors[i]->pin_step, 1);
                // Small delay for pulse width
                busy_wait_us_32(2);
                gpio_put(coord->motors[i]->pin_step, 0);

                // Update position and counters
                coord->motors[i]->position_steps += coord->direction[i];
                coord->steps_executed[i]++;
            }
        }
    }

    // Update current position tracking
    for (uint8_t i = 0; i < MAX_COORDINATED_AXES; i++) {
        if (coord->motors[i] != NULL) {
            coord->current_position[i] = coord->motors[i]->position_steps;
        }
    }

    // Acceleration/deceleration logic (similar to original implementation)
    uint32_t dominant_steps_executed = coord->steps_executed[coord->dominant_axis];

    if (coord->ramp_steps == 0) {
        // Acceleration phase
        coord->accel_counter++;
        coord->current_step_interval_us = coord->current_step_interval_us -
            (2.0f * coord->current_step_interval_us) / (4.0f * coord->accel_counter + 1.0f);

        if (coord->current_step_interval_us <= coord->min_step_interval_us) {
            coord->current_step_interval_us = coord->min_step_interval_us;
            coord->ramp_steps = dominant_steps_executed;
        }
        if (dominant_steps_executed >= coord->total_steps / 2) {
            coord->ramp_steps = dominant_steps_executed;
        }
    } else if (dominant_steps_executed >= coord->total_steps - coord->ramp_steps) {
        // Deceleration phase
        coord->accel_counter--;
        if (coord->accel_counter > 0) {
            coord->current_step_interval_us = (coord->current_step_interval_us *
                (4.0f * coord->accel_counter + 1)) / (4.0f * coord->accel_counter + 1 - 2);
        }
    }

    return (int64_t)(coord->current_step_interval_us * 0.5f);
}

/**
 * @brief Get current position of coordinated axes
 */
void stepperGetPositionXYZ(int32_t position[MAX_COORDINATED_AXES]) {
    for (uint8_t i = 0; i < MAX_COORDINATED_AXES; i++) {
        position[i] = coord_stepper.current_position[i];
    }
}

/**
 * @brief Set current position as reference (zero) for all axes
 */
void stepperSetReferenceXYZ(void) {
    // Wait for any movement to complete
    while (!stepperIsIdleXYZ()) {
        tight_loop_contents();
    }

    for (uint8_t i = 0; i < MAX_COORDINATED_AXES; i++) {
        if (coord_stepper.motors[i] != NULL) {
            coord_stepper.motors[i]->position_steps = 0;
            coord_stepper.current_position[i] = 0;
        }
    }
}

/**
 * @brief Stop all coordinated movement immediately
 */
void stepperStopXYZ(void) {
    if (coord_stepper.movement_active) {
        cancel_alarm(coord_stepper.coord_timer_id);
        coord_stepper.movement_active = false;
        coord_stepper.movement_complete = true;

        // Ensure all step pins are low
        for (uint8_t i = 0; i < MAX_COORDINATED_AXES; i++) {
            if (coord_stepper.motors[i] != NULL) {
                gpio_put(coord_stepper.motors[i]->pin_step, 0);
            }
        }
    }

    // Clear command queue
    queue_head = queue_tail = queue_count = 0;
    for (uint8_t i = 0; i < COMMAND_QUEUE_SIZE; i++) {
        command_queue[i].valid = false;
    }
}
