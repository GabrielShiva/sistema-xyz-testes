/**
 * @file multiple_motors_control.c
 * @brief Demo de controle de motores de passo com alarme/timer do RP2040.
 *
 * Contém definição da estrutura do motor, inicialização, agendamento de
 * pulsos via alarm IRQ e uma API simples (bloqueante e não-bloqueante).
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

/**
 * @brief Structure for synchronized movement of multiple motors
 */
typedef struct {
    stepper_motor_t *motor_x;           ///< Pointer to X-axis motor
    stepper_motor_t *motor_y;           ///< Pointer to Y-axis motor
    stepper_motor_t *motor_z;           ///< Pointer to Z-axis motor (optional)

    int32_t target_steps[3];            ///< Target steps for each axis [X, Y, Z]
    int32_t delta_steps[3];             ///< Delta steps for each axis (absolute values)
    int32_t direction[3];               ///< Direction for each axis (-1, 0, or +1)

    int32_t total_steps;                ///< Total steps in the dominant axis
    int32_t steps_executed;             ///< Steps executed so far

    int32_t bresenham_error[3];         ///< Bresenham error terms for each axis
    int32_t bresenham_delta[3];         ///< Bresenham delta terms for each axis

    volatile bool movement_complete;    ///< Flag indicating movement completion
    volatile bool step_ready[3];        ///< Flags indicating which motors should step

    /* Acceleration/deceleration variables */
    float initial_step_interval_us;     ///< Initial step interval
    float current_step_interval_us;     ///< Current step interval
    float half_sync_interval_us;        ///< Half of current sync interval
    float min_step_interval_us;         ///< Minimum step interval (max speed)
    uint32_t ramp_steps;                ///< Steps used for acceleration ramp
    uint32_t accel_counter;             ///< Acceleration counter
    volatile bool sync_step_state;      ///< Current step state (high/low)

    alarm_id_t sync_timer_id;           ///< Timer ID for synchronized movement

    uint8_t active_motors;              ///< Number of active motors (2 or 3)
} synchronized_movement_t;

/* Global synchronized movement instance */
synchronized_movement_t sync_move = {0};

/* -------------------------
   Declaração das funções
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

/* Function declarations for synchronized movement */

/**
 * @brief IRQ handler for synchronized movement
 * @param id Alarm ID
 * @param user_data Pointer to synchronized_movement_t structure
 * @return Next delay in microseconds or 0 to stop
 */
int64_t sync_alarm_irq_handler(alarm_id_t id, void *user_data);

/**
 * @brief Initialize synchronized movement structure
 * @param sync Pointer to synchronized_movement_t structure
 * @param motor_x Pointer to X-axis motor
 * @param motor_y Pointer to Y-axis motor
 * @param motor_z Pointer to Z-axis motor (can be NULL for 2D movement)
 */
void init_synchronized_movement(synchronized_movement_t *sync,
                               stepper_motor_t *motor_x,
                               stepper_motor_t *motor_y,
                               stepper_motor_t *motor_z);

/**
 * @brief Start synchronized movement to target position
 * @param sync Pointer to synchronized_movement_t structure
 * @param target_x Target position for X-axis
 * @param target_y Target position for Y-axis
 * @param target_z Target position for Z-axis (ignored if motor_z is NULL)
 */
void start_synchronized_move_to(synchronized_movement_t *sync,
                               int32_t target_x,
                               int32_t target_y,
                               int32_t target_z);

/**
 * @brief Blocking synchronized movement to target position
 * @param sync Pointer to synchronized_movement_t structure
 * @param target_x Target position for X-axis
 * @param target_y Target position for Y-axis
 * @param target_z Target position for Z-axis (ignored if motor_z is NULL)
 */
void synchronized_move_to_blocking(synchronized_movement_t *sync,
                                  int32_t target_x,
                                  int32_t target_y,
                                  int32_t target_z);

/**
 * @brief Check if synchronized movement is complete
 * @param sync Pointer to synchronized_movement_t structure
 * @return true if movement is complete, false otherwise
 */
bool is_synchronized_movement_done(synchronized_movement_t *sync);

/**
 * @brief Wait until synchronized movement is complete
 * @param sync Pointer to synchronized_movement_t structure
 */
void wait_synchronized_movement(synchronized_movement_t *sync);

/**
 * @brief Stop synchronized movement
 * @param sync Pointer to synchronized_movement_t structure
 */
void stop_synchronized_movement(synchronized_movement_t *sync);


/* -------------------------
   Implementações
   ------------------------- */

int main (void) {
    stdio_init_all();

    gpio_init(LED_PIN); gpio_set_dir(LED_PIN, GPIO_OUT); gpio_put(LED_PIN, 0);

    // Inicializa Botão
    gpio_init(BTN_CONTROL); gpio_set_dir(BTN_CONTROL, GPIO_IN); gpio_pull_up(BTN_CONTROL);
    gpio_set_irq_enabled_with_callback(BTN_CONTROL, GPIO_IRQ_EDGE_FALL, true, &btn_irq);

    repeating_timer_t led_timer;

    /* Cria instância do timer para o LED */
    add_repeating_timer_ms(500, led_callback, NULL, &led_timer);

    /* Inicializa os motores */
    for (uint i = 0; i < 2; i++) {
        init_stepper_motor(&steppers[i]);
    }

    /* Initialize synchronized movement for 2 motors (X and Y) */
    init_synchronized_movement(&sync_move, &steppers[0], &steppers[1], NULL);

    sleep_ms(5000);
    printf("Starting synchronized movement demo...\n");

    /* Define trajectory points */
    int32_t xy[3][2] = {
        {4608, 6912},
        {1500, 2000},
        {0, 0}
    };

    /* Execute synchronized movements */
    for (int i = 0; i < 3; i++) {
        printf("Moving to position (%ld, %ld)\n", xy[i][0], xy[i][1]);

        /* Use synchronized movement - both motors reach destination simultaneously */
        synchronized_move_to_blocking(&sync_move, xy[i][0], xy[i][1], 0);

        printf("Reached position (%ld, %ld)\n", steppers[0].position_steps, steppers[1].position_steps);

        sleep_ms(1000); /* Pause between movements */
    }

    printf("Synchronized movement demo complete!\n");

    while (true) {
        tight_loop_contents();
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

/* Implementation of synchronized movement functions */
/* Implementation of synchronized movement functions */

int64_t sync_alarm_irq_handler(alarm_id_t id, void *user_data) {
    synchronized_movement_t *sync = (synchronized_movement_t*)user_data;

    /* Check if movement is complete */
    if (sync->steps_executed >= sync->total_steps) {
        sync->movement_complete = true;
        return 0;
    }

    /* Handle step pulse generation */
    if (!sync->sync_step_state) {
        /* Rising edge - set step pins high for active motors */
        sync->sync_step_state = true;

        if (sync->step_ready[0] && sync->motor_x) {
            gpio_put(sync->motor_x->pin_step, 1);
        }
        if (sync->step_ready[1] && sync->motor_y) {
            gpio_put(sync->motor_y->pin_step, 1);
        }
        if (sync->step_ready[2] && sync->motor_z) {
            gpio_put(sync->motor_z->pin_step, 1);
        }

        return (int64_t)sync->half_sync_interval_us;
    } else {
        /* Falling edge - set step pins low and update positions */
        sync->sync_step_state = false;

        /* Clear step pins and update positions */
        if (sync->step_ready[0] && sync->motor_x) {
            gpio_put(sync->motor_x->pin_step, 0);
            sync->motor_x->position_steps += sync->direction[0];
        }
        if (sync->step_ready[1] && sync->motor_y) {
            gpio_put(sync->motor_y->pin_step, 0);
            sync->motor_y->position_steps += sync->direction[1];
        }
        if (sync->step_ready[2] && sync->motor_z) {
            gpio_put(sync->motor_z->pin_step, 0);
            sync->motor_z->position_steps += sync->direction[2];
        }

        /* Increment step counter */
        sync->steps_executed++;

        /* Apply acceleration/deceleration profile */
        if (sync->ramp_steps == 0) {
            /* Acceleration phase */
            sync->accel_counter++;
            sync->current_step_interval_us = sync->current_step_interval_us -
                (2.0f * sync->current_step_interval_us) / (4.0f * sync->accel_counter + 1.0f);

            if (sync->current_step_interval_us <= sync->min_step_interval_us) {
                sync->current_step_interval_us = sync->min_step_interval_us;
                sync->ramp_steps = sync->steps_executed;
            }
            if (sync->steps_executed >= sync->total_steps / 2) {
                sync->ramp_steps = sync->steps_executed;
            }
        } else if (sync->steps_executed >= sync->total_steps - sync->ramp_steps) {
            /* Deceleration phase */
            sync->accel_counter--;
            sync->current_step_interval_us = (sync->current_step_interval_us *
                (4.0f * sync->accel_counter + 1)) / (4.0f * sync->accel_counter + 1 - 2);
        }

        /* Update timing */
        sync->half_sync_interval_us = sync->current_step_interval_us * 0.5f;

        /* Reset step ready flags */
        sync->step_ready[0] = sync->step_ready[1] = sync->step_ready[2] = false;

        /* Bresenham algorithm to determine which motors should step */
        for (int i = 0; i < sync->active_motors; i++) {
            if (sync->delta_steps[i] > 0) {
                sync->bresenham_error[i] += sync->bresenham_delta[i];
                if (sync->bresenham_error[i] >= sync->total_steps) {
                    sync->step_ready[i] = true;
                    sync->bresenham_error[i] -= sync->total_steps;
                }
            }
        }

        return (int64_t)sync->half_sync_interval_us;
    }
}

void init_synchronized_movement(synchronized_movement_t *sync,
                               stepper_motor_t *motor_x,
                               stepper_motor_t *motor_y,
                               stepper_motor_t *motor_z) {
    if (!sync || !motor_x || !motor_y) return;

    sync->motor_x = motor_x;
    sync->motor_y = motor_y;
    sync->motor_z = motor_z;

    sync->active_motors = motor_z ? 3 : 2;
    sync->movement_complete = true;
    sync->sync_step_state = false;
    sync->steps_executed = 0;
    sync->total_steps = 0;

    /* Initialize acceleration parameters */
    sync->initial_step_interval_us = 1500.0f;  // Faster initial speed (1.5ms)
    sync->current_step_interval_us = sync->initial_step_interval_us;
    sync->min_step_interval_us = 400.0f;       // Much faster max speed (400us)
    sync->ramp_steps = 0;
    sync->accel_counter = 0;

    /* Initialize Bresenham variables */
    for (int i = 0; i < 3; i++) {
        sync->target_steps[i] = 0;
        sync->delta_steps[i] = 0;
        sync->direction[i] = 0;
        sync->bresenham_error[i] = 0;
        sync->bresenham_delta[i] = 0;
        sync->step_ready[i] = false;
    }

    /* Set synchronized timing based on motors' settings */
    float min_initial_interval = motor_x->initial_step_interval_us;
    if (motor_y->initial_step_interval_us < min_initial_interval) {
        min_initial_interval = motor_y->initial_step_interval_us;
    }
    if (motor_z && motor_z->initial_step_interval_us < min_initial_interval) {
        min_initial_interval = motor_z->initial_step_interval_us;
    }

    /* Use faster settings - you can adjust these values */
    sync->initial_step_interval_us = fmin(1500.0f, min_initial_interval);
    sync->current_step_interval_us = sync->initial_step_interval_us;
    sync->half_sync_interval_us = sync->current_step_interval_us * 0.5f;
}

void start_synchronized_move_to(synchronized_movement_t *sync,
                               int32_t target_x,
                               int32_t target_y,
                               int32_t target_z) {
    if (!sync || !sync->motor_x || !sync->motor_y) return;

    /* Calculate delta movements */
    int32_t current_pos[3] = {
        (int32_t)sync->motor_x->position_steps,
        (int32_t)sync->motor_y->position_steps,
        sync->motor_z ? (int32_t)sync->motor_z->position_steps : 0
    };

    sync->target_steps[0] = target_x;
    sync->target_steps[1] = target_y;
    sync->target_steps[2] = target_z;

    int32_t delta_x = target_x - current_pos[0];
    int32_t delta_y = target_y - current_pos[1];
    int32_t delta_z = sync->motor_z ? (target_z - current_pos[2]) : 0;

    sync->delta_steps[0] = abs(delta_x);
    sync->delta_steps[1] = abs(delta_y);
    sync->delta_steps[2] = abs(delta_z);

    sync->direction[0] = (delta_x > 0) ? 1 : ((delta_x < 0) ? -1 : 0);
    sync->direction[1] = (delta_y > 0) ? 1 : ((delta_y < 0) ? -1 : 0);
    sync->direction[2] = (delta_z > 0) ? 1 : ((delta_z < 0) ? -1 : 0);

    /* Find the dominant axis (largest movement) */
    sync->total_steps = sync->delta_steps[0];
    if (sync->delta_steps[1] > sync->total_steps) {
        sync->total_steps = sync->delta_steps[1];
    }
    if (sync->motor_z && sync->delta_steps[2] > sync->total_steps) {
        sync->total_steps = sync->delta_steps[2];
    }

    /* If no movement required, return */
    if (sync->total_steps == 0) {
        sync->movement_complete = true;
        return;
    }

    /* Set direction pins */
    gpio_put(sync->motor_x->pin_dir, sync->direction[0] < 0 ? 1 : 0);
    gpio_put(sync->motor_y->pin_dir, sync->direction[1] < 0 ? 1 : 0);
    if (sync->motor_z) {
        gpio_put(sync->motor_z->pin_dir, sync->direction[2] < 0 ? 1 : 0);
    }

    /* Initialize Bresenham algorithm */
    for (int i = 0; i < sync->active_motors; i++) {
        sync->bresenham_error[i] = sync->delta_steps[i] - sync->total_steps;
        sync->bresenham_delta[i] = sync->delta_steps[i];
        sync->step_ready[i] = false;
    }

    /* Reset movement state */
    sync->steps_executed = 0;
    sync->movement_complete = false;
    sync->sync_step_state = false;
    sync->ramp_steps = 0;
    sync->accel_counter = 0;

    /* Initialize acceleration parameters */
    sync->current_step_interval_us = sync->initial_step_interval_us;
    sync->half_sync_interval_us = sync->current_step_interval_us * 0.5f;

    /* Start the synchronized timer */
    sync->sync_timer_id = add_alarm_in_us((int64_t)sync->half_sync_interval_us,
                                         sync_alarm_irq_handler, sync, false);
}

void synchronized_move_to_blocking(synchronized_movement_t *sync,
                                  int32_t target_x,
                                  int32_t target_y,
                                  int32_t target_z) {
    start_synchronized_move_to(sync, target_x, target_y, target_z);
    wait_synchronized_movement(sync);
}

bool is_synchronized_movement_done(synchronized_movement_t *sync) {
    if (!sync) return true;
    return sync->movement_complete;
}

void wait_synchronized_movement(synchronized_movement_t *sync) {
    if (!sync) return;
    while (!is_synchronized_movement_done(sync)) {
        tight_loop_contents();
    }
}

void stop_synchronized_movement(synchronized_movement_t *sync) {
    if (!sync) return;

    cancel_alarm(sync->sync_timer_id);
    sync->movement_complete = true;
    sync->sync_step_state = false;

    /* Ensure all step pins are low */
    if (sync->motor_x) gpio_put(sync->motor_x->pin_step, 0);
    if (sync->motor_y) gpio_put(sync->motor_y->pin_step, 0);
    if (sync->motor_z) gpio_put(sync->motor_z->pin_step, 0);
}
