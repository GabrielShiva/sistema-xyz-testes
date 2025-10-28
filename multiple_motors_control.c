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
    { .pin_dir = 0, .pin_step = 1, .pin_ms1 = 13, .pin_ms2 = 14, .pin_ms3 = 15 },
    { .pin_dir = 16, .pin_step = 17, .pin_ms1 = 13, .pin_ms2 = 14, .pin_ms3 = 15 },
    { .pin_dir = 19, .pin_step = 18, .pin_ms1 = 13, .pin_ms2 = 14, .pin_ms3 = 15 }
};

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

    /* Inicializa os motores */
    for (uint i = 0; i < 3; i++) {
        init_stepper_motor(&steppers[i]);
    }

    const int MOTOR_STEPS = 48;
    int32_t steps_per_rev = MOTOR_STEPS * 16; /* 768 passos/rev */

    /* Exemplo de configuração inicial (comentado) */
    /* steppers[0].initial_step_interval_us = 4000.0f; // 4 ms entre passos no início */
    // steppers[0].min_step_interval_us = 3000; // não acelera além de 1 ms por passo

    /* Cria instância do timer para o LED */
    add_repeating_timer_ms(500, led_callback, NULL, &led_timer);

    sleep_ms(5000);
    printf("Iniciando demo para 3 motores...\n");

    // mov_state = false;

    while (true) {
        // --------- TESTE PARA MOVIMENTAR DOIS MOTORES AO MESMO TEMPO
        start_move_to(&steppers[0], 4608);
        start_move_to(&steppers[1], 6912);
        start_move_to(&steppers[2], 6912);

        while (!(is_movement_done(&steppers[0]) && is_movement_done(&steppers[1]) && is_movement_done(&steppers[2]))) {
            tight_loop_contents();
        }

        sleep_ms(2000);

        start_move_to(&steppers[0], 0);
        start_move_to(&steppers[1], 0);
        start_move_to(&steppers[2], 0);

        while(true);

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


        // --------- TESTE PARA MOVIMENTAR DOIS MOTORES
        // start_move_to(&steppers[0], 192);
        // start_move_to(&steppers[1], 192);

        // while (!(is_movement_done(&steppers[0]) && is_movement_done(&steppers[1]))) {
        //     tight_loop_contents();
        // }

        // /* Agenda movimentos para 384 passos */
        // start_move_to(&steppers[0], 384);
        // start_move_to(&steppers[1], 384);

        // while (!(is_movement_done(&steppers[0]) && is_movement_done(&steppers[1]))) {
        //     tight_loop_contents();
        // }

        // /* Agenda movimentos para 576 passos */
        // start_move_to(&steppers[0], 576);
        // start_move_to(&steppers[1], 576);

        // while (!(is_movement_done(&steppers[0]) && is_movement_done(&steppers[1]))) {
        //     tight_loop_contents();
        // }

        // start_move_to(&steppers[2], -900);

        // /* Agenda movimentos para 768 passos */
        // start_move_to(&steppers[0], 768);
        // start_move_to(&steppers[1], 768);

        // while (!(is_movement_done(&steppers[0]) && is_movement_done(&steppers[1]))) {
        //     tight_loop_contents();
        // }

        // sleep_ms(1500);

        // /* Volta para a posição 0 (ambos motores) */
        // start_move_to(&steppers[0], 0);
        // start_move_to(&steppers[1], 0);

        // while (!(is_movement_done(&steppers[0]) && is_movement_done(&steppers[1]))) {
        //     tight_loop_contents();
        // }

        printf("Todos os motores executaram os movimentos!\n");

        while (true);
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
    motor->min_step_interval_us     = 200;
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
