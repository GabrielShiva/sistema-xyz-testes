#include "homing.h"
#include "stepper_control.h"
#include "command_handler.h"

// Realiza o HOMING de um motor
bool home_single_motor(stepper_motor_t *motor, int motor_id) {
    // Caso o motor já se encontre no fim do eixo (x ou y)
    if (read_limit_switch(motor)) {
        printf("ACK,Motor %d ja esta no fim de curso - realizando retorno \n", motor_id);

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

    // Parte 1 -> Aproxima até a chave de fim de curso ativar
    printf("ACK,Motor %d: Fase 1 - aproximacao rapida\n", motor_id);

    motor->homing_mode = true;
    motor->limit_switch_triggered = false;
    motor->initial_step_interval = HOMING_SPEED_FAST;
    motor->max_speed = HOMING_SPEED_FAST;
    motor->actual_step_interval = HOMING_SPEED_FAST;

    motor->dir = -1;
    gpio_put(motor->dir_pin, 1);

    start_motor_continuous(motor);

    // Espera o motor chegar na chave (máximo de 50 segundos)
    uint32_t start_time = to_ms_since_boot(get_absolute_time());
    while (!motor->limit_switch_triggered && motor->alarm_active) {
        if (to_ms_since_boot(get_absolute_time()) - start_time > 50000) {
            stop_motor(motor);
            printf("ERROR,Timeout no homing do motor %d - limit switch nao encontrado\n", motor_id);
            return false;
        }
        sleep_ms(1);
    }

    if (!motor->limit_switch_triggered) {
        printf("ERROR,Motor %d nao atingiu o limit switch\n", motor_id);
        return false;
    }

    printf("ACK,Motor %d: Limit switch atingido\n", motor_id);
    sleep_ms(100);

    // Parte 2 -> Recua da chave
    printf("ACK,Motor %d: Fase 2 - recuo do limit switch\n", motor_id);

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
    printf("ACK,Motor %d: Fase 3 - aproximacao final lenta\n", motor_id);

    motor->homing_mode = true;
    motor->limit_switch_triggered = false;
    motor->initial_step_interval = HOMING_SPEED_SLOW;
    motor->max_speed = HOMING_SPEED_SLOW;
    motor->actual_step_interval = HOMING_SPEED_SLOW;

    motor->dir = -1;
    gpio_put(motor->dir_pin, 1);

    start_motor_continuous(motor);

    start_time = to_ms_since_boot(get_absolute_time());
    while (!motor->limit_switch_triggered && motor->alarm_active) {
        if (to_ms_since_boot(get_absolute_time()) - start_time > 10000) {
            stop_motor(motor);
            printf("ERROR,Timeout na aproximacao final do motor %d\n", motor_id);
            return false;
        }
        sleep_ms(1);
    }

    // Define a posição atual como zero
    motor->step_position = 0;
    motor->is_homed = true;
    motor->homing_mode = false;

    // Reseta os parâmetros para os valores normais
    motor->initial_step_interval = 6200.0f;
    motor->max_speed = 500;
    motor->actual_step_interval = motor->initial_step_interval;

    printf("ACK,Motor %d: Posicao home definida\n", motor_id);
    send_position_update();

    return true;
}

// Realiza o HOMING de todos os motores
bool home_all_motors(void) {
    bool all_success = true;

    // Executa o processo de homing (definição do ponto zero) para cada motor
    for (uint i = 0; i < active_motor_count && i < 3; i++) {
        printf("ACK,Iniciando homing do motor %d\n", i);
        if (!home_single_motor(&steppers[i], i)) {
            all_success = false;
            printf("ERROR,Falha no homing do motor %d\n", i);
        }

        sleep_ms(500);
    }

    return all_success;
}

// Retorna o estado da chave de fim de curso (pressionado ou não)
bool read_limit_switch(stepper_motor_t *motor) {
    return gpio_get(motor->limit_switch_pin);
}

// Inicializa as chaves de fim de curso e ativa os resistores de pull-down
void init_limit_switches(void) {
    gpio_init(LIMIT_SWITCH_X_PIN);
    gpio_set_dir(LIMIT_SWITCH_X_PIN, GPIO_IN);
    gpio_pull_down(LIMIT_SWITCH_X_PIN);

    gpio_init(LIMIT_SWITCH_Y_PIN);
    gpio_set_dir(LIMIT_SWITCH_Y_PIN, GPIO_IN);
    gpio_pull_down(LIMIT_SWITCH_Y_PIN);
}