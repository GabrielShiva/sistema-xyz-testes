#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "pico/stdlib.h"

#include "hardware/irq.h"
#include "hardware/timer.h"

#define LED_PIN   25

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

    alarm_id_t alarm_id;
} stepper_motor_t;

stepper_motor_t steppers[3] = {
    // Motor 1
    {
        .dir_pin = 0,
        .step_pin = 1,
        .ms1_pin = 13,
        .ms2_pin = 14,
        .ms3_pin = 15,
    },
    // Motor 2
    {
        .dir_pin = 17,
        .step_pin = 16,
        .ms1_pin = 13,
        .ms2_pin = 14,
        .ms3_pin = 15,
    },
    // // Motor 3
    // {
    //     .dir_pin = 4,
    //     .step_pin = 5,
    //     .ms1_pin = 13,
    //     .ms2_pin = 14,
    //     .ms3_pin = 15,
    // },
};

// Declaração de funções
int64_t alarm_irq_handler(alarm_id_t id, void *user_data);
void init_stepper_motor(stepper_motor_t *motor);
void move_n_steps(stepper_motor_t *motor, int32_t steps);
void move_to_position(stepper_motor_t *motor, int32_t target, bool wait);
bool led_callback(repeating_timer_t *rt);


// Função principal
int main (void) {
    stdio_init_all();

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

    sleep_ms(5000);
    printf("Iniciando demo para 3 motores...\n");

    while (true) {
        move_to_position(&steppers[0], steps_per_rev * 5, false);
        move_to_position(&steppers[1], steps_per_rev * 2, false);
        // move_to_position(&steppers[2], 1536, false);

        // while (!(steppers[0].movement_done && steppers[1].movement_done)) {
        //     tight_loop_contents();
        // }

        while(!steppers[1].movement_done) {
            tight_loop_contents();
        }

        move_to_position(&steppers[1], -steps_per_rev * 2, false);

        while(!(steppers[0].movement_done && steppers[1].movement_done)) {
            tight_loop_contents();
        }

        printf("Todos os motores executaram os movimentos!\n");

        while (true);
    }

    return 0;
}


int64_t alarm_irq_handler(alarm_id_t id, void *user_data) {
    // Obtém a instância do motor
    stepper_motor_t *motor = (stepper_motor_t*)user_data;

    // Caso o movimento tenha acabado, interrompe o agendamento de alarmes
    if (motor->step_count >= motor->total_steps) {
        motor->movement_done = true;
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
}

void move_to_position(stepper_motor_t *motor, int32_t target, bool wait) {
    move_n_steps(motor, target - (int32_t)motor->step_position);
    if (wait) {
        while (!motor->movement_done);
    }
}
