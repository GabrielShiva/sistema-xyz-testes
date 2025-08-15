#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "pico/stdlib.h"

#include "hardware/irq.h"
#include "hardware/timer.h"
#include "hardware/adc.h"

#define LED_PIN        25
#define JOYSTICK_X_PIN 26

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

float x_reading = 0.0f;

// Declaração de funções
int64_t alarm_irq_handler(alarm_id_t id, void *user_data);
void init_stepper_motor(stepper_motor_t *motor);
void move_n_steps(stepper_motor_t *motor, int32_t steps);
void move_to_position(stepper_motor_t *motor, int32_t target, bool wait);
bool led_callback(repeating_timer_t *rt);

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

    while (true) {
        uint16_t reading = adc_read();
        int delta = (int)reading - (int)joystick_center;

        if (abs(delta) <= (int)DEADZONE) {
            // dentro da zona morta -> garante que o motor esteja parado
            printf("Motor parado\n");
            if (m0->alarm_active) {
                stop_motor(m0);
            }
        } else {
            // fora da zona morta -> direção e velocidade proporcional à distância do centro
            // determina direção consistente com move_n_steps: steps > 0 -> dir = 1 -> gpio_put(dir_pin, 0)
            if (delta > 0) {
                m0->dir = 1;
                gpio_put(m0->dir_pin, 0);
            } else {
                m0->dir = -1;
                gpio_put(m0->dir_pin, 1);
            }


            // calcula normalização usando distância máxima do centro (considera assimetria)
            uint16_t max_pos_dev = ADC_MAX - joystick_center;
            uint16_t max_neg_dev = joystick_center;
            float max_dev = (delta > 0) ? (float)max_pos_dev : (float)max_neg_dev;
            if (max_dev <= 0.0f) max_dev = (float)ADC_MAX / 2.0f;

            float norm = (float)abs(delta) / max_dev;
            if (norm > 1.0f) norm = 1.0f;

            // mapeia norm (0..1) para intervalo de tempo entre passos [initial_step_interval .. max_speed]
            // initial_step_interval é lento (maior), max_speed é rápido (menor)
            float min_int = (float)m0->max_speed;
            float max_int = m0->initial_step_interval;
            float desired_interval = max_int - norm * (max_int - min_int); // linear

            if (desired_interval < MAX_INTERVAL_JOYSTICK) desired_interval = MAX_INTERVAL_JOYSTICK;

            // Define o intervalo dos pulsos
            m0->actual_step_interval = desired_interval;
            m0->half_period_interval = desired_interval * 0.5f;

            printf("Motor em movimento (leitura=%u) (dir=%d) (intervalo=%.1fus)\n", reading, m0->dir, m0->actual_step_interval);

            // se motor não está rodando, inicia em modo contínuo
            if (!m0->alarm_active) {
                start_motor_continuous(m0);
            }
        }

        sleep_ms(10); // taxa de leitura do joystick (50 Hz). Ajuste conforme necessidade
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

