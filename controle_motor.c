#include <stdio.h>
#include "pico/stdlib.h"
#include "tasksRTOS.h"

// --- Definição das Variáveis Globais ---
// (Aqui é onde elas são realmente criadas na memória)
stepper_motor_t steppers[3] = {
    { .dir_pin = 0, .step_pin = 1, .ms1_pin = 13, .ms2_pin = 14, .ms3_pin = 15 },
    { .dir_pin = 17, .step_pin = 16, .ms1_pin = 13, .ms2_pin = 14, .ms3_pin = 15 }
};
char command_buffer[COMMAND_BUFFER_SIZE];
int command_buffer_pos = 0;
bool command_ready = false;
uint16_t joystick_x_center = 2048;
uint16_t joystick_y_center = 2048;
system_state_t current_state = STATE_JOYSTICK;
uint8_t active_motor_count = 2;

saved_position_t saved_positions[MAX_SAVED_POSITIONS];


// Função de callback para o LED
bool led_callback(repeating_timer_t *rt) {
    gpio_put(LED_PIN, !gpio_get(LED_PIN));
    return true;
}

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

    // Initialize limit switches
    init_limit_switches();

    // Initialize saved positions array
    init_saved_positions();

    // Inicializa os motores
    for (uint i = 0; i < 2; i++) {
        init_stepper_motor(&steppers[i]);
    }

    // Delay para dar tempo de ligar o serial
    sleep_ms(5000);

    // Calibração inicial do joystick: lê 100 amostras e define o centro
    // Nesse momento, o joystick não deve ser movido    joystick_x_center = read_joystick_average(0, NUM_CAL_SAMPLES, CAL_SAMPLE_DELAY_MS);
    printf("Centro - Joystick X  (media para %d amostras) = %u\n", NUM_CAL_SAMPLES, joystick_x_center);
    joystick_y_center = read_joystick_average(1, NUM_CAL_SAMPLES, CAL_SAMPLE_DELAY_MS);
    printf("Centro - Joystick Y (media para %d amostras) = %u\n", NUM_CAL_SAMPLES, joystick_y_center);

    printf("Iniciando 2 motores...\n");

    // Envia o estado do sistema e a posição atual para a interface (INICIALIZAÇÃO DA INTERFACE)
    send_state_update();
    send_position_update();
    send_saved_positions_update();
    send_homing_status();

    // xTaskCreate(vSerialTask, "Serial Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    // xTaskCreate(vJoystickTask, "Joystick Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);
    xTaskCreate(vHttpClientTask, "Http Client Task", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, NULL);

    vTaskStartScheduler();
    
    panic_unsupported();

    return 0;
}