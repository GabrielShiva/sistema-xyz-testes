#include "tasksRTOS.h"

void vSerialTask()
{
    while (true)
    {
        // Processa os dados vindos via serial (dados enviados pela interface)
        process_serial_input();

        // Se o comando foi processado, lidar com o comando
        if (command_ready) {
            // Executa a função relacionada ao comando
            handle_command(command_buffer);
            command_buffer_pos = 0;
            command_ready = false;
        }

        // Envia atualizações de estado do sistema
        static int position_update_counter = 0;
        if (++position_update_counter >= 60) {
            send_position_update();
            send_homing_status();
            position_update_counter = 0;
        }

        sleep_ms(25);
    }
}

void vJoystickTask() 
{
    while (true) 
    {
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
            // In command mode, send minimal data or status
            printf("DATA,0,0,0,0,0.0,0.0\n"); // Or comment this out if no data needed
        }
    }
}