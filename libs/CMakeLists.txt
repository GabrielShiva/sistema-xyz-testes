add_library(libs STATIC
    ${CMAKE_CURRENT_LIST_DIR}/stepper_control/stepper_control.c
    ${CMAKE_CURRENT_LIST_DIR}/joystick_control/joystick_control.c
    ${CMAKE_CURRENT_LIST_DIR}/command_handler/command_handler.c
    ${CMAKE_CURRENT_LIST_DIR}/tasksRTOS/tasksRTOS.c
    ${CMAKE_CURRENT_LIST_DIR}/tasksRTOS/httpClientTask.c
    ${CMAKE_CURRENT_LIST_DIR}/position_handler/position_handler.c
    ${CMAKE_CURRENT_LIST_DIR}/position_handler/homing.c
    ${CMAKE_CURRENT_LIST_DIR}/http_client/http_client.c
    ${CMAKE_CURRENT_LIST_DIR}/http_client/connect_wifi.c
)

target_include_directories (libs PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/stepper_control
    ${CMAKE_CURRENT_LIST_DIR}/joystick_control
    ${CMAKE_CURRENT_LIST_DIR}/command_handler
    ${CMAKE_CURRENT_LIST_DIR}/freeRTOS
    ${CMAKE_CURRENT_LIST_DIR}/tasksRTOS
    ${CMAKE_CURRENT_LIST_DIR}/position_handler
    ${CMAKE_CURRENT_LIST_DIR}/http_client
    ${CMAKE_SOURCE_DIR}
    ${FREERTOS_KERNEL_PATH}/include
)

target_link_libraries(libs PUBLIC
    pico_stdlib
    hardware_timer
    hardware_adc
    hardware_irq
    pico_cyw43_arch_lwip_threadsafe_background
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)