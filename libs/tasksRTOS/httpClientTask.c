#include "tasksRTOS.h"

// --- Definições da API ---
#define API_SERVER_IP "" // Ex: "192.168.1.100" ou um IP de teste
#define API_SERVER_PORT 8080

/**
 * @brief Esta é a nossa função de callback. Ela será executada
 * quando uma resposta da API for recebida.
 * * @param response_body O corpo da resposta (ex: o JSON).
 */
void api_response_handler(const char* response_body) {
    printf("=====================\n");
    printf("Resposta da API Recebida:\n");
    printf("%s\n", response_body);
    printf("=====================\n\n");
}

void vHttpClientTask() 
{
    // Conecta ao Wi-Fi usando sua função já existente
    char* ip = connect_wifi();
    if (!ip) {
        // Fica em loop se não conseguir conectar
        while(true) { sleep_ms(1000); }
    }

    // Loop principal para fazer requisições periódicas
    while (true) {
        cyw43_arch_poll();
        
        // --- Exemplo de Requisição GET ---
        printf("Fazendo requisição GET para /info...\n");
        http_get_request(API_SERVER_IP, API_SERVER_PORT, "/fighter", api_response_handler);
        sleep_ms(10000); // Espera 10 segundos

        // // --- Exemplo de Requisição POST com JSON ---
        // printf("Fazendo requisição POST para /data...\n");
        // const char* json_data_to_send = "{\"sensor\":\"pico_w\",\"temperatura\":25.7}";
        // http_post_request(API_SERVER_IP, API_SERVER_PORT, "/data", json_data_to_send, api_response_handler);
        // sleep_ms(10000); // Espera mais 10 segundos
    }
}