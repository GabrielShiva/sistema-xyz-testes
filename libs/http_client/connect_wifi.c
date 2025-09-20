#include "connect_wifi.h"

#include <stdio.h>         // Biblioteca manipular strings
#include "pico/stdlib.h"

// #include "lwip/netif.h"

const char WIFI_SSID[] = "";  // adicione o ssid da rede
const char WIFI_PASSWORD[] = "";  // adicione a senha

char* connect_wifi()
{
    if (cyw43_arch_init())
    {
        printf("Não foi possível conectar.");
        return NULL;
    }
    printf("Wi-Fi inicializado com sucesso\n"); // Mensagem de sucesso na inicialização
    cyw43_arch_enable_sta_mode(); // Habilita o modo estação (client) para a Pico

    // Conectar à rede WiFI - fazer um loop até que esteja conectado
    while(cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000) != 0){
        printf("Tentando conexão...\n");
    }
    printf("Conectado com sucesso! \n");

    uint8_t *ip = (uint8_t *)&(cyw43_state.netif[0].ip_addr.addr);
    static char ip_str[24];
    snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
    printf("IP: %s\n", ip_str);

    return ip_str;
}