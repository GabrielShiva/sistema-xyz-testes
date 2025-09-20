// Ficheiro: http_client.c (com printf's de depuração)

#include "pico/cyw43_arch.h"
#include "lwip/tcp.h"
#include "lwip/dns.h"
#include <string.h>
#include <stdlib.h>
#include "http_client.h"

#define RESPONSE_BUFFER_SIZE 2048

typedef struct HTTP_STATE_T {
    struct tcp_pcb *pcb;
    ip_addr_t remote_addr;
    char request_buffer[1024];
    char response_buffer[RESPONSE_BUFFER_SIZE];
    int response_pos;
    void (*response_callback)(const char*);
} HTTP_STATE;

// Função para fechar a conexão e liberar recursos
static void _http_client_close(HTTP_STATE *state) {
    printf("[HTTP DEBUG] Fechando conexao e liberando recursos.\n");
    if (state && state->pcb) {
        tcp_arg(state->pcb, NULL);
        tcp_recv(state->pcb, NULL);
        tcp_err(state->pcb, NULL);
        tcp_sent(state->pcb, NULL);
        tcp_close(state->pcb);
        state->pcb = NULL;
    }
    if (state) {
        free(state);
    }
}

// Callback para erros de TCP
static void _http_client_err(void *arg, err_t err) {
    HTTP_STATE *state = (HTTP_STATE*)arg;
    printf("[HTTP DEBUG] ERRO TCP: Código %d. Fechando conexão.\n", err);
    _http_client_close(state);
}

// Encontra o início do corpo da resposta HTTP
static char* _find_body_start(char* http_response) {
    char *body_start = strstr(http_response, "\r\n\r\n");
    if (body_start) {
        return body_start + 4;
    }
    return http_response;
}

// Callback para recebimento de dados
static err_t _http_client_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err) {
    HTTP_STATE *state = (HTTP_STATE*)arg;

    if (!p) {
        printf("[HTTP DEBUG] Conexao fechada pelo servidor. Resposta completa recebida.\n");
        state->response_buffer[state->response_pos] = '\0';
        
        if (state->response_callback) {
            printf("[HTTP DEBUG] Processando resposta e chamando o callback do usuario...\n");
            char* response_body = _find_body_start(state->response_buffer);
            state->response_callback(response_body);
            printf("[HTTP DEBUG] Callback do usuario executado.\n");
        }
        _http_client_close(state);
        return ERR_OK;
    }

    tcp_recved(tpcb, p->tot_len);
    printf("[HTTP DEBUG] Recebendo dados... (%d bytes)\n", p->len);

    if (state->response_pos + p->len < RESPONSE_BUFFER_SIZE) {
        memcpy(state->response_buffer + state->response_pos, p->payload, p->len);
        state->response_pos += p->len;
    } else {
        printf("[HTTP DEBUG] Erro: Buffer de resposta excedido!\n");
    }
    
    pbuf_free(p);
    return ERR_OK;
}

// Callback para quando a requisição foi enviada com sucesso
static err_t _http_client_sent(void *arg, struct tcp_pcb *tpcb, u16_t len) {
    printf("[HTTP DEBUG] Requisição HTTP enviada com sucesso (%d bytes).\n", len);
    return ERR_OK;
}

// Callback para quando a conexão TCP é estabelecida
static err_t _http_client_connected(void *arg, struct tcp_pcb *tpcb, err_t err) {
    HTTP_STATE *state = (HTTP_STATE*)arg;
    if (err != ERR_OK) {
        printf("[HTTP DEBUG] Falha ao conectar. Código de erro: %d\n", err);
        _http_client_close(state);
        return err;
    }
    
    printf("[HTTP DEBUG] CONEXAO TCP ESTABELECIDA. Enviando requisição...\n");
    tcp_sent(tpcb, _http_client_sent);
    
    tcp_write(tpcb, state->request_buffer, strlen(state->request_buffer), TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);

    return ERR_OK;
}

// Função interna genérica para iniciar uma requisição
static void _start_http_request(const char* server_ip, uint16_t port, const char* request_str, void (*response_callback)(const char*)) {
    printf("[HTTP DEBUG] Iniciando nova requisicao para %s:%d\n", server_ip, port);
    
    HTTP_STATE *state = calloc(1, sizeof(HTTP_STATE));
    if (!state) {
        printf("[HTTP DEBUG] Falha ao alocar memória para o estado.\n");
        return;
    }

    if (!ipaddr_aton(server_ip, &state->remote_addr)) {
        printf("[HTTP DEBUG] IP do servidor inválido: %s\n", server_ip);
        free(state);
        return;
    }
    printf("[HTTP DEBUG] IP resolvido com sucesso.\n");

    strncpy(state->request_buffer, request_str, sizeof(state->request_buffer) - 1);
    state->response_callback = response_callback;
    
    state->pcb = tcp_new();
    if (!state->pcb) {
        printf("[HTTP DEBUG] Falha ao criar PCB TCP.\n");
        free(state);
        return;
    }
    printf("[HTTP DEBUG] PCB TCP criado. Configurando callbacks...\n");
    
    tcp_arg(state->pcb, state);
    tcp_recv(state->pcb, _http_client_recv);
    tcp_err(state->pcb, _http_client_err);

    cyw43_arch_lwip_begin();
    printf("[HTTP DEBUG] Tentando conectar ao servidor...\n");
    err_t err = tcp_connect(state->pcb, &state->remote_addr, port, _http_client_connected);
    cyw43_arch_lwip_end();

    if (err != ERR_OK) {
        printf("[HTTP DEBUG] Falha imediata ao chamar tcp_connect. Código: %d\n", err);
        _http_client_close(state);
    }
}

// Implementação da função GET
void http_get_request(const char* server_ip, uint16_t port, const char* endpoint, void (*response_callback)(const char* response_body)) {
    printf("[HTTP DEBUG] Preparando requisição GET para o endpoint: %s\n", endpoint);
    static char request_str[512];
    snprintf(request_str, sizeof(request_str),
             "GET %s HTTP/1.1\r\n"
             "Host: %s\r\n"
             "Connection: close\r\n"
             "\r\n",
             endpoint, server_ip);

    _start_http_request(server_ip, port, request_str, response_callback);
}

// Implementação da função POST
void http_post_request(const char* server_ip, uint16_t port, const char* endpoint, const char* json_payload, void (*response_callback)(const char* response_body)) {
    printf("[HTTP DEBUG] Preparando requisição POST para o endpoint: %s\n", endpoint);
    static char request_str[1024];
    snprintf(request_str, sizeof(request_str),
             "POST %s HTTP/1.1\r\n"
             "Host: %s\r\n"
             "Content-Type: application/json\r\n"
             "Content-Length: %d\r\n"
             "Connection: close\r\n"
             "\r\n"
             "%s",
             endpoint, server_ip, (int)strlen(json_payload), json_payload);

    _start_http_request(server_ip, port, request_str, response_callback);
}