// Ficheiro: http_client.h

#ifndef HTTP_CLIENT_H
#define HTTP_CLIENT_H

/**
 * @brief Realiza uma requisição HTTP GET para um servidor.
 * * @param server_ip O endereço IP do servidor como uma string (ex: "192.168.1.100").
 * @param port A porta do servidor (geralmente 80 para HTTP).
 * @param endpoint O caminho do recurso no servidor (ex: "/dados").
 * @param response_callback Um ponteiro para uma função que será chamada com o corpo da resposta.
 */
void http_get_request(const char* server_ip, uint16_t port, const char* endpoint, void (*response_callback)(const char* response_body));

/**
 * @brief Realiza uma requisição HTTP POST com um corpo JSON para um servidor.
 * * @param server_ip O endereço IP do servidor como uma string.
 * @param port A porta do servidor.
 * @param endpoint O caminho do recurso no servidor.
 * @param json_payload A string contendo os dados JSON a serem enviados.
 * @param response_callback Um ponteiro para uma função que será chamada com o corpo da resposta.
 */
void http_post_request(const char* server_ip, uint16_t port, const char* endpoint, const char* json_payload, void (*response_callback)(const char* response_body));


#endif // HTTP_CLIENT_H