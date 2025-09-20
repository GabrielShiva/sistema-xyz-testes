// Ficheiro: lwipopts.h (Versão Otimizada para Servidor Web)

#ifndef __LWIPOPTS_H__
#define __LWIPOPTS_H__

// --- Configurações Gerais Mantidas do Original ---
#define NO_SYS                          1
#define LWIP_SOCKET                     0
#if PICO_CYW43_ARCH_POLL
#define MEM_LIBC_MALLOC                 1
#else
#define MEM_LIBC_MALLOC                 0
#endif
#define MEM_ALIGNMENT                   4
#define LWIP_ARP                        1
#define LWIP_ETHERNET                   1
#define LWIP_ICMP                       1
#define LWIP_RAW                        1
#define LWIP_NETIF_STATUS_CALLBACK      1
#define LWIP_NETIF_LINK_CALLBACK        1
#define LWIP_NETIF_HOSTNAME             1
#define LWIP_NETCONN                    0
#define LWIP_CHKSUM_ALGORITHM           3
#define LWIP_DHCP                       1
#define LWIP_IPV4                       1
#define LWIP_TCP                        1
#define LWIP_UDP                        1
#define LWIP_DNS                        1
#define LWIP_TCP_KEEPALIVE              1
#define LWIP_NETIF_TX_SINGLE_PBUF       1
#define DHCP_DOES_ARP_CHECK             0
#define LWIP_DHCP_DOES_ACD_CHECK        0

// --- Otimizações de Memória para o Servidor Web ---

// [ALTERADO] Aumenta o tamanho total da memória disponível para a lwIP.
#define MEM_SIZE                        16000

// [ALTERADO] Aumenta o número de segmentos TCP que podem ser enfileirados.
// Ajuda no envio de arquivos grandes em pedaços.
#define MEMP_NUM_TCP_SEG                64

// [ALTERADO] Aumenta drasticamente o número de buffers de pacotes.
// Esta é a principal correção para o erro ERR_MEM ao chamar tcp_write.
#define PBUF_POOL_SIZE                  48

// [ALTERADO] Define um buffer de envio TCP maior e mais explícito.
// 10 * 1460 = 14600 bytes.
#define TCP_SND_BUF                     (10 * TCP_MSS)

// Mantém as configurações originais relacionadas ao buffer de envio
#define TCP_MSS                         1460
#define TCP_WND                         (8 * TCP_MSS)
#define TCP_SND_QUEUELEN                ((4 * (TCP_SND_BUF) + (TCP_MSS - 1)) / (TCP_MSS))


// --- Configurações de Debug (mantidas do original) ---
#ifndef NDEBUG
#define LWIP_DEBUG                      1
#define LWIP_STATS                      1
#define LWIP_STATS_DISPLAY              1
#endif

#define ETHARP_DEBUG                    LWIP_DBG_OFF
#define NETIF_DEBUG                     LWIP_DBG_OFF
#define PBUF_DEBUG                      LWIP_DBG_OFF
#define API_LIB_DEBUG                   LWIP_DBG_OFF
#define API_MSG_DEBUG                   LWIP_DBG_OFF
#define SOCKETS_DEBUG                   LWIP_DBG_OFF
#define ICMP_DEBUG                      LWIP_DBG_OFF
#define INET_DEBUG                      LWIP_DBG_OFF
#define IP_DEBUG                        LWIP_DBG_OFF
#define IP_REASS_DEBUG                  LWIP_DBG_OFF
#define RAW_DEBUG                       LWIP_DBG_OFF
#define MEM_DEBUG                       LWIP_DBG_OFF
#define MEMP_DEBUG                      LWIP_DBG_OFF
#define SYS_DEBUG                       LWIP_DBG_OFF
#define TCP_DEBUG                       LWIP_DBG_OFF
#define TCP_INPUT_DEBUG                 LWIP_DBG_OFF
#define TCP_OUTPUT_DEBUG                LWIP_DBG_OFF
#define TCP_RTO_DEBUG                   LWIP_DBG_OFF
#define TCP_CWND_DEBUG                  LWIP_DBG_OFF
#define TCP_WND_DEBUG                   LWIP_DBG_OFF
#define TCP_FR_DEBUG                    LWIP_DBG_OFF
#define TCP_QLEN_DEBUG                  LWIP_DBG_OFF
#define TCP_RST_DEBUG                   LWIP_DBG_OFF
#define UDP_DEBUG                       LWIP_DBG_OFF
#define TCPIP_DEBUG                     LWIP_DBG_OFF
#define PPP_DEBUG                       LWIP_DBG_OFF
#define SLIP_DEBUG                      LWIP_DBG_OFF
#define DHCP_DEBUG                      LWIP_DBG_OFF

#endif /* __LWIPOPTS_H__ */