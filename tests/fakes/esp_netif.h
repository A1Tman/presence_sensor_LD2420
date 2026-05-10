#pragma once
#include <stdint.h>

typedef struct esp_netif_t esp_netif_t;

typedef struct {
    uint32_t addr;
} esp_ip4_addr_t;

typedef struct {
    esp_ip4_addr_t ip;
} esp_netif_ip_info_t;

#define IPSTR "%d.%d.%d.%d"
#define IP2STR(ipaddr) \
    ((int)((ipaddr)->addr & 0xff)), \
    ((int)(((ipaddr)->addr >> 8) & 0xff)), \
    ((int)(((ipaddr)->addr >> 16) & 0xff)), \
    ((int)(((ipaddr)->addr >> 24) & 0xff))

esp_netif_t *esp_netif_get_handle_from_ifkey(const char *ifkey);
int esp_netif_get_ip_info(esp_netif_t *netif, esp_netif_ip_info_t *ip);

