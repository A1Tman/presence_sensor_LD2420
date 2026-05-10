#pragma once
#include <stdint.h>

typedef struct {
    int8_t rssi;
} wifi_ap_record_t;

int esp_wifi_sta_get_ap_info(wifi_ap_record_t *ap);

