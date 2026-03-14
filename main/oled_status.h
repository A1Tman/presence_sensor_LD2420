#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    bool sensor_ready;
    bool sensor_packets_valid;
    bool presence;
    bool wifi_connected;
    bool mqtt_connected;
    int distance_cm;
    int rssi_dbm;
    uint8_t ip_last_octet;
    int min_gate;
    int max_gate;
    int delay_ms;
    int trigger_sens;
    int maintain_sens;
    char fw_version[16];
} oled_status_snapshot_t;

typedef void (*oled_status_snapshot_cb_t)(oled_status_snapshot_t *out_snapshot);

bool oled_status_init(oled_status_snapshot_cb_t snapshot_cb, const char *app_version);

#ifdef __cplusplus
}
#endif
