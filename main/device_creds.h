#pragma once
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// Network credentials live in their own NVS partition ("creds", see
// partitions.csv), written over USB by tools/provision.ps1. They are never
// compiled into the firmware image, and OTA updates do not touch them.
#define DEVICE_CREDS_PARTITION "creds"
#define DEVICE_CREDS_NAMESPACE "creds"

typedef struct {
    char wifi_ssid[33];   // 32 chars max (802.11)
    char wifi_pass[65];   // 8..63 passphrase or 64 hex
    char mqtt_user[65];   // empty = anonymous
    char mqtt_pass[129];
} device_creds_t;

/**
 * Load credentials from the creds partition (read-only; never erased).
 * Returns ESP_OK only if at least wifi_ssid is present and every stored value
 * fits. On failure *out is zeroed and the reason is logged without values.
 */
esp_err_t device_creds_load(device_creds_t *out);

#ifdef __cplusplus
}
#endif
