#include "device_creds.h"

#include <stdbool.h>
#include <string.h>

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"

static const char *TAG = "device_creds";

static esp_err_t read_str(nvs_handle_t h, const char *key, char *out, size_t out_size, bool required) {
    size_t len = out_size;
    esp_err_t err = nvs_get_str(h, key, out, &len);
    if (err == ESP_ERR_NVS_NOT_FOUND && !required) {
        out[0] = '\0';
        return ESP_OK;
    }
    if (err == ESP_ERR_NVS_INVALID_LENGTH) {
        ESP_LOGE(TAG, "Credential '%s' is too long (max %u chars)", key, (unsigned)(out_size - 1));
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Credential '%s' missing (%s)", key, esp_err_to_name(err));
    }
    return err;
}

esp_err_t device_creds_load(device_creds_t *out) {
    memset(out, 0, sizeof(*out));

    // Never erase this partition on error: it only holds provisioned data.
    esp_err_t err = nvs_flash_init_partition(DEVICE_CREDS_PARTITION);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No usable '%s' partition (%s); run tools/provision.ps1",
                 DEVICE_CREDS_PARTITION, esp_err_to_name(err));
        return err;
    }

    nvs_handle_t h;
    err = nvs_open_from_partition(DEVICE_CREDS_PARTITION, DEVICE_CREDS_NAMESPACE, NVS_READONLY, &h);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "No credentials provisioned (%s); run tools/provision.ps1", esp_err_to_name(err));
        return err;
    }

    err = read_str(h, "wifi_ssid", out->wifi_ssid, sizeof(out->wifi_ssid), true);
    if (err == ESP_OK) err = read_str(h, "wifi_pass", out->wifi_pass, sizeof(out->wifi_pass), true);
    if (err == ESP_OK) err = read_str(h, "mqtt_user", out->mqtt_user, sizeof(out->mqtt_user), false);
    if (err == ESP_OK) err = read_str(h, "mqtt_pass", out->mqtt_pass, sizeof(out->mqtt_pass), false);
    nvs_close(h);

    if (err == ESP_OK && out->wifi_ssid[0] == '\0') {
        ESP_LOGE(TAG, "Credential 'wifi_ssid' is empty");
        err = ESP_ERR_INVALID_STATE;
    }
    if (err != ESP_OK) {
        memset(out, 0, sizeof(*out));
        return err;
    }

    ESP_LOGI(TAG, "Credentials loaded (MQTT %s)", out->mqtt_user[0] ? "authenticated" : "anonymous");
    return ESP_OK;
}
