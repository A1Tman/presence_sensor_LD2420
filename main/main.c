#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "esp_err.h"

#include "ld2420.h"  // LD2420 library
#include "ha_mqtt.h"
#include "oled_status.h"
#include "../config/secrets.h"

#define DEVICE_VERSION "1.5.0"

// ==================== CONSTANTS ====================
#define DIST_MIN_VALID_CM          10
#define DIST_MAX_VALID_CM          400
#define MOVEMENT_THRESHOLD_MIN_CM  1
#define MOVEMENT_THRESHOLD_MAX_CM  50
#define PRESENCE_TIMEOUT_MIN_S     5
#define PRESENCE_TIMEOUT_MAX_S     300
#define GATE_MIN                   0
#define GATE_MAX                   15
#define DELAY_MIN_MS               0
#define DELAY_MAX_MS               65535
#define DETECT_LOG_DELTA_CM        5
#define LOOP_STATUS_INTERVAL_ITERS 100   // ~10s at 100ms loop delay
#define RAW_PRESENCE_STALE_US      (2LL * 1000000LL)
#define APPLY_CONFIG_TASK_STACK    4096
#define APPLY_CONFIG_TASK_PRIO     5

#ifndef MQTT_ALLOW_ANONYMOUS_COMMANDS
#define MQTT_ALLOW_ANONYMOUS_COMMANDS 0
#endif

// Pin configuration
#define UART_PORT UART_NUM_1
#define UART_TX_PIN GPIO_NUM_10  // ESP32 TX -> LD2420 RX
#define UART_RX_PIN GPIO_NUM_7   // ESP32 RX <- LD2420 TX
#define OT2_PIN GPIO_NUM_4       // Detection output pin
#define BAUD_RATE 115200

static const char *TAG = "LD2420_PRESENCE";

// ==================== GLOBALS ====================
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;
static SemaphoreHandle_t s_state_mutex = NULL;
static TaskHandle_t s_apply_config_task_handle = NULL;

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// ==================== PRESENCE DETECTION ====================
static int s_movement_threshold_cm = 5;     // Distance change to detect movement
static int s_presence_timeout_sec = 30;     // Hold presence after movement
static int s_distance_history[3] = {-1, -1, -1};
static int s_history_idx = 0;
static bool s_current_presence = false;
static int64_t s_last_presence_time = -1;
static int64_t s_last_raw_presence_time = -1;
static bool s_raw_presence_active = false;
static int s_last_distance = -1;
static ld2420_t* s_sensor = NULL;
static bool s_sensor_ready = false;
static bool s_wifi_connected = false;
static uint8_t s_ip_last_octet = 0;
static char s_ld_fw_version[16] = "?";

// LD2420 tuning (current values)
static int s_ld_min_gate = 0;        // 0..15
static int s_ld_max_gate = 15;       // 0..15
static int s_ld_delay_ms = 0;        // 0..65535
static int s_ld_trigger_sens = 200;  // 0..65535
static int s_ld_maintain_sens = 150; // 0..65535

static bool detect_movement(int distance_cm) {
    // Caller must hold s_state_mutex
    s_distance_history[s_history_idx] = distance_cm;
    s_history_idx = (s_history_idx + 1) % 3;

    // Check for significant change
    int max_dist = distance_cm, min_dist = distance_cm;
    for (int i = 0; i < 3; i++) {
        if (s_distance_history[i] < 0) continue;
        if (s_distance_history[i] > max_dist) max_dist = s_distance_history[i];
        if (s_distance_history[i] < min_dist) min_dist = s_distance_history[i];
    }

    return (max_dist - min_dist) >= s_movement_threshold_cm;
}

static void update_presence_state(bool raw_present, int distance_cm) {
    bool valid_distance = (distance_cm >= DIST_MIN_VALID_CM && distance_cm <= DIST_MAX_VALID_CM);
    int64_t now = esp_timer_get_time();
    bool movement = false;

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_raw_presence_active = raw_present;
    if (raw_present) {
        s_last_raw_presence_time = now;
        s_last_presence_time = now;
    }

    if (valid_distance) {
        movement = detect_movement(distance_cm);
        if (movement) {
            s_last_presence_time = now;
            ESP_LOGI(TAG, "Movement detected at %d cm", distance_cm);
        }
    }

    // Occupancy stays on while the radar reports presence and can still be
    // extended briefly after movement if packets go idle or become noisy.
    bool hold_active = false;
    if (s_last_presence_time >= 0) {
        int64_t time_since_presence = (now - s_last_presence_time) / 1000000LL;
        hold_active = time_since_presence < s_presence_timeout_sec;
    }
    bool presence = (raw_present || movement || hold_active);

    bool should_publish = false;
    int publish_distance_mm = (s_last_distance >= 0) ? s_last_distance * 10 : -1;

    if (valid_distance && (s_last_distance < 0 || abs(distance_cm - s_last_distance) > 3)) {
        s_last_distance = distance_cm;
        publish_distance_mm = distance_cm * 10; // Convert to mm
        should_publish = true;
    }

    if (presence != s_current_presence) {
        ESP_LOGI(TAG, "Presence: %s", presence ? "ON" : "OFF");
        s_current_presence = presence;
        should_publish = true;
    }
    xSemaphoreGive(s_state_mutex);

    if (should_publish) {
        ha_mqtt_publish_presence(presence, publish_distance_mm);
    }
}

// ==================== LD2420 CALLBACKS ====================
// Callback for detection events
void onDetection(uint16_t distance) {
    // Only log significant distance changes to reduce spam
    // Early-out if sensor is not initialized
    if (s_sensor == NULL) return;
    static uint16_t last_distance = 0;
    if (abs(distance - last_distance) > DETECT_LOG_DELTA_CM) {  // Only log if distance changed sufficiently
        ESP_LOGD(TAG, ">>> DETECTION: Target at %d cm", distance);
        last_distance = distance;
    }
    
}

// Callback for state changes
void onStateChange(LD2420_DetectionState oldState, LD2420_DetectionState newState) {
    if (s_sensor == NULL) return;
    if (newState == LD2420_DETECTION_ACTIVE) {
        ESP_LOGI(TAG, "=== MOTION DETECTED ===");
    } else {
        ESP_LOGI(TAG, "=== AREA CLEAR ===");
    }
}

// Callback for data updates (called frequently)
void onDataUpdate(ld2420_data_t data) {
    if (s_sensor == NULL) return;

    update_presence_state(data.state == LD2420_DETECTION_ACTIVE, data.distance);

    // Log every 100th update to reduce spam
    static int counter = 0;
    if (++counter % 100 == 0) {
        if (data.isValid) {
            ESP_LOGD(TAG, "Update: State=%s, Distance=%d cm", 
                     data.state == LD2420_DETECTION_ACTIVE ? "ACTIVE" : "IDLE",
                     data.distance);
        }
    }
}

// ==================== CONFIG FUNCTIONS FOR HA SLIDERS ====================
static int get_movement_threshold(void) {
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    int v = s_movement_threshold_cm;
    xSemaphoreGive(s_state_mutex);
    return v;
}
static void set_movement_threshold(int val) { 
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    if (val < MOVEMENT_THRESHOLD_MIN_CM) val = MOVEMENT_THRESHOLD_MIN_CM;
    if (val > MOVEMENT_THRESHOLD_MAX_CM) val = MOVEMENT_THRESHOLD_MAX_CM;
    s_movement_threshold_cm = val;
    xSemaphoreGive(s_state_mutex);
    ESP_LOGI(TAG, "Movement threshold set to %d cm", s_movement_threshold_cm);
}

static int get_presence_timeout_ms(void) { 
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    int v = s_presence_timeout_sec * 1000;  // Convert to ms
    xSemaphoreGive(s_state_mutex);
    return v;
}
static void set_presence_timeout_ms(int val_ms) { 
    int val_sec = val_ms / 1000;  // Convert from ms
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    if (val_sec < PRESENCE_TIMEOUT_MIN_S) val_sec = PRESENCE_TIMEOUT_MIN_S;
    if (val_sec > PRESENCE_TIMEOUT_MAX_S) val_sec = PRESENCE_TIMEOUT_MAX_S;
    s_presence_timeout_sec = val_sec;
    xSemaphoreGive(s_state_mutex);
    ESP_LOGI(TAG, "Presence timeout set to %d seconds", s_presence_timeout_sec);
}

// LD2420 tuning get/set (exposed to MQTT)
static int  get_ld_min_gate(void)        { xSemaphoreTake(s_state_mutex, portMAX_DELAY); int v=s_ld_min_gate; xSemaphoreGive(s_state_mutex); return v; }
static void set_ld_min_gate(int v)       { xSemaphoreTake(s_state_mutex, portMAX_DELAY); if (v < GATE_MIN) v = GATE_MIN; if (v > GATE_MAX) v = GATE_MAX; s_ld_min_gate = v; if (s_ld_min_gate > s_ld_max_gate) s_ld_max_gate = s_ld_min_gate; xSemaphoreGive(s_state_mutex); ESP_LOGI(TAG, "LD min_gate=%d", s_ld_min_gate); }
static int  get_ld_max_gate(void)        { xSemaphoreTake(s_state_mutex, portMAX_DELAY); int v=s_ld_max_gate; xSemaphoreGive(s_state_mutex); return v; }
static void set_ld_max_gate(int v)       { xSemaphoreTake(s_state_mutex, portMAX_DELAY); if (v < GATE_MIN) v = GATE_MIN; if (v > GATE_MAX) v = GATE_MAX; s_ld_max_gate = v; if (s_ld_max_gate < s_ld_min_gate) s_ld_min_gate = s_ld_max_gate; xSemaphoreGive(s_state_mutex); ESP_LOGI(TAG, "LD max_gate=%d", s_ld_max_gate); }
static int  get_ld_delay_ms(void)        { xSemaphoreTake(s_state_mutex, portMAX_DELAY); int v=s_ld_delay_ms; xSemaphoreGive(s_state_mutex); return v; }
static void set_ld_delay_ms(int v)       { xSemaphoreTake(s_state_mutex, portMAX_DELAY); if (v < DELAY_MIN_MS) v = DELAY_MIN_MS; if (v > DELAY_MAX_MS) v = DELAY_MAX_MS; s_ld_delay_ms = v; xSemaphoreGive(s_state_mutex); ESP_LOGI(TAG, "LD delay_ms=%d", s_ld_delay_ms); }
static int  get_ld_trigger_sens(void)    { xSemaphoreTake(s_state_mutex, portMAX_DELAY); int v=s_ld_trigger_sens; xSemaphoreGive(s_state_mutex); return v; }
static void set_ld_trigger_sens(int v)   { xSemaphoreTake(s_state_mutex, portMAX_DELAY); if (v < 0) v = 0; if (v > 65535) v = 65535; s_ld_trigger_sens = v; xSemaphoreGive(s_state_mutex); ESP_LOGI(TAG, "LD trigger_sens=%d", s_ld_trigger_sens); }
static int  get_ld_maintain_sens(void)   { xSemaphoreTake(s_state_mutex, portMAX_DELAY); int v=s_ld_maintain_sens; xSemaphoreGive(s_state_mutex); return v; }
static void set_ld_maintain_sens(int v)  { xSemaphoreTake(s_state_mutex, portMAX_DELAY); if (v < 0) v = 0; if (v > 65535) v = 65535; s_ld_maintain_sens = v; xSemaphoreGive(s_state_mutex); ESP_LOGI(TAG, "LD maintain_sens=%d", s_ld_maintain_sens); }

static void update_ld_state_from_snapshot(const ld2420_config_snapshot_t *snapshot) {
    if (!snapshot) return;

    int min_gate = snapshot->min_gate;
    int max_gate = snapshot->max_gate;
    int delay_ms = snapshot->delay_ms;
    int trig0_local = (snapshot->trigger_sensitivity > 65535U) ? 65535 : (int)snapshot->trigger_sensitivity;
    int hold0_local = (snapshot->maintain_sensitivity > 65535U) ? 65535 : (int)snapshot->maintain_sensitivity;

    if (min_gate < GATE_MIN) min_gate = GATE_MIN;
    if (min_gate > GATE_MAX) min_gate = GATE_MAX;
    if (max_gate < GATE_MIN) max_gate = GATE_MIN;
    if (max_gate > GATE_MAX) max_gate = GATE_MAX;
    if (min_gate > max_gate) max_gate = min_gate;
    if (delay_ms < DELAY_MIN_MS) delay_ms = DELAY_MIN_MS;
    if (delay_ms > DELAY_MAX_MS) delay_ms = DELAY_MAX_MS;

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_ld_min_gate = min_gate;
    s_ld_max_gate = max_gate;
    s_ld_delay_ms = delay_ms;
    s_ld_trigger_sens = trig0_local;
    s_ld_maintain_sens = hold0_local;
    xSemaphoreGive(s_state_mutex);
}

static esp_err_t sync_ld_config_from_sensor(void) {
    if (!s_sensor) return ESP_ERR_INVALID_STATE;

    ld2420_config_snapshot_t snapshot = {0};
    esp_err_t err = ld2420_read_config(s_sensor, &snapshot);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Unable to sync LD2420 config from sensor (%s)", esp_err_to_name(err));
        return err;
    }

    update_ld_state_from_snapshot(&snapshot);
    ESP_LOGI(TAG, "Synced LD2420 config: min_gate=%d max_gate=%d delay_ms=%d trig0=%" PRIu32 " maintain0=%" PRIu32,
             snapshot.min_gate, snapshot.max_gate, snapshot.delay_ms,
             snapshot.trigger_sensitivity, snapshot.maintain_sensitivity);
    return ESP_OK;
}

static void apply_ld_config(void) {
    if (!s_sensor) return;

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    int min_gate = s_ld_min_gate;
    int max_gate = s_ld_max_gate;
    int delay_ms = s_ld_delay_ms;
    int trig0    = s_ld_trigger_sens;
    int hold0    = s_ld_maintain_sens;

    if (min_gate < GATE_MIN) min_gate = GATE_MIN;
    if (min_gate > GATE_MAX) min_gate = GATE_MAX;
    if (max_gate < GATE_MIN) max_gate = GATE_MIN;
    if (max_gate > GATE_MAX) max_gate = GATE_MAX;
    if (min_gate > max_gate) max_gate = min_gate;
    if (delay_ms < DELAY_MIN_MS) delay_ms = DELAY_MIN_MS;
    if (delay_ms > DELAY_MAX_MS) delay_ms = DELAY_MAX_MS;

    int trig0_local = (trig0 < 0) ? 0 : (trig0 > 65535 ? 65535 : trig0);
    int hold0_local = (hold0 < 0) ? 0 : (hold0 > 65535 ? 65535 : hold0);

    s_ld_min_gate = min_gate;
    s_ld_max_gate = max_gate;
    s_ld_delay_ms = delay_ms;
    s_ld_trigger_sens = trig0_local;
    s_ld_maintain_sens = hold0_local;
    xSemaphoreGive(s_state_mutex);

    ESP_LOGI(TAG, "Applying LD2420 config: min_gate=%d max_gate=%d delay_ms=%d trig0=%d maintain0=%d",
             min_gate, max_gate, delay_ms, trig0_local, hold0_local);

    if (!ld2420_lock(s_sensor, pdMS_TO_TICKS(500))) {
        ESP_LOGW(TAG, "Unable to acquire LD2420 bus for config apply");
        return;
    }

    esp_err_t err = ld2420_enter_command_mode(s_sensor);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to enter command mode (%s)", esp_err_to_name(err));
        ld2420_unlock(s_sensor);
        return;
    }

    bool write_ok = true;
    if (ld2420_set_gate_range(s_sensor, min_gate, max_gate) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set gate range");
        write_ok = false;
    }
    if (ld2420_set_delay_ms(s_sensor, delay_ms) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set delay");
        write_ok = false;
    }
    if (ld2420_set_trigger_sens(s_sensor, 0, (uint32_t)trig0_local) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set trigger sensitivity");
        write_ok = false;
    }
    if (ld2420_set_maintain_sens(s_sensor, 0, (uint32_t)hold0_local) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set maintain sensitivity");
        write_ok = false;
    }

    esp_err_t exit_err = ld2420_exit_command_mode(s_sensor);
    if (exit_err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to exit command mode (%s)", esp_err_to_name(exit_err));
        write_ok = false;
    }

    ld2420_config_snapshot_t applied = {0};
    esp_err_t read_err = ld2420_read_config(s_sensor, &applied);
    ld2420_unlock(s_sensor);

    if (read_err != ESP_OK) {
        ESP_LOGW(TAG, "Unable to read back LD2420 config (%s)", esp_err_to_name(read_err));
        return;
    }

    ESP_LOGI(TAG, "LD2420 reported config: min_gate=%d max_gate=%d delay_ms=%d trig0=%" PRIu32 " maintain0=%" PRIu32,
             applied.min_gate, applied.max_gate, applied.delay_ms,
             applied.trigger_sensitivity, applied.maintain_sensitivity);

    bool mismatch = (applied.min_gate != min_gate) ||
                    (applied.max_gate != max_gate) ||
                    (applied.delay_ms != delay_ms) ||
                    ((int)applied.trigger_sensitivity != trig0_local) ||
                    ((int)applied.maintain_sensitivity != hold0_local);

    update_ld_state_from_snapshot(&applied);

    if (!write_ok) {
        ESP_LOGW(TAG, "One or more LD2420 config writes reported errors");
    }
    if (mismatch) {
        ESP_LOGW(TAG, "LD2420 applied values differ from requested");
    } else if (write_ok) {
        ESP_LOGI(TAG, "LD2420 configuration verified");
    }
}

static void apply_ld_config_task(void *arg) {
    (void)arg;

    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        apply_ld_config();
    }
}

static void request_apply_ld_config(void) {
    if (s_apply_config_task_handle == NULL) {
        ESP_LOGW(TAG, "Apply config requested before worker task is ready");
        return;
    }

    xTaskNotifyGive(s_apply_config_task_handle);
}

static void collect_oled_snapshot(oled_status_snapshot_t *out_snapshot) {
    if (out_snapshot == NULL || s_state_mutex == NULL) {
        return;
    }

    ld2420_t *sensor = s_sensor;
    ld2420_data_t sensor_data = sensor ? ld2420_get_current_data(sensor) : (ld2420_data_t){0};

    memset(out_snapshot, 0, sizeof(*out_snapshot));
    out_snapshot->distance_cm = -1;
    out_snapshot->rssi_dbm = 0;

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    out_snapshot->sensor_ready = s_sensor_ready;
    out_snapshot->sensor_packets_valid = sensor_data.isValid;
    out_snapshot->presence = s_current_presence;
    out_snapshot->wifi_connected = s_wifi_connected;
    out_snapshot->ip_last_octet = s_ip_last_octet;
    out_snapshot->distance_cm = s_last_distance;
    out_snapshot->min_gate = s_ld_min_gate;
    out_snapshot->max_gate = s_ld_max_gate;
    out_snapshot->delay_ms = s_ld_delay_ms;
    out_snapshot->trigger_sens = s_ld_trigger_sens;
    out_snapshot->maintain_sens = s_ld_maintain_sens;
    snprintf(out_snapshot->fw_version, sizeof(out_snapshot->fw_version), "%s", s_ld_fw_version);
    xSemaphoreGive(s_state_mutex);

    out_snapshot->mqtt_connected = ha_mqtt_is_connected();

    if (out_snapshot->wifi_connected) {
        wifi_ap_record_t ap_info = {0};
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            out_snapshot->rssi_dbm = ap_info.rssi;
        }
    }
}

// ==================== MQTT SETUP ====================
static void start_mqtt(void) {
    // Guard: only initialise once. On subsequent WiFi reconnects the MQTT
    // client handles reconnection internally; we just kick it explicitly in
    // case its backoff timer has stalled.
    static bool s_mqtt_initialized = false;
    if (s_mqtt_initialized) {
        ha_mqtt_reconnect_if_disconnected();
        return;
    }

    char uri[96];
#ifdef MQTT_BROKER_CA_CERT_PEM
    const char *ca_pem = MQTT_BROKER_CA_CERT_PEM;
#else
    const char *ca_pem = NULL;
#endif
    const char *scheme = (ca_pem != NULL) ? "mqtts" : "mqtt";
    snprintf(uri, sizeof(uri), "%s://%s:%d", scheme, MQTT_BROKER_HOST, MQTT_BROKER_PORT);

    bool mqtt_credentials_configured = (MQTT_USERNAME[0] != '\0');
    bool command_topics_enabled = mqtt_credentials_configured || MQTT_ALLOW_ANONYMOUS_COMMANDS;
    if (!command_topics_enabled) {
        ESP_LOGW(TAG, "MQTT command topics disabled: configure MQTT_USERNAME or set MQTT_ALLOW_ANONYMOUS_COMMANDS=1");
    } else if (ca_pem == NULL) {
        ESP_LOGW(TAG, "MQTT command topics enabled without TLS; rely on trusted LAN and broker ACLs");
    }

    ha_mqtt_cfg_t cfg = {
        .broker_uri = uri,
        .username = MQTT_USERNAME[0] ? MQTT_USERNAME : NULL,
        .password = MQTT_PASSWORD[0] ? MQTT_PASSWORD : NULL,
        .friendly_name = DEVICE_NAME,
        .suggested_area = DEVICE_LOCATION,
        .app_version = DEVICE_VERSION,
        .discovery_prefix = HA_DISCOVERY_PREFIX,
        .distance_supported = true,
        .broker_ca_cert_pem = ca_pem,
        .command_topics_enabled = command_topics_enabled,
        .get_distance_thresh_cm = get_movement_threshold,
        .set_distance_thresh_cm = set_movement_threshold,
        .get_hold_on_ms = get_presence_timeout_ms,
        .set_hold_on_ms = set_presence_timeout_ms,
        // LD2420 tuning exposure
        .get_ld_min_gate = get_ld_min_gate,
        .set_ld_min_gate = set_ld_min_gate,
        .get_ld_max_gate = get_ld_max_gate,
        .set_ld_max_gate = set_ld_max_gate,
        .get_ld_delay_ms = get_ld_delay_ms,
        .set_ld_delay_ms = set_ld_delay_ms,
        .get_ld_trigger_sens = get_ld_trigger_sens,
        .set_ld_trigger_sens = set_ld_trigger_sens,
        .get_ld_maintain_sens = get_ld_maintain_sens,
        .set_ld_maintain_sens = set_ld_maintain_sens,
        .action_apply_config = request_apply_ld_config,
    };

    ha_mqtt_init(&cfg);
    ha_mqtt_start();
    s_mqtt_initialized = true;
}

// ==================== WIFI ====================
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_state_mutex != NULL) {
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            s_wifi_connected = false;
            s_ip_last_octet = 0;
            xSemaphoreGive(s_state_mutex);
        }
        if (s_wifi_event_group != NULL) {
            xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        }
        esp_wifi_connect();
        s_retry_num++;
        ESP_LOGW(TAG, "WiFi disconnected, retry #%d", s_retry_num);
        if (s_retry_num == 5) {
            // Allow the main task to continue after initial failures while retries persist in background
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        if (event_data == NULL) return;
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
        if (s_state_mutex != NULL) {
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            s_wifi_connected = true;
            s_ip_last_octet = (uint8_t)esp_ip4_addr4(&event->ip_info.ip);
            xSemaphoreGive(s_state_mutex);
        }
        ESP_LOGI(TAG, "WiFi connected: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
        start_mqtt();
    }
}

static esp_err_t wifi_init(void) {
    s_wifi_event_group = xEventGroupCreate();
    
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL, &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    return (bits & WIFI_CONNECTED_BIT) ? ESP_OK : ESP_FAIL;
}

// ==================== MAIN ====================
void app_main(void) {
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_log_level_set("LD2420_LIB", ESP_LOG_INFO);
    // Create state mutex early
    s_state_mutex = xSemaphoreCreateMutex();
    if (s_state_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create state mutex");
        return; // Avoid running without synchronization primitives
    }
    
    ESP_LOGI(TAG, "=====================================");
    ESP_LOGI(TAG, "LD2420 24GHz Radar Sensor with MQTT");
    ESP_LOGI(TAG, "Based on ESPHome implementation");
    ESP_LOGI(TAG, "=====================================");
    ESP_LOGI(TAG, "Hardware Configuration:");
    ESP_LOGI(TAG, "  UART%d: TX=GPIO%d, RX=GPIO%d", 
             UART_PORT, UART_TX_PIN, UART_RX_PIN);
    ESP_LOGI(TAG, "  OT2 Pin: GPIO%d", OT2_PIN);
    ESP_LOGI(TAG, "  Baud Rate: %d", BAUD_RATE);
    ESP_LOGI(TAG, "  Power: 3.3V");
    ESP_LOGI(TAG, "-------------------------------------");

    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    if (!oled_status_init(collect_oled_snapshot, DEVICE_VERSION)) {
        ESP_LOGW(TAG, "OLED status display init failed");
    }

    // LD2420 INITIALIZATION
    s_sensor = ld2420_create();
    if (s_sensor == NULL) {
        ESP_LOGE(TAG, "Failed to create sensor instance!");
        return;
    }

    ret = ld2420_begin_with_ot2(s_sensor, UART_PORT, UART_TX_PIN, 
                                UART_RX_PIN, OT2_PIN, BAUD_RATE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize sensor!");
        ESP_LOGE(TAG, "Check connections:");
        ESP_LOGE(TAG, "  LD2420 TX -> ESP32 GPIO%d (RX)", UART_RX_PIN);
        ESP_LOGE(TAG, "  LD2420 RX -> ESP32 GPIO%d (TX)", UART_TX_PIN);
        ESP_LOGE(TAG, "  LD2420 OT2 -> ESP32 GPIO%d", OT2_PIN);
        ESP_LOGE(TAG, "  LD2420 VCC -> 3.3V");
        ESP_LOGE(TAG, "  LD2420 GND -> GND");
        return;
    }

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_sensor_ready = true;
    xSemaphoreGive(s_state_mutex);
    ESP_LOGI(TAG, "Sensor initialized successfully");
    sync_ld_config_from_sensor();

    // Read firmware version and publish (diagnostic)
    {
        char fw[48];
        if (ld2420_read_firmware_version(s_sensor, fw, sizeof(fw)) == ESP_OK) {
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            snprintf(s_ld_fw_version, sizeof(s_ld_fw_version), "%.15s", fw);
            xSemaphoreGive(s_state_mutex);
            ESP_LOGI(TAG, "LD2420 FW: %s", fw);
            ha_mqtt_publish_ld2420_fw_version(fw);
        } else {
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            snprintf(s_ld_fw_version, sizeof(s_ld_fw_version), "%s", "?");
            xSemaphoreGive(s_state_mutex);
            ESP_LOGW(TAG, "Unable to read LD2420 firmware version");
        }
    }
    
    // Register callbacks
    ld2420_on_detection(s_sensor, onDetection);
    ld2420_on_state_change(s_sensor, onStateChange);
    ld2420_on_data_update(s_sensor, onDataUpdate);

    if (xTaskCreate(apply_ld_config_task, "ld_apply_cfg",
                    APPLY_CONFIG_TASK_STACK, NULL,
                    APPLY_CONFIG_TASK_PRIO,
                    &s_apply_config_task_handle) != pdPASS) {
        ESP_LOGW(TAG, "Failed to create LD2420 apply-config worker");
        s_apply_config_task_handle = NULL;
    }
    
    ESP_LOGI(TAG, "Callbacks registered");
    ESP_LOGI(TAG, "-------------------------------------");
    ESP_LOGI(TAG, "Expected Energy Mode packet format:");
    ESP_LOGI(TAG, "  Header: F4 F3 F2 F1");
    ESP_LOGI(TAG, "  Length: 2 bytes");
    ESP_LOGI(TAG, "  Data: Presence(1) + Distance(2) + Energy(32)");
    ESP_LOGI(TAG, "  Footer: F8 F7 F6 F5");
    ESP_LOGI(TAG, "=====================================");
    ESP_LOGI(TAG, "Starting detection loop...");
    ESP_LOGI(TAG, "Movement threshold: %d cm, Presence timeout: %d sec", 
             s_movement_threshold_cm, s_presence_timeout_sec);
    ESP_LOGI(TAG, "");

    // Initialize WiFi and MQTT after the sensor state is fully synchronized so
    // Home Assistant sees the actual LD2420 config on first connect.
    ESP_LOGI(TAG, "Starting WiFi...");
    esp_err_t wifi_rc = wifi_init();
    if (wifi_rc != ESP_OK) {
        ESP_LOGW(TAG, "wifi_init returned %s; continuing, background retries may proceed", esp_err_to_name(wifi_rc));
    }
    
    // MAIN LOOP WITH MQTT ADDITIONS
    bool last_ot2_state = false;
    int no_packet_counter = 0;
    
    while (1) {
        // Update sensor (checks for new UART data)
        ld2420_update(s_sensor);
        
        // Check OT2 pin for simple detection (backup method)
        bool ot2_state = ld2420_check_ot2(OT2_PIN);
        if (ot2_state != last_ot2_state) {
            last_ot2_state = ot2_state;
            ESP_LOGI(TAG, "[OT2 Pin] %s", ot2_state ? "MOTION" : "CLEAR");
        }
        
        // PERIODIC STATUS CHECK
        static int loop_counter = 0;
        if (++loop_counter >= LOOP_STATUS_INTERVAL_ITERS) {  // Every ~10 seconds
            loop_counter = 0;

            ld2420_data_t sensor_data = ld2420_get_current_data(s_sensor);
            bool presence_snapshot = false;
            xSemaphoreTake(s_state_mutex, portMAX_DELAY);
            presence_snapshot = s_current_presence;
            xSemaphoreGive(s_state_mutex);

            if (sensor_data.isValid) {
                // We're getting valid packets
                ESP_LOGD(TAG, "Status: %s | Distance: %d cm | OT2: %s | MQTT Presence: %s",
                         sensor_data.state == LD2420_DETECTION_ACTIVE ? "DETECTING" : "IDLE",
                         sensor_data.distance,
                         ot2_state ? "HIGH" : "LOW",
                         presence_snapshot ? "ON" : "OFF");
                no_packet_counter = 0;
            } else {
                // No valid packets yet
                no_packet_counter++;
                ESP_LOGW(TAG, "No valid Energy packets (attempt %d) | OT2: %s", 
                         no_packet_counter, ot2_state ? "HIGH" : "LOW");
                
                if (no_packet_counter == 3) {
                    ESP_LOGW(TAG, "Troubleshooting:");
                    ESP_LOGW(TAG, "  1. Power cycle the sensor");
                    ESP_LOGW(TAG, "  2. Check if TX/RX are swapped");
                    ESP_LOGW(TAG, "  3. OT2 pin %s working for basic detection",
                             ot2_state ? "IS" : "might be");
                }
            }
        }

        // Timeout-based clear: ensure presence clears even if no valid packets arrive
        int64_t now = esp_timer_get_time();
        bool do_clear = false;
        int last_distance_local = -1;
        xSemaphoreTake(s_state_mutex, portMAX_DELAY);
        if (s_current_presence) {
            bool raw_presence_recent = s_raw_presence_active &&
                                       s_last_raw_presence_time >= 0 &&
                                       (now - s_last_raw_presence_time) <= RAW_PRESENCE_STALE_US;
            int64_t elapsed = (s_last_presence_time >= 0)
                                  ? (now - s_last_presence_time) / 1000000LL
                                  : INT64_MAX;
            if (!raw_presence_recent && elapsed >= s_presence_timeout_sec) {
                s_current_presence = false;
                last_distance_local = s_last_distance;
                do_clear = true;
            }
        }
        xSemaphoreGive(s_state_mutex);
        if (do_clear) {
            ESP_LOGI(TAG, "Presence timeout elapsed -> CLEAR");
            ha_mqtt_publish_presence(false, last_distance_local >= 0 ? last_distance_local * 10 : -1);
        }

        // Small delay to prevent watchdog
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
