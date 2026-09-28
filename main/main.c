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
#include "nvs.h"
#include "esp_err.h"
#include "esp_app_desc.h"
#include "esp_netif_sntp.h"
#include <time.h>

#include "ld2420.h"  // LD2420 library
#include "ha_mqtt.h"
#include "oled_status.h"
#include "ota_update.h"
#include "device_creds.h"
#include "../config/secrets.h"

// Wi-Fi/MQTT credentials now come from the "creds" NVS partition
// (tools/provision.ps1). Values left in secrets.h are ignored.
#if defined(WIFI_SSID) || defined(WIFI_PASSWORD) || defined(MQTT_USERNAME) || defined(MQTT_PASSWORD)
#warning "WIFI_*/MQTT_USERNAME/MQTT_PASSWORD in secrets.h are ignored; move them to config/creds.csv and remove them"
#endif

#define DEVICE_VERSION (esp_app_get_description()->version)  // PROJECT_VER in CMakeLists.txt

// ==================== CONSTANTS ====================
#define DIST_MIN_VALID_CM          10
#define DIST_MAX_VALID_CM          400
#define MOVEMENT_THRESHOLD_MIN_CM  1
#define MOVEMENT_THRESHOLD_MAX_CM  50
#define PRESENCE_TIMEOUT_MIN_S     5
#define PRESENCE_TIMEOUT_MAX_S     300
#define GATE_MIN                   0
#define GATE_MAX                   15
#define RADAR_HOLD_MIN_S               0
#define RADAR_HOLD_MAX_S           3600    // LD2420 "delay time" register is in seconds
#define DETECT_LOG_DELTA_CM        5
#define LOOP_STATUS_INTERVAL_ITERS 100   // ~10s at 100ms loop delay
// A freshly installed OTA image must reach MQTT and see valid radar frames
// within this window, otherwise the bootloader falls back to the old image.
#define OTA_ROLLBACK_TIMEOUT_S     300
#define RAW_PRESENCE_STALE_US      (2LL * 1000000LL)
#define MOVEMENT_LOG_INTERVAL_US   (2LL * 1000000LL)
#define APPLY_CONFIG_TASK_STACK    4096
#define APPLY_CONFIG_TASK_PRIO     3
// Radar settings changed from HA are written once they stop changing.
#define APPLY_DEBOUNCE_US          (2LL * 1000000LL)
// Radar watchdog: no valid frame for this long means the radar has stopped
// streaming (e.g. stuck in command mode after a brown-out); restart it.
#define RADAR_SILENT_US            (30LL * 1000000LL)
#define RADAR_RETRY_US             (120LL * 1000000LL)
// Any wall-clock time before this means SNTP has not synced yet.
#define MIN_VALID_EPOCH_S          1700000000LL

// LD2420 factory energy thresholds per gate (as used by ESPHome's ld2420
// component). Sensitivity presets scale them: a lower threshold means a
// weaker reflection already counts, i.e. more sensitive.
static const uint32_t FACTORY_TRIGGER_THRESH[LD2420_GATE_COUNT] = {
    60000, 30000, 400, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250, 250};
static const uint32_t FACTORY_MAINTAIN_THRESH[LD2420_GATE_COUNT] = {
    40000, 20000, 200, 200, 200, 200, 200, 150, 150, 100, 100, 100, 100, 100, 100, 100};
// Threshold scale in percent for HA_MQTT_SENSITIVITY_LOW / MEDIUM / HIGH.
static const uint32_t SENSITIVITY_SCALE_PCT[3] = {160, 100, 60};

#ifndef MQTT_ALLOW_ANONYMOUS_COMMANDS
#define MQTT_ALLOW_ANONYMOUS_COMMANDS 0
#endif

// Allow MQTT command topics when the broker is plaintext (no CA configured).
// Default is 0: commands require TLS so a misconfigured deployment cannot
// expose restart / apply-config to broker-side ACL bypasses or LAN sniffers.
// Set to 1 in secrets.h only if you have a hardened LAN where broker ACLs
// + plaintext are an explicit choice.
#ifndef MQTT_ALLOW_INSECURE_COMMANDS
#define MQTT_ALLOW_INSECURE_COMMANDS 0
#endif

// NVS keys for app-level tunables that should survive reboots. The LD2420
// vendor params (gates, delay, sensitivities) live in the radar's own NVRAM
// and are read back via sync_ld_config_from_sensor(), so they are not stored
// here.
#define APP_NVS_NAMESPACE        "ld2420_app"
#define NVS_KEY_MOVEMENT_THRESH  "mv_thresh"
#define NVS_KEY_PRESENCE_TIMEOUT "pres_to_s"

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
static esp_timer_handle_t s_apply_debounce_timer = NULL;
static void schedule_apply_ld_config(void);

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
static int64_t s_last_movement_log_time = -1;
static bool s_raw_presence_active = false;
static int s_last_distance = -1;
static ld2420_t* s_sensor = NULL;
static bool s_sensor_ready = false;
static bool s_wifi_connected = false;
static uint8_t s_ip_last_octet = 0;
static char s_ld_fw_version[16] = "?";
static device_creds_t s_creds;          // loaded once at boot, read-only after
static bool s_creds_ok = false;

// LD2420 tuning (current values)
static int s_ld_min_gate = 0;        // 0..15
static int s_ld_max_gate = 15;       // 0..15
static int s_ld_delay_s = 0;        // 0..65535
static int s_ld_trigger_sens = 200;  // 0..65535
static int s_ld_maintain_sens = 150; // 0..65535
static int s_ld_sensitivity = -1;         // HA_MQTT_SENSITIVITY_*, -1 = custom/unknown
static bool s_ld_sensitivity_pending = false; // preset chosen in HA, not yet written
static bool s_ld_config_synced = false;  // radar settings read successfully
static int64_t s_last_radar_frame_us = 0; // main task only

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
            if (s_last_movement_log_time < 0 ||
                (now - s_last_movement_log_time) >= MOVEMENT_LOG_INTERVAL_US) {
                s_last_movement_log_time = now;
                ESP_LOGI(TAG, "Movement detected at %d cm", distance_cm);
            }
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
    if (data.isValid) s_last_radar_frame_us = esp_timer_get_time();

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

// ==================== NVS PERSISTENCE ====================
static void app_config_save_i32(const char *key, int32_t value) {
    nvs_handle_t h;
    esp_err_t err = nvs_open(APP_NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS open for write failed: %s", esp_err_to_name(err));
        return;
    }
    int32_t existing;
    if (nvs_get_i32(h, key, &existing) == ESP_OK && existing == value) {
        nvs_close(h);
        return;
    }
    err = nvs_set_i32(h, key, value);
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "NVS save %s failed: %s", key, esp_err_to_name(err));
    }
}

static bool app_setting_load(const char *key, int *out) {
    nvs_handle_t h;
    if (nvs_open(APP_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) return false;
    int32_t v;
    bool ok = nvs_get_i32(h, key, &v) == ESP_OK;
    nvs_close(h);
    if (ok) *out = (int)v;
    return ok;
}

static void app_setting_save(const char *key, int value) {
    app_config_save_i32(key, (int32_t)value);
}

// Restore persisted tunables. Called once at boot, after nvs_flash_init.
// Out-of-range values are ignored so a corrupted entry can't push the device
// outside its operating envelope.
static void app_config_load(void) {
    nvs_handle_t h;
    if (nvs_open(APP_NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK) {
        return;
    }
    int32_t v;
    if (nvs_get_i32(h, NVS_KEY_MOVEMENT_THRESH, &v) == ESP_OK &&
        v >= MOVEMENT_THRESHOLD_MIN_CM && v <= MOVEMENT_THRESHOLD_MAX_CM) {
        s_movement_threshold_cm = (int)v;
        ESP_LOGI(TAG, "NVS restored movement_threshold = %d cm", s_movement_threshold_cm);
    }
    if (nvs_get_i32(h, NVS_KEY_PRESENCE_TIMEOUT, &v) == ESP_OK &&
        v >= PRESENCE_TIMEOUT_MIN_S && v <= PRESENCE_TIMEOUT_MAX_S) {
        s_presence_timeout_sec = (int)v;
        ESP_LOGI(TAG, "NVS restored presence_timeout = %d s", s_presence_timeout_sec);
    }
    nvs_close(h);
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
    int saved = s_movement_threshold_cm;
    xSemaphoreGive(s_state_mutex);
    ESP_LOGI(TAG, "Movement threshold set to %d cm", saved);
    app_config_save_i32(NVS_KEY_MOVEMENT_THRESH, saved);
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
    int saved = s_presence_timeout_sec;
    xSemaphoreGive(s_state_mutex);
    ESP_LOGI(TAG, "Presence timeout set to %d seconds", saved);
    app_config_save_i32(NVS_KEY_PRESENCE_TIMEOUT, saved);
}

// LD2420 tuning get/set (exposed to MQTT)
static int  get_ld_min_gate(void)        { xSemaphoreTake(s_state_mutex, portMAX_DELAY); int v=s_ld_min_gate; xSemaphoreGive(s_state_mutex); return v; }
static void set_ld_min_gate(int v)       { xSemaphoreTake(s_state_mutex, portMAX_DELAY); if (v < GATE_MIN) v = GATE_MIN; if (v > GATE_MAX) v = GATE_MAX; s_ld_min_gate = v; if (s_ld_min_gate > s_ld_max_gate) s_ld_max_gate = s_ld_min_gate; xSemaphoreGive(s_state_mutex); ESP_LOGI(TAG, "LD min_gate=%d", s_ld_min_gate); schedule_apply_ld_config(); }
static int  get_ld_max_gate(void)        { xSemaphoreTake(s_state_mutex, portMAX_DELAY); int v=s_ld_max_gate; xSemaphoreGive(s_state_mutex); return v; }
static void set_ld_max_gate(int v)       { xSemaphoreTake(s_state_mutex, portMAX_DELAY); if (v < GATE_MIN) v = GATE_MIN; if (v > GATE_MAX) v = GATE_MAX; s_ld_max_gate = v; if (s_ld_max_gate < s_ld_min_gate) s_ld_min_gate = s_ld_max_gate; xSemaphoreGive(s_state_mutex); ESP_LOGI(TAG, "LD max_gate=%d", s_ld_max_gate); schedule_apply_ld_config(); }
static int  get_ld_delay_s(void)        { xSemaphoreTake(s_state_mutex, portMAX_DELAY); int v=s_ld_delay_s; xSemaphoreGive(s_state_mutex); return v; }
static void set_ld_delay_s(int v)       { xSemaphoreTake(s_state_mutex, portMAX_DELAY); if (v < RADAR_HOLD_MIN_S) v = RADAR_HOLD_MIN_S; if (v > RADAR_HOLD_MAX_S) v = RADAR_HOLD_MAX_S; s_ld_delay_s = v; xSemaphoreGive(s_state_mutex); ESP_LOGI(TAG, "LD delay_s=%d", s_ld_delay_s); schedule_apply_ld_config(); }

static uint32_t preset_threshold(int level, const uint32_t *table, int gate) {
    uint32_t v = table[gate] * SENSITIVITY_SCALE_PCT[level] / 100U;
    return v > 65535U ? 65535U : v;
}

// Which preset (if any) exactly matches the radar's threshold table.
static int classify_sensitivity(const uint32_t trig[LD2420_GATE_COUNT],
                                const uint32_t maint[LD2420_GATE_COUNT]) {
    for (int level = HA_MQTT_SENSITIVITY_LOW; level <= HA_MQTT_SENSITIVITY_HIGH; ++level) {
        bool match = true;
        for (int g = 0; g < LD2420_GATE_COUNT && match; ++g) {
            match = trig[g] == preset_threshold(level, FACTORY_TRIGGER_THRESH, g) &&
                    maint[g] == preset_threshold(level, FACTORY_MAINTAIN_THRESH, g);
        }
        if (match) return level;
    }
    return -1;
}

static void schedule_apply_ld_config(void) {
    if (s_apply_debounce_timer == NULL) return;
    esp_timer_stop(s_apply_debounce_timer);  // restart the quiet period
    esp_timer_start_once(s_apply_debounce_timer, APPLY_DEBOUNCE_US);
}

static int  get_sensitivity(void) { xSemaphoreTake(s_state_mutex, portMAX_DELAY); int v = s_ld_sensitivity; xSemaphoreGive(s_state_mutex); return v; }
static void set_sensitivity(int level) {
    if (level < HA_MQTT_SENSITIVITY_LOW || level > HA_MQTT_SENSITIVITY_HIGH) return;
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_ld_sensitivity = level;
    s_ld_sensitivity_pending = true;
    xSemaphoreGive(s_state_mutex);
    ESP_LOGI(TAG, "LD sensitivity preset=%d", level);
    schedule_apply_ld_config();
}

static void update_ld_state_from_snapshot(const ld2420_config_snapshot_t *snapshot) {
    if (!snapshot) return;

    int min_gate = snapshot->min_gate;
    int max_gate = snapshot->max_gate;
    int delay_s = snapshot->delay_s;
    int trig0_local = (snapshot->trigger_sensitivity > 65535U) ? 65535 : (int)snapshot->trigger_sensitivity;
    int hold0_local = (snapshot->maintain_sensitivity > 65535U) ? 65535 : (int)snapshot->maintain_sensitivity;

    if (min_gate < GATE_MIN) min_gate = GATE_MIN;
    if (min_gate > GATE_MAX) min_gate = GATE_MAX;
    if (max_gate < GATE_MIN) max_gate = GATE_MIN;
    if (max_gate > GATE_MAX) max_gate = GATE_MAX;
    if (min_gate > max_gate) max_gate = min_gate;
    if (delay_s < RADAR_HOLD_MIN_S) delay_s = RADAR_HOLD_MIN_S;
    if (delay_s > RADAR_HOLD_MAX_S) delay_s = RADAR_HOLD_MAX_S;

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_ld_min_gate = min_gate;
    s_ld_max_gate = max_gate;
    s_ld_delay_s = delay_s;
    s_ld_trigger_sens = trig0_local;
    s_ld_maintain_sens = hold0_local;
    xSemaphoreGive(s_state_mutex);
}

static void sync_ld_sensitivity_from_sensor(void) {
    uint32_t trig[LD2420_GATE_COUNT] = {0};
    uint32_t maint[LD2420_GATE_COUNT] = {0};
    esp_err_t err = ld2420_read_thresholds(s_sensor, trig, maint);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Unable to read LD2420 gate thresholds (%s)", esp_err_to_name(err));
        return;
    }
    int level = classify_sensitivity(trig, maint);
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_ld_sensitivity = level;
    xSemaphoreGive(s_state_mutex);
    ESP_LOGI(TAG, "LD2420 thresholds (move/still per gate), preset=%d:", level);
    for (int g = 0; g < LD2420_GATE_COUNT; ++g) {
        ESP_LOGI(TAG, "  gate %2d (%3d-%3d cm): %5" PRIu32 " / %5" PRIu32,
                 g, g * 70, (g + 1) * 70, trig[g], maint[g]);
    }
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
    sync_ld_sensitivity_from_sensor();
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    s_ld_config_synced = true;
    xSemaphoreGive(s_state_mutex);
    ESP_LOGI(TAG, "Synced LD2420 config: min_gate=%d max_gate=%d delay_s=%d trig0=%" PRIu32 " maintain0=%" PRIu32,
             snapshot.min_gate, snapshot.max_gate, snapshot.delay_s,
             snapshot.trigger_sensitivity, snapshot.maintain_sensitivity);
    return ESP_OK;
}

static bool ld_config_valid(void) {
    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    bool v = s_ld_config_synced;
    xSemaphoreGive(s_state_mutex);
    return v;
}

static void apply_ld_config(void) {
    if (!s_sensor) return;
    if (!ld_config_valid()) {
        // Writing now would push firmware defaults over the radar's settings.
        ESP_LOGW(TAG, "Skipping LD2420 apply: radar settings not read yet");
        return;
    }

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    int min_gate = s_ld_min_gate;
    int max_gate = s_ld_max_gate;
    int delay_s = s_ld_delay_s;
    int trig0    = s_ld_trigger_sens;
    int hold0    = s_ld_maintain_sens;

    if (min_gate < GATE_MIN) min_gate = GATE_MIN;
    if (min_gate > GATE_MAX) min_gate = GATE_MAX;
    if (max_gate < GATE_MIN) max_gate = GATE_MIN;
    if (max_gate > GATE_MAX) max_gate = GATE_MAX;
    if (min_gate > max_gate) max_gate = min_gate;
    if (delay_s < RADAR_HOLD_MIN_S) delay_s = RADAR_HOLD_MIN_S;
    if (delay_s > RADAR_HOLD_MAX_S) delay_s = RADAR_HOLD_MAX_S;

    int trig0_local = (trig0 < 0) ? 0 : (trig0 > 65535 ? 65535 : trig0);
    int hold0_local = (hold0 < 0) ? 0 : (hold0 > 65535 ? 65535 : hold0);
    int sens_level = s_ld_sensitivity_pending ? s_ld_sensitivity : -1;
    s_ld_sensitivity_pending = false;
    if (sens_level >= 0) {
        trig0_local = (int)preset_threshold(sens_level, FACTORY_TRIGGER_THRESH, 0);
        hold0_local = (int)preset_threshold(sens_level, FACTORY_MAINTAIN_THRESH, 0);
    }

    s_ld_min_gate = min_gate;
    s_ld_max_gate = max_gate;
    s_ld_delay_s = delay_s;
    s_ld_trigger_sens = trig0_local;
    s_ld_maintain_sens = hold0_local;
    xSemaphoreGive(s_state_mutex);

    ESP_LOGI(TAG, "Applying LD2420 config: min_gate=%d max_gate=%d delay_s=%d trig0=%d maintain0=%d",
             min_gate, max_gate, delay_s, trig0_local, hold0_local);

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
    if (ld2420_set_delay_s(s_sensor, delay_s) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to set delay");
        write_ok = false;
    }
    if (sens_level >= 0) {
        for (int g = 0; g < LD2420_GATE_COUNT; ++g) {
            if (ld2420_set_trigger_sens(s_sensor, g, preset_threshold(sens_level, FACTORY_TRIGGER_THRESH, g)) != ESP_OK ||
                ld2420_set_maintain_sens(s_sensor, g, preset_threshold(sens_level, FACTORY_MAINTAIN_THRESH, g)) != ESP_OK) {
                ESP_LOGW(TAG, "Failed to set gate %d thresholds", g);
                write_ok = false;
            }
        }
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

    ESP_LOGI(TAG, "LD2420 reported config: min_gate=%d max_gate=%d delay_s=%d trig0=%" PRIu32 " maintain0=%" PRIu32,
             applied.min_gate, applied.max_gate, applied.delay_s,
             applied.trigger_sensitivity, applied.maintain_sensitivity);

    bool mismatch = (applied.min_gate != min_gate) ||
                    (applied.max_gate != max_gate) ||
                    (applied.delay_s != delay_s) ||
                    ((int)applied.trigger_sensitivity != trig0_local) ||
                    ((int)applied.maintain_sensitivity != hold0_local);

    update_ld_state_from_snapshot(&applied);
    sync_ld_sensitivity_from_sensor();
    ha_mqtt_publish_ld2420_config_states();

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

static void ota_install_result(bool ok, const char *message) {
    if (!ok) esp_wifi_set_ps(WIFI_PS_MIN_MODEM);  // back to the default power save
    ha_mqtt_publish_ota_result(ok, message);
}

static bool start_ota_install(const char *url, const char *sha256_hex,
                              uint32_t size, const char *version) {
    const ota_update_request_t req = {
        .url = url,
        .sha256_hex = sha256_hex,
        .size = size,
        .version = version,
    };
    // Modem sleep throttles the download on this chip; stay awake until done.
    esp_wifi_set_ps(WIFI_PS_NONE);
    esp_err_t err = ota_update_start(&req, ha_mqtt_publish_ota_progress, ota_install_result);
    if (err != ESP_OK) {
        esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
        ESP_LOGW(TAG, "OTA start failed: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

static void request_apply_ld_config(void);

static void apply_debounce_timer_cb(void *arg) {
    (void)arg;
    request_apply_ld_config();
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
    out_snapshot->delay_s = s_ld_delay_s;
    out_snapshot->trigger_sens = s_ld_trigger_sens;
    out_snapshot->maintain_sens = s_ld_maintain_sens;
    snprintf(out_snapshot->fw_version, sizeof(out_snapshot->fw_version), "%s", s_ld_fw_version);
    xSemaphoreGive(s_state_mutex);

    out_snapshot->mqtt_connected = ha_mqtt_is_connected();
    out_snapshot->creds_missing = !s_creds_ok;

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

    bool mqtt_credentials_configured = (s_creds.mqtt_user[0] != '\0');
    bool tls_enabled = (ca_pem != NULL);
    bool auth_ok = mqtt_credentials_configured || MQTT_ALLOW_ANONYMOUS_COMMANDS;
    bool transport_ok = tls_enabled || MQTT_ALLOW_INSECURE_COMMANDS;
    bool command_topics_enabled = auth_ok && transport_ok;

    if (!auth_ok) {
        ESP_LOGW(TAG, "MQTT command topics disabled: provision mqtt_user (tools/provision.ps1) or set MQTT_ALLOW_ANONYMOUS_COMMANDS=1");
    } else if (!transport_ok) {
        ESP_LOGW(TAG, "MQTT command topics disabled: TLS required (define MQTT_BROKER_CA_CERT_PEM, or set MQTT_ALLOW_INSECURE_COMMANDS=1 to opt in to plaintext)");
    } else if (!tls_enabled) {
        ESP_LOGW(TAG, "MQTT command topics enabled over plaintext via MQTT_ALLOW_INSECURE_COMMANDS; rely on trusted LAN and broker ACLs");
    }

    ha_mqtt_cfg_t cfg = {
        .broker_uri = uri,
        .username = s_creds.mqtt_user[0] ? s_creds.mqtt_user : NULL,
        .password = s_creds.mqtt_pass[0] ? s_creds.mqtt_pass : NULL,
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
        .get_ld_delay_s = get_ld_delay_s,
        .set_ld_delay_s = set_ld_delay_s,
        .get_sensitivity = get_sensitivity,
        .set_sensitivity = set_sensitivity,
        .ld_config_valid = ld_config_valid,
        .load_setting = app_setting_load,
        .save_setting = app_setting_save,
        .action_ota_install = start_ota_install,
    };

    ha_mqtt_init(&cfg);
    ha_mqtt_start();
    s_mqtt_initialized = true;
}

// Firmware before 2.4.0 let the Wi-Fi driver persist its config (SSID and
// passphrase) in the default NVS namespace "nvs.net80211". Drop those entries
// now that the driver runs with WIFI_STORAGE_RAM. NVS marks entries erased;
// the page itself is reclaimed later by NVS garbage collection.
static void purge_wifi_driver_nvs(void) {
    nvs_handle_t h;
    if (nvs_open("nvs.net80211", NVS_READONLY, &h) != ESP_OK) return;  // never created
    nvs_close(h);
    if (nvs_open("nvs.net80211", NVS_READWRITE, &h) != ESP_OK) return;
    esp_err_t err = nvs_erase_all(h);
    if (err == ESP_OK) err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Could not purge stored Wi-Fi driver config: %s", esp_err_to_name(err));
    }
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
        xEventGroupClearBits(s_wifi_event_group, WIFI_FAIL_BIT);
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
            // Reject APs weaker than WPA2-PSK so an evil-twin open/WEP/WPA1
            // AP advertising the same SSID cannot lure the device off-network.
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = { .capable = true, .required = false },
        },
    };

    // Keep the driver's copy of the config in RAM: the creds partition is the
    // only place the passphrase is stored.
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    // Lengths are bounded by device_creds_t (ssid 32, password 64).
    memcpy(wifi_config.sta.ssid, s_creds.wifi_ssid, strnlen(s_creds.wifi_ssid, sizeof(wifi_config.sta.ssid)));
    memcpy(wifi_config.sta.password, s_creds.wifi_pass, strnlen(s_creds.wifi_pass, sizeof(wifi_config.sta.password)));

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

    // Arm the rollback guard before anything below can return early.
    ota_update_init(OTA_ROLLBACK_TIMEOUT_S);

    s_creds_ok = (device_creds_load(&s_creds) == ESP_OK);
    purge_wifi_driver_nvs();

    // Restore persisted tunables before anything reads or publishes them.
    app_config_load();

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

    const esp_timer_create_args_t debounce_args = {
        .callback = apply_debounce_timer_cb,
        .name = "ld_apply_debounce",
    };
    if (esp_timer_create(&debounce_args, &s_apply_debounce_timer) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to create LD2420 apply debounce timer");
        s_apply_debounce_timer = NULL;
    }

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
    if (s_creds_ok) {
        ESP_LOGI(TAG, "Starting WiFi...");
        esp_err_t wifi_rc = wifi_init();
        if (wifi_rc != ESP_OK) {
            ESP_LOGW(TAG, "wifi_init returned %s; continuing, background retries may proceed", esp_err_to_name(wifi_rc));
        }
        // Wall-clock time is only used for the "Last restart" timestamp.
        esp_sntp_config_t sntp_cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
        if (esp_netif_sntp_init(&sntp_cfg) != ESP_OK) {
            ESP_LOGW(TAG, "SNTP init failed; Last restart will stay unknown");
        }
    } else {
        ESP_LOGE(TAG, "No network credentials provisioned - staying offline. Run tools/provision.ps1");
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

            static bool boot_time_published = false;
            if (!boot_time_published) {
                time_t now_s = time(NULL);
                if ((int64_t)now_s > MIN_VALID_EPOCH_S) {
                    ha_mqtt_publish_boot_time((int64_t)now_s - esp_timer_get_time() / 1000000LL);
                    boot_time_published = true;
                }
            }

            if (sensor_data.isValid && ota_update_pending_verify() && ha_mqtt_is_connected()) {
                ota_update_mark_valid();
            }

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

        // Heartbeat (signal + availability) independent of radar data.
        ha_mqtt_tick();

        // Radar watchdog: restart a radar that stopped streaming.
        {
            static int64_t last_recovery_us = 0;
            static int radar_state = -1;  // -1 unknown, 0 ok, 1 silent
            int64_t now_us = esp_timer_get_time();
            int silent = ((now_us - s_last_radar_frame_us) > RADAR_SILENT_US) ? 1 : 0;
            if (silent != radar_state) {
                if (silent) {
                    ESP_LOGW(TAG, "No radar data for %lld s",
                             (long long)((now_us - s_last_radar_frame_us) / 1000000LL));
                } else if (radar_state == 1) {
                    ESP_LOGI(TAG, "Radar data is back");
                    last_recovery_us = 0;
                    if (!ld_config_valid() && sync_ld_config_from_sensor() == ESP_OK) {
                        ha_mqtt_publish_ld2420_config_states();
                    }
                }
                radar_state = silent;
                ha_mqtt_publish_radar_fault(silent);
            }
            if (silent && (last_recovery_us == 0 || now_us - last_recovery_us > RADAR_RETRY_US)) {
                last_recovery_us = now_us;
                if (ld2420_lock(s_sensor, pdMS_TO_TICKS(1000))) {
                    esp_err_t rerr = ld2420_recover(s_sensor);
                    ld2420_unlock(s_sensor);
                    ESP_LOGW(TAG, "Radar recovery %s", rerr == ESP_OK ? "done" : esp_err_to_name(rerr));
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
