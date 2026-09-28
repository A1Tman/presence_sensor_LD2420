#include "ha_mqtt.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdlib.h>

#include "esp_chip_info.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "sensor_info.h"
#include "cJSON.h"
#include <strings.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

/* ======================= Module state ======================= */
static const char *TAG = "ha_mqtt";

static ha_mqtt_cfg_t s_cfg = {
    .broker_uri    = NULL,
    .username      = NULL,
    .password      = NULL,
    .friendly_name = "Radar Sensor",
    .suggested_area= NULL,
    .app_version   = NULL,
    .distance_supported = true,
    .broker_ca_cert_pem = NULL,
};

static esp_mqtt_client_handle_t s_client = NULL;
static bool s_connected = false;
static SemaphoreHandle_t s_publish_lock = NULL;

/* Internal owned storage for dynamic strings */
static char s_broker_uri[128];

/* Derived identifiers & topics */
static char s_mac_str[18];              // AA:BB:CC:DD:EE:FF
static char s_serial[13];               // AABBCCDDEEFF (HA device serial_number)
static char s_hw_version[32];           // ESP32-C3 rev v0.4 (HA device hw_version)
static char s_devid[32];                // presence-aabbcc
static char s_entity_slug[48];          // living_room_presence (from friendly name)
static char s_topic_base[64];           // presence/presence-aabbcc
static char s_topic_status[96];
static char s_topic_presence[96];
static char s_topic_movement_distance[96];
static char s_topic_attrs[96];
static char s_topic_rssi[96];
static char s_topic_boot_time[96];
static char s_topic_fwver[96];

/* Config state + command topics */
static char s_topic_cfg_movement_thresh_stat[96];
static char s_topic_cfg_movement_thresh_cmd[96];
static char s_topic_cfg_presence_timeout_stat[96];
static char s_topic_cfg_presence_timeout_cmd[96];

/* LD2420 tuning (vendor params). Gate settings are exchanged with HA in
 * metres (70 cm per gate); the radar hold time in seconds. */
#define GATE_DEPTH_CM 70
static char s_topic_cfg_ld_min_stat[96];
static char s_topic_cfg_ld_min_cmd[96];
static char s_topic_cfg_ld_max_stat[96];
static char s_topic_cfg_ld_max_cmd[96];
static char s_topic_cfg_ld_delay_stat[96];
static char s_topic_cfg_ld_delay_cmd[96];
static char s_topic_cfg_sens_stat[96];
static char s_topic_cfg_sens_cmd[96];
/* Command topics (actions) */
static char s_topic_cmd_restart[96];
static char s_topic_cmd_resend_disc[96];
static char s_topic_cmd_apply_cfg[96];

/* Firmware update (HA update entity). The manifest is published retained by
 * tools/ota_release.ps1; state is guarded by s_publish_lock. */
static char s_topic_ota_state[96];
static char s_topic_ota_install_cmd[96];
static char s_topic_ota_manifest_cmd[96];
typedef struct {
    bool     valid;
    char     version[32];
    char     url[256];
    char     sha256[65];
    uint32_t size;
    char     notes[128];
} ota_manifest_t;
static ota_manifest_t s_ota_manifest;
static bool s_ota_in_progress = false;
static int  s_ota_percent = -1;
static char s_ota_last_error[64];

/* Distance smoothing + movement zones */
#define SMOOTH_BUFFER_SIZE 16
#define ZONE_COUNT 3
#define ZONE_DISTANCE_MAX_CM 600
#define MQTT_RX_TOPIC_MAX_LEN   128
#define MQTT_RX_PAYLOAD_MAX_LEN 768     // OTA manifest is the largest command
#define OTA_MAX_IMAGE_SIZE      (4U * 1024U * 1024U)
static int  s_smooth_win = 5;
static int  s_smooth_ring[SMOOTH_BUFFER_SIZE];
static int  s_smooth_count = 0;
static int  s_smooth_head = 0;
static int  s_zone_min_cm[ZONE_COUNT] = {0, 100, 200};  // Near, Mid, Far
static int  s_zone_max_cm[ZONE_COUNT] = {99, 199, 400};
static int  s_zone_last_on[ZONE_COUNT] = {0, 0, 0};
static char s_topic_zone_movement[ZONE_COUNT][96];
static char s_topic_cfg_zone_max_stat[ZONE_COUNT][96];
static char s_topic_cfg_zone_max_cmd[ZONE_COUNT][96];
static char s_topic_cfg_smooth_stat[96];
static char s_topic_cfg_smooth_cmd[96];

/* HA Discovery */
static char s_disc_prefix[64] = "homeassistant";
static bool s_restart_migration_done = false;

/* Cached state for reconnect */
static bool s_have_last = false;
static bool s_last_present = false;
static int  s_last_distance_mm = -1;
static bool s_have_ld2420_fw_version = false;
static char s_last_ld2420_fw_version[64] = {0};
static int64_t s_boot_epoch_s = 0;      // 0 = unknown (no wall clock yet)

/* Uptime ticker */
static int64_t s_last_diag_us = 0;
static int64_t s_last_restart_us = 0;
static int64_t s_last_apply_us = 0;

typedef struct {
    bool active;
    bool overflow;
    int expected_len;
    int topic_len;
    char topic[MQTT_RX_TOPIC_MAX_LEN];
    char data[MQTT_RX_PAYLOAD_MAX_LEN + 1];
} mqtt_rx_assembly_t;

// Reassembly buffer for fragmented MQTT_EVENT_DATA payloads. Accessed only
// from the esp-mqtt event task, which is single-threaded, so no lock is
// needed. If you ever dispatch MQTT events from multiple tasks this becomes
// unsafe.
static mqtt_rx_assembly_t s_mqtt_rx = {0};

#define RESTART_MIN_INTERVAL_US (30LL * 1000000LL)
#define APPLY_MIN_INTERVAL_US   (5LL  * 1000000LL)

/* ======================= Utilities ======================= */
static void pub(const char *topic, const char *payload, int qos, int retain);

static void mac_to_str(uint8_t mac[6], char out[18]) {
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void make_entity_slug(const char *src, char *out, size_t out_size) {
    if (!out || out_size == 0) return;

    size_t j = 0;
    bool last_was_sep = false;
    if (src) {
        for (size_t i = 0; src[i] != '\0' && j + 1 < out_size; ++i) {
            unsigned char c = (unsigned char)src[i];
            if (isalnum(c)) {
                out[j++] = (char)tolower(c);
                last_was_sep = false;
            } else if (!last_was_sep) {
                out[j++] = '_';
                last_was_sep = true;
            }
        }
    }

    while (j > 0 && out[j - 1] == '_') {
        --j;
    }
    if (j == 0) {
        out[j++] = 'e';
    }
    out[j] = '\0';
}

static bool json_appendf(char *payload, size_t payload_size, int *len, const char *fmt, ...) {
    if (!payload || payload_size == 0 || !len || !fmt || *len < 0) {
        return false;
    }

    size_t used = (size_t)*len;
    if (used >= payload_size) {
        payload[payload_size - 1] = '\0';
        *len = (int)(payload_size - 1);
        return false;
    }

    va_list args;
    va_start(args, fmt);
    int written = vsnprintf(payload + used, payload_size - used, fmt, args);
    va_end(args);

    if (written < 0) {
        payload[used] = '\0';
        *len = -1;  // sentinel: subsequent json_appendf calls short-circuit
        return false;
    }

    if ((size_t)written >= payload_size - used) {
        payload[payload_size - 1] = '\0';
        *len = -1;  // sentinel: builder is now truncated
        return false;
    }

    *len += written;
    return true;
}

// Publish a discovery payload only if it was built without truncation.
// Pair with json_appendf's -1 sentinel: if any append in the chain overflowed,
// len < 0 here and we skip the pub instead of shipping malformed JSON to HA.
static void try_pub_disc(const char *topic, const char *payload, int len) {
    if (len < 0) {
        ESP_LOGW(TAG, "Discovery payload truncated, skipping %s", topic);
        return;
    }
    pub(topic, payload, 1, 1);
}

// suffix NULL/"" gives the bare device slug, e.g. binary_sensor.living_room_presence.
static void append_default_entity_id(char *payload, size_t payload_size, int *len,
                                     const char *domain, const char *suffix) {
    if (!payload || !len || !domain) return;
    if (suffix && suffix[0]) {
        json_appendf(payload, payload_size, len,
                     "\"default_entity_id\":\"%s.%s_%s\",",
                     domain, s_entity_slug, suffix);
    } else {
        json_appendf(payload, payload_size, len,
                     "\"default_entity_id\":\"%s.%s\",",
                     domain, s_entity_slug);
    }
}

/* Safe string to integer conversion with validation */
static bool safe_atoi(const char *str, int len, int *out_val, int min, int max) {
    if (!str || len <= 0 || len >= 16) return false;
    
    char tmp[16];
    memcpy(tmp, str, len);
    tmp[len] = '\0';
    
    // Remove leading/trailing whitespace
    char *start = tmp;
    while (*start && isspace((unsigned char)*start)) start++;
    if (*start == '\0') return false;
    char *end = start + strlen(start) - 1;
    while (end > start && isspace((unsigned char)*end)) *end-- = '\0';
    
    char *endptr;
    long val = strtol(start, &endptr, 10);
    
    if (*endptr != '\0') return false;
    if (val < min || val > max) return false;

    *out_val = (int)val;
    return true;
}

static void mqtt_rx_reset(void) {
    memset(&s_mqtt_rx, 0, sizeof(s_mqtt_rx));
}

static bool mqtt_event_get_complete_payload(esp_mqtt_event_handle_t e,
                                            const char **topic, int *topic_len,
                                            const char **data, int *data_len) {
    if (!e || !topic || !topic_len || !data || !data_len) {
        return false;
    }

    if (e->total_data_len <= 0 ||
        (e->current_data_offset == 0 && e->data_len == e->total_data_len)) {
        *topic = e->topic;
        *topic_len = e->topic_len;
        *data = e->data;
        *data_len = e->data_len;
        return true;
    }

    if (e->current_data_offset == 0) {
        mqtt_rx_reset();
        s_mqtt_rx.active = true;
        s_mqtt_rx.expected_len = e->total_data_len;

        if (e->topic_len >= MQTT_RX_TOPIC_MAX_LEN) {
            s_mqtt_rx.overflow = true;
            ESP_LOGW(TAG, "Dropping MQTT command with topic length %d", e->topic_len);
        } else if (e->topic && e->topic_len > 0) {
            memcpy(s_mqtt_rx.topic, e->topic, e->topic_len);
            s_mqtt_rx.topic[e->topic_len] = '\0';
            s_mqtt_rx.topic_len = e->topic_len;
        }
    } else if (!s_mqtt_rx.active) {
        ESP_LOGW(TAG, "Dropping MQTT fragment without an active message");
        return false;
    }

    if (e->current_data_offset < 0 || e->data_len < 0 ||
        (e->current_data_offset + e->data_len) > e->total_data_len) {
        ESP_LOGW(TAG, "Dropping invalid MQTT fragment offset=%d len=%d total=%d",
                 e->current_data_offset, e->data_len, e->total_data_len);
        mqtt_rx_reset();
        return false;
    }

    if ((e->current_data_offset + e->data_len) > MQTT_RX_PAYLOAD_MAX_LEN) {
        s_mqtt_rx.overflow = true;
    } else if (!s_mqtt_rx.overflow && e->data && e->data_len > 0) {
        memcpy(s_mqtt_rx.data + e->current_data_offset, e->data, e->data_len);
    }

    if ((e->current_data_offset + e->data_len) < e->total_data_len) {
        return false;
    }

    if (s_mqtt_rx.overflow) {
        ESP_LOGW(TAG, "Dropping MQTT command payload larger than %d bytes",
                 MQTT_RX_PAYLOAD_MAX_LEN);
        mqtt_rx_reset();
        return false;
    }

    s_mqtt_rx.data[s_mqtt_rx.expected_len] = '\0';
    *topic = s_mqtt_rx.topic;
    *topic_len = s_mqtt_rx.topic_len;
    *data = s_mqtt_rx.data;
    *data_len = s_mqtt_rx.expected_len;
    // Caller's pointers reference s_mqtt_rx; do not memset it here. The next
    // multi-fragment message resets via the offset==0 path; single-fragment
    // messages do not touch s_mqtt_rx at all.
    s_mqtt_rx.active = false;
    return true;
}

static void clamp_zone_value(int *value) {
    if (!value) return;
    if (*value < 0) *value = 0;
    if (*value > ZONE_DISTANCE_MAX_CM) *value = ZONE_DISTANCE_MAX_CM;
}

// Zones are contiguous: Near starts at 0 and each zone starts right after the
// previous one ends, so only the three end distances are configurable.
static void normalize_zone_ranges_locked(void) {
    int prev_end = -1;
    for (int i = 0; i < ZONE_COUNT; ++i) {
        int start = prev_end + 1;
        if (start > ZONE_DISTANCE_MAX_CM) start = ZONE_DISTANCE_MAX_CM;
        s_zone_min_cm[i] = start;
        clamp_zone_value(&s_zone_max_cm[i]);
        if (s_zone_max_cm[i] < start) s_zone_max_cm[i] = start;
        prev_end = s_zone_max_cm[i];
    }
}

static void snapshot_zone_ranges(int mins[ZONE_COUNT], int maxs[ZONE_COUNT]) {
    bool lock_taken = false;
    if (s_publish_lock) {
        lock_taken = xSemaphoreTake(s_publish_lock, portMAX_DELAY) == pdTRUE;
    }

    for (int i = 0; i < ZONE_COUNT; ++i) {
        mins[i] = s_zone_min_cm[i];
        maxs[i] = s_zone_max_cm[i];
    }

    if (lock_taken) {
        xSemaphoreGive(s_publish_lock);
    }
}

static void publish_zone_config_states(void) {
    int mins[ZONE_COUNT];
    int maxs[ZONE_COUNT];
    snapshot_zone_ranges(mins, maxs);

    for (int i = 0; i < ZONE_COUNT; ++i) {
        char buf[16];
        snprintf(buf, sizeof(buf), "%d", maxs[i]);
        pub(s_topic_cfg_zone_max_stat[i], buf, 1, 1);
    }
}

static void zone_setting_key(int index, char *key, size_t key_size) {
    snprintf(key, key_size, "zone%d_end", index);
}

static void set_zone_end(int index, int value) {
    int ends[ZONE_COUNT];
    int start = 0;
    bool lock_taken = false;
    if (s_publish_lock) {
        lock_taken = xSemaphoreTake(s_publish_lock, portMAX_DELAY) == pdTRUE;
    }
    if (s_publish_lock && !lock_taken) return;

    s_zone_max_cm[index] = value;
    normalize_zone_ranges_locked();
    for (int i = 0; i < ZONE_COUNT; ++i) ends[i] = s_zone_max_cm[i];
    start = s_zone_min_cm[index];

    if (lock_taken) {
        xSemaphoreGive(s_publish_lock);
    }

    if (s_cfg.save_setting) {
        char key[16];
        for (int i = 0; i < ZONE_COUNT; ++i) {
            zone_setting_key(i, key, sizeof(key));
            s_cfg.save_setting(key, ends[i]);
        }
    }
    publish_zone_config_states();
    ESP_LOGI(TAG, "Set zone %d range: %d-%d cm", index, start, ends[index]);
}

// Restore zone ends and smoothing saved by earlier set commands.
static void load_persisted_settings(void) {
    if (!s_cfg.load_setting) return;
    char key[16];
    int v;
    for (int i = 0; i < ZONE_COUNT; ++i) {
        zone_setting_key(i, key, sizeof(key));
        if (s_cfg.load_setting(key, &v) && v >= 0 && v <= ZONE_DISTANCE_MAX_CM) {
            s_zone_max_cm[i] = v;
        }
    }
    if (s_cfg.load_setting("smoothing", &v) && v >= 1 && v <= 10) {
        s_smooth_win = v;
    }
    normalize_zone_ranges_locked();
}

static const char *chip_model_name(esp_chip_model_t model) {
    switch (model) {
        case CHIP_ESP32:   return "ESP32";
        case CHIP_ESP32S2: return "ESP32-S2";
        case CHIP_ESP32S3: return "ESP32-S3";
        case CHIP_ESP32C3: return "ESP32-C3";
        case CHIP_ESP32C2: return "ESP32-C2";
        case CHIP_ESP32C6: return "ESP32-C6";
        case CHIP_ESP32H2: return "ESP32-H2";
        default:           return "ESP32";
    }
}

static void derive_ids_and_topics(void) {
    uint8_t mac[6] = {0};
    esp_efuse_mac_get_default(mac);
    mac_to_str(mac, s_mac_str);
    snprintf(s_serial, sizeof(s_serial), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    /* revision is encoded as MXX (major * 100 + minor) */
    esp_chip_info_t chip = {0};
    esp_chip_info(&chip);
    snprintf(s_hw_version, sizeof(s_hw_version), "%s rev v%u.%u",
             chip_model_name(chip.model),
             (unsigned)(chip.revision / 100), (unsigned)(chip.revision % 100));

    /* short hex id (last 3 bytes) */
    snprintf(s_devid, sizeof(s_devid), "presence-%02x%02x%02x", mac[3], mac[4], mac[5]);
    // Entity IDs follow the device name ("Living Room Presence" ->
    // living_room_presence_*), like entities HA names itself.
    make_entity_slug((s_cfg.friendly_name && s_cfg.friendly_name[0]) ? s_cfg.friendly_name : s_devid,
                     s_entity_slug, sizeof(s_entity_slug));

    /* Use custom discovery prefix if provided */
    if (s_cfg.discovery_prefix && s_cfg.discovery_prefix[0]) {
        snprintf(s_disc_prefix, sizeof(s_disc_prefix), "%s", s_cfg.discovery_prefix);
    }

    snprintf(s_topic_base, sizeof(s_topic_base), "presence/%s", s_devid);
    snprintf(s_topic_status, sizeof(s_topic_status), "%s/status", s_topic_base);
    snprintf(s_topic_presence, sizeof(s_topic_presence), "%s/presence", s_topic_base);
    snprintf(s_topic_movement_distance, sizeof(s_topic_movement_distance), "%s/movement_distance_cm", s_topic_base);
    snprintf(s_topic_attrs, sizeof(s_topic_attrs), "%s/attributes", s_topic_base);
    snprintf(s_topic_rssi, sizeof(s_topic_rssi), "%s/rssi", s_topic_base);
    snprintf(s_topic_boot_time, sizeof(s_topic_boot_time), "%s/last_restart", s_topic_base);
    snprintf(s_topic_fwver, sizeof(s_topic_fwver), "%s/ld2420/fw_version", s_topic_base);
    
    snprintf(s_topic_cfg_movement_thresh_stat, sizeof(s_topic_cfg_movement_thresh_stat), "%s/cfg/movement_threshold_cm", s_topic_base);
    snprintf(s_topic_cfg_movement_thresh_cmd, sizeof(s_topic_cfg_movement_thresh_cmd), "%s/cmd/movement_threshold_cm", s_topic_base);
    snprintf(s_topic_cfg_presence_timeout_stat, sizeof(s_topic_cfg_presence_timeout_stat), "%s/cfg/presence_timeout_sec", s_topic_base);
    snprintf(s_topic_cfg_presence_timeout_cmd, sizeof(s_topic_cfg_presence_timeout_cmd), "%s/cmd/presence_timeout_sec", s_topic_base);

    /* LD2420 tuning */
    snprintf(s_topic_cfg_ld_min_stat, sizeof(s_topic_cfg_ld_min_stat), "%s/cfg/ld2420/min_distance_m", s_topic_base);
    snprintf(s_topic_cfg_ld_min_cmd, sizeof(s_topic_cfg_ld_min_cmd), "%s/cmd/ld2420/min_distance_m", s_topic_base);
    snprintf(s_topic_cfg_ld_max_stat, sizeof(s_topic_cfg_ld_max_stat), "%s/cfg/ld2420/detection_range_m", s_topic_base);
    snprintf(s_topic_cfg_ld_max_cmd, sizeof(s_topic_cfg_ld_max_cmd), "%s/cmd/ld2420/detection_range_m", s_topic_base);
    snprintf(s_topic_cfg_ld_delay_stat, sizeof(s_topic_cfg_ld_delay_stat), "%s/cfg/ld2420/delay_time", s_topic_base);
    snprintf(s_topic_cfg_ld_delay_cmd, sizeof(s_topic_cfg_ld_delay_cmd), "%s/cmd/ld2420/delay_time", s_topic_base);
    snprintf(s_topic_cfg_sens_stat, sizeof(s_topic_cfg_sens_stat), "%s/cfg/ld2420/sensitivity", s_topic_base);
    snprintf(s_topic_cfg_sens_cmd, sizeof(s_topic_cfg_sens_cmd), "%s/cmd/ld2420/sensitivity", s_topic_base);

    /* Movement zones */
    const char* zone_names[] = {"near", "mid", "far"};
    for (int i = 0; i < ZONE_COUNT; ++i) {
        snprintf(s_topic_zone_movement[i], sizeof(s_topic_zone_movement[i]), "%s/movement/%s_range", s_topic_base, zone_names[i]);
        snprintf(s_topic_cfg_zone_max_stat[i], sizeof(s_topic_cfg_zone_max_stat[i]), "%s/cfg/zone/%s/max_cm", s_topic_base, zone_names[i]);
        snprintf(s_topic_cfg_zone_max_cmd[i], sizeof(s_topic_cfg_zone_max_cmd[i]), "%s/cmd/zone/%s/max_cm", s_topic_base, zone_names[i]);
    }
    snprintf(s_topic_cfg_smooth_stat, sizeof(s_topic_cfg_smooth_stat), "%s/cfg/distance_smoothing", s_topic_base);
    snprintf(s_topic_cfg_smooth_cmd, sizeof(s_topic_cfg_smooth_cmd), "%s/cmd/distance_smoothing", s_topic_base);
    /* Action command topics */
    snprintf(s_topic_cmd_restart, sizeof(s_topic_cmd_restart), "%s/cmd/restart", s_topic_base);
    snprintf(s_topic_cmd_resend_disc, sizeof(s_topic_cmd_resend_disc), "%s/cmd/resend_discovery", s_topic_base);
    snprintf(s_topic_cmd_apply_cfg, sizeof(s_topic_cmd_apply_cfg), "%s/cmd/apply_config", s_topic_base);
    snprintf(s_topic_ota_state, sizeof(s_topic_ota_state), "%s/ota/state", s_topic_base);
    snprintf(s_topic_ota_install_cmd, sizeof(s_topic_ota_install_cmd), "%s/cmd/ota/install", s_topic_base);
    snprintf(s_topic_ota_manifest_cmd, sizeof(s_topic_ota_manifest_cmd), "%s/cmd/ota/manifest", s_topic_base);

}

static bool should_downgrade_publish_qos(const char *topic) {
    if (!topic) {
        return false;
    }

    size_t discovery_len = strlen(s_disc_prefix);
    if (discovery_len > 0 &&
        strncmp(topic, s_disc_prefix, discovery_len) == 0 &&
        topic[discovery_len] == '/') {
        return true;
    }

    return strstr(topic, "/cfg/") != NULL;
}

static void set_connected(bool connected) {
    bool lock_taken = false;
    if (s_publish_lock) {
        lock_taken = xSemaphoreTake(s_publish_lock, portMAX_DELAY) == pdTRUE;
    }

    s_connected = connected;

    if (lock_taken) {
        xSemaphoreGive(s_publish_lock);
    }
}

static bool is_connected_snapshot(void) {
    bool connected;
    bool lock_taken = false;
    if (s_publish_lock) {
        lock_taken = xSemaphoreTake(s_publish_lock, portMAX_DELAY) == pdTRUE;
    }

    connected = s_connected;

    if (lock_taken) {
        xSemaphoreGive(s_publish_lock);
    }

    return connected;
}

static void pub(const char *topic, const char *payload, int qos, int retain) {
    if (!s_client || !is_connected_snapshot()) return;
    int effective_qos = should_downgrade_publish_qos(topic) ? 0 : qos;
    int mid = esp_mqtt_client_publish(s_client, topic, payload, 0, effective_qos, retain);
    if (mid < 0) ESP_LOGW(TAG, "Publish failed to %s", topic);
}

/* ======================= Discovery payloads ======================= */
/* Forward declaration for helper used below */
static void json_escape(const char *in, char *out, size_t out_size);

#define AVAIL_FIELDS "\"avty_t\":\"%s\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\""

static void discovery_topic(char *out, size_t out_size, const char *component, const char *object_id) {
    snprintf(out, out_size, "%s/%s/%s/%s/config", s_disc_prefix, component, s_devid, object_id);
}

// Publish one retained discovery config: {<body>,"default_entity_id":..,"dev":{..}}.
// entity_suffix feeds default_entity_id (NULL = bare device slug).
// Discovery runs on the MQTT task (connect handler, resend button), whose
// stack is small: keep the large buffers static rather than on the stack.
// Not reentrant; do not publish discovery from two tasks at once.
static void publish_entity(const char *component, const char *object_id, const char *entity_suffix,
                           const char *dev_block, const char *body_fmt, ...) {
    static char body[1024];
    static char payload[2048];
    va_list args;
    va_start(args, body_fmt);
    int n = vsnprintf(body, sizeof(body), body_fmt, args);
    va_end(args);
    if (n < 0 || (size_t)n >= sizeof(body)) {
        ESP_LOGW(TAG, "Discovery body truncated for %s/%s", component, object_id);
        return;
    }

    int len = 0;
    json_appendf(payload, sizeof(payload), &len, "{%s,", body);
    append_default_entity_id(payload, sizeof(payload), &len, component, entity_suffix);
    json_appendf(payload, sizeof(payload), &len, "%s}", dev_block);

    char topic[192];
    discovery_topic(topic, sizeof(topic), component, object_id);
    try_pub_disc(topic, payload, len);
}

static void clear_entity(const char *component, const char *object_id) {
    char topic[192];
    discovery_topic(topic, sizeof(topic), component, object_id);
    pub(topic, "", 0, 1);
}

// Entities published by firmware before 2.5.0. Clearing their retained
// discovery configs makes Home Assistant remove them.
static void clear_legacy_discovery(void) {
    static const char *const legacy[][2] = {
        {"sensor", "movement_distance"}, {"sensor", "rssi"}, {"sensor", "uptime"}, {"sensor", "ld_fw"},
        {"binary_sensor", "movement_1"}, {"binary_sensor", "movement_2"}, {"binary_sensor", "movement_3"},
        {"number", "movement_thresh"}, {"number", "ld_min_gate"}, {"number", "ld_max_gate"},
        {"number", "ld_delay"}, {"number", "ld_trig_sens"}, {"number", "ld_maint_sens"},
        {"number", "ld_hold00"}, {"number", "distance_smoothing"},
        {"number", "near_min"}, {"number", "near_max"}, {"number", "mid_min"},
        {"number", "mid_max"}, {"number", "far_min"}, {"number", "far_max"},
        {"button", "apply_config"},
    };
    for (size_t i = 0; i < sizeof(legacy) / sizeof(legacy[0]); ++i) {
        clear_entity(legacy[i][0], legacy[i][1]);
    }
}

static const char *const ZONE_KEYS[ZONE_COUNT] = {"near", "mid", "far"};
static const char *const ZONE_TITLES[ZONE_COUNT] = {"Near", "Mid", "Far"};

static void clear_command_discovery_configs(void) {
    static const char *const numbers[] = {
        "detection_range", "presence_timeout", "near_end", "mid_end", "far_end",
        "min_distance", "radar_hold", "movement_threshold", "smoothing",
    };
    for (size_t i = 0; i < sizeof(numbers) / sizeof(numbers[0]); ++i) {
        clear_entity("number", numbers[i]);
    }
    clear_entity("select", "sensitivity");
    clear_entity("button", "restart");
    clear_entity("button", "resend_discovery");
    clear_entity("update", "firmware");
}

static void publish_command_entities(const char *dev) {
    /* Everyday settings */
    if (s_cfg.get_sensitivity && s_cfg.set_sensitivity) {
        publish_entity("select", "sensitivity", "sensitivity", dev,
            "\"name\":\"Sensitivity\",\"uniq_id\":\"%s_sensitivity\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\","
            "\"options\":[\"Low\",\"Medium\",\"High\",\"Custom\"],\"ic\":\"mdi:tune-variant\","
            "\"ent_cat\":\"config\"," AVAIL_FIELDS,
            s_devid, s_topic_cfg_sens_cmd, s_topic_cfg_sens_stat, s_topic_status);
    } else {
        clear_entity("select", "sensitivity");
    }

    if (s_cfg.get_ld_max_gate && s_cfg.set_ld_max_gate) {
        publish_entity("number", "detection_range", "detection_range", dev,
            "\"name\":\"Detection range\",\"uniq_id\":\"%s_detection_range\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\","
            "\"min\":0.7,\"max\":11.2,\"step\":0.7,\"mode\":\"slider\",\"unit_of_meas\":\"m\","
            "\"ic\":\"mdi:signal-distance-variant\",\"ent_cat\":\"config\"," AVAIL_FIELDS,
            s_devid, s_topic_cfg_ld_max_cmd, s_topic_cfg_ld_max_stat, s_topic_status);
    } else {
        clear_entity("number", "detection_range");
    }

    if (s_cfg.get_hold_on_ms && s_cfg.set_hold_on_ms) {
        publish_entity("number", "presence_timeout", "presence_timeout", dev,
            "\"name\":\"Presence timeout\",\"uniq_id\":\"%s_presence_timeout\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\","
            "\"min\":5,\"max\":300,\"step\":5,\"mode\":\"slider\",\"unit_of_meas\":\"s\","
            "\"ic\":\"mdi:timer-sand\",\"ent_cat\":\"config\"," AVAIL_FIELDS,
            s_devid, s_topic_cfg_presence_timeout_cmd, s_topic_cfg_presence_timeout_stat, s_topic_status);
    } else {
        clear_entity("number", "presence_timeout");
    }

    for (int i = 0; i < ZONE_COUNT; ++i) {
        char object_id[16];
        snprintf(object_id, sizeof(object_id), "%s_end", ZONE_KEYS[i]);
        publish_entity("number", object_id, object_id, dev,
            "\"name\":\"%s zone ends at\",\"uniq_id\":\"%s_%s\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\","
            "\"min\":10,\"max\":%d,\"step\":10,\"mode\":\"slider\",\"unit_of_meas\":\"cm\","
            "\"ic\":\"mdi:arrow-expand-horizontal\",\"ent_cat\":\"config\"," AVAIL_FIELDS,
            ZONE_TITLES[i], s_devid, object_id, s_topic_cfg_zone_max_cmd[i], s_topic_cfg_zone_max_stat[i],
            ZONE_DISTANCE_MAX_CM, s_topic_status);
    }

    /* Advanced settings: registered disabled; enable them in HA if needed */
    if (s_cfg.get_ld_min_gate && s_cfg.set_ld_min_gate) {
        publish_entity("number", "min_distance", "min_distance", dev,
            "\"name\":\"Ignore closer than\",\"uniq_id\":\"%s_min_distance\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\","
            "\"min\":0,\"max\":10.5,\"step\":0.7,\"mode\":\"slider\",\"unit_of_meas\":\"m\","
            "\"ic\":\"mdi:arrow-collapse-left\",\"ent_cat\":\"config\",\"en\":false," AVAIL_FIELDS,
            s_devid, s_topic_cfg_ld_min_cmd, s_topic_cfg_ld_min_stat, s_topic_status);
    } else {
        clear_entity("number", "min_distance");
    }

    if (s_cfg.get_ld_delay_s && s_cfg.set_ld_delay_s) {
        publish_entity("number", "radar_hold", "radar_hold", dev,
            "\"name\":\"Radar hold time\",\"uniq_id\":\"%s_radar_hold\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\","
            "\"min\":0,\"max\":3600,\"step\":1,\"mode\":\"box\",\"unit_of_meas\":\"s\","
            "\"ic\":\"mdi:timer-cog-outline\",\"ent_cat\":\"config\",\"en\":false," AVAIL_FIELDS,
            s_devid, s_topic_cfg_ld_delay_cmd, s_topic_cfg_ld_delay_stat, s_topic_status);
    } else {
        clear_entity("number", "radar_hold");
    }

    if (s_cfg.get_distance_thresh_cm && s_cfg.set_distance_thresh_cm) {
        publish_entity("number", "movement_threshold", "movement_threshold", dev,
            "\"name\":\"Movement threshold\",\"uniq_id\":\"%s_movement_threshold\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\","
            "\"min\":1,\"max\":50,\"step\":1,\"mode\":\"slider\",\"unit_of_meas\":\"cm\","
            "\"ic\":\"mdi:walk\",\"ent_cat\":\"config\",\"en\":false," AVAIL_FIELDS,
            s_devid, s_topic_cfg_movement_thresh_cmd, s_topic_cfg_movement_thresh_stat, s_topic_status);
    } else {
        clear_entity("number", "movement_threshold");
    }

    publish_entity("number", "smoothing", "distance_smoothing", dev,
        "\"name\":\"Distance smoothing\",\"uniq_id\":\"%s_smoothing\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\","
        "\"min\":1,\"max\":10,\"step\":1,\"mode\":\"slider\","
        "\"ic\":\"mdi:chart-bell-curve-cumulative\",\"ent_cat\":\"config\",\"en\":false," AVAIL_FIELDS,
        s_devid, s_topic_cfg_smooth_cmd, s_topic_cfg_smooth_stat, s_topic_status);

    /* Actions */
    if (!s_restart_migration_done) {
        clear_entity("button", "restart");
        s_restart_migration_done = true;
    }
    publish_entity("button", "restart", "restart", dev,
        "\"name\":\"Restart\",\"uniq_id\":\"%s_restart\",\"cmd_t\":\"%s\",\"dev_cla\":\"restart\","
        "\"ent_cat\":\"diagnostic\"," AVAIL_FIELDS,
        s_devid, s_topic_cmd_restart, s_topic_status);
    publish_entity("button", "resend_discovery", "resend_discovery", dev,
        "\"name\":\"Resend discovery\",\"uniq_id\":\"%s_resend_disc\",\"cmd_t\":\"%s\","
        "\"ic\":\"mdi:refresh\",\"ent_cat\":\"diagnostic\"," AVAIL_FIELDS,
        s_devid, s_topic_cmd_resend_disc, s_topic_status);

    /* Firmware update */
    if (s_cfg.action_ota_install) {
        publish_entity("update", "firmware", "firmware", dev,
            "\"name\":\"Firmware\",\"uniq_id\":\"%s_firmware\","
            "\"stat_t\":\"%s\",\"cmd_t\":\"%s\",\"pl_inst\":\"install\","
            "\"dev_cla\":\"firmware\",\"ent_cat\":\"config\"," AVAIL_FIELDS,
            s_devid, s_topic_ota_state, s_topic_ota_install_cmd, s_topic_status);
    } else {
        clear_entity("update", "firmware");
    }
}

static void publish_discovery_all(void) {
    static char dev_block[1024];
    char area[96] = {0};
    char dev_name_esc[128];
    char dev_model_esc[128];
    char app_ver_esc[64];
    char area_val_esc[64];
    const char *dev_name = s_cfg.friendly_name ? s_cfg.friendly_name : "Radar Sensor";
    const char *dev_model = s_cfg.device_model ? s_cfg.device_model : "HLK-LD2420 + ESP32";

    json_escape(dev_name, dev_name_esc, sizeof(dev_name_esc));
    json_escape(dev_model, dev_model_esc, sizeof(dev_model_esc));
    json_escape(s_cfg.app_version ? s_cfg.app_version : "unknown", app_ver_esc, sizeof(app_ver_esc));

    if (s_cfg.suggested_area && s_cfg.suggested_area[0]) {
        json_escape(s_cfg.suggested_area, area_val_esc, sizeof(area_val_esc));
        snprintf(area, sizeof(area), ",\"suggested_area\":\"%s\"", area_val_esc);
    }

    snprintf(dev_block, sizeof(dev_block),
        "\"dev\":{\"ids\":[\"%s\"],\"name\":\"%s\",\"mf\":\"Hi-Link + DIY\","
        "\"mdl\":\"%s\",\"sw\":\"%s\",\"hw\":\"%s\",\"sn\":\"%s\","
        "\"connections\":[[\"mac\",\"%s\"]]}%s",
        s_devid, dev_name_esc, dev_model_esc, app_ver_esc,
        s_hw_version, s_serial, s_mac_str, area);

    clear_legacy_discovery();

    /* Sensors */
    publish_entity("binary_sensor", "presence", NULL, dev_block,
        "\"name\":\"Presence\",\"uniq_id\":\"%s_presence\",\"stat_t\":\"%s\","
        "\"dev_cla\":\"occupancy\",\"pl_on\":\"ON\",\"pl_off\":\"OFF\","
        "\"json_attr_t\":\"%s\"," AVAIL_FIELDS,
        s_devid, s_topic_presence, s_topic_attrs, s_topic_status);

    if (s_cfg.distance_supported) {
        publish_entity("sensor", "distance", "distance", dev_block,
            "\"name\":\"Distance\",\"uniq_id\":\"%s_distance\",\"stat_t\":\"%s\","
            "\"dev_cla\":\"distance\",\"unit_of_meas\":\"cm\",\"stat_cla\":\"measurement\","
            "\"sug_dsp_prc\":0,\"ic\":\"mdi:signal-distance-variant\"," AVAIL_FIELDS,
            s_devid, s_topic_movement_distance, s_topic_status);
    } else {
        clear_entity("sensor", "distance");
    }

    for (int i = 0; i < ZONE_COUNT; ++i) {
        char object_id[16];
        snprintf(object_id, sizeof(object_id), "%s_zone", ZONE_KEYS[i]);
        publish_entity("binary_sensor", object_id, object_id, dev_block,
            "\"name\":\"%s zone\",\"uniq_id\":\"%s_%s\",\"stat_t\":\"%s\","
            "\"dev_cla\":\"occupancy\",\"pl_on\":\"ON\",\"pl_off\":\"OFF\"," AVAIL_FIELDS,
            ZONE_TITLES[i], s_devid, object_id, s_topic_zone_movement[i], s_topic_status);
    }

    /* Diagnostics */
    publish_entity("sensor", "signal", "signal", dev_block,
        "\"name\":\"Signal\",\"uniq_id\":\"%s_signal\",\"stat_t\":\"%s\","
        "\"dev_cla\":\"signal_strength\",\"unit_of_meas\":\"dBm\",\"stat_cla\":\"measurement\","
        "\"ent_cat\":\"diagnostic\"," AVAIL_FIELDS,
        s_devid, s_topic_rssi, s_topic_status);
    publish_entity("sensor", "last_restart", "last_restart", dev_block,
        "\"name\":\"Last restart\",\"uniq_id\":\"%s_last_restart\",\"stat_t\":\"%s\","
        "\"dev_cla\":\"timestamp\",\"ic\":\"mdi:restart\",\"ent_cat\":\"diagnostic\"," AVAIL_FIELDS,
        s_devid, s_topic_boot_time, s_topic_status);
    publish_entity("sensor", "radar_firmware", "radar_firmware", dev_block,
        "\"name\":\"Radar firmware\",\"uniq_id\":\"%s_radar_firmware\",\"stat_t\":\"%s\","
        "\"ic\":\"mdi:chip\",\"ent_cat\":\"diagnostic\"," AVAIL_FIELDS,
        s_devid, s_topic_fwver, s_topic_status);

    if (s_cfg.command_topics_enabled) {
        publish_command_entities(dev_block);
    } else {
        clear_command_discovery_configs();
    }
}

/* Minimal JSON string escaper: escapes quotes, backslashes and control chars */
static void json_escape(const char *in, char *out, size_t out_size) {
    if (!in || !out || out_size == 0) { if (out && out_size) out[0] = '\0'; return; }
    size_t o = 0;
    for (size_t i = 0; in[i] && o + 2 < out_size; ++i) {
        unsigned char c = (unsigned char)in[i];
        switch (c) {
            case '"': case '\\':
                if (o + 2 < out_size) { out[o++] = '\\'; out[o++] = (char)c; }
                break;
            case '\b': out[o++] = '\\'; out[o++] = 'b'; break;
            case '\f': out[o++] = '\\'; out[o++] = 'f'; break;
            case '\n': out[o++] = '\\'; out[o++] = 'n'; break;
            case '\r': out[o++] = '\\'; out[o++] = 'r'; break;
            case '\t': out[o++] = '\\'; out[o++] = 't'; break;
            default:
                if (c < 0x20) {
                    // drop other control chars
                } else {
                    out[o++] = (char)c;
                }
        }
    }
    out[(o < out_size) ? o : out_size - 1] = '\0';
}

/* Accept only typical HA button press payloads */
static bool payload_is_press(const char *data, int len) {
    if (!data) return false;
    int s = 0, e = len;
    while (s < e && (unsigned char)data[s] <= ' ') s++;
    while (e > s && (unsigned char)data[e-1] <= ' ') e--;
    int n = e - s;
    if (n == 0) return false;
    if (n == 1 && (data[s] == '1' || data[s] == 'P' || data[s] == 'p')) return true;
    if (n == 2 && (data[s] == 'O' || data[s] == 'o') && (data[s+1] == 'N' || data[s+1] == 'n')) return true; // ON
    if (n == 5 && (strncasecmp(&data[s], "PRESS", 5) == 0)) return true;
    if (n == 5 && (strncasecmp(&data[s], "press", 5) == 0)) return true;
    return false;
}

static void log_mqtt_error_event(esp_mqtt_event_handle_t e) {
    if (!e || !e->error_handle) {
        ESP_LOGE(TAG, "MQTT error event without details");
        return;
    }

    const esp_mqtt_error_codes_t *err = e->error_handle;
    ESP_LOGE(TAG,
             "MQTT error: type=%d protocol=%d connect_rc=%d tls_esp=0x%x tls_stack=%d verify=0x%x sock_errno=%d",
             err->error_type,
             e->protocol_ver,
             err->connect_return_code,
             err->esp_tls_last_esp_err,
             err->esp_tls_stack_err,
             err->esp_tls_cert_verify_flags,
             err->esp_transport_sock_errno);

#if CONFIG_MQTT_PROTOCOL_5
    if (e->protocol_ver == MQTT_PROTOCOL_V_5 && err->disconnect_return_code != 0) {
        ESP_LOGE(TAG, "MQTT 5 disconnect reason: 0x%02x", err->disconnect_return_code);
    }
#endif
}

static void publish_birth_online(void) {
    pub(s_topic_status, "online", 1, 1);
}

// While a firmware download runs, telemetry is held back so the Wi-Fi link
// is not shared with a stream of QoS1 presence/distance updates. The latest
// state is cached and republished if the install fails.
static bool ota_download_active(void) {
    bool taken = s_publish_lock && xSemaphoreTake(s_publish_lock, portMAX_DELAY) == pdTRUE;
    bool active = s_ota_in_progress;
    if (taken) xSemaphoreGive(s_publish_lock);
    return active;
}

/* Periodic diagnostics: RSSI + availability heartbeat */
static void publish_periodic_diag_if_due(void) {
    const int64_t now_us = esp_timer_get_time();
    const int64_t interval = 30000000LL; // 30 seconds
    if (now_us - s_last_diag_us < interval) return;
    if (ota_download_active()) return;
    s_last_diag_us = now_us;

    wifi_ap_record_t ap = {0};
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
        char rssi[16];
        snprintf(rssi, sizeof(rssi), "%d", ap.rssi);
        pub(s_topic_rssi, rssi, 0, 0);
    }

    pub(s_topic_status, "online", 1, 1);
}

/* Attributes blob (mac, ip, etc.) */
static void publish_attrs_once(void) {
    esp_netif_ip_info_t ip;
    char json[512];
    char ip_str[32] = "0.0.0.0";

    esp_netif_t *netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (netif && esp_netif_get_ip_info(netif, &ip) == ESP_OK) {
        snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ip.ip));
    }
    
    // Caller-provided strings (device_model, app_version) need escaping so a
    // friendly_name like `Foo "Bar"` cannot break the published JSON. mac,
    // device_id, ip are computed from controlled inputs and are safe raw.
    char model_esc[128];
    char ver_esc[64];
    json_escape(s_cfg.device_model ? s_cfg.device_model : "HLK-LD2420 + ESP32",
                model_esc, sizeof(model_esc));
    json_escape(s_cfg.app_version ? s_cfg.app_version : "unknown",
                ver_esc, sizeof(ver_esc));

    int len = 0;
    json_appendf(json, sizeof(json), &len, "{");
    json_appendf(json, sizeof(json), &len, "\"mac\":\"%s\",", s_mac_str);
    json_appendf(json, sizeof(json), &len, "\"device_id\":\"%s\",", s_devid);
    json_appendf(json, sizeof(json), &len, "\"model\":\"%s\",", model_esc);
    json_appendf(json, sizeof(json), &len, "\"sw_version\":\"%s\",", ver_esc);
    json_appendf(json, sizeof(json), &len, "\"hw_version\":\"%s\",", s_hw_version);
    json_appendf(json, sizeof(json), &len, "\"ip\":\"%s\"", ip_str);
    json_appendf(json, sizeof(json), &len, "}");
    pub(s_topic_attrs, json, 0, 0);
}

// Gate g covers g*70 .. (g+1)*70 cm. HA sees metres with one decimal.
static void format_decimetres(char *out, size_t out_size, int decimetres) {
    snprintf(out, out_size, "%d.%d", decimetres / 10, decimetres % 10);
}

// Parse a metre value from HA ("4.9") into decimetres; false if malformed.
static bool parse_decimetres(const char *payload, int len, int *out_dm) {
    if (!payload || len <= 0 || len >= 16) return false;
    char tmp[16];
    memcpy(tmp, payload, (size_t)len);
    tmp[len] = '\0';
    char *end = NULL;
    float m = strtof(tmp, &end);
    while (end && *end && isspace((unsigned char)*end)) end++;
    if (end == tmp || (end && *end != '\0') || !(m >= 0.0f && m <= 20.0f)) return false;
    *out_dm = (int)(m * 10.0f + 0.5f);
    return true;
}

static const char *const SENSITIVITY_NAMES[] = {"Low", "Medium", "High"};

static void publish_sensitivity_state(void) {
    if (!s_cfg.command_topics_enabled || !s_cfg.get_sensitivity || !s_cfg.set_sensitivity) return;
    int level = s_cfg.get_sensitivity();
    const char *name = (level >= HA_MQTT_SENSITIVITY_LOW && level <= HA_MQTT_SENSITIVITY_HIGH)
                           ? SENSITIVITY_NAMES[level] : "Custom";
    pub(s_topic_cfg_sens_stat, name, 1, 1);
}

static void publish_ld2420_config_states(void) {
    if (!s_cfg.command_topics_enabled) {
        return;
    }

    char buf[16];

    if (s_cfg.get_ld_min_gate && s_cfg.set_ld_min_gate) {
        format_decimetres(buf, sizeof(buf), s_cfg.get_ld_min_gate() * GATE_DEPTH_CM / 10);
        pub(s_topic_cfg_ld_min_stat, buf, 1, 1);
    }
    if (s_cfg.get_ld_max_gate && s_cfg.set_ld_max_gate) {
        format_decimetres(buf, sizeof(buf), (s_cfg.get_ld_max_gate() + 1) * GATE_DEPTH_CM / 10);
        pub(s_topic_cfg_ld_max_stat, buf, 1, 1);
    }
    if (s_cfg.get_ld_delay_s && s_cfg.set_ld_delay_s) {
        snprintf(buf, sizeof(buf), "%d", s_cfg.get_ld_delay_s());
        pub(s_topic_cfg_ld_delay_stat, buf, 1, 1);
    }
    publish_sensitivity_state();
}

// Unix seconds -> "YYYY-MM-DDThh:mm:ss+00:00" (civil-from-days, no libc tz).
static void format_iso8601_utc(int64_t epoch, char *out, size_t out_size) {
    int64_t days = epoch / 86400;
    int64_t secs = epoch % 86400;
    if (secs < 0) { secs += 86400; days -= 1; }
    int64_t z = days + 719468;
    int64_t era = (z >= 0 ? z : z - 146096) / 146097;
    int64_t doe = z - era * 146097;
    int64_t yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    int64_t y = yoe + era * 400;
    int64_t doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    int64_t mp = (5 * doy + 2) / 153;
    int64_t d = doy - (153 * mp + 2) / 5 + 1;
    int64_t m = mp < 10 ? mp + 3 : mp - 9;
    if (m <= 2) y += 1;
    snprintf(out, out_size, "%04d-%02d-%02dT%02d:%02d:%02d+00:00",
             (int)y, (int)m, (int)d, (int)(secs / 3600), (int)(secs % 3600 / 60), (int)(secs % 60));
}

static void publish_boot_time(void) {
    int64_t epoch = 0;
    bool taken = s_publish_lock && xSemaphoreTake(s_publish_lock, portMAX_DELAY) == pdTRUE;
    epoch = s_boot_epoch_s;
    if (taken) xSemaphoreGive(s_publish_lock);
    if (epoch <= 0) return;

    char iso[32];
    format_iso8601_utc(epoch, iso, sizeof(iso));
    pub(s_topic_boot_time, iso, 1, 1);
}

/* ======================= Firmware update ======================= */
static bool ota_enabled(void) {
    return s_cfg.command_topics_enabled && s_cfg.action_ota_install;
}

static const char *installed_version(void) {
    return s_cfg.app_version ? s_cfg.app_version : "unknown";
}

static void ota_lock(bool *taken) {
    *taken = s_publish_lock && xSemaphoreTake(s_publish_lock, portMAX_DELAY) == pdTRUE;
}

static void ota_unlock(bool taken) {
    if (taken) xSemaphoreGive(s_publish_lock);
}

static void publish_ota_state(void) {
    if (!ota_enabled()) return;

    ota_manifest_t m;
    bool in_progress;
    int percent;
    char last_error[sizeof(s_ota_last_error)];
    bool taken; ota_lock(&taken);
    m = s_ota_manifest;
    in_progress = s_ota_in_progress;
    percent = s_ota_percent;
    memcpy(last_error, s_ota_last_error, sizeof(last_error));
    ota_unlock(taken);

    char installed_esc[64];
    char latest_esc[64];
    char summary_esc[320];
    json_escape(installed_version(), installed_esc, sizeof(installed_esc));
    json_escape(m.valid ? m.version : installed_version(), latest_esc, sizeof(latest_esc));

    char summary[160] = {0};
    if (last_error[0]) {
        snprintf(summary, sizeof(summary), "Last install failed: %s", last_error);
    } else if (m.valid && m.notes[0]) {
        snprintf(summary, sizeof(summary), "%s", m.notes);
    }
    json_escape(summary, summary_esc, sizeof(summary_esc));

    char json[640];
    int len = 0;
    json_appendf(json, sizeof(json), &len,
        "{\"installed_version\":\"%s\",\"latest_version\":\"%s\","
        "\"title\":\"LD2420 presence firmware\",\"in_progress\":%s,",
        installed_esc, latest_esc, in_progress ? "true" : "false");
    if (in_progress && percent >= 0) {
        json_appendf(json, sizeof(json), &len, "\"update_percentage\":%d,", percent);
    } else {
        json_appendf(json, sizeof(json), &len, "\"update_percentage\":null,");
    }
    // HA's MQTT update schema requires a string here; null is rejected and
    // drops the whole state update. "" clears a previous summary.
    json_appendf(json, sizeof(json), &len, "\"release_summary\":\"%s\"}", summary_esc);
    if (len < 0) {
        ESP_LOGW(TAG, "OTA state payload truncated");
        return;
    }
    pub(s_topic_ota_state, json, 1, 1);
}

static bool is_hex_string(const char *s, size_t want_len) {
    if (!s || strlen(s) != want_len) return false;
    for (size_t i = 0; i < want_len; ++i) {
        if (!isxdigit((unsigned char)s[i])) return false;
    }
    return true;
}

static bool is_safe_version(const char *s) {
    size_t n = s ? strlen(s) : 0;
    if (n == 0 || n >= sizeof(s_ota_manifest.version)) return false;
    for (size_t i = 0; i < n; ++i) {
        unsigned char c = (unsigned char)s[i];
        if (!isalnum(c) && c != '.' && c != '-' && c != '+' && c != '_') return false;
    }
    return true;
}

static bool is_safe_url(const char *s) {
    size_t n = s ? strlen(s) : 0;
    if (n == 0 || n >= sizeof(s_ota_manifest.url)) return false;
    if (strncmp(s, "http://", 7) != 0 && strncmp(s, "https://", 8) != 0) return false;
    for (size_t i = 0; i < n; ++i) {
        unsigned char c = (unsigned char)s[i];
        if (c <= ' ' || c >= 0x7f || c == '"' || c == '\\') return false;
    }
    return true;
}

// Manifest: {"version":"2.3.0","url":"http://...","sha256":"<64 hex>",
//            "size":1043584,"notes":"optional"}. An empty (cleared retained)
// payload forgets the manifest. Invalid manifests are ignored so a bad
// publish cannot erase a good one.
static void handle_ota_manifest(const char *payload, int payload_len) {
    ota_manifest_t next = {0};

    if (payload_len > 0) {
        cJSON *root = cJSON_ParseWithLength(payload, (size_t)payload_len);
        const cJSON *version = cJSON_GetObjectItemCaseSensitive(root, "version");
        const cJSON *url = cJSON_GetObjectItemCaseSensitive(root, "url");
        const cJSON *sha = cJSON_GetObjectItemCaseSensitive(root, "sha256");
        const cJSON *size = cJSON_GetObjectItemCaseSensitive(root, "size");
        const cJSON *notes = cJSON_GetObjectItemCaseSensitive(root, "notes");

        bool ok = cJSON_IsObject(root) &&
                  cJSON_IsString(version) && is_safe_version(version->valuestring) &&
                  cJSON_IsString(url) && is_safe_url(url->valuestring) &&
                  cJSON_IsString(sha) && is_hex_string(sha->valuestring, 64) &&
                  cJSON_IsNumber(size) && size->valuedouble >= 1 &&
                  size->valuedouble <= OTA_MAX_IMAGE_SIZE &&
                  (notes == NULL || cJSON_IsString(notes));
        if (ok) {
            next.valid = true;
            snprintf(next.version, sizeof(next.version), "%s", version->valuestring);
            snprintf(next.url, sizeof(next.url), "%s", url->valuestring);
            snprintf(next.sha256, sizeof(next.sha256), "%s", sha->valuestring);
            next.size = (uint32_t)size->valuedouble;
            if (notes) snprintf(next.notes, sizeof(next.notes), "%s", notes->valuestring);
        }
        cJSON_Delete(root);

        if (!ok) {
            ESP_LOGW(TAG, "Ignoring invalid OTA manifest");
            return;
        }
        ESP_LOGI(TAG, "OTA manifest: version %s (%" PRIu32 " bytes)", next.version, next.size);
    } else {
        ESP_LOGI(TAG, "OTA manifest cleared");
    }

    bool taken; ota_lock(&taken);
    s_ota_manifest = next;
    s_ota_last_error[0] = '\0';
    ota_unlock(taken);
    publish_ota_state();
}

static void handle_ota_install(void) {
    ota_manifest_t m;
    bool busy;
    bool taken; ota_lock(&taken);
    m = s_ota_manifest;
    busy = s_ota_in_progress;
    if (!busy && m.valid && strcmp(m.version, installed_version()) != 0) {
        s_ota_in_progress = true;
        s_ota_percent = 0;
        s_ota_last_error[0] = '\0';
    }
    ota_unlock(taken);

    if (busy) {
        ESP_LOGW(TAG, "OTA install ignored: already in progress");
        return;
    }
    if (!m.valid) {
        ESP_LOGW(TAG, "OTA install ignored: no manifest");
        return;
    }
    if (strcmp(m.version, installed_version()) == 0) {
        ESP_LOGW(TAG, "OTA install ignored: %s already installed", m.version);
        return;
    }

    ESP_LOGW(TAG, "OTA install requested: %s -> %s", installed_version(), m.version);
    publish_ota_state();
    if (!s_cfg.action_ota_install(m.url, m.sha256, m.size, m.version)) {
        ha_mqtt_publish_ota_result(false, "could not start download");
    }
}

/* ======================= MQTT event handling ======================= */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    (void)handler_args;
    (void)base;

    esp_mqtt_event_handle_t e = (esp_mqtt_event_handle_t)event_data;

    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            set_connected(true);
            ESP_LOGI(TAG, "MQTT connected (protocol=%d, session_present=%d)",
                     e ? e->protocol_ver : -1,
                     e ? e->session_present : 0);
            
            publish_discovery_all();
            publish_birth_online();
            publish_attrs_once();
            
            if (s_cfg.command_topics_enabled) {
                /* Subscribe to config commands and publish initial states */
                if (s_cfg.get_distance_thresh_cm && s_cfg.set_distance_thresh_cm) {
                    esp_mqtt_client_subscribe(s_client, s_topic_cfg_movement_thresh_cmd, 1);
                    char buf[16]; snprintf(buf, sizeof(buf), "%d", s_cfg.get_distance_thresh_cm());
                    pub(s_topic_cfg_movement_thresh_stat, buf, 1, 1);
                }
                if (s_cfg.get_hold_on_ms && s_cfg.set_hold_on_ms) {
                    esp_mqtt_client_subscribe(s_client, s_topic_cfg_presence_timeout_cmd, 1);
                    char buf[16];
                    // Convert milliseconds to seconds for display
                    snprintf(buf, sizeof(buf), "%d", s_cfg.get_hold_on_ms() / 1000);
                    pub(s_topic_cfg_presence_timeout_stat, buf, 1, 1);
                }

                /* Subscribe LD2420 tuning commands and publish initial states */
                if (s_cfg.get_ld_min_gate && s_cfg.set_ld_min_gate) {
                    esp_mqtt_client_subscribe(s_client, s_topic_cfg_ld_min_cmd, 1);
                }
                if (s_cfg.get_ld_max_gate && s_cfg.set_ld_max_gate) {
                    esp_mqtt_client_subscribe(s_client, s_topic_cfg_ld_max_cmd, 1);
                }
                if (s_cfg.get_ld_delay_s && s_cfg.set_ld_delay_s) {
                    esp_mqtt_client_subscribe(s_client, s_topic_cfg_ld_delay_cmd, 1);
                }
                if (s_cfg.get_sensitivity && s_cfg.set_sensitivity) {
                    esp_mqtt_client_subscribe(s_client, s_topic_cfg_sens_cmd, 1);
                }
                publish_ld2420_config_states();

                /* Action buttons */
                esp_mqtt_client_subscribe(s_client, s_topic_cmd_restart, 1);
                esp_mqtt_client_subscribe(s_client, s_topic_cmd_resend_disc, 1);
                if (s_cfg.action_apply_config) {
                    esp_mqtt_client_subscribe(s_client, s_topic_cmd_apply_cfg, 1);
                }

                if (s_cfg.action_ota_install) {
                    esp_mqtt_client_subscribe(s_client, s_topic_ota_manifest_cmd, 1);
                    esp_mqtt_client_subscribe(s_client, s_topic_ota_install_cmd, 1);
                    publish_ota_state();
                }

                /* Zone and smoothing defaults */
                if (s_publish_lock && xSemaphoreTake(s_publish_lock, portMAX_DELAY) == pdTRUE) {
                    normalize_zone_ranges_locked();
                    xSemaphoreGive(s_publish_lock);
                } else {
                    normalize_zone_ranges_locked();
                }
                for (int i = 0; i < ZONE_COUNT; ++i) {
                    esp_mqtt_client_subscribe(s_client, s_topic_cfg_zone_max_cmd[i], 1);
                }
                publish_zone_config_states();
                esp_mqtt_client_subscribe(s_client, s_topic_cfg_smooth_cmd, 1);
                char v[8]; snprintf(v, sizeof(v), "%d", s_smooth_win);
                pub(s_topic_cfg_smooth_stat, v, 1, 1);
            } else {
                ESP_LOGW(TAG, "MQTT command topics disabled; publishing telemetry only");
            }
            
        /* Re-send last presence state */
            bool cached_have = s_have_last;
            bool cached_present = s_last_present;
            int cached_distance = s_last_distance_mm;
            if (s_publish_lock && xSemaphoreTake(s_publish_lock, portMAX_DELAY) == pdTRUE) {
                cached_have = s_have_last;
                cached_present = s_last_present;
                cached_distance = s_last_distance_mm;
                xSemaphoreGive(s_publish_lock);
            }
            if (cached_have) {
                ha_mqtt_publish_presence(cached_present, cached_distance);
            } else {
                // Publish a baseline OFF state to avoid HA showing 'unknown'
                ha_mqtt_publish_presence(false, -1);
            }

            char fw_buf[sizeof(s_last_ld2420_fw_version)] = {0};
            bool publish_fw = false;
            if (s_publish_lock && xSemaphoreTake(s_publish_lock, portMAX_DELAY) == pdTRUE) {
                if (s_have_ld2420_fw_version) {
                    strncpy(fw_buf, s_last_ld2420_fw_version, sizeof(fw_buf) - 1);
                    fw_buf[sizeof(fw_buf) - 1] = '\0';
                    publish_fw = true;
                }
                xSemaphoreGive(s_publish_lock);
            } else if (s_have_ld2420_fw_version) {
                strncpy(fw_buf, s_last_ld2420_fw_version, sizeof(fw_buf) - 1);
                fw_buf[sizeof(fw_buf) - 1] = '\0';
                publish_fw = true;
            }
            if (publish_fw) {
                ha_mqtt_publish_ld2420_fw_version(fw_buf);
            }
            publish_boot_time();
            break;

        case MQTT_EVENT_DISCONNECTED:
            set_connected(false);
            ESP_LOGW(TAG, "MQTT disconnected");
            break;

        case MQTT_EVENT_ERROR:
            set_connected(false);
            log_mqtt_error_event(e);
            break;

        case MQTT_EVENT_DATA:
            if (e && e->data) {
                const char *t = NULL;
                const char *payload = NULL;
                int tlen = 0;
                int payload_len = 0;
                if (!mqtt_event_get_complete_payload(e, &t, &tlen, &payload, &payload_len)) {
                    break;
                }

                if (!s_cfg.command_topics_enabled) {
                    ESP_LOGW(TAG, "Ignoring MQTT command while command topics are disabled");
                    break;
                }

                // The OTA manifest is the one intentionally retained input: it
                // describes the latest release and is only acted on when HA
                // sends a separate, non-retained install command.
                if (s_cfg.action_ota_install &&
                    tlen == (int)strlen(s_topic_ota_manifest_cmd) &&
                    strncmp(t, s_topic_ota_manifest_cmd, tlen) == 0) {
                    handle_ota_manifest(payload, payload_len);
                    break;
                }

                // Reject retained command messages. The device only subscribes to
                // /cmd/* topics, so any retained inbound is a replay risk: a
                // retained PRESS on cmd/restart would re-trigger restart on every
                // reconnect (rate-limit window is uptime-relative and fresh after
                // boot), producing a boot loop until the broker drops the retain.
                if (e->retain) {
                    ESP_LOGW(TAG, "Ignoring retained command on %.*s", tlen, t);
                    break;
                }
                
                /* Handle movement threshold command */
                if (s_cfg.set_distance_thresh_cm && 
                    tlen == (int)strlen(s_topic_cfg_movement_thresh_cmd) && 
                    strncmp(t, s_topic_cfg_movement_thresh_cmd, tlen) == 0) {
                    
                    int v;
                    if (safe_atoi(payload, payload_len, &v, 1, 50)) {
                        s_cfg.set_distance_thresh_cm(v);
                        char buf[16]; 
                        snprintf(buf, sizeof(buf), "%d", v); 
                        pub(s_topic_cfg_movement_thresh_stat, buf, 1, 1);
                        ESP_LOGI(TAG, "Set movement threshold: %d cm", v);
                    } else {
                        ESP_LOGW(TAG, "Invalid movement threshold value");
                    }
                    
                /* Handle presence timeout command */
                } else if (s_cfg.set_hold_on_ms && 
                          tlen == (int)strlen(s_topic_cfg_presence_timeout_cmd) && 
                          strncmp(t, s_topic_cfg_presence_timeout_cmd, tlen) == 0) {
                    
                    int v;
                    if (safe_atoi(payload, payload_len, &v, 5, 300)) {
                        // Convert seconds from HA to milliseconds for internal use
                        s_cfg.set_hold_on_ms(v * 1000);
                        char buf[16]; 
                        snprintf(buf, sizeof(buf), "%d", v); // Keep as seconds for HA
                        pub(s_topic_cfg_presence_timeout_stat, buf, 1, 1);
                        ESP_LOGI(TAG, "Set presence timeout: %d sec", v);
                    } else {
                        ESP_LOGW(TAG, "Invalid presence timeout value");
                    }
                    
                /* LD2420 tuning. Distances arrive in metres, 70 cm per gate. */
                } else if (tlen == (int)strlen(s_topic_cfg_ld_min_cmd) && strncmp(t, s_topic_cfg_ld_min_cmd, tlen) == 0) {
                    int dm;
                    if (s_cfg.set_ld_min_gate && parse_decimetres(payload, payload_len, &dm) &&
                        dm <= 15 * GATE_DEPTH_CM / 10) {
                        int gate = (dm * 10 + GATE_DEPTH_CM / 2) / GATE_DEPTH_CM;
                        s_cfg.set_ld_min_gate(gate);
                        publish_ld2420_config_states();
                        ESP_LOGI(TAG, "Set LD2420 min gate: %d", gate);
                    } else {
                        ESP_LOGW(TAG, "Invalid minimum distance");
                    }
                } else if (tlen == (int)strlen(s_topic_cfg_ld_max_cmd) && strncmp(t, s_topic_cfg_ld_max_cmd, tlen) == 0) {
                    int dm;
                    if (s_cfg.set_ld_max_gate && parse_decimetres(payload, payload_len, &dm) &&
                        dm >= GATE_DEPTH_CM / 10 && dm <= 16 * GATE_DEPTH_CM / 10) {
                        int gate = (dm * 10 + GATE_DEPTH_CM / 2) / GATE_DEPTH_CM - 1;
                        s_cfg.set_ld_max_gate(gate);
                        publish_ld2420_config_states();
                        ESP_LOGI(TAG, "Set LD2420 max gate: %d", gate);
                    } else {
                        ESP_LOGW(TAG, "Invalid detection range");
                    }
                } else if (tlen == (int)strlen(s_topic_cfg_ld_delay_cmd) && strncmp(t, s_topic_cfg_ld_delay_cmd, tlen) == 0) {
                    int v;
                    if (s_cfg.set_ld_delay_s && safe_atoi(payload, payload_len, &v, 0, 3600)) {
                        s_cfg.set_ld_delay_s(v);
                        publish_ld2420_config_states();
                        ESP_LOGI(TAG, "Set LD2420 hold time: %d s", v);
                    } else {
                        ESP_LOGW(TAG, "Invalid radar hold time");
                    }
                } else if (tlen == (int)strlen(s_topic_cfg_sens_cmd) && strncmp(t, s_topic_cfg_sens_cmd, tlen) == 0) {
                    int level = -1;
                    for (int i = 0; i < 3; ++i) {
                        size_t n = strlen(SENSITIVITY_NAMES[i]);
                        if ((size_t)payload_len == n && strncmp(payload, SENSITIVITY_NAMES[i], n) == 0) level = i;
                    }
                    if (s_cfg.set_sensitivity && level >= 0) {
                        s_cfg.set_sensitivity(level);
                        ESP_LOGI(TAG, "Set sensitivity: %s", SENSITIVITY_NAMES[level]);
                    } else {
                        ESP_LOGW(TAG, "Ignoring sensitivity \"%.*s\"", payload_len, payload);
                    }
                    publish_sensitivity_state();
                }

                /* Zone ends (starts follow the previous zone) */
                for (int i = 0; i < ZONE_COUNT; ++i) {
                    if (tlen == (int)strlen(s_topic_cfg_zone_max_cmd[i]) &&
                        strncmp(t, s_topic_cfg_zone_max_cmd[i], tlen) == 0) {
                        int v;
                        if (safe_atoi(payload, payload_len, &v, 0, ZONE_DISTANCE_MAX_CM)) {
                            set_zone_end(i, v);
                        } else {
                            ESP_LOGW(TAG, "Invalid zone %d end value", i);
                        }
                        break;
                    }
                }

                /* Handle smoothing window command */
                if (tlen == (int)strlen(s_topic_cfg_smooth_cmd) &&
                    strncmp(t, s_topic_cfg_smooth_cmd, tlen) == 0) {
                    int v;
                    if (safe_atoi(payload, payload_len, &v, 1, 10)) {
                        bool lock_taken = false;
                        if (s_publish_lock) {
                            lock_taken = xSemaphoreTake(s_publish_lock, portMAX_DELAY) == pdTRUE;
                        }
                        s_smooth_win = v;
                        if (lock_taken) {
                            xSemaphoreGive(s_publish_lock);
                        }
                        if (s_cfg.save_setting) s_cfg.save_setting("smoothing", v);
                        char buf[16];
                        snprintf(buf, sizeof(buf), "%d", v);
                        pub(s_topic_cfg_smooth_stat, buf, 1, 1);
                        ESP_LOGI(TAG, "Set smoothing window: %d", v);
                    } else {
                        ESP_LOGW(TAG, "Invalid smoothing window value");
                    }
                }

                /* Action buttons */
                if (tlen == (int)strlen(s_topic_cmd_restart) && strncmp(t, s_topic_cmd_restart, tlen) == 0) {
                    if (payload_is_press(payload, payload_len)) {
                        int64_t now = esp_timer_get_time();
                        if (now - s_last_restart_us < RESTART_MIN_INTERVAL_US) {
                            ESP_LOGW(TAG, "Restart ignored: rate limited");
                        } else {
                            s_last_restart_us = now;
                            ESP_LOGW(TAG, "MQTT restart requested");
                            pub(s_topic_status, "offline", 1, 1);
                            vTaskDelay(pdMS_TO_TICKS(100));
                            esp_restart();
                        }
                    }
                } else if (tlen == (int)strlen(s_topic_cmd_resend_disc) && strncmp(t, s_topic_cmd_resend_disc, tlen) == 0) {
                    if (payload_is_press(payload, payload_len)) {
                        ESP_LOGI(TAG, "MQTT resend discovery requested");
                        ha_mqtt_resend_discovery();
                    }
                } else if (s_cfg.action_apply_config &&
                           tlen == (int)strlen(s_topic_cmd_apply_cfg) && strncmp(t, s_topic_cmd_apply_cfg, tlen) == 0) {
                    if (payload_is_press(payload, payload_len)) {
                        int64_t now = esp_timer_get_time();
                        if (now - s_last_apply_us < APPLY_MIN_INTERVAL_US) {
                            ESP_LOGW(TAG, "Apply config ignored: rate limited");
                        } else {
                            s_last_apply_us = now;
                            ESP_LOGI(TAG, "MQTT apply config requested");
                            if (s_cfg.action_apply_config) s_cfg.action_apply_config();
                        }
                    }
                } else if (s_cfg.action_ota_install &&
                           tlen == (int)strlen(s_topic_ota_install_cmd) &&
                           strncmp(t, s_topic_ota_install_cmd, tlen) == 0) {
                    if (payload_len == 7 && strncmp(payload, "install", 7) == 0) {
                        handle_ota_install();
                    }
                }
            }
            break;

        default:
            break;
    }
}

/* ======================= Public API ======================= */
void ha_mqtt_init(const ha_mqtt_cfg_t *cfg) {
    if (cfg) s_cfg = *cfg;

    if (s_publish_lock == NULL) {
        s_publish_lock = xSemaphoreCreateMutex();
        if (s_publish_lock == NULL) {
            ESP_LOGE(TAG, "Failed to create publish mutex");
        }
    }

    if (cfg && cfg->broker_uri && cfg->broker_uri[0]) {
        // If a CA cert is provided but URI uses mqtt://, upgrade to mqtts://
        const char *src = cfg->broker_uri;
        if (cfg->broker_ca_cert_pem && strncmp(src, "mqtt://", 7) == 0) {
            size_t rest_len = strlen(src) - 7;
            if (rest_len + 8 < sizeof(s_broker_uri)) { // "mqtts://" + rest + NUL
                snprintf(s_broker_uri, sizeof(s_broker_uri), "mqtts://%s", src + 7);
                s_cfg.broker_uri = s_broker_uri;
            } else {
                snprintf(s_broker_uri, sizeof(s_broker_uri), "%s", src);
                s_cfg.broker_uri = s_broker_uri;
            }
        } else {
            snprintf(s_broker_uri, sizeof(s_broker_uri), "%s", src);
            s_cfg.broker_uri = s_broker_uri;
        }
    }
    
    derive_ids_and_topics();
    load_persisted_settings();
    s_boot_epoch_s = 0;
    memset(&s_ota_manifest, 0, sizeof(s_ota_manifest));
    s_ota_in_progress = false;
    s_ota_percent = -1;
    s_ota_last_error[0] = '\0';
    s_last_diag_us = 0;
    
    /* Fresh measurement state: smoothing buffer, zone edges, cached presence */
    memset(s_smooth_ring, 0, sizeof(s_smooth_ring));
    s_smooth_count = 0;
    s_smooth_head = 0;
    memset(s_zone_last_on, 0, sizeof(s_zone_last_on));
    s_have_last = false;
    s_last_present = false;
    s_last_distance_mm = -1;
}

void ha_mqtt_start(void) {
    if (s_client) return;

    esp_mqtt_client_config_t mc = {
        .broker.address.uri = s_cfg.broker_uri ? s_cfg.broker_uri : "mqtt://mqtt.local",
        .broker.verification.certificate = s_cfg.broker_ca_cert_pem,
        .credentials = {
            .client_id = s_devid,
            .username = s_cfg.username,
            .authentication.password = s_cfg.password
        },
        .session = {
            .protocol_ver = MQTT_PROTOCOL_V_5,
            .keepalive = 15,
            .last_will = {
                .topic   = s_topic_status,
                .msg     = "offline",
                .qos     = 1,
                .retain  = 1
            }
        },
        .network = {
            .timeout_ms = 5000,
            .reconnect_timeout_ms = 5000,
        },
        .buffer = {
            .size = 2048,
            .out_size = 2048,
        },
    };

    s_client = esp_mqtt_client_init(&mc);
    if (!s_client) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return;
    }

#if CONFIG_MQTT_PROTOCOL_5
    esp_mqtt5_connection_property_config_t connect_props = {
        .session_expiry_interval = 0,
        .maximum_packet_size = 2048,
        .receive_maximum = 32,
        .topic_alias_maximum = 0,
        .request_resp_info = false,
        .request_problem_info = true,
    };
    esp_err_t mqtt5_err = esp_mqtt5_client_set_connect_property(s_client, &connect_props);
    if (mqtt5_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure MQTT 5 connect properties: %s",
                 esp_err_to_name(mqtt5_err));
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
        return;
    }
#endif
    
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_client);
}

void ha_mqtt_stop(void) {
    if (!s_client) return;
    esp_mqtt_client_stop(s_client);
    esp_mqtt_client_destroy(s_client);
    s_client = NULL;
    set_connected(false);
}

bool ha_mqtt_is_connected(void) {
    return is_connected_snapshot();
}

void ha_mqtt_reconnect_if_disconnected(void) {
    if (s_client && !is_connected_snapshot()) {
        ESP_LOGI(TAG, "Wi-Fi up, MQTT not connected - triggering immediate reconnect");
        esp_mqtt_client_reconnect(s_client);
    }
}

void ha_mqtt_publish_presence(bool present, int distance_mm) {
    bool publish_distance = false;
    int avg_mm_snapshot = -1;
    int zone_updates[3] = { -1, -1, -1 };

    bool lock_taken = false;
    if (s_publish_lock) {
        lock_taken = xSemaphoreTake(s_publish_lock, portMAX_DELAY) == pdTRUE;
    }

    if (!s_publish_lock || lock_taken) {
        // Cache last known state for reconnect scenarios
        s_have_last = true;
        s_last_present = present;
        s_last_distance_mm = distance_mm;

        if (distance_mm >= 0 && s_cfg.distance_supported) {
            // s_smooth_win is bounded to [1, 10] at the only setter via
            // safe_atoi range check; no runtime clamp needed here.

            s_smooth_ring[s_smooth_head] = distance_mm;
            s_smooth_head = (s_smooth_head + 1) % SMOOTH_BUFFER_SIZE;
            if (s_smooth_count < s_smooth_win) s_smooth_count++;

            int count = (s_smooth_count < s_smooth_win) ? s_smooth_count : s_smooth_win;
            int sum = 0;
            for (int i = 0; i < count; i++) {
                int idx = (s_smooth_head - 1 - i + SMOOTH_BUFFER_SIZE) % SMOOTH_BUFFER_SIZE;
                sum += s_smooth_ring[idx];
            }
            avg_mm_snapshot = (count ? sum / count : distance_mm);
            publish_distance = true;

            if (present) {
                int cur_cm = (avg_mm_snapshot + 5) / 10;
                for (int i = 0; i < ZONE_COUNT; ++i) {
                    int on = (cur_cm >= s_zone_min_cm[i] && cur_cm <= s_zone_max_cm[i]) ? 1 : 0;
                    if (on != s_zone_last_on[i]) {
                        s_zone_last_on[i] = on;
                        zone_updates[i] = on;
                    }
                }
            } else {
                for (int i = 0; i < ZONE_COUNT; ++i) {
                    if (s_zone_last_on[i]) {
                        s_zone_last_on[i] = 0;
                        zone_updates[i] = 0;
                    }
                }
            }
        } else if (!present) {
            for (int i = 0; i < ZONE_COUNT; ++i) {
                if (s_zone_last_on[i]) {
                    s_zone_last_on[i] = 0;
                    zone_updates[i] = 0;
                }
            }
        }

        if (lock_taken) {
            xSemaphoreGive(s_publish_lock);
        }
    } else {
        // Mutex unavailable: still keep cached state coherent
        s_have_last = true;
        s_last_present = present;
        s_last_distance_mm = distance_mm;
    }

    if (!is_connected_snapshot() || ota_download_active()) return;

    // Retain presence so HA keeps state across restarts
    pub(s_topic_presence, present ? "ON" : "OFF", 1, 1);
    pub(s_topic_status, "online", 1, 1);

    if (publish_distance && avg_mm_snapshot >= 0) {
        char buf[16];
        float cm = avg_mm_snapshot / 10.0f;
        snprintf(buf, sizeof(buf), "%.1f", cm);
        pub(s_topic_movement_distance, buf, 1, 1);
    }

    for (int i = 0; i < ZONE_COUNT; ++i) {
        if (zone_updates[i] != -1) {
            pub(s_topic_zone_movement[i], zone_updates[i] ? "ON" : "OFF", 1, 1);
        }
    }

    publish_periodic_diag_if_due();
}

void ha_mqtt_publish_rssi_now(void) {
    s_last_diag_us = 0;
    publish_periodic_diag_if_due();
}

void ha_mqtt_resend_discovery(void) {
    if (!is_connected_snapshot()) return;
    publish_discovery_all();
    publish_attrs_once();
}

void ha_mqtt_publish_ld2420_config_states(void) {
    if (!is_connected_snapshot()) return;
    publish_ld2420_config_states();
}

void ha_mqtt_publish_ld2420_fw_version(const char *version) {
    const char *incoming = (version && version[0]) ? version : "unknown";
    bool lock_taken = false;
    if (s_publish_lock) {
        lock_taken = xSemaphoreTake(s_publish_lock, portMAX_DELAY) == pdTRUE;
    }

    if (!s_publish_lock) {
        size_t len = strlen(incoming);
        if (len >= sizeof(s_last_ld2420_fw_version)) len = sizeof(s_last_ld2420_fw_version) - 1;
        memcpy(s_last_ld2420_fw_version, incoming, len);
        s_last_ld2420_fw_version[len] = '\0';
        s_have_ld2420_fw_version = true;
    } else {
        if (lock_taken) {
            size_t len = strlen(incoming);
            if (len >= sizeof(s_last_ld2420_fw_version)) len = sizeof(s_last_ld2420_fw_version) - 1;
            memcpy(s_last_ld2420_fw_version, incoming, len);
            s_last_ld2420_fw_version[len] = '\0';
            s_have_ld2420_fw_version = true;
            xSemaphoreGive(s_publish_lock);
        } else {
            ESP_LOGW(TAG, "publish_ld2420_fw_version: publish mutex unavailable");
            return;
        }
    }

    if (!is_connected_snapshot()) return;

    const char *to_send = (s_have_ld2420_fw_version && s_last_ld2420_fw_version[0]) ? s_last_ld2420_fw_version : incoming;
    pub(s_topic_fwver, to_send, 1, 1);
}

void ha_mqtt_publish_ota_progress(int percent) {
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;
    bool taken; ota_lock(&taken);
    s_ota_in_progress = true;
    s_ota_percent = percent;
    ota_unlock(taken);
    publish_ota_state();
}

void ha_mqtt_publish_ota_result(bool ok, const char *message) {
    bool taken; ota_lock(&taken);
    if (ok) {
        // Stay "in progress" at 100% until the restart; the new image then
        // reports its own installed_version.
        s_ota_in_progress = true;
        s_ota_percent = 100;
        s_ota_last_error[0] = '\0';
    } else {
        s_ota_in_progress = false;
        s_ota_percent = -1;
        snprintf(s_ota_last_error, sizeof(s_ota_last_error), "%s",
                 (message && message[0]) ? message : "unknown error");
    }
    ota_unlock(taken);
    publish_ota_state();
    if (ok) {
        // Restart follows; mark offline now instead of waiting for the LWT.
        pub(s_topic_status, "offline", 1, 1);
    } else {
        // Telemetry was paused during the download: catch HA up.
        bool have, present;
        int distance;
        bool t2 = s_publish_lock && xSemaphoreTake(s_publish_lock, portMAX_DELAY) == pdTRUE;
        have = s_have_last; present = s_last_present; distance = s_last_distance_mm;
        if (t2) xSemaphoreGive(s_publish_lock);
        if (have) ha_mqtt_publish_presence(present, distance);
    }
}

void ha_mqtt_publish_boot_time(int64_t boot_epoch_s) {
    bool taken = s_publish_lock && xSemaphoreTake(s_publish_lock, portMAX_DELAY) == pdTRUE;
    s_boot_epoch_s = boot_epoch_s;
    if (taken) xSemaphoreGive(s_publish_lock);
    if (is_connected_snapshot()) publish_boot_time();
}

void ha_mqtt_publish_sensitivity_state(void) {
    if (is_connected_snapshot()) publish_sensitivity_state();
}

/* Diagnostic hooks (no-op by default) */
void ha_mqtt_diag_publish_out(int raw, int active, int present) {
    (void)raw; (void)active; (void)present;
}

void ha_mqtt_diag_publish_uart(int alive, int baud) {
    (void)alive; (void)baud;
}

/* Direction events (LD2411) - unused in this build */
void ha_mqtt_publish_dir_approach(int on) {
    (void)on;
}

void ha_mqtt_publish_dir_away(int on) {
    (void)on;
}
