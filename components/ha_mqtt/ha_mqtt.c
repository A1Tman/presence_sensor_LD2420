#include "ha_mqtt.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>
#include <ctype.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "sensor_info.h"
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
    .app_version   = "1.1.1",
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
static char s_devid[32];                // presence-aabbcc
static char s_topic_base[64];           // presence/presence-aabbcc
static char s_topic_status[96];
static char s_topic_presence[96];
static char s_topic_movement_distance[96];
static char s_topic_attrs[96];
static char s_topic_rssi[96];
static char s_topic_uptime[96];
static char s_topic_fwver[96];

/* Config state + command topics */
static char s_topic_cfg_movement_thresh_stat[96];
static char s_topic_cfg_movement_thresh_cmd[96];
static char s_topic_cfg_presence_timeout_stat[96];
static char s_topic_cfg_presence_timeout_cmd[96];

/* LD2420 tuning (vendor params) */
static char s_topic_cfg_ld_min_stat[96];
static char s_topic_cfg_ld_min_cmd[96];
static char s_topic_cfg_ld_max_stat[96];
static char s_topic_cfg_ld_max_cmd[96];
static char s_topic_cfg_ld_delay_stat[96];
static char s_topic_cfg_ld_delay_cmd[96];
static char s_topic_cfg_ld_trig0_stat[96];
static char s_topic_cfg_ld_trig0_cmd[96];
static char s_topic_cfg_ld_hold0_stat[96];
static char s_topic_cfg_ld_hold0_cmd[96];
/* Command topics (actions) */
static char s_topic_cmd_restart[96];
static char s_topic_cmd_resend_disc[96];
static char s_topic_cmd_apply_cfg[96];

/* Distance smoothing + movement zones */
#define SMOOTH_BUFFER_SIZE 16
static int  s_smooth_win = 5;
static int  s_smooth_ring[SMOOTH_BUFFER_SIZE];
static int  s_smooth_count = 0;
static int  s_smooth_head = 0;
static int  s_zone_min_cm[3] = {0, 100, 200};  // Near, Mid, Far
static int  s_zone_max_cm[3] = {99, 199, 400};
static int  s_zone_last_on[3] = {0, 0, 0};
static char s_topic_zone_movement[3][96];
static char s_topic_cfg_zone_min_stat[3][96];
static char s_topic_cfg_zone_min_cmd[3][96];
static char s_topic_cfg_zone_max_stat[3][96];
static char s_topic_cfg_zone_max_cmd[3][96];
static char s_topic_cfg_smooth_stat[96];
static char s_topic_cfg_smooth_cmd[96];

/* HA Discovery topics */
static char s_disc_bs_presence[128];
static char s_disc_sensor_movement[128];
static char s_disc_sensor_rssi[128];
static char s_disc_sensor_uptime[128];
static char s_disc_prefix[64] = "homeassistant";
static char s_disc_button_restart[128];
static char s_disc_button_resend_disc[128];
static char s_disc_button_apply_cfg[128];
static char s_disc_sensor_fwver[128];
static bool s_restart_migration_done = false;

/* Cached state for reconnect */
static bool s_have_last = false;
static bool s_last_present = false;
static int  s_last_distance_mm = -1;
static bool s_have_fw_version = false;
static char s_last_fw_version[64] = {0};

/* Uptime ticker */
static int64_t s_boot_us = 0;
static int64_t s_last_diag_us = 0;
static int64_t s_last_restart_us = 0;
static int64_t s_last_apply_us = 0;

#define RESTART_MIN_INTERVAL_US (30LL * 1000000LL)
#define APPLY_MIN_INTERVAL_US   (5LL  * 1000000LL)

/* ======================= Utilities ======================= */
static void mac_to_str(uint8_t mac[6], char out[18]) {
    snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
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
    char *end = start + strlen(start) - 1;
    while (end > start && isspace((unsigned char)*end)) *end-- = '\0';
    
    if (*start == '\0') return false;
    
    char *endptr;
    long val = strtol(start, &endptr, 10);
    
    if (*endptr != '\0') return false;
    if (val < min || val > max) {
        val = (val < min) ? min : max;
    }
    
    *out_val = (int)val;
    return true;
}

static void derive_ids_and_topics(void) {
    uint8_t mac[6] = {0};
    esp_efuse_mac_get_default(mac);
    mac_to_str(mac, s_mac_str);

    /* short hex id (last 3 bytes) */
    snprintf(s_devid, sizeof(s_devid), "presence-%02x%02x%02x", mac[3], mac[4], mac[5]);

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
    snprintf(s_topic_uptime, sizeof(s_topic_uptime), "%s/uptime_s", s_topic_base);
    snprintf(s_topic_fwver, sizeof(s_topic_fwver), "%s/ld2420/fw_version", s_topic_base);
    
    snprintf(s_topic_cfg_movement_thresh_stat, sizeof(s_topic_cfg_movement_thresh_stat), "%s/cfg/movement_threshold_cm", s_topic_base);
    snprintf(s_topic_cfg_movement_thresh_cmd, sizeof(s_topic_cfg_movement_thresh_cmd), "%s/cmd/movement_threshold_cm", s_topic_base);
    snprintf(s_topic_cfg_presence_timeout_stat, sizeof(s_topic_cfg_presence_timeout_stat), "%s/cfg/presence_timeout_sec", s_topic_base);
    snprintf(s_topic_cfg_presence_timeout_cmd, sizeof(s_topic_cfg_presence_timeout_cmd), "%s/cmd/presence_timeout_sec", s_topic_base);

    /* LD2420 tuning */
    snprintf(s_topic_cfg_ld_min_stat, sizeof(s_topic_cfg_ld_min_stat), "%s/cfg/ld2420/min_gate", s_topic_base);
    snprintf(s_topic_cfg_ld_min_cmd, sizeof(s_topic_cfg_ld_min_cmd), "%s/cmd/ld2420/min_gate", s_topic_base);
    snprintf(s_topic_cfg_ld_max_stat, sizeof(s_topic_cfg_ld_max_stat), "%s/cfg/ld2420/max_gate", s_topic_base);
    snprintf(s_topic_cfg_ld_max_cmd, sizeof(s_topic_cfg_ld_max_cmd), "%s/cmd/ld2420/max_gate", s_topic_base);
    snprintf(s_topic_cfg_ld_delay_stat, sizeof(s_topic_cfg_ld_delay_stat), "%s/cfg/ld2420/delay_time", s_topic_base);
    snprintf(s_topic_cfg_ld_delay_cmd, sizeof(s_topic_cfg_ld_delay_cmd), "%s/cmd/ld2420/delay_time", s_topic_base);
    snprintf(s_topic_cfg_ld_trig0_stat, sizeof(s_topic_cfg_ld_trig0_stat), "%s/cfg/ld2420/trigger_sens", s_topic_base);
    snprintf(s_topic_cfg_ld_trig0_cmd, sizeof(s_topic_cfg_ld_trig0_cmd), "%s/cmd/ld2420/trigger_sens", s_topic_base);
    snprintf(s_topic_cfg_ld_hold0_stat, sizeof(s_topic_cfg_ld_hold0_stat), "%s/cfg/ld2420/maintain_sens", s_topic_base);
    snprintf(s_topic_cfg_ld_hold0_cmd, sizeof(s_topic_cfg_ld_hold0_cmd), "%s/cmd/ld2420/maintain_sens", s_topic_base);

    /* Movement zones */
    const char* zone_names[] = {"near", "mid", "far"};
    for (int i = 0; i < 3; ++i) {
        snprintf(s_topic_zone_movement[i], sizeof(s_topic_zone_movement[i]), "%s/movement/%s_range", s_topic_base, zone_names[i]);
        snprintf(s_topic_cfg_zone_min_stat[i], sizeof(s_topic_cfg_zone_min_stat[i]), "%s/cfg/zone/%s/min_cm", s_topic_base, zone_names[i]);
        snprintf(s_topic_cfg_zone_min_cmd[i], sizeof(s_topic_cfg_zone_min_cmd[i]), "%s/cmd/zone/%s/min_cm", s_topic_base, zone_names[i]);
        snprintf(s_topic_cfg_zone_max_stat[i], sizeof(s_topic_cfg_zone_max_stat[i]), "%s/cfg/zone/%s/max_cm", s_topic_base, zone_names[i]);
        snprintf(s_topic_cfg_zone_max_cmd[i], sizeof(s_topic_cfg_zone_max_cmd[i]), "%s/cmd/zone/%s/max_cm", s_topic_base, zone_names[i]);
    }
    snprintf(s_topic_cfg_smooth_stat, sizeof(s_topic_cfg_smooth_stat), "%s/cfg/distance_smoothing", s_topic_base);
    snprintf(s_topic_cfg_smooth_cmd, sizeof(s_topic_cfg_smooth_cmd), "%s/cmd/distance_smoothing", s_topic_base);
    /* Action command topics */
    snprintf(s_topic_cmd_restart, sizeof(s_topic_cmd_restart), "%s/cmd/restart", s_topic_base);
    snprintf(s_topic_cmd_resend_disc, sizeof(s_topic_cmd_resend_disc), "%s/cmd/resend_discovery", s_topic_base);
    snprintf(s_topic_cmd_apply_cfg, sizeof(s_topic_cmd_apply_cfg), "%s/cmd/apply_config", s_topic_base);

    /* Home Assistant discovery topics */
    snprintf(s_disc_bs_presence, sizeof(s_disc_bs_presence),
             "%s/binary_sensor/%s/presence/config", s_disc_prefix, s_devid);
    snprintf(s_disc_sensor_movement, sizeof(s_disc_sensor_movement),
             "%s/sensor/%s/movement_distance/config", s_disc_prefix, s_devid);
    snprintf(s_disc_sensor_rssi, sizeof(s_disc_sensor_rssi),
             "%s/sensor/%s/rssi/config", s_disc_prefix, s_devid);
    snprintf(s_disc_sensor_uptime, sizeof(s_disc_sensor_uptime),
             "%s/sensor/%s/uptime/config", s_disc_prefix, s_devid);
    snprintf(s_disc_sensor_fwver, sizeof(s_disc_sensor_fwver),
             "%s/sensor/%s/ld_fw/config", s_disc_prefix, s_devid);
    snprintf(s_disc_button_restart, sizeof(s_disc_button_restart),
             "%s/button/%s/restart/config", s_disc_prefix, s_devid);
    snprintf(s_disc_button_resend_disc, sizeof(s_disc_button_resend_disc),
             "%s/button/%s/resend_discovery/config", s_disc_prefix, s_devid);
    snprintf(s_disc_button_apply_cfg, sizeof(s_disc_button_apply_cfg),
             "%s/button/%s/apply_config/config", s_disc_prefix, s_devid);
}

static void pub(const char *topic, const char *payload, int qos, int retain) {
    if (!s_client || !s_connected) return;
    int mid = esp_mqtt_client_publish(s_client, topic, payload, 0, qos, retain);
    if (mid < 0) ESP_LOGW(TAG, "Publish failed to %s", topic);
}

/* ======================= Discovery payloads ======================= */
/* Forward declaration for helper used below */
static void json_escape(const char *in, char *out, size_t out_size);
static void publish_discovery_all(void) {
    char dev_block[768];
    char area[96] = {0};
    char dev_name_esc[128];
    char dev_model_esc[128];
    char app_ver_esc[64];
    char area_val_esc[64];
    const char *dev_name = s_cfg.friendly_name ? s_cfg.friendly_name : "Radar Sensor";
    const char *dev_model = s_cfg.device_model ? s_cfg.device_model : "HLK-LD2420 + ESP32";

    json_escape(dev_name, dev_name_esc, sizeof(dev_name_esc));
    json_escape(dev_model, dev_model_esc, sizeof(dev_model_esc));
    json_escape(s_cfg.app_version ? s_cfg.app_version : "1.0.0", app_ver_esc, sizeof(app_ver_esc));

    if (s_cfg.suggested_area && s_cfg.suggested_area[0]) {
        json_escape(s_cfg.suggested_area, area_val_esc, sizeof(area_val_esc));
        snprintf(area, sizeof(area), ",\"suggested_area\":\"%s\"", area_val_esc);
    }

    /* Device object */
    snprintf(dev_block, sizeof(dev_block),
        "\"dev\":{\"ids\":[\"%s\"],\"name\":\"%s\",\"mf\":\"Hi-Link + DIY\","
        "\"mdl\":\"%s\",\"sw\":\"%s\","
        "\"connections\":[[\"mac\",\"%s\"]]}%s",
        s_devid, dev_name_esc,
        dev_model_esc,
        app_ver_esc,
        s_mac_str,
        area
    );

    /* Presence (binary_sensor) */
    {
        char payload[1024];
        int len = 0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{"
            "\"name\":\"Presence\","
            "\"uniq_id\":\"%s_presence\","
            "\"stat_t\":\"%s\","
            "\"dev_cla\":\"occupancy\","
            "\"pl_on\":\"ON\",\"pl_off\":\"OFF\","
            "\"avty_t\":\"%s\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\","
            "\"json_attr_t\":\"%s\","
            "\"obj_id\":\"presence\",",
            s_devid, s_topic_presence,
            s_topic_status,
            s_topic_attrs
        );
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        pub(s_disc_bs_presence, payload, 1, 1);
    }

    /* Distance (sensor) - only if supported */
    if (s_cfg.distance_supported) {
        char payload[1024];
        int len = 0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{"
            "\"name\":\"Distance\","
            "\"uniq_id\":\"%s_movement_distance\","
            "\"stat_t\":\"%s\","
            "\"dev_cla\":\"distance\","
            "\"unit_of_meas\":\"cm\",\"stat_cla\":\"measurement\","
            "\"avty_t\":\"%s\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\","
            "\"obj_id\":\"distance\",",
            s_devid, s_topic_movement_distance,
            s_topic_status
        );
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        pub(s_disc_sensor_movement, payload, 1, 1);
    }

    /* Signal (sensor) - diagnostic */
    {
        char payload[1024];
        int len = 0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{"
            "\"name\":\"Signal\","
            "\"uniq_id\":\"%s_rssi\","
            "\"stat_t\":\"%s\","
            "\"dev_cla\":\"signal_strength\",\"unit_of_meas\":\"dBm\",\"ent_cat\":\"diagnostic\","
            "\"avty_t\":\"%s\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\","
            "\"obj_id\":\"signal\",",
            s_devid, s_topic_rssi,
            s_topic_status
        );
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        pub(s_disc_sensor_rssi, payload, 1, 1);
    }

    /* Uptime (sensor) - diagnostic */
    {
        char payload[1024];
        int len = 0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{"
            "\"name\":\"Uptime\","
            "\"uniq_id\":\"%s_uptime\","
            "\"stat_t\":\"%s\","
            "\"unit_of_meas\":\"s\",\"stat_cla\":\"measurement\",\"ent_cat\":\"diagnostic\","
            "\"avty_t\":\"%s\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\","
            "\"obj_id\":\"uptime\",",
            s_devid, s_topic_uptime,
            s_topic_status
        );
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        pub(s_disc_sensor_uptime, payload, 1, 1);
    }

    /* LD2420 Firmware Version (sensor) - diagnostic */
    {
        char payload[1024];
        int len = 0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{"
            "\"name\":\"LD2420 Firmware\","
            "\"uniq_id\":\"%s_ld_fw\","
            "\"stat_t\":\"%s\","
            "\"avty_t\":\"%s\",\"pl_avail\":\"online\",\"pl_not_avail\":\"offline\","
            "\"ent_cat\":\"diagnostic\",\"obj_id\":\"ld_fw\",",
            s_devid, s_topic_fwver, s_topic_status);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        pub(s_disc_sensor_fwver, payload, 1, 1);
    }

    /* Movement Threshold config (number) */
    if (s_cfg.get_distance_thresh_cm && s_cfg.set_distance_thresh_cm) {
        char payload[512]; int len=0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{\"name\":\"Movement Threshold\",\"uniq_id\":\"%s_movement_thresh\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\",\"min\":1,\"max\":50,\"step\":1,\"mode\":\"slider\",\"unit_of_meas\":\"cm\",\"ent_cat\":\"config\",",
            s_devid, s_topic_cfg_movement_thresh_cmd, s_topic_cfg_movement_thresh_stat);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[192]; snprintf(disc, sizeof(disc), "%s/number/%s/movement_thresh/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
    }

    /* Hold Time config (number) */
    if (s_cfg.get_hold_on_ms && s_cfg.set_hold_on_ms) {
        char payload[512]; int len=0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{\"name\":\"Hold Time\",\"uniq_id\":\"%s_presence_timeout\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\",\"min\":5,\"max\":300,\"step\":5,\"mode\":\"slider\",\"unit_of_meas\":\"s\",\"ent_cat\":\"config\",",
            s_devid, s_topic_cfg_presence_timeout_cmd, s_topic_cfg_presence_timeout_stat);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[192]; snprintf(disc, sizeof(disc), "%s/number/%s/presence_timeout/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
    }

    /* Min Range (LD2420) */
    if (s_cfg.get_ld_min_gate && s_cfg.set_ld_min_gate) {
        char payload[512]; int len=0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{\"name\":\"Min Range\",\"uniq_id\":\"%s_ld_min_gate\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\",\"min\":0,\"max\":15,\"step\":1,\"mode\":\"slider\",\"ent_cat\":\"config\",",
            s_devid, s_topic_cfg_ld_min_cmd, s_topic_cfg_ld_min_stat);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[192]; snprintf(disc, sizeof(disc), "%s/number/%s/ld_min_gate/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
    }
    
    /* Max Range (LD2420) */
    if (s_cfg.get_ld_max_gate && s_cfg.set_ld_max_gate) {
        char payload[512]; int len=0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{\"name\":\"Max Range\",\"uniq_id\":\"%s_ld_max_gate\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\",\"min\":0,\"max\":15,\"step\":1,\"mode\":\"slider\",\"ent_cat\":\"config\",",
            s_devid, s_topic_cfg_ld_max_cmd, s_topic_cfg_ld_max_stat);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[192]; snprintf(disc, sizeof(disc), "%s/number/%s/ld_max_gate/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
    }
    
    /* Response Delay (LD2420) */
    if (s_cfg.get_ld_delay_ms && s_cfg.set_ld_delay_ms) {
        char payload[512]; int len=0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{\"name\":\"Response Delay\",\"uniq_id\":\"%s_ld_delay\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\",\"min\":0,\"max\":65535,\"step\":100,\"mode\":\"slider\",\"ent_cat\":\"config\",",
            s_devid, s_topic_cfg_ld_delay_cmd, s_topic_cfg_ld_delay_stat);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[192]; snprintf(disc, sizeof(disc), "%s/number/%s/ld_delay/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
    }
    
    /* Trigger Level (LD2420) */
    if (s_cfg.get_ld_trigger_sens && s_cfg.set_ld_trigger_sens) {
        char payload[512]; int len=0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{\"name\":\"Trigger Level\",\"uniq_id\":\"%s_ld_trig_sens\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\",\"min\":0,\"max\":65535,\"step\":10,\"mode\":\"slider\",\"ent_cat\":\"config\",",
            s_devid, s_topic_cfg_ld_trig0_cmd, s_topic_cfg_ld_trig0_stat);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[192]; snprintf(disc, sizeof(disc), "%s/number/%s/ld_trig_sens/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
    }
    
    /* Tracking Level (LD2420) */
    if (s_cfg.get_ld_maintain_sens && s_cfg.set_ld_maintain_sens) {
        char payload[512]; int len=0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{\"name\":\"Tracking Level\",\"uniq_id\":\"%s_ld_maint_sens\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\",\"min\":0,\"max\":65535,\"step\":10,\"mode\":\"slider\",\"ent_cat\":\"config\",",
            s_devid, s_topic_cfg_ld_hold0_cmd, s_topic_cfg_ld_hold0_stat);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[192]; snprintf(disc, sizeof(disc), "%s/number/%s/ld_maint_sens/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
    }

    /* Movement zone binary_sensors */
    const char* zone_names[] = {"Near Zone", "Mid Zone", "Far Zone"};
    for (int i = 0; i < 3; ++i) {
        char payload[1024]; int len=0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{\"name\":\"%s\",\"uniq_id\":\"%s_movement_%d\",\"stat_t\":\"%s\",\"avty_t\":\"%s\",\"pl_on\":\"ON\",\"pl_off\":\"OFF\",\"obj_id\":\"zone_%d\",",
            zone_names[i], s_devid, i+1, s_topic_zone_movement[i], s_topic_status, i+1);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[192]; snprintf(disc, sizeof(disc), "%s/binary_sensor/%s/movement_%d/config", s_disc_prefix, s_devid, i+1);
        pub(disc, payload, 1, 1);
    }

    /* Zone range configuration */
    const char* zone_display_names[] = {"Near", "Mid", "Far"};
    for (int i = 0; i < 3; ++i) {
        const char* zone_names_lower[] = {"near", "mid", "far"};
        // Start Distance
        {
            char payload[512]; int len=0;
            len += snprintf(payload+len, sizeof(payload)-len,
                "{\"name\":\"%s Start\",\"uniq_id\":\"%s_%s_min\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\",\"min\":0,\"max\":600,\"step\":10,\"mode\":\"slider\",\"unit_of_meas\":\"cm\",\"ent_cat\":\"config\",",
                zone_display_names[i], s_devid, zone_names_lower[i], s_topic_cfg_zone_min_cmd[i], s_topic_cfg_zone_min_stat[i]);
            len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
            len += snprintf(payload+len, sizeof(payload)-len, "}");
            char disc[192]; snprintf(disc, sizeof(disc), "%s/number/%s/%s_min/config", s_disc_prefix, s_devid, zone_names_lower[i]);
            pub(disc, payload, 1, 1);
        }
        // End Distance
        {
            char payload[512]; int len=0;
            len += snprintf(payload+len, sizeof(payload)-len,
                "{\"name\":\"%s End\",\"uniq_id\":\"%s_%s_max\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\",\"min\":0,\"max\":600,\"step\":10,\"mode\":\"slider\",\"unit_of_meas\":\"cm\",\"ent_cat\":\"config\",",
                zone_display_names[i], s_devid, zone_names_lower[i], s_topic_cfg_zone_max_cmd[i], s_topic_cfg_zone_max_stat[i]);
            len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
            len += snprintf(payload+len, sizeof(payload)-len, "}");
            char disc[192]; snprintf(disc, sizeof(disc), "%s/number/%s/%s_max/config", s_disc_prefix, s_devid, zone_names_lower[i]);
            pub(disc, payload, 1, 1);
        }
    }

    /* Averaging (Distance Smoothing) */
    {
        char payload[512]; int len=0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{\"name\":\"Averaging\",\"uniq_id\":\"%s_dist_smooth\",\"cmd_t\":\"%s\",\"stat_t\":\"%s\",\"min\":1,\"max\":10,\"step\":1,\"mode\":\"slider\",\"ent_cat\":\"config\",",
            s_devid, s_topic_cfg_smooth_cmd, s_topic_cfg_smooth_stat);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        char disc[192]; snprintf(disc, sizeof(disc), "%s/number/%s/distance_smoothing/config", s_disc_prefix, s_devid);
        pub(disc, payload, 1, 1);
    }

    /* Action buttons */
    {
        char payload[512]; int len=0;
        if (!s_restart_migration_done) {
            esp_mqtt_client_publish(s_client, s_disc_button_restart, "", 0, 1, 1);
            s_restart_migration_done = true;
        }
        len += snprintf(payload+len, sizeof(payload)-len,
            "{\"name\":\"Restart\",\"uniq_id\":\"%s_restart\",\"cmd_t\":\"%s\",\"entity_category\":\"diagnostic\",",
            s_devid, s_topic_cmd_restart);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        pub(s_disc_button_restart, payload, 1, 1);
    }
    {
        char payload[512]; int len=0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{\"name\":\"Resend Discovery\",\"uniq_id\":\"%s_resend_disc\",\"cmd_t\":\"%s\",\"entity_category\":\"diagnostic\",",
            s_devid, s_topic_cmd_resend_disc);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        pub(s_disc_button_resend_disc, payload, 1, 1);
    }
    {
        char payload[512]; int len=0;
        len += snprintf(payload+len, sizeof(payload)-len,
            "{\"name\":\"Apply Config\",\"uniq_id\":\"%s_apply_cfg\",\"cmd_t\":\"%s\",\"entity_category\":\"config\",",
            s_devid, s_topic_cmd_apply_cfg);
        len += snprintf(payload+len, sizeof(payload)-len, "%s", dev_block);
        len += snprintf(payload+len, sizeof(payload)-len, "}");
        pub(s_disc_button_apply_cfg, payload, 1, 1);
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
    if (n == 0) return true; // empty payload treated as press
    if (n == 1 && (data[s] == '1' || data[s] == 'P' || data[s] == 'p')) return true;
    if (n == 2 && (data[s] == 'O' || data[s] == 'o') && (data[s+1] == 'N' || data[s+1] == 'n')) return true; // ON
    if (n == 5 && (strncasecmp(&data[s], "PRESS", 5) == 0)) return true;
    if (n == 5 && (strncasecmp(&data[s], "press", 5) == 0)) return true;
    return false;
}

static void publish_birth_online(void) {
    pub(s_topic_status, "online", 1, 1);
}

/* Periodic diagnostics: uptime + RSSI */
static void publish_periodic_diag_if_due(void) {
    const int64_t now_us = esp_timer_get_time();
    const int64_t interval = 30000000LL; // 30 seconds
    if (now_us - s_last_diag_us < interval) return;
    s_last_diag_us = now_us;

    int64_t uptime_s = (now_us - s_boot_us) / 1000000LL;
    char buf[32];
    snprintf(buf, sizeof(buf), "%" PRId64, uptime_s);
    pub(s_topic_uptime, buf, 0, 0);

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
    
    int len = 0;
    len += snprintf(json+len, sizeof(json)-len, "{");
    len += snprintf(json+len, sizeof(json)-len, "\"mac\":\"%s\",", s_mac_str);
    len += snprintf(json+len, sizeof(json)-len, "\"device_id\":\"%s\",", s_devid);
    len += snprintf(json+len, sizeof(json)-len, "\"model\":\"%s\",", 
                    s_cfg.device_model ? s_cfg.device_model : "HLK-LD2420 + ESP32");
    len += snprintf(json+len, sizeof(json)-len, "\"sw_version\":\"%s\",", 
                    s_cfg.app_version ? s_cfg.app_version : "1.0.0");
    len += snprintf(json+len, sizeof(json)-len, "\"ip\":\"%s\"", ip_str);
    len += snprintf(json+len, sizeof(json)-len, "}");
    pub(s_topic_attrs, json, 0, 0);
}

/* ======================= MQTT event handling ======================= */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t e = (esp_mqtt_event_handle_t)event_data;

    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            s_connected = true;
            ESP_LOGI(TAG, "MQTT connected");
            
            publish_discovery_all();
            publish_birth_online();
            publish_attrs_once();
            
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
                char buf[16]; snprintf(buf, sizeof(buf), "%d", s_cfg.get_ld_min_gate());
                pub(s_topic_cfg_ld_min_stat, buf, 1, 1);
            }
            if (s_cfg.get_ld_max_gate && s_cfg.set_ld_max_gate) {
                esp_mqtt_client_subscribe(s_client, s_topic_cfg_ld_max_cmd, 1);
                char buf[16]; snprintf(buf, sizeof(buf), "%d", s_cfg.get_ld_max_gate());
                pub(s_topic_cfg_ld_max_stat, buf, 1, 1);
            }
            if (s_cfg.get_ld_delay_ms && s_cfg.set_ld_delay_ms) {
                esp_mqtt_client_subscribe(s_client, s_topic_cfg_ld_delay_cmd, 1);
                char buf[16]; snprintf(buf, sizeof(buf), "%d", s_cfg.get_ld_delay_ms());
                pub(s_topic_cfg_ld_delay_stat, buf, 1, 1);
            }
            if (s_cfg.get_ld_trigger_sens && s_cfg.set_ld_trigger_sens) {
                esp_mqtt_client_subscribe(s_client, s_topic_cfg_ld_trig0_cmd, 1);
                char buf[16]; snprintf(buf, sizeof(buf), "%d", s_cfg.get_ld_trigger_sens());
                pub(s_topic_cfg_ld_trig0_stat, buf, 1, 1);
            }
            if (s_cfg.get_ld_maintain_sens && s_cfg.set_ld_maintain_sens) {
                esp_mqtt_client_subscribe(s_client, s_topic_cfg_ld_hold0_cmd, 1);
                char buf[16]; snprintf(buf, sizeof(buf), "%d", s_cfg.get_ld_maintain_sens());
                pub(s_topic_cfg_ld_hold0_stat, buf, 1, 1);
            }

            /* Action buttons */
            esp_mqtt_client_subscribe(s_client, s_topic_cmd_restart, 1);
            esp_mqtt_client_subscribe(s_client, s_topic_cmd_resend_disc, 1);
            esp_mqtt_client_subscribe(s_client, s_topic_cmd_apply_cfg, 1);
            
            /* Zone and smoothing defaults */
            for (int i = 0; i < 3; ++i) {
                esp_mqtt_client_subscribe(s_client, s_topic_cfg_zone_min_cmd[i], 1);
                esp_mqtt_client_subscribe(s_client, s_topic_cfg_zone_max_cmd[i], 1);
                char v[8]; 
                snprintf(v, sizeof(v), "%d", s_zone_min_cm[i]); 
                pub(s_topic_cfg_zone_min_stat[i], v, 1, 1);
                snprintf(v, sizeof(v), "%d", s_zone_max_cm[i]); 
                pub(s_topic_cfg_zone_max_stat[i], v, 1, 1);
            }
            esp_mqtt_client_subscribe(s_client, s_topic_cfg_smooth_cmd, 1);
            char v[8]; snprintf(v, sizeof(v), "%d", s_smooth_win); 
            pub(s_topic_cfg_smooth_stat, v, 1, 1);
            
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

            char fw_buf[sizeof(s_last_fw_version)] = {0};
            bool publish_fw = false;
            if (s_publish_lock && xSemaphoreTake(s_publish_lock, portMAX_DELAY) == pdTRUE) {
                if (s_have_fw_version) {
                    strncpy(fw_buf, s_last_fw_version, sizeof(fw_buf) - 1);
                    fw_buf[sizeof(fw_buf) - 1] = '\0';
                    publish_fw = true;
                }
                xSemaphoreGive(s_publish_lock);
            } else if (s_have_fw_version) {
                strncpy(fw_buf, s_last_fw_version, sizeof(fw_buf) - 1);
                fw_buf[sizeof(fw_buf) - 1] = '\0';
                publish_fw = true;
            }
            if (publish_fw) {
                ha_mqtt_publish_fw_version(fw_buf);
            }
            break;

        case MQTT_EVENT_DISCONNECTED:
            s_connected = false;
            ESP_LOGW(TAG, "MQTT disconnected");
            break;

        case MQTT_EVENT_DATA:
            if (e && e->topic && e->data) {
                const char *t = e->topic; 
                int tlen = e->topic_len;
                
                /* Handle movement threshold command */
                if (s_cfg.set_distance_thresh_cm && 
                    tlen == (int)strlen(s_topic_cfg_movement_thresh_cmd) && 
                    strncmp(t, s_topic_cfg_movement_thresh_cmd, tlen) == 0) {
                    
                    int v;
                    if (safe_atoi(e->data, e->data_len, &v, 1, 50)) {
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
                    if (safe_atoi(e->data, e->data_len, &v, 5, 300)) {
                        // Convert seconds from HA to milliseconds for internal use
                        s_cfg.set_hold_on_ms(v * 1000);
                        char buf[16]; 
                        snprintf(buf, sizeof(buf), "%d", v); // Keep as seconds for HA
                        pub(s_topic_cfg_presence_timeout_stat, buf, 1, 1);
                        ESP_LOGI(TAG, "Set presence timeout: %d sec", v);
                    } else {
                        ESP_LOGW(TAG, "Invalid presence timeout value");
                    }
                    
                /* Handle LD2420 tuning commands (typed setters) */
                } else if (tlen == (int)strlen(s_topic_cfg_ld_min_cmd) && strncmp(t, s_topic_cfg_ld_min_cmd, tlen) == 0) {
                    if (s_cfg.set_ld_min_gate) {
                        int v; if (safe_atoi(e->data, e->data_len, &v, 0, 15)) {
                            s_cfg.set_ld_min_gate(v);
                            char buf[16]; snprintf(buf, sizeof(buf), "%d", s_cfg.get_ld_min_gate ? s_cfg.get_ld_min_gate() : v);
                            pub(s_topic_cfg_ld_min_stat, buf, 1, 1);
                            ESP_LOGI(TAG, "Set LD2420 min_gate: %d", v);
                        }
                    }
                } else if (tlen == (int)strlen(s_topic_cfg_ld_max_cmd) && strncmp(t, s_topic_cfg_ld_max_cmd, tlen) == 0) {
                    if (s_cfg.set_ld_max_gate) {
                        int v; if (safe_atoi(e->data, e->data_len, &v, 0, 15)) {
                            s_cfg.set_ld_max_gate(v);
                            char buf[16]; snprintf(buf, sizeof(buf), "%d", s_cfg.get_ld_max_gate ? s_cfg.get_ld_max_gate() : v);
                            pub(s_topic_cfg_ld_max_stat, buf, 1, 1);
                            ESP_LOGI(TAG, "Set LD2420 max_gate: %d", v);
                        }
                    }
                } else if (tlen == (int)strlen(s_topic_cfg_ld_delay_cmd) && strncmp(t, s_topic_cfg_ld_delay_cmd, tlen) == 0) {
                    if (s_cfg.set_ld_delay_ms) {
                        int v; if (safe_atoi(e->data, e->data_len, &v, 0, 65535)) {
                            s_cfg.set_ld_delay_ms(v);
                            char buf[16]; snprintf(buf, sizeof(buf), "%d", s_cfg.get_ld_delay_ms ? s_cfg.get_ld_delay_ms() : v);
                            pub(s_topic_cfg_ld_delay_stat, buf, 1, 1);
                            ESP_LOGI(TAG, "Set LD2420 delay_time: %d ms", v);
                        }
                    }
                } else if (tlen == (int)strlen(s_topic_cfg_ld_trig0_cmd) && strncmp(t, s_topic_cfg_ld_trig0_cmd, tlen) == 0) {
                    if (s_cfg.set_ld_trigger_sens) {
                        int v; if (safe_atoi(e->data, e->data_len, &v, 0, 65535)) {
                            s_cfg.set_ld_trigger_sens(v);
                            char buf[16]; snprintf(buf, sizeof(buf), "%d", s_cfg.get_ld_trigger_sens ? s_cfg.get_ld_trigger_sens() : v);
                            pub(s_topic_cfg_ld_trig0_stat, buf, 1, 1);
                            ESP_LOGI(TAG, "Set LD2420 trigger_sens: %d", v);
                        }
                    }
                } else if (tlen == (int)strlen(s_topic_cfg_ld_hold0_cmd) && strncmp(t, s_topic_cfg_ld_hold0_cmd, tlen) == 0) {
                    if (s_cfg.set_ld_maintain_sens) {
                        int v; if (safe_atoi(e->data, e->data_len, &v, 0, 65535)) {
                            s_cfg.set_ld_maintain_sens(v);
                            char buf[16]; snprintf(buf, sizeof(buf), "%d", s_cfg.get_ld_maintain_sens ? s_cfg.get_ld_maintain_sens() : v);
                            pub(s_topic_cfg_ld_hold0_stat, buf, 1, 1);
                            ESP_LOGI(TAG, "Set LD2420 maintain_sens: %d", v);
                        }
                    }
                }
                
                /* Handle zone configuration commands */
                for (int i = 0; i < 3; ++i) {
                    if (tlen == (int)strlen(s_topic_cfg_zone_min_cmd[i]) && 
                        strncmp(t, s_topic_cfg_zone_min_cmd[i], tlen) == 0) {
                        int v;
                        if (safe_atoi(e->data, e->data_len, &v, 0, 600)) {
                            s_zone_min_cm[i] = v;
                            char buf[16]; 
                            snprintf(buf, sizeof(buf), "%d", v); 
                            pub(s_topic_cfg_zone_min_stat[i], buf, 1, 1);
                            ESP_LOGI(TAG, "Set zone %d min: %d cm", i, v);
                        }
                        break;
                    } else if (tlen == (int)strlen(s_topic_cfg_zone_max_cmd[i]) && 
                              strncmp(t, s_topic_cfg_zone_max_cmd[i], tlen) == 0) {
                        int v;
                        if (safe_atoi(e->data, e->data_len, &v, 0, 600)) {
                            s_zone_max_cm[i] = v;
                            char buf[16]; 
                            snprintf(buf, sizeof(buf), "%d", v); 
                            pub(s_topic_cfg_zone_max_stat[i], buf, 1, 1);
                            ESP_LOGI(TAG, "Set zone %d max: %d cm", i, v);
                        }
                        break;
                    }
                }
                
                /* Handle smoothing window command */
                if (tlen == (int)strlen(s_topic_cfg_smooth_cmd) && 
                    strncmp(t, s_topic_cfg_smooth_cmd, tlen) == 0) {
                    int v;
                    if (safe_atoi(e->data, e->data_len, &v, 1, 10)) {
                        s_smooth_win = v;
                        char buf[16]; 
                        snprintf(buf, sizeof(buf), "%d", v); 
                        pub(s_topic_cfg_smooth_stat, buf, 1, 1);
                        ESP_LOGI(TAG, "Set smoothing window: %d", v);
                    }
                }

                /* Action buttons */
                if (tlen == (int)strlen(s_topic_cmd_restart) && strncmp(t, s_topic_cmd_restart, tlen) == 0) {
                    if (payload_is_press(e->data, e->data_len)) {
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
                    if (payload_is_press(e->data, e->data_len)) {
                        ESP_LOGI(TAG, "MQTT resend discovery requested");
                        ha_mqtt_resend_discovery();
                    }
                } else if (tlen == (int)strlen(s_topic_cmd_apply_cfg) && strncmp(t, s_topic_cmd_apply_cfg, tlen) == 0) {
                    if (payload_is_press(e->data, e->data_len)) {
                        int64_t now = esp_timer_get_time();
                        if (now - s_last_apply_us < APPLY_MIN_INTERVAL_US) {
                            ESP_LOGW(TAG, "Apply config ignored: rate limited");
                        } else {
                            s_last_apply_us = now;
                            ESP_LOGI(TAG, "MQTT apply config requested");
                            if (s_cfg.action_apply_config) s_cfg.action_apply_config();
                        }
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
    s_boot_us = esp_timer_get_time();
    s_last_diag_us = 0;
    
    /* Initialize smoothing buffer */
    memset(s_smooth_ring, 0, sizeof(s_smooth_ring));
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
            .keepalive = 15,
            .last_will = {
                .topic   = s_topic_status,
                .msg     = "offline",
                .qos     = 1,
                .retain  = 1
            }
        },
        .network.timeout_ms = 5000,
    };

    s_client = esp_mqtt_client_init(&mc);
    if (!s_client) {
        ESP_LOGE(TAG, "Failed to initialize MQTT client");
        return;
    }
    
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_client);
}

void ha_mqtt_stop(void) {
    if (!s_client) return;
    esp_mqtt_client_stop(s_client);
    esp_mqtt_client_destroy(s_client);
    s_client = NULL;
    s_connected = false;
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
            if (s_smooth_win < 1) s_smooth_win = 1;
            if (s_smooth_win > 10) s_smooth_win = 10;

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
                for (int i = 0; i < 3; ++i) {
                    int on = (cur_cm >= s_zone_min_cm[i] && cur_cm <= s_zone_max_cm[i]) ? 1 : 0;
                    if (on != s_zone_last_on[i]) {
                        s_zone_last_on[i] = on;
                        zone_updates[i] = on;
                    }
                }
            } else {
                for (int i = 0; i < 3; ++i) {
                    if (s_zone_last_on[i]) {
                        s_zone_last_on[i] = 0;
                        zone_updates[i] = 0;
                    }
                }
            }
        } else if (!present) {
            for (int i = 0; i < 3; ++i) {
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

    if (!s_connected) return;

    // Retain presence so HA keeps state across restarts
    pub(s_topic_presence, present ? "ON" : "OFF", 1, 1);
    pub(s_topic_status, "online", 1, 1);

    if (publish_distance && avg_mm_snapshot >= 0) {
        char buf[16];
        float cm = avg_mm_snapshot / 10.0f;
        snprintf(buf, sizeof(buf), "%.1f", cm);
        pub(s_topic_movement_distance, buf, 1, 1);
    }

    for (int i = 0; i < 3; ++i) {
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
    if (!s_connected) return;
    publish_discovery_all();
    publish_attrs_once();
}

void ha_mqtt_publish_fw_version(const char *version) {
    const char *incoming = (version && version[0]) ? version : "unknown";
    bool lock_taken = false;
    if (s_publish_lock) {
        lock_taken = xSemaphoreTake(s_publish_lock, portMAX_DELAY) == pdTRUE;
    }

    if (!s_publish_lock) {
        size_t len = strlen(incoming);
        if (len >= sizeof(s_last_fw_version)) len = sizeof(s_last_fw_version) - 1;
        memcpy(s_last_fw_version, incoming, len);
        s_last_fw_version[len] = '\0';
        s_have_fw_version = true;
    } else {
        if (lock_taken) {
            size_t len = strlen(incoming);
            if (len >= sizeof(s_last_fw_version)) len = sizeof(s_last_fw_version) - 1;
            memcpy(s_last_fw_version, incoming, len);
            s_last_fw_version[len] = '\0';
            s_have_fw_version = true;
            xSemaphoreGive(s_publish_lock);
        } else {
            ESP_LOGW(TAG, "publish_fw_version: publish mutex unavailable");
            return;
        }
    }

    if (!s_connected) return;

    const char *to_send = (s_have_fw_version && s_last_fw_version[0]) ? s_last_fw_version : incoming;
    pub(s_topic_fwver, to_send, 1, 1);
}

/* Diagnostic hooks (no‑op by default) */
void ha_mqtt_diag_publish_out(int raw, int active, int present) {
    (void)raw; (void)active; (void)present;
}

void ha_mqtt_diag_publish_uart(int alive, int baud) {
    (void)alive; (void)baud;
}

/* Direction events (LD2411) – unused in this build */
void ha_mqtt_publish_dir_approach(int on) {
    (void)on;
}

void ha_mqtt_publish_dir_away(int on) {
    (void)on;
}
