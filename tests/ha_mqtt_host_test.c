#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CONFIG_MQTT_PROTOCOL_5 1

#include "esp_chip_info.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "mqtt_client.h"
#include "freertos/semphr.h"

#define MAX_RECORDS 256

typedef struct {
    char topic[192];
    char payload[2048];
    int qos;
    int retain;
} publish_record_t;

typedef struct {
    char topic[192];
    int qos;
} subscribe_record_t;

static publish_record_t g_publishes[MAX_RECORDS];
static subscribe_record_t g_subscribes[MAX_RECORDS];
static int g_publish_count;
static int g_subscribe_count;
static int64_t g_now_us = 100000000LL;
static int g_restart_called;
static int g_apply_called;
static int g_movement_threshold = 5;
static int g_hold_on_ms = 30000;
static int g_ld_min_gate = 0;
static int g_ld_max_gate = 12;
static int g_ld_delay_ms = 30;
static int g_ld_trigger = 60000;
static int g_ld_maintain = 40000;
static esp_mqtt_client_config_t g_last_client_config;
static int g_client_started;
static int g_ota_calls;
static bool g_ota_start_ok = true;

static void reset_records(void) {
    memset(g_publishes, 0, sizeof(g_publishes));
    memset(g_subscribes, 0, sizeof(g_subscribes));
    g_publish_count = 0;
    g_subscribe_count = 0;
    g_restart_called = 0;
    g_apply_called = 0;
    g_client_started = 0;
    g_ota_calls = 0;
    g_ota_start_ok = true;
    memset(&g_last_client_config, 0, sizeof(g_last_client_config));
}

static bool topic_contains(const char *needle) {
    for (int i = 0; i < g_publish_count; ++i) {
        if (strstr(g_publishes[i].topic, needle)) return true;
    }
    return false;
}

static bool payload_contains_for_topic(const char *topic_needle, const char *payload_needle) {
    for (int i = 0; i < g_publish_count; ++i) {
        if (strstr(g_publishes[i].topic, topic_needle) &&
            strstr(g_publishes[i].payload, payload_needle)) {
            return true;
        }
    }
    return false;
}

static const char *last_payload_for_topic(const char *topic_needle) {
    for (int i = g_publish_count - 1; i >= 0; --i) {
        if (strstr(g_publishes[i].topic, topic_needle)) return g_publishes[i].payload;
    }
    return NULL;
}

static bool last_payload_contains(const char *topic_needle, const char *payload_needle) {
    const char *payload = last_payload_for_topic(topic_needle);
    return payload && strstr(payload, payload_needle);
}

static bool subscribed_to(const char *topic_needle) {
    for (int i = 0; i < g_subscribe_count; ++i) {
        if (strstr(g_subscribes[i].topic, topic_needle)) return true;
    }
    return false;
}

static int get_movement_threshold(void) { return g_movement_threshold; }
static void set_movement_threshold(int value) { g_movement_threshold = value; }
static int get_hold_on_ms(void) { return g_hold_on_ms; }
static void set_hold_on_ms(int value) { g_hold_on_ms = value; }
static int get_ld_min_gate(void) { return g_ld_min_gate; }
static void set_ld_min_gate(int value) {
    g_ld_min_gate = value;
    if (g_ld_min_gate > g_ld_max_gate) g_ld_max_gate = g_ld_min_gate;
}
static int get_ld_max_gate(void) { return g_ld_max_gate; }
static void set_ld_max_gate(int value) {
    g_ld_max_gate = value;
    if (g_ld_max_gate < g_ld_min_gate) g_ld_min_gate = g_ld_max_gate;
}
static int get_ld_delay_ms(void) { return g_ld_delay_ms; }
static void set_ld_delay_ms(int value) { g_ld_delay_ms = value; }
static int get_ld_trigger(void) { return g_ld_trigger; }
static void set_ld_trigger(int value) { g_ld_trigger = value; }
static int get_ld_maintain(void) { return g_ld_maintain; }
static void set_ld_maintain(int value) { g_ld_maintain = value; }
static void apply_config(void) { g_apply_called++; }

static char g_ota_url[256];
static char g_ota_sha[65];
static uint32_t g_ota_size;
static char g_ota_version[32];
static bool ota_install(const char *url, const char *sha256_hex, uint32_t size, const char *version) {
    g_ota_calls++;
    snprintf(g_ota_url, sizeof(g_ota_url), "%s", url);
    snprintf(g_ota_sha, sizeof(g_ota_sha), "%s", sha256_hex);
    g_ota_size = size;
    snprintf(g_ota_version, sizeof(g_ota_version), "%s", version);
    return g_ota_start_ok;
}

void esp_efuse_mac_get_default(uint8_t mac[6]) {
    const uint8_t fake[6] = {0x50, 0x78, 0x7d, 0xba, 0xca, 0xd4};
    memcpy(mac, fake, sizeof(fake));
}

void esp_chip_info(esp_chip_info_t *out_info) {
    memset(out_info, 0, sizeof(*out_info));
    out_info->model = CHIP_ESP32C3;
    out_info->revision = 4;
    out_info->cores = 1;
}

esp_netif_t *esp_netif_get_handle_from_ifkey(const char *ifkey) {
    (void)ifkey;
    return (esp_netif_t *)0x1;
}

int esp_netif_get_ip_info(esp_netif_t *netif, esp_netif_ip_info_t *ip) {
    (void)netif;
    ip->ip.addr = (142U << 24) | (1U << 16) | (168U << 8) | 192U;
    return ESP_OK;
}

int esp_wifi_sta_get_ap_info(wifi_ap_record_t *ap) {
    ap->rssi = -65;
    return ESP_OK;
}

const char *esp_err_to_name(esp_err_t err) {
    (void)err;
    return "ESP_OK";
}

void esp_restart(void) {
    g_restart_called++;
}

int64_t esp_timer_get_time(void) {
    return g_now_us;
}

void vTaskDelay(TickType_t ticks) {
    (void)ticks;
}

SemaphoreHandle_t xSemaphoreCreateMutex(void) {
    static int mutex;
    return &mutex;
}

BaseType_t xSemaphoreTake(SemaphoreHandle_t semaphore, TickType_t ticks) {
    (void)semaphore;
    (void)ticks;
    return pdTRUE;
}

BaseType_t xSemaphoreGive(SemaphoreHandle_t semaphore) {
    (void)semaphore;
    return pdTRUE;
}

esp_mqtt_client_handle_t esp_mqtt_client_init(const esp_mqtt_client_config_t *config) {
    g_last_client_config = *config;
    return (esp_mqtt_client_handle_t)0x1;
}

int esp_mqtt_client_publish(esp_mqtt_client_handle_t client, const char *topic,
                            const char *data, int len, int qos, int retain) {
    (void)client;
    assert(g_publish_count < MAX_RECORDS);
    snprintf(g_publishes[g_publish_count].topic, sizeof(g_publishes[g_publish_count].topic),
             "%s", topic ? topic : "");
    if (data) {
        if (len > 0) {
            int copy_len = len < (int)sizeof(g_publishes[g_publish_count].payload) - 1
                               ? len
                               : (int)sizeof(g_publishes[g_publish_count].payload) - 1;
            memcpy(g_publishes[g_publish_count].payload, data, (size_t)copy_len);
            g_publishes[g_publish_count].payload[copy_len] = '\0';
        } else {
            snprintf(g_publishes[g_publish_count].payload,
                     sizeof(g_publishes[g_publish_count].payload), "%s", data);
        }
    }
    g_publishes[g_publish_count].qos = qos;
    g_publishes[g_publish_count].retain = retain;
    g_publish_count++;
    return g_publish_count;
}

int esp_mqtt_client_subscribe(esp_mqtt_client_handle_t client, const char *topic, int qos) {
    (void)client;
    assert(g_subscribe_count < MAX_RECORDS);
    snprintf(g_subscribes[g_subscribe_count].topic, sizeof(g_subscribes[g_subscribe_count].topic),
             "%s", topic ? topic : "");
    g_subscribes[g_subscribe_count].qos = qos;
    g_subscribe_count++;
    return g_subscribe_count;
}

esp_err_t esp_mqtt5_client_set_connect_property(esp_mqtt_client_handle_t client,
                                                const esp_mqtt5_connection_property_config_t *property) {
    (void)client;
    (void)property;
    return ESP_OK;
}

esp_err_t esp_mqtt_client_register_event(esp_mqtt_client_handle_t client, int32_t event_id,
                                         void *event_handler, void *event_handler_arg) {
    (void)client;
    (void)event_id;
    (void)event_handler;
    (void)event_handler_arg;
    return ESP_OK;
}

esp_err_t esp_mqtt_client_start(esp_mqtt_client_handle_t client) {
    (void)client;
    g_client_started++;
    return ESP_OK;
}

esp_err_t esp_mqtt_client_stop(esp_mqtt_client_handle_t client) {
    (void)client;
    return ESP_OK;
}

esp_err_t esp_mqtt_client_destroy(esp_mqtt_client_handle_t client) {
    (void)client;
    return ESP_OK;
}

esp_err_t esp_mqtt_client_reconnect(esp_mqtt_client_handle_t client) {
    (void)client;
    return ESP_OK;
}

#include "../components/ha_mqtt/ha_mqtt.c"

static ha_mqtt_cfg_t base_config(bool commands_enabled) {
    ha_mqtt_cfg_t cfg = {
        .broker_uri = "mqtt://192.168.1.62:1883",
        .username = commands_enabled ? "device-user" : NULL,
        .password = commands_enabled ? "secret" : NULL,
        .friendly_name = "Kitchen Radar",
        .suggested_area = "Kitchen",
        .app_version = "9.8.7",
        .discovery_prefix = "homeassistant",
        .distance_supported = true,
        .command_topics_enabled = commands_enabled,
        .get_distance_thresh_cm = get_movement_threshold,
        .set_distance_thresh_cm = set_movement_threshold,
        .get_hold_on_ms = get_hold_on_ms,
        .set_hold_on_ms = set_hold_on_ms,
        .get_ld_min_gate = get_ld_min_gate,
        .set_ld_min_gate = set_ld_min_gate,
        .get_ld_max_gate = get_ld_max_gate,
        .set_ld_max_gate = set_ld_max_gate,
        .get_ld_delay_ms = get_ld_delay_ms,
        .set_ld_delay_ms = set_ld_delay_ms,
        .get_ld_trigger_sens = get_ld_trigger,
        .set_ld_trigger_sens = set_ld_trigger,
        .get_ld_maintain_sens = get_ld_maintain,
        .set_ld_maintain_sens = set_ld_maintain,
        .action_apply_config = apply_config,
        .action_ota_install = ota_install,
    };
    return cfg;
}

static void reset_component(bool commands_enabled) {
    ha_mqtt_stop();
    reset_records();
    ha_mqtt_cfg_t cfg = base_config(commands_enabled);
    ha_mqtt_init(&cfg);
    ha_mqtt_start();
    assert(g_client_started == 1);
    assert(strcmp(g_last_client_config.broker.address.uri, "mqtt://192.168.1.62:1883") == 0);
}

static void emit_connected(void) {
    esp_mqtt_event_t event = {
        .protocol_ver = MQTT_PROTOCOL_V_5,
        .session_present = 0,
    };
    mqtt_event_handler(NULL, NULL, MQTT_EVENT_CONNECTED, &event);
}

static void emit_data_ex(const char *topic, const char *payload, int retain) {
    esp_mqtt_event_t event = {
        .topic = topic,
        .topic_len = (int)strlen(topic),
        .data = payload,
        .data_len = (int)strlen(payload),
        .total_data_len = (int)strlen(payload),
        .current_data_offset = 0,
        .retain = retain,
    };
    mqtt_event_handler(NULL, NULL, MQTT_EVENT_DATA, &event);
}

static void emit_data(const char *topic, const char *payload) {
    emit_data_ex(topic, payload, 0);
}

static void emit_retained_data(const char *topic, const char *payload) {
    emit_data_ex(topic, payload, 1);
}

static void test_discovery_without_command_topics(void) {
    reset_component(false);
    emit_connected();

    assert(topic_contains("binary_sensor/presence-bacad4/presence/config"));
    assert(topic_contains("sensor/presence-bacad4/ld_fw/config"));
    assert(payload_contains_for_topic("sensor/presence-bacad4/ld_fw/config", "\"name\":\"LD2420 Firmware\""));
    assert(payload_contains_for_topic("presence-bacad4/attributes", "\"sw_version\":\"9.8.7\""));
    assert(!payload_contains_for_topic("number/presence-bacad4/movement_thresh/config", "\"cmd_t\""));
    assert(!subscribed_to("/cmd/"));

    printf("ok discovery_without_command_topics\n");
}

static void test_discovery_with_command_topics(void) {
    reset_component(true);
    emit_connected();

    assert(payload_contains_for_topic("number/presence-bacad4/movement_thresh/config", "\"cmd_t\""));
    assert(payload_contains_for_topic("button/presence-bacad4/apply_config/config", "\"cmd_t\""));
    assert(subscribed_to("/cmd/movement_threshold_cm"));
    assert(subscribed_to("/cmd/apply_config"));
    assert(payload_contains_for_topic("binary_sensor/presence-bacad4/presence/config",
                                      "\"hw\":\"ESP32-C3 rev v0.4\""));
    assert(payload_contains_for_topic("binary_sensor/presence-bacad4/presence/config",
                                      "\"sn\":\"50787DBACAD4\""));

    printf("ok discovery_with_command_topics\n");
}

static void test_reconnect_republishes_cached_state(void) {
    reset_component(true);
    ha_mqtt_publish_presence(true, 1234);
    ha_mqtt_publish_ld2420_fw_version("v1.6.1");
    reset_records();

    emit_connected();

    assert(payload_contains_for_topic("presence/presence-bacad4/presence", "ON"));
    assert(payload_contains_for_topic("presence/presence-bacad4/movement_distance_cm", "123.4"));
    assert(payload_contains_for_topic("presence/presence-bacad4/ld2420/fw_version", "v1.6.1"));

    printf("ok reconnect_republishes_cached_state\n");
}

static void test_command_validation_and_gating(void) {
    reset_component(false);
    emit_connected();
    emit_data("presence/presence-bacad4/cmd/movement_threshold_cm", "12");
    assert(g_movement_threshold == 5);

    reset_component(true);
    emit_connected();
    emit_data("presence/presence-bacad4/cmd/movement_threshold_cm", "bad");
    assert(g_movement_threshold == 5);
    emit_data("presence/presence-bacad4/cmd/movement_threshold_cm", "   ");
    assert(g_movement_threshold == 5);
    emit_retained_data("presence/presence-bacad4/cmd/movement_threshold_cm", "12");
    assert(g_movement_threshold == 5);
    emit_data("presence/presence-bacad4/cmd/movement_threshold_cm", "12");
    assert(g_movement_threshold == 12);

    emit_data("presence/presence-bacad4/cmd/apply_config", "PRESS");
    assert(g_apply_called == 1);

    printf("ok command_validation_and_gating\n");
}

static void test_ld_gate_normalization_republishes_pair(void) {
    reset_component(true);
    emit_connected();
    reset_records();

    emit_data("presence/presence-bacad4/cmd/ld2420/min_gate", "15");

    assert(g_ld_min_gate == 15);
    assert(g_ld_max_gate == 15);
    assert(payload_contains_for_topic("presence/presence-bacad4/cfg/ld2420/min_gate", "15"));
    assert(payload_contains_for_topic("presence/presence-bacad4/cfg/ld2420/max_gate", "15"));

    printf("ok ld_gate_normalization_republishes_pair\n");
}

#define OTA_STATE    "presence/presence-bacad4/ota/state"
#define OTA_MANIFEST "presence/presence-bacad4/cmd/ota/manifest"
#define OTA_INSTALL  "presence/presence-bacad4/cmd/ota/install"
#define OTA_SHA      "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef"
#define OTA_GOOD_MANIFEST \
    "{\"version\":\"9.9.0\",\"url\":\"http://192.168.1.62:8123/local/ota/x/fw.bin\"," \
    "\"sha256\":\"" OTA_SHA "\",\"size\":1245184,\"notes\":\"Adds OTA\"}"

static void test_ota_discovery_and_initial_state(void) {
    reset_component(true);
    emit_connected();

    assert(payload_contains_for_topic("update/presence-bacad4/firmware/config", "\"pl_inst\":\"install\""));
    assert(payload_contains_for_topic("update/presence-bacad4/firmware/config", "\"dev_cla\":\"firmware\""));
    assert(payload_contains_for_topic("update/presence-bacad4/firmware/config", "\"cmd_t\":\"" OTA_INSTALL "\""));
    assert(subscribed_to("/cmd/ota/manifest"));
    assert(subscribed_to("/cmd/ota/install"));
    assert(last_payload_contains(OTA_STATE, "\"installed_version\":\"9.8.7\""));
    assert(last_payload_contains(OTA_STATE, "\"latest_version\":\"9.8.7\""));
    assert(last_payload_contains(OTA_STATE, "\"in_progress\":false"));
    // HA rejects the whole state if release_summary is null.
    assert(last_payload_contains(OTA_STATE, "\"release_summary\":\"\""));
    assert(!last_payload_contains(OTA_STATE, "\"release_summary\":null"));

    reset_component(false);
    emit_connected();
    const char *cleared = last_payload_for_topic("update/presence-bacad4/firmware/config");
    assert(cleared && cleared[0] == '\0');
    assert(!subscribed_to("/cmd/ota/"));
    assert(!topic_contains(OTA_STATE));

    printf("ok ota_discovery_and_initial_state\n");
}

static void test_ota_manifest_and_install_flow(void) {
    reset_component(true);
    emit_connected();

    // The manifest is accepted even though it is retained.
    emit_retained_data(OTA_MANIFEST, OTA_GOOD_MANIFEST);
    assert(last_payload_contains(OTA_STATE, "\"latest_version\":\"9.9.0\""));
    assert(last_payload_contains(OTA_STATE, "\"release_summary\":\"Adds OTA\""));

    // A retained or wrong-payload install command never starts a download.
    emit_retained_data(OTA_INSTALL, "install");
    emit_data(OTA_INSTALL, "PRESS");
    assert(g_ota_calls == 0);

    emit_data(OTA_INSTALL, "install");
    assert(g_ota_calls == 1);
    assert(strcmp(g_ota_url, "http://192.168.1.62:8123/local/ota/x/fw.bin") == 0);
    assert(strcmp(g_ota_sha, OTA_SHA) == 0);
    assert(g_ota_size == 1245184);
    assert(strcmp(g_ota_version, "9.9.0") == 0);
    assert(last_payload_contains(OTA_STATE, "\"in_progress\":true"));
    assert(last_payload_contains(OTA_STATE, "\"update_percentage\":0"));

    // A second press while running is ignored.
    emit_data(OTA_INSTALL, "install");
    assert(g_ota_calls == 1);

    ha_mqtt_publish_ota_progress(40);
    assert(last_payload_contains(OTA_STATE, "\"update_percentage\":40"));

    ha_mqtt_publish_ota_result(false, "sha256 mismatch");
    assert(last_payload_contains(OTA_STATE, "\"in_progress\":false"));
    assert(last_payload_contains(OTA_STATE, "\"update_percentage\":null"));
    assert(last_payload_contains(OTA_STATE, "\"release_summary\":\"Last install failed: sha256 mismatch\""));

    // Retry is allowed after a failure; failing to start is reported.
    g_ota_start_ok = false;
    emit_data(OTA_INSTALL, "install");
    assert(g_ota_calls == 2);
    assert(last_payload_contains(OTA_STATE, "Last install failed: could not start download"));
    assert(last_payload_contains(OTA_STATE, "\"in_progress\":false"));

    // Success keeps the entity busy at 100% until the reboot.
    g_ota_start_ok = true;
    emit_data(OTA_INSTALL, "install");
    assert(g_ota_calls == 3);
    ha_mqtt_publish_ota_result(true, "installed 9.9.0");
    assert(last_payload_contains(OTA_STATE, "\"in_progress\":true"));
    assert(last_payload_contains(OTA_STATE, "\"update_percentage\":100"));
    assert(last_payload_contains("presence/presence-bacad4/status", "offline"));

    // Clearing the retained manifest drops the pending update.
    reset_component(true);
    emit_connected();
    emit_retained_data(OTA_MANIFEST, OTA_GOOD_MANIFEST);
    emit_retained_data(OTA_MANIFEST, "");
    assert(last_payload_contains(OTA_STATE, "\"latest_version\":\"9.8.7\""));
    emit_data(OTA_INSTALL, "install");
    assert(g_ota_calls == 0);

    printf("ok ota_manifest_and_install_flow\n");
}

static void test_ota_rejects_bad_manifests(void) {
    const char *bad[] = {
        // short sha256
        "{\"version\":\"9.9.0\",\"url\":\"http://h/fw.bin\",\"sha256\":\"0123\",\"size\":10}",
        // non-hex sha256
        "{\"version\":\"9.9.0\",\"url\":\"http://h/fw.bin\",\"sha256\":\"zz23456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef\",\"size\":10}",
        // unsupported scheme
        "{\"version\":\"9.9.0\",\"url\":\"ftp://h/fw.bin\",\"sha256\":\"" OTA_SHA "\",\"size\":10}",
        // quote in version
        "{\"version\":\"9\\\"9\",\"url\":\"http://h/fw.bin\",\"sha256\":\"" OTA_SHA "\",\"size\":10}",
        // size missing / zero / too large
        "{\"version\":\"9.9.0\",\"url\":\"http://h/fw.bin\",\"sha256\":\"" OTA_SHA "\"}",
        "{\"version\":\"9.9.0\",\"url\":\"http://h/fw.bin\",\"sha256\":\"" OTA_SHA "\",\"size\":0}",
        "{\"version\":\"9.9.0\",\"url\":\"http://h/fw.bin\",\"sha256\":\"" OTA_SHA "\",\"size\":99999999}",
        // not JSON
        "install",
    };

    reset_component(true);
    emit_connected();
    for (size_t i = 0; i < sizeof(bad) / sizeof(bad[0]); ++i) {
        emit_retained_data(OTA_MANIFEST, bad[i]);
        assert(last_payload_contains(OTA_STATE, "\"latest_version\":\"9.8.7\""));
    }
    emit_data(OTA_INSTALL, "install");
    assert(g_ota_calls == 0);

    // A bad manifest does not erase a good one.
    emit_retained_data(OTA_MANIFEST, OTA_GOOD_MANIFEST);
    emit_retained_data(OTA_MANIFEST, bad[0]);
    assert(last_payload_contains(OTA_STATE, "\"latest_version\":\"9.9.0\""));

    // Same version as installed: nothing to do.
    emit_retained_data(OTA_MANIFEST,
        "{\"version\":\"9.8.7\",\"url\":\"http://h/fw.bin\",\"sha256\":\"" OTA_SHA "\",\"size\":10}");
    emit_data(OTA_INSTALL, "install");
    assert(g_ota_calls == 0);

    printf("ok ota_rejects_bad_manifests\n");
}

int main(void) {
    test_discovery_without_command_topics();
    test_discovery_with_command_topics();
    test_reconnect_republishes_cached_state();
    test_command_validation_and_gating();
    test_ld_gate_normalization_republishes_pair();
    test_ota_discovery_and_initial_state();
    test_ota_manifest_and_install_flow();
    test_ota_rejects_bad_manifests();
    return 0;
}
