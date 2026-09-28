#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define HA_MQTT_SENSITIVITY_LOW    0
#define HA_MQTT_SENSITIVITY_MEDIUM 1
#define HA_MQTT_SENSITIVITY_HIGH   2

typedef struct {
    const char *broker_uri;      // e.g. "mqtt://192.168.1.23"
    const char *username;        // NULL if none
    const char *password;        // NULL if none
    const char *friendly_name;   // e.g. "Living Room Presence"
    const char *suggested_area;  // e.g. "Living Room" (optional)
    const char *app_version;     // e.g. "0.2.0"
    const char *discovery_prefix;// e.g. "homeassistant" (optional)
    bool        distance_supported; // true if distance entity should be advertised
    const char *device_model;    // Model string for HA device (optional)
    // Optional TLS root CA certificate (PEM). If provided, MQTT over TLS (mqtts://)
    // is used and the server certificate is validated. If NULL, plain MQTT is used
    // unless the broker_uri already specifies mqtts://.
    const char *broker_ca_cert_pem; // NULL-terminated PEM string

    // Enables MQTT command entities and subscriptions for config sliders and
    // action buttons. Keep this false unless broker ACLs or explicit local
    // policy protect the command topics.
    bool command_topics_enabled;

    // Optional: legacy node IDs used in previous firmware versions.
    // If provided, the integration will publish empty retained discovery
    // configs to these old IDs to let Home Assistant remove duplicate entities.
    const char *const *legacy_node_ids;   // Array of C strings
    int legacy_node_ids_count;            // Number of entries in legacy_node_ids

    // Optional runtime tuning hooks (for HA number sliders)
    int  (*get_debounce_ms)(void);
    int  (*get_hold_on_ms)(void);
    void (*set_debounce_ms)(int);
    void (*set_hold_on_ms)(int);

    // Optional runtime toggles (HA switches)
    bool (*get_out_active_high)(void);
    void (*set_out_active_high)(bool);
    bool (*get_out_pullup_enabled)(void);
    void (*set_out_pullup_enabled)(bool);
    bool (*get_uart_presence_enabled)(void);
    void (*set_uart_presence_enabled)(bool);

    // Presence source and distance threshold
    int  (*get_presence_source)(void);           // 0=out,1=uart,2=combined,3=distance
    void (*set_presence_source)(int);
    int  (*get_distance_thresh_cm)(void);
    void (*set_distance_thresh_cm)(int);
    bool (*get_distance_presence_enable)(void);
    void (*set_distance_presence_enable)(bool);

    // Optional actions
    void (*action_force_publish)(void);
    void (*action_reautobaud)(void);
    void (*action_reset_tuning)(void);
    void (*action_apply_config)(void);

    // Optional firmware update (HA `update` entity). Requires
    // command_topics_enabled. Called from the MQTT task when HA presses
    // Install and a valid manifest has been received on cmd/ota/manifest.
    // Must start the download asynchronously and return true if it did;
    // report back via ha_mqtt_publish_ota_progress/_result.
    bool (*action_ota_install)(const char *url, const char *sha256_hex,
                               uint32_t size, const char *version);

    // Optional LD2420 tuning getters/setters. Gates are 0..15, 70 cm each;
    // HA shows them as metres ("Detection range", "Ignore closer than").
    // The radar's own hold ("delay time") register is in seconds.
    int  (*get_ld_min_gate)(void);
    void (*set_ld_min_gate)(int);
    int  (*get_ld_max_gate)(void);
    void (*set_ld_max_gate)(int);
    int  (*get_ld_delay_s)(void);
    void (*set_ld_delay_s)(int);

    // Optional sensitivity preset over all 16 gate thresholds.
    // get returns HA_MQTT_SENSITIVITY_* or -1 when the thresholds match no
    // preset ("Custom"); set takes HA_MQTT_SENSITIVITY_*.
    int  (*get_sensitivity)(void);
    void (*set_sensitivity)(int level);

    // Optional persistence for settings kept on the ESP32 (zones, smoothing).
    // load returns false when the key is absent.
    bool (*load_setting)(const char *key, int *out);
    void (*save_setting)(const char *key, int value);

    // Legacy escape hatch for raw LD2420 commands (optional)
    // If provided, it may receive ad-hoc text commands.
    void (*ld2420_handle_cmd)(const char *payload, int len);
} ha_mqtt_cfg_t;

/** Initialize (does not connect yet). Safe to call once at boot. */
void ha_mqtt_init(const ha_mqtt_cfg_t *cfg);

/** Start MQTT client. Call after Wi-Fi is up (e.g. on IP_EVENT_STA_GOT_IP). */
void ha_mqtt_start(void);

/**
 * If the MQTT client exists but is not currently connected, trigger an
 * immediate reconnect attempt. Call this on every IP_EVENT_STA_GOT_IP after
 * the first boot so that a stalled auto-reconnect is kicked immediately when
 * Wi-Fi comes back, rather than waiting for the next backoff timeout.
 */
void ha_mqtt_reconnect_if_disconnected(void);

/** Stop MQTT client (optional). */
void ha_mqtt_stop(void);

/** True if the MQTT client is currently connected. */
bool ha_mqtt_is_connected(void);

/** Publish presence + optional distance (mm). distance_mm < 0 if unknown. */
void ha_mqtt_publish_presence(bool present, int distance_mm);

/** Optionally publish RSSI immediately (otherwise it is sent periodically). */
void ha_mqtt_publish_rssi_now(void);

/** Optionally force re-sending HA discovery configs (retained). */
void ha_mqtt_resend_discovery(void);

/** Publish retained LD2420 config states from the current getter callbacks. */
void ha_mqtt_publish_ld2420_config_states(void);

// Diagnostics helpers (optional)
void ha_mqtt_diag_publish_out(int raw, int active, int present);
void ha_mqtt_diag_publish_uart(int alive, int baud);

// Direction events (LD2411): approach (walk-in) and away
void ha_mqtt_publish_dir_approach(int on);
void ha_mqtt_publish_dir_away(int on);

// Publish the LD2420 module firmware version as a diagnostic sensor.
// The ESP application firmware version is advertised separately as device
// sw_version from ha_mqtt_cfg_t.app_version.
void ha_mqtt_publish_ld2420_fw_version(const char *version);

/** Report firmware download progress (0..100) to the HA update entity. */
void ha_mqtt_publish_ota_progress(int percent);

/**
 * Report the end of a firmware install. On failure, message is shown in the
 * HA update dialog. On success the caller is expected to restart shortly.
 */
void ha_mqtt_publish_ota_result(bool ok, const char *message);

/**
 * Publish when the device last restarted (Unix time, seconds), shown in HA as
 * a "Last restart" timestamp. Call once wall-clock time is known (SNTP); the
 * value is cached and republished on reconnect.
 */
void ha_mqtt_publish_boot_time(int64_t boot_epoch_s);

/** Publish the current sensitivity preset (e.g. after thresholds change). */
void ha_mqtt_publish_sensitivity_state(void);

#ifdef __cplusplus
}
#endif
