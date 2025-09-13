#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

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

    // Optional LD2420 tuning getters/setters
    int  (*get_ld_min_gate)(void);
    void (*set_ld_min_gate)(int);
    int  (*get_ld_max_gate)(void);
    void (*set_ld_max_gate)(int);
    int  (*get_ld_delay_ms)(void);
    void (*set_ld_delay_ms)(int);
    int  (*get_ld_trigger_sens)(void);
    void (*set_ld_trigger_sens)(int);
    int  (*get_ld_maintain_sens)(void);
    void (*set_ld_maintain_sens)(int);

    // Legacy escape hatch for raw LD2420 commands (optional)
    // If provided, it may receive ad-hoc text commands.
    void (*ld2420_handle_cmd)(const char *payload, int len);
} ha_mqtt_cfg_t;

/** Initialize (does not connect yet). Safe to call once at boot. */
void ha_mqtt_init(const ha_mqtt_cfg_t *cfg);

/** Start MQTT client. Call after Wi-Fi is up (e.g. on IP_EVENT_STA_GOT_IP). */
void ha_mqtt_start(void);

/** Stop MQTT client (optional). */
void ha_mqtt_stop(void);

/** Publish presence + optional distance (mm). distance_mm < 0 if unknown. */
void ha_mqtt_publish_presence(bool present, int distance_mm);

/** Optionally publish RSSI immediately (otherwise it’s sent periodically). */
void ha_mqtt_publish_rssi_now(void);

/** Optionally force re-sending HA discovery configs (retained). */
void ha_mqtt_resend_discovery(void);

// Diagnostics helpers (optional)
void ha_mqtt_diag_publish_out(int raw, int active, int present);
void ha_mqtt_diag_publish_uart(int alive, int baud);

// Direction events (LD2411): approach (walk-in) and away
void ha_mqtt_publish_dir_approach(int on);
void ha_mqtt_publish_dir_away(int on);

// Publish LD2420 firmware version (diagnostic sensor)
void ha_mqtt_publish_fw_version(const char *version);

#ifdef __cplusplus
}
#endif
