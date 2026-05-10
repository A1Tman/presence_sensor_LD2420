#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_event.h"
#include "esp_system.h"

typedef void *esp_mqtt_client_handle_t;

typedef enum {
    MQTT_EVENT_ERROR = 0,
    MQTT_EVENT_CONNECTED = 1,
    MQTT_EVENT_DISCONNECTED = 2,
    MQTT_EVENT_SUBSCRIBED = 3,
    MQTT_EVENT_UNSUBSCRIBED = 4,
    MQTT_EVENT_PUBLISHED = 5,
    MQTT_EVENT_DATA = 6,
} esp_mqtt_event_id_t;

#define MQTT_PROTOCOL_V_5 5

typedef struct esp_mqtt_error_codes {
    int error_type;
    int connect_return_code;
    int esp_tls_last_esp_err;
    int esp_tls_stack_err;
    int esp_tls_cert_verify_flags;
    int esp_transport_sock_errno;
    int disconnect_return_code;
} esp_mqtt_error_codes_t;

typedef struct esp_mqtt_event {
    esp_mqtt_client_handle_t client;
    esp_mqtt_event_id_t event_id;
    const char *topic;
    int topic_len;
    const char *data;
    int data_len;
    int total_data_len;
    int current_data_offset;
    int retain;
    esp_mqtt_error_codes_t *error_handle;
    int protocol_ver;
    int session_present;
} esp_mqtt_event_t;

typedef esp_mqtt_event_t *esp_mqtt_event_handle_t;

typedef struct {
    struct {
        struct {
            const char *uri;
        } address;
        struct {
            const char *certificate;
        } verification;
    } broker;
    struct {
        const char *client_id;
        const char *username;
        struct {
            const char *password;
        } authentication;
    } credentials;
    struct {
        int protocol_ver;
        int keepalive;
        struct {
            const char *topic;
            const char *msg;
            int qos;
            int retain;
        } last_will;
    } session;
    struct {
        int timeout_ms;
        int reconnect_timeout_ms;
    } network;
    struct {
        int size;
        int out_size;
    } buffer;
} esp_mqtt_client_config_t;

typedef struct {
    int session_expiry_interval;
    int maximum_packet_size;
    int receive_maximum;
    int topic_alias_maximum;
    bool request_resp_info;
    bool request_problem_info;
} esp_mqtt5_connection_property_config_t;

esp_mqtt_client_handle_t esp_mqtt_client_init(const esp_mqtt_client_config_t *config);
int esp_mqtt_client_publish(esp_mqtt_client_handle_t client, const char *topic,
                            const char *data, int len, int qos, int retain);
int esp_mqtt_client_subscribe(esp_mqtt_client_handle_t client, const char *topic, int qos);
esp_err_t esp_mqtt5_client_set_connect_property(esp_mqtt_client_handle_t client,
                                                const esp_mqtt5_connection_property_config_t *property);
esp_err_t esp_mqtt_client_register_event(esp_mqtt_client_handle_t client, int32_t event_id,
                                         void *event_handler, void *event_handler_arg);
esp_err_t esp_mqtt_client_start(esp_mqtt_client_handle_t client);
esp_err_t esp_mqtt_client_stop(esp_mqtt_client_handle_t client);
esp_err_t esp_mqtt_client_destroy(esp_mqtt_client_handle_t client);
esp_err_t esp_mqtt_client_reconnect(esp_mqtt_client_handle_t client);
