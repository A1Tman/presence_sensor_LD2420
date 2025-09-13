#include <stdio.h>
#include <string.h>
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
#include "../config/secrets.h"

#define DEVICE_VERSION "1.1.0"

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

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// ==================== PRESENCE DETECTION ====================
static int s_movement_threshold_cm = 5;     // Distance change to detect movement
static int s_presence_timeout_sec = 30;     // Hold presence after movement
static int s_distance_history[3] = {-1, -1, -1};
static int s_history_idx = 0;
static bool s_current_presence = false;
static int64_t s_last_movement_time = 0;
static int s_last_distance = -1;
static ld2420_t* s_sensor = NULL;

// LD2420 tuning (current values; TODO: apply to sensor when library supports it)
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

static void update_presence_state(int distance_cm) {
    // Filter invalid readings
    if (distance_cm < DIST_MIN_VALID_CM || distance_cm > DIST_MAX_VALID_CM) return;

    int64_t now = esp_timer_get_time();
    bool movement;

    xSemaphoreTake(s_state_mutex, portMAX_DELAY);
    movement = detect_movement(distance_cm);
    if (movement) {
        s_last_movement_time = now;
        ESP_LOGI(TAG, "Movement detected at %d cm", distance_cm);
    }
    // Present if we have recent movement
    int64_t time_since_movement = (now - s_last_movement_time) / 1000000LL;
    bool presence = (movement || time_since_movement < s_presence_timeout_sec);

    bool should_publish = false;
    int publish_distance_mm = 0;
    if (presence != s_current_presence || abs(distance_cm - s_last_distance) > 3) {
        if (presence != s_current_presence) {
            ESP_LOGI(TAG, "Presence: %s", presence ? "ON" : "OFF");
        }
        s_current_presence = presence;
        s_last_distance = distance_cm;
        should_publish = true;
        publish_distance_mm = distance_cm * 10; // Convert to mm
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
    
    // Update our smart presence detection
    update_presence_state(distance);
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

static void apply_ld_config(void) {
    if (!s_sensor) return;
    // Snapshot and validate under lock
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
    if (min_gate > max_gate) max_gate = min_gate; // ensure min <= max
    if (delay_ms < DELAY_MIN_MS) delay_ms = DELAY_MIN_MS;
    if (delay_ms > DELAY_MAX_MS) delay_ms = DELAY_MAX_MS;
    s_ld_min_gate = min_gate; s_ld_max_gate = max_gate; s_ld_delay_ms = delay_ms;
    s_ld_trigger_sens = (trig0 < 0) ? 0 : (trig0 > 65535 ? 65535 : trig0);
    s_ld_maintain_sens = (hold0 < 0) ? 0 : (hold0 > 65535 ? 65535 : hold0);
    xSemaphoreGive(s_state_mutex);

    int trig0_local = (trig0 < 0) ? 0 : (trig0 > 65535 ? 65535 : trig0);
    int hold0_local = (hold0 < 0) ? 0 : (hold0 > 65535 ? 65535 : hold0);
    ESP_LOGI(TAG, "Applying LD2420 config: min_gate=%d max_gate=%d delay_ms=%d trig0=%d maintain0=%d",
             min_gate, max_gate, delay_ms, trig0_local, hold0_local);

    // Enter command mode and apply parameters
    if (ld2420_enter_command_mode(s_sensor) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to enter command mode");
        return;
    }
    // Apply gate range
    ld2420_set_gate_range(s_sensor, min_gate, max_gate);
    // Apply delay (ms)
    ld2420_set_delay_ms(s_sensor, delay_ms);
    // Apply trigger/maintain sensitivity for index 0 (00)
    ld2420_set_trigger_sens(s_sensor, 0, (uint32_t)trig0_local);
    ld2420_set_maintain_sens(s_sensor, 0, (uint32_t)hold0_local);
    // Exit command mode
    ld2420_exit_command_mode(s_sensor);
}

// ==================== MQTT SETUP ====================
static void start_mqtt(void) {
    char uri[96];
#ifdef MQTT_BROKER_CA_CERT_PEM
    const char *ca_pem = MQTT_BROKER_CA_CERT_PEM;
#else
    const char *ca_pem = NULL;
#endif
    const char *scheme = (ca_pem != NULL) ? "mqtts" : "mqtt";
    snprintf(uri, sizeof(uri), "%s://%s:%d", scheme, MQTT_BROKER_HOST, MQTT_BROKER_PORT);

    ha_mqtt_cfg_t cfg = {
        .broker_uri = uri,
        .username = MQTT_USERNAME[0] ? MQTT_USERNAME : NULL,
        .password = MQTT_PASSWORD[0] ? MQTT_PASSWORD : NULL,
        .friendly_name = DEVICE_NAME,
        .suggested_area = DEVICE_LOCATION,
        .app_version = DEVICE_VERSION,
        .distance_supported = true,
        .broker_ca_cert_pem = ca_pem,
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
        .action_apply_config = apply_ld_config,
    };
    
    ha_mqtt_init(&cfg);
    ha_mqtt_start();
}

// ==================== WIFI ====================
static void event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 5) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGW(TAG, "WiFi retry %d/5", s_retry_num);
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        if (event_data == NULL) return;
        ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
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

    // Initialize WiFi and MQTT
    ESP_LOGI(TAG, "Starting WiFi...");
    esp_err_t wifi_rc = wifi_init();
    if (wifi_rc != ESP_OK) {
        ESP_LOGW(TAG, "wifi_init returned %s; continuing, background retries may proceed", esp_err_to_name(wifi_rc));
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

    ESP_LOGI(TAG, "✓ Sensor initialized successfully");
    // Read firmware version and publish (diagnostic)
    {
        char fw[48];
        if (ld2420_read_firmware_version(s_sensor, fw, sizeof(fw)) == ESP_OK) {
            ESP_LOGI(TAG, "LD2420 FW: %s", fw);
            ha_mqtt_publish_fw_version(fw);
        } else {
            ESP_LOGW(TAG, "Unable to read LD2420 firmware version");
        }
    }
    
    // Register callbacks
    ld2420_on_detection(s_sensor, onDetection);
    ld2420_on_state_change(s_sensor, onStateChange);
    ld2420_on_data_update(s_sensor, onDataUpdate);
    
    ESP_LOGI(TAG, "✓ Callbacks registered");
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
            
            if (s_sensor && s_sensor->current_data.isValid) {
                // We're getting valid packets
                ESP_LOGD(TAG, "Status: %s | Distance: %d cm | OT2: %s | MQTT Presence: %s",
                         s_sensor->current_data.state == LD2420_DETECTION_ACTIVE ? "DETECTING" : "IDLE",
                         s_sensor->current_data.distance,
                         ot2_state ? "HIGH" : "LOW",
                         s_current_presence ? "ON" : "OFF");
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
            int64_t elapsed = (now - s_last_movement_time) / 1000000LL;
            if (elapsed >= s_presence_timeout_sec) {
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
