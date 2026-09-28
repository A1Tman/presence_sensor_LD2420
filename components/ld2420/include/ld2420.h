#ifndef LD2420_H
#define LD2420_H

#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <stdbool.h>
#include <stdint.h>

// Default configuration values
#define LD2420_DEFAULT_BAUD_RATE 115200
#define LD2420_RX_BUF_SIZE 512
#define LD2420_TX_BUF_SIZE 256

// Detection states
typedef enum {
    LD2420_NO_DETECTION = 0,
    LD2420_DETECTION_ACTIVE = 1,
} LD2420_DetectionState;

// Sensor data structure
typedef struct {
    uint16_t distance;          // Distance in cm
    LD2420_DetectionState state;
    uint64_t timestamp;         // Timestamp in microseconds
    bool isValid;
} ld2420_data_t;

// Forward declarations
typedef struct ld2420_t ld2420_t;

// Callback function types
typedef void (*ld2420_detection_cb)(uint16_t distance);
typedef void (*ld2420_state_change_cb)(LD2420_DetectionState oldState, LD2420_DetectionState newState);
typedef void (*ld2420_data_cb)(ld2420_data_t data);

// Main sensor instance structure
struct ld2420_t {
    uart_port_t uart_port;
    ld2420_data_t current_data;   // Mutated by parse_energy_packet under data_lock.
    ld2420_data_t last_snapshot;  // Mirror of current_data refreshed only on the
                                  // lock-success path of ld2420_get_current_data.
                                  // Returned (unlocked) on the rare data_lock
                                  // timeout so callers never observe a partial
                                  // current_data write from the parser.
    SemaphoreHandle_t uart_lock;  // Recursive mutex - allows nested locking by same task
    SemaphoreHandle_t data_lock;  // Lightweight mutex guarding current_data reads/writes;
                                  // independent of uart_lock so snapshot readers (OLED,
                                  // status loop) do not block on long UART operations
                                  // such as apply-config bursts.

    // Callbacks
    ld2420_detection_cb on_detection;
    ld2420_state_change_cb on_state_change;
    ld2420_data_cb on_data_update;
    
    // Parsing state machine for Energy mode packets
    uint8_t parse_state;        // 0=header, 1=length, 2=data, 3=footer
    uint8_t header_buffer[4];   // Buffer for header bytes
    uint8_t header_index;       // Current position in header buffer
    uint8_t data_buffer[64];    // Buffer for packet data
    uint8_t data_index;         // Current position in data buffer
    uint8_t tail_index;         // Current position in footer check
    uint16_t packet_length;     // Expected packet data length
    
    // Legacy compatibility
    uint8_t rx_buffer[LD2420_RX_BUF_SIZE];
    size_t buffer_index;
    bool config_mode;
};

#define LD2420_GATE_COUNT 16      // gates 0..15, each ~70 cm deep

typedef struct {
    int min_gate;
    int max_gate;
    int delay_s;
    uint32_t trigger_sensitivity;
    uint32_t maintain_sensitivity;
} ld2420_config_snapshot_t;

// Public functions
ld2420_t* ld2420_create(void);
void ld2420_destroy(ld2420_t* sensor);
esp_err_t ld2420_begin(ld2420_t* sensor, uart_port_t uart_port, gpio_num_t tx_pin, gpio_num_t rx_pin, int baud_rate);
esp_err_t ld2420_begin_with_ot2(ld2420_t* sensor, uart_port_t uart_port, gpio_num_t tx_pin, 
                                  gpio_num_t rx_pin, gpio_num_t ot2_pin, int baud_rate);
void ld2420_update(ld2420_t* sensor);
bool ld2420_is_detecting(ld2420_t* sensor);
ld2420_data_t ld2420_get_current_data(ld2420_t* sensor);
bool ld2420_check_ot2(gpio_num_t ot2_pin);

bool ld2420_lock(ld2420_t* sensor, TickType_t timeout_ticks);
void ld2420_unlock(ld2420_t* sensor);

// Callback registration
void ld2420_on_detection(ld2420_t* sensor, ld2420_detection_cb callback);
void ld2420_on_state_change(ld2420_t* sensor, ld2420_state_change_cb callback);
void ld2420_on_data_update(ld2420_t* sensor, ld2420_data_cb callback);

// Read firmware version string into buffer (null-terminated)
esp_err_t ld2420_read_firmware_version(ld2420_t* sensor, char *out, size_t out_size);

// Command-mode helpers and parameter writers
esp_err_t ld2420_enter_command_mode(ld2420_t* sensor);
esp_err_t ld2420_exit_command_mode(ld2420_t* sensor);
esp_err_t ld2420_restart(ld2420_t* sensor);
// Restart a radar that stopped streaming and make sure it outputs Energy
// Mode frames again. Blocks ~3 s. Caller should hold ld2420_lock().
esp_err_t ld2420_recover(ld2420_t* sensor);

// Write single parameter (low-level): param_id as in protocol tables
esp_err_t ld2420_set_param(ld2420_t* sensor, uint16_t param_id, uint32_t value);

// Convenience setters (high-level)
esp_err_t ld2420_set_gate_range(ld2420_t* sensor, int min_gate, int max_gate);
esp_err_t ld2420_set_delay_s(ld2420_t* sensor, int delay_s);
esp_err_t ld2420_set_trigger_sens(ld2420_t* sensor, int index, uint32_t value);   // index 0..15 maps to 0x0010+index
esp_err_t ld2420_set_maintain_sens(ld2420_t* sensor, int index, uint32_t value);  // index 0..15 maps to 0x0020+index
esp_err_t ld2420_read_config(ld2420_t* sensor, ld2420_config_snapshot_t *out_config);
// Read the move (trigger) and still (maintain) energy thresholds of all 16 gates.
esp_err_t ld2420_read_thresholds(ld2420_t* sensor, uint32_t trigger[LD2420_GATE_COUNT],
                                 uint32_t maintain[LD2420_GATE_COUNT]);

#endif // LD2420_H
