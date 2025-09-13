#ifndef LD2420_H
#define LD2420_H

#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"
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
    ld2420_data_t current_data;
    
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

// Public functions
ld2420_t* ld2420_create(void);
esp_err_t ld2420_begin(ld2420_t* sensor, uart_port_t uart_port, gpio_num_t tx_pin, gpio_num_t rx_pin, int baud_rate);
esp_err_t ld2420_begin_with_ot2(ld2420_t* sensor, uart_port_t uart_port, gpio_num_t tx_pin, 
                                  gpio_num_t rx_pin, gpio_num_t ot2_pin, int baud_rate);
void ld2420_update(ld2420_t* sensor);
bool ld2420_is_detecting(ld2420_t* sensor);
ld2420_data_t ld2420_get_current_data(ld2420_t* sensor);
bool ld2420_check_ot2(gpio_num_t ot2_pin);

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

// Write single parameter (low-level): param_id as in protocol tables
esp_err_t ld2420_set_param(ld2420_t* sensor, uint16_t param_id, uint32_t value);

// Convenience setters (high-level)
esp_err_t ld2420_set_gate_range(ld2420_t* sensor, int min_gate, int max_gate);
esp_err_t ld2420_set_delay_ms(ld2420_t* sensor, int delay_ms);
esp_err_t ld2420_set_trigger_sens(ld2420_t* sensor, int index, uint32_t value);   // index 0..15 maps to 0x0010+index
esp_err_t ld2420_set_maintain_sens(ld2420_t* sensor, int index, uint32_t value);  // index 0..15 maps to 0x0020+index

#endif // LD2420_H
