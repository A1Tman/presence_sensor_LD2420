#include "ld2420.h"
#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"

static const char *TAG = "LD2420_LIB";
#define BUF_SIZE (256)
#define PACKET_DATA_SIZE 35  // Energy mode data size (not including header/footer)

// Command packet format (from ESPHome)
const uint8_t CMD_HEADER[] = {0xFD, 0xFC, 0xFB, 0xFA};
const uint8_t CMD_FOOTER[] = {0x04, 0x03, 0x02, 0x01};

// Energy mode data packet format (from ESPHome)
const uint8_t DATA_HEADER[] = {0xF4, 0xF3, 0xF2, 0xF1};
const uint8_t DATA_FOOTER[] = {0xF8, 0xF7, 0xF6, 0xF5};

// Commands from ESPHome
#define CMD_ENABLE_CONFIG   0x00FF
#define CMD_DISABLE_CONFIG  0x00FE
#define CMD_WRITE_SYS_PARAM 0x0012
#define CMD_RESTART         0x0068
#define CMD_PROTOCOL_VER    0x0002

// System modes from ESPHome
#define MODE_ENERGY  0x0004  // Structured packets
#define MODE_DEBUG   0x0000  // Raw waveform data
#define MODE_SIMPLE  0x0064  // Text output

// Helper function to log hex data
static void log_buffer_hex(const char* tag, const uint8_t* buffer, size_t len) {
    char hex_str[len * 3 + 1];
    for (size_t i = 0; i < len; i++) {
        sprintf(hex_str + i * 3, "%02X ", buffer[i]);
    }
    ESP_LOGD(tag, "%s", hex_str);
}

// Send raw command frame
static esp_err_t send_command(ld2420_t* sensor, const uint8_t* cmd_data, size_t cmd_len) {
    if (!sensor) return ESP_ERR_INVALID_ARG;
    uart_write_bytes(sensor->uart_port, (const char*)cmd_data, cmd_len);
    uart_wait_tx_done(sensor->uart_port, pdMS_TO_TICKS(100));
    return ESP_OK;
}

// Build and send a command frame: header + len + [cmd (2) + payload] + footer
static esp_err_t send_frame(ld2420_t* sensor, uint16_t cmd, const uint8_t* payload, size_t payload_len) {
    if (!sensor) return ESP_ERR_INVALID_ARG;
    uint8_t frame[4 + 2 + 2 + 64 + 4];
    size_t idx = 0;
    // Header
    frame[idx++] = 0xFD; frame[idx++] = 0xFC; frame[idx++] = 0xFB; frame[idx++] = 0xFA;
    // Length (little endian): cmd(2) + payload_len
    uint16_t data_len = (uint16_t)(2 + payload_len);
    frame[idx++] = (uint8_t)(data_len & 0xFF);
    frame[idx++] = (uint8_t)((data_len >> 8) & 0xFF);
    // Command (little endian)
    frame[idx++] = (uint8_t)(cmd & 0xFF);
    frame[idx++] = (uint8_t)((cmd >> 8) & 0xFF);
    // Payload
    if (payload && payload_len) {
        if (payload_len > 64) return ESP_ERR_INVALID_SIZE;
        memcpy(&frame[idx], payload, payload_len);
        idx += payload_len;
    }
    // Footer
    frame[idx++] = 0x04; frame[idx++] = 0x03; frame[idx++] = 0x02; frame[idx++] = 0x01;

    // Send
    ESP_LOGD(TAG, "send_frame cmd=0x%04X len=%u", cmd, (unsigned)payload_len);
    return send_command(sensor, frame, idx);
}

// Read an ACK frame (best-effort). Returns ESP_OK if something that looks like an ACK arrived.
static esp_err_t read_ack(ld2420_t* sensor, int timeout_ms) {
    if (!sensor) return ESP_ERR_INVALID_ARG;
    uint8_t rx[128];
    int len = uart_read_bytes(sensor->uart_port, rx, sizeof(rx), pdMS_TO_TICKS(timeout_ms));
    if (len <= 0) return ESP_ERR_TIMEOUT;
    // Best-effort: look for header and footer
    for (int i = 0; i + 8 < len; ++i) {
        if (rx[i] == 0xFD && rx[i+1] == 0xFC && rx[i+2] == 0xFB && rx[i+3] == 0xFA) {
            // plausible ACK
            ESP_LOGD(TAG, "ACK %d bytes", len - i);
            return ESP_OK;
        }
    }
    return ESP_OK; // len>0 but didn’t find header; ignore
}

esp_err_t ld2420_enter_command_mode(ld2420_t* sensor) {
    if (!sensor) return ESP_ERR_INVALID_ARG;
    // As per guidance: send twice; clear buffer in between
    uint8_t open_cmd[] = {
        0xFD,0xFC,0xFB,0xFA,
        0x04,0x00,
        0xFF,0x00,
        0x02,0x00,   // protocol version 2
        0x04,0x03,0x02,0x01
    };
    send_command(sensor, open_cmd, sizeof(open_cmd));
    vTaskDelay(pdMS_TO_TICKS(120));
    uart_flush(sensor->uart_port);
    send_command(sensor, open_cmd, sizeof(open_cmd));
    read_ack(sensor, 300);
    return ESP_OK;
}

esp_err_t ld2420_exit_command_mode(ld2420_t* sensor) {
    if (!sensor) return ESP_ERR_INVALID_ARG;
    uint8_t close_cmd[] = {
        0xFD,0xFC,0xFB,0xFA,
        0x02,0x00,
        0xFE,0x00,
        0x04,0x03,0x02,0x01
    };
    send_command(sensor, close_cmd, sizeof(close_cmd));
    read_ack(sensor, 300);
    return ESP_OK;
}

esp_err_t ld2420_restart(ld2420_t* sensor) {
    if (!sensor) return ESP_ERR_INVALID_ARG;
    uint8_t restart_cmd[] = {
        0xFD,0xFC,0xFB,0xFA,
        0x02,0x00,
        0x68,0x00,
        0x04,0x03,0x02,0x01
    };
    send_command(sensor, restart_cmd, sizeof(restart_cmd));
    read_ack(sensor, 300);
    return ESP_OK;
}

esp_err_t ld2420_set_param(ld2420_t* sensor, uint16_t param_id, uint32_t value) {
    if (!sensor) return ESP_ERR_INVALID_ARG;
    // Command 0x0007: [param_id (2)] [value (4)]
    uint8_t payload[6];
    payload[0] = (uint8_t)(param_id & 0xFF);
    payload[1] = (uint8_t)((param_id >> 8) & 0xFF);
    payload[2] = (uint8_t)(value & 0xFF);
    payload[3] = (uint8_t)((value >> 8) & 0xFF);
    payload[4] = (uint8_t)((value >> 16) & 0xFF);
    payload[5] = (uint8_t)((value >> 24) & 0xFF);
    esp_err_t err = send_frame(sensor, 0x0007, payload, sizeof(payload));
    read_ack(sensor, 400);
    return err;
}

esp_err_t ld2420_set_gate_range(ld2420_t* sensor, int min_gate, int max_gate) {
    if (min_gate < 0) min_gate = 0;
    if (min_gate > 15) min_gate = 15;
    if (max_gate < 0) max_gate = 0;
    if (max_gate > 15) max_gate = 15;
    // Write individually
    esp_err_t e1 = ld2420_set_param(sensor, 0x0000, (uint32_t)min_gate);
    esp_err_t e2 = ld2420_set_param(sensor, 0x0001, (uint32_t)max_gate);
    return (e1 == ESP_OK && e2 == ESP_OK) ? ESP_OK : ESP_FAIL;
}

esp_err_t ld2420_set_delay_ms(ld2420_t* sensor, int delay_ms) {
    if (delay_ms < 0) delay_ms = 0;
    if (delay_ms > 65535) delay_ms = 65535;
    return ld2420_set_param(sensor, 0x0004, (uint32_t)delay_ms);
}

esp_err_t ld2420_set_trigger_sens(ld2420_t* sensor, int index, uint32_t value) {
    if (index < 0) index = 0;
    if (index > 15) index = 15;
    return ld2420_set_param(sensor, (uint16_t)(0x0010 + index), value);
}

esp_err_t ld2420_set_maintain_sens(ld2420_t* sensor, int index, uint32_t value) {
    if (index < 0) index = 0;
    if (index > 15) index = 15;
    return ld2420_set_param(sensor, (uint16_t)(0x0020 + index), value);
}

// Read firmware version using command 0x0000
esp_err_t ld2420_read_firmware_version(ld2420_t* sensor, char *out, size_t out_size) {
    if (!sensor || !out || out_size == 0) return ESP_ERR_INVALID_ARG;
    out[0] = '\0';

    // Build simple frame: cmd=0x0000, no payload
    esp_err_t err = send_frame(sensor, 0x0000, NULL, 0);
    if (err != ESP_OK) return err;

    // Read ACK
    uint8_t rx[128];
    int len = uart_read_bytes(sensor->uart_port, rx, sizeof(rx), pdMS_TO_TICKS(300));
    if (len <= 0) return ESP_ERR_TIMEOUT;

    // Find header
    int i = 0;
    for (; i + 8 < len; ++i) {
        if (rx[i]==0xFD && rx[i+1]==0xFC && rx[i+2]==0xFB && rx[i+3]==0xFA) break;
    }
    if (i + 8 >= len) return ESP_ERR_NOT_FOUND;

    // Content length
    if (i + 6 >= len) return ESP_ERR_INVALID_SIZE;
    uint16_t content_len = (uint16_t)(rx[i+4] | (rx[i+5] << 8));
    int content_start = i + 6;
    if (content_start + content_len > len) content_len = len - content_start;

    // Expect: [cmd(2)] [status(2)] [strlen(2)] [str...]
    if (content_len < 6) return ESP_ERR_INVALID_RESPONSE;
    int p = content_start + 4; // skip cmd+status
    uint16_t slen = (uint16_t)(rx[p] | (rx[p+1] << 8));
    p += 2;
    if (p + slen > content_start + content_len) slen = (content_start + content_len) - p;
    if (slen >= out_size) slen = (uint16_t)(out_size - 1);
    memcpy(out, &rx[p], slen);
    out[slen] = '\0';
    return ESP_OK;
}

// Set sensor to Energy Mode (structured packet output)
static esp_err_t set_energy_mode(ld2420_t* sensor) {
    ESP_LOGI(TAG, "Configuring sensor for Energy Mode (structured packets)...");
    
    // Step 1: Enable configuration mode with protocol version 2
    uint8_t enable_config[] = {
        0xFD, 0xFC, 0xFB, 0xFA,  // Header
        0x04, 0x00,              // Length
        0xFF, 0x00,              // Command: Enable config
        0x02, 0x00,              // Protocol version 2
        0x04, 0x03, 0x02, 0x01   // Footer
    };
    
    ESP_LOGD(TAG, "Step 1: Enabling config mode...");
    send_command(sensor, enable_config, sizeof(enable_config));
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Read any response
    uint8_t rx_buf[64];
    int len = uart_read_bytes(sensor->uart_port, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(500));
    if (len > 0) {
        ESP_LOGD(TAG, "Config mode response (%d bytes):", len);
        log_buffer_hex(TAG, rx_buf, len > 32 ? 32 : len);
    }
    
    // Step 2: Set system mode to Energy (0x04)
    uint8_t set_mode[] = {
        0xFD, 0xFC, 0xFB, 0xFA,  // Header
        0x08, 0x00,              // Length (8 bytes of data)
        0x12, 0x00,              // Command: Write system param
        0x00, 0x00,              // Parameter: System mode
        0x04, 0x00,              // Value: Energy mode (0x04)
        0x00, 0x00,              // Reserved/unknown
        0x04, 0x03, 0x02, 0x01   // Footer
    };
    
    ESP_LOGD(TAG, "Step 2: Setting Energy Mode (0x04)...");
    send_command(sensor, set_mode, sizeof(set_mode));
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // Read response
    len = uart_read_bytes(sensor->uart_port, rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(500));
    if (len > 0) {
        ESP_LOGD(TAG, "Mode set response (%d bytes):", len);
        log_buffer_hex(TAG, rx_buf, len > 32 ? 32 : len);
    }
    
    // Step 3: Disable configuration mode (saves settings)
    uint8_t disable_config[] = {
        0xFD, 0xFC, 0xFB, 0xFA,  // Header
        0x02, 0x00,              // Length
        0xFE, 0x00,              // Command: Disable config
        0x04, 0x03, 0x02, 0x01   // Footer
    };
    
    ESP_LOGD(TAG, "Step 3: Disabling config mode (saves settings)...");
    send_command(sensor, disable_config, sizeof(disable_config));
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // Clear UART buffer
    uart_flush(sensor->uart_port);
    
    ESP_LOGI(TAG, "Energy Mode configuration complete");
    ESP_LOGD(TAG, "Sensor should now output packets with header F4 F3 F2 F1");
    
    return ESP_OK;
}

// Create sensor instance
ld2420_t* ld2420_create(void) {
    ld2420_t* sensor = (ld2420_t*)calloc(1, sizeof(ld2420_t));
    if (sensor == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for sensor");
        return NULL;
    }
    sensor->parse_state = 0;
    sensor->header_index = 0;
    sensor->data_index = 0;
    sensor->tail_index = 0;
    return sensor;
}

// Initialize sensor
esp_err_t ld2420_begin(ld2420_t* sensor, uart_port_t uart_port, gpio_num_t tx_pin, gpio_num_t rx_pin, int baud_rate) {
    if (sensor == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    sensor->uart_port = uart_port;
    
    // Configure UART
    const uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    // Install UART driver
    ESP_ERROR_CHECK(uart_driver_install(uart_port, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_port, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    // Clear any existing data
    uart_flush(uart_port);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    // Check what mode the sensor is in
    ESP_LOGI(TAG, "Checking sensor output mode...");
    uint8_t test_buf[BUF_SIZE];
    int test_len = uart_read_bytes(uart_port, test_buf, sizeof(test_buf), pdMS_TO_TICKS(500));
    
    if (test_len > 0) {
        ESP_LOGD(TAG, "Sensor is outputting data (%d bytes)", test_len);
        
        // Check for waveform mode (mostly zeros)
        int zero_count = 0;
        bool found_f4_header = false;
        
        for (int i = 0; i < test_len; i++) {
            if (test_buf[i] == 0x00) zero_count++;
            
            // Check for Energy mode header
            if (i <= test_len - 4) {
                if (test_buf[i] == 0xF4 && test_buf[i+1] == 0xF3 && 
                    test_buf[i+2] == 0xF2 && test_buf[i+3] == 0xF1) {
                    found_f4_header = true;
                    ESP_LOGD(TAG, "Found Energy Mode header at offset %d - sensor already configured", i);
                }
            }
        }
        
        if (!found_f4_header) {
            if (zero_count > test_len * 0.8) {
                ESP_LOGW(TAG, "Sensor in Debug/Waveform mode (>80%% zeros)");
                ESP_LOGI(TAG, "Switching to Energy Mode...");
                set_energy_mode(sensor);
            } else {
                ESP_LOGI(TAG, "Unknown data format, attempting Energy Mode configuration...");
                set_energy_mode(sensor);
            }
        } else {
            ESP_LOGI(TAG, "Sensor already in Energy Mode - ready to parse packets");
        }
    } else {
        ESP_LOGW(TAG, "No initial data from sensor - configuring Energy Mode anyway");
        set_energy_mode(sensor);
    }
    
    return ESP_OK;
}

// Initialize with OT2 pin support
esp_err_t ld2420_begin_with_ot2(ld2420_t* sensor, uart_port_t uart_port, gpio_num_t tx_pin, 
                                  gpio_num_t rx_pin, gpio_num_t ot2_pin, int baud_rate) {
    // First initialize UART
    esp_err_t ret = ld2420_begin(sensor, uart_port, tx_pin, rx_pin, baud_rate);
    
    if (ret == ESP_OK && ot2_pin != GPIO_NUM_NC) {
        // Configure OT2 pin as input with pull-down
        gpio_config_t io_conf = {
            .intr_type = GPIO_INTR_DISABLE,
            .mode = GPIO_MODE_INPUT,
            .pin_bit_mask = (1ULL << ot2_pin),
            .pull_down_en = GPIO_PULLDOWN_ENABLE,
            .pull_up_en = GPIO_PULLUP_DISABLE
        };
        gpio_config(&io_conf);
        
        ESP_LOGI(TAG, "OT2 pin configured on GPIO%d for simple detection", ot2_pin);
    }
    
    return ret;
}

// Check OT2 pin status
bool ld2420_check_ot2(gpio_num_t ot2_pin) {
    if (ot2_pin != GPIO_NUM_NC) {
        return gpio_get_level(ot2_pin) == 1;
    }
    return false;
}

// Parse Energy mode data packet
static void parse_energy_packet(ld2420_t* sensor) {
    // Energy mode packet structure (from ESPHome):
    // Byte 0: Presence (0=none, 1=detected)
    // Bytes 1-2: Distance (little-endian, cm)
    // Bytes 3-34: Gate energy values (16 gates * 2 bytes each)
    
    uint8_t presence = sensor->data_buffer[0];
    uint16_t distance = sensor->data_buffer[1] | (sensor->data_buffer[2] << 8);
    
    ESP_LOGD(TAG, "Energy packet: Presence=%d, Distance=%d cm", presence, distance);
    
    // Update sensor state
    LD2420_DetectionState new_state = presence ? LD2420_DETECTION_ACTIVE : LD2420_NO_DETECTION;
    
    if (new_state != sensor->current_data.state && sensor->on_state_change) {
        sensor->on_state_change(sensor->current_data.state, new_state);
    }
    
    sensor->current_data.state = new_state;
    sensor->current_data.distance = distance;
    sensor->current_data.timestamp = esp_timer_get_time();
    sensor->current_data.isValid = true;
    
    // Trigger callbacks
    if (sensor->on_detection && new_state == LD2420_DETECTION_ACTIVE) {
        sensor->on_detection(distance);
    }
    if (sensor->on_data_update) {
        sensor->on_data_update(sensor->current_data);
    }
}

// Parse incoming byte using state machine
static void parse_byte(ld2420_t* sensor, uint8_t byte) {
    switch (sensor->parse_state) {
        case 0:  // Looking for header F4 F3 F2 F1
            sensor->header_buffer[sensor->header_index] = byte;
            sensor->header_index++;
            
            if (sensor->header_index >= 4) {
                if (memcmp(sensor->header_buffer, DATA_HEADER, 4) == 0) {
                    // Found valid Energy mode header
                    sensor->parse_state = 1;
                    sensor->data_index = 0;
                    sensor->header_index = 0;
                } else {
                    // Not a valid header, shift buffer
                    sensor->header_buffer[0] = sensor->header_buffer[1];
                    sensor->header_buffer[1] = sensor->header_buffer[2];
                    sensor->header_buffer[2] = sensor->header_buffer[3];
                    sensor->header_index = 3;
                }
            }
            break;
            
        case 1:  // Read length (2 bytes)
            if (sensor->data_index == 0) {
                sensor->packet_length = byte;
                sensor->data_index++;
            } else {
                sensor->packet_length |= (byte << 8);
                sensor->parse_state = 2;
                sensor->data_index = 0;
                // Validate length (Energy mode typically sends 35 bytes)
                if (sensor->packet_length > 64 || sensor->packet_length < 3) {
                    ESP_LOGW(TAG, "Invalid packet length %d, resetting", sensor->packet_length);
                    sensor->parse_state = 0;
                    sensor->header_index = 0;
                }
            }
            break;
            
        case 2:  // Receiving data
            sensor->data_buffer[sensor->data_index] = byte;
            sensor->data_index++;
            
            if (sensor->data_index >= sensor->packet_length) {
                sensor->parse_state = 3;
                sensor->tail_index = 0;
            }
            break;
            
        case 3:  // Checking footer F8 F7 F6 F5
            if (sensor->tail_index == 0 && byte == 0xF8) {
                sensor->tail_index = 1;
            } else if (sensor->tail_index == 1 && byte == 0xF7) {
                sensor->tail_index = 2;
            } else if (sensor->tail_index == 2 && byte == 0xF6) {
                sensor->tail_index = 3;
            } else if (sensor->tail_index == 3 && byte == 0xF5) {
                // Complete Energy packet received!
                ESP_LOGD(TAG, "Complete Energy packet received");
                parse_energy_packet(sensor);
                sensor->parse_state = 0;
                sensor->header_index = 0;
            } else {
                // Invalid footer
                ESP_LOGW(TAG, "Invalid footer at position %d", sensor->tail_index);
                sensor->parse_state = 0;
                sensor->header_index = 0;
            }
            break;
    }
}

void ld2420_update(ld2420_t* sensor) {
    if (sensor == NULL) return;
    
    uint8_t temp_buf[BUF_SIZE];
    int len = uart_read_bytes(sensor->uart_port, temp_buf, sizeof(temp_buf), 0);
    
    if (len <= 0) return;
    
    for (int i = 0; i < len; i++) {
        parse_byte(sensor, temp_buf[i]);
    }
    
    static int update_counter = 0;
    if (++update_counter % 500 == 0) {  // Every 5 seconds
        if (!sensor->current_data.isValid) {
            ESP_LOGW(TAG, "No valid Energy packets yet. Check if sensor needs reconfiguration.");
        }
    }
}

bool ld2420_is_detecting(ld2420_t* sensor) {
    if (sensor == NULL) return false;
    return sensor->current_data.state == LD2420_DETECTION_ACTIVE;
}

ld2420_data_t ld2420_get_current_data(ld2420_t* sensor) {
    if (sensor != NULL) {
        return sensor->current_data;
    }
    return (ld2420_data_t){0};
}

// Register callbacks
void ld2420_on_detection(ld2420_t* sensor, ld2420_detection_cb callback) {
    if (sensor != NULL) {
        sensor->on_detection = callback;
    }
}

void ld2420_on_state_change(ld2420_t* sensor, ld2420_state_change_cb callback) {
    if (sensor != NULL) {
        sensor->on_state_change = callback;
    }
}

void ld2420_on_data_update(ld2420_t* sensor, ld2420_data_cb callback) {
    if (sensor != NULL) {
        sensor->on_data_update = callback;
    }
}
