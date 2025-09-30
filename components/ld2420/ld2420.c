#include "ld2420.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_timer.h"

static const char *TAG = "LD2420_LIB";
#define BUF_SIZE (256)
#define PACKET_DATA_SIZE 35  // Energy mode data size (not including header/footer)

#define CMD_READ_ABD_PARAM 0x0008
#define CMD_MIN_GATE_REG   0x0000
#define CMD_MAX_GATE_REG   0x0001
#define CMD_TIMEOUT_REG    0x0004
#define CMD_TRIGGER_BASE   0x0010
#define CMD_MAINTAIN_BASE  0x0020
#define RESPONSE_HEADER0   0xFD
#define RESPONSE_HEADER1   0xFC
#define RESPONSE_HEADER2   0xFB
#define RESPONSE_HEADER3   0xFA
#define RESPONSE_FOOTER0   0x04
#define RESPONSE_FOOTER1   0x03
#define RESPONSE_FOOTER2   0x02
#define RESPONSE_FOOTER3   0x01

static bool uart_lock_take(ld2420_t *sensor, TickType_t timeout)
{
    if (!sensor || !sensor->uart_lock) {
        return false;
    }
    return xSemaphoreTakeRecursive(sensor->uart_lock, timeout) == pdTRUE;
}

static void uart_lock_give(ld2420_t *sensor)
{
    if (sensor && sensor->uart_lock) {
        xSemaphoreGiveRecursive(sensor->uart_lock);
    }
}

bool ld2420_lock(ld2420_t* sensor, TickType_t timeout_ticks)
{
    return uart_lock_take(sensor, timeout_ticks);
}

void ld2420_unlock(ld2420_t* sensor)
{
    uart_lock_give(sensor);
}

static uint32_t byteswap32(uint32_t v)
{
    return ((v & 0x000000FFu) << 24) |
           ((v & 0x0000FF00u) << 8)  |
           ((v & 0x00FF0000u) >> 8)  |
           ((v & 0xFF000000u) >> 24);
}

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
    if (!sensor) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t rx[64] = {0};
    size_t have = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms > 0 ? timeout_ms : 0);
    bool wait_forever = timeout_ms <= 0;

    while (true) {
        if (have >= sizeof(rx)) {
            return ESP_ERR_INVALID_SIZE;
        }

        size_t max_read = sizeof(rx) - have;
        int chunk = uart_read_bytes(sensor->uart_port, rx + have, max_read, pdMS_TO_TICKS(20));
        if (chunk > 0) {
            have += (size_t)chunk;
        } else if (!wait_forever) {
            TickType_t now = xTaskGetTickCount();
            if ((int32_t)(deadline - now) <= 0) {
                return ESP_ERR_TIMEOUT;
            }
        }

        size_t header_idx = have;
        for (size_t i = 0; i + 4 <= have; ++i) {
            if (rx[i] == RESPONSE_HEADER0 && rx[i + 1] == RESPONSE_HEADER1 &&
                rx[i + 2] == RESPONSE_HEADER2 && rx[i + 3] == RESPONSE_HEADER3) {
                header_idx = i;
                break;
            }
        }

        if (header_idx == have) {
            if (have > 4) {
                size_t keep = 4;
                memmove(rx, rx + have - keep, keep);
                have = keep;
            }
            continue;
        }

        if (header_idx > 0) {
            memmove(rx, rx + header_idx, have - header_idx);
            have -= header_idx;
        }

        if (have < 6) {
            continue;
        }

        uint16_t payload_len = (uint16_t)(rx[4] | (rx[5] << 8));
        size_t frame_len = 4 + 2 + (size_t)payload_len + 4;
        if (frame_len > sizeof(rx)) {
            return ESP_ERR_INVALID_SIZE;
        }

        if (have < frame_len) {
            continue;
        }

        size_t footer_idx = frame_len - 4;
        if (rx[footer_idx] != RESPONSE_FOOTER0 || rx[footer_idx + 1] != RESPONSE_FOOTER1 ||
            rx[footer_idx + 2] != RESPONSE_FOOTER2 || rx[footer_idx + 3] != RESPONSE_FOOTER3) {
            if (have > 1) {
                memmove(rx, rx + 1, have - 1);
                have -= 1;
            } else {
                have = 0;
            }
            continue;
        }

        uint16_t status = 0;
        if (payload_len >= 4) {
            status = (uint16_t)(rx[8] | (rx[9] << 8));
        }

        if (status != 0) {
            ESP_LOGW(TAG, "LD2420 ACK status 0x%04X", status);
            return ESP_FAIL;
        }

        return ESP_OK;
    }
}


static esp_err_t read_response(ld2420_t* sensor, uint8_t *rx, size_t rx_size, int timeout_ms, size_t *out_len)
{
    if (!sensor || !rx || rx_size < 16) {
        return ESP_ERR_INVALID_ARG;
    }

    size_t total = 0;
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms > 0 ? timeout_ms : 0);

    while (true) {
        if (total >= rx_size) {
            return ESP_ERR_INVALID_SIZE;
        }
        int chunk = uart_read_bytes(sensor->uart_port, rx + total, rx_size - total, pdMS_TO_TICKS(20));
        if (chunk > 0) {
            total += (size_t)chunk;
            if (total >= 6) {
                uint16_t payload_len = (uint16_t)(rx[4] | (rx[5] << 8));
                size_t frame_len = 4 + 2 + (size_t)payload_len + 4;
                if (frame_len > rx_size) {
                    return ESP_ERR_INVALID_SIZE;
                }
                if (total >= frame_len) {
                    if (out_len) {
                        *out_len = frame_len;
                    }
                    return ESP_OK;
                }
            }
        } else {
            if (timeout_ms <= 0) {
                continue;
            }
            TickType_t now = xTaskGetTickCount();
            if ((int32_t)(deadline - now) <= 0) {
                return ESP_ERR_TIMEOUT;
            }
        }
    }
}

esp_err_t ld2420_enter_command_mode(ld2420_t* sensor) {
    if (!sensor) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!uart_lock_take(sensor, portMAX_DELAY)) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = ESP_OK;
    uint8_t open_cmd[] = {
        0xFD,0xFC,0xFB,0xFA,
        0x04,0x00,
        0xFF,0x00,
        0x02,0x00,
        0x04,0x03,0x02,0x01
    };

    err = send_command(sensor, open_cmd, sizeof(open_cmd));
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(120));
        esp_err_t flush_rc = uart_flush(sensor->uart_port);
        if (flush_rc != ESP_OK) {
            err = flush_rc;
        }
    }

    if (err == ESP_OK) {
        err = send_command(sensor, open_cmd, sizeof(open_cmd));
    }

    if (err == ESP_OK) {
        err = read_ack(sensor, 300);
    }

    uart_lock_give(sensor);
    return err;
}


esp_err_t ld2420_exit_command_mode(ld2420_t* sensor) {
    if (!sensor) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!uart_lock_take(sensor, portMAX_DELAY)) {
        return ESP_ERR_TIMEOUT;
    }
    uint8_t close_cmd[] = {
        0xFD,0xFC,0xFB,0xFA,
        0x02,0x00,
        0xFE,0x00,
        0x04,0x03,0x02,0x01
    };
    esp_err_t err = send_command(sensor, close_cmd, sizeof(close_cmd));
    if (err == ESP_OK) {
        err = read_ack(sensor, 300);
    }
    uart_lock_give(sensor);
    return err;
}


esp_err_t ld2420_restart(ld2420_t* sensor) {
    if (!sensor) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!uart_lock_take(sensor, portMAX_DELAY)) {
        return ESP_ERR_TIMEOUT;
    }
    uint8_t restart_cmd[] = {
        0xFD,0xFC,0xFB,0xFA,
        0x02,0x00,
        0x68,0x00,
        0x04,0x03,0x02,0x01
    };
    esp_err_t err = send_command(sensor, restart_cmd, sizeof(restart_cmd));
    if (err == ESP_OK) {
        err = read_ack(sensor, 300);
    }
    uart_lock_give(sensor);
    return err;
}


esp_err_t ld2420_set_param(ld2420_t* sensor, uint16_t param_id, uint32_t value) {
    if (!sensor) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!uart_lock_take(sensor, portMAX_DELAY)) {
        return ESP_ERR_TIMEOUT;
    }
    uint8_t payload[6];
    payload[0] = (uint8_t)(param_id & 0xFF);
    payload[1] = (uint8_t)((param_id >> 8) & 0xFF);
    payload[2] = (uint8_t)(value & 0xFF);
    payload[3] = (uint8_t)((value >> 8) & 0xFF);
    payload[4] = (uint8_t)((value >> 16) & 0xFF);
    payload[5] = (uint8_t)((value >> 24) & 0xFF);
    esp_err_t err = send_frame(sensor, 0x0007, payload, sizeof(payload));
    if (err == ESP_OK) {
        err = read_ack(sensor, 400);
    }
    uart_lock_give(sensor);
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

static esp_err_t ld2420_read_config_locked_internal(ld2420_t* sensor, ld2420_config_snapshot_t *out_config)
{
    if (!sensor || !out_config) {
        return ESP_ERR_INVALID_ARG;
    }

    const uint16_t regs[] = {
        CMD_MIN_GATE_REG,
        CMD_MAX_GATE_REG,
        CMD_TIMEOUT_REG,
        (uint16_t)(CMD_TRIGGER_BASE + 0),
        (uint16_t)(CMD_MAINTAIN_BASE + 0),
    };
    uint8_t payload[sizeof(regs)] = {0};
    for (size_t i = 0; i < sizeof(regs) / sizeof(regs[0]); ++i) {
        payload[i * 2]     = (uint8_t)(regs[i] & 0xFF);
        payload[i * 2 + 1] = (uint8_t)((regs[i] >> 8) & 0xFF);
    }

    esp_err_t err = send_frame(sensor, CMD_READ_ABD_PARAM, payload, sizeof(payload));
    if (err != ESP_OK) {
        return err;
    }

    uint8_t rx[64] = {0};
    size_t frame_len = 0;
    err = read_response(sensor, rx, sizeof(rx), 400, &frame_len);
    if (err != ESP_OK) {
        return err;
    }

    if (frame_len < 18) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    if (rx[0] != RESPONSE_HEADER0 || rx[1] != RESPONSE_HEADER1 ||
        rx[2] != RESPONSE_HEADER2 || rx[3] != RESPONSE_HEADER3) {
        return ESP_ERR_INVALID_RESPONSE;
    }

    uint16_t payload_len = (uint16_t)(rx[4] | (rx[5] << 8));
    if (payload_len < 4) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    size_t footer_index = 4 + 2 + payload_len;
    if (rx[footer_index] != RESPONSE_FOOTER0 || rx[footer_index + 1] != RESPONSE_FOOTER1 ||
        rx[footer_index + 2] != RESPONSE_FOOTER2 || rx[footer_index + 3] != RESPONSE_FOOTER3) {
        return ESP_ERR_INVALID_RESPONSE;
    }
    uint16_t status = (uint16_t)(rx[8] | (rx[9] << 8));
    if (status != 0) {
        ESP_LOGW(TAG, "LD2420 read config status 0x%04X", status);
        return ESP_FAIL;
    }

    size_t data_bytes = payload_len - 4; // subtract command + status
    const size_t expected_bytes = (sizeof(regs) / sizeof(regs[0])) * sizeof(uint32_t);
    if (data_bytes < expected_bytes) {
        return ESP_ERR_INVALID_SIZE;
    }

    size_t offset = 10;
    uint32_t values[sizeof(regs) / sizeof(regs[0])] = {0};
    for (size_t i = 0; i < sizeof(values) / sizeof(values[0]); ++i) {
        uint32_t raw = 0;
        memcpy(&raw, &rx[offset + (i * 4)], sizeof(raw));
        values[i] = byteswap32(raw);
    }

    out_config->min_gate = (int)values[0];
    out_config->max_gate = (int)values[1];
    out_config->delay_ms = (int)values[2];
    out_config->trigger_sensitivity = values[3];
    out_config->maintain_sensitivity = values[4];

    return ESP_OK;
}

esp_err_t ld2420_read_config(ld2420_t* sensor, ld2420_config_snapshot_t *out_config)
{
    if (!sensor || !out_config) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!uart_lock_take(sensor, portMAX_DELAY)) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = ld2420_enter_command_mode(sensor);
    if (err != ESP_OK) {
        uart_lock_give(sensor);
        return err;
    }

    err = ld2420_read_config_locked_internal(sensor, out_config);
    esp_err_t exit_err = ld2420_exit_command_mode(sensor);
    if (err == ESP_OK && exit_err != ESP_OK) {
        err = exit_err;
    }

    uart_lock_give(sensor);
    return err;
}

// Read firmware version using command 0x0000
esp_err_t ld2420_read_firmware_version(ld2420_t* sensor, char *out, size_t out_size) {
    if (!sensor || !out || out_size == 0) return ESP_ERR_INVALID_ARG;
    out[0] = '\0';

    if (!uart_lock_take(sensor, portMAX_DELAY)) {
        return ESP_ERR_TIMEOUT;
    }

    // Build simple frame: cmd=0x0000, no payload
    esp_err_t err = send_frame(sensor, 0x0000, NULL, 0);
    if (err != ESP_OK) {
        uart_lock_give(sensor);
        return err;
    }

    // Read ACK
    uint8_t rx[128];
    int len = uart_read_bytes(sensor->uart_port, rx, sizeof(rx), pdMS_TO_TICKS(300));
    if (len <= 0) {
        uart_lock_give(sensor);
        return ESP_ERR_TIMEOUT;
    }

    // Find header
    int i = 0;
    for (; i + 8 < len; ++i) {
        if (rx[i]==0xFD && rx[i+1]==0xFC && rx[i+2]==0xFB && rx[i+3]==0xFA) break;
    }
    if (i + 8 >= len) {
        uart_lock_give(sensor);
        return ESP_ERR_NOT_FOUND;
    }

    // Content length
    if (i + 6 >= len) {
        uart_lock_give(sensor);
        return ESP_ERR_INVALID_SIZE;
    }
    uint16_t content_len = (uint16_t)(rx[i+4] | (rx[i+5] << 8));
    int content_start = i + 6;
    if (content_start + content_len > len) content_len = len - content_start;

    // Expect: [cmd(2)] [status(2)] [strlen(2)] [str...]
    if (content_len < 6) {
        uart_lock_give(sensor);
        return ESP_ERR_INVALID_RESPONSE;
    }
    int p = content_start + 4; // skip cmd+status
    uint16_t slen = (uint16_t)(rx[p] | (rx[p+1] << 8));
    p += 2;
    if (p + slen > content_start + content_len) slen = (content_start + content_len) - p;
    if (slen >= out_size) slen = (uint16_t)(out_size - 1);
    memcpy(out, &rx[p], slen);
    out[slen] = '\0';
    uart_lock_give(sensor);
    return ESP_OK;
}

// Set sensor to Energy Mode (structured packet output)
static esp_err_t set_energy_mode(ld2420_t* sensor) {
    ESP_LOGI(TAG, "Configuring sensor for Energy Mode (structured packets)...");
    if (!uart_lock_take(sensor, portMAX_DELAY)) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t err = ESP_OK;

    uint8_t enable_config[] = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x04, 0x00,
        0xFF, 0x00,
        0x02, 0x00,
        0x04, 0x03, 0x02, 0x01
    };

    uint8_t set_mode[] = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x08, 0x00,
        0x12, 0x00,
        0x00, 0x00,
        0x04, 0x00,
        0x00, 0x00,
        0x04, 0x03, 0x02, 0x01
    };

    uint8_t disable_config[] = {
        0xFD, 0xFC, 0xFB, 0xFA,
        0x02, 0x00,
        0xFE, 0x00,
        0x04, 0x03, 0x02, 0x01
    };

    do {
        ESP_LOGD(TAG, "Step 1: Enabling config mode...");
        err = send_command(sensor, enable_config, sizeof(enable_config));
        if (err != ESP_OK) {
            break;
        }
        err = read_ack(sensor, 500);
        if (err != ESP_OK) {
            break;
        }

        ESP_LOGD(TAG, "Step 2: Setting Energy Mode (0x04)...");
        err = send_command(sensor, set_mode, sizeof(set_mode));
        if (err != ESP_OK) {
            break;
        }
        err = read_ack(sensor, 500);
        if (err != ESP_OK) {
            break;
        }

        ESP_LOGD(TAG, "Step 3: Disabling config mode (saves settings)...");
        err = send_command(sensor, disable_config, sizeof(disable_config));
        if (err != ESP_OK) {
            break;
        }
        err = read_ack(sensor, 500);
        if (err != ESP_OK) {
            break;
        }

        err = uart_flush(sensor->uart_port);
    } while (false);

    uart_lock_give(sensor);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Energy Mode configuration complete");
        ESP_LOGD(TAG, "Sensor should now output packets with header F4 F3 F2 F1");
    } else {
        ESP_LOGW(TAG, "Energy Mode configuration failed (%s)", esp_err_to_name(err));
    }

    return err;
}

ld2420_t* ld2420_create(void) {
    ld2420_t* sensor = (ld2420_t*)calloc(1, sizeof(ld2420_t));
    if (sensor == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for sensor");
        return NULL;
    }
    sensor->uart_lock = xSemaphoreCreateRecursiveMutex();
    if (sensor->uart_lock == NULL) {
        ESP_LOGE(TAG, "Failed to create UART mutex");
        free(sensor);
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

    bool driver_installed_here = false;
    esp_err_t err = ESP_OK;

    const uart_config_t uart_config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    err = uart_driver_install(uart_port, BUF_SIZE * 2, BUF_SIZE * 2, 0, NULL, 0);
    if (err == ESP_OK) {
        driver_installed_here = true;
    } else if (err != ESP_ERR_INVALID_STATE) {
        return err;
    }

    err = uart_param_config(uart_port, &uart_config);
    if (err != ESP_OK) {
        goto fail;
    }

    err = uart_set_pin(uart_port, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        goto fail;
    }

    err = uart_flush(uart_port);
    if (err != ESP_OK) {
        goto fail;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Checking sensor output mode...");
    uint8_t test_buf[BUF_SIZE];
    int test_len = uart_read_bytes(uart_port, test_buf, sizeof(test_buf), pdMS_TO_TICKS(500));

    if (test_len > 0) {
        ESP_LOGD(TAG, "Sensor is outputting data (%d bytes)", test_len);

        int zero_count = 0;
        bool found_f4_header = false;

        for (int i = 0; i < test_len; i++) {
            if (test_buf[i] == 0x00) {
                zero_count++;
            }

            if (i <= test_len - 4) {
                if (test_buf[i] == 0xF4 && test_buf[i + 1] == 0xF3 &&
                    test_buf[i + 2] == 0xF2 && test_buf[i + 3] == 0xF1) {
                    found_f4_header = true;
                    ESP_LOGD(TAG, "Found Energy Mode header at offset %d - sensor already configured", i);
                }
            }
        }

        if (!found_f4_header) {
            if (zero_count > test_len * 0.8) {
                ESP_LOGW(TAG, "Sensor in Debug/Waveform mode (>80%% zeros)");
                ESP_LOGI(TAG, "Switching to Energy Mode...");
            } else {
                ESP_LOGI(TAG, "Unknown data format, attempting Energy Mode configuration...");
            }
            err = set_energy_mode(sensor);
            if (err != ESP_OK) {
                goto fail;
            }
        } else {
            ESP_LOGI(TAG, "Sensor already in Energy Mode - ready to parse packets");
        }
    } else {
        ESP_LOGW(TAG, "No initial data from sensor - configuring Energy Mode anyway");
        err = set_energy_mode(sensor);
        if (err != ESP_OK) {
            goto fail;
        }
    }

    return ESP_OK;

fail:
    if (driver_installed_here) {
        uart_driver_delete(uart_port);
    }
    return err;
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
    if (!uart_lock_take(sensor, 0)) {
        return;
    }
    
    uint8_t temp_buf[BUF_SIZE];
    int len = uart_read_bytes(sensor->uart_port, temp_buf, sizeof(temp_buf), 0);
    
    if (len <= 0) {
        uart_lock_give(sensor);
        return;
    }
    
    for (int i = 0; i < len; i++) {
        parse_byte(sensor, temp_buf[i]);
    }
    
    static int update_counter = 0;
    if (++update_counter % 500 == 0) {  // Every 5 seconds
        if (!sensor->current_data.isValid) {
            ESP_LOGW(TAG, "No valid Energy packets yet. Check if sensor needs reconfiguration.");
        }
    }
    uart_lock_give(sensor);
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
