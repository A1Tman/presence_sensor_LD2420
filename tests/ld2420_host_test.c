#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_err.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

static uint8_t g_uart_rx[512];
static size_t g_uart_rx_len;
static size_t g_uart_rx_pos;
static TickType_t g_tick;

static void feed_uart(const uint8_t *data, size_t len) {
    assert(len <= sizeof(g_uart_rx));
    memcpy(g_uart_rx, data, len);
    g_uart_rx_len = len;
    g_uart_rx_pos = 0;
    g_tick = 0;
}

const char *esp_err_to_name(esp_err_t err) {
    (void)err;
    return "ERR";
}

int64_t esp_timer_get_time(void) {
    return 123456;
}

TickType_t xTaskGetTickCount(void) {
    return g_tick++;
}

void vTaskDelay(TickType_t ticks) {
    g_tick += ticks;
}

SemaphoreHandle_t xSemaphoreCreateMutex(void) {
    static int mutex;
    return &mutex;
}

SemaphoreHandle_t xSemaphoreCreateRecursiveMutex(void) {
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

BaseType_t xSemaphoreTakeRecursive(SemaphoreHandle_t semaphore, TickType_t ticks) {
    (void)semaphore;
    (void)ticks;
    return pdTRUE;
}

BaseType_t xSemaphoreGiveRecursive(SemaphoreHandle_t semaphore) {
    (void)semaphore;
    return pdTRUE;
}

void vSemaphoreDelete(SemaphoreHandle_t semaphore) {
    (void)semaphore;
}

int uart_read_bytes(uart_port_t uart_num, void *buf, uint32_t length, TickType_t ticks_to_wait) {
    (void)uart_num;
    (void)ticks_to_wait;
    if (g_uart_rx_pos >= g_uart_rx_len) {
        g_tick += 25;
        return 0;
    }
    size_t remaining = g_uart_rx_len - g_uart_rx_pos;
    size_t chunk = remaining < length ? remaining : length;
    if (chunk > 3) chunk = 3; // force fragmented reads
    memcpy(buf, g_uart_rx + g_uart_rx_pos, chunk);
    g_uart_rx_pos += chunk;
    g_tick++;
    return (int)chunk;
}

int uart_write_bytes(uart_port_t uart_num, const char *src, size_t size) {
    (void)uart_num;
    (void)src;
    return (int)size;
}

esp_err_t uart_wait_tx_done(uart_port_t uart_num, TickType_t ticks_to_wait) {
    (void)uart_num;
    (void)ticks_to_wait;
    return ESP_OK;
}

esp_err_t uart_flush(uart_port_t uart_num) {
    (void)uart_num;
    return ESP_OK;
}

esp_err_t uart_driver_install(uart_port_t uart_num, int rx_buffer_size, int tx_buffer_size,
                              int queue_size, void *uart_queue, int intr_alloc_flags) {
    (void)uart_num;
    (void)rx_buffer_size;
    (void)tx_buffer_size;
    (void)queue_size;
    (void)uart_queue;
    (void)intr_alloc_flags;
    return ESP_OK;
}

esp_err_t uart_driver_delete(uart_port_t uart_num) {
    (void)uart_num;
    return ESP_OK;
}

esp_err_t uart_param_config(uart_port_t uart_num, const uart_config_t *uart_config) {
    (void)uart_num;
    (void)uart_config;
    return ESP_OK;
}

esp_err_t uart_set_pin(uart_port_t uart_num, int tx_io_num, int rx_io_num,
                       int rts_io_num, int cts_io_num) {
    (void)uart_num;
    (void)tx_io_num;
    (void)rx_io_num;
    (void)rts_io_num;
    (void)cts_io_num;
    return ESP_OK;
}

esp_err_t gpio_config(const gpio_config_t *config) {
    (void)config;
    return ESP_OK;
}

int gpio_get_level(gpio_num_t gpio_num) {
    (void)gpio_num;
    return 0;
}

#include "../components/ld2420/ld2420.c"

static ld2420_t test_sensor(void) {
    ld2420_t sensor = {
        .uart_port = 1,
        .uart_lock = (SemaphoreHandle_t)0x1,
    };
    return sensor;
}

static size_t append_response(uint8_t *out, size_t pos, const uint8_t *payload,
                              uint16_t payload_len, bool good_footer) {
    out[pos++] = 0xFD;
    out[pos++] = 0xFC;
    out[pos++] = 0xFB;
    out[pos++] = 0xFA;
    out[pos++] = (uint8_t)(payload_len & 0xff);
    out[pos++] = (uint8_t)(payload_len >> 8);
    memcpy(out + pos, payload, payload_len);
    pos += payload_len;
    if (good_footer) {
        out[pos++] = 0x04;
        out[pos++] = 0x03;
        out[pos++] = 0x02;
        out[pos++] = 0x01;
    } else {
        out[pos++] = 0xaa;
        out[pos++] = 0xbb;
        out[pos++] = 0xcc;
        out[pos++] = 0xdd;
    }
    return pos;
}

static void test_read_response_skips_noise(void) {
    ld2420_t sensor = test_sensor();
    uint8_t stream[64] = {0x99, 0x88, 0x77};
    const uint8_t payload[] = {0x08, 0x00, 0x00, 0x00};
    size_t stream_len = append_response(stream, 3, payload, sizeof(payload), true);

    uint8_t rx[64];
    size_t out_len = 0;
    feed_uart(stream, stream_len);
    assert(read_response(&sensor, rx, sizeof(rx), 200, &out_len) == ESP_OK);
    assert(out_len == 14);
    assert(rx[0] == 0xFD && rx[1] == 0xFC && rx[2] == 0xFB && rx[3] == 0xFA);
    printf("ok ld2420_read_response_skips_noise\n");
}

static void test_read_response_rejects_bad_footer_then_recovers(void) {
    ld2420_t sensor = test_sensor();
    uint8_t stream[96] = {0};
    const uint8_t payload[] = {0x08, 0x00, 0x00, 0x00};
    size_t pos = append_response(stream, 0, payload, sizeof(payload), false);
    stream[pos++] = 0x55;
    pos = append_response(stream, pos, payload, sizeof(payload), true);

    uint8_t rx[64];
    size_t out_len = 0;
    feed_uart(stream, pos);
    assert(read_response(&sensor, rx, sizeof(rx), 200, &out_len) == ESP_OK);
    assert(out_len == 14);
    assert(rx[0] == 0xFD && rx[1] == 0xFC && rx[2] == 0xFB && rx[3] == 0xFA);
    printf("ok ld2420_read_response_rejects_bad_footer_then_recovers\n");
}

static void test_read_response_times_out_on_partial_frame(void) {
    ld2420_t sensor = test_sensor();
    const uint8_t partial[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0x08};
    uint8_t rx[64];
    size_t out_len = 0;
    feed_uart(partial, sizeof(partial));
    assert(read_response(&sensor, rx, sizeof(rx), 20, &out_len) == ESP_ERR_TIMEOUT);
    printf("ok ld2420_read_response_times_out_on_partial_frame\n");
}

static void test_read_ack_status_failure(void) {
    ld2420_t sensor = test_sensor();
    uint8_t stream[64] = {0};
    const uint8_t payload[] = {0x12, 0x00, 0x01, 0x00};
    size_t stream_len = append_response(stream, 0, payload, sizeof(payload), true);
    feed_uart(stream, stream_len);
    assert(read_ack(&sensor, 200) == ESP_FAIL);
    printf("ok ld2420_read_ack_status_failure\n");
}

int main(void) {
    test_read_response_skips_noise();
    test_read_response_rejects_bad_footer_then_recovers();
    test_read_response_times_out_on_partial_frame();
    test_read_ack_status_failure();
    return 0;
}

