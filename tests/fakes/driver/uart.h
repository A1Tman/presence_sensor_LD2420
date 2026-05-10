#pragma once
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

typedef int uart_port_t;

#define UART_PIN_NO_CHANGE (-1)
#define UART_DATA_8_BITS 8
#define UART_PARITY_DISABLE 0
#define UART_STOP_BITS_1 1
#define UART_HW_FLOWCTRL_DISABLE 0
#define UART_SCLK_DEFAULT 0

typedef struct {
    int baud_rate;
    int data_bits;
    int parity;
    int stop_bits;
    int flow_ctrl;
    int source_clk;
} uart_config_t;

int uart_read_bytes(uart_port_t uart_num, void *buf, uint32_t length, TickType_t ticks_to_wait);
int uart_write_bytes(uart_port_t uart_num, const char *src, size_t size);
esp_err_t uart_wait_tx_done(uart_port_t uart_num, TickType_t ticks_to_wait);
esp_err_t uart_flush(uart_port_t uart_num);
esp_err_t uart_driver_install(uart_port_t uart_num, int rx_buffer_size, int tx_buffer_size,
                              int queue_size, void *uart_queue, int intr_alloc_flags);
esp_err_t uart_driver_delete(uart_port_t uart_num);
esp_err_t uart_param_config(uart_port_t uart_num, const uart_config_t *uart_config);
esp_err_t uart_set_pin(uart_port_t uart_num, int tx_io_num, int rx_io_num,
                       int rts_io_num, int cts_io_num);

