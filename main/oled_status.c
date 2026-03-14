#include "oled_status.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "u8g2.h"
#include "esp32_hw_i2c.h"

static const char *TAG = "OLED_STATUS";

#define OLED_I2C_PORT          0
#define OLED_SDA_PIN           5
#define OLED_SCL_PIN           6
#define OLED_I2C_ADDR_7BIT     0x3C
#define OLED_I2C_CLK_HZ        1000000U
#define OLED_TIMEOUT_MS        100
#define OLED_CONTRAST          30
#define OLED_REFRESH_MS        250
#define OLED_BOOT_SCREEN_MS    2500
#define OLED_PAGE_ROTATE_MS    4000
#define OLED_LINE_COUNT        5
#define OLED_BODY_LINE_COUNT   4
#define OLED_LINE_LEN          16

static u8g2_t s_u8g2;
static u8g2_esp32_i2c_ctx_t s_i2c_ctx;
static oled_status_snapshot_cb_t s_snapshot_cb;
static char s_app_version[16];
static TickType_t s_started_at;

static uint16_t clamp_display_distance(int distance_cm)
{
    if (distance_cm < 0) {
        return 0;
    }
    if (distance_cm > 999) {
        return 999;
    }
    return (uint16_t)distance_cm;
}

static void draw_page(const char *title, const char lines[OLED_BODY_LINE_COUNT][OLED_LINE_LEN])
{
    u8g2_ClearBuffer(&s_u8g2);
    u8g2_SetFont(&s_u8g2, u8g2_font_5x8_tf);
    u8g2_SetFontPosTop(&s_u8g2);
    u8g2_SetFontMode(&s_u8g2, 0);

    u8g2_SetDrawColor(&s_u8g2, 1);
    u8g2_DrawBox(&s_u8g2, 0, 0, 72, 8);
    u8g2_SetDrawColor(&s_u8g2, 0);
    u8g2_DrawStr(&s_u8g2, 1, 0, title);
    u8g2_SetDrawColor(&s_u8g2, 1);

    for (int i = 0; i < OLED_BODY_LINE_COUNT; ++i) {
        u8g2_DrawStr(&s_u8g2, 1, (i + 1) * 8, lines[i]);
    }

    u8g2_SendBuffer(&s_u8g2);
}

static void render_boot_screen(void)
{
    char lines[OLED_BODY_LINE_COUNT][OLED_LINE_LEN] = {{0}};

    snprintf(lines[0], sizeof(lines[0]), "LD2420");
    snprintf(lines[1], sizeof(lines[1]), "Presence");
    snprintf(lines[2], sizeof(lines[2]), "%.14s", s_app_version[0] ? s_app_version : "boot");
    snprintf(lines[3], sizeof(lines[3]), "Starting");

    draw_page("BOOT", lines);
}

static void render_fault_screen(const oled_status_snapshot_t *snapshot)
{
    char lines[OLED_BODY_LINE_COUNT][OLED_LINE_LEN] = {{0}};

    if (!snapshot->sensor_ready) {
        snprintf(lines[0], sizeof(lines[0]), "Radar init");
        snprintf(lines[1], sizeof(lines[1]), "Check UART");
    } else if (!snapshot->sensor_packets_valid) {
        snprintf(lines[0], sizeof(lines[0]), "Radar wait");
        snprintf(lines[1], sizeof(lines[1]), "No data");
    } else if (!snapshot->wifi_connected) {
        snprintf(lines[0], sizeof(lines[0]), "WiFi down");
        snprintf(lines[1], sizeof(lines[1]), "Retrying");
    } else {
        snprintf(lines[0], sizeof(lines[0]), "MQTT wait");
        snprintf(lines[1], sizeof(lines[1]), "Broker?");
    }

    snprintf(lines[2], sizeof(lines[2]), "Pres %s", snapshot->presence ? "ON" : "OFF");

    if (snapshot->distance_cm >= 0) {
        uint16_t dist = clamp_display_distance(snapshot->distance_cm);
        snprintf(lines[3], sizeof(lines[3]), "Dist %ucm", (unsigned)dist);
    } else {
        snprintf(lines[3], sizeof(lines[3]), "Dist ---");
    }

    draw_page("FAULT", lines);
}

static void render_status_page(const oled_status_snapshot_t *snapshot)
{
    char lines[OLED_BODY_LINE_COUNT][OLED_LINE_LEN] = {{0}};

    snprintf(lines[0], sizeof(lines[0]), "Pres %s", snapshot->presence ? "ON" : "OFF");

    if (snapshot->distance_cm >= 0) {
        uint16_t dist = clamp_display_distance(snapshot->distance_cm);
        snprintf(lines[1], sizeof(lines[1]), "Dist %ucm", (unsigned)dist);
    } else {
        snprintf(lines[1], sizeof(lines[1]), "Dist ---");
    }

    if (snapshot->wifi_connected) {
        snprintf(lines[2], sizeof(lines[2]), "WiFi %d", snapshot->rssi_dbm);
    } else {
        snprintf(lines[2], sizeof(lines[2]), "WiFi --");
    }

    if (snapshot->wifi_connected) {
        snprintf(lines[3], sizeof(lines[3]), "Net %s .%u",
                 snapshot->mqtt_connected ? "OK" : "--",
                 snapshot->ip_last_octet);
    } else {
        snprintf(lines[3], sizeof(lines[3]), "Net %s",
                 snapshot->mqtt_connected ? "OK" : "--");
    }

    draw_page("STATUS", lines);
}

static void render_config_page(const oled_status_snapshot_t *snapshot)
{
    char lines[OLED_BODY_LINE_COUNT][OLED_LINE_LEN] = {{0}};

    snprintf(lines[0], sizeof(lines[0]), "Gate %d-%d", snapshot->min_gate, snapshot->max_gate);
    snprintf(lines[1], sizeof(lines[1]), "Delay %dms", snapshot->delay_ms);
    snprintf(lines[2], sizeof(lines[2]), "Trig %d", snapshot->trigger_sens);
    snprintf(lines[3], sizeof(lines[3]), "T %d %.6s",
             snapshot->maintain_sens,
             snapshot->fw_version[0] ? snapshot->fw_version : "?");

    draw_page("TUNING", lines);
}

static void oled_status_task(void *arg)
{
    (void)arg;

    while (1) {
        oled_status_snapshot_t snapshot = {0};
        if (s_snapshot_cb != NULL) {
            s_snapshot_cb(&snapshot);
        }

        uint32_t elapsed_ms = (uint32_t)(pdTICKS_TO_MS(xTaskGetTickCount() - s_started_at));
        bool boot_screen = elapsed_ms < OLED_BOOT_SCREEN_MS;
        bool fault = !snapshot.sensor_ready || !snapshot.sensor_packets_valid ||
                     !snapshot.wifi_connected || !snapshot.mqtt_connected;

        if (boot_screen) {
            render_boot_screen();
        } else if (fault) {
            render_fault_screen(&snapshot);
        } else if (((elapsed_ms - OLED_BOOT_SCREEN_MS) / OLED_PAGE_ROTATE_MS) % 2 == 0) {
            render_status_page(&snapshot);
        } else {
            render_config_page(&snapshot);
        }

        vTaskDelay(pdMS_TO_TICKS(OLED_REFRESH_MS));
    }
}

bool oled_status_init(oled_status_snapshot_cb_t snapshot_cb, const char *app_version)
{
    s_snapshot_cb = snapshot_cb;
    s_started_at = xTaskGetTickCount();
    memset(&s_i2c_ctx, 0, sizeof(s_i2c_ctx));
    s_i2c_ctx.cfg = (u8g2_esp32_i2c_config_t)U8G2_ESP32_I2C_CONFIG_DEFAULT();
    s_i2c_ctx.cfg.i2c_port = OLED_I2C_PORT;
    s_i2c_ctx.cfg.sda_pin = OLED_SDA_PIN;
    s_i2c_ctx.cfg.scl_pin = OLED_SCL_PIN;
    s_i2c_ctx.cfg.clk_hz = OLED_I2C_CLK_HZ;
    s_i2c_ctx.cfg.dev_addr_7bit = OLED_I2C_ADDR_7BIT;
    s_i2c_ctx.cfg.timeout_ms = OLED_TIMEOUT_MS;

    if (app_version != NULL) {
        snprintf(s_app_version, sizeof(s_app_version), "%s", app_version);
    } else {
        s_app_version[0] = '\0';
    }

    ESP_ERROR_CHECK(u8g2_esp32_i2c_set_default_context(&s_i2c_ctx));

    u8g2_Setup_ssd1306_i2c_72x40_er_f(&s_u8g2, U8G2_R0,
                                      u8x8_byte_esp32_hw_i2c,
                                      u8x8_gpio_and_delay_esp32_i2c);
    u8g2_SetI2CAddress(&s_u8g2, OLED_I2C_ADDR_7BIT << 1);
    u8g2_InitDisplay(&s_u8g2);
    u8g2_SetPowerSave(&s_u8g2, 0);
    u8g2_SetContrast(&s_u8g2, OLED_CONTRAST);

    BaseType_t task_ok = xTaskCreate(oled_status_task, "oled_status", 4096, NULL, 4, NULL);
    if (task_ok != pdPASS) {
        ESP_LOGE(TAG, "Failed to create OLED task");
        return false;
    }

    ESP_LOGI(TAG, "OLED status display enabled on I2C%d SDA=%d SCL=%d",
             OLED_I2C_PORT, OLED_SDA_PIN, OLED_SCL_PIN);
    return true;
}
