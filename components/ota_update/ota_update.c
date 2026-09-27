#include "ota_update.h"

#include <ctype.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_app_desc.h"
#include "esp_crt_bundle.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mbedtls/sha256.h"

static const char *TAG = "ota_update";

#define OTA_TASK_STACK        8192
#define OTA_TASK_PRIO         4
#define OTA_READ_CHUNK        4096
#define OTA_HTTP_TIMEOUT_MS   15000
#define OTA_PROGRESS_STEP     5

typedef struct {
    char     url[256];
    char     sha256_hex[65];
    uint32_t size;
    char     version[32];
} ota_job_t;

static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static bool s_in_progress = false;
static bool s_pending_verify = false;
static esp_timer_handle_t s_rollback_timer = NULL;

static ota_job_t s_job;
static ota_update_progress_cb_t s_on_progress = NULL;
static ota_update_result_cb_t s_on_result = NULL;

/* ======================= Rollback guard ======================= */

static void rollback_timer_cb(void *arg) {
    (void)arg;
    bool pending;
    taskENTER_CRITICAL(&s_lock);
    pending = s_pending_verify;
    taskEXIT_CRITICAL(&s_lock);
    if (!pending) return;

    ESP_LOGE(TAG, "New firmware was not confirmed healthy in time; rolling back");
    esp_err_t err = esp_ota_mark_app_invalid_rollback_and_reboot();
    // Only returns on failure, e.g. no previous valid image to fall back to.
    ESP_LOGE(TAG, "Rollback failed (%s); keeping current image", esp_err_to_name(err));
}

void ota_update_init(uint32_t rollback_timeout_s) {
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t state = ESP_OTA_IMG_UNDEFINED;
    if (!running || esp_ota_get_state_partition(running, &state) != ESP_OK) {
        ESP_LOGI(TAG, "Running from %s (no OTA state)", running ? running->label : "?");
        return;
    }

    ESP_LOGI(TAG, "Running from %s (state=%d)", running->label, (int)state);
    if (state != ESP_OTA_IMG_PENDING_VERIFY) return;

    taskENTER_CRITICAL(&s_lock);
    s_pending_verify = true;
    taskEXIT_CRITICAL(&s_lock);

    const esp_timer_create_args_t args = {
        .callback = rollback_timer_cb,
        .name = "ota_rollback",
    };
    if (esp_timer_create(&args, &s_rollback_timer) == ESP_OK &&
        esp_timer_start_once(s_rollback_timer, (uint64_t)rollback_timeout_s * 1000000ULL) == ESP_OK) {
        ESP_LOGW(TAG, "New firmware pending verification; rollback in %" PRIu32 " s unless confirmed",
                 rollback_timeout_s);
    } else {
        ESP_LOGE(TAG, "Failed to arm rollback timer");
    }
}

bool ota_update_pending_verify(void) {
    bool pending;
    taskENTER_CRITICAL(&s_lock);
    pending = s_pending_verify;
    taskEXIT_CRITICAL(&s_lock);
    return pending;
}

void ota_update_mark_valid(void) {
    bool was_pending;
    taskENTER_CRITICAL(&s_lock);
    was_pending = s_pending_verify;
    s_pending_verify = false;
    taskEXIT_CRITICAL(&s_lock);
    if (!was_pending) return;

    if (s_rollback_timer) {
        esp_timer_stop(s_rollback_timer);
    }
    esp_err_t err = esp_ota_mark_app_valid_cancel_rollback();
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "New firmware confirmed healthy; rollback cancelled");
    } else {
        ESP_LOGE(TAG, "Failed to confirm firmware: %s", esp_err_to_name(err));
    }
}

/* ======================= Download + flash ======================= */

static bool sha256_hex_matches(const unsigned char digest[32], const char *expected_hex) {
    static const char hex[] = "0123456789abcdef";
    if (strlen(expected_hex) != 64) return false;
    for (int i = 0; i < 32; ++i) {
        if (tolower((unsigned char)expected_hex[2 * i]) != hex[digest[i] >> 4] ||
            tolower((unsigned char)expected_hex[2 * i + 1]) != hex[digest[i] & 0x0f]) {
            return false;
        }
    }
    return true;
}

static void report_progress(int percent) {
    if (s_on_progress) s_on_progress(percent);
}

static bool run_ota(char *msg, size_t msg_size) {
    bool ok = false;
    esp_http_client_handle_t client = NULL;
    esp_ota_handle_t ota = 0;
    unsigned char *buf = NULL;
    mbedtls_sha256_context sha;
    mbedtls_sha256_init(&sha);

    const esp_partition_t *target = esp_ota_get_next_update_partition(NULL);
    if (!target) {
        snprintf(msg, msg_size, "no OTA partition");
        goto cleanup;
    }
    if (s_job.size > target->size) {
        snprintf(msg, msg_size, "image larger than slot");
        goto cleanup;
    }

    esp_http_client_config_t hc = {
        .url = s_job.url,
        .timeout_ms = OTA_HTTP_TIMEOUT_MS,
        .buffer_size = 2048,
        .keep_alive_enable = true,
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    client = esp_http_client_init(&hc);
    if (!client) {
        snprintf(msg, msg_size, "http init failed");
        goto cleanup;
    }

    esp_err_t err = esp_http_client_open(client, 0);
    if (err != ESP_OK) {
        snprintf(msg, msg_size, "connect failed: %s", esp_err_to_name(err));
        goto cleanup;
    }

    int64_t content_len = esp_http_client_fetch_headers(client);
    int status = esp_http_client_get_status_code(client);
    if (status != 200) {
        snprintf(msg, msg_size, "HTTP %d", status);
        goto cleanup;
    }
    if (s_job.size && content_len > 0 && (uint64_t)content_len != s_job.size) {
        snprintf(msg, msg_size, "size mismatch (%" PRId64 ")", content_len);
        goto cleanup;
    }
    uint32_t expected = s_job.size ? s_job.size : (content_len > 0 ? (uint32_t)content_len : 0);

    buf = malloc(OTA_READ_CHUNK);
    if (!buf) {
        snprintf(msg, msg_size, "out of memory");
        goto cleanup;
    }

    err = esp_ota_begin(target, OTA_WITH_SEQUENTIAL_WRITES, &ota);
    if (err != ESP_OK) {
        ota = 0;
        snprintf(msg, msg_size, "ota begin: %s", esp_err_to_name(err));
        goto cleanup;
    }

    mbedtls_sha256_starts(&sha, 0);
    ESP_LOGI(TAG, "Downloading %" PRIu32 " bytes into %s", expected, target->label);

    uint32_t total = 0;
    int last_pct = 0;
    report_progress(0);
    while (true) {
        int n = esp_http_client_read(client, (char *)buf, OTA_READ_CHUNK);
        if (n < 0) {
            snprintf(msg, msg_size, "read error");
            goto cleanup;
        }
        if (n == 0) break;
        if ((uint64_t)total + (uint64_t)n > target->size) {
            snprintf(msg, msg_size, "image larger than slot");
            goto cleanup;
        }
        mbedtls_sha256_update(&sha, buf, (size_t)n);
        err = esp_ota_write(ota, buf, (size_t)n);
        if (err != ESP_OK) {
            snprintf(msg, msg_size, "flash write: %s", esp_err_to_name(err));
            goto cleanup;
        }
        total += (uint32_t)n;
        if (expected) {
            int pct = (int)(((uint64_t)total * 100U) / expected);
            if (pct > 99) pct = 99;  // 100 is reported only after validation
            if (pct >= last_pct + OTA_PROGRESS_STEP) {
                last_pct = pct;
                report_progress(pct);
            }
        }
    }

    if (expected && total != expected) {
        snprintf(msg, msg_size, "truncated (%" PRIu32 "/%" PRIu32 ")", total, expected);
        goto cleanup;
    }

    unsigned char digest[32];
    mbedtls_sha256_finish(&sha, digest);
    if (!sha256_hex_matches(digest, s_job.sha256_hex)) {
        snprintf(msg, msg_size, "sha256 mismatch");
        goto cleanup;
    }

    // Validates the image; with signed-app verification enabled this also
    // checks the signature against the key the running image was signed with.
    err = esp_ota_end(ota);
    ota = 0;
    if (err == ESP_ERR_OTA_VALIDATE_FAILED) {
        snprintf(msg, msg_size, "image invalid or not signed");
        goto cleanup;
    } else if (err != ESP_OK) {
        snprintf(msg, msg_size, "ota end: %s", esp_err_to_name(err));
        goto cleanup;
    }

    esp_app_desc_t desc;
    if (esp_ota_get_partition_description(target, &desc) != ESP_OK) {
        snprintf(msg, msg_size, "no app description");
        goto cleanup;
    }
    if (s_job.version[0] && strncmp(desc.version, s_job.version, sizeof(desc.version)) != 0) {
        snprintf(msg, msg_size, "version mismatch (%.20s)", desc.version);
        goto cleanup;
    }

    err = esp_ota_set_boot_partition(target);
    if (err != ESP_OK) {
        snprintf(msg, msg_size, "set boot: %s", esp_err_to_name(err));
        goto cleanup;
    }

    report_progress(100);
    snprintf(msg, msg_size, "installed %.20s", desc.version);
    ok = true;

cleanup:
    if (ota) esp_ota_abort(ota);
    mbedtls_sha256_free(&sha);
    free(buf);
    if (client) {
        esp_http_client_close(client);
        esp_http_client_cleanup(client);
    }
    return ok;
}

static void ota_task(void *arg) {
    (void)arg;
    char msg[64] = {0};
    bool ok = run_ota(msg, sizeof(msg));

    if (ok) {
        ESP_LOGW(TAG, "OTA succeeded: %s; restarting", msg);
    } else {
        ESP_LOGE(TAG, "OTA failed: %s", msg);
    }
    if (s_on_result) s_on_result(ok, msg);

    if (ok) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        esp_restart();
    }

    taskENTER_CRITICAL(&s_lock);
    s_in_progress = false;
    taskEXIT_CRITICAL(&s_lock);
    vTaskDelete(NULL);
}

esp_err_t ota_update_start(const ota_update_request_t *req,
                           ota_update_progress_cb_t on_progress,
                           ota_update_result_cb_t on_result) {
    if (!req || !req->url || !req->sha256_hex || strlen(req->sha256_hex) != 64 ||
        strlen(req->url) >= sizeof(s_job.url)) {
        return ESP_ERR_INVALID_ARG;
    }

    taskENTER_CRITICAL(&s_lock);
    bool busy = s_in_progress;
    if (!busy) s_in_progress = true;
    taskEXIT_CRITICAL(&s_lock);
    if (busy) return ESP_ERR_INVALID_STATE;

    memset(&s_job, 0, sizeof(s_job));
    snprintf(s_job.url, sizeof(s_job.url), "%s", req->url);
    snprintf(s_job.sha256_hex, sizeof(s_job.sha256_hex), "%s", req->sha256_hex);
    snprintf(s_job.version, sizeof(s_job.version), "%s", req->version ? req->version : "");
    s_job.size = req->size;
    s_on_progress = on_progress;
    s_on_result = on_result;

    if (xTaskCreate(ota_task, "ota_update", OTA_TASK_STACK, NULL, OTA_TASK_PRIO, NULL) != pdPASS) {
        taskENTER_CRITICAL(&s_lock);
        s_in_progress = false;
        taskEXIT_CRITICAL(&s_lock);
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

bool ota_update_in_progress(void) {
    bool busy;
    taskENTER_CRITICAL(&s_lock);
    busy = s_in_progress;
    taskEXIT_CRITICAL(&s_lock);
    return busy;
}
