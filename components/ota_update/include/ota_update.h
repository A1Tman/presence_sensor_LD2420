#pragma once
#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    const char *url;         // http:// or https:// image URL
    const char *sha256_hex;  // 64 lowercase/uppercase hex chars of the whole .bin
    uint32_t    size;        // expected image size in bytes (0 = do not check)
    const char *version;     // expected esp_app_desc_t.version of the new image
} ota_update_request_t;

// Called from the OTA worker task. percent is 0..100.
typedef void (*ota_update_progress_cb_t)(int percent);
// Called once from the OTA worker task when the download ends. On success the
// device restarts ~1 s after this returns; message is a short reason on failure.
typedef void (*ota_update_result_cb_t)(bool ok, const char *message);

/**
 * Inspect the running image's OTA state. If it is a freshly installed image
 * awaiting confirmation (rollback pending), arm a one-shot timer that rolls
 * back to the previous image unless ota_update_mark_valid() is called within
 * timeout_s. Call early in app_main, before anything that may return early.
 */
void ota_update_init(uint32_t rollback_timeout_s);

/** True while the running image still needs ota_update_mark_valid(). */
bool ota_update_pending_verify(void);

/** Confirm the running image and cancel the rollback timer. Idempotent. */
void ota_update_mark_valid(void);

/**
 * Start a download + flash in a background task. The request strings are
 * copied. The image is streamed into the inactive OTA slot, its SHA-256 is
 * compared against req->sha256_hex, then esp_ota_end() validates the image
 * (and its signature when CONFIG_SECURE_SIGNED_ON_UPDATE_NO_SECURE_BOOT).
 * Only then is the boot partition switched and the device restarted.
 */
esp_err_t ota_update_start(const ota_update_request_t *req,
                           ota_update_progress_cb_t on_progress,
                           ota_update_result_cb_t on_result);

/** True while a download/flash task is running. */
bool ota_update_in_progress(void);

#ifdef __cplusplus
}
#endif
