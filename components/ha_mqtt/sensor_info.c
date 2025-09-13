// components/ha_mqtt/sensor_info.c
#include "sensor_info.h"
#include <string.h>

// Simple storage for firmware version
static char s_fw_version[64] = {0};

const char* sensor_info_get_fw(void) {
    if (s_fw_version[0] == '\0') {
        // Return a default if no firmware version has been detected yet
        return "LD2420 v1.0";
    }
    return s_fw_version;
}

// Internal function to set firmware version (can be called from the LD2420 code)
void sensor_info_set_fw(const char* fw_version) {
    if (fw_version && fw_version[0]) {
        size_t len = strlen(fw_version);
        if (len >= sizeof(s_fw_version)) {
            len = sizeof(s_fw_version) - 1;
        }
        memcpy(s_fw_version, fw_version, len);
        s_fw_version[len] = '\0';
    }
}