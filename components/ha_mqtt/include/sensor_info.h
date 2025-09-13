// components/ha_mqtt/include/sensor_info.h
#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Get the firmware version string from the sensor
 * @return Firmware version string, or NULL if not available
 */
const char* sensor_info_get_fw(void);

#ifdef __cplusplus
}
#endif