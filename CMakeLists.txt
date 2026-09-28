cmake_minimum_required(VERSION 3.16)

# Single source of truth for the firmware version. Read at runtime via
# esp_app_get_description()->version (HA sw_version, OLED boot screen).
set(PROJECT_VER "2.5.0")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(presence_sensor_ld2420)
