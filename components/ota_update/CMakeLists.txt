# components/ota_update/CMakeLists.txt

idf_component_register(
    SRCS "ota_update.c"
    INCLUDE_DIRS "include"
    REQUIRES
        app_update
        esp_app_format
        esp_http_client
        esp_timer
        mbedtls
)
