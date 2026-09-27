# main/CMakeLists.txt

idf_component_register(
    SRCS "main.c" "oled_status.c" "device_creds.c"
    INCLUDE_DIRS "."
    REQUIRES 
        esp_wifi 
        esp_event 
        esp_netif 
        esp_timer
        nvs_flash
        esp_app_format
        mqtt 
        lwip
        ha_mqtt
        ota_update
        ld2420
        nixy4__u8g2
)
