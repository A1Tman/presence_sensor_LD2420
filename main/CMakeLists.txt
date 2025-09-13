# main/CMakeLists.txt

idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        esp_wifi 
        esp_event 
        esp_netif 
        esp_timer
        nvs_flash 
        mqtt 
        lwip
        ha_mqtt
        ld2420
)