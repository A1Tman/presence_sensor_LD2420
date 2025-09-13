# components/ha_mqtt/CMakeLists.txt

idf_component_register(
    SRCS "ha_mqtt.c" "sensor_info.c"
    INCLUDE_DIRS "include"
    REQUIRES 
        esp_wifi 
        esp_event 
        esp_netif 
        esp_timer
        mqtt 
        lwip
        json
)