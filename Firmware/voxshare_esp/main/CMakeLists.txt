idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        esp-opus 
        esp_event 
        nvs_flash 
        esp_eth 
        esp_netif
        esp_wifi
        lwip
        driver
        esp_timer
        esp_system
 )
