idf_component_register(
    SRCS "acm_core.c" "main.c"
    INCLUDE_DIRS "."
    REQUIRES 
        driver 
        esp_timer 
        nvs_flash
        freertos
        esp_common
        mavlink_lib
    EMBED_FILES
)
