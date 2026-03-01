idf_component_register(
        SRCS "esp-sparkmax.c"
        INCLUDE_DIRS "include"
        REQUIRES driver freertos esp_driver_twai
)