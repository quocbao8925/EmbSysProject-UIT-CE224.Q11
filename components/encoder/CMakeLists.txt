idf_component_register(
    SRCS "encoder.c"
    INCLUDE_DIRS "include"        # encoder.h
    REQUIRES driver esp_timer freertos     # depends on these components
)