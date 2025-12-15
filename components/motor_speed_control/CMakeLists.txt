idf_component_register(
    SRCS "motor_speed_control.c"
    INCLUDE_DIRS "include"
    REQUIRES esp_common motor_l298n encoder pid_speed
)