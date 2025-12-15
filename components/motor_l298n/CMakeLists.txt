idf_component_register(
    SRCS "motor_l298n.c"
    INCLUDE_DIRS "include"        # motor_l298n.h
    REQUIRES driver freertos      # depends on these components
)