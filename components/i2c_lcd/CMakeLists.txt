idf_component_register(
    SRCS "i2c_lcd.c"
    INCLUDE_DIRS "include"      
    REQUIRES driver freertos      # depends on these components
)