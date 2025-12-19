idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "."
                       REQUIRES ir_sensor tcs34725 pca9685 
                       pid_speed motor_l298n encoder pid_speed 
                       motor_speed_control i2c_lcd freertos 
                       driver esp_timer esp_wifi esp_event 
                       nvs_flash esp_netif mqtt json driver
                       net_mqtt)
