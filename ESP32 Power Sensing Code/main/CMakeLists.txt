idf_component_register(SRCS "power_controller.c"
                    INCLUDE_DIRS "."
                    REQUIRES esp_adc esp_driver_i2c esp_driver_gpio esp_driver_rmt)
