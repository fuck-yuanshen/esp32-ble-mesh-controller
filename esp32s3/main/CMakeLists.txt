idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES bt nvs_flash esp_driver_gpio esp_driver_uart freertos log esp_system)
