idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES esp_timer esp_driver_gpio esp_driver_touch_sens spi_flash esp_wifi nvs_flash lwio)
