idf_component_register(SRCS "light.c"
                    INCLUDE_DIRS .
                    REQUIRES console esp_driver_gpio esp_driver_ledc pthread nvs_flash)
