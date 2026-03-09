idf_component_register(SRCS "main.cpp"
                    "esp_hid_gap.c"
                    "../lib/DevBLib/Common/libraries/Counter/Counter.cpp"
                    INCLUDE_DIRS "."
                    "../lib/DevBLib/Common/libraries/Counter"
                    REQUIRES
                        esp_timer
                        freertos
                        esp_system
                        esp_wifi
                        esp_netif
                        bt
                        nvs_flash
                        esp_driver_gpio
                        esp_driver_ledc
                        esp_driver_i2c
                        esp_http_server
                        esp_hid
                        )
