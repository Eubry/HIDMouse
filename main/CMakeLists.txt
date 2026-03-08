idf_component_register(SRCS "main.cpp" 
                    "../lib/DevBLib/Common/libraries/Counter/Counter.cpp"
                    INCLUDE_DIRS "." 
                    "../lib/DevBLib/Common/libraries/Counter"
                    REQUIRES esp_timer freertos esp_system esp_wifi esp_netif nvs_flash esp_driver_gpio esp_driver_ledc esp_http_server)
