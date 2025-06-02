idf_component_register(SRCS "main.c"
                    REQUIRES driver
                    REQUIRES esp_rainmaker
                    REQUIRES app_network
                    REQUIRES app_insights
                    REQUIRES nvs_flash
                    # REQUIRES esp_bt
                    INCLUDE_DIRS "")