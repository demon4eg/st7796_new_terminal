idf_component_register(SRCS main.c display_config.c ui_manager.c touch_input.c ros_manager.c
                    REQUIRES esp_wifi esp_event nvs_flash micro_ros_espidf_component
                    INCLUDE_DIRS ""
                    )