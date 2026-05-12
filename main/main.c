#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "display_config.h"
#include "ui_manager.h"
#include "ros_manager.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "Starting application...");
    
    // Initialize display hardware
    ESP_ERROR_CHECK(display_init());
    
    // Initialize UI system
    ESP_ERROR_CHECK(ui_init());
    
    // Create robot control UI (instead of demo)
    ui_create_robot_control();
    
    // Test: Update some values
    float test_vals[] = {10.5, 20.3, 30.1, 40.2, 50.4, 60.7, 70.1, 80.2, 90.3, 100.4, 110.5, 120.6, 130.7};
    ui_update_state_values(test_vals, 13);
    
    // Turn on backlight
    display_set_brightness(75);
    
    ESP_LOGI(TAG, "Application ready");

    ros_manager_init();
    
    // Main loop
    while (1) {
        ui_task_handler();
    }
}