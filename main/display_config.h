#ifndef DISPLAY_CONFIG_H
#define DISPLAY_CONFIG_H

#include <esp_err.h>
#include <esp_lcd_panel_ops.h>

esp_err_t display_init(void);
void display_set_brightness(int brightness_percent);
void display_backlight_off(void);
void display_backlight_on(void);
esp_lcd_panel_handle_t display_get_panel_handle(void);

#endif