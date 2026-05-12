#ifndef TOUCH_INPUT_H
#define TOUCH_INPUT_H

#include <esp_err.h>
#include <lvgl.h>

esp_err_t touch_init(void);
void touch_driver_read(lv_indev_drv_t *drv, lv_indev_data_t *data);

#endif