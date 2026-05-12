#ifndef SETTINGS_H
#define SETTINGS_H

#include <esp_err.h>

typedef struct {
    int brightness;
    int timeout_seconds;
    bool auto_brightness;
} system_settings_t;

esp_err_t settings_load(system_settings_t *settings);
esp_err_t settings_save(const system_settings_t *settings);
esp_err_t settings_reset(void);

#endif