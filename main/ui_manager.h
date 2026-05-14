#ifndef UI_MANAGER_H
#define UI_MANAGER_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// Тип колбэка для команд в ROS
typedef void (*ui_command_callback_t)(int type, int id, float value);

// Инициализация UI
esp_err_t ui_init(void);
void ui_task_handler(void);

// Создание UI интерфейса
void ui_create_robot_control(void);
void ui_create_demo(void);
void ui_clear_screen(void);

// Функции для обновления UI данными из ROS
void ui_update_state_values(float *values, int count);
void ui_update_telemetry(float *values, int count, uint8_t state, 
                        uint8_t tool_id, uint8_t work_offset_id, 
                        bool cartesian_mode, float speed_override);
void ui_add_debug_log(const char *message);
void ui_set_terminal_text(const char *text);
void ui_auto_color_terminal_text(const char *text);
void ui_add_debug_line(const char *text);

// Функции для получения текущего состояния UI
uint8_t ui_get_tool_id(void);
uint8_t ui_get_work_offset(void);
uint8_t ui_get_robot_state(void);
bool ui_get_cartesian_mode(void);
float ui_get_speed_override(void);

// Регистрация колбэка для отправки команд в ROS
void ui_set_command_callback(ui_command_callback_t callback);

// Функции для установки состояния UI (из ROS)
void ui_set_work_offset(int index);
void ui_set_tool_orientation(int index);
void ui_set_robot_state(int index);
void ui_set_control_mode(bool is_cartesian);
void ui_set_speed(float percent);
void ui_update_container_color(uint8_t state);
void ui_update_work_display(uint8_t id);
void ui_update_tool_display(uint8_t id);
void ui_update_mode_display(bool cartesian);
void ui_update_speed_display(float speed);
void ui_update_state_display(uint8_t state);
void ui_set_motion_state(bool in_motion);


#endif