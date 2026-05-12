#ifndef UI_MANAGER_H
#define UI_MANAGER_H

#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// Типы команд для ROS
typedef enum {
    CMD_TYPE_WORK_OFFSET = 1,
    CMD_TYPE_TOOL_ID = 2,
    CMD_TYPE_STATE = 3,
    CMD_TYPE_AUTO_STATE = 4,
    CMD_TYPE_CARTESIAN_MODE = 5,
    CMD_TYPE_SPEED_OVERRIDE = 6,
    CMD_TYPE_ESTOP = 7,
    CMD_TYPE_ACTION_TRIGGER = 8
} ui_command_type_t;

// Колбэк для отправки команд в ROS
typedef void (*ui_command_callback_t)(int type, int id, float value);

// Инициализация UI
esp_err_t ui_init(void);
void ui_task_handler(void);

// Создание UI интерфейса
void ui_create_robot_control(void);
void ui_create_demo(void);
void ui_clear_screen(void);

// Функции для обновления UI данными из ROS
void ui_update_telemetry(float *values, int count, uint8_t state, 
                        uint8_t tool_id, uint8_t work_offset_id, 
                        bool cartesian_mode, float speed_override);
void ui_add_debug_log(const char *message);

// Функции для получения текущего состояния UI
uint8_t ui_get_tool_id(void);
uint8_t ui_get_work_offset_id(void);
uint8_t ui_get_robot_state(void);
uint8_t ui_get_auto_state(void);
bool ui_get_cartesian_mode(void);
float ui_get_speed_override(void);
bool ui_get_estop_state(void);

// Регистрация колбэка для команд
void ui_register_command_callback(ui_command_callback_t callback);

// Отправка команды из UI
void ui_send_command(ui_command_type_t type, int id, float value);

// Старые функции для обратной совместимости
void ui_set_terminal_text(const char *text);
void ui_update_state_values(float *values, int count);
void ui_set_work_offset(int index);
void ui_set_tool_orientation(int index);
void ui_set_robot_state(int index);
void ui_set_control_mode(bool is_cartesian);
void ui_set_speed(float percent);

#endif