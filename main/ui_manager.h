#ifndef UI_MANAGER_H
#define UI_MANAGER_H

#include <esp_err.h>
#include <lvgl.h>

esp_err_t ui_init(void);
void ui_task_handler(void);
void ui_create_demo(void);  // Keep old demo for testing
void ui_create_robot_control(void);  // New robot control UI
void ui_clear_screen(void);

// Widget access functions
void ui_set_terminal_text(const char *text);
void ui_update_state_values(float *values, int count);
void ui_set_work_offset(int index);
void ui_set_tool_orientation(int index);
void ui_set_robot_state(int index);
void ui_set_control_mode(bool is_cartesian);
void ui_set_speed(float percent);

#endif