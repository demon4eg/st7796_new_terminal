#include "ui_manager.h"
#include "display_config.h"
#include "touch_input.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lvgl.h>
#include <math.h>
#include <freertos/semphr.h>

static const char *TAG = "ui_manager";

// Display dimensions
#define DISPLAY_WIDTH   320
#define DISPLAY_HEIGHT  480
#define LV_BUFFER_SIZE  (DISPLAY_WIDTH * 25)
#define LVGL_TICK_MS    5

#define TERMINAL_QUEUE_LEN 10
#define TERMINAL_MSG_LEN 128
#define TERMINAL_VISIBLE_LINES 5
#define TERMINAL_QUEUE_LEN 10
#define TERMINAL_MSG_LEN 128
#define TERMINAL_HISTORY_SIZE 1500

static char terminal_history_buffer[TERMINAL_HISTORY_SIZE] = ">>> TERMINAL READY <<<\n";
static bool history_has_new_data = false;

static QueueHandle_t terminal_log_queue = NULL;

static SemaphoreHandle_t lvgl_mutex = NULL;
static lv_disp_draw_buf_t draw_buf;
static lv_disp_drv_t disp_drv;
static lv_color_t *buf1 = NULL;
static lv_color_t *buf2 = NULL;

static lv_obj_t * dd_work = NULL;
static lv_obj_t * dd_tool = NULL;
static lv_obj_t * dd_state = NULL;
static lv_obj_t *debug_container = NULL;

static lv_obj_t * state_labels[13] = {NULL};
static lv_obj_t * mode_toggle_btn = NULL;
static lv_obj_t * dd_speed = NULL;
static float last_vals[13] = {0};
static lv_obj_t * coord_container = NULL;  // Coord container color

// feedback from ROS to UI
static lv_obj_t * work_value_label = NULL;
static lv_obj_t * tool_value_label = NULL;
static lv_obj_t * state_value_label = NULL;
static lv_obj_t * mode_value_label = NULL;
static lv_obj_t * speed_value_label = NULL;

// Forward declarations for event callbacks
static void dropdown_event_cb(lv_event_t * e);
static void toggle_button_event_cb(lv_event_t * e);
static ui_command_callback_t command_callback = NULL;
static bool robot_in_motion = false;

static lv_obj_t * tool_uid_value = NULL;
static lv_obj_t * tool_type_value = NULL;
static lv_obj_t * tool_status_value = NULL;

// Dropdown event handler
static void dropdown_event_cb(lv_event_t * e) {
    lv_obj_t * dropdown = lv_event_get_target(e);
    int index = lv_dropdown_get_selected(dropdown);
    
    // Block state changes while robot is moving
    if (dropdown == dd_state && robot_in_motion) {
        ESP_LOGW(TAG, "Cannot change state while robot is moving!");
        lv_dropdown_set_selected(dd_state, 1);  // Force back to IDLE
        return;
    }
    
    if (command_callback) {
        if (dropdown == dd_work) {
            command_callback(1, index, 0);
        } else if (dropdown == dd_tool) {
            command_callback(2, index, 0);
        } else if (dropdown == dd_state) {
            command_callback(3, index, 0);
        }
    }
}

// Toggle button event handler
static void toggle_button_event_cb(lv_event_t * e) {
    if (robot_in_motion) {
        ESP_LOGW(TAG, "Cannot change mode while robot is moving!");
        return;
    }
    
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;
    
    lv_obj_t * btn = lv_event_get_target(e);
    bool is_toggled = lv_obj_has_state(btn, LV_STATE_CHECKED);
    
    lv_obj_t *btn_label = lv_obj_get_child(btn, 0);
    lv_label_set_text(btn_label, is_toggled ? "CART" : "JOINT");
    
    if (command_callback) {
        command_callback(5, is_toggled ? 1 : 0, 0);
    }
}

// Speed button matrix event handler
static void speed_dropdown_event_cb(lv_event_t * e)
{
    if (robot_in_motion) {
        ESP_LOGW(TAG, "Cannot change speed while robot is moving!");
        return;
    }
    
    lv_obj_t * dropdown = lv_event_get_target(e);
    int index = lv_dropdown_get_selected(dropdown);
    
    float speed_percent = index * 25.0f;
    ESP_LOGI(TAG, "Speed changed to %.0f%%", speed_percent);
    
    if (command_callback) {
        command_callback(6, 0, speed_percent / 100.0f);
    }
}

// Tool button callbacks (regular C functions)
static void attach_btn_cb(lv_event_t * e) {
    if (command_callback) {
        command_callback(8, 3, 0);  // action_trigger = 3 (attach)
        ESP_LOGI(TAG, "ATTACH button pressed - trigger=3");
    }
}

static void detach_btn_cb(lv_event_t * e) {
    if (command_callback) {
        command_callback(8, 4, 0);  // action_trigger = 4 (detach)
        ESP_LOGI(TAG, "DETACH button pressed - trigger=4");
    }
}

void ui_update_state_values(float *values, int count)
{
    if (count > 13) count = 13;
    
    // Define format for each value
    const char *formats[] = {
        "%1.3f",  // X (meters) - 0.000
        "%1.3f",  // Y (meters) - 0.000
        "%1.3f",  // Z (meters) - 0.000
        "%1.3f",  // R (radians) - 0.00
        "%1.3f",  // P (radians) - 0.00
        "%1.3f",  // Y (radians) - 0.00
        "%.1f",  // J1 (degrees) - 000.0
        "%.1f",  // J2 (degrees) - 000.0
        "%.1f",  // J3 (degrees) - 000.0
        "%.1f",  // J4 (degrees) - 000.0
        "%.1f",  // J5 (degrees) - 000.0
        "%.1f",  // J6 (degrees) - 000.0
        "%.3f"   // Gripper (meters) - 0.000
    };
    
    for (int i = 0; i < count; i++) {
        if (state_labels[i] && fabs(values[i] - last_vals[i]) > 0.001f) {
            char buf[32];
            snprintf(buf, sizeof(buf), formats[i], values[i]);
            lv_label_set_text(state_labels[i], buf);
            last_vals[i] = values[i];
        }
    }
}

void ui_set_work_offset(int index)
{
    if (dd_work) {
        lv_dropdown_set_selected(dd_work, index);
    }
}

void ui_set_tool_orientation(int index)
{
    if (dd_tool) {
        lv_dropdown_set_selected(dd_tool, index);
    }
}

void ui_set_robot_state(int index)
{
    if (dd_state) {
        lv_dropdown_set_selected(dd_state, index);
    }
}

void ui_set_control_mode(bool is_cartesian)
{
    if (mode_toggle_btn) {
        if (is_cartesian) {
            lv_obj_add_state(mode_toggle_btn, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(mode_toggle_btn, LV_STATE_CHECKED);
        }
    }
}

void ui_set_speed(float percent)
{
    if (dd_speed) {
        int btn_id = (int)(percent / 25.0f);
        if (btn_id < 0) btn_id = 0;
        if (btn_id > 4) btn_id = 4;
        lv_dropdown_set_selected(dd_speed, btn_id);
    }
}

void ui_create_robot_control(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clean(scr);
    
    // Grid container - Now with more rows for tool info
    static lv_coord_t col_dsc[] = {105, 110, LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_dsc[] = {20, 20, 20, 20, 20, 20, 20, 15, 15, 15, LV_GRID_TEMPLATE_LAST};  // 10 rows

    // Grid container
    lv_obj_t * cont = lv_obj_create(scr);
    coord_container = cont; // Container color
    lv_obj_set_grid_dsc_array(cont, col_dsc, row_dsc);
    lv_obj_set_size(cont, 262, 245);  // Increased height to 230
    lv_obj_align(cont, LV_ALIGN_TOP_LEFT, 5, 5);
    lv_obj_set_style_pad_all(cont, 8, 0);
    lv_obj_set_style_pad_row(cont, 4, 0);
    lv_obj_set_style_pad_column(cont, 20, 0);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

    // State labels with names (13 items)
    const char * names[] = {
        "X(m): ", "Y(m): ", "Z(m): ", "R(rad): ", "P(rad): ", "Y(rad): ", 
        "J1(°): ", "J2(°): ", "J3(°): ", "J4(°): ", "J5(°): ", "J6(°): ", "T(units): "
    };

    for(int i = 0; i < 13; i++) {
        int col = (i < 6) ? 0 : 1;
        int row = (i < 6) ? i : (i - 6);
        
        // Create name label
        lv_obj_t * name_label = lv_label_create(cont);
        lv_label_set_text(name_label, names[i]);
        lv_obj_set_style_text_font(name_label, &lv_font_montserrat_14, 0);
        lv_obj_set_grid_cell(name_label, 
                            LV_GRID_ALIGN_START, col, 1, 
                            LV_GRID_ALIGN_CENTER, row, 1);
        lv_obj_set_style_text_color(name_label, lv_color_black(), 0);
        
        // Create value label
        state_labels[i] = lv_label_create(cont);
        lv_obj_set_style_text_font(state_labels[i], &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(state_labels[i], lv_color_black(), 0);
        lv_obj_set_grid_cell(state_labels[i], 
                            LV_GRID_ALIGN_END, col, 1,
                            LV_GRID_ALIGN_CENTER, row, 1);
        lv_label_set_text(state_labels[i], "0.00");
        last_vals[i] = -9999.0f;
    }

    // ========== TOOL INFORMATION DISPLAY (Rows 7, 8, 9) ==========

    // Row 7: Tool UID
    lv_obj_t * tool_uid_name = lv_label_create(cont);
    lv_label_set_text(tool_uid_name, "Tool UID:");
    lv_obj_set_style_text_font(tool_uid_name, &lv_font_montserrat_12, 0);
    lv_obj_set_grid_cell(tool_uid_name, LV_GRID_ALIGN_START, 0, 1, LV_GRID_ALIGN_CENTER, 7, 1);
    lv_obj_set_style_text_color(tool_uid_name, lv_color_black(), 0);

    tool_uid_value = lv_label_create(cont);
    lv_obj_set_style_text_font(tool_uid_value, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(tool_uid_value, lv_color_black(), 0);
    lv_obj_set_grid_cell(tool_uid_value, LV_GRID_ALIGN_END, 0, 2, LV_GRID_ALIGN_CENTER, 7, 1);
    lv_label_set_text(tool_uid_value, "----");

    // Row 8: Tool Type
    lv_obj_t * tool_type_name = lv_label_create(cont);
    lv_label_set_text(tool_type_name, "Tool Type:");
    lv_obj_set_style_text_font(tool_type_name, &lv_font_montserrat_12, 0);
    lv_obj_set_grid_cell(tool_type_name, LV_GRID_ALIGN_START, 0, 1, LV_GRID_ALIGN_CENTER, 8, 1);
    lv_obj_set_style_text_color(tool_type_name, lv_color_black(), 0);

    tool_type_value = lv_label_create(cont);
    lv_obj_set_style_text_font(tool_type_value, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(tool_type_value, lv_color_black(), 0);
    lv_obj_set_grid_cell(tool_type_value, LV_GRID_ALIGN_END, 0, 2, LV_GRID_ALIGN_CENTER, 8, 1);
    lv_label_set_text(tool_type_value, "NONE");

    // Row 9: Tool Status
    lv_obj_t * tool_status_name = lv_label_create(cont);
    lv_label_set_text(tool_status_name, "Tool Status:");
    lv_obj_set_style_text_font(tool_status_name, &lv_font_montserrat_12, 0);
    lv_obj_set_grid_cell(tool_status_name, LV_GRID_ALIGN_START, 0, 1, LV_GRID_ALIGN_CENTER, 9, 1);
    lv_obj_set_style_text_color(tool_status_name, lv_color_black(), 0);

    tool_status_value = lv_label_create(cont);
    lv_obj_set_style_text_font(tool_status_value, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(tool_status_value, lv_color_black(), 0);
    lv_obj_set_grid_cell(tool_status_value, LV_GRID_ALIGN_END, 0, 2, LV_GRID_ALIGN_CENTER, 9, 1);
    lv_label_set_text(tool_status_value, "IDLE");
    
    // 1. WORK OFFSETS
    dd_work = lv_dropdown_create(scr);
    lv_dropdown_set_options(dd_work, "BASE\nUSER1\nUSER2\nUSER3\nUSER4\nUSER5");
    lv_obj_set_width(dd_work, 100);
    lv_obj_set_style_text_font(dd_work, &lv_font_montserrat_14, 0);
    lv_obj_align(dd_work, LV_ALIGN_TOP_RIGHT, -30, 25);
    lv_obj_add_event_cb(dd_work, dropdown_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    work_value_label = lv_label_create(scr);
    lv_label_set_text(work_value_label, "[0]");
    lv_obj_set_style_text_font(work_value_label, &lv_font_montserrat_10, 0);
    lv_obj_align_to(work_value_label, dd_work, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
    
    lv_obj_t * lbl_work = lv_label_create(scr);
    lv_label_set_text(lbl_work, "Work Offset:  (fb)");
    lv_obj_set_style_text_font(lbl_work, &lv_font_montserrat_14, 0);
    lv_obj_align_to(lbl_work, dd_work, LV_ALIGN_OUT_TOP_RIGHT, 25, -5);
    
    // 2. TOOL ORIENTATION
    dd_tool = lv_dropdown_create(scr);
    lv_dropdown_set_options(dd_tool, "FLANGE\nTOOL1\nTOOL2\nTOOL3\nTOOL4\nTOOL5\n");
    lv_obj_set_width(dd_tool, 100);
    lv_obj_set_style_text_font(dd_tool, &lv_font_montserrat_14, 0);
    lv_obj_align_to(dd_tool, dd_work, LV_ALIGN_OUT_BOTTOM_MID, 0, 25);
    lv_obj_add_event_cb(dd_tool, dropdown_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    tool_value_label = lv_label_create(scr);
    lv_label_set_text(tool_value_label, "[0]");
    lv_obj_set_style_text_font(tool_value_label, &lv_font_montserrat_10, 0);
    lv_obj_align_to(tool_value_label, dd_tool, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
    
    lv_obj_t * lbl_tool = lv_label_create(scr);
    lv_label_set_text(lbl_tool, "Tool Orientation:");
    lv_obj_set_style_text_font(lbl_tool, &lv_font_montserrat_14, 0);
    lv_obj_align_to(lbl_tool, dd_tool, LV_ALIGN_OUT_TOP_RIGHT, 23, -5);
    
    // 2.1 tool buttons (attach/detach) - for future use, currently just display
    
    // Create vertical container for buttons (one under another)
    lv_obj_t * tool_btn_cont = lv_obj_create(scr);
    lv_obj_set_size(tool_btn_cont, 75, 75);  // Width 80, Height 75 for two buttons
    lv_obj_align_to(tool_btn_cont, dd_tool, LV_ALIGN_OUT_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_color(tool_btn_cont, lv_color_white(), 0);
    lv_obj_set_style_border_width(tool_btn_cont, 0, 0);
    lv_obj_set_flex_flow(tool_btn_cont, LV_FLEX_FLOW_COLUMN);  // Column layout
    lv_obj_set_flex_align(tool_btn_cont, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(tool_btn_cont, 2, 0);

    // Attach button (top)
    lv_obj_t * attach_btn = lv_btn_create(tool_btn_cont);
    lv_obj_set_size(attach_btn, 60, 30);
    lv_obj_set_style_bg_color(attach_btn, lv_color_make(0, 150, 0), 0);  // Green
    lv_obj_t * attach_label = lv_label_create(attach_btn);
    lv_label_set_text(attach_label, "ATTACH");
    lv_obj_set_style_text_font(attach_label, &lv_font_montserrat_12, 0);
    lv_obj_center(attach_label);
    lv_obj_add_event_cb(attach_btn, attach_btn_cb, LV_EVENT_CLICKED, NULL);  // Use named function

    // Detach button (bottom)
    lv_obj_t * detach_btn = lv_btn_create(tool_btn_cont);
    lv_obj_set_size(detach_btn, 60, 30);
    lv_obj_set_style_bg_color(detach_btn, lv_color_make(200, 0, 0), 0);  // Red
    lv_obj_t * detach_label = lv_label_create(detach_btn);
    lv_label_set_text(detach_label, "DETACH");
    lv_obj_set_style_text_font(detach_label, &lv_font_montserrat_12, 0);
    lv_obj_center(detach_label);
    lv_obj_add_event_cb(detach_btn, detach_btn_cb, LV_EVENT_CLICKED, NULL);  // Use named function
    
    // 3. ROBOT STATE
    dd_state = lv_dropdown_create(scr);
    lv_dropdown_set_options(dd_state, "ESTOP\nIDLE\nTEACH\nJOG\nAUTO");
    lv_dropdown_set_selected(dd_state, 3);
    lv_obj_set_width(dd_state, 100);
    lv_obj_set_style_text_font(dd_state, &lv_font_montserrat_14, 0);
    lv_obj_align_to(dd_state, dd_tool, LV_ALIGN_OUT_BOTTOM_MID, 0, 25);
    lv_obj_add_event_cb(dd_state, dropdown_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    state_value_label = lv_label_create(scr);
    lv_label_set_text(state_value_label, "[J]");
    lv_obj_set_style_text_font(state_value_label, &lv_font_montserrat_10, 0);
    lv_obj_align_to(state_value_label, dd_state, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
    
    lv_obj_t * lbl_state = lv_label_create(scr);
    lv_label_set_text(lbl_state, "Robot State:");
    lv_obj_set_style_text_font(lbl_state, &lv_font_montserrat_14, 0);
    lv_obj_align_to(lbl_state, dd_state, LV_ALIGN_OUT_TOP_RIGHT, -10, -5);
    
    // 4. MODE TOGGLE BUTTON
    mode_toggle_btn = lv_btn_create(scr);
    lv_obj_set_size(mode_toggle_btn, 100, 30);
    lv_obj_align_to(mode_toggle_btn, dd_state, LV_ALIGN_OUT_BOTTOM_MID, 0, 25);
    lv_obj_add_flag(mode_toggle_btn, LV_OBJ_FLAG_CHECKABLE);
    lv_obj_add_event_cb(mode_toggle_btn, toggle_button_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    mode_value_label = lv_label_create(scr);
    lv_label_set_text(mode_value_label, "[J]");
    lv_obj_set_style_text_font(mode_value_label, &lv_font_montserrat_10, 0);
    lv_obj_align_to(mode_value_label, mode_toggle_btn, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
    
    lv_obj_set_style_bg_color(mode_toggle_btn, lv_palette_main(LV_PALETTE_BLUE), LV_STATE_DEFAULT);
    lv_obj_set_style_border_color(mode_toggle_btn, lv_palette_main(LV_PALETTE_BLUE), LV_STATE_DEFAULT);
    lv_obj_set_style_radius(mode_toggle_btn, 8, 0);
    lv_obj_set_style_bg_color(mode_toggle_btn, lv_palette_main(LV_PALETTE_GREEN), LV_STATE_CHECKED);
    
    lv_obj_t * btn_label = lv_label_create(mode_toggle_btn);
    lv_label_set_text(btn_label, "JOINT");
    lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_14, 0);
    lv_obj_center(btn_label);
    
    lv_obj_t * lbl_mode = lv_label_create(scr);
    lv_label_set_text(lbl_mode, "Control Mode:");
    lv_obj_set_style_text_font(lbl_mode, &lv_font_montserrat_14, 0);
    lv_obj_align_to(lbl_mode, mode_toggle_btn, LV_ALIGN_OUT_TOP_RIGHT, 3, -5);
    
    // 5. SPEED DROPDOWN (under Control Mode)
    dd_speed = lv_dropdown_create(scr);
    lv_dropdown_set_options(dd_speed, "0%\n25%\n50%\n75%\n100%");
    lv_dropdown_set_selected(dd_speed, 2);  // 50% default
    lv_obj_set_width(dd_speed, 80);
    lv_obj_set_style_text_font(dd_speed, &lv_font_montserrat_14, 0);
    lv_obj_align_to(dd_speed, mode_toggle_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 25);
    lv_obj_add_event_cb(dd_speed, speed_dropdown_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Label for dropdown
    lv_obj_t * lbl_speed = lv_label_create(scr);
    lv_label_set_text(lbl_speed, "Speed Override:");
    lv_obj_set_style_text_font(lbl_speed, &lv_font_montserrat_14, 0);
    lv_obj_align_to(lbl_speed, dd_speed, LV_ALIGN_OUT_TOP_RIGHT, 30, -5);

    // Value display next to dropdown
    speed_value_label = lv_label_create(scr);
    lv_label_set_text(speed_value_label, "[50]");
    lv_obj_set_style_text_font(speed_value_label, &lv_font_montserrat_8, 0);
    lv_obj_align_to(speed_value_label, dd_speed, LV_ALIGN_OUT_RIGHT_MID, 5, 0);
    
    // 6. DEBUG TERMINAL - Текстовое поле с поддержкой скроллинга и переноса строк
    debug_container = lv_textarea_create(scr);
    lv_obj_set_size(debug_container, 335, 65);
    lv_obj_align(debug_container, LV_ALIGN_BOTTOM_LEFT, 5, -5);
    
    // Настройки взаимодействия для тачскрина (Разрешаем скролл пальцем)
    lv_obj_add_flag(debug_container, LV_OBJ_FLAG_CLICKABLE);       // Чтобы работал скролл рукой
    lv_obj_clear_flag(debug_container, LV_OBJ_FLAG_CLICK_FOCUSABLE); // Запрет фокуса (курсор не появится)
    lv_obj_add_flag(debug_container, LV_OBJ_FLAG_SCROLLABLE);       // Разрешаем физический скролл
    
    // Полоса прокрутки будет красиво появляться только при прокрутке пальцем
    lv_obj_set_scrollbar_mode(debug_container, LV_SCROLLBAR_MODE_AUTO); 
    
    lv_obj_set_style_text_font(debug_container, &lv_font_montserrat_12, 0);
    lv_obj_set_style_bg_color(debug_container, lv_color_white(), 0);
    lv_obj_set_style_text_color(debug_container, lv_color_black(), 0);
    lv_obj_set_style_radius(debug_container, 5, 0);
    
    // Инициализируем стартовый текст из буфера памяти
    lv_textarea_set_text(debug_container, terminal_history_buffer);


    ESP_LOGI(TAG, "Debug text area initialized with scrolling");
    
    ESP_LOGI(TAG, "Robot control UI created");
}

static void lvgl_flush_cb(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map)
{
    esp_lcd_panel_handle_t panel = display_get_panel_handle();
    esp_lcd_panel_draw_bitmap(panel, area->x1, area->y1, area->x2 + 1, area->y2 + 1, color_map);
    lv_disp_flush_ready(drv);
}

static void IRAM_ATTR lvgl_tick_cb(void *param)
{
    lv_tick_inc(LVGL_TICK_MS);
}

void ui_lock(void) {
    if (lvgl_mutex) {
        xSemaphoreTake(lvgl_mutex, portMAX_DELAY);
    }
}

void ui_unlock(void) {
    if (lvgl_mutex) {
        xSemaphoreGive(lvgl_mutex);
    }
}

esp_err_t ui_init(void)
{
    // Создаем очередь для терминала
    terminal_log_queue = xQueueCreate(TERMINAL_QUEUE_LEN, TERMINAL_MSG_LEN);
    if (terminal_log_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create terminal log queue");
        return ESP_FAIL;
    }

    // Create LVGL mutex
    lvgl_mutex = xSemaphoreCreateMutex();
    if (lvgl_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create LVGL mutex");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Initializing LVGL");
    lv_init();
    
    // Allocate buffers
    ESP_LOGI(TAG, "Allocating LVGL buffer: %zu bytes", LV_BUFFER_SIZE * sizeof(lv_color_t));
    buf1 = (lv_color_t *)heap_caps_malloc(LV_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    
#ifdef USE_DOUBLE_BUFFERING
    ESP_LOGI(TAG, "Allocating second LVGL buffer");
    buf2 = (lv_color_t *)heap_caps_malloc(LV_BUFFER_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
#endif
    
    lv_disp_draw_buf_init(&draw_buf, buf1, buf2, LV_BUFFER_SIZE);
    
    // Initialize display driver
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = DISPLAY_WIDTH;
    disp_drv.ver_res = DISPLAY_HEIGHT;
    disp_drv.flush_cb = lvgl_flush_cb;
    disp_drv.draw_buf = &draw_buf;
    disp_drv.user_data = display_get_panel_handle();
    disp_drv.sw_rotate = 1;
    disp_drv.rotated = LV_DISP_ROT_90; 
    lv_disp_t *disp = lv_disp_drv_register(&disp_drv);
    
    // Initialize touch (if available)
    esp_err_t ret = touch_init();
    if (ret == ESP_OK) {
        static lv_indev_drv_t indev_drv;
        lv_indev_drv_init(&indev_drv);
        indev_drv.type = LV_INDEV_TYPE_POINTER;
        indev_drv.disp = disp;
        indev_drv.read_cb = touch_driver_read;
        // Note: touch_driver_read will get the tp handle from its own static variable
        lv_indev_drv_register(&indev_drv);
        ESP_LOGI(TAG, "Touch registered with LVGL");
    } else {
        ESP_LOGW(TAG, "Touch not available, continuing without touch");
    }
    
    // Create LVGL tick timer
    const esp_timer_create_args_t timer_args = {
        .callback = &lvgl_tick_cb,
        .name = "lvgl_tick"
    };
    esp_timer_handle_t tick_timer;
    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, LVGL_TICK_MS * 1000));
    
    ESP_LOGI(TAG, "UI initialized");
    return ESP_OK;
}

void ui_task_handler(void)
{
    ui_lock();

    char incoming_msg[TERMINAL_MSG_LEN];

    // 1. Выгребаем логи из очереди FreeRTOS прямо в буфер RAM, не затрагивая виджеты
    if (terminal_log_queue) {
        while (xQueueReceive(terminal_log_queue, incoming_msg, 0) == pdTRUE) {
            
            // Проверяем, не переполнится ли наш буфер истории логов (оставляем запас)
            if (strlen(terminal_history_buffer) + strlen(incoming_msg) + 2 >= TERMINAL_HISTORY_SIZE) {
                
                // Находим позицию смещения (удаляем старую половину логов для освобождения места)
                size_t offset = strlen(terminal_history_buffer) - 512;
                const char *clean_start = strchr(&terminal_history_buffer[offset], '\n');
                
                if (clean_start) {
                    size_t tail_len = strlen(clean_start + 1);
                    const char *header = ">>> LOGS FLUSHED <<<\n";
                    size_t header_len = strlen(header);
                    
                    // Безопасно формируем новый кусок истории прямо в RAM
                    memcpy(terminal_history_buffer, header, header_len);
                    memmove(&terminal_history_buffer[header_len], clean_start + 1, tail_len + 1);
                } else {
                    strcpy(terminal_history_buffer, ">>> LOGS FLUSHED <<<\n");
                }
            }

            // Дописываем новое сообщение (сохраняя префиксы [ERROR], [INFO] и длинный текст)
            strcat(terminal_history_buffer, incoming_msg);
            strcat(terminal_history_buffer, "\n");
            
            history_has_new_data = true;
        }
    }

    // 2. Безопасное атомарное обновление интерфейса
    if (debug_container && lv_obj_is_valid(debug_container)) {
        
        // Если пользователь СЕЙЧАС касается экрана или листает логи вверх пальцем:
        if (lv_obj_has_state(debug_container, LV_STATE_PRESSED)) {
            // МЫ НЕ ТРОГАЕМ ВИДЖЕТ. Логи спокойно копятся в terminal_history_buffer в фоне.
            // Это дает вам 100% плавный скроллинг без фризов интерфейса!
        } 
        // Если экран отпущен и у нас накопились свежие логи за время касания:
        else if (history_has_new_data) {
            
            // Атомарно обновляем весь текст одной операцией. Занимает < 1 мс.
            lv_textarea_set_text(debug_container, terminal_history_buffer);
            
            // Сдвигаем невидимый курсор в самый конец текста
            lv_textarea_set_cursor_pos(debug_container, LV_TEXTAREA_CURSOR_LAST);
            // Заставляем текстовое поле сфокусировать область видимости на конце логов
            lv_obj_scroll_to_view(debug_container, LV_ANIM_OFF);
            
            history_has_new_data = false;
        }
    }

    lv_timer_handler();
    ui_unlock();
    vTaskDelay(pdMS_TO_TICKS(10));
}


void ui_clear_screen(void)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_clean(scr);
}


// ========== ROS интеграция (минимально) ==========

// Update container color based on robot state
void ui_update_container_color(uint8_t state)
{
    //ESP_LOGI(TAG, "Updating container color to state: %d", state);
    
    if (coord_container == NULL) {
        //ESP_LOGW(TAG, "coord_container is NULL!");
        return;
    }
    
    switch(state) {
        case 0: // ESTOP
            lv_obj_set_style_bg_color(coord_container, lv_color_hex(0xFF0000), 0);
            break;
        case 1: // IDLE
            lv_obj_set_style_bg_color(coord_container, lv_color_hex(0xFFFF00), 0);
            break;
        case 2: // TEACH
            lv_obj_set_style_bg_color(coord_container, lv_color_hex(0x0000FF), 0);
            break;
        case 3: // JOG
            lv_obj_set_style_bg_color(coord_container, lv_color_hex(0xFFFFFF), 0);
            break;
        case 4: // AUTO
            lv_obj_set_style_bg_color(coord_container, lv_color_hex(0x00FF00), 0);
            break;
        default:
            lv_obj_set_style_bg_color(coord_container, lv_color_hex(0x1a1a2e), 0);
            break;
    }
}

void ui_add_debug_line(const char *text) {
    if (!text || !terminal_log_queue) return;

    char msg_buf[TERMINAL_MSG_LEN];
    // Сохраняем строку со всеми префиксами ([ERROR], [INFO]...) как есть
    snprintf(msg_buf, sizeof(msg_buf), "%s", text);

    // Отправляем в очередь. xQueueSend — атомарная и ультра-быстрая операция.
    // timeout = 0 означает, что если очередь переполнена, лог просто дропнется,
    // но робот и сетевой стек НИКОГДА не зависнут.
    xQueueSend(terminal_log_queue, msg_buf, 0);
}


void ui_set_command_callback(ui_command_callback_t cb)
{
    command_callback = cb;
}

// Получение состояния
uint8_t ui_get_work_offset(void) {
    return dd_work ? lv_dropdown_get_selected(dd_work) : 0;
}

uint8_t ui_get_tool_id(void) {
    return dd_tool ? lv_dropdown_get_selected(dd_tool) : 0;
}

uint8_t ui_get_robot_state(void) {
    return dd_state ? lv_dropdown_get_selected(dd_state) : 0;
}

bool ui_get_cartesian_mode(void) {
    return mode_toggle_btn ? lv_obj_has_state(mode_toggle_btn, LV_STATE_CHECKED) : false;
}

float ui_get_speed_override(void)
{
    if (dd_speed) {
        int selected = lv_dropdown_get_selected(dd_speed);
        return selected * 0.25f;  // 0, 0.25, 0.5, 0.75, 1.0
    }
    return 0.5f;
}

void ui_update_state_display(uint8_t state)
{
    if (state_value_label) {
        const char *state_names[] = {"E", "I", "T", "J", "A"};
        const char *state_name = (state < 5) ? state_names[state] : "UNKNOWN";
        char buf[32];
        snprintf(buf, sizeof(buf), "[%s]", state_name);
        lv_label_set_text(state_value_label, buf);
    }
}

void ui_update_work_display(uint8_t id)
{
    if (work_value_label) {
        char buf[16];
        snprintf(buf, sizeof(buf), "[%d]", id);
        lv_label_set_text(work_value_label, buf);
    }
}

void ui_update_tool_display(uint8_t id)
{
    if (tool_value_label) {
        char buf[16];
        snprintf(buf, sizeof(buf), "[%d]", id);
        lv_label_set_text(tool_value_label, buf);
    }
}

void ui_update_mode_display(bool cartesian)
{
    if (mode_value_label) {
        lv_label_set_text(mode_value_label, cartesian ? "[C]" : "[J]");
    }
}

void ui_update_speed_display(float speed)
{
    if (speed_value_label) {
        char buf[16];
        snprintf(buf, sizeof(buf), "[%.0f]", speed * 100);
        lv_label_set_text(speed_value_label, buf);
    }
}

// Tool type to string conversion
static const char* tool_type_to_string(uint8_t type) {
    const char* types[] = {"NONE", "GRIPPER", "DISPENSER", "VACUUM", "UART"};
    if (type <= 4) return types[type];
    return "UNKNOWN";
}

// Tool status to string conversion
static const char* tool_status_to_string(uint8_t status) {
    const char* statuses[] = {"IDLE", "ATTACHING", "ATTACHED", "RELEASING", "ERROR"};
    if (status <= 4) return statuses[status];
    return "UNKNOWN";
}

// Обновление из ROS
void ui_update_telemetry(float *values, int count, uint8_t state, 
                        uint8_t tool_id, uint8_t work_offset, 
                        bool cartesian, float speed,
                        uint16_t tool_uid, uint8_t tool_type, uint8_t tool_status)
{
    ui_update_state_values(values, count);
    
    if (dd_state) lv_dropdown_set_selected(dd_state, state);
    if (dd_tool) lv_dropdown_set_selected(dd_tool, tool_id);
    if (dd_work) lv_dropdown_set_selected(dd_work, work_offset);
    
    if (mode_toggle_btn) {
        if (cartesian) {
            lv_obj_add_state(mode_toggle_btn, LV_STATE_CHECKED);
        } else {
            lv_obj_clear_state(mode_toggle_btn, LV_STATE_CHECKED);
        }
    }
    
    if (dd_speed) {
        int btn = (int)(speed * 4);
        if (btn < 0) btn = 0;
        if (btn > 4) btn = 4;
        lv_dropdown_set_selected(dd_speed, btn);
    }
    
    // Update tool info display
    if (tool_uid_value) {
        char buf[16];
        if (tool_uid == 0) {
            snprintf(buf, sizeof(buf), "NONE");
        } else {
            snprintf(buf, sizeof(buf), "%05u", tool_uid);
        }
        lv_label_set_text(tool_uid_value, buf);
    }
    
    if (tool_type_value) {
        lv_label_set_text(tool_type_value, tool_type_to_string(tool_type));
    }
    
    if (tool_status_value) {
    // Status text color - always black for readability on colored backgrounds
    lv_obj_set_style_text_color(tool_status_value, lv_color_make(0, 0, 0), 0);
    lv_label_set_text(tool_status_value, tool_status_to_string(tool_status));
    }
}

void ui_set_motion_state(bool in_motion) {
    robot_in_motion = in_motion;
    if (in_motion) {
        //ESP_LOGI(TAG, "Robot in motion - UI controls locked");
        // Force state dropdown to IDLE during motion
        //if (dd_state) {
            lv_dropdown_set_selected(dd_state, 1);  // IDLE
       // }
    } else {
       // ESP_LOGI(TAG, "Robot stopped - UI controls unlocked");
    }
}