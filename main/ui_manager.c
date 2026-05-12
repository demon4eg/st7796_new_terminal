#include "ui_manager.h"
#include "display_config.h"
#include "touch_input.h"
#include <esp_log.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <lvgl.h>
#include <math.h>

static const char *TAG = "ui_manager";

// Display dimensions
#define DISPLAY_WIDTH   320
#define DISPLAY_HEIGHT  480
#define LV_BUFFER_SIZE  (DISPLAY_WIDTH * 25)
#define LVGL_TICK_MS    5

static lv_disp_draw_buf_t draw_buf;
static lv_disp_drv_t disp_drv;
static lv_color_t *buf1 = NULL;
static lv_color_t *buf2 = NULL;

static lv_obj_t * dd_work = NULL;
static lv_obj_t * dd_tool = NULL;
static lv_obj_t * dd_state = NULL;
static lv_obj_t * terminal = NULL;
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
static void speed_btnmatrix_event_cb(lv_event_t * e);
static ui_command_callback_t command_callback = NULL;


// Dropdown event handler
static void dropdown_event_cb(lv_event_t * e) {
    lv_obj_t * dropdown = lv_event_get_target(e);
    int index = lv_dropdown_get_selected(dropdown);
    
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
    lv_obj_t * dropdown = lv_event_get_target(e);
    int index = lv_dropdown_get_selected(dropdown);
    
    float speed_percent = index * 25.0f;
    ESP_LOGI(TAG, "Speed changed to %.0f%%", speed_percent);
    
    if (command_callback) {
        command_callback(6, 0, speed_percent / 100.0f);
    }
}

// Public functions for updating UI from micro-ros
void ui_set_terminal_text(const char *text)
{
    if (terminal) {
        lv_textarea_add_text(terminal, text);
        lv_textarea_set_cursor_pos(terminal, LV_TEXTAREA_CURSOR_LAST);
    }
}

void ui_update_state_values(float *values, int count)
{
    if (count > 13) count = 13;
    
    for (int i = 0; i < count; i++) {
        if (state_labels[i] && fabs(values[i] - last_vals[i]) > 0.01f) {
            char buf[32];
            snprintf(buf, sizeof(buf), "%.2f", values[i]);
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
    
    // Column and row definitions
    static lv_coord_t col_dsc[] = {110, 110, LV_GRID_TEMPLATE_LAST};
    static lv_coord_t row_dsc[] = {20, 20, 20, 20, 20, 20, 20, LV_GRID_TEMPLATE_LAST};
    
    // Grid container
    lv_obj_t * cont = lv_obj_create(scr);
    coord_container = cont; // Container color
    lv_obj_set_grid_dsc_array(cont, col_dsc, row_dsc);
    lv_obj_set_size(cont, 260, 180);
    lv_obj_align(cont, LV_ALIGN_TOP_LEFT, 5, 5);
    lv_obj_set_style_pad_all(cont, 7, 0);
    lv_obj_set_style_pad_row(cont, 4, 0);
    lv_obj_set_style_pad_column(cont, 20, 0);
    lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);
    
    // State labels with names
    const char * names[] = {
        "X(m): ", "Y(m): ", "Z(m): ", "R(rad): ", "P(rad): ", "Y(rad): ", 
        "J1(°): ", "J2(°): ", "J3(°): ", "J4(°): ", "J5(°): ", "J6(°): ", "T(units): "
    };

    for(int i = 0; i < 13; i++) {
        int col = (i < 6) ? 0 : 1;
        int row = (i < 6) ? i : (i - 6);
        
        // Создаем лейбл с именем
        lv_obj_t * name_label = lv_label_create(cont);
        lv_label_set_text(name_label, names[i]);
        lv_obj_set_style_text_font(name_label, &lv_font_montserrat_14, 0);
        lv_obj_set_grid_cell(name_label, 
                            LV_GRID_ALIGN_START, col, 1, 
                            LV_GRID_ALIGN_CENTER, row, 1);
        lv_obj_set_style_text_color(name_label, lv_color_black(), 0);
        
        // Создаем лейбл со значением (сдвигаем его правее)
        state_labels[i] = lv_label_create(cont);
        lv_obj_set_style_text_font(state_labels[i], &lv_font_montserrat_16, 0);
        lv_obj_set_style_text_color(name_label, lv_color_black(), 0);
        lv_obj_set_grid_cell(state_labels[i], 
                            LV_GRID_ALIGN_END, col, 1,  // ALIGN_END для правой стороны
                            LV_GRID_ALIGN_CENTER, row, 1);
        lv_label_set_text(state_labels[i], "0.00");
        last_vals[i] = -9999.0f;
    }
    
    

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
    
    // 6. TERMINAL
    terminal = lv_textarea_create(scr);
    lv_obj_set_size(terminal, 325, 60);
    lv_obj_align(terminal, LV_ALIGN_BOTTOM_LEFT, 10, -5);
    lv_textarea_set_text(terminal, "ROS2 terminal ready...\n");
    lv_obj_set_style_text_font(terminal, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(terminal, lv_color_black(), 0);
    lv_obj_set_style_bg_color(terminal, lv_color_white(), 0);
    
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

esp_err_t ui_init(void)
{
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
    lv_timer_handler();
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
            lv_obj_set_style_bg_color(coord_container, lv_color_hex(0xFFA500), 0);
            break;
        case 3: // JOG
            lv_obj_set_style_bg_color(coord_container, lv_color_hex(0x0000FF), 0);
            break;
        case 4: // AUTO
            lv_obj_set_style_bg_color(coord_container, lv_color_hex(0x00FF00), 0);
            break;
        default:
            lv_obj_set_style_bg_color(coord_container, lv_color_hex(0x1a1a2e), 0);
            break;
    }
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

// Обновление из ROS
void ui_update_telemetry(float *values, int count, uint8_t state, 
                        uint8_t tool_id, uint8_t work_offset, 
                        bool cartesian, float speed)
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
}


void ui_add_debug_log(const char *message)
{
    ui_set_terminal_text(message);
    ui_set_terminal_text("\n");
}