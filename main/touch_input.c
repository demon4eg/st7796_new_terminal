#include "touch_input.h"
#include "display_config.h"
#include <esp_log.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_touch_xpt2046.h>
#include <driver/spi_master.h>

static const char *TAG = "touch_input";

#define TOUCH_MOSI_PIN      GPIO_NUM_35
#define TOUCH_MISO_PIN      GPIO_NUM_37
#define TOUCH_CLK_PIN       GPIO_NUM_36
#define TOUCH_CS_PIN        GPIO_NUM_39
#define TOUCH_IRQ_PIN       GPIO_NUM_40

#define DISPLAY_WIDTH       320
#define DISPLAY_HEIGHT      480

static esp_lcd_touch_handle_t tp = NULL;

esp_err_t touch_init(void)
{
    ESP_LOGI(TAG, "Initializing XPT2046 on dedicated SPI3 bus");
    
    // First, initialize SPI3 bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = TOUCH_MOSI_PIN,
        .miso_io_num = TOUCH_MISO_PIN,
        .sclk_io_num = TOUCH_CLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 64,
    };
    
    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &bus_cfg, SPI_DMA_DISABLED);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI3 bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure touch SPI on SPI3_HOST
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_spi_config_t tp_io_config = {
        .cs_gpio_num = TOUCH_CS_PIN,
        .dc_gpio_num = -1,
        .spi_mode = 0,
        .pclk_hz = 2 * 1000 * 1000,
        .trans_queue_depth = 5,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    
    ret = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI3_HOST, &tp_io_config, &tp_io_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create touch SPI IO: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Touch config with rotation handling
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = DISPLAY_WIDTH,
        .y_max = DISPLAY_HEIGHT,
        .rst_gpio_num = -1,
        .int_gpio_num = TOUCH_IRQ_PIN,
        .flags = {
            .swap_xy = 0,      // Swap for 270° rotation
            .mirror_x = 1,     // Mirror after swap
            .mirror_y = 0,
        },
    };
    
    ret = esp_lcd_touch_new_spi_xpt2046(tp_io_handle, &tp_cfg, &tp);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize touch: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Touch initialized successfully with rotation correction");
    return ESP_OK;
}

void touch_driver_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
    if (!tp) {
        data->state = LV_INDEV_STATE_REL;
        data->continue_reading = false;
        return;
    }
    
    uint16_t x[1], y[1], strength[1];
    uint8_t count = 0;
    
    esp_lcd_touch_read_data(tp);
    
    if (esp_lcd_touch_get_coordinates(tp, x, y, strength, &count, 1)) {
        data->point.x = x[0];
        data->point.y = y[0];
        data->state = LV_INDEV_STATE_PRESSED;
        
        static int counter = 0;
        if (counter++ % 100 == 0) {
            ESP_LOGI(TAG, "Touch: screen(%d, %d)", x[0], y[0]);
        }
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
    
    data->continue_reading = false;
}