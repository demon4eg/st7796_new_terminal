#include "display_config.h"
#include <driver/gpio.h>
#include <driver/ledc.h>
#include <driver/spi_master.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_ili9488.h>
#include <esp_log.h>
#include "sdkconfig.h"

static const char *TAG = "display_config";

// Display configuration
#define DISPLAY_HORIZONTAL_PIXELS    320
#define DISPLAY_VERTICAL_PIXELS      480
#define DISPLAY_COMMAND_BITS         8
#define DISPLAY_PARAMETER_BITS       8
#define DISPLAY_REFRESH_HZ           40000000
#define DISPLAY_SPI_QUEUE_LEN        10
#define SPI_MAX_TRANSFER_SIZE        32768
#define LV_BUFFER_SIZE               (DISPLAY_HORIZONTAL_PIXELS * 25)

// Backlight configuration
#define BACKLIGHT_LEDC_MODE          LEDC_LOW_SPEED_MODE
#define BACKLIGHT_LEDC_CHANNEL       LEDC_CHANNEL_0
#define BACKLIGHT_LEDC_TIMER         LEDC_TIMER_1
#define BACKLIGHT_LEDC_TIMER_BIT     LEDC_TIMER_10_BIT
#define BACKLIGHT_LEDC_FREQUENCY     5000

// Pin mapping for ESP32-S3 (YOUR correct pins)
#define PIN_SPI_CLK     GPIO_NUM_12
#define PIN_SPI_MOSI    GPIO_NUM_11
#define PIN_SPI_MISO    GPIO_NUM_13
#define PIN_TFT_CS      GPIO_NUM_10
#define PIN_TFT_DC      GPIO_NUM_9
#define PIN_TFT_RST     GPIO_NUM_NC
#define PIN_TFT_BL      GPIO_NUM_1

static esp_lcd_panel_io_handle_t io_handle = NULL;
static esp_lcd_panel_handle_t panel_handle = NULL;

static void backlight_init(void)
{
    ledc_timer_config_t timer_config = {
        .speed_mode = BACKLIGHT_LEDC_MODE,
        .duty_resolution = BACKLIGHT_LEDC_TIMER_BIT,
        .timer_num = BACKLIGHT_LEDC_TIMER,
        .freq_hz = BACKLIGHT_LEDC_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK
    };
    
    ledc_channel_config_t channel_config = {
        .gpio_num = PIN_TFT_BL,
        .speed_mode = BACKLIGHT_LEDC_MODE,
        .channel = BACKLIGHT_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BACKLIGHT_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    
    ESP_ERROR_CHECK(ledc_timer_config(&timer_config));
    ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
    ESP_LOGI(TAG, "Backlight initialized on pin %d", PIN_TFT_BL);
}

esp_err_t display_init(void)
{
    // Initialize backlight
    backlight_init();
    display_set_brightness(0);
    
    // Initialize SPI bus
    ESP_LOGI(TAG, "Initializing SPI bus (MOSI:%d, MISO:%d, CLK:%d)",
             PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_CLK);
    
    spi_bus_config_t bus_config = {
        .mosi_io_num = PIN_SPI_MOSI,
        .miso_io_num = PIN_SPI_MISO,
        .sclk_io_num = PIN_SPI_CLK,
        .quadwp_io_num = GPIO_NUM_NC,
        .quadhd_io_num = GPIO_NUM_NC,
        .max_transfer_sz = SPI_MAX_TRANSFER_SIZE,
        .flags = SPICOMMON_BUSFLAG_SCLK | SPICOMMON_BUSFLAG_MISO |
                 SPICOMMON_BUSFLAG_MOSI | SPICOMMON_BUSFLAG_MASTER,
        .intr_flags = ESP_INTR_FLAG_LOWMED | ESP_INTR_FLAG_IRAM
    };
    
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &bus_config, SPI_DMA_CH_AUTO));
    
    // LCD IO configuration
    esp_lcd_panel_io_spi_config_t io_config = {
        .cs_gpio_num = PIN_TFT_CS,
        .dc_gpio_num = PIN_TFT_DC,
        .spi_mode = 0,
        .pclk_hz = DISPLAY_REFRESH_HZ,
        .trans_queue_depth = DISPLAY_SPI_QUEUE_LEN,
        .lcd_cmd_bits = DISPLAY_COMMAND_BITS,
        .lcd_param_bits = DISPLAY_PARAMETER_BITS,
        .flags = {
            .dc_low_on_data = 0,
            .octal_mode = 0,
            .sio_mode = 0,
            .lsb_first = 0,
            .cs_high_active = 0
        }
    };
    
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI2_HOST, &io_config, &io_handle));
    
    // LCD panel configuration
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_TFT_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 18,
        .flags = {
            .reset_active_high = 0
        },
        .vendor_config = NULL
    };
    
    ESP_ERROR_CHECK(esp_lcd_new_panel_ili9488(io_handle, &panel_config, LV_BUFFER_SIZE, &panel_handle));
    
    // Initialize panel
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(panel_handle, false));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(panel_handle, true, true));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(panel_handle, 0, 0));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));
    ESP_LOGI(TAG, "Display initialized successfully");
    return ESP_OK;
}

void display_set_brightness(int percent)
{
    if (percent > 100) percent = 100;
    if (percent < 0) percent = 0;
    
    uint32_t duty_cycle = (1023 * percent) / 100;
    ESP_ERROR_CHECK(ledc_set_duty(BACKLIGHT_LEDC_MODE, BACKLIGHT_LEDC_CHANNEL, duty_cycle));
    ESP_ERROR_CHECK(ledc_update_duty(BACKLIGHT_LEDC_MODE, BACKLIGHT_LEDC_CHANNEL));
    
    ESP_LOGI(TAG, "Brightness set to %d%%", percent);
}

void display_backlight_off(void)
{
    display_set_brightness(0);
}

void display_backlight_on(void)
{
    display_set_brightness(75);
}

esp_lcd_panel_handle_t display_get_panel_handle(void)
{
    return panel_handle;
}