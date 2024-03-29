/*

ESP32-4848S040-demo
See
https://github.com/arendst/Tasmota/discussions/20527


*/

#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <esp_check.h>

// lcd
#include "driver/gpio.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_io_additions.h"
#include "esp_lcd_st7701.h"
#include "driver/ledc.h"

#include "lvgl.h"
#include "esp_lvgl_port.h"

#if CONFIG_EXAMPLE_TOUCH_I2C_NUM > -1
#include "esp_lcd_touch.h"
#include "driver/i2c.h"
#include "esp_lcd_touch_gt911.h"
#endif

// sd card
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include <driver/sdmmc_defs.h>
#include <sys/types.h>
#include <dirent.h>

static const char *TAG = "demo";

#define LVGL_TICK_PERIOD_MS 2
#include "esp_attr.h"
#include "esp_timer.h"

#include <esp_random.h>
#include "demos/widgets/lv_demo_widgets.h"

typedef struct {
    esp_lcd_panel_io_handle_t   panel_io_handle;
    esp_lcd_panel_handle_t      panel_handle;
    esp_lcd_panel_io_handle_t   touch_io_handle;
    esp_lcd_touch_handle_t      touch_handle;
    char *mount_point;
    sdmmc_card_t *card; // = &sdmmc_card;
    int host_slot;
    lv_obj_t *file_table;
    lv_obj_t *fsinfo_label;
    lv_obj_t *mount_label;
    lv_obj_t * mbox1;
    lv_timer_t * backlight_timer;
} example_display_ctx_t;

static void lcd_panel_test(esp_lcd_panel_handle_t panel_handle);
static void sd_widget(example_display_ctx_t *disp_ctx);
static void sd_filetest(sdmmc_card_t *card);
static bool sd_mount(example_display_ctx_t *ctx);
static void sd_unmount(example_display_ctx_t *ctx);
static void sd_list_files(char *path);
#define MOUNT_POINT "/sdcard"

const st7701_lcd_init_cmd_t lcd_init_cmds1[] = {
    //BEGIN_WRITE,
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x10,}, 5, 0},
    {0xC0, (uint8_t []){0x3B, 0x00}, 2, 0},
    {0xC1, (uint8_t []){0x0D, 0x02}, 2, 0},
    {0xC2, (uint8_t []){0x31, 0x05}, 2, 0},
    {0xCD, (uint8_t []){0x00}, 1, 0},//0x08
     // Positive Voltage Gamma Control
    {0xB0, (uint8_t []){0x00, 0x11, 0x18, 0x0E, 0x11, 0x06, 0x07, 0x08, 0x07, 0x22, 0x04, 0x12, 0x0F, 0xAA, 0x31, 0x18,}, 16, 0},
     // Negative Voltage Gamma Control
    {0xB1, (uint8_t []){0x00, 0x11, 0x19, 0x0E, 0x12, 0x07, 0x08, 0x08, 0x08, 0x22, 0x04, 0x11, 0x11, 0xA9, 0x32, 0x18,}, 16, 0},
    // PAGE1
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x11,}, 5, 0},
    {0xB0, (uint8_t []){0x60}, 1, 0}, // Vop=4.7375v
    {0xB1, (uint8_t []){0x32}, 1, 0}, // VCOM=32
    {0xB2, (uint8_t []){0x07}, 1, 0}, // VGH=15v
    {0xB3, (uint8_t []){0x80}, 1, 0},
    {0xB5, (uint8_t []){0x49}, 1, 0}, // VGL=-10.17v
    {0xB7, (uint8_t []){0x85}, 1, 0},
    {0xB8, (uint8_t []){0x21}, 1, 0}, // AVDD=6.6 & AVCL=-4.6
    {0xC1, (uint8_t []){0x78}, 1, 0},
    {0xC2, (uint8_t []){0x78}, 1, 0},
    {0xE0, (uint8_t []){0x00, 0x1B, 0x02,}, 3, 0},
    {0xE1, (uint8_t []){0x08, 0xA0, 0x00, 0x00, 0x07, 0xA0, 0x00, 0x00, 0x00, 0x44, 0x44,}, 11, 0},
    {0xE2, (uint8_t []){0x11, 0x11, 0x44, 0x44, 0xED, 0xA0, 0x00, 0x00, 0xEC, 0xA0, 0x00, 0x00,}, 12, 0},
    {0xE3, (uint8_t []){0x00, 0x00, 0x11, 0x11,}, 4, 0},
    {0xE4, (uint8_t []){0x44, 0x44}, 2, 0},
    {0xE5, (uint8_t []){0x0A, 0xE9, 0xD8, 0xA0, 0x0C, 0xEB, 0xD8, 0xA0, 0x0E, 0xED, 0xD8, 0xA0, 0x10, 0xEF, 0xD8, 0xA0,}, 16, 0},
    {0xE6, (uint8_t []){0x00, 0x00, 0x11, 0x11,}, 4, 0},
    {0xE7, (uint8_t []){0x44, 0x44}, 2, 0},
    {0xE8, (uint8_t []){0x09, 0xE8, 0xD8, 0xA0, 0x0B, 0xEA, 0xD8, 0xA0, 0x0D, 0xEC, 0xD8, 0xA0, 0x0F, 0xEE, 0xD8, 0xA0,}, 16, 0},
    {0xEB, (uint8_t []){0x02, 0x00, 0xE4, 0xE4, 0x88, 0x00, 0x40,}, 7, 0},
    {0xEC, (uint8_t []){0x3C, 0x00}, 2, 0},
    {0xED, (uint8_t []){0xAB, 0x89, 0x76, 0x54, 0x02, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x20, 0x45, 0x67, 0x98, 0xBA,}, 16, 0},
    //-----------VAP & VAN---------------
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x13,}, 5, 0},
    {0xE5, (uint8_t []){0xE4}, 1, 0},
    {0xFF, (uint8_t []){0x77, 0x01, 0x00, 0x00, 0x00,}, 5, 0},
    {0x3A, (uint8_t []){0x60}, 1, 10}, // 0x70 RGB888, 0x60 RGB666, 0x50 RGB565
    //{0x21, (uint8_t []){0x00}, 0, 0},0X00,
    ////END_WRITE,
    //DELAY, 10,
    {0x11, (uint8_t []){0x00}, 0, 120}, // Sleep Out
    //END_WRITE,
    //DELAY, 120,
    //BEGIN_WRITE,
    {0x29, (uint8_t []){0x00}, 0, 0}, // Display On
    //END_WRITE
};

#define USE_BACKLIGHT_PWM 1
#if USE_BACKLIGHT_PWM
#define EXAMPLE_LEDC_CHANNEL LEDC_CHANNEL_3
#define EXAMPLE_LEDC_TIMER LEDC_TIMER_3

void backlight_level(int level) { // 0 - 1023
    ledc_set_duty(LEDC_LOW_SPEED_MODE, EXAMPLE_LEDC_CHANNEL, level);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, EXAMPLE_LEDC_CHANNEL);
}

void backlight_on() {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, EXAMPLE_LEDC_CHANNEL, 1023);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, EXAMPLE_LEDC_CHANNEL);
}

void backlight_off() {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, EXAMPLE_LEDC_CHANNEL, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, EXAMPLE_LEDC_CHANNEL);
}

static void backlight_del() {
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_reset_pin(CONFIG_EXAMPLE_LCD_BK_LIGHT_PIN));
}

static void backlight_init() {
    ledc_timer_config_t ledc_timer = {
        .duty_resolution = LEDC_TIMER_10_BIT, // resolution of PWM duty
        .freq_hz = 5000,                      // frequency of PWM signal
        .speed_mode = LEDC_LOW_SPEED_MODE,           // timer mode
        .timer_num = EXAMPLE_LEDC_TIMER,            // timer index
        .clk_cfg = LEDC_AUTO_CLK,              // Auto select the source clock
    };
    ledc_timer_config(&ledc_timer);
    ledc_channel_config_t led_cfg = {
        .channel    = EXAMPLE_LEDC_CHANNEL,
        .duty       = 0,
        .gpio_num   = CONFIG_EXAMPLE_LCD_BK_LIGHT_PIN,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .hpoint     = 0,
        .timer_sel  = EXAMPLE_LEDC_TIMER
    };
    ledc_channel_config(&led_cfg);
}

#else
void backlight_on() {
    // Turn on backlight (Different LCD screens may need different levels)
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(CONFIG_EXAMPLE_LCD_BK_LIGHT_PIN, CONFIG_EXAMPLE_LCD_BK_LIGHT_ON_LEVEL));
}

void backlight_off() {
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_set_level(CONFIG_EXAMPLE_LCD_BK_LIGHT_PIN, !CONFIG_EXAMPLE_LCD_BK_LIGHT_ON_LEVEL));
}

static void backlight_init() {
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << CONFIG_EXAMPLE_LCD_BK_LIGHT_PIN
    };
    // Initialize the GPIO of backlight
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));
}

static void backlight_del() {
    ESP_ERROR_CHECK_WITHOUT_ABORT(gpio_reset_pin(CONFIG_EXAMPLE_LCD_BK_LIGHT_PIN));
}
#endif

static bool example_panel_init(example_display_ctx_t *disp_ctx) {
    esp_err_t ret = ESP_OK;

    backlight_init();
    // Turn off backlight to avoid unpredictable display on the LCD screen while initializing
    // the LCD panel driver. (Different LCD screens may need different levels)
    backlight_off();

    esp_lcd_panel_io_handle_t panel_io_handle = NULL;
    esp_lcd_panel_handle_t panel_handle;

    ESP_LOGI(TAG, "Install 3-wire SPI panel IO");
    spi_line_config_t line_config = {
        .cs_io_type = IO_TYPE_GPIO, //IO_TYPE_EXPANDER, // Set to `IO_TYPE_GPIO` if using GPIO, same to below
        .cs_gpio_num = CONFIG_EXAMPLE_LCD_IO_SPI_CS,
        .scl_io_type = IO_TYPE_GPIO,
        .scl_gpio_num = CONFIG_EXAMPLE_LCD_IO_SPI_SCL,
        .sda_io_type = IO_TYPE_GPIO,
        .sda_gpio_num = CONFIG_EXAMPLE_LCD_IO_SPI_SDA,
        .io_expander = NULL,    // Set to NULL if not using IO expander
    };
    esp_lcd_panel_io_3wire_spi_config_t panel_io_config = ST7701_PANEL_IO_3WIRE_SPI_CONFIG(line_config, 0);
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_3wire_spi(&panel_io_config, &panel_io_handle));

    ESP_LOGI(TAG, "Install ST7701S panel driver");
    esp_lcd_rgb_panel_config_t rgb_config = {
        .clk_src = LCD_CLK_SRC_DEFAULT, // LCD_CLK_SRC_PLL160M
        .data_width = CONFIG_EXAMPLE_RGB_DATA_WIDTH,
        .bits_per_pixel = CONFIG_EXAMPLE_RGB_BIT_PER_PIXEL,
        .de_gpio_num = CONFIG_EXAMPLE_LCD_IO_RGB_DE,
        .pclk_gpio_num = CONFIG_EXAMPLE_LCD_IO_RGB_PCLK,
        .vsync_gpio_num = CONFIG_EXAMPLE_LCD_IO_RGB_VSYNC,
        .hsync_gpio_num = CONFIG_EXAMPLE_LCD_IO_RGB_HSYNC,
        .disp_gpio_num = CONFIG_EXAMPLE_LCD_IO_RGB_DISP,
        .data_gpio_nums = {
            CONFIG_EXAMPLE_LCD_IO_RGB_DATA0,
            CONFIG_EXAMPLE_LCD_IO_RGB_DATA1,
            CONFIG_EXAMPLE_LCD_IO_RGB_DATA2,
            CONFIG_EXAMPLE_LCD_IO_RGB_DATA3,
            CONFIG_EXAMPLE_LCD_IO_RGB_DATA4,
            CONFIG_EXAMPLE_LCD_IO_RGB_DATA5,
            CONFIG_EXAMPLE_LCD_IO_RGB_DATA6,
            CONFIG_EXAMPLE_LCD_IO_RGB_DATA7,
            CONFIG_EXAMPLE_LCD_IO_RGB_DATA8,
            CONFIG_EXAMPLE_LCD_IO_RGB_DATA9,
            CONFIG_EXAMPLE_LCD_IO_RGB_DATA10,
            CONFIG_EXAMPLE_LCD_IO_RGB_DATA11,
            CONFIG_EXAMPLE_LCD_IO_RGB_DATA12,
            CONFIG_EXAMPLE_LCD_IO_RGB_DATA13,
            CONFIG_EXAMPLE_LCD_IO_RGB_DATA14,
            CONFIG_EXAMPLE_LCD_IO_RGB_DATA15,
        },
        .timings = {
            .pclk_hz = CONFIG_EXAMPLE_LCD_PIXEL_CLOCK_HZ, /* > 10MHz does not work */
            .h_res = CONFIG_EXAMPLE_LCD_H_RES,
            .v_res = CONFIG_EXAMPLE_LCD_V_RES,
            .hsync_front_porch = CONFIG_EXAMPLE_LCD_H_F_PORCH,
            .hsync_pulse_width = CONFIG_EXAMPLE_LCD_H_P_WIDTH,
            .hsync_back_porch = CONFIG_EXAMPLE_LCD_H_B_PORCH,
            .vsync_front_porch = CONFIG_EXAMPLE_LCD_V_F_PORCH,
            .vsync_pulse_width = CONFIG_EXAMPLE_LCD_V_P_WIDTH,
            .vsync_back_porch = CONFIG_EXAMPLE_LCD_V_B_PORCH,
            .flags.pclk_active_neg = false,
            .flags.hsync_idle_low = 0, // (hsync_polarity == 0)
            .flags.vsync_idle_low = 0, // (vsync_polarity == 0)
            //.flags.de_idle_high = 0,
            //.flags.pclk_idle_high = 0,
        },
        .psram_trans_align = 64,
        .sram_trans_align = 8,
#if CONFIG_EXAMPLE_DOUBLE_FB
        .num_fbs = 2,
#else
        .num_fbs = 1,
#endif
        //.bounce_buffer_size_px = CONFIG_EXAMPLE_LCD_H_RES*8,
        .flags = {
            .fb_in_psram = 1, // allocate frame buffer in PSRAM
#if CONFIG_EXAMPLE_DOUBLE_FB
            .double_fb = 1,
#endif
            //.refresh_on_demand = 0,
            .bb_invalidate_cache = 1,
            //.disp_active_low = 0,
            //.relax_on_idle = 0,
        },
    };

    st7701_vendor_config_t vendor_config = {
        .rgb_config = &rgb_config,
        .init_cmds = lcd_init_cmds1,      // Uncomment these line if use custom initialization commands
        .init_cmds_size = sizeof(lcd_init_cmds1) / sizeof(st7701_lcd_init_cmd_t),
        .flags = {
            .auto_del_panel_io = 0,
            /**
             * Set to 1 if panel IO is no longer needed after LCD initialization.
             * If the panel IO pins are sharing other pins of the RGB interface to save GPIOs,
             * Please set it to 1 to release the pins.
             */
        },
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = CONFIG_EXAMPLE_LCD_IO_RST,           // Set to -1 if not use
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,     // Implemented by LCD command `36h`
        .bits_per_pixel = CONFIG_EXAMPLE_LCD_BIT_PER_PIXEL,    // Implemented by LCD command `3Ah` (16/18/24)
        .vendor_config = &vendor_config,
    };

    /**
     * Only create RGB when `auto_del_panel_io` is set to 0,
     * or initialize st7701 meanwhile
    */
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7701(panel_io_handle, &panel_config, &panel_handle));

    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));     // Only reset RGB when `auto_del_panel_io` is set to 1, or reset st7701 meanwhile
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));      // Only initialize RGB when `auto_del_panel_io` is set to 1, or initialize st7701 meanwhile

    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(panel_handle, false));
    // the gap is LCD panel specific, even panels with the same driver IC, can have different gap value
    esp_lcd_panel_set_gap(panel_handle, 0, 0);
    // Turn on the screen
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel_handle, true));

#if CONFIG_EXAMPLE_TOUCH_I2C_NUM > -1
    /*
    * Touch panel init
    */
    i2c_config_t conf = {
      .mode = I2C_MODE_MASTER,
      .sda_io_num = CONFIG_EXAMPLE_TOUCH_I2C_SDA,
      .scl_io_num = CONFIG_EXAMPLE_TOUCH_I2C_SCL,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .master = {
        .clk_speed = CONFIG_EXAMPLE_TOUCH_I2C_CLK_HZ,
      },
      .clk_flags = 0, /*!< Optional, you can use
        I2C_SCLK_SRC_FLAG_*
        flags to choose i2c source clock here. */
  };

  ESP_ERROR_CHECK(i2c_param_config(CONFIG_EXAMPLE_TOUCH_I2C_NUM, &conf));
  ESP_ERROR_CHECK(i2c_driver_install(CONFIG_EXAMPLE_TOUCH_I2C_NUM, I2C_MODE_MASTER, 0, 0, 0));

  esp_lcd_panel_io_handle_t touch_io_handle = NULL;
  esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
  // Attach the TOUCH to the I2C bus
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c((esp_lcd_i2c_bus_handle_t)CONFIG_EXAMPLE_TOUCH_I2C_NUM,
                                           &tp_io_config, &touch_io_handle));

  esp_lcd_touch_config_t tp_cfg = {
      .x_max = CONFIG_EXAMPLE_LCD_H_RES,
      .y_max = CONFIG_EXAMPLE_LCD_V_RES,
      .rst_gpio_num = CONFIG_EXAMPLE_TOUCH_GPIO_RST,
      .int_gpio_num = CONFIG_EXAMPLE_TOUCH_GPIO_INT,
      .levels =
          {
              .reset = 0,
              .interrupt = 0,
          },
      .flags =
          {
              .swap_xy = 0,
              .mirror_x = 0,
              .mirror_y = 0,
          },
      .process_coordinates = NULL,
      .interrupt_callback = NULL,
  };

  ESP_LOGI(TAG, "Initialize touch controller GT911");
#if DO_I2C_SCAN
    i2c_cmd_handle_t cmd;
    for (int i = 0; i < 0x7f; i++)
    {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (i << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        if (i2c_master_cmd_begin(CONFIG_EXAMPLE_TOUCH_I2C_NUM, cmd, portMAX_DELAY) == ESP_OK)
        {
            ESP_LOGI(TAG, "Got I2C response from address %02X", i);
        }
        i2c_cmd_link_delete(cmd);
    }
#endif
    esp_lcd_touch_handle_t      touch_handle;
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(touch_io_handle, &tp_cfg, &touch_handle));

    disp_ctx->touch_io_handle = touch_io_handle;
    disp_ctx->touch_handle = touch_handle;
#endif
    disp_ctx->panel_io_handle = panel_io_handle;
    disp_ctx->panel_handle = panel_handle;

    backlight_on();
    return true;
}

static void example_panel_del(example_display_ctx_t *disp_ctx, lv_disp_t *disp) {
    backlight_off();
    if (disp) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(lvgl_port_remove_disp(disp));
    }
    esp_lcd_panel_disp_on_off(disp_ctx->panel_handle, false);
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_lcd_panel_del(disp_ctx->panel_handle));
    ESP_ERROR_CHECK_WITHOUT_ABORT(esp_lcd_panel_io_del(disp_ctx->panel_io_handle));
    //ESP_ERROR_CHECK_WITHOUT_ABORT(spi_bus_free(CONFIG_EXAMPLE_SPI_HOST_ID));
    // TODO touch_del
    backlight_del();
}

IRAM_ATTR
static bool lvgl_flush_ready_callback(struct esp_lcd_panel_t *panel_io, const esp_lcd_rgb_panel_event_data_t *edata, void *user_ctx)
{
    lv_display_t *disp_drv = (lv_display_t *)user_ctx;
    assert(disp_drv != NULL);
    lv_display_flush_ready(disp_drv);
    return false;
}

static lv_disp_t * example_lvgl_init(example_display_ctx_t *disp_ctx) {
    ESP_LOGI(TAG, "Initialize LVGL with port");
    lvgl_port_cfg_t lvgl_cfg = ESP_LVGL_PORT_INIT_CONFIG();
    lvgl_cfg.timer_period_ms = LVGL_TICK_PERIOD_MS;
    lvgl_cfg.task_affinity = 0;
    lvgl_port_init(&lvgl_cfg);

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = disp_ctx->panel_io_handle,
        .panel_handle = disp_ctx->panel_handle,
        .buffer_size = CONFIG_EXAMPLE_LCD_H_RES * CONFIG_EXAMPLE_LCD_V_RES,
#if CONFIG_EXAMPLE_DOUBLE_FB
        .double_buffer = 1,
#endif
        .hres = CONFIG_EXAMPLE_LCD_H_RES,
        .vres = CONFIG_EXAMPLE_LCD_V_RES,
        .monochrome = false,
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = false,
            .buff_spiram = true,
        }
    };
    lv_disp_t * disp = lvgl_port_add_disp(&disp_cfg);

    // lvgl_port flush does not work with 3wire spi
    esp_lcd_rgb_panel_event_callbacks_t cbs = {
        .on_vsync = lvgl_flush_ready_callback,
    };
    ESP_ERROR_CHECK(esp_lcd_rgb_panel_register_event_callbacks(disp_ctx->panel_handle, &cbs,
                                                             disp));
    /* Rotation of the screen */
    lv_disp_set_rotation(disp, LV_DISPLAY_ROTATION_0);

#if CONFIG_EXAMPLE_TOUCH_I2C_NUM > -1
    /* Add touch input (for selected screen) */
    const lvgl_port_touch_cfg_t touch_cfg = {
        .disp = disp,
        .handle = disp_ctx->touch_handle,
    };
    // lv_indev_t* th =
    lvgl_port_add_touch(&touch_cfg);
#endif

    return disp;
}


void app_main(void)
{
    esp_err_t ret = ESP_OK;
    lv_disp_t *disp =  0;

    ESP_LOGI(TAG, "PSRAM free size: %d", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    example_display_ctx_t *ctx = malloc(sizeof(example_display_ctx_t));
    ESP_GOTO_ON_FALSE(ctx, ESP_ERR_NO_MEM, err, TAG, "Not enough memory for display context allocation!");
    memset(ctx, 0, sizeof(example_display_ctx_t));
    ctx->host_slot = SPI2_HOST;
    ctx->mount_point = MOUNT_POINT;
    ctx->card = 0;
    example_panel_init(ctx);

    lcd_panel_test(ctx->panel_handle);

    disp = example_lvgl_init(ctx);

    //lv_demo_widgets();
#if 1
    sd_widget(ctx);
#endif

    while (1) {
        /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
        vTaskDelay(pdMS_TO_TICKS(LVGL_TICK_PERIOD_MS));
        lv_task_handler();
    }
    err:
    sd_unmount(ctx);
    example_panel_del(ctx, disp);
}

#define TEST_IMG_SIZE (200 * 200 * sizeof(uint16_t))

// Draw some coloured rectangles
static void lcd_panel_test(esp_lcd_panel_handle_t panel_handle)
{
    uint8_t *img = heap_caps_malloc(TEST_IMG_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_ERROR_CHECK(img == 0);

    for (int i = 0; i < 100; i++) {
        uint8_t color_byte = esp_random() & 0xFF;
        int x_start = esp_random() % (CONFIG_EXAMPLE_LCD_H_RES - 200);
        int y_start = esp_random() % (CONFIG_EXAMPLE_LCD_V_RES - 200);
        memset(img, color_byte, TEST_IMG_SIZE);
        esp_lcd_panel_draw_bitmap(panel_handle, x_start, y_start, x_start + 200, y_start + 200, img);
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    free(img);
}

#if 0
#define CONFIG_EXAMPLE_LCD_H_RES              (480)
#define CONFIG_EXAMPLE_LCD_V_RES              (480)
#define CONFIG_EXAMPLE_LCD_BIT_PER_PIXEL      (16)
#define CONFIG_EXAMPLE_RGB_BIT_PER_PIXEL      (16)
#define CONFIG_EXAMPLE_RGB_DATA_WIDTH         (16)

#define CONFIG_EXAMPLE_DOUBLE_FB 1

// LCD SPI Control interface
//#define CONFIG_EXAMPLE_LCD_IO_SPI_CS_1        (GPIO_NUM_)
//#define CONFIG_EXAMPLE_LCD_IO_SPI_CS_2        (IO_EXPANDER_PIN_NUM_1)
#define CONFIG_EXAMPLE_LCD_IO_SPI_CS          (GPIO_NUM_39)
#define CONFIG_EXAMPLE_LCD_IO_SPI_SCL         (GPIO_NUM_48)
#define CONFIG_EXAMPLE_LCD_IO_SPI_SDA         (GPIO_NUM_47)
#define CONFIG_EXAMPLE_LCD_IO_RST             (GPIO_NUM_NC)
// RGB timings
#define CONFIG_EXAMPLE_LCD_PIXEL_CLOCK_HZ     (9000000L)
#define CONFIG_EXAMPLE_LCD_H_F_PORCH           10
#define CONFIG_EXAMPLE_LCD_H_P_WIDTH           8
#define CONFIG_EXAMPLE_LCD_H_B_PORCH           50
#define CONFIG_EXAMPLE_LCD_V_F_PORCH           10
#define CONFIG_EXAMPLE_LCD_V_P_WIDTH           8
#define CONFIG_EXAMPLE_LCD_V_B_PORCH           20

// LCD RGB parallel interface
#define CONFIG_EXAMPLE_LCD_IO_RGB_DISP        (GPIO_NUM_NC)
#define CONFIG_EXAMPLE_LCD_IO_RGB_VSYNC       (GPIO_NUM_17)
#define CONFIG_EXAMPLE_LCD_IO_RGB_HSYNC       (GPIO_NUM_16)
#define CONFIG_EXAMPLE_LCD_IO_RGB_DE          (GPIO_NUM_18)
#define CONFIG_EXAMPLE_LCD_IO_RGB_PCLK        (GPIO_NUM_21)
// Red
#define CONFIG_EXAMPLE_LCD_IO_RGB_DATA0      (GPIO_NUM_11)
#define CONFIG_EXAMPLE_LCD_IO_RGB_DATA1      (GPIO_NUM_12)
#define CONFIG_EXAMPLE_LCD_IO_RGB_DATA2      (GPIO_NUM_13)
#define CONFIG_EXAMPLE_LCD_IO_RGB_DATA3      (GPIO_NUM_14)
#define CONFIG_EXAMPLE_LCD_IO_RGB_DATA4      (GPIO_NUM_0)
// Green
#define CONFIG_EXAMPLE_LCD_IO_RGB_DATA5       (GPIO_NUM_8)
#define CONFIG_EXAMPLE_LCD_IO_RGB_DATA6       (GPIO_NUM_20)
#define CONFIG_EXAMPLE_LCD_IO_RGB_DATA7       (GPIO_NUM_3)
#define CONFIG_EXAMPLE_LCD_IO_RGB_DATA8       (GPIO_NUM_46)
#define CONFIG_EXAMPLE_LCD_IO_RGB_DATA9       (GPIO_NUM_9)
#define CONFIG_EXAMPLE_LCD_IO_RGB_DATA10      (GPIO_NUM_10)
// Blue
#define CONFIG_EXAMPLE_LCD_IO_RGB_DATA11       (GPIO_NUM_4)
#define CONFIG_EXAMPLE_LCD_IO_RGB_DATA12       (GPIO_NUM_5)
#define CONFIG_EXAMPLE_LCD_IO_RGB_DATA13       (GPIO_NUM_6)
#define CONFIG_EXAMPLE_LCD_IO_RGB_DATA14       (GPIO_NUM_7)
#define CONFIG_EXAMPLE_LCD_IO_RGB_DATA15       (GPIO_NUM_15)
/* LCD Backlight */
#define CONFIG_EXAMPLE_LCD_BK_LIGHT_PIN       (GPIO_NUM_38)
#define CONFIG_EXAMPLE_LCD_BK_LIGHT_ON_LEVEL   1

/* Touch settings */
#define CONFIG_EXAMPLE_TOUCH_I2C_NUM           (I2C_NUM_0)
#define CONFIG_EXAMPLE_TOUCH_I2C_CLK_HZ        (400000)
#define CONFIG_EXAMPLE_TOUCH_I2C_SDA           (GPIO_NUM_19)
#define CONFIG_EXAMPLE_TOUCH_I2C_SCL           (GPIO_NUM_45)
#define CONFIG_EXAMPLE_TOUCH_GPIO_INT          (GPIO_NUM_NC)
#define CONFIG_EXAMPLE_TOUCH_GPIO_RST          (GPIO_NUM_NC)

/* SD card */
#define CONFIG_EXAMPLE_PIN_MOSI 47
#define CONFIG_EXAMPLE_PIN_MISO 41
#define CONFIG_EXAMPLE_PIN_CLK 48
#define CONFIG_EXAMPLE_PIN_CS 42

#endif

static void show_dir(lv_obj_t * file_table, const char * path);


// Initialize sdcard without mounting
static void sdcard_init(example_display_ctx_t *ctx) {
    esp_err_t ret = ESP_OK;

    ESP_LOGI(TAG, "Initializing SD card");

    ESP_LOGI(TAG, "Using SPI peripheral");
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = CONFIG_EXAMPLE_PIN_MOSI,
        .miso_io_num = CONFIG_EXAMPLE_PIN_MISO,
        .sclk_io_num = CONFIG_EXAMPLE_PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(ctx->host_slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "Failed to initialize bus.");

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = CONFIG_EXAMPLE_PIN_CS;
    slot_config.host_id = ctx->host_slot;

    int card_handle = -1;   //uninitialized
    static sdmmc_card_t sdmmc_card;
    sdmmc_card_t *card = &sdmmc_card;

    ESP_LOGI(TAG, "init");
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = ctx->host_slot;
    ret = host.init();
    ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "Host init failed");

    ESP_LOGI(TAG, "init_sdspi_host");

    ret = sdspi_host_init_device(&slot_config, &card_handle);
    ESP_GOTO_ON_ERROR(ret, cleanup, TAG, "sdspi_host_init_device failed.");

    // probe and initialize card
    ESP_LOGI(TAG, "sdmmc_card_init");
    ret = sdmmc_card_init(&host, card);
    ESP_GOTO_ON_ERROR(ret, cleanup_host, TAG, "sdmmc_card_init failed");

    ctx->card = card;
    return;

cleanup_host:
    //call_host_deinit(host_config);
    if (host.flags & SDMMC_HOST_FLAG_DEINIT_ARG) {
        host.deinit_p(host.slot);
    } else {
        host.deinit();
    }
cleanup:
  return ;
}

static void sdcard_deinit(example_display_ctx_t *ctx) {
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = ctx->host_slot;
    if (host.flags & SDMMC_HOST_FLAG_DEINIT_ARG) {
        host.deinit_p(host.slot);
    } else {
        host.deinit();
    }
    spi_bus_free(ctx->host_slot);
    ctx->card = 0;
}

static bool sd_mount(example_display_ctx_t *ctx) {
    esp_err_t ret;

    spi_bus_config_t bus_cfg = {
        .mosi_io_num = CONFIG_EXAMPLE_PIN_MOSI,
        .miso_io_num = CONFIG_EXAMPLE_PIN_MISO,
        .sclk_io_num = CONFIG_EXAMPLE_PIN_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    ret = spi_bus_initialize(ctx->host_slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return false;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = CONFIG_EXAMPLE_PIN_CS;
    //slot_config.gpio_cd = PIN_NUM_CD;
    slot_config.host_id = ctx->host_slot;

    ESP_LOGI(TAG, "Mounting filesystem");
    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .format_if_mount_failed = true,
#else
        .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 20MHz for SDSPI)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = ctx->host_slot;
    ret = esp_vfs_fat_sdspi_mount(ctx->mount_point, &host, &slot_config, &mount_config, &ctx->card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return false;
    }
    ESP_LOGI(TAG, "Filesystem mounted");
    return true;
}

static void sd_unmount(example_display_ctx_t *ctx) {
  if (ctx->card) {
    esp_vfs_fat_sdcard_unmount(ctx->mount_point, ctx->card);
    ESP_LOGI(TAG, "Card unmounted");

    // deinitialize the bus after all devices are removed
    spi_bus_free(ctx->host_slot);
    ctx->card = 0;
  }
}

bool sd_format(example_display_ctx_t *ctx) {
    esp_err_t ret;
    if (!ctx->card) {
        return false;
    }
    // Format FATFS
    ret = esp_vfs_fat_sdcard_format(ctx->mount_point, ctx->card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to format FATFS (%s)", esp_err_to_name(ret));
        return false;
    }
    ESP_LOGI(TAG, "Filesystem formatted");
    return true;
}

// UI functions

static void sdmmc_card_info(lv_obj_t *label, const sdmmc_card_t *card) {
  const char *type;

  if (!card) {
    lv_label_set_text(label, "No card mounted\n");
    return;
  }

  if (card->is_sdio) {
    type = "SDIO";
  } else if (card->is_mmc) {
    type = "MMC";
  } else {
    type = (card->ocr & SD_OCR_SDHC_CAP) ? "SDHC/SDXC" : "SDSC";
  }

  lv_label_set_text_fmt(
      label, "Name: %s\nType: %s\nSize: %lluMB\n", card->cid.name, type,
      ((uint64_t)card->csd.capacity) * card->csd.sector_size / (1024 * 1024));
}

static void show_disk_free(lv_obj_t *fsinfo_label, const char *mount_point) {
    uint64_t out_total_bytes;
    uint64_t out_free_bytes;
    int ret = esp_vfs_fat_info(mount_point, &out_total_bytes, &out_free_bytes);
    if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to esp_vfs_fat_info.");
    return;
    }
    ESP_LOGI(TAG, "Total bytes: %lld, free bytes: %lld", out_total_bytes,
            out_free_bytes);

    lv_label_set_text_fmt(fsinfo_label, "Total bytes: %lld\nFree bytes: %lld",
        out_total_bytes, out_free_bytes);
}

static void mount_event_handler(lv_event_t *e) {
    example_display_ctx_t *ctx = (example_display_ctx_t *)e->user_data;

    if (!ctx->card) {
        ESP_LOGI(TAG, "Mounting SD card");
        sd_mount(ctx);
        sdmmc_card_print_info(stdout, ctx->card);
        show_disk_free(ctx->fsinfo_label, ctx->mount_point);
        lv_label_set_text(ctx->mount_label, "Unmount");
        sd_list_files(ctx->mount_point);
        show_dir(ctx->file_table, ctx->mount_point);
    } else {
        ESP_LOGI(TAG, "Unmounting SD card");
        sd_unmount(ctx);
        lv_label_set_text_fmt(ctx->fsinfo_label, "Not mounted");
        lv_label_set_text(ctx->mount_label, "Mount");
        lv_table_set_row_count(ctx->file_table, 0);
    }
}

void backlight_timer_cb(lv_timer_t * t) {
    example_display_ctx_t *ctx = (example_display_ctx_t *)t->user_data;
    backlight_level(80);
    lv_timer_pause(t);
}

static void backlight_event_cb(lv_event_t * e) {
    example_display_ctx_t *ctx = (example_display_ctx_t *)e->user_data;
    if (lv_timer_get_paused(ctx->backlight_timer)) {
        backlight_level(1023);
        lv_timer_reset(ctx->backlight_timer);
        lv_timer_resume(ctx->backlight_timer);
    }
}

static void dialog_yes_event_handler(lv_event_t *e) {
    example_display_ctx_t *ctx = (example_display_ctx_t *)e->user_data;
    lv_msgbox_close(ctx->mbox1);
    if (!ctx->card) {
        ESP_LOGI(TAG, "Mounting SD card");
        sd_mount(ctx);
    }
    ESP_LOGI(TAG, "Formatting SD card");
    sd_format(ctx);
    show_dir(ctx->file_table, ctx->mount_point);
    show_disk_free(ctx->fsinfo_label, ctx->mount_point);
    lv_label_set_text(ctx->mount_label, "Unmount");
}

static void dialog_no_event_handler(lv_event_t *e) {
    example_display_ctx_t *ctx = (example_display_ctx_t *)e->user_data;
    lv_msgbox_close(ctx->mbox1);
}

static void format_event_handler(lv_event_t *e) {
    example_display_ctx_t *ctx = (example_display_ctx_t *)e->user_data;

    ctx->mbox1 = lv_msgbox_create(NULL);

    lv_msgbox_add_title(ctx->mbox1, "Are you sure?");

    lv_msgbox_add_text(ctx->mbox1, "Formatting will wipe out all files");
    lv_msgbox_add_close_button(ctx->mbox1);

    lv_obj_t * btn;
    //btn = lv_msgbox_add_footer_button(ctx->mbox1, "Yes");
    //lv_obj_add_event_cb(btn, dialog_yes_event_handler, LV_EVENT_CLICKED, ctx);
    btn = lv_msgbox_add_footer_button(ctx->mbox1, "No");
    lv_obj_add_event_cb(btn, dialog_no_event_handler, LV_EVENT_CLICKED, ctx);
}

static void test_event_handler(lv_event_t *e) {
    example_display_ctx_t *ctx = (example_display_ctx_t *)e->user_data;
    ESP_LOGI(TAG, "Testing SD card file operations");
    if (ctx->card) {
        sd_filetest(ctx->card);
        sd_list_files(ctx->mount_point);
        show_dir(ctx->file_table, ctx->mount_point);
    }
}

static bool is_end_with(const char * str1, const char * str2)
{
    if(str1 == NULL || str2 == NULL)
        return false;
    uint16_t len1 = lv_strlen(str1);
    uint16_t len2 = lv_strlen(str2);
    if((len1 < len2) || (len1 == 0 || len2 == 0))
        return false;
    while(len2 >= 1) {
        if(str2[len2 - 1] != str1[len1 - 1])
            return false;
        len2--;
        len1--;
    }
    return true;
}

#define LV_FILE_EXPLORER_PATH_MAX_LEN 64

static void show_dir(lv_obj_t * file_table, const char * path)
{
    uint16_t index = 0;
#if CONFIG_LV_USE_FS_POSIX
    DIR* dir = opendir(path);
    if (dir == NULL) {
        LV_LOG_USER("Open dir error!");
        return;
    }
#else
    char fn[LV_FILE_EXPLORER_PATH_MAX_LEN];
    lv_fs_dir_t dir;
    lv_fs_res_t res;

    res = lv_fs_dir_open(&dir, path);
    if(res != LV_FS_RES_OK) {
        LV_LOG_USER("Open dir error %d!", res);
        return;
    }
#endif

    /*
    lv_table_set_cell_value_fmt(file_table, index++, 0, LV_SYMBOL_DIRECTORY "  %s", ".");
    lv_table_set_cell_value_fmt(file_table, index++, 0, LV_SYMBOL_DIRECTORY "  %s", "..");
    lv_table_set_cell_value(file_table, 0, 1, "0");
    lv_table_set_cell_value(file_table, 1, 1, "0");
    */

    while(1) {
#if CONFIG_LV_USE_FS_POSIX
        struct dirent* de = readdir(dir);
        if (!de) {
            break;
        }
        char *fn = de->d_name;
#else
        res = lv_fs_dir_read(&dir, fn, sizeof(fn));
        if(res != LV_FS_RES_OK) {
            LV_LOG_USER("Driver, file or directory is not exists %d!", res);
            break;
        }

        /*fn is empty, if not more files to read*/
        if(lv_strlen(fn) == 0) {
            LV_LOG_USER("Not more files to read!");
            break;
        }
#endif

        if((is_end_with(fn, ".png") == true)  || (is_end_with(fn, ".PNG") == true)  || \
           (is_end_with(fn, ".jpg") == true) || (is_end_with(fn, ".JPG") == true) || \
           (is_end_with(fn, ".bmp") == true) || (is_end_with(fn, ".BMP") == true) || \
           (is_end_with(fn, ".gif") == true) || (is_end_with(fn, ".GIF") == true)) {
            lv_table_set_cell_value_fmt(file_table, index, 0, LV_SYMBOL_IMAGE "  %s", fn);
            lv_table_set_cell_value(file_table, index, 1, "1");
        }
        else if((is_end_with(fn, ".mp3") == true) || (is_end_with(fn, ".MP3") == true) || \
                (is_end_with(fn, ".wav") == true) || (is_end_with(fn, ".WAV") == true)) {
            lv_table_set_cell_value_fmt(file_table, index, 0, LV_SYMBOL_AUDIO "  %s", fn);
            lv_table_set_cell_value(file_table, index, 1, "2");
        }
        else if((is_end_with(fn, ".mp4") == true) || (is_end_with(fn, ".MP4") == true)) {
            lv_table_set_cell_value_fmt(file_table, index, 0, LV_SYMBOL_VIDEO "  %s", fn);
            lv_table_set_cell_value(file_table, index, 1, "3");
        }
        else if((is_end_with(fn, ".") == true) || (is_end_with(fn, "..") == true)) {
            /*is dir*/
            continue;
        }
#if CONFIG_LV_USE_FS_POSIX
        else if(de->d_type == DT_DIR) {/*is dir*/
#else
        else if(fn[0] == '/') {/*is dir*/
#endif

            lv_table_set_cell_value_fmt(file_table, index, 0, LV_SYMBOL_DIRECTORY "  %s", fn + 1);
            lv_table_set_cell_value(file_table, index, 1, "0");
        }
        else {
            lv_table_set_cell_value_fmt(file_table, index, 0, LV_SYMBOL_FILE "  %s", fn);
            lv_table_set_cell_value(file_table, index, 1, "4");
        }

        index++;
    }

#if CONFIG_LV_USE_FS_POSIX
    closedir(dir);
#else
    lv_fs_dir_close(&dir);
#endif

    lv_table_set_row_count(file_table, index);
    lv_obj_scroll_to_y(file_table, 0, LV_ANIM_OFF);
}

static void sd_widget(example_display_ctx_t *ctx) {

    lv_disp_t *dispp = lv_disp_get_default();
    lv_theme_t *theme = lv_theme_default_init(dispp, lv_palette_main(LV_PALETTE_BLUE), lv_palette_main(LV_PALETTE_RED), true, LV_FONT_DEFAULT);
    lv_disp_set_theme(dispp, theme);

    ctx->backlight_timer = lv_timer_create(backlight_timer_cb, 30000,  ctx);
    lv_obj_add_event_cb(lv_scr_act(), backlight_event_cb, LV_EVENT_ALL, ctx);

    lv_obj_t *container = lv_obj_create(lv_scr_act());
    lv_obj_set_size(container, LV_PCT(100), LV_PCT(100));
    lv_obj_clear_flag( container, LV_OBJ_FLAG_SCROLLABLE );    /// Flags
    lv_obj_set_style_bg_color(container, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT );
    lv_obj_set_style_bg_opa(container, 255, LV_PART_MAIN| LV_STATE_DEFAULT);

    lv_obj_set_style_text_font(lv_scr_act(), LV_FONT_DEFAULT, 0);


    lv_obj_set_flex_flow(container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(container, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                            LV_FLEX_ALIGN_CENTER);

    lv_obj_t *cont1 = lv_obj_create(container);
    // lv_obj_set_size(cont, 300, 220);
    lv_obj_set_width(cont1, lv_pct(100));
    lv_obj_set_height(cont1, LV_SIZE_CONTENT);
    lv_obj_center(cont1);
    lv_obj_set_flex_flow(cont1, LV_FLEX_FLOW_ROW);

    lv_obj_t *card_label = lv_label_create(cont1);
    lv_obj_set_width(card_label, lv_pct(50));
    if (ctx->card) {
        sdmmc_card_print_info(stdout, ctx->card);
        sdmmc_card_info(card_label, ctx->card);
    } else {
        // Temporary init to get info
        sdcard_init(ctx);
        ESP_LOGI(TAG, "sdmmc_card_print_info");
        // Card has been initialized, print its properties
        sdmmc_card_print_info(stdout, ctx->card);
        sdmmc_card_info(card_label, ctx->card);
        sdcard_deinit(ctx);
    }
    ctx->fsinfo_label = lv_label_create(cont1);
    lv_obj_set_width(ctx->fsinfo_label, lv_pct(50));
    lv_label_set_text_fmt(ctx->fsinfo_label, "Not mounted");

    lv_obj_t *cont2 = lv_obj_create(container);
    // lv_obj_set_size(cont, 300, 220);
    lv_obj_set_width(cont2, lv_pct(100));
    lv_obj_set_height(cont2, LV_SIZE_CONTENT);
    lv_obj_center(cont2);
    lv_obj_set_flex_flow(cont2, LV_FLEX_FLOW_ROW);

    //lv_example_spinbox_1(cont2);

    lv_obj_t *mount_btn = lv_btn_create(cont2);
    lv_obj_set_height(mount_btn, LV_SIZE_CONTENT);
    lv_obj_set_width(mount_btn, LV_PCT(33));
    ctx->mount_label = lv_label_create(mount_btn);
    lv_label_set_text(ctx->mount_label, "Mount"); // or Unmount
    //lv_obj_set_width(ctx->mount_label, 100);
    lv_obj_center(ctx->mount_label);
    lv_obj_add_event_cb(mount_btn, mount_event_handler, LV_EVENT_CLICKED, ctx);

    lv_obj_t *format_btn = lv_btn_create(cont2);
    lv_obj_set_height(format_btn, LV_SIZE_CONTENT);
    lv_obj_set_width(format_btn, LV_PCT(33));
    lv_obj_t *format_label = lv_label_create(format_btn);
    lv_label_set_text(format_label, "Format");
    lv_obj_center(format_label);
    lv_obj_add_event_cb(format_btn, format_event_handler, LV_EVENT_CLICKED, ctx);

    lv_obj_t *test_btn = lv_btn_create(cont2);
    lv_obj_set_height(test_btn, LV_SIZE_CONTENT);
    lv_obj_set_width(test_btn, LV_PCT(33));
    lv_obj_t *test_label = lv_label_create(test_btn);
    lv_label_set_text(test_label, "Test");
    lv_obj_center(test_label);
    lv_obj_add_event_cb(test_btn, test_event_handler, LV_EVENT_CLICKED, ctx);

    /*Table showing the contents of the table of contents*/
    lv_obj_t *file_table = lv_table_create(container);
    lv_obj_set_size(file_table, LV_PCT(100), LV_PCT(86));
    lv_table_set_column_width(file_table, 0, LV_PCT(100));
    lv_table_set_column_count(file_table, 1);

    lv_obj_set_scroll_dir(file_table, LV_DIR_TOP | LV_DIR_BOTTOM);
    ctx->file_table = file_table;
}

// test utility

static void sd_list_files(char *path) {
  DIR *dir;
  struct dirent *ent;
  ESP_LOGI(TAG, "DIR------------ %s ", path);
  if ((dir = opendir(path)) != NULL) {
    /* print all the files and directories within directory */
    while ((ent = readdir(dir)) != NULL) {
      ESP_LOGI(TAG, " %s", ent->d_name);
    }
    closedir(dir);
  } else {
    ESP_LOGE(TAG, "Cannot open %s", path);
  }
  ESP_LOGI(TAG, "------------");
}

#define EXAMPLE_MAX_CHAR_SIZE    64

static esp_err_t s_example_write_file(const char *path, char *data)
{
    ESP_LOGI(TAG, "Opening file %s", path);
    FILE *f = fopen(path, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return ESP_FAIL;
    }
    fprintf(f, data);
    fclose(f);
    ESP_LOGI(TAG, "File written");

    return ESP_OK;
}

static esp_err_t s_example_read_file(const char *path)
{
    ESP_LOGI(TAG, "Reading file %s", path);
    FILE *f = fopen(path, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return ESP_FAIL;
    }
    char line[EXAMPLE_MAX_CHAR_SIZE];
    fgets(line, sizeof(line), f);
    fclose(f);

    // strip newline
    char *pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);

    return ESP_OK;
}

static void sd_filetest(sdmmc_card_t *card)
{
    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    const char *file_hello = MOUNT_POINT"/hello.txt";
    char data[EXAMPLE_MAX_CHAR_SIZE];
    snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%s %s!\n", "Hello", card->cid.name);
    esp_err_t ret = s_example_write_file(file_hello, data);
    if (ret != ESP_OK) {
        return;
    }

    const char *file_foo = MOUNT_POINT"/foo.txt";

    // Check if destination file exists before renaming
    struct stat st;
    if (stat(file_foo, &st) == 0) {
        // Delete it if it exists
        unlink(file_foo);
    }

    // Rename original file
    ESP_LOGI(TAG, "Renaming file %s to %s", file_hello, file_foo);
    if (rename(file_hello, file_foo) != 0) {
        ESP_LOGE(TAG, "Rename failed");
        return;
    }

    ret = s_example_read_file(file_foo);
    if (ret != ESP_OK) {
        return;
    }

    if (stat(file_foo, &st) == 0) {
        ESP_LOGI(TAG, "file still exists");
        //return;
    } else {
        ESP_LOGI(TAG, "file doesnt exist, format done");
    }

    const char *file_nihao = MOUNT_POINT"/nihao.txt";
    memset(data, 0, EXAMPLE_MAX_CHAR_SIZE);
    snprintf(data, EXAMPLE_MAX_CHAR_SIZE, "%s %s!\n", "Nihao", card->cid.name);
    ret = s_example_write_file(file_nihao, data);
    if (ret != ESP_OK) {
        return;
    }

    //Open file for reading
    ret = s_example_read_file(file_nihao);
    if (ret != ESP_OK) {
        return;
    }

    sd_list_files(MOUNT_POINT);
}
