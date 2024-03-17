/*

ESP32-4848S040-demo
See
https://github.com/arendst/Tasmota/discussions/20527


*/

#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <esp_check.h>

#include "driver/gpio.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_io_additions.h"
#include "esp_lcd_st7701.h"

#include "lvgl.h"
#include "esp_lvgl_port.h"

#if CONFIG_EXAMPLE_TOUCH_I2C_NUM > -1
#include "esp_lcd_touch.h"
#include "driver/i2c.h"
#include "esp_lcd_touch_gt911.h"
#endif

static const char *TAG = "demo";

#define LVGL_TICK_PERIOD_MS 2
#include "esp_attr.h"
#include "esp_timer.h"

static void lcd_panel_test(esp_lcd_panel_handle_t panel_handle);

#include <esp_random.h>
#include "demos/widgets/lv_demo_widgets.h"

typedef struct {
    esp_lcd_panel_io_handle_t   panel_io_handle;
    esp_lcd_panel_handle_t      panel_handle;
    esp_lcd_panel_io_handle_t   touch_io_handle;
    esp_lcd_touch_handle_t      touch_handle;
} example_display_ctx_t;

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


static example_display_ctx_t *example_panel_init() {
    esp_err_t ret = ESP_OK;
    example_display_ctx_t *disp_ctx = malloc(sizeof(example_display_ctx_t));
    ESP_GOTO_ON_FALSE(disp_ctx, ESP_ERR_NO_MEM, err, TAG, "Not enough memory for display context allocation!");

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
    return disp_ctx;

    err:
    return NULL;
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
    lv_disp_drv_t *disp_drv = (lv_disp_drv_t *)user_ctx;
    assert(disp_drv != NULL);
    lv_disp_flush_ready(disp_drv);
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
                                                             disp->driver));
    /* Rotation of the screen */
    lv_disp_set_rotation(disp, LV_DISP_ROT_NONE);

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
    ESP_LOGI(TAG, "PSRAM free size: %d", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    example_display_ctx_t *ctx = example_panel_init();

    lcd_panel_test(ctx->panel_handle);

    lv_disp_t *disp = example_lvgl_init(ctx);

    lv_demo_widgets();

    while (1) {
        /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
        vTaskDelay(pdMS_TO_TICKS(LVGL_TICK_PERIOD_MS));
        lv_task_handler();
    }
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
#endif