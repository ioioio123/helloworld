#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_err.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"

#include "board_config.h"
#include "display.h"

static const char *TAG = "lcd";

static esp_lcd_panel_handle_t s_panel;
static uint16_t *s_line;        /* 单行像素缓冲，必须 DMA 可访问 */
static uint16_t *s_envbuf;      /* 包络区域整块缓冲 */

#define ENV_W      LCD_H_RES
#define ENV_H      88
#define ENV_Y      68
#define ENV_BYTES  (ENV_W * ENV_H * sizeof(uint16_t))

/* ------------------------------------------------------------ 基础绘制 */

static inline uint16_t be16(uint16_t c)
{
    /* ST7789 默认大端（RAMCTRL 0xf0），驱动不做字节交换 */
    return (uint16_t)((c >> 8) | (c << 8));
}

static void fill_rect(int x, int y, int w, int h, uint16_t color)
{
    if (x < 0) { w += x; x = 0; }
    if (y < 0) { h += y; y = 0; }
    if (x + w > LCD_H_RES) { w = LCD_H_RES - x; }
    if (y + h > LCD_V_RES) { h = LCD_V_RES - y; }
    if (w <= 0 || h <= 0) {
        return;
    }

    uint16_t c = be16(color);
    for (int i = 0; i < w; i++) {
        s_line[i] = c;
    }
    for (int j = 0; j < h; j++) {
        esp_lcd_panel_draw_bitmap(s_panel, x, y + j, x + w, y + j + 1, s_line);
    }
}

/* ------------------------------------------------------------ 5x7 字体 */

typedef struct {
    char    ch;
    uint8_t rows[7];
} glyph_t;

static const glyph_t s_font[] = {
    { 'A', { 0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11 } },
    { 'C', { 0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E } },
    { 'D', { 0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E } },
    { 'E', { 0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F } },
    { 'I', { 0x0E, 0x04, 0x04, 0x04, 0x04, 0x04, 0x0E } },
    { 'K', { 0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11 } },
    { 'L', { 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F } },
    { 'M', { 0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11 } },
    { 'N', { 0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11 } },
    { 'O', { 0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E } },
    { 'P', { 0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10 } },
    { 'R', { 0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11 } },
    { 'S', { 0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E } },
    { 'T', { 0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04 } },
    { 'U', { 0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E } },
    { 'W', { 0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11 } },
    { 'm', { 0x00, 0x00, 0x1A, 0x15, 0x15, 0x11, 0x11 } },
    { 's', { 0x00, 0x00, 0x0F, 0x10, 0x0E, 0x01, 0x1E } },
};

static const uint8_t *glyph_find(char c)
{
    for (size_t i = 0; i < sizeof(s_font) / sizeof(s_font[0]); i++) {
        if (s_font[i].ch == c) {
            return s_font[i].rows;
        }
    }
    return NULL;
}

static void draw_char(int x, int y, int scale, char c, uint16_t color)
{
    const uint8_t *rows = glyph_find(c);
    if (!rows) {
        return;
    }
    for (int r = 0; r < 7; r++) {
        for (int b = 0; b < 5; b++) {
            if (rows[r] & (1 << (4 - b))) {
                fill_rect(x + b * scale, y + r * scale, scale, scale, color);
            }
        }
    }
}

static void draw_text(int x, int y, int scale, const char *str, uint16_t color)
{
    int cx = x;
    for (const char *p = str; *p; p++) {
        draw_char(cx, y, scale, *p, color);
        cx += 6 * scale;
    }
}

/* ------------------------------------------------------------ 七段数码管 */

static const uint8_t s_seg_map[10] = {
    0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F
};

static void draw_digit(int x, int y, int w, int h, int t, int d, uint16_t color)
{
    if (d < 0 || d > 9) {
        return;
    }
    uint8_t m = s_seg_map[d];
    if (m & 0x01) { fill_rect(x + t, y, w - 2 * t, t, color); }                  /* a */
    if (m & 0x02) { fill_rect(x + w - t, y + t, t, h / 2 - t, color); }          /* b */
    if (m & 0x04) { fill_rect(x + w - t, y + h / 2, t, h / 2 - t, color); }      /* c */
    if (m & 0x08) { fill_rect(x + t, y + h - t, w - 2 * t, t, color); }          /* d */
    if (m & 0x10) { fill_rect(x, y + h / 2, t, h / 2 - t, color); }              /* e */
    if (m & 0x20) { fill_rect(x, y + t, t, h / 2 - t, color); }                  /* f */
    if (m & 0x40) { fill_rect(x + t, y + h / 2 - t / 2, w - 2 * t, t, color); }  /* g */
}

/* 按 "12.3" 这种字符串渲染，数字用七段，'.' 画成点 */
static void draw_number(int x, int y, int dw, int dh, int t, const char *s, uint16_t color)
{
    int cx = x;
    for (const char *p = s; *p; p++) {
        if (*p >= '0' && *p <= '9') {
            draw_digit(cx, y, dw, dh, t, *p - '0', color);
            cx += dw + 2;
        } else if (*p == '.') {
            fill_rect(cx, y + dh - t, t, t, color);
            cx += 6;
        } else {
            cx += dw / 2;
        }
    }
}

/* ------------------------------------------------------------ 包络曲线 */

static void env_clear(uint16_t color)
{
    uint16_t c = be16(color);
    for (int i = 0; i < ENV_W * ENV_H; i++) {
        s_envbuf[i] = c;
    }
}

static void env_px(int x, int y, uint16_t color)
{
    if (x < 0 || y < 0 || x >= ENV_W || y >= ENV_H) {
        return;
    }
    s_envbuf[y * ENV_W + x] = be16(color);
}

static void env_render(const app_state_t *s)
{
    env_clear(COLOR_BLACK);

    /* 水平网格 */
    for (int g = 1; g < 4; g++) {
        int y = ENV_H - 1 - (ENV_H - 1) * g / 4;
        for (int x = 0; x < ENV_W; x++) {
            env_px(x, y, COLOR_DKGREY);
        }
    }

    int prev_y = ENV_H - 1;
    for (int i = 0; i < ENV_BUCKETS; i++) {
        int y = ENV_H - 1 - (int)((uint32_t)s->env[i] * (ENV_H - 1) / 4095u);
        int y0 = prev_y < y ? prev_y : y;
        int y1 = prev_y < y ? y : prev_y;
        for (int yy = y0; yy <= y1; yy++) {
            env_px(i, yy, COLOR_CYAN);
        }
        prev_y = y;
    }

    /* 峰值标记 */
    if (s->env_valid) {
        int x = s->env_peak_idx;
        for (int yy = 0; yy < ENV_H; yy += 3) {
            env_px(x, yy, COLOR_RED);
            env_px(x + 1, yy, COLOR_RED);
        }
    }
}

/* ------------------------------------------------------------ 对外接口 */

esp_err_t display_init(void)
{
    /* 1. SPI 总线 */
    spi_bus_config_t buscfg = {
        .sclk_io_num     = PIN_LCD_SCLK,
        .mosi_io_num     = PIN_LCD_MOSI,
        .miso_io_num     = -1,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    /* 2. 面板 IO：CS=GPIO6, DC=GPIO7 */
    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num       = PIN_LCD_CS,
        .dc_gpio_num       = PIN_LCD_DC,
        .spi_mode          = 0,
        .pclk_hz           = LCD_PIXEL_CLOCK_HZ,
        .trans_queue_depth = 10,
        .lcd_cmd_bits      = LCD_CMD_BITS,
        .lcd_param_bits    = LCD_PARAM_BITS,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_cfg, &io));

    /* 3. ST7789 面板（v6.1 自带驱动，不需要外部组件） */
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = PIN_LCD_RST,          /* -1：RES 硬连 3.3V */
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_BGR,
        .data_endian    = LCD_RGB_DATA_ENDIAN_BIG,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io, &panel_cfg, &s_panel));

    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    /* 多数 240x240 ST7789 模组出厂是反显，这里打开反色；
     * 若颜色整体反了，把下面这一行改成 false 即可。 */
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));

    /* 4. 绘制缓冲：必须放在 DMA 可访问的内存里 */
    s_line = (uint16_t *)spi_bus_dma_memory_alloc(LCD_HOST, LCD_H_RES * sizeof(uint16_t), 0);
    s_envbuf = (uint16_t *)spi_bus_dma_memory_alloc(LCD_HOST, ENV_BYTES, 0);
    if (!s_line || !s_envbuf) {
        ESP_LOGE(TAG, "显示缓冲分配失败（line=%p env=%p）", s_line, s_envbuf);
        return ESP_ERR_NO_MEM;
    }

    fill_rect(0, 0, LCD_H_RES, LCD_V_RES, COLOR_BLACK);
    draw_text(4, 4, 2, "PEAK", COLOR_GREY);

    ESP_LOGI(TAG, "ST7789 240x240 就绪（SPI2 / SCK=4 MOSI=5 CS=6 DC=7）");
    return ESP_OK;
}

void display_render(const app_state_t *s)
{
    static int32_t  last_peak   = 0x7FFFFFFF;
    static int32_t  last_temp   = 0x7FFFFFFF;
    static int32_t  last_pulse  = 0x7FFFFFFF;
    static int32_t  last_status = 0x7FFFFFFF;
    static uint32_t last_seq    = 0xFFFFFFFF;
    char buf[16];

    /* ---- 峰值电流（0.1A 分辨率） ---- */
    int32_t peak = s->env_valid
                 ? (int32_t)((float)s->env_peak * CURRENT_A_PER_COUNT * 10.0f + 0.5f)
                 : 0;
    if (peak != last_peak) {
        fill_rect(0, 22, LCD_H_RES, 46, COLOR_BLACK);
        snprintf(buf, sizeof(buf), "%4d.%d",
                 (int)(peak / 10), (int)(peak % 10));
        draw_number(8, 22, 26, 44, 6, buf, COLOR_CYAN);
        last_peak = peak;
    }

    /* ---- 包络曲线 ---- */
    if (s->env_seq != last_seq) {
        env_render(s);
        esp_lcd_panel_draw_bitmap(s_panel, 0, ENV_Y, ENV_W, ENV_Y + ENV_H, s_envbuf);
        last_seq = s->env_seq;
    }

    /* ---- 温度 ---- */
    int32_t temp10 = (s->temp_c > -100.0f) ? (int32_t)(s->temp_c * 10.0f) : -999;
    if (temp10 != last_temp) {
        fill_rect(0, 160, LCD_H_RES, 18, COLOR_BLACK);
        draw_text(4, 162, 2, "T", COLOR_GREY);
        if (temp10 > -900) {
            snprintf(buf, sizeof(buf), "%3d.%d", (int)(temp10 / 10), (int)(temp10 % 10));
        } else {
            snprintf(buf, sizeof(buf), "---.-");
        }
        draw_number(28, 160, 10, 18, 2, buf, s->temp_locked ? COLOR_RED : COLOR_GREEN);
        draw_text(104, 162, 2, "C", COLOR_GREY);
        last_temp = temp10;
    }

    /* ---- 脉宽 ---- */
    int32_t pulse10 = (int32_t)(s->pulse_us / 100);
    if (pulse10 != last_pulse) {
        fill_rect(120, 160, LCD_H_RES - 120, 18, COLOR_BLACK);
        draw_text(124, 162, 2, "P", COLOR_GREY);
        snprintf(buf, sizeof(buf), "%3d.%d", (int)(pulse10 / 10), (int)(pulse10 % 10));
        draw_number(148, 160, 10, 18, 2, buf, COLOR_WHITE);
        draw_text(200, 162, 1, "ms", COLOR_GREY);
        last_pulse = pulse10;
    }

    /* ---- 状态 ---- */
    int32_t status = s->temp_locked ? 2 : (s->welding ? 1 : 0);
    if (status != last_status) {
        fill_rect(0, 182, LCD_H_RES, 20, COLOR_BLACK);
        if (status == 0) {
            draw_text(4, 184, 2, "READY", COLOR_GREEN);
        } else if (status == 1) {
            draw_text(4, 184, 2, "WELD", COLOR_YELLOW);
        } else {
            draw_text(4, 184, 2, "LOCK", COLOR_RED);
        }
        last_status = status;
    }

    /* ---- 累计次数 ---- */
    fill_rect(0, 206, LCD_H_RES, 20, COLOR_BLACK);
    snprintf(buf, sizeof(buf), "N %lu", (unsigned long)s->weld_count);
    draw_text(4, 208, 2, buf, COLOR_WHITE);
}
