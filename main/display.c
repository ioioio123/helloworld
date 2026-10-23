#include <string.h>
#include <stdio.h>

#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"

#include "board_config.h"
#include "cn_font.h"
#include "display.h"

static const char *TAG = "lcd";

static esp_lcd_panel_handle_t s_panel;
static SemaphoreHandle_t s_blit_done;
static uint16_t *s_envbuf;         /* 包络缓冲，兼作填充暂存 */
static uint16_t s_cnbuf[16 * 16];  /* 单个汉字缓冲 */

/* ---------------- 布局（240x240） ---------------- */
#define MARGIN_X      6

#define STATUS_Y      2
#define STATUS_H      22

#define PEAK_LABEL_Y  26
#define PEAK_NUM_Y    44
#define PEAK_H        46
#define PEAK_W        26
#define PEAK_T        7
#define PEAK_RIGHT    200
#define PEAK_UNIT_X   206

#define ENV_Y         94
#define ENV_H         84
#define ENV_W         LCD_H_RES

#define INFO_Y        182
#define PULSE_Y       206
#define BOT_H         20

/* ------------------------------------------------------------ blit */

static bool IRAM_ATTR on_color_done(esp_lcd_panel_io_handle_t io,
                                    esp_lcd_panel_io_event_data_t *edata,
                                    void *ctx)
{
    BaseType_t woken = pdFALSE;
    xSemaphoreGiveFromISR(s_blit_done, &woken);
    return (woken == pdTRUE);
}

/* draw_bitmap 只是排队，缓冲区在传输完成前不能复用 */
static void blit(int x0, int y0, int x1, int y1, const void *buf)
{
    esp_lcd_panel_draw_bitmap(s_panel, x0, y0, x1, y1, buf);
    xSemaphoreTake(s_blit_done, pdMS_TO_TICKS(200));
}

static inline uint16_t be16(uint16_t c)
{
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
    int chunk = ENV_H;
    if (chunk > h) {
        chunk = h;
    }
    int n = w * chunk;
    for (int i = 0; i < n; i++) {
        s_envbuf[i] = c;
    }
    for (int done = 0; done < h; done += chunk) {
        int rows = h - done;
        if (rows > chunk) {
            rows = chunk;
        }
        blit(x, y + done, x + w, y + done + rows, s_envbuf);
    }
}

/* ------------------------------------------------------------ 5x7 字体 */

typedef struct {
    char    ch;
    uint8_t rows[7];
} glyph_t;

static const glyph_t s_font[] = {
    { '0', { 0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E } },
    { '1', { 0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E } },
    { '2', { 0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F } },
    { '3', { 0x1F, 0x02, 0x04, 0x02, 0x01, 0x11, 0x0E } },
    { '4', { 0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02 } },
    { '5', { 0x1F, 0x10, 0x1E, 0x01, 0x01, 0x11, 0x0E } },
    { '6', { 0x06, 0x08, 0x10, 0x1E, 0x11, 0x11, 0x0E } },
    { '7', { 0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08 } },
    { '8', { 0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E } },
    { '9', { 0x0E, 0x11, 0x11, 0x0F, 0x01, 0x02, 0x0C } },
    { 'A', { 0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11 } },
    { 'C', { 0x0E, 0x11, 0x10, 0x10, 0x10, 0x11, 0x0E } },
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

static int text_width(int scale, const char *str)
{
    return (int)strlen(str) * 6 * scale - scale;
}

/* ------------------------------------------------------------ 中文点阵 */

static bool utf8_next(const char **pp, uint16_t *cp)
{
    const unsigned char *p = (const unsigned char *)*pp;
    if (*p == 0) {
        return false;
    }
    if (*p < 0x80) {
        *cp = *p;
        *pp += 1;
        return true;
    }
    if ((*p & 0xE0) == 0xC0 && p[1]) {
        *cp = (uint16_t)(((p[0] & 0x1F) << 6) | (p[1] & 0x3F));
        *pp += 2;
        return true;
    }
    if ((*p & 0xF0) == 0xE0 && p[1] && p[2]) {
        *cp = (uint16_t)(((p[0] & 0x0F) << 12) | ((p[1] & 0x3F) << 6) | (p[2] & 0x3F));
        *pp += 3;
        return true;
    }
    *pp += 1;
    return false;
}

static const uint8_t *cn_glyph_find(uint16_t cp)
{
    for (int i = 0; i < g_cn_glyph_count; i++) {
        if (g_cn_glyphs[i].code == cp) {
            return g_cn_glyphs[i].bmp;
        }
    }
    return NULL;
}

static void draw_cn_char(int x, int y, uint16_t cp, uint16_t fg)
{
    const uint8_t *bmp = cn_glyph_find(cp);
    if (!bmp) {
        return;
    }
    uint16_t f = be16(fg);
    uint16_t b = be16(COLOR_BLACK);

    for (int r = 0; r < 16; r++) {
        uint8_t hi = bmp[r * 2];
        uint8_t lo = bmp[r * 2 + 1];
        for (int c = 0; c < 16; c++) {
            int set = (c < 8) ? (hi & (1 << (7 - c))) : (lo & (1 << (15 - c)));
            s_cnbuf[r * 16 + c] = set ? f : b;
        }
    }
    blit(x, y, x + 16, y + 16, s_cnbuf);
}

static void draw_cn_text(int x, int y, const char *s, uint16_t fg)
{
    const char *p = s;
    int cx = x;
    uint16_t cp;
    while (*p) {
        if (!utf8_next(&p, &cp)) {
            break;
        }
        draw_cn_char(cx, y, cp, fg);
        cx += 16;
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
    if (m & 0x01) { fill_rect(x + t, y, w - 2 * t, t, color); }
    if (m & 0x02) { fill_rect(x + w - t, y + t, t, h / 2 - t, color); }
    if (m & 0x04) { fill_rect(x + w - t, y + h / 2, t, h / 2 - t, color); }
    if (m & 0x08) { fill_rect(x + t, y + h - t, w - 2 * t, t, color); }
    if (m & 0x10) { fill_rect(x, y + h / 2, t, h / 2 - t, color); }
    if (m & 0x20) { fill_rect(x, y + t, t, h / 2 - t, color); }
    if (m & 0x40) { fill_rect(x + t, y + h / 2 - t / 2, w - 2 * t, t, color); }
}

static int number_width(int dw, const char *s)
{
    int w = 0;
    for (const char *p = s; *p; p++) {
        if (*p >= '0' && *p <= '9') {
            w += dw + 2;
        } else if (*p == '.' || *p == '-') {
            w += 6;
        } else {
            w += dw / 2;
        }
    }
    return w;
}

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
        } else if (*p == '-') {
            fill_rect(cx, y + dh / 2 - t / 2, dw / 2, t, color);
            cx += 6;
        } else {
            cx += dw / 2;
        }
    }
}

static void draw_number_right(int x_right, int y, int dw, int dh, int t, const char *s, uint16_t color)
{
    draw_number(x_right - number_width(dw, s), y, dw, dh, t, s, color);
}

/* ------------------------------------------------------------ 包络（带坐标轴） */

/* 直接往包络缓冲里写字符（1:1 像素，仅支持 scale=1） */
static void env_char(int x, int y, char c, uint16_t color)
{
    const uint8_t *rows = glyph_find(c);
    if (!rows) {
        return;
    }
    uint16_t v = be16(color);
    for (int r = 0; r < 7; r++) {
        for (int b = 0; b < 5; b++) {
            if (rows[r] & (1 << (4 - b))) {
                int px = x + b;
                int py = y + r;
                if (px >= 0 && px < ENV_W && py >= 0 && py < ENV_H) {
                    s_envbuf[py * ENV_W + px] = v;
                }
            }
        }
    }
}

static void env_text(int x, int y, const char *s, uint16_t color)
{
    int cx = x;
    for (const char *p = s; *p; p++) {
        env_char(cx, y, *p, color);
        cx += 6;
    }
}

/* 绘图区（留出左侧 Y 轴标签、下方 X 轴标签） */
#define PLOT_X0  28
#define PLOT_X1  (ENV_W - 2)
#define PLOT_Y0  2
#define PLOT_Y1  (ENV_H - 14)

static void env_render(const app_state_t *s)
{
    uint16_t bg = be16(COLOR_BLACK);
    for (int i = 0; i < ENV_W * ENV_H; i++) {
        s_envbuf[i] = bg;
    }

    /* 网格 */
    for (int g = 0; g <= 4; g++) {
        int y = PLOT_Y1 - (PLOT_Y1 - PLOT_Y0) * g / 4;
        uint16_t c = be16(g == 0 ? COLOR_GREY : COLOR_DKGREY);
        for (int x = PLOT_X0; x <= PLOT_X1; x++) {
            s_envbuf[y * ENV_W + x] = c;
        }
    }
    for (int g = 0; g <= 4; g++) {
        int x = PLOT_X0 + (PLOT_X1 - PLOT_X0) * g / 4;
        uint16_t c = be16(g == 0 ? COLOR_GREY : COLOR_DKGREY);
        for (int y = PLOT_Y0; y <= PLOT_Y1; y++) {
            s_envbuf[y * ENV_W + x] = c;
        }
    }

    /* Y 轴刻度：满量程 / 半量程 / 0（单位 A） */
    int fs_a = (int)(4095 * CURRENT_A_PER_COUNT);
    char lbl[16];
    snprintf(lbl, sizeof(lbl), "%d", fs_a);
    env_text(2, PLOT_Y0 - 1, lbl, COLOR_GREY);
    snprintf(lbl, sizeof(lbl), "%d", fs_a / 2);
    env_text(2, (PLOT_Y0 + PLOT_Y1) / 2 - 3, lbl, COLOR_GREY);
    env_text(2, PLOT_Y1 - 7, "0", COLOR_GREY);

    /* X 轴刻度：0 与窗口末端（单位 ms） */
    uint32_t span_us = s->env_span_us ? s->env_span_us : 20000;
    int span_ms = (int)(span_us / 1000);
    env_text(PLOT_X0, PLOT_Y1 + 3, "0", COLOR_GREY);
    snprintf(lbl, sizeof(lbl), "%dms", span_ms);
    env_text(PLOT_X1 - text_width(1, lbl), PLOT_Y1 + 3, lbl, COLOR_GREY);

    /* 曲线 */
    int prev_y = PLOT_Y1;
    for (int i = 0; i < ENV_BUCKETS; i++) {
        int x = PLOT_X0 + (PLOT_X1 - PLOT_X0) * i / (ENV_BUCKETS - 1);
        int y = PLOT_Y1 - (int)((uint32_t)s->env[i] * (PLOT_Y1 - PLOT_Y0) / 4095u);
        int y0 = prev_y < y ? prev_y : y;
        int y1 = prev_y < y ? y : prev_y;
        uint16_t c = be16(COLOR_CYAN);
        for (int yy = y0; yy <= y1; yy++) {
            if (yy >= PLOT_Y0 && yy <= PLOT_Y1) {
                s_envbuf[yy * ENV_W + x] = c;
            }
        }
        prev_y = y;
    }

    /* 峰值标记 */
    if (s->env_valid && s->env_peak > 0) {
        int x = PLOT_X0 + (PLOT_X1 - PLOT_X0) * s->env_peak_idx / (ENV_BUCKETS - 1);
        uint16_t c = be16(COLOR_RED);
        for (int yy = PLOT_Y0; yy <= PLOT_Y1; yy += 4) {
            s_envbuf[yy * ENV_W + x] = c;
        }
    }
}

/* ------------------------------------------------------------ 各区域 */

static void draw_status(int status)
{
    fill_rect(0, STATUS_Y, 140, STATUS_H, COLOR_BLACK);
    if (status == 2) {
        draw_cn_text(MARGIN_X, STATUS_Y, "锁定", COLOR_RED);
    } else if (status == 1) {
        draw_cn_text(MARGIN_X, STATUS_Y, "放电", COLOR_YELLOW);
    } else {
        draw_cn_text(MARGIN_X, STATUS_Y, "就绪", COLOR_GREEN);
    }
}

static void draw_count(uint32_t count)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)count);

    fill_rect(146, STATUS_Y, LCD_H_RES - 146, STATUS_H, COLOR_BLACK);
    draw_cn_text(146, STATUS_Y, "次数", COLOR_GREY);
    draw_text(180, STATUS_Y + 4, 2, buf, COLOR_WHITE);
}

static void draw_peak(int32_t peak10)
{
    char buf[16];
    if (peak10 >= 1000) {
        snprintf(buf, sizeof(buf), "%ld", (long)(peak10 / 10));
    } else {
        snprintf(buf, sizeof(buf), "%ld.%ld", (long)(peak10 / 10), (long)(peak10 % 10));
    }

    fill_rect(0, PEAK_LABEL_Y, LCD_H_RES, PEAK_NUM_Y + PEAK_H - PEAK_LABEL_Y, COLOR_BLACK);
    draw_cn_text(MARGIN_X, PEAK_LABEL_Y, "峰值电流", COLOR_GREY);

    draw_number_right(PEAK_RIGHT, PEAK_NUM_Y, PEAK_W, PEAK_H, PEAK_T, buf, COLOR_CYAN);
    draw_text(PEAK_UNIT_X, PEAK_NUM_Y + PEAK_H - 16, 2, "A", COLOR_GREY);
}

static void draw_info(int32_t t10, int32_t rms10)
{
    char buf[16];

    fill_rect(0, INFO_Y, LCD_H_RES, BOT_H + 2, COLOR_BLACK);

    /* 左：温度 */
    draw_cn_text(MARGIN_X, INFO_Y, "温度", COLOR_GREY);
    if (t10 <= -900) {
        snprintf(buf, sizeof(buf), "---");
    } else if (t10 < 0) {
        snprintf(buf, sizeof(buf), "-%ld.%ld", (long)((-t10) / 10), (long)((-t10) % 10));
    } else {
        snprintf(buf, sizeof(buf), "%ld.%ld", (long)(t10 / 10), (long)(t10 % 10));
    }
    draw_number_right(104, INFO_Y, 11, BOT_H, 3, buf, COLOR_WHITE);
    draw_text(108, INFO_Y + 2, 1, "C", COLOR_GREY);

    /* 右：有效电流 */
    draw_cn_text(124, INFO_Y, "有效", COLOR_GREY);
    if (rms10 >= 1000) {
        snprintf(buf, sizeof(buf), "%ld", (long)(rms10 / 10));
    } else {
        snprintf(buf, sizeof(buf), "%ld.%ld", (long)(rms10 / 10), (long)(rms10 % 10));
    }
    draw_number_right(222, INFO_Y, 11, BOT_H, 3, buf, COLOR_WHITE);
    draw_text(226, INFO_Y + 2, 1, "A", COLOR_GREY);
}

static void draw_pulse(int32_t p10)
{
    char buf[16];
    snprintf(buf, sizeof(buf), "%ld.%ld", (long)(p10 / 10), (long)(p10 % 10));

    fill_rect(0, PULSE_Y, LCD_H_RES, BOT_H + 2, COLOR_BLACK);
    draw_cn_text(MARGIN_X, PULSE_Y, "脉宽", COLOR_GREY);
    draw_number_right(190, PULSE_Y, 11, BOT_H, 3, buf, COLOR_WHITE);
    draw_text(195, PULSE_Y + 2, 1, "ms", COLOR_GREY);
}

/* ------------------------------------------------------------ 初始化 */

esp_err_t display_init(void)
{
    spi_bus_config_t buscfg = {
        .sclk_io_num     = PIN_LCD_SCLK,
        .mosi_io_num     = PIN_LCD_MOSI,
        .miso_io_num     = -1,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = LCD_H_RES * LCD_V_RES * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

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

    s_blit_done = xSemaphoreCreateBinary();
    if (!s_blit_done) {
        ESP_LOGE(TAG, "创建 blit 信号量失败");
        return ESP_ERR_NO_MEM;
    }
    const esp_lcd_panel_io_callbacks_t io_cbs = {
        .on_color_trans_done = on_color_done,
    };
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io, &io_cbs, NULL));

    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = PIN_LCD_RST,
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_BGR,
        .data_endian    = LCD_RGB_DATA_ENDIAN_BIG,
        .bits_per_pixel = 16,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(io, &panel_cfg, &s_panel));

    ESP_ERROR_CHECK(esp_lcd_panel_init(s_panel));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(s_panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(s_panel, true));

    s_envbuf = (uint16_t *)spi_bus_dma_memory_alloc(LCD_HOST, ENV_W * ENV_H * sizeof(uint16_t), 0);
    if (!s_envbuf) {
        ESP_LOGE(TAG, "显示缓冲分配失败");
        return ESP_ERR_NO_MEM;
    }

    fill_rect(0, 0, LCD_H_RES, LCD_V_RES, COLOR_BLACK);

    ESP_LOGI(TAG, "ST7789 240x240 就绪（%d MHz）", LCD_PIXEL_CLOCK_HZ / 1000000);
    return ESP_OK;
}

void display_render(const app_state_t *s)
{
    static int32_t  last_status = -1;
    static int32_t  last_count  = -1;
    static int32_t  last_peak   = -1;
    static int32_t  last_temp   = -1;
    static int32_t  last_rms    = -1;
    static int32_t  last_pulse  = -1;
    static uint32_t last_seq    = 0xFFFFFFFF;

    int32_t status = s->temp_locked ? 2 : (s->welding ? 1 : 0);
    int32_t count  = (int32_t)s->weld_count;

    if (status != last_status) {
        draw_status(status);
        last_status = status;
        draw_count((uint32_t)count);
        last_count = count;
    }
    if (count != last_count) {
        draw_count((uint32_t)count);
        last_count = count;
    }

    int32_t peak10 = s->env_valid ? (int32_t)(s->env_peak * CURRENT_A_PER_COUNT * 10.0f + 0.5f) : 0;
    if (peak10 != last_peak) {
        draw_peak(peak10);
        last_peak = peak10;
    }

    int32_t t10   = (s->temp_c > -100.0f) ? (int32_t)(s->temp_c * 10.0f) : -999;
    int32_t rms10 = s->env_valid ? (int32_t)(s->env_rms * CURRENT_A_PER_COUNT * 10.0f + 0.5f) : 0;
    if (t10 != last_temp || rms10 != last_rms) {
        draw_info(t10, rms10);
        last_temp = t10;
        last_rms  = rms10;
    }

    int32_t p10 = (int32_t)(s->pulse_us / 100);
    if (p10 != last_pulse) {
        draw_pulse(p10);
        last_pulse = p10;
    }

    if (s->env_seq != last_seq) {
        env_render(s);
        blit(0, ENV_Y, ENV_W, ENV_Y + ENV_H, s_envbuf);
        last_seq = s->env_seq;
    }
}
