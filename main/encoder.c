#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "board_config.h"
#include "encoder.h"

static volatile int32_t s_count;
static volatile int64_t s_last_edge_us;

/* v6.1 的 taskENTER_CRITICAL() 要求传入 portMUX_TYPE */
static portMUX_TYPE s_count_mux = portMUX_INITIALIZER_UNLOCKED;

/* EC11 一格约 1~2ms，200us 足以滤掉触点抖动又不会吞掉快速旋转 */
#define EDGE_DEBOUNCE_US 500

static void IRAM_ATTR ec11_isr(void *arg)
{
    int64_t now = esp_timer_get_time();
    if (now - s_last_edge_us < EDGE_DEBOUNCE_US) {
        return;
    }
    s_last_edge_us = now;

    int a = gpio_get_level(PIN_EC11_A);
    int b = gpio_get_level(PIN_EC11_B);
    /* A 相边沿后读 B 相：同相为顺时针，异相为逆时针 */
    /* 顺时针递增：本编码器实测方向与直觉相反，这里取反 */
    s_count += (a == b) ? -1 : 1;
}

int32_t encoder_take_steps(void)
{
    taskENTER_CRITICAL(&s_count_mux);
    int32_t raw = s_count;
    s_count = 0;
    taskEXIT_CRITICAL(&s_count_mux);

    /* 实测本编码器每格 2 个边沿。用余数累积折算成「格」，
     * 避免整数除法把落单的边沿永久丢掉。 */
    static int32_t residual;
    residual += raw;
    int32_t steps = residual / 2;
    residual -= steps * 2;
    return steps;
}

bool encoder_switch_pressed(void)
{
    return gpio_get_level(PIN_EC11_SW) == 0;
}

int32_t encoder_take_clicks(void)
{
    static bool    last_pressed;
    static int64_t last_change_us;
    static int32_t clicks;

    bool    pressed = encoder_switch_pressed();
    int64_t now     = esp_timer_get_time();

    if (pressed != last_pressed && (now - last_change_us) > 20000) {
        last_change_us = now;
        last_pressed   = pressed;
        if (pressed) {
            clicks++;
        }
    }

    int32_t out = clicks;
    clicks = 0;
    return out;
}

void encoder_init(void)
{
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_EC11_A) | (1ULL << PIN_EC11_B) | (1ULL << PIN_EC11_SW),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io));

    /* A 相双边沿中断，B 相纯电平读取 */
    ESP_ERROR_CHECK(gpio_set_intr_type(PIN_EC11_A, GPIO_INTR_ANYEDGE));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_EC11_A, ec11_isr, NULL));

    ESP_LOGI("encoder", "EC11 已初始化（软件正交解码，C3 无 PCNT）");
}
