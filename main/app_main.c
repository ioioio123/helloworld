#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "board_config.h"
#include "app_state.h"
#include "display.h"
#include "current_sense.h"
#include "encoder.h"
#include "weld.h"
#include "ble.h"

static const char *TAG = "app";

#define BUZZER_ON_LEVEL    1
#define BUZZER_IDLE_LEVEL  (!BUZZER_ON_LEVEL)

#if !CONFIG_BENCH_MODE
static esp_timer_handle_t s_beep_timer;

static void beep_off_cb(void *arg)
{
    gpio_set_level(PIN_BUZZER, BUZZER_IDLE_LEVEL);
}

static void beep(int ms)
{
    gpio_set_level(PIN_BUZZER, BUZZER_ON_LEVEL);
    esp_timer_stop(s_beep_timer);
    esp_timer_start_once(s_beep_timer, (uint64_t)ms * 1000);
}
#else
static int s_led_level;

static void bench_led_set(int on)
{
    gpio_set_level(PIN_BENCH_LED, on ? BENCH_LED_ON : BENCH_LED_OFF);
}
#endif

/* ------------------------------------------------------------ UI 任务 */

static void ui_task(void *arg)
{
    app_state_t snap;

    for (;;) {
        static int32_t sel = 0;
        static bool    btn_down = false;
        static int64_t btn_down_us = 0;
        static bool    long_done = false;

        int32_t steps = encoder_take_steps();
        (void)encoder_take_clicks();

        if (steps != 0) {
            int32_t next = (int32_t)weld_get_pulse_us(sel) + steps * 500;
            if (next < 1000) {
                next = 1000;
            } else if (next > 48000) {
                next = 48000;
            }
            weld_set_pulse_us(sel, (uint32_t)next);
            ESP_LOGI(TAG, "旋钮 P%d steps=%d -> %d.%d ms", sel + 1, (int)steps,
                     (int)(next / 1000), (int)((next % 1000) / 100));
        }

        bool pressed = encoder_switch_pressed();
        int64_t now_us = esp_timer_get_time();
        if (pressed && !btn_down) {
            btn_down = true;
            btn_down_us = now_us;
            long_done = false;
        } else if (pressed && btn_down && !long_done && (now_us - btn_down_us) > 800000) {
            long_done = true;
            weld_set_pulse_us(0, WELD_P1_US);
            weld_set_pulse_us(1, WELD_P2_US);
            ESP_LOGI(TAG, "长按：恢复默认 P1=%d P2=%d ms", WELD_P1_US / 1000, WELD_P2_US / 1000);
        } else if (!pressed && btn_down) {
            btn_down = false;
            if (!long_done) {
                sel = !sel;
                ESP_LOGI(TAG, "切换调节项 -> P%d", sel + 1);
            }
        }

        app_state_t *s = app_state_lock();
        if (steps) {
            s->knob += steps;
        }
        s->pulse1_us = weld_get_pulse_us(0);
        s->pulse2_us = weld_get_pulse_us(1);
        s->pulse_sel = sel;
        s->cap_mv    = 0;      /* 电压检测硬件未接 */
        snap = *s;                      /* 加锁拷贝快照 */
        app_state_unlock();

#if CONFIG_BENCH_MODE
        /* 无屏幕：用板载 LED 做 1Hz 心跳，证明任务在跑 */
        static int beat;
        if (++beat >= 10) {             /* 50ms * 10 = 500ms 翻转 */
            beat = 0;
            s_led_level = !s_led_level;
            bench_led_set(s_led_level);
        }
        (void)snap;
#else
        display_render(&snap);
#endif

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ------------------------------------------------------------ 入口 */

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-C3 点焊机控制器启动");

    /* NVS 是 BLE 协议栈的依赖（存储 PHY 校准 / 绑定信息） */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

#if CONFIG_BENCH_MODE
    ESP_LOGW(TAG, "===== BENCH 模式 =====");
    ESP_LOGW(TAG, "触发: 板载 BOOT 键 (GPIO9)   心跳: 板载 LED (GPIO%d)", CONFIG_BENCH_LED_GPIO);
    ESP_LOGW(TAG, "屏幕/ADC 不初始化，NTC 超温锁定旁路");

    gpio_config_t led = {
        .pin_bit_mask = (1ULL << PIN_BENCH_LED),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&led));
    bench_led_set(0);
#elif CONFIG_SIM_WELD
    /* 模拟点焊模式：GPIO9 用作触发输入，不驱动蜂鸣器 */
#else
    /* 蜂鸣器：空闲拉低（静音），响时拉高。GPIO9 是 strapping 脚，
     * 若外部电路会在上电瞬间把它拉低，请务必检查，否则会进下载模式。 */
    gpio_config_t bz = {
        .pin_bit_mask = (1ULL << PIN_BUZZER),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&bz));
    gpio_set_level(PIN_BUZZER, BUZZER_IDLE_LEVEL);

    const esp_timer_create_args_t beep_args = {
        .callback = beep_off_cb,
        .name     = "beep",
    };
    ESP_ERROR_CHECK(esp_timer_create(&beep_args, &s_beep_timer));
#endif

    /* 共享状态 */
    app_state_init();

#if !CONFIG_BENCH_MODE
    ESP_ERROR_CHECK(display_init());
#if CONFIG_BOOT_ANIM
    display_boot_anim();
#endif
#endif

    /* GPIO 中断服务必须先安装，encoder/weld 都要用它 */
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    encoder_init();
    weld_init();

#if !CONFIG_BENCH_MODE && !CONFIG_SIM_WELD
    current_sense_init();
#endif

    /* BLE 放在 weld 之后：一旦初始化完成就能接收远程触发，
     * 此时安全联锁已经就绪，不会出现"能连上但拦不住"的窗口。 */
    ble_init();

    xTaskCreate(ui_task, "ui", 4096, NULL, 5, NULL);

#if !CONFIG_BENCH_MODE && !CONFIG_SIM_WELD
    beep(60);
#endif

    ESP_LOGI(TAG, "初始化完成");
}
