#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "board_config.h"
#include "app_state.h"
#include "display.h"
#include "current_sense.h"
#include "encoder.h"
#include "weld.h"

static const char *TAG = "app";

#define BUZZER_ON_LEVEL   1
#define BUZZER_IDLE_LEVEL (!BUZZER_ON_LEVEL)

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

/* ------------------------------------------------------------ UI 任务 */

static void ui_task(void *arg)
{
    app_state_t snap;

    for (;;) {
        int32_t steps  = encoder_take_steps();
        int32_t clicks = encoder_take_clicks();
        (void)clicks;

        app_state_t *s = app_state_lock();
        if (steps) {
            s->knob += steps;
        }
        snap = *s;                      /* 加锁拷贝快照 */
        app_state_unlock();

        display_render(&snap);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

/* ------------------------------------------------------------ 入口 */

void app_main(void)
{
    ESP_LOGI(TAG, "ESP32-C3 点焊机控制器启动");

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

    /* 共享状态与屏幕 */
    app_state_init();
    ESP_ERROR_CHECK(display_init());

    /* GPIO 中断服务必须先安装，encoder/weld 都要用它 */
    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    encoder_init();
    weld_init();
    current_sense_init();

    xTaskCreate(ui_task, "ui", 4096, NULL, 5, NULL);

    beep(60);

    ESP_LOGI(TAG, "初始化完成");
}
