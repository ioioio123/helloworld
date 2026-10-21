#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "board_config.h"
#include "app_state.h"
#include "current_sense.h"
#include "weld.h"

static const char *TAG = "weld";

enum {
    PH_IDLE = 0,
    PH_PRECHARGE,
    PH_ON,
    PH_COOLDOWN,
};

static esp_timer_handle_t   s_seq;      /* 时序主定时器，单次触发 */
static esp_timer_handle_t   s_guard;    /* 50ms 硬看门狗 */
static volatile int         s_phase = PH_IDLE;
static volatile TaskHandle_t s_task;
static int64_t              s_last_off_us;
static int64_t              s_last_trig_us;
static int64_t              s_on_us;    /* 拉高触发脚的时刻，用于实测脉宽 */

/* ---------------------------------------------------------------- ISR */

static void IRAM_ATTR foot_isr(void *arg)
{
    TaskHandle_t t = s_task;
    if (!t) {
        return;
    }
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(t, &woken);
    if (woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/* ---------------------------------------------------------------- 时序 */

static void seq_cb(void *arg)
{
    switch (s_phase) {
    case PH_PRECHARGE: {
        /* 预压结束，开始放电 */
        app_state_t *st = app_state_lock();
        st->welding    = true;
        st->pulse_us   = WELD_PULSE_US;
        app_state_unlock();

        current_sense_arm();            /* 先布好采集，再拉高，避免丢上升沿 */
        s_on_us = esp_timer_get_time();
        gpio_set_level(PIN_WELD_TRIG, 1);

        s_phase = PH_ON;
        esp_timer_start_once(s_guard, WELD_MAX_PULSE_US);
        esp_timer_start_once(s_seq, WELD_PULSE_US);
        break;
    }
    case PH_ON: {
        /* 正常关断 */
        gpio_set_level(PIN_WELD_TRIG, 0);
        esp_timer_stop(s_guard);

        int64_t off_us = esp_timer_get_time();
        int64_t actual = off_us - s_on_us;
        s_last_off_us  = off_us;
        s_phase        = PH_COOLDOWN;

        app_state_t *st = app_state_lock();
        st->welding     = false;
        st->weld_count++;
        st->pulse_us    = (uint32_t)actual;     /* 实测值，不是标称值 */
        app_state_unlock();

        ESP_LOGI(TAG, "关断：实测导通 %d us（标称 %d us，误差 %d us）",
                 (int)actual, WELD_PULSE_US, (int)(actual - WELD_PULSE_US));

        esp_timer_start_once(s_seq, WELD_COOLDOWN_US);
        break;
    }
    case PH_COOLDOWN:
        s_phase = PH_IDLE;
        break;
    default:
        gpio_set_level(PIN_WELD_TRIG, 0);
        s_phase = PH_IDLE;
        break;
    }
}

static void guard_cb(void *arg)
{
    /* 走到这里说明 10ms 关断没生效，无条件拉低 */
    gpio_set_level(PIN_WELD_TRIG, 0);

    app_state_t *st = app_state_lock();
    st->welding = false;
    app_state_unlock();

    esp_timer_stop(s_seq);
    s_last_off_us = esp_timer_get_time();
    s_phase = PH_COOLDOWN;
    esp_timer_start_once(s_seq, WELD_COOLDOWN_US);

    ESP_LOGE(TAG, "安全看门狗强制关断：导通时间超过 %d ms", WELD_MAX_PULSE_US / 1000);
}

/* ---------------------------------------------------------------- 任务 */

static void weld_task(void *arg)
{
    s_task = xTaskGetCurrentTaskHandle();

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int64_t now = esp_timer_get_time();

        /* 软件防抖 */
        if (now - s_last_trig_us < WELD_RETRIGGER_US) {
            continue;
        }
        s_last_trig_us = now;

        if (s_phase != PH_IDLE) {
            continue;
        }
        if (now - s_last_off_us < WELD_COOLDOWN_US) {
            ESP_LOGW(TAG, "冷却中（%d ms），忽略触发",
                     (int)((WELD_COOLDOWN_US - (now - s_last_off_us)) / 1000));
            continue;
        }

        bool locked;
        app_state_t *st = app_state_lock();
        locked = st->temp_locked;
        app_state_unlock();

        if (locked) {
            ESP_LOGW(TAG, "超温锁定（>%.0f℃），拒绝触发", WELD_TEMP_LIMIT_C);
            continue;
        }

        ESP_LOGI(TAG, "检测到脚踏，进入 200ms 预压");
        s_phase = PH_PRECHARGE;
        esp_timer_start_once(s_seq, WELD_PRECHARGE_US);
    }
}

/* ---------------------------------------------------------------- 初始化 */

void weld_init(void)
{
    /* 触发脚：上电先拉低，硬件另有 10k 下拉 */
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_WELD_TRIG),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out));
    gpio_set_level(PIN_WELD_TRIG, 0);

    /* 脚踏：上拉输入，下降沿中断。
     * BENCH 模式下 PIN_FOOT 被改写为 GPIO9（板载 BOOT 键）。 */
    gpio_config_t in = {
        .pin_bit_mask = (1ULL << PIN_FOOT),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_NEGEDGE,
    };
    ESP_ERROR_CHECK(gpio_config(&in));

    const esp_timer_create_args_t seq_args = {
        .callback = seq_cb,
        .name     = "weld_seq",
    };
    ESP_ERROR_CHECK(esp_timer_create(&seq_args, &s_seq));

    const esp_timer_create_args_t guard_args = {
        .callback = guard_cb,
        .name     = "weld_guard",
    };
    ESP_ERROR_CHECK(esp_timer_create(&guard_args, &s_guard));

    /* 先建任务（任务入口设置 s_task），再挂中断 */
    xTaskCreate(weld_task, "weld", 3072, NULL, 10, NULL);
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_FOOT, foot_isr, NULL));

    s_last_off_us = esp_timer_get_time() - WELD_COOLDOWN_US;

#if CONFIG_BENCH_MODE
    ESP_LOGW(TAG, "BENCH: 触发源 = 板载 BOOT 键 (GPIO%d)", PIN_FOOT);
#endif

    ESP_LOGI(TAG, "点焊控制就绪：预压 %dms / 导通 %dms / 上限 %dms / 冷却 %dms",
             WELD_PRECHARGE_US / 1000, WELD_PULSE_US / 1000,
             WELD_MAX_PULSE_US / 1000, WELD_COOLDOWN_US / 1000);
}
