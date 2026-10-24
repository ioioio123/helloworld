#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "board_config.h"
#include "app_state.h"
#include "current_sense.h"
#include "weld.h"

#if CONFIG_SIM_WELD
#include <string.h>
#include <math.h>
#endif

static const char *TAG = "weld";

enum {
    PH_IDLE = 0,
    PH_PRECHARGE,
    PH_P1_ON,
    PH_GAP,
    PH_P2_ON,
    PH_COOLDOWN,
};

static esp_timer_handle_t   s_seq;
static esp_timer_handle_t   s_guard;
static volatile int         s_phase = PH_IDLE;
static volatile TaskHandle_t s_task;
static int64_t              s_last_off_us;
static int64_t              s_last_trig_us;
static int64_t              s_on_us;

static uint32_t             s_p1_us = WELD_P1_US;
static uint32_t             s_p2_us = WELD_P2_US;

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

/* ---------------------------------------------------------------- 脉冲 */

static void pulse_start(uint32_t us)
{
    s_on_us = esp_timer_get_time();
    gpio_set_level(PIN_WELD_TRIG, 1);
    esp_timer_start_once(s_guard, WELD_MAX_PULSE_US);
    esp_timer_start_once(s_seq, us);
}

static void pulse_stop(void)
{
    gpio_set_level(PIN_WELD_TRIG, 0);
    esp_timer_stop(s_guard);
}

/* ---------------------------------------------------------------- 时序 */

#if CONFIG_SIM_WELD
static void publish_sim_envelope(void);
#endif

static void seq_cb(void *arg)
{
    int64_t now = esp_timer_get_time();

    switch (s_phase) {
    case PH_PRECHARGE: {
        /* 预压结束，开始第一颗脉冲（预热） */
        app_state_t *st = app_state_lock();
        st->welding = true;
        app_state_unlock();

        current_sense_arm();
        ESP_LOGI(TAG, "脉冲1 开始 %d us", (int)s_p1_us);
        s_phase = PH_P1_ON;
        pulse_start(s_p1_us);
        break;
    }
    case PH_P1_ON: {
        pulse_stop();
        int64_t meas = now - s_on_us;

        app_state_t *st = app_state_lock();
        st->pulse1_meas_us = (uint32_t)meas;
        app_state_unlock();

        ESP_LOGI(TAG, "脉冲1 实测 %d us，间隔 %d us", (int)meas, (int)WELD_GAP_US);
        s_phase = PH_GAP;
        esp_timer_start_once(s_seq, WELD_GAP_US);
        break;
    }
    case PH_GAP: {
        ESP_LOGI(TAG, "脉冲2 开始 %d us", (int)s_p2_us);
        s_phase = PH_P2_ON;
        pulse_start(s_p2_us);
        break;
    }
    case PH_P2_ON: {
        pulse_stop();
        int64_t meas = now - s_on_us;

        s_last_off_us = now;
        s_phase      = PH_COOLDOWN;

        app_state_t *st = app_state_lock();
        st->welding        = false;
        st->pulse2_meas_us = (uint32_t)meas;
        st->weld_count++;
        app_state_unlock();

        ESP_LOGI(TAG, "脉冲2 实测 %d us", (int)meas);
#if CONFIG_SIM_WELD
        publish_sim_envelope();
#endif
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

#if CONFIG_SIM_WELD
/* 无电流传感器时合成双峰包络：脉冲1 -> 间隔归零 -> 脉冲2 */
static uint16_t sim_tri(uint32_t t, uint32_t width, uint16_t amp)
{
    if (width == 0 || t >= width) {
        return 0;
    }
    uint32_t d = (t < width / 2) ? t : (width - t);
    return (uint16_t)((uint64_t)amp * d * 2 / width);
}

static void publish_sim_envelope(void)
{
    const uint32_t win = s_p1_us + WELD_GAP_US + s_p2_us + 2000;
    const uint16_t amp1 = 1400;      /* 预热峰值（ADC 原始计数） */
    const uint16_t amp2 = 2600;      /* 焊接峰值 */
    const uint32_t p2_at = s_p1_us + WELD_GAP_US;

    app_state_t *st = app_state_lock();
    for (int i = 0; i < ENV_BUCKETS; i++) {
        uint32_t t = (uint32_t)((uint64_t)i * win / ENV_BUCKETS);
        uint16_t v = 0;
        if (t < s_p1_us) {
            v = sim_tri(t, s_p1_us, amp1);
        } else if (t >= p2_at && t < p2_at + s_p2_us) {
            v = sim_tri(t - p2_at, s_p2_us, amp2);
        }
        st->env[i] = v;
    }
    st->env_peak   = amp2;
    st->env_peak1  = amp1;
    st->env_rms    = (uint16_t)(amp2 * 0.62f);
    st->env_valid  = true;
    st->env_seq++;
    st->env_span_us = win;
    app_state_unlock();
}
#endif

static void guard_cb(void *arg)
{
    /* 单颗脉冲超时：无条件关断 */
    gpio_set_level(PIN_WELD_TRIG, 0);
    esp_timer_stop(s_seq);

    s_last_off_us = esp_timer_get_time();
    s_phase = PH_COOLDOWN;
    esp_timer_start_once(s_seq, WELD_COOLDOWN_US);

    app_state_t *st = app_state_lock();
    st->welding = false;
    app_state_unlock();

    ESP_LOGE(TAG, "安全看门狗强制关断：单颗脉冲超过 %d ms", WELD_MAX_PULSE_US / 1000);
}

/* ---------------------------------------------------------------- 任务 */

static void weld_task(void *arg)
{
    s_task = xTaskGetCurrentTaskHandle();

    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int64_t now = esp_timer_get_time();

        if (now - s_last_trig_us < WELD_RETRIGGER_US) {
            continue;
        }
        s_last_trig_us = now;

        if (s_phase != PH_IDLE && s_phase != PH_COOLDOWN) {
            ESP_LOGW(TAG, "正在放电中（phase=%d），忽略重复触发", s_phase);
            continue;
        }

        if (s_phase == PH_COOLDOWN || (now - s_last_off_us) < WELD_COOLDOWN_US) {
            int64_t left_us = WELD_COOLDOWN_US - (now - s_last_off_us);
            if (left_us < 0) {
                left_us = 0;
            }
            ESP_LOGW(TAG, "冷却中（剩余 %d ms），忽略触发", (int)(left_us / 1000));
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

        ESP_LOGI(TAG, "触发：预压 %dms -> 脉冲1 %dus -> 间隔 %dus -> 脉冲2 %dus",
                 WELD_PRECHARGE_US / 1000, (int)s_p1_us, (int)WELD_GAP_US, (int)s_p2_us);
        s_phase = PH_PRECHARGE;
        esp_timer_start_once(s_seq, WELD_PRECHARGE_US);
    }
}

/* ---------------------------------------------------------------- 对外 */

void weld_request_trigger(void)
{
    TaskHandle_t t = s_task;
    if (t) {
        xTaskNotifyGive(t);
    }
}

void weld_set_pulse_us(int idx, uint32_t us)
{
    const uint32_t lo = 1000;
    const uint32_t hi = WELD_MAX_PULSE_US - 2000;

    if (us < lo) {
        us = lo;
    }
    if (us > hi) {
        us = hi;
    }
    if (idx == 0) {
        s_p1_us = us;
    } else {
        s_p2_us = us;
    }
}

uint32_t weld_get_pulse_us(int idx)
{
    return (idx == 0) ? s_p1_us : s_p2_us;
}

uint32_t weld_get_capture_us(void)
{
    /* 采集窗口覆盖两颗脉冲 + 中间间隔 + 余量 */
    return s_p1_us + WELD_GAP_US + s_p2_us + 2000;
}

/* ---------------------------------------------------------------- 初始化 */

void weld_init(void)
{
    gpio_config_t out = {
        .pin_bit_mask = (1ULL << PIN_WELD_TRIG),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&out));
    gpio_set_level(PIN_WELD_TRIG, 0);

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

    xTaskCreate(weld_task, "weld", 3072, NULL, 10, NULL);
    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_FOOT, foot_isr, NULL));

    s_last_off_us = esp_timer_get_time() - WELD_COOLDOWN_US;

#if CONFIG_BENCH_MODE
    ESP_LOGW(TAG, "BENCH: 触发源 = 板载 BOOT 键 (GPIO%d)", PIN_FOOT);
#endif

    ESP_LOGI(TAG, "双脉冲就绪：预压 %dms / P1 %dus / 间隔 %dus / P2 %dus / 上限 %dms / 冷却 %dms",
             WELD_PRECHARGE_US / 1000, (int)s_p1_us, (int)WELD_GAP_US,
             (int)s_p2_us, WELD_MAX_PULSE_US / 1000, WELD_COOLDOWN_US / 1000);
}
