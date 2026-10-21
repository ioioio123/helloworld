#include <string.h>
#include <math.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_adc/adc_continuous.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "soc/soc_caps.h"

#include "board_config.h"
#include "app_state.h"
#include "current_sense.h"
#include "weld.h"

static const char *TAG = "adc";

static adc_continuous_handle_t s_handle;
static volatile TaskHandle_t  s_task;
static adc_cali_handle_t      s_cali;

/* 抓取状态：arm() 由 esp_timer 回调设置，其余由 ADC 任务独占 */
static volatile bool s_armed;
static bool          s_capturing;
static uint32_t      s_sample_idx;
static uint64_t      s_sum_sq;
static uint32_t      s_n;
static uint32_t      s_capture_samples = ENV_CAPTURE_SAMPLES;
static uint32_t      s_window_us       = ENV_CAPTURE_MS * 1000;
static uint16_t      s_buckets[ENV_BUCKETS];

/* NTC 平均 */
static uint32_t s_ntc_sum;
static uint32_t s_ntc_cnt;
static int64_t  s_ntc_last_us;

/* ---------------------------------------------------------------- 温度 */

static float ntc_raw_to_c(uint32_t raw)
{
    int mv = 0;
    if (s_cali) {
        if (adc_cali_raw_to_voltage(s_cali, (int)raw, &mv) != ESP_OK) {
            return -273.0f;
        }
    } else {
        mv = (int)((raw * (uint32_t)NTC_VCC_MV) / 4095u);
    }

    float v = (float)mv;
    if (v <= 1.0f || v >= (NTC_VCC_MV - 1.0f)) {
        return -273.0f;             /* 开路 / 短路 */
    }
    /* NTC 接地、10k 上拉到 VCC */
    float r_ntc = NTC_R_FIXED_OHM * v / (NTC_VCC_MV - v);
    if (r_ntc <= 1.0f) {
        return -273.0f;
    }
    /* Beta 方程 */
    float inv_t = (1.0f / 298.15f) + (logf(r_ntc / NTC_R25_OHM) / NTC_BETA);
    return (1.0f / inv_t) - 273.15f;
}

static void publish_temperature(void)
{
    int64_t now = esp_timer_get_time();
    if (s_ntc_cnt == 0 || (now - s_ntc_last_us) < (NTC_PUBLISH_MS * 1000)) {
        return;
    }

    uint32_t avg_raw = s_ntc_sum / s_ntc_cnt;
    s_ntc_sum = 0;
    s_ntc_cnt = 0;
    s_ntc_last_us = now;

    float t = ntc_raw_to_c(avg_raw);

    /* 长时间没有有效读数就当作传感器故障，按超温处理最安全 */
    bool fault = (t < -50.0f || t > 300.0f);
    if (fault) {
        ESP_LOGW(TAG, "NTC 读数异常 raw=%" PRIu32 " t=%.1f (按超温处理)", avg_raw, t);
    }

    app_state_t *s = app_state_lock();
    if (!fault) {
        s->temp_c = t;
        s->temp_locked = (t > WELD_TEMP_LIMIT_C);
    } else {
        s->temp_locked = true;
    }
    app_state_unlock();
}

/* ---------------------------------------------------------------- 包络 */

static void finish_capture(void)
{
    uint16_t peak = 0;
    int peak_idx = 0;
    for (int i = 0; i < ENV_BUCKETS; i++) {
        if (s_buckets[i] > peak) {
            peak = s_buckets[i];
            peak_idx = i;
        }
    }

    app_state_t *s = app_state_lock();
    uint32_t rms = 0;
    if (s_n > 0) {
        rms = (uint32_t)(sqrtf((float)((double)s_sum_sq / (double)s_n)) + 0.5f);
    }
    memcpy(s->env, s_buckets, sizeof(s->env));
    s->env_rms = (uint16_t)rms;
    s->env_peak = peak;
    s->env_peak_idx = peak_idx;
    s->env_valid = true;
    s->env_seq++;
    s->env_span_us = s_window_us;
    app_state_unlock();

    s_capturing = false;
    s_armed = false;

    ESP_LOGI(TAG, "包络完成 peak=%u rms=%u @bucket %d", (unsigned)peak, (unsigned)rms, peak_idx);
}

static void feed_current(uint32_t raw)
{
    if (!s_armed) {
        return;
    }
    if (!s_capturing) {
        s_capturing = true;
        s_sample_idx = 0;
        memset(s_buckets, 0, sizeof(s_buckets));
        s_sum_sq = 0;
        s_n = 0;
    }

    /* 把第 s_sample_idx 个样本映射到 0..ENV_BUCKETS-1 */
    uint32_t idx = (uint32_t)(((uint64_t)s_sample_idx * ENV_BUCKETS) / s_capture_samples);
    if (idx >= ENV_BUCKETS) {
        idx = ENV_BUCKETS - 1;
    }
    s_sum_sq += (uint64_t)raw * raw;
    s_n++;

    if (raw > s_buckets[idx]) {
        s_buckets[idx] = (uint16_t)raw;
    }

    s_sample_idx++;
    if (s_sample_idx >= s_capture_samples) {
        finish_capture();
    }
}

static void feed_ntc(uint32_t raw)
{
    if (s_ntc_cnt < 200000u) {      /* 防止极端情况下溢出 */
        s_ntc_sum += raw;
        s_ntc_cnt++;
    }
}

/* ---------------------------------------------------------------- 驱动 */

static bool IRAM_ATTR on_conv_done(adc_continuous_handle_t handle,
                                   const adc_continuous_evt_data_t *edata,
                                   void *user_ctx)
{
    TaskHandle_t t = s_task;
    if (!t) {
        return false;
    }
    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(t, &woken);
    return (woken == pdTRUE);
}

static void adc_task(void *arg)
{
    s_task = xTaskGetCurrentTaskHandle();

    /* 512 字节 / 4 字节每样本 = 128 个样本 */
    static uint8_t raw[ADC_CONV_FRAME_SIZE];
    static adc_continuous_data_t parsed[ADC_CONV_FRAME_SIZE / SOC_ADC_DIGI_RESULT_BYTES];

    for (;;) {
        /* DMA 满一帧才唤醒，200ms 兜底防止任务看门狗 */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(200));

        for (;;) {
            uint32_t got = 0;
            esp_err_t err = adc_continuous_read(s_handle, raw, sizeof(raw), &got, 0);
            if (err != ESP_OK) {
                break;              /* ESP_ERR_TIMEOUT：当前没数据了 */
            }
            uint32_t cnt = 0;
            if (adc_continuous_parse_data(s_handle, raw, got, parsed, &cnt) != ESP_OK) {
                continue;
            }
            for (uint32_t i = 0; i < cnt; i++) {
                if (!parsed[i].valid) {
                    continue;
                }
                if (parsed[i].channel == ADC_CH_CURRENT) {
                    feed_current(parsed[i].raw_data);
                } else if (parsed[i].channel == ADC_CH_NTC) {
                    feed_ntc(parsed[i].raw_data);
                }
            }
        }

        publish_temperature();
    }
}

void current_sense_arm(void)
{
    /* 抓取窗口跟随当前脉宽：前后留余量，缩短窗口让曲线更"放大"，
     * 同时保证 48ms 的长脉宽也不会被截断。 */
    const uint32_t MARGIN_US = 2000;
    const uint32_t MIN_US    = 2000;
    const uint32_t MAX_US    = 60000;

    uint32_t win = weld_get_pulse_us() + MARGIN_US;
    if (win < MIN_US) { win = MIN_US; }
    if (win > MAX_US) { win = MAX_US; }

    s_window_us       = win;
    s_capture_samples = (uint32_t)((uint64_t)ADC_SAMPLE_FREQ_HZ * win / 2000000ULL);
    if (s_capture_samples == 0) { s_capture_samples = 1; }

    s_sample_idx = 0;
    s_capturing  = false;
    s_armed      = true;
}

void current_sense_init(void)
{
    adc_continuous_handle_cfg_t hdl_cfg = {
        .max_store_buf_size = ADC_STORE_BUF_SIZE,
        .conv_frame_size    = ADC_CONV_FRAME_SIZE,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&hdl_cfg, &s_handle));

    /* pattern 必须保持有效，驱动在 config 时会读取 */
    static adc_digi_pattern_config_t pattern[2] = {
        {
            .atten     = ADC_ATTEN_USED,
            .channel   = (ADC_CH_CURRENT & 0x7),
            .unit      = ADC_UNIT_USED,
            .bit_width = ADC_BITWIDTH_USED,
        },
        {
            .atten     = ADC_ATTEN_USED,
            .channel   = (ADC_CH_NTC & 0x7),
            .unit      = ADC_UNIT_USED,
            .bit_width = ADC_BITWIDTH_USED,
        },
    };

    adc_continuous_config_t cfg = {
        .pattern_num    = 2,
        .adc_pattern    = pattern,
        .sample_freq_hz = ADC_SAMPLE_FREQ_HZ,
        .conv_mode      = ADC_CONV_SINGLE_UNIT_1,
        /* .format 在 v6.1 已标记 deprecated，不再填写 */
    };
    ESP_ERROR_CHECK(adc_continuous_config(s_handle, &cfg));

    /* 曲线拟合校准（C3 支持），失败就退回线性换算 */
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT_USED,
        .chan     = ADC_CH_NTC,
        .atten    = ADC_ATTEN_USED,
        .bitwidth = ADC_BITWIDTH_USED,
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali) != ESP_OK) {
        ESP_LOGW(TAG, "ADC 校准不可用，温度改用线性换算");
        s_cali = NULL;
    }

    s_ntc_last_us = esp_timer_get_time();

    /* 先建任务再启动 ADC，避免丢失通知（任务入口会设置 s_task） */
    xTaskCreate(adc_task, "adc", 4096, NULL, 8, NULL);

    adc_continuous_evt_cbs_t cbs = {
        .on_conv_done = on_conv_done,
    };
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(s_handle, &cbs, NULL));
    ESP_ERROR_CHECK(adc_continuous_start(s_handle));

    ESP_LOGI(TAG, "adc_continuous 启动：总 %d Hz，两通道各 %d Hz",
             ADC_SAMPLE_FREQ_HZ, ADC_SAMPLE_FREQ_HZ / 2);
}
