#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "board_config.h"

/* 各任务共享的运行状态。所有读写都必须经过 app_state_lock/unlock。 */
typedef struct {
    /* 点焊 */
    bool     welding;        /* 正在放电 */
    bool     temp_locked;    /* 超温锁定 */
    uint32_t pulse1_us;      /* 脉冲1（预热）设定 */
    uint32_t pulse2_us;      /* 脉冲2（焊接）设定 */
    uint32_t pulse1_meas_us; /* 脉冲1 实测 */
    uint32_t pulse2_meas_us; /* 脉冲2 实测 */
    int32_t  pulse_sel;      /* 当前调节项：0=P1 1=P2 */
    uint32_t cap_mv;         /* 电容组电压 mV（未接时为 0） */
    uint32_t weld_count;     /* 累计次数 */
    int32_t  cooldown_pct;   /* 0~100，冷却进度 */

    /* 电流包络 */
    uint16_t env[ENV_BUCKETS];
    bool     env_valid;
    uint16_t env_peak;
    uint16_t env_peak1;      /* 脉冲1（预热）峰值 */
    uint16_t env_rms;        /* 有效值 RMS，同样以 ADC 计数表示 */
    uint32_t env_span_us;    /* 本次包络覆盖的时间窗口（跟随脉宽自适应） */
    int      env_peak_idx;
    uint32_t env_seq;        /* 每次新包络 +1，供 UI 判断是否重绘 */

    /* 温度 */
    float    temp_c;

    /* EC11 */
    int32_t  knob;           /* 累计计数，2 计数 = 1 格 */
} app_state_t;

extern app_state_t g_state;

void         app_state_init(void);
app_state_t *app_state_lock(void);
void         app_state_unlock(void);
