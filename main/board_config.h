/*
 * board_config.h - ESP32-C3 点焊机控制板 硬件抽象层
 *
 * 严格对应《项目开发指南》的引脚分配表。
 * 标注【规格修正】的地方，是原始规格书与 ESP32-C3 实际硬件能力冲突、
 * 或与 ESP-IDF v6.1 实际 API 冲突之处，已按硬件真实能力处理。
 */
#pragma once

#include "sdkconfig.h"
#include "driver/gpio.h"
#include "hal/adc_types.h"

/* ============================ 引脚分配 ============================ */

/* GPIO0 / GPIO1：电流与温度，必须走 ADC1（C3 的 ADC2 只服务于 Wi-Fi） */
#define PIN_CURRENT_ADC     GPIO_NUM_0
#define PIN_NTC_ADC         GPIO_NUM_1

#define ADC_UNIT_USED       ADC_UNIT_1
#define ADC_CH_CURRENT      ADC_CHANNEL_0   /* GPIO0 -> ADC1_CH0 */
#define ADC_CH_NTC          ADC_CHANNEL_1   /* GPIO1 -> ADC1_CH1 */
#define ADC_ATTEN_USED      ADC_ATTEN_DB_12 /* 0~3.3V 量程 */
#define ADC_BITWIDTH_USED   ADC_BITWIDTH_12

/* GPIO2 / GPIO3 / GPIO21：EC11
 * 【规格修正】ESP32-C3 没有 PCNT 外设：soc_caps.h 里没有 SOC_PCNT_SUPPORTED，
 * esp_driver_pcnt 的 CMakeLists 在该目标下不会编译任何源码。
 * 所以 EC11 只能用 GPIO 双边沿中断 + 软件正交解码，pcnt_* API 在 C3 上不可用。
 */
#define PIN_EC11_A          GPIO_NUM_2
#define PIN_EC11_B          GPIO_NUM_3
#define PIN_EC11_SW         GPIO_NUM_21

/* GPIO4/5/6/7：ST7789 SPI（RES 与 BLK 硬连 3.3V，故无复位脚、无背光脚） */
#define PIN_LCD_SCLK        GPIO_NUM_4
#define PIN_LCD_MOSI        GPIO_NUM_5
#define PIN_LCD_CS          GPIO_NUM_6
#define PIN_LCD_DC          GPIO_NUM_7
#define PIN_LCD_RST         (-1)            /* 硬连 3.3V，不接复位 */

#define LCD_HOST            SPI2_HOST
#define LCD_H_RES           240
#define LCD_V_RES           240
#define LCD_PIXEL_CLOCK_HZ  (40 * 1000 * 1000)
#define LCD_CMD_BITS        8
#define LCD_PARAM_BITS      8

/* GPIO9：蜂鸣器（NPN 驱动）。
 * 注意：这是 C3 的 strapping 脚，复位瞬间必须为高，外部电路不能把它拉死。
 * 在 BENCH 模式下该脚被征用为 BOOT 按键输入，见文件末尾。 */
#define PIN_BUZZER          GPIO_NUM_9

/* GPIO10：点焊触发（光耦 + 图腾柱）。硬件必须 10k 下拉，防上电误触。 */
#define PIN_WELD_TRIG       GPIO_NUM_10

/* GPIO20：脚踏 / 焊件接触检测，低电平有效，内部上拉 */
#define PIN_FOOT            GPIO_NUM_20

/* ============================ 采样参数 ============================ */

/* 【规格修正】规格书要求 ~100kHz，但 C3 数字控制器上限是
 * SOC_ADC_SAMPLE_FREQ_THRES_HIGH = 83333 Hz，超限配置会直接失败。
 * 这里取 80kHz —— 注意这是"总"采样率，两通道交替后每通道 40kHz。 */
#define ADC_SAMPLE_FREQ_HZ  (80 * 1000)
#define ADC_CONV_FRAME_SIZE 512                  /* 字节，必须是 4 的整数倍 */
#define ADC_STORE_BUF_SIZE  (ADC_CONV_FRAME_SIZE * 8)

/* 包络抓取窗口：一次放电 10ms，留 20ms 余量 */
#define ENV_BUCKETS         240
#define ENV_CAPTURE_MS      20
#define ENV_CAPTURE_SAMPLES ((ADC_SAMPLE_FREQ_HZ / 2) * ENV_CAPTURE_MS / 1000)

/* 温度发布间隔 */
#define NTC_PUBLISH_MS      500

/* ============================ 安全参数 ============================ */

#define WELD_PRECHARGE_US   (200 * 1000)    /* 预压 200ms */
#define WELD_PULSE_US       (10 * 1000)     /* 标称导通 10ms */
#define WELD_MAX_PULSE_US   (50 * 1000)     /* 硬上限 50ms，独立看门狗 */
#define WELD_COOLDOWN_US    (1000 * 1000)   /* 冷却 1s */
#define WELD_TEMP_LIMIT_C   80.0f           /* 超温禁止触发 */
#define WELD_RETRIGGER_US   (50 * 1000)     /* 脚踏软件防抖 */

/* ============================ NTC 参数 ============================ */

#define NTC_R25_OHM         10000.0f
#define NTC_BETA            3950.0f
#define NTC_R_FIXED_OHM     10000.0f
#define NTC_VCC_MV          3300.0f

/* ============================ 电流标定 ============================ */

/* 罗氏线圈 + 积分器输出 -> ADC 计数 -> 安培 的换算系数。
 * 占位值，实测后用钳形表标定。 */
#define CURRENT_A_PER_COUNT 0.01f

/* ============================ 颜色 ============================ */

#define RGB565(r, g, b) ((uint16_t)((((r) & 0x1F) << 11) | (((g) & 0x3F) << 5) | ((b) & 0x1F)))
#define COLOR_BLACK   RGB565(0, 0, 0)
#define COLOR_WHITE   RGB565(31, 63, 31)
#define COLOR_GREY    RGB565(12, 24, 12)
#define COLOR_DKGREY  RGB565(6, 12, 6)
#define COLOR_RED     RGB565(31, 0, 0)
#define COLOR_GREEN   RGB565(0, 63, 0)
#define COLOR_CYAN    RGB565(0, 48, 31)
#define COLOR_YELLOW  RGB565(31, 63, 0)
#define COLOR_ORANGE  RGB565(31, 32, 0)

/* ====================== 裸板自测模式（BENCH） ======================
 * menuconfig: Component config -> 点焊机控制器 -> 裸板自测模式
 * 开启后借用 BOOT 按键（GPIO9）当触发源，GPIO9 不能再驱动蜂鸣器。 */
#if CONFIG_BENCH_MODE
#undef  PIN_FOOT
#define PIN_FOOT        GPIO_NUM_9              /* 板载 BOOT 键 */
#define PIN_BENCH_LED   ((gpio_num_t)CONFIG_BENCH_LED_GPIO)
#define BENCH_LED_ON    (CONFIG_BENCH_LED_ACTIVE_LOW ? 0 : 1)
#define BENCH_LED_OFF   (!BENCH_LED_ON)
#endif
