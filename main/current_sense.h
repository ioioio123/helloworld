#pragma once

/*
 * current_sense - adc_continuous (DMA) 电流采集 + 分桶求最大值包络
 *
 * 说明：电流(GPIO0) 与 NTC(GPIO1) 共用 ADC1，用同一个 adc_continuous 句柄的
 * 双通道 pattern 交替采样。这样避免了 adc_oneshot 与 adc_continuous 争用同一个
 * ADC 单元（IDF 不允许两者同时占用同一 unit）。
 */

void current_sense_init(void);

/* 由点焊时序在拉高触发脚之前调用，开始抓取 20ms 窗口 */
void current_sense_arm(void);
