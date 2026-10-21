#pragma once

/*
 * weld - 点焊时序与安全联锁
 *
 * 时序：触发 -> 200ms 预压 -> 拉高 GPIO10 -> 10ms -> 拉低
 * 保护：50ms 独立看门狗强制关断；1s 冷却锁定；NTC > 80℃ 禁止触发。
 *       全程不使用 vTaskDelay 轮询，脚踏/远程触发统一走任务通知。
 */

#include <stdint.h>

void weld_init(void);

/* 远程触发（BLE）。刻意复用与脚踏中断完全相同的通知路径，
 * 因此冷却、超温、防抖、看门狗全部照常生效，没有"远程快捷通道"。 */
void weld_request_trigger(void);

/* 设定导通脉宽，内部按 1ms ~ 50ms 钳位 */
void weld_set_pulse_us(uint32_t us);

uint32_t weld_get_pulse_us(void);
