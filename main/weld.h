#pragma once

/*
 * weld - 双脉冲点焊时序与安全联锁
 *
 * 时序：触发 -> 200ms 预压 -> 脉冲1(预热) -> 固定间隔 -> 脉冲2(焊接) -> 1s 冷却
 * 保护：单颗脉冲 50ms 看门狗；1s 冷却锁定；NTC > 80℃ 禁止触发。
 */

#include <stdint.h>

void weld_init(void);

/* 远程触发（BLE），与脚踏走同一条路径，安全联锁完全生效 */
void weld_request_trigger(void);

/* idx: 0 = 脉冲1(预热)，1 = 脉冲2(焊接)。内部按 1ms~48ms 钳位 */
void     weld_set_pulse_us(int idx, uint32_t us);
uint32_t weld_get_pulse_us(int idx);

/* 一次点焊的采集窗口：P1 + 间隔 + P2 + 余量 */
uint32_t weld_get_capture_us(void);
