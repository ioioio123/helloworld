#pragma once

/*
 * weld - 点焊时序与安全联锁
 *
 * 时序：脚踏低电平 -> 200ms 预压 -> 拉高 GPIO10 -> 10ms -> 拉低
 * 保护：50ms 独立看门狗强制关断；1s 冷却锁定；NTC > 80℃ 禁止触发。
 *       全程不使用 vTaskDelay 轮询，脚踏走 GPIO 中断 + 任务通知。
 */

void weld_init(void);
