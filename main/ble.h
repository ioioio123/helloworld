#pragma once

#include <stdint.h>

/*
 * ble - BLE 5.0 GATT 服务（ESP32-C3 只有 BLE，没有经典蓝牙）
 *
 * 服务 UUID: 1a2b3c4d-0001-4a5b-8c9d-0e1f2a3b4c5d
 *   0002  遥测   Notify  文本 "W,<peak>,<temp x10>,<pulse_us>,<count>,<status>\n"
 *   0003  包络   Notify  240 字节，每字节 0~255 对应 ADC 0~4095
 *   0004  控制   Write   "F"=触发  "P<us>"=设脉宽  "?"=回报配置
 */

void ble_init(void);

/* 由 UI/采集任务在包络更新后调用，把整条曲线推给手机 */
void ble_notify_envelope(const uint16_t *env, int buckets);
