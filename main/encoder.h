#pragma once

#include <stdint.h>
#include <stdbool.h>

void    encoder_init(void);

/* 读取并清零自上次调用以来的格数（正 = 顺时针）。EC11 一格 2 次中断，已除 2。 */
int32_t encoder_take_steps(void);

/* 按键当前是否按下 */
bool    encoder_switch_pressed(void);

/* 读取并清零按键按下事件次数（含 20ms 消抖） */
int32_t encoder_take_clicks(void);
