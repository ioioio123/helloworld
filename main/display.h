#pragma once

#include "esp_err.h"
#include "app_state.h"

/* 初始化 SPI 总线、面板 IO、ST7789 驱动。失败返回对应 esp_err_t。 */
esp_err_t display_init(void);

/* 按状态刷新界面（内部做了区域比对，只重画变化的部分）。 */
void display_render(const app_state_t *s);

/* 开机动画：96x96 四色鲸鱼娘，播放 2 轮 */
void display_boot_anim(void);
