#pragma once

#include <stdint.h>

#define WHALE_W      240
#define WHALE_H      240
#define WHALE_PIXELS (WHALE_W * WHALE_H)

/* 115200 字节：240x240 每像素 16 位 RGB565，无调色板 */
extern const uint16_t g_whale_frame[WHALE_PIXELS];
