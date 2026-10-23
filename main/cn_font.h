#pragma once

#include <stdint.h>

typedef struct {
    uint16_t code;      /* Unicode 码点 */
    uint8_t  bmp[32];   /* 16x16 点阵，行优先，每行 2 字节，高位在左 */
} cn_glyph_t;

extern const cn_glyph_t g_cn_glyphs[];
extern const int g_cn_glyph_count;
