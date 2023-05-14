#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int8_t s8;
typedef int16_t s16;
typedef int32_t s32;
typedef int64_t s64;

#define LCDC_OFFSET 0x6800000

#define REG_POWCNT1 *((u16 *)0x4000304)
#define REG_DISPCNT_A *((u32 *)0x4000000)
#define REG_VRAMCNT_A *((u8 *)0x4000240)

#define FB_WIDTH 256
#define FB_HEIGHT 384

#define BLACK 0x0
#define WHITE 0x7FFF
#define RED 0x1F
#define GREEN 0x3E0
#define BLUE 0x7C00

void print_string(const char *s, int row, int col, u16 color);
void print_char(char c, int row, int col, u16 color);

#endif
