#ifndef TWICE_COLOR_H
#define TWICE_COLOR_H

#include "common/types.h"

namespace twice {

struct color6 {
	u8 r;
	u8 g;
	u8 b;
};

inline u32
bgr555_to_bgr888(u16 color)
{
	u32 r = color & 0x1F;
	u32 g = color >> 5 & 0x1F;
	u32 b = color >> 10 & 0x1F;

	r = (r * 527 + 23) >> 6;
	g = (g * 527 + 23) >> 6;
	b = (b * 527 + 23) >> 6;

	return b << 16 | g << 8 | r;
}

inline u32
color6_to_bgr888(color6 color)
{
	u32 r = (color.r * 259 + 33) >> 6;
	u32 g = (color.g * 259 + 33) >> 6;
	u32 b = (color.b * 259 + 33) >> 6;

	return b << 16 | g << 8 | r;
}

} // namespace twice

#endif
