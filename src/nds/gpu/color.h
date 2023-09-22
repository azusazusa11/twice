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

inline void
unpack_bgr555_3d(u16 color, u8 *color_out)
{
	u8 r = color & 0x1F;
	u8 g = color >> 5 & 0x1F;
	u8 b = color >> 10 & 0x1F;

	if (r != 0)
		r = (r << 1) + 1;
	if (g != 0)
		g = (g << 1) + 1;
	if (b != 0)
		b = (b << 1) + 1;

	color_out[0] = r;
	color_out[1] = g;
	color_out[2] = b;
}

inline void
unpack_bgr555_3d(u16 color, u8 *r_out, u8 *g_out, u8 *b_out)
{
	u8 r = color & 0x1F;
	u8 g = color >> 5 & 0x1F;
	u8 b = color >> 10 & 0x1F;

	if (r != 0)
		r = (r << 1) + 1;
	if (g != 0)
		g = (g << 1) + 1;
	if (b != 0)
		b = (b << 1) + 1;

	*r_out = r;
	*g_out = g;
	*b_out = b;
}

inline void
unpack_abgr1555_3d(u16 color, u8 *color_out)
{
	u8 r = color & 0x1F;
	u8 g = color >> 5 & 0x1F;
	u8 b = color >> 10 & 0x1F;
	u8 a = color >> 15 & 1;

	if (r != 0)
		r = (r << 1) + 1;
	if (g != 0)
		g = (g << 1) + 1;
	if (b != 0)
		b = (b << 1) + 1;
	if (a != 0)
		a = 31;

	color_out[0] = r;
	color_out[1] = g;
	color_out[2] = b;
	color_out[3] = a;
}

inline u32
pack_abgr5666(u8 *color)
{
	u32 r = color[0];
	u32 g = color[1];
	u32 b = color[2];
	u32 a = color[3];

	return a << 18 | b << 12 | g << 6 | r;
}

} // namespace twice

#endif
