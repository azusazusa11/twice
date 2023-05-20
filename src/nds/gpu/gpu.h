#ifndef TWICE_GPU_H
#define TWICE_GPU_H

#include "common/types.h"
#include "common/util.h"

namespace twice {

struct NDS;

struct Gpu2D {
	Gpu2D(NDS *nds);

	u32 *fb{};
	NDS *nds{};

	void draw_scanline(u16 scanline);
};

void gpu_on_scanline_start(NDS *nds);

} // namespace twice

#endif
