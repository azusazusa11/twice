#ifndef TWICE_GPU_H
#define TWICE_GPU_H

#include "common/types.h"
#include "common/util.h"

namespace twice {

enum {
	VRAM_A,
	VRAM_B,
	VRAM_C,
	VRAM_D,
	VRAM_E,
	VRAM_F,
	VRAM_G,
	VRAM_H,
	VRAM_I,
	VRAM_NUM_BANKS,
};

enum VramSizes : u32 {
	VRAM_A_SIZE = 128_KiB,
	VRAM_A_MASK = 128_KiB - 1,
	VRAM_B_SIZE = 128_KiB,
	VRAM_B_MASK = 128_KiB - 1,
	VRAM_C_SIZE = 128_KiB,
	VRAM_C_MASK = 128_KiB - 1,
	VRAM_D_SIZE = 128_KiB,
	VRAM_D_MASK = 128_KiB - 1,
	VRAM_E_SIZE = 64_KiB,
	VRAM_E_MASK = 64_KiB - 1,
	VRAM_F_SIZE = 16_KiB,
	VRAM_F_MASK = 16_KiB - 1,
	VRAM_G_SIZE = 16_KiB,
	VRAM_G_MASK = 16_KiB - 1,
	VRAM_H_SIZE = 32_KiB,
	VRAM_H_MASK = 32_KiB - 1,
	VRAM_I_SIZE = 16_KiB,
	VRAM_I_MASK = 16_KiB - 1,
};

struct Gpu {
	u8 vram_a[VRAM_A_SIZE]{};
	u8 vram_b[VRAM_B_SIZE]{};
	u8 vram_c[VRAM_C_SIZE]{};
	u8 vram_d[VRAM_D_SIZE]{};
	u8 vram_e[VRAM_E_SIZE]{};
	u8 vram_f[VRAM_F_SIZE]{};
	u8 vram_g[VRAM_G_SIZE]{};
	u8 vram_h[VRAM_H_SIZE]{};
	u8 vram_i[VRAM_I_SIZE]{};

	u8 *bank_to_base_ptr[VRAM_NUM_BANKS]{ vram_a, vram_b, vram_c, vram_d,
		vram_e, vram_f, vram_g, vram_h, vram_i };

	u8 bank_to_page_mask[VRAM_NUM_BANKS]{ 7, 7, 7, 7, 3, 0, 0, 1, 0 };

	bool bank_mapped[VRAM_NUM_BANKS]{};

	u8 *lcdc_pt[64]{};
	u8 *abg_pt[32]{};
	u16 abg_bank[32]{};
	u8 *aobj_pt[16]{};
	u16 aobj_bank[16]{};
	u8 *bbg_pt[8]{};
	u16 bbg_bank[4]{};
	u8 *bobj_pt[8]{};
	u16 bobj_bank{};
	u8 *arm7_pt[2]{};
	u16 arm7_bank[2]{};

	u8 vramcnt[VRAM_NUM_BANKS]{};
};

} // namespace twice

#endif
