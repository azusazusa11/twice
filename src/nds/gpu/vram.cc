#include "nds/nds.h"

#include "libtwice/exception.h"

namespace twice {

u8 *
get_vram_ptr(nds_ctx *nds, int bank, int page)
{
	u8 *base = nds->vram.bank_to_base_ptr[bank];
	u8 mask = nds->vram.bank_to_page_mask[bank];

	return base + (page & mask) * 16_KiB;
}

void
map_lcdc(nds_ctx *nds, u8 *base, int start, int len)
{
	for (int i = start; i < start + len; i++, base += 16_KiB) {
		nds->vram.lcdc_pt[i] = base;
	}
}

void
unmap_lcdc(nds_ctx *nds, int start, int len)
{
	for (int i = start; i < start + len; i++) {
		nds->vram.lcdc_pt[i] = nullptr;
	}
}

void
map_abg(nds_ctx *nds, int bank, u8 *base, int start, int len)
{
	for (int i = start; i < start + len; i++, base += 16_KiB) {
		auto& mask = nds->vram.abg_bank[i];

		if (mask == 0) {
			nds->vram.abg_pt[i] = base;
		} else {
			nds->vram.abg_pt[i] = nullptr;
		}

		mask |= BIT(bank);
	}
}

void
unmap_abg(nds_ctx *nds, int bank, int start, int len)
{
	for (int i = start; i < start + len; i++) {
		auto& mask = nds->vram.abg_bank[i];
		mask &= ~BIT(bank);

		if (std::has_single_bit(mask)) {
			int only_bank = std::countr_zero(mask);
			nds->vram.abg_pt[i] = get_vram_ptr(nds, only_bank, i);
		} else {
			nds->vram.abg_pt[i] = nullptr;
		}
	}
}

void
map_aobj(nds_ctx *nds, int bank, u8 *base, int start, int len)
{
	for (int i = start; i < start + len; i++, base += 16_KiB) {
		auto& mask = nds->vram.aobj_bank[i];

		if (mask == 0) {
			nds->vram.aobj_pt[i] = base;
		} else {
			nds->vram.aobj_pt[i] = nullptr;
		}

		mask |= BIT(bank);
	}
}

void
unmap_aobj(nds_ctx *nds, int bank, int start, int len)
{
	for (int i = start; i < start + len; i++) {
		auto& mask = nds->vram.aobj_bank[i];
		mask &= ~BIT(bank);

		if (std::has_single_bit(mask)) {
			int only_bank = std::countr_zero(mask);
			nds->vram.aobj_pt[i] = get_vram_ptr(nds, only_bank, i);
		} else {
			nds->vram.aobj_pt[i] = nullptr;
		}
	}
}

void
map_bbg(nds_ctx *nds, int bank, u8 *base, int start, int len)
{
	for (int i = start; i < start + len; i += 2, base += 32_KiB) {
		auto& mask = nds->vram.bbg_bank[i >> 1];

		if (mask == 0) {
			nds->vram.bbg_pt[i] = base;
			if (bank == VRAM_I) {
				nds->vram.bbg_pt[i + 1] = base;
			} else {
				nds->vram.bbg_pt[i + 1] = base + 16_KiB;
			}
		} else {
			nds->vram.bbg_pt[i] = nullptr;
			nds->vram.bbg_pt[i + 1] = nullptr;
		}

		mask |= BIT(bank);
	}
}

void
unmap_bbg(nds_ctx *nds, int bank, int start, int len)
{
	for (int i = start; i < start + len; i += 2) {
		auto& mask = nds->vram.bbg_bank[i >> 1];
		mask &= ~BIT(bank);

		if (std::has_single_bit(mask)) {
			int only_bank = std::countr_zero(mask);
			u8 *p = get_vram_ptr(nds, only_bank, i);
			nds->vram.bbg_pt[i] = p;
			if (bank == VRAM_I) {
				nds->vram.bbg_pt[i + 1] = p;
			} else {
				nds->vram.bbg_pt[i + 1] = p + 16_KiB;
			}
		} else {
			nds->vram.bbg_pt[i] = nullptr;
			nds->vram.bbg_pt[i + 1] = nullptr;
		}
	}
}

void
map_bobj(nds_ctx *nds, int bank, u8 *base)
{
	auto& mask = nds->vram.bobj_bank;

	if (mask == 0) {
		if (bank == VRAM_D) {
			for (int i = 0; i < 8; i++) {
				nds->vram.bobj_pt[i] = base + i * 16_KiB;
			}
		} else {
			for (int i = 0; i < 8; i++) {
				nds->vram.bobj_pt[i] = base;
			}
		}
	} else {
		for (int i = 0; i < 8; i++) {
			nds->vram.bobj_pt[i] = nullptr;
		}
	}

	mask |= BIT(bank);
}

void
unmap_bobj(nds_ctx *nds, int bank)
{
	auto& mask = nds->vram.bobj_bank;
	mask &= ~BIT(bank);

	if (mask == BIT(VRAM_D)) {
		for (int i = 0; i < 8; i++) {
			nds->vram.bobj_pt[i] = nds->vram.vram_d + i * 16_KiB;
		}
	} else if (mask == BIT(VRAM_I)) {
		for (int i = 0; i < 8; i++) {
			nds->vram.bobj_pt[i] = nds->vram.vram_i;
		}
	} else {
		for (int i = 0; i < 8; i++) {
			nds->vram.bobj_pt[i] = nullptr;
		}
	}
}

void
map_arm7(nds_ctx *nds, int bank, u8 *base, int i)
{
	auto& mask = nds->vram.arm7_bank[i];

	if (mask == 0) {
		nds->vram.arm7_pt[i] = base;
	} else {
		nds->vram.arm7_pt[i] = nullptr;
	}

	mask |= BIT(bank);
}

void
unmap_arm7(nds_ctx *nds, int bank, int i)
{
	auto& mask = nds->vram.arm7_bank[i];
	mask &= ~BIT(bank);

	if (std::has_single_bit(mask)) {
		int only_bank = std::countr_zero(mask);
		nds->vram.arm7_pt[i] = nds->vram.bank_to_base_ptr[only_bank];
	} else {
		nds->vram.arm7_pt[i] = nullptr;
	}
}

void
map_abg_palette(nds_ctx *nds, int bank, u8 *base, int start, int len)
{
	for (int i = start; i < start + len; i++, base += 16_KiB) {
		auto& mask = nds->vram.abg_palette_bank[i];

		if (mask == 0) {
			nds->vram.abg_palette_pt[i] = base;
		} else {
			nds->vram.abg_palette_pt[i] = nullptr;
		}

		mask |= BIT(bank);
	}
}

void
unmap_abg_palette(nds_ctx *nds, int bank, int start, int len)
{
	for (int i = start; i < start + len; i++) {
		auto& mask = nds->vram.abg_palette_bank[i];
		mask &= ~BIT(bank);

		if (std::has_single_bit(mask)) {
			int only_bank = std::countr_zero(mask);
			nds->vram.abg_palette_pt[i] =
					get_vram_ptr(nds, only_bank, i);
		} else {
			nds->vram.abg_pt[i] = nullptr;
		}
	}
}

void
map_bbg_palette(nds_ctx *nds, u8 *base)
{
	nds->vram.bbg_palette_pt = base;
}

void
unmap_bbg_palette(nds_ctx *nds)
{
	nds->vram.bbg_palette_pt = nullptr;
}

template <int bank>
void
vramcnt_ab_write(nds_ctx *nds, u8 value)
{
	auto& vram = nds->vram;

	u8 old = vram.vramcnt[bank];

	if (value == old) {
		return;
	}

	u8 *base = vram.bank_to_base_ptr[bank];

	if (vram.bank_mapped[bank]) {
		u8 mst = old & 0x3;
		u8 ofs = old >> 3 & 0x3;

		switch (mst) {
		case 0:
			unmap_lcdc(nds, bank << 3, 8);
			break;
		case 1:
			unmap_abg(nds, bank, ofs << 3, 8);
			break;
		case 2:
			unmap_aobj(nds, bank, (ofs & 1) << 3, 8);
			break;
		case 3:
			throw twice_error("unmap vram bank a/b to texture");
		}

		vram.bank_mapped[bank] = false;
	}

	if (value & 0x80) {
		u8 mst = value & 0x3;
		u8 ofs = value >> 3 & 0x3;

		switch (mst) {
		case 0:
			map_lcdc(nds, base, bank << 3, 8);
			break;
		case 1:
			map_abg(nds, bank, base, ofs << 3, 8);
			break;
		case 2:
			map_aobj(nds, bank, base, (ofs & 1) << 3, 8);
			break;
		case 3:
			throw twice_error("map vram bank a/b to texture");
		}

		vram.bank_mapped[bank] = true;
	}

	vram.vramcnt[bank] = value;
}

template <int bank>
void
vramcnt_cd_write(nds_ctx *nds, u8 value)
{
	auto& vram = nds->vram;

	u8 old = vram.vramcnt[bank];

	if (value == old) {
		return;
	}

	u8 *base = vram.bank_to_base_ptr[bank];

	if (vram.bank_mapped[bank]) {
		u8 mst = old & 0x7;
		u8 ofs = old >> 3 & 0x3;

		switch (mst) {
		case 0:
			unmap_lcdc(nds, bank << 3, 8);
			break;
		case 1:
			unmap_abg(nds, bank, ofs << 3, 8);
			break;
		case 2:
			unmap_arm7(nds, bank, ofs & 1);
			break;
		case 3:
			throw twice_error("unmap vram bank c/d to texture");
		case 4:
			if (bank == VRAM_C) {
				unmap_bbg(nds, bank, 0, 8);
			} else {
				unmap_bobj(nds, bank);
			}
			break;
		}

		nds->vramstat &= ~BIT(bank - VRAM_C);

		vram.bank_mapped[bank] = false;
	}

	if (value & 0x80) {
		u8 mst = value & 0x7;
		u8 ofs = value >> 3 & 0x3;

		switch (mst) {
		case 0:
			map_lcdc(nds, base, bank << 3, 8);
			break;
		case 1:
			map_abg(nds, bank, base, ofs << 3, 8);
			break;
		case 2:
			map_arm7(nds, bank, base, ofs & 1);
			nds->vramstat |= BIT(bank - VRAM_C);
			break;
		case 3:
			throw twice_error("map vram bank c/d to texture");
		case 4:
			if (bank == VRAM_C) {
				map_bbg(nds, bank, base, 0, 8);
			} else {
				map_bobj(nds, bank, base);
			}
			break;
		default:
			throw twice_error("map vram bank c/d invalid mst");
		}

		vram.bank_mapped[bank] = true;
	}

	vram.vramcnt[bank] = value;
}

void
vramcnt_e_write(nds_ctx *nds, u8 value)
{
	auto& vram = nds->vram;

	u8 old = vram.vramcnt[VRAM_E];

	if (value == old) {
		return;
	}

	u8 *base = vram.bank_to_base_ptr[VRAM_E];

	if (vram.bank_mapped[VRAM_E]) {
		u8 mst = old & 0x7;

		switch (mst) {
		case 0:
			unmap_lcdc(nds, 0x20, 4);
			break;
		case 1:
			unmap_abg(nds, VRAM_E, 0, 4);
			break;
		case 2:
			unmap_aobj(nds, VRAM_E, 0, 4);
			break;
		case 3:
			throw twice_error("unmap vram bank e tex pal");
		case 4:
			unmap_abg_palette(nds, VRAM_E, 0, 2);
			break;
		}

		vram.bank_mapped[VRAM_E] = false;
	}

	if (value & 0x80) {
		u8 mst = value & 0x7;

		switch (mst) {
		case 0:
			map_lcdc(nds, base, 0x20, 4);
			break;
		case 1:
			map_abg(nds, VRAM_E, base, 0, 4);
			break;
		case 2:
			map_aobj(nds, VRAM_E, base, 0, 4);
			break;
		case 3:
			throw twice_error("map vram bank e tex pal");
		case 4:
			map_abg_palette(nds, VRAM_E, base, 0, 2);
			break;
		default:
			throw twice_error("map vram bank e invalid mst");
		}

		vram.bank_mapped[VRAM_E] = true;
	}

	vram.vramcnt[VRAM_E] = value;
}

template <int bank>
void
vramcnt_fg_write(nds_ctx *nds, u8 value)
{
	auto& vram = nds->vram;

	u8 old = vram.vramcnt[bank];

	if (value == old) {
		return;
	}

	u8 *base = vram.bank_to_base_ptr[bank];

	if (vram.bank_mapped[bank]) {
		u8 mst = old & 0x7;
		u8 ofs = old >> 3 & 0x3;
		u32 offset = (ofs & 1) | (ofs & 2) << 1;

		switch (mst) {
		case 0:
			map_lcdc(nds, base, 0x24 + bank - VRAM_F, 1);
			break;
		case 1:
			map_abg(nds, bank, base, offset, 1);
			map_abg(nds, bank, base, offset | 2, 1);
			break;
		case 2:
			map_aobj(nds, bank, base, offset, 1);
			map_aobj(nds, bank, base, offset | 2, 1);
			break;
		case 3:
			throw twice_error("unmap vram bank f/g tex pal");
		case 4:
			unmap_abg_palette(nds, bank, ofs & 1, 1);
			break;
		case 5:
			throw twice_error("unmap vram bank f/g aobj ext pal");
		}
		vram.bank_mapped[bank] = false;
	}

	if (value & 0x80) {
		u8 mst = value & 0x7;
		u8 ofs = value >> 3 & 0x3;
		u32 offset = (ofs & 1) | (ofs & 2) << 1;

		switch (mst) {
		case 0:
			map_lcdc(nds, base, 0x24 + bank - VRAM_F, 1);
			break;
		case 1:
			map_abg(nds, bank, base, offset, 1);
			map_abg(nds, bank, base, offset | 2, 1);
			break;
		case 2:
			map_aobj(nds, bank, base, offset, 1);
			map_aobj(nds, bank, base, offset | 2, 1);
			break;
		case 3:
			throw twice_error("map vram bank f/g tex pal");
		case 4:
			map_abg_palette(nds, bank, base, ofs & 1, 1);
			break;
		case 5:
			throw twice_error("map vram bank f/g aobj ext pal");
		default:
			throw twice_error("map vram bank f/g invalid mst");
		}

		vram.bank_mapped[bank] = true;
	}

	vram.vramcnt[bank] = value;
}

void
vramcnt_h_write(nds_ctx *nds, u8 value)
{
	auto& vram = nds->vram;

	u8 old = vram.vramcnt[VRAM_H];

	if (value == old) {
		return;
	}

	u8 *base = vram.bank_to_base_ptr[VRAM_H];

	if (vram.bank_mapped[VRAM_H]) {
		u8 mst = old & 0x3;

		switch (mst) {
		case 0:
			unmap_lcdc(nds, 0x26, 2);
			break;
		case 1:
			unmap_bbg(nds, VRAM_H, 0, 2);
			unmap_bbg(nds, VRAM_H, 4, 2);
			break;
		case 2:
			unmap_bbg_palette(nds);
			break;
		}

		vram.bank_mapped[VRAM_H] = false;
	}

	if (value & 0x80) {
		u8 mst = value & 0x3;

		switch (mst) {
		case 0:
			map_lcdc(nds, base, 0x26, 2);
			break;
		case 1:
			map_bbg(nds, VRAM_H, base, 0, 2);
			map_bbg(nds, VRAM_H, base, 4, 2);
			break;
		case 2:
			map_bbg_palette(nds, base);
			break;
		default:
			throw twice_error("map vram bank h invalid mst");
		}

		vram.bank_mapped[VRAM_H] = true;
	}

	vram.vramcnt[VRAM_H] = value;
}

void
vramcnt_i_write(nds_ctx *nds, u8 value)
{
	auto& vram = nds->vram;

	u8 old = vram.vramcnt[VRAM_I];

	if (value == old) {
		return;
	}

	u8 *base = vram.bank_to_base_ptr[VRAM_I];

	if (vram.bank_mapped[VRAM_I]) {
		u8 mst = old & 0x3;

		switch (mst) {
		case 0:
			unmap_lcdc(nds, 0x28, 1);
			break;
		case 1:
			unmap_bbg(nds, VRAM_I, 2, 2);
			unmap_bbg(nds, VRAM_I, 6, 2);
			break;
		case 2:
			unmap_bobj(nds, VRAM_I);
			break;
		case 3:
			throw twice_error("unmap vram bank i bobj ext pal");
		}

		vram.bank_mapped[VRAM_I] = false;
	}

	if (value & 0x80) {
		u8 mst = value & 0x3;

		switch (mst) {
		case 0:
			map_lcdc(nds, base, 0x28, 1);
			break;
		case 1:
			map_bbg(nds, VRAM_I, base, 2, 2);
			map_bbg(nds, VRAM_I, base, 6, 2);
			break;
		case 2:
			map_bobj(nds, VRAM_I, base);
			break;
		case 3:
			throw twice_error("map vram bank i bobj ext pal");
		}

		vram.bank_mapped[VRAM_I] = true;
	}

	vram.vramcnt[VRAM_I] = value;
}

void
vramcnt_a_write(nds_ctx *nds, u8 value)
{
	vramcnt_ab_write<VRAM_A>(nds, value);
}

void
vramcnt_b_write(nds_ctx *nds, u8 value)
{
	vramcnt_ab_write<VRAM_B>(nds, value);
}

void
vramcnt_c_write(nds_ctx *nds, u8 value)
{
	vramcnt_cd_write<VRAM_C>(nds, value);
}

void
vramcnt_d_write(nds_ctx *nds, u8 value)
{
	vramcnt_cd_write<VRAM_D>(nds, value);
}

void
vramcnt_f_write(nds_ctx *nds, u8 value)
{
	vramcnt_fg_write<VRAM_F>(nds, value);
}

void
vramcnt_g_write(nds_ctx *nds, u8 value)
{
	vramcnt_fg_write<VRAM_G>(nds, value);
}

} // namespace twice
