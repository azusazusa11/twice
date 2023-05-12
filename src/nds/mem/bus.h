#ifndef TWICE_BUS_H
#define TWICE_BUS_H

#include "nds/arm/arm.h"
#include "nds/mem/io.h"
#include "nds/mem/vram.h"
#include "nds/nds.h"

#include "common/util.h"

namespace twice {

template <typename T>
T
bus9_read(NDS *nds, u32 addr)
{
	T value = 0;
	bool undef = false;

	switch (addr >> 24) {
	case 0x2:
		value = readarr<T>(nds->main_ram, addr & MAIN_RAM_MASK);
		break;
	case 0x3:
		value = readarr<T>(nds->shared_wram_p[0],
				addr & nds->shared_wram_mask[0]);
		if (nds->shared_wram_mask[0] == 0) {
			fprintf(stderr, "nds9 read empty wram\n");
		}
		break;
	case 0x4:
		value = io_read<T, 0>(nds, addr);
		break;
	case 0x5:
		value = readarr<T>(nds->palette, addr & PALETTE_MASK);
		break;
	case 0x6:
		value = vram_read<T>(nds, addr);
		break;
	case 0x7:
		value = readarr<T>(nds->oam, addr & OAM_MASK);
		break;
	case 0xFF:
		if (addr < 0xFFFF0000) {
			undef = true;
		} else if (addr < 0xFFFF0000 + ARM9_BIOS_SIZE) {
			value = readarr<T>(
					nds->arm9_bios, addr & ARM9_BIOS_MASK);
		} else {
			value = 0;
		}
		break;
	default:
		undef = true;
	}

	if (undef) {
		fprintf(stderr, "undefined nds9 read at %08X\n", addr);
	}

	return value;
}

template <typename T>
void
bus9_write(NDS *nds, u32 addr, T value)
{
	bool undef = false;

	switch (addr >> 24) {
	case 0x2:
		writearr<T>(nds->main_ram, addr & MAIN_RAM_MASK, value);
		break;
	case 0x3:
		writearr<T>(nds->shared_wram_p[0],
				addr & nds->shared_wram_mask[0], value);
		if (nds->shared_wram_mask[0] == 0) {
			fprintf(stderr, "nds9 write empty wram\n");
		}
		break;
	case 0x4:
		io_write<T, 0>(nds, addr, value);
		break;
	case 0x5:
		writearr<T>(nds->palette, addr & PALETTE_MASK, value);
		break;
	case 0x6:
		vram_write<T>(nds, addr, value);
		break;
	case 0x7:
		writearr<T>(nds->oam, addr & OAM_MASK, value);
		break;
	case 0xFF:
		if (addr < 0xFFFF0000) {
			undef = true;
		}
		break;
	default:
		undef = true;
	}

	if (undef) {
		fprintf(stderr, "undefined nds9 write to %08X\n", addr);
	}
}

template <typename T>
T
bus7_read(NDS *nds, u32 addr)
{
	T value = 0;
	bool undef = false;

	switch (addr >> 23) {
	case 0x0:
		if (addr < ARM7_BIOS_SIZE) {
			if (nds->cpu[1]->gpr[15] >= ARM7_BIOS_SIZE) {
				value = -1;
			} else {
				value = readarr<T>(nds->arm7_bios, addr);
			}
		} else {
			undef = true;
		}
		break;
	case 0x20 >> 3:
	case 0x28 >> 3:
		value = readarr<T>(nds->main_ram, addr & MAIN_RAM_MASK);
		break;
	case 0x30 >> 3:
		value = readarr<T>(nds->shared_wram_p[1],
				addr & nds->shared_wram_mask[1]);
		break;
	case 0x38 >> 3:
		value = readarr<T>(nds->arm7_wram, addr & ARM7_WRAM_MASK);
		break;
	case 0x40 >> 3:
		value = io_read<T, 1>(nds, addr);
		break;
	case 0x60 >> 3:
	case 0x68 >> 3:
		value = vram_arm7_read<T>(nds, addr);
		break;
	default:
		undef = true;
	}

	if (undef) {
		fprintf(stderr, "undefined nds7 read from %08X\n", addr);
	}

	return value;
}

template <typename T>
void
bus7_write(NDS *nds, u32 addr, T value)
{
	bool undef = false;

	switch (addr >> 23) {
	case 0x0:
		if (addr >= ARM7_BIOS_SIZE) {
			undef = true;
		}
		break;
	case 0x20 >> 3:
	case 0x28 >> 3:
		writearr<T>(nds->main_ram, addr & MAIN_RAM_MASK, value);
		break;
	case 0x30 >> 3:
		writearr<T>(nds->shared_wram_p[1],
				addr & nds->shared_wram_mask[1], value);
		break;
	case 0x38 >> 3:
		writearr<T>(nds->arm7_wram, addr & ARM7_WRAM_MASK, value);
		break;
	case 0x40 >> 3:
		io_write<T, 1>(nds, addr, value);
		break;
	case 0x60 >> 3:
	case 0x68 >> 3:
		vram_arm7_write<T>(nds, addr, value);
		break;
	default:
		undef = true;
	}

	if (undef) {
		fprintf(stderr, "undefined nds7 write to %08X\n", addr);
	}
}

} // namespace twice

#endif
