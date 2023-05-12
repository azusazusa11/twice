#ifndef TWICE_IO_9
#define TWICE_IO_9

#include "nds/mem/io.h"

namespace twice {
void wramcnt_write(NDS *nds, u8 value);

inline u8
io9_read8(NDS *nds, u32 addr)
{
	switch (addr) {
	case 0x4000208:
		return nds->cpu[0]->IME;
	case 0x4000210:
		IO_READ8_FROM_32(0x4000210, nds->cpu[0]->IE);
	case 0x4000214:
		IO_READ8_FROM_32(0x4000214, nds->cpu[0]->IF);
	}

	fprintf(stderr, "nds9 io read 8 at %08X\n", addr);
	return 0;
}

inline u16
io9_read16(NDS *nds, u32 addr)
{
	switch (addr) {
	case 0x4000004:
		return nds->dispstat[0];
	case 0x4000130:
		return nds->keyinput;
	case 0x4000180:
		return nds->ipcsync[0];
	case 0x4000208:
		return nds->cpu[0]->IME;
	case 0x4000210:
		IO_READ16_FROM_32(0x4000210, nds->cpu[0]->IE);
	case 0x4000214:
		IO_READ16_FROM_32(0x4000214, nds->cpu[0]->IF);
	}

	fprintf(stderr, "nds9 io read 16 at %08X\n", addr);
	return 0;
}

inline u32
io9_read32(NDS *nds, u32 addr)
{
	switch (addr) {
	case 0x4000208:
		return nds->cpu[0]->IME;
	case 0x4000210:
		return nds->cpu[0]->IE;
	case 0x4000214:
		return nds->cpu[0]->IF;
	}

	fprintf(stderr, "nds9 io read 32 at %08X\n", addr);
	return 0;
}

inline void
io9_write8(NDS *nds, u32 addr, u8 value)
{
	switch (addr) {
	case 0x4000208:
		nds->cpu[0]->IME = value & 1;
		nds->cpu[0]->check_interrupt();
		break;
	default:
		fprintf(stderr, "nds9 io write 8 to %08X\n", addr);
	}
}

inline void
io9_write16(NDS *nds, u32 addr, u16 value)
{
	switch (addr) {
	case 0x4000180:
		nds->ipcsync[1] &= ~0xF;
		nds->ipcsync[1] |= value >> 8 & 0xF;
		nds->ipcsync[0] &= ~0x4F00;
		nds->ipcsync[0] |= value & 0x4F00;

		if ((value & (1 << 13)) && (nds->ipcsync[1] & (1 << 14))) {
			nds->cpu[1]->request_interrupt(16);
		}
		break;
	case 0x4000208:
		nds->cpu[0]->IME = value & 1;
		nds->cpu[0]->check_interrupt();
		break;
	default:
		fprintf(stderr, "nds9 io write 16 to %08X\n", addr);
	}
}

inline void
io9_write32(NDS *nds, u32 addr, u32 value)
{
	switch (addr) {
	case 0x4000208:
		nds->cpu[0]->IME = value & 1;
		nds->cpu[0]->check_interrupt();
		break;
	case 0x4000210:
		nds->cpu[0]->IE = value;
		nds->cpu[0]->check_interrupt();
		break;
	case 0x4000214:
		nds->cpu[0]->IF &= ~value;
		nds->cpu[0]->check_interrupt();
		break;
	default:
		fprintf(stderr, "nds9 io write 32 to %08X\n", addr);
	}
}

template <typename T>
T
io9_read(NDS *nds, u32 addr)
{
	if constexpr (sizeof(T) == 1) {
		return io9_read8(nds, addr);
	} else if constexpr (sizeof(T) == 2) {
		return io9_read16(nds, addr);
	} else {
		return io9_read32(nds, addr);
	}
}

template <typename T>
void
io9_write(NDS *nds, u32 addr, T value)
{
	if constexpr (sizeof(T) == 1) {
		io9_write8(nds, addr, value);
	} else if constexpr (sizeof(T) == 2) {
		io9_write16(nds, addr, value);
	} else {
		io9_write32(nds, addr, value);
	}
}

} // namespace twice

#endif
