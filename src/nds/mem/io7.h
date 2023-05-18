#ifndef TWICE_IO7_H
#define TWICE_IO7_H

#include "nds/mem/io.h"

namespace twice {

inline u8
io7_read8(NDS *nds, u32 addr)
{
	switch (addr) {
	default:
		IO_READ8_COMMON(1);
	case 0x4000240:
		return nds->vramstat;
	case 0x4000241:
		return nds->wramcnt;
	}
}

inline u16
io7_read16(NDS *nds, u32 addr)
{
	switch (addr) {
	default:
		IO_READ16_COMMON(1);
	}
}

inline u32
io7_read32(NDS *nds, u32 addr)
{
	switch (addr) {
	default:
		IO_READ32_COMMON(1);
	}
}

inline void
io7_write8(NDS *nds, u32 addr, u8 value)
{
	switch (addr) {
	default:
		IO_WRITE8_COMMON(1);
	case 0x4000301:
		switch (value >> 6 & 0x3) {
		case 2:
			nds->cpu[1]->halted = true;
			nds->cpu[1]->force_stop();
			break;
		default:
			fprintf(stderr, "unhandled haltcnt\n");
		}
		nds->haltcnt = value;
		break;
	}
}

inline void
io7_write16(NDS *nds, u32 addr, u16 value)
{
	switch (addr) {
	default:
		IO_WRITE16_COMMON(1);
	}
}

inline void
io7_write32(NDS *nds, u32 addr, u32 value)
{
	switch (addr) {
	default:
		IO_WRITE32_COMMON(1);
	}
}

} // namespace twice

#endif
