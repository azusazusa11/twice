#include "nds/arm/arm9.h"
#include "nds/arm/interpreter/lut.h"
#include "nds/mem/bus.h"

#include "libtwice/exception.h"

namespace twice {

Arm9::Arm9(NDS *nds)
	: Arm(nds)
{
	cpuid = 0;
}

void
Arm9::jump(u32 addr)
{
	if (in_thumb()) {
		thumb_jump(addr);
	} else {
		arm_jump(addr);
	}
}

void
Arm9::arm_jump(u32 addr)
{
	pc() = addr + 4;
	pipeline[0] = fetch32(addr);
	pipeline[1] = fetch32(addr + 4);
}

void
Arm9::thumb_jump(u32 addr)
{
	pc() = addr + 2;
	pipeline[0] = fetch16(addr);
	pipeline[1] = fetch16(addr + 2);
}

void
Arm9::jump_cpsr(u32 addr)
{
	cpsr = spsr();
	on_cpsr_write();

	if (in_thumb()) {
		thumb_jump(addr & ~1);
	} else {
		arm_jump(addr & ~3);
	}
}

template <typename T>
T
Arm9::fetch(u32 addr)
{
	if (read_itcm && 0 == (addr & itcm_addr_mask)) {
		return readarr<T>(itcm, addr & itcm_array_mask);
	}

	return bus9_read<T>(nds, addr);
}

template <typename T>
T
Arm9::load(u32 addr)
{
	if (read_itcm && 0 == (addr & itcm_addr_mask)) {
		return readarr<T>(itcm, addr & itcm_array_mask);
	}

	if (read_dtcm && dtcm_base == (addr & dtcm_addr_mask)) {
		return readarr<T>(dtcm, addr & dtcm_array_mask);
	}

	return bus9_read<T>(nds, addr);
}

template <typename T>
void
Arm9::store(u32 addr, T value)
{
	if (write_itcm && 0 == (addr & itcm_addr_mask)) {
		writearr<T>(itcm, addr & itcm_array_mask, value);
		return;
	}

	if (write_dtcm && dtcm_base == (addr & dtcm_addr_mask)) {
		writearr<T>(dtcm, addr & dtcm_array_mask, value);
		return;
	}

	bus9_write<T>(nds, addr, value);
}

u32
Arm9::fetch32(u32 addr)
{
	return fetch<u32>(addr);
}

u16
Arm9::fetch16(u32 addr)
{
	return fetch<u16>(addr);
}

u32
Arm9::load32(u32 addr)
{
	return load<u32>(addr);
}

u16
Arm9::load16(u32 addr)
{
	return load<u16>(addr);
}

u8
Arm9::load8(u32 addr)
{
	return load<u8>(addr);
}

void
Arm9::store32(u32 addr, u32 value)
{
	store<u32>(addr, value);
}

void
Arm9::store16(u32 addr, u16 value)
{
	store<u16>(addr, value);
}

void
Arm9::store8(u32 addr, u8 value)
{
	store<u8>(addr, value);
}

u16
Arm9::ldrh(u32 addr)
{
	return load16(addr & ~1);
}

s16
Arm9::ldrsh(u32 addr)
{
	return load16(addr & ~1);
}

u32
Arm9::cp15_read(u32 reg)
{
	switch (reg) {
	/* cache type register */
	case 0x001:
		return 0x0F0D2112;
	case 0x100:
		return ctrl_reg;
	/* protection unit */
	case 0x200:
	case 0x201:
	case 0x300:
	case 0x500:
	case 0x501:
	case 0x502:
	case 0x503:
	case 0x600:
	case 0x610:
	case 0x620:
	case 0x630:
	case 0x640:
	case 0x650:
	case 0x660:
	case 0x670:
		fprintf(stderr, "cp15 read %03X\n", reg);
		return 0;
	case 0x910:
		return dtcm_reg;
	case 0x911:
		return itcm_reg;
	default:
		fprintf(stderr, "cp15 read %03X\n", reg);
		throw TwiceError("unhandled cp15 read");
	}
}

static void
ctrl_reg_write(Arm9 *cpu, u32 value)
{
	u32 diff = cpu->ctrl_reg ^ value;
	u32 unhandled = (1 << 0) | (1 << 2) | (1 << 7) | (1 << 12) |
			(1 << 14) | (1 << 15);

	if (diff & unhandled) {
		fprintf(stderr, "unhandled bits in cp15 write %08X\n", value);
	}

	cpu->exception_base = value & (1 << 13) ? 0xFFFF0000 : 0;

	if (diff & (1 << 16 | 1 << 17)) {
		cpu->read_dtcm = (value & 1 << 16) && !(value & 1 << 17);
		cpu->write_dtcm = value & 1 << 16;
	}

	if (diff & (1 << 18 | 1 << 19)) {
		cpu->read_itcm = (value & 1 << 18) && !(value & 1 << 19);
		cpu->write_itcm = value & 1 << 18;
	}

	cpu->ctrl_reg = (cpu->ctrl_reg & ~0xFF085) | (value & 0xFF085);
}

void
Arm9::cp15_write(u32 reg, u32 value)
{
	switch (reg) {
	case 0x100:
		ctrl_reg_write(this, value);
		break;
	case 0x910:
	{
		u32 shift = std::clamp<u32>(value >> 1 & 0x1F, 3, 23);
		u32 mask = ((u64)512 << shift) - 1;
		dtcm_base = value & ~mask;
		dtcm_addr_mask = ~mask;
		dtcm_array_mask = mask & DTCM_MASK;
		dtcm_reg = value & 0xFFFFF03E;
		break;
	}
	case 0x911:
	{
		u32 shift = std::clamp<u32>(value >> 1 & 0x1F, 3, 23);
		u32 mask = ((u64)512 << shift) - 1;
		itcm_addr_mask = ~mask;
		itcm_array_mask = mask & ITCM_MASK;
		itcm_reg = value & 0x3E;
		break;
	}
	default:
		fprintf(stderr, "unhandled cp15 write to %03X\n", reg);
	}
}

} // namespace twice
