#ifndef TWICE_ARM_H
#define TWICE_ARM_H

#include "common/types.h"

namespace twice {

enum CpuMode {
	MODE_SYS,
	MODE_FIQ,
	MODE_SVC,
	MODE_ABT,
	MODE_IRQ,
	MODE_UND,
	MODE_USR = 8
};

enum CpuModeBits {
	USR_MODE_BITS = 0x10,
	FIQ_MODE_BITS = 0x11,
	IRQ_MODE_BITS = 0x12,
	SVC_MODE_BITS = 0x13,
	ABT_MODE_BITS = 0x17,
	UND_MODE_BITS = 0x1B,
	SYS_MODE_BITS = 0x1F,
};

struct NDS;

struct Arm {
	Arm(NDS *nds)
		: nds(nds)
	{
	}

	u32 gpr[16]{};
	u32 bankedr[6][3]{};
	u32 fiqr[5]{};
	u32 cpsr = (1 << 7) | (1 << 6) | SYS_MODE_BITS;
	u32 opcode;
	u32 pipeline[2]{};
	u32 exception_base{};

	NDS *nds{};
	int cpuid{};
	u32 mode{ MODE_SYS };

	u32& pc() { return gpr[15]; }

	u32& spsr() { return bankedr[0][2]; }

	bool in_privileged_mode() { return mode != MODE_USR; }

	bool in_sys_or_usr_mode()
	{
		return mode == MODE_SYS || mode == MODE_USR;
	}

	bool in_fiq_mode() { return mode == MODE_FIQ; }

	bool current_mode_has_spsr() { return !in_sys_or_usr_mode(); }

	bool in_thumb() { return cpsr & (1 << 5); }

	bool get_c() { return cpsr & (1 << 29); }

	void set_q(bool q) { cpsr = (cpsr & ~(1 << 27)) | (q << 27); }

	void set_t(bool t) { cpsr = (cpsr & ~(1 << 5)) | (t << 5); }

	void set_nz(bool n, bool z)
	{
		cpsr &= ~0xC0000000;
		cpsr |= n << 31;
		cpsr |= z << 30;
	}

	void set_nzc(bool n, bool z, bool c)
	{
		cpsr &= ~0xE0000000;
		cpsr |= n << 31;
		cpsr |= z << 30;
		cpsr |= c << 29;
	}

	void set_nzcv(bool n, bool z, bool c, bool v)
	{
		cpsr &= ~0xF0000000;
		cpsr |= n << 31;
		cpsr |= z << 30;
		cpsr |= c << 29;
		cpsr |= v << 28;
	}

	bool is_arm7() { return cpuid; }

	bool is_arm9() { return !is_arm7(); }

	virtual void jump(u32 addr) = 0;
	virtual void arm_jump(u32 addr) = 0;
	virtual void thumb_jump(u32 addr) = 0;
	virtual void jump_cpsr(u32 addr) = 0;
	virtual u32 fetch32(u32 addr) = 0;
	virtual u16 fetch16(u32 addr) = 0;
	virtual u32 load32(u32 addr) = 0;
	virtual u16 load16(u32 addr) = 0;
	virtual u8 load8(u32 addr) = 0;
	virtual void store32(u32 addr, u32 value) = 0;
	virtual void store16(u32 addr, u16 value) = 0;
	virtual void store8(u32 addr, u8 value) = 0;
	virtual u16 ldrh(u32 addr) = 0;
	virtual s16 ldrsh(u32 addr) = 0;

	void on_cpsr_write();
	void switch_mode(u32 new_mode);
	void swap_registers(u32 old_mode, u32 new_mode);
};

} // namespace twice

#endif
