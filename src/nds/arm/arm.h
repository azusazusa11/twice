#ifndef TWICE_ARM_H
#define TWICE_ARM_H

#include "common/types.h"
#include "common/util.h"

namespace twice {

enum {
	MODE_SYS,
	MODE_FIQ,
	MODE_SVC,
	MODE_ABT,
	MODE_IRQ,
	MODE_UND,
	MODE_USR = 8
};

enum {
	USR_MODE_BITS = 0x10,
	FIQ_MODE_BITS = 0x11,
	IRQ_MODE_BITS = 0x12,
	SVC_MODE_BITS = 0x13,
	ABT_MODE_BITS = 0x17,
	UND_MODE_BITS = 0x1B,
	SYS_MODE_BITS = 0x1F,
};

struct nds_ctx;

struct arm_cpu {
	arm_cpu(nds_ctx *nds, int cpuid);

	u32 gpr[16]{};
	u32 bankedr[6][3]{};
	u32 fiqr[5]{};
	u32 cpsr = BIT(7) | BIT(6) | SYS_MODE_BITS;
	u32 opcode;
	u32 pipeline[2]{};
	u32 exception_base{};
	u32 mode{ MODE_SYS };

	u32 IME{};
	u32 IF{};
	u32 IE{};
	bool interrupt{};
	bool halted{};

	nds_ctx *nds{};
	int cpuid{};

	u64& target_cycles;
	u64& cycles;

	u32& pc() { return gpr[15]; }

	u32& spsr() { return bankedr[0][2]; }

	virtual void step() = 0;
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
	virtual void check_halted() = 0;
};

inline void
arm_set_thumb(arm_cpu *cpu, bool t)
{
	cpu->cpsr = (cpu->cpsr & ~BIT(5)) | (t << 5);
}

inline bool
arm_in_thumb(arm_cpu *cpu)
{
	return cpu->cpsr & BIT(5);
}

void run_cpu(arm_cpu *cpu);
void arm_switch_mode(arm_cpu *cpu, u32 new_mode);
void arm_on_cpsr_write(arm_cpu *cpu);
void arm_force_stop(arm_cpu *cpu);
void arm_check_interrupt(arm_cpu *cpu);
void arm_request_interrupt(arm_cpu *cpu, int bit);
void arm_do_irq(arm_cpu *cpu);
bool arm_check_cond(arm_cpu *cpu, u32 cond);

} // namespace twice

#endif
