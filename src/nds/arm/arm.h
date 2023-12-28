#ifndef TWICE_ARM_H
#define TWICE_ARM_H

#include "common/types.h"
#include "common/util.h"

namespace twice {

struct nds_ctx;

struct arm_cpu {
	virtual ~arm_cpu();

	enum cpu_mode {
		MODE_SYS,
		MODE_FIQ,
		MODE_SVC,
		MODE_ABT,
		MODE_IRQ,
		MODE_UND,
		MODE_USR = 8
	};

	enum halt_mode {
		CPU_HALT = 0x1,
		CPU_GXFIFO_HALT = 0x2,
		CPU_STOP = 0x4,
	};

	enum : u32 {
		PAGE_SHIFT = 14,
		PAGE_SIZE = (u32)1 << PAGE_SHIFT,
		PAGE_MASK = PAGE_SIZE - 1,
		PAGE_TABLE_SIZE = (u32)1 << (32 - PAGE_SHIFT),
	};

	u32 gpr[16]{};
	u32 bankedr[6][3]{};
	u32 fiqr[5]{};
	u32 cpsr{ 0x9F };
	u32 opcode;
	u32 pipeline[2]{};
	u32 exception_base{};
	u32 mode{ MODE_SYS };

	u32 IME{};
	u32 IF{};
	u32 IE{};
	bool interrupt{};
	u32 halted{};

	nds_ctx *nds{};
	int cpuid{};

	timestamp *curr_cycles{};
	timestamp *target_cycles{};
	u32 code_cycles{};
	u32 data_cycles{};
	u32 cycles{};

	u32& pc() { return gpr[15]; }

	u32& spsr() { return bankedr[0][2]; }

	virtual void run() = 0;
	virtual void step() = 0;
	virtual void arm_jump(u32 addr) = 0;
	virtual void thumb_jump(u32 addr) = 0;
	virtual void jump_cpsr(u32 addr) = 0;
	virtual u32 load32(u32 addr, bool nonseq) = 0;
	virtual u32 load32n(u32 addr) = 0;
	virtual u32 load32s(u32 addr) = 0;
	virtual u16 load16n(u32 addr) = 0;
	virtual u8 load8n(u32 addr) = 0;
	virtual void store32(u32 addr, u32 value, bool nonseq) = 0;
	virtual void store32n(u32 addr, u32 value) = 0;
	virtual void store32s(u32 addr, u32 value) = 0;
	virtual void store16n(u32 addr, u16 value) = 0;
	virtual void store8n(u32 addr, u8 value) = 0;
	virtual u16 ldrh(u32 addr) = 0;
	virtual s16 ldrsh(u32 addr) = 0;
	virtual bool check_halted() = 0;

	void add_cycles_c(u32 internal_cycles = 0)
	{
		cycles = code_cycles + internal_cycles;
	}

	virtual void add_cycles_cdi() = 0;
	virtual void add_cycles_cd(u32 internal_cycles = 0) = 0;
};

void arm_init(nds_ctx *nds, int cpuid);
void arm_switch_mode(arm_cpu *cpu, u32 new_mode);
void arm_check_interrupt(arm_cpu *cpu);
void arm_on_cpsr_write(arm_cpu *cpu);
void arm_do_irq(arm_cpu *cpu);
void halt_cpu(arm_cpu *cpu, int halt_bits);
void unhalt_cpu(arm_cpu *cpu, int halt_bits);
void request_interrupt(arm_cpu *cpu, int bit);

extern const u16 arm_cond_table[16];

} // namespace twice

#endif
