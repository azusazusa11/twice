#ifndef TWICE_ARM7_H
#define TWICE_ARM7_H

#include "nds/arm/arm.h"

namespace twice {

struct arm7_cpu final : arm_cpu {
	u8 *pt[PAGE_TABLE_SIZE]{};

	enum : u32 {
		TIMING_PAGE_SHIFT = 15,
		TIMING_PAGE_SIZE = (u32)1 << TIMING_PAGE_SHIFT,
		TIMING_TABLE_SIZE = (u32)1 << (32 - TIMING_PAGE_SHIFT),
	};

	/* n32 s32 n16 s16: assume code / data are the same for now */
	u8 timings[TIMING_TABLE_SIZE][4]{};

	void run() override;
	void step() override;
	void arm_jump(u32 addr) override;
	void thumb_jump(u32 addr) override;
	void jump_cpsr(u32 addr) override;
	u32 fetch32n(u32 addr);
	u32 fetch32s(u32 addr);
	u16 fetch16n(u32 addr);
	u16 fetch16s(u32 addr);
	u32 load32(u32 addr, bool nonseq) override;
	u32 load32n(u32 addr) override;
	u32 load32s(u32 addr) override;
	u16 load16n(u32 addr) override;
	u8 load8n(u32 addr) override;
	void store32(u32 addr, u32 value, bool nonseq) override;
	void store32n(u32 addr, u32 value) override;
	void store32s(u32 addr, u32 value) override;
	void store16n(u32 addr, u16 value) override;
	void store8n(u32 addr, u8 value) override;
	u16 ldrh(u32 addr) override;
	s16 ldrsh(u32 addr) override;

	bool check_halted() override
	{
		if (IE & IF) {
			halted &= ~CPU_HALT;
		}

		return halted;
	}

	void add_cycles_cdi() override
	{
		cycles = code_cycles + data_cycles + 1;
	}

	void add_cycles_cd(u32 internal_cycles = 0) override
	{
		cycles = code_cycles + data_cycles + internal_cycles;
	}
};

void arm7_direct_boot(arm7_cpu *cpu, u32 entry_addr);
void arm7_frame_start(arm7_cpu *cpu);
void arm7_frame_end(arm7_cpu *cpu);
void update_arm7_page_tables(arm7_cpu *cpu);

} // namespace twice

#endif
