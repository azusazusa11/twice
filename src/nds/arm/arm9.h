#ifndef TWICE_ARM9_H
#define TWICE_ARM9_H

#include "nds/arm/arm.h"

namespace twice {

struct arm9_cpu final : arm_cpu {
	u8 *fetch_pt[PAGE_TABLE_SIZE]{};
	u8 *load_pt[PAGE_TABLE_SIZE]{};
	u8 *store_pt[PAGE_TABLE_SIZE]{};

	enum : u32 {
		TIME_PAGE_SHIFT = 12,
		TIME_PAGE_SIZE = (u32)1 << TIME_PAGE_SHIFT,
		TIME_TABLE_SIZE = (u32)1 << (32 - TIME_PAGE_SHIFT),
	};

	/* n32 */
	u8 fetch_timings[TIME_TABLE_SIZE][4]{};
	/* n32 s32 n16 s16 */
	u8 load_timings[TIME_TABLE_SIZE][4]{};
	u8 store_timings[TIME_TABLE_SIZE][4]{};

	enum {
		ITCM_SIZE = 32_KiB,
		ITCM_MASK = 32_KiB - 1,
		DTCM_SIZE = 16_KiB,
		DTCM_MASK = 16_KiB - 1,
	};

	u8 itcm[ITCM_SIZE]{};
	u8 dtcm[DTCM_SIZE]{};

	u32 itcm_addr_mask{};
	u32 itcm_array_mask{};
	u32 dtcm_base{};
	u32 dtcm_addr_mask{};
	u32 dtcm_array_mask{};

	bool read_itcm{};
	bool write_itcm{};
	bool read_dtcm{};
	bool write_dtcm{};

	u32 ctrl_reg{ 0x78 };
	u32 dtcm_reg{};
	u32 itcm_reg{};

	void run() override;
	void step() override;
	void arm_jump(u32 addr) override;
	void thumb_jump(u32 addr) override;
	void jump_cpsr(u32 addr) override;
	u32 fetch32n(u32 addr);
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
		if ((IE & IF) && (IME & 1)) {
			halted &= ~CPU_HALT;
		}

		return halted;
	}

	void add_cycles_cdi() override
	{
		cycles = std::max(code_cycles, data_cycles);
	}

	void add_cycles_cd(u32 internal_cycles = 0) override
	{
		cycles = std::max(code_cycles, data_cycles) + internal_cycles;
	}
};

void arm9_direct_boot(arm9_cpu *gpu, u32 entry_addr);
void arm9_frame_start(arm9_cpu *cpu);
void arm9_frame_end(arm9_cpu *cpu);
void update_arm9_page_tables(arm9_cpu *cpu);
u32 cp15_read(arm9_cpu *cpu, u32 reg);
void cp15_write(arm9_cpu *cpu, u32 reg, u32 value);

} // namespace twice

#endif
