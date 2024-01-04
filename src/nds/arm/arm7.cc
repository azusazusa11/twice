#include "nds/arm/arm7.h"
#include "nds/arm/interpreter/lut.h"
#include "nds/mem/bus.h"

#include "libtwice/exception.h"

namespace twice {

static const u8 gba_rom_nonseq_timings[4] = { 10, 8, 6, 18 };
static const u8 gba_rom_seq_timings[2] = { 6, 4 };

void
arm7_direct_boot(arm7_cpu *cpu, u32 entry_addr)
{
	cpu->gpr[12] = entry_addr;
	cpu->gpr[13] = 0x0380FD80;
	cpu->gpr[14] = entry_addr;
	cpu->bankedr[arm_cpu::MODE_IRQ][0] = 0x0380FF80;
	cpu->bankedr[arm_cpu::MODE_SVC][0] = 0x0380FFC0;

	update_arm7_page_tables(cpu);
	cpu->arm_jump(entry_addr);
}

void
arm7_frame_start(arm7_cpu *cpu)
{
}

void
arm7_frame_end(arm7_cpu *cpu)
{
}

void
arm7_cpu::run()
{
	if (halted) {
		if (check_halted()) {
			*curr_cycles = *target_cycles;
			return;
		} else if (interrupt) {
			arm_do_irq(this);
		}
	}

	while (*curr_cycles < *target_cycles) {
		step();
	}
}

static bool check_cond(u32 cond, u32 cpsr);

void
arm7_cpu::step()
{
	if (cpsr & 0x20) {
		pc() += 2;
		opcode = pipeline[0];
		pipeline[0] = pipeline[1];
		pipeline[1] = fetch16s(pc());
		thumb_inst_lut[opcode >> 6 & 0x3FF](this);
	} else {
		pc() += 4;
		opcode = pipeline[0];
		pipeline[0] = pipeline[1];
		pipeline[1] = fetch32s(pc());

		u32 cond = opcode >> 28;
		if (cond == 0xE || check_cond(cond, cpsr)) {
			u32 op1 = opcode >> 20 & 0xFF;
			u32 op2 = opcode >> 4 & 0xF;
			arm_inst_lut[op1 << 4 | op2](this);
		} else {
			add_cycles_c();
		}
	}

	if (interrupt) {
		arm_do_irq(this);
	}

	*curr_cycles += cycles;
	cycles = 0;
	code_cycles = 0;
}

static bool
check_cond(u32 cond, u32 cpsr)
{
	return arm_cond_table[cond] & (1 << (cpsr >> 28));
}

void
arm7_cpu::arm_jump(u32 addr)
{
	pc() = addr + 4;
	pipeline[0] = fetch32n(addr);
	pipeline[1] = fetch32s(addr + 4);
	cycles += code_cycles;
}

void
arm7_cpu::thumb_jump(u32 addr)
{
	pc() = addr + 2;
	pipeline[0] = fetch16n(addr);
	pipeline[1] = fetch16s(addr + 2);
	cycles += code_cycles;
}

void
arm7_cpu::jump_cpsr(u32 addr)
{
	cpsr = spsr();
	arm_on_cpsr_write(this);

	if (cpsr & 0x20) {
		thumb_jump(addr & ~1);
	} else {
		arm_jump(addr & ~3);
	}
}

template <typename T>
static T
fetch(arm7_cpu *cpu, u32 addr)
{
	u8 *p = cpu->pt[addr >> arm_cpu::PAGE_SHIFT];
	if (p) {
		return readarr<T>(p, addr & arm_cpu::PAGE_MASK);
	}

	return bus7_read<T>(cpu->nds, addr);
}

template <typename T>
static T
load(arm7_cpu *cpu, u32 addr)
{
	u8 *p = cpu->pt[addr >> arm_cpu::PAGE_SHIFT];
	if (p) {
		return readarr<T>(p, addr & arm_cpu::PAGE_MASK);
	}

	return bus7_read<T>(cpu->nds, addr);
}

template <typename T>
static void
store(arm7_cpu *cpu, u32 addr, T value)
{
	u8 *p = cpu->pt[addr >> arm_cpu::PAGE_SHIFT];
	if (p) {
		writearr<T>(p, addr & arm_cpu::PAGE_MASK, value);
		return;
	}

	bus7_write<T>(cpu->nds, addr, value);
}

u32
arm7_cpu::fetch32n(u32 addr)
{
	code_cycles = timings[addr >> TIMING_PAGE_SHIFT][0];
	return fetch<u32>(this, addr);
}

u32
arm7_cpu::fetch32s(u32 addr)
{
	code_cycles += timings[addr >> TIMING_PAGE_SHIFT][1];
	return fetch<u32>(this, addr);
}

u16
arm7_cpu::fetch16n(u32 addr)
{
	code_cycles = timings[addr >> TIMING_PAGE_SHIFT][2];
	return fetch<u16>(this, addr);
}

u16
arm7_cpu::fetch16s(u32 addr)
{
	code_cycles += timings[addr >> TIMING_PAGE_SHIFT][3];
	return fetch<u16>(this, addr);
}

u32
arm7_cpu::load32(u32 addr, bool nonseq)
{
	return nonseq ? load32n(addr) : load32s(addr);
}

u32
arm7_cpu::load32n(u32 addr)
{
	data_cycles = timings[addr >> TIMING_PAGE_SHIFT][0];
	return load<u32>(this, addr);
}

u32
arm7_cpu::load32s(u32 addr)
{
	data_cycles += timings[addr >> TIMING_PAGE_SHIFT][1];
	return load<u32>(this, addr);
}

u16
arm7_cpu::load16n(u32 addr)
{
	data_cycles = timings[addr >> TIMING_PAGE_SHIFT][2];
	return load<u16>(this, addr);
}

u8
arm7_cpu::load8n(u32 addr)
{
	data_cycles = timings[addr >> TIMING_PAGE_SHIFT][2];
	return load<u8>(this, addr);
}

void
arm7_cpu::store32(u32 addr, u32 value, bool nonseq)
{
	nonseq ? store32n(addr, value) : store32s(addr, value);
}

void
arm7_cpu::store32n(u32 addr, u32 value)
{
	data_cycles = timings[addr >> TIMING_PAGE_SHIFT][0];
	store<u32>(this, addr, value);
}

void
arm7_cpu::store32s(u32 addr, u32 value)
{
	data_cycles += timings[addr >> TIMING_PAGE_SHIFT][1];
	store<u32>(this, addr, value);
}

void
arm7_cpu::store16n(u32 addr, u16 value)
{
	data_cycles = timings[addr >> TIMING_PAGE_SHIFT][2];
	store<u16>(this, addr, value);
}

void
arm7_cpu::store8n(u32 addr, u8 value)
{
	data_cycles = timings[addr >> TIMING_PAGE_SHIFT][2];
	store<u8>(this, addr, value);
}

u16
arm7_cpu::ldrh(u32 addr)
{
	return std::rotr(load16n(addr & ~1), (addr & 1) << 3);
}

s16
arm7_cpu::ldrsh(u32 addr)
{
	if (addr & 1) {
		return (s8)load8n(addr);
	} else {
		return load16n(addr);
	}
}

static void
update_arm7_page_table(nds_ctx *nds, u8 **pt, u8 (*tt)[4], u64 start, u64 end)
{
	for (u64 addr = start; addr < end; addr += arm_cpu::PAGE_SIZE) {
		u64 page = addr >> arm_cpu::PAGE_SHIFT;
		switch (addr >> 23) {
		case 0x20 >> 3:
		case 0x28 >> 3:
			pt[page] = &nds->main_ram[addr & MAIN_RAM_MASK];
			break;
		case 0x30 >> 3:
		{
			u8 *p = nds->shared_wram_p[1];
			pt[page] = &p[addr & nds->shared_wram_mask[1]];
			break;
		}
		case 0x38 >> 3:
			pt[page] = &nds->arm7_wram[addr & ARM7_WRAM_MASK];
			break;
		default:
			pt[page] = nullptr;
		}
	}

	for (u64 addr = start; addr < end;
			addr += arm7_cpu::TIMING_PAGE_SIZE) {
		u64 page = addr >> arm7_cpu::TIMING_PAGE_SHIFT;
		switch (addr >> 23) {
		case 0x20 >> 3:
			tt[page][0] = 9;
			tt[page][1] = 2;
			tt[page][2] = 8;
			tt[page][3] = 1;
			break;
		case 0x60 >> 3:
		case 0x68 >> 3:
			tt[page][0] = 2;
			tt[page][1] = 2;
			tt[page][2] = 1;
			tt[page][3] = 1;
			break;
		case 0x80 >> 3:
		case 0x88 >> 3:
		case 0x90 >> 3:
		case 0x98 >> 3:
			if (nds->gba_slot_cpu == 1) {
				u8 n16 = gba_rom_nonseq_timings
						[nds->exmem[1] >> 2 & 3];
				u8 s16 = gba_rom_seq_timings
						[nds->exmem[1] >> 4 & 1];
				tt[page][0] = n16 + s16;
				tt[page][1] = s16 + s16;
				tt[page][2] = n16;
				tt[page][3] = s16;
			} else {
				tt[page][0] = 1;
				tt[page][1] = 1;
				tt[page][2] = 1;
				tt[page][3] = 1;
			}
			break;
		default:
			tt[page][0] = 1;
			tt[page][1] = 1;
			tt[page][2] = 1;
			tt[page][3] = 1;
		}
	}
}

void
update_arm7_page_tables(arm7_cpu *cpu)
{
	update_arm7_page_table(
			cpu->nds, cpu->pt, cpu->timings, 0, 0x100000000);
}

} // namespace twice
