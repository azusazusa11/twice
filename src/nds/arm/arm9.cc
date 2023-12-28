#include "nds/arm/arm9.h"

#include "nds/arm/interpreter/lut.h"
#include "nds/mem/bus.h"

#include "common/logger.h"
#include "libtwice/exception.h"

namespace twice {

static const u8 gba_rom_nonseq_timings[4] = { 26, 22, 18, 42 };
static const u8 gba_rom_seq_timings[2] = { 12, 8 };

void
arm9_direct_boot(arm9_cpu *cpu, u32 entry_addr)
{
	cp15_write(cpu, 0x100, 0x00012078);
	cp15_write(cpu, 0x910, 0x0300000A);
	cp15_write(cpu, 0x911, 0x00000020);

	cpu->gpr[12] = entry_addr;
	cpu->gpr[13] = 0x03002F7C;
	cpu->gpr[14] = entry_addr;
	cpu->bankedr[arm_cpu::MODE_IRQ][0] = 0x03003F80;
	cpu->bankedr[arm_cpu::MODE_SVC][0] = 0x03003FC0;

	update_arm9_page_tables(cpu);
	cpu->arm_jump(entry_addr);
}

void
arm9_frame_start(arm9_cpu *cpu)
{
}

void
arm9_frame_end(arm9_cpu *cpu)
{
}

void
arm9_cpu::run()
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
arm9_cpu::step()
{
	if (cpsr & 0x20) {
		pc() += 2;
		opcode = pipeline[0] & 0xFFFF;
		pipeline[0] = pipeline[1];
		if (pc() & 2) {
			pipeline[1] >>= 16;
			code_cycles = 0;
		} else {
			pipeline[1] = fetch32n(pc());
		}
		thumb_inst_lut[opcode >> 6 & 0x3FF](this);
	} else {
		pc() += 4;
		opcode = pipeline[0];
		pipeline[0] = pipeline[1];
		pipeline[1] = fetch32n(pc());

		u32 cond = opcode >> 28;
		if (cond == 0xE || check_cond(cond, cpsr)) {
			u32 op1 = opcode >> 20 & 0xFF;
			u32 op2 = opcode >> 4 & 0xF;
			arm_inst_lut[op1 << 4 | op2](this);
		} else if ((opcode & 0xFE000000) == 0xFA000000) {
			bool H = opcode & BIT(24);
			s32 offset = ((s32)(opcode << 8) >> 6) + (H << 1);

			gpr[14] = pc() - 4;
			cpsr |= 0x20;
			add_cycles_c();
			thumb_jump(pc() + offset);
		} else {
			add_cycles_c();
		}
	}

	if (interrupt) {
		arm_do_irq(this);
	}

	*curr_cycles += cycles;
	cycles = 0;
}

static bool
check_cond(u32 cond, u32 cpsr)
{
	return arm_cond_table[cond] & (1 << (cpsr >> 28));
}

void
arm9_cpu::arm_jump(u32 addr)
{
	pc() = addr + 4;
	pipeline[0] = fetch32n(addr);
	cycles += code_cycles;
	pipeline[1] = fetch32n(addr + 4);
	cycles += code_cycles;
}

void
arm9_cpu::thumb_jump(u32 addr)
{
	pc() = addr + 2;

	if (addr & 2) {
		pipeline[0] = fetch32n(addr - 2) >> 16;
		cycles += code_cycles;
		pipeline[1] = fetch32n(addr + 2);
		cycles += code_cycles;
	} else {
		u32 opc = fetch32n(addr);
		cycles += code_cycles;
		pipeline[0] = opc & 0xFFFF;
		pipeline[1] = opc >> 16;
	}
}

void
arm9_cpu::jump_cpsr(u32 addr)
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
fetch(arm9_cpu *cpu, u32 addr)
{
	u8 *p = cpu->fetch_pt[addr >> arm_cpu::PAGE_SHIFT];
	if (p) {
		return readarr<T>(p, addr & arm_cpu::PAGE_MASK);
	}

	if (cpu->read_itcm && 0 == (addr & cpu->itcm_addr_mask)) {
		return readarr<T>(cpu->itcm, addr & cpu->itcm_array_mask);
	}

	return bus9_read<T>(cpu->nds, addr);
}

template <typename T>
static T
load(arm9_cpu *cpu, u32 addr)
{
	u8 *p = cpu->load_pt[addr >> arm_cpu::PAGE_SHIFT];
	if (p) {
		return readarr<T>(p, addr & arm_cpu::PAGE_MASK);
	}

	if (cpu->read_itcm && 0 == (addr & cpu->itcm_addr_mask)) {
		return readarr<T>(cpu->itcm, addr & cpu->itcm_array_mask);
	}

	if (cpu->read_dtcm && cpu->dtcm_base == (addr & cpu->dtcm_addr_mask)) {
		return readarr<T>(cpu->dtcm, addr & cpu->dtcm_array_mask);
	}

	return bus9_read<T>(cpu->nds, addr);
}

template <typename T>
static void
store(arm9_cpu *cpu, u32 addr, T value)
{
	u8 *p = cpu->store_pt[addr >> arm_cpu::PAGE_SHIFT];
	if (p) {
		writearr<T>(p, addr & arm_cpu::PAGE_MASK, value);
		return;
	}

	if (cpu->write_itcm && 0 == (addr & cpu->itcm_addr_mask)) {
		writearr<T>(cpu->itcm, addr & cpu->itcm_array_mask, value);
		return;
	}

	if (cpu->write_dtcm &&
			cpu->dtcm_base == (addr & cpu->dtcm_addr_mask)) {
		writearr<T>(cpu->dtcm, addr & cpu->dtcm_array_mask, value);
		return;
	}

	bus9_write<T>(cpu->nds, addr, value);
}

u32
arm9_cpu::fetch32n(u32 addr)
{
	code_cycles = fetch_timings[addr >> TIME_PAGE_SHIFT][0];
	return fetch<u32>(this, addr);
}

u32
arm9_cpu::load32(u32 addr, bool nonseq)
{
	return nonseq ? load32n(addr) : load32s(addr);
}

u32
arm9_cpu::load32n(u32 addr)
{
	data_cycles = load_timings[addr >> TIME_PAGE_SHIFT][0];
	return load<u32>(this, addr);
}

u32
arm9_cpu::load32s(u32 addr)
{
	data_cycles += load_timings[addr >> TIME_PAGE_SHIFT][1];
	return load<u32>(this, addr);
}

u16
arm9_cpu::load16n(u32 addr)
{
	data_cycles = load_timings[addr >> TIME_PAGE_SHIFT][2];
	return load<u16>(this, addr);
}

u8
arm9_cpu::load8n(u32 addr)
{
	data_cycles = load_timings[addr >> TIME_PAGE_SHIFT][2];
	return load<u8>(this, addr);
}

void
arm9_cpu::store32(u32 addr, u32 value, bool nonseq)
{
	nonseq ? store32n(addr, value) : store32s(addr, value);
}

void
arm9_cpu::store32n(u32 addr, u32 value)
{
	data_cycles = store_timings[addr >> TIME_PAGE_SHIFT][0];
	store<u32>(this, addr, value);
}

void
arm9_cpu::store32s(u32 addr, u32 value)
{
	data_cycles += store_timings[addr >> TIME_PAGE_SHIFT][1];
	store<u32>(this, addr, value);
}

void
arm9_cpu::store16n(u32 addr, u16 value)
{
	data_cycles = store_timings[addr >> TIME_PAGE_SHIFT][2];
	store<u16>(this, addr, value);
}

void
arm9_cpu::store8n(u32 addr, u8 value)
{
	data_cycles = store_timings[addr >> TIME_PAGE_SHIFT][2];
	store<u8>(this, addr, value);
}

u16
arm9_cpu::ldrh(u32 addr)
{
	return load16n(addr & ~1);
}

s16
arm9_cpu::ldrsh(u32 addr)
{
	return load16n(addr & ~1);
}

static void
reset_page_table(nds_ctx *nds, u8 **pt, u64 start, u64 end)
{
	for (u64 addr = start; addr < end; addr += arm_cpu::PAGE_SIZE) {
		u64 page = addr >> arm_cpu::PAGE_SHIFT;
		switch (addr >> 24) {
		case 0x2:
			pt[page] = &nds->main_ram[addr & MAIN_RAM_MASK];
			break;
		case 0x3:
		{
			u8 *p = nds->shared_wram_p[0];
			pt[page] = &p[addr & nds->shared_wram_mask[0]];
			break;
		}
		default:
			pt[page] = nullptr;
			break;
		}
	}
}

static void
reset_time_table(nds_ctx *nds, u8 (*ft)[4], u8 (*lt)[4], u8 (*st)[4],
		u64 start, u64 end)
{
	for (u64 addr = start; addr < end; addr += arm9_cpu::TIME_PAGE_SIZE) {
		bool def = false;
		u64 page = addr >> arm9_cpu::TIME_PAGE_SHIFT;
		switch (addr >> 24) {
		case 0x2:
			ft[page][0] = 1;
			lt[page][0] = 1;
			lt[page][1] = 1;
			lt[page][2] = 1;
			lt[page][3] = 1;
			st[page][0] = 1;
			st[page][1] = 1;
			st[page][2] = 1;
			st[page][3] = 1;
			break;
		case 0x3:
		case 0x4:
		case 0x7:
		case 0xFF:
			ft[page][0] = 8;
			lt[page][0] = 8;
			lt[page][1] = 2;
			lt[page][2] = 8;
			lt[page][3] = 2;
			st[page][0] = 8;
			st[page][1] = 2;
			st[page][2] = 8;
			st[page][3] = 2;
			break;
		case 0x5:
		case 0x6:
			ft[page][0] = 10;
			lt[page][0] = 10;
			lt[page][1] = 4;
			lt[page][2] = 8;
			lt[page][3] = 2;
			st[page][0] = 10;
			st[page][1] = 4;
			st[page][2] = 8;
			st[page][3] = 2;
			break;
		case 0x8:
		case 0x9:
			if (nds->gba_slot_cpu == 0) {
				u8 n16 = gba_rom_nonseq_timings
						[nds->exmem[0] >> 2 & 3];
				u8 s16 = gba_rom_seq_timings
						[nds->exmem[0] >> 4 & 1];
				ft[page][0] = n16 + s16;
				lt[page][0] = n16 + s16;
				lt[page][1] = s16 + s16;
				lt[page][2] = n16;
				lt[page][3] = s16;
				st[page][0] = n16 + s16;
				st[page][1] = s16 + s16;
				st[page][2] = n16;
				st[page][3] = s16;
			} else {
				def = true;
			}
			break;
		default:
			def = true;
		}

		if (def) {
			ft[page][0] = 1;
			lt[page][0] = 1;
			lt[page][1] = 1;
			lt[page][2] = 1;
			lt[page][3] = 1;
			st[page][0] = 1;
			st[page][1] = 1;
			st[page][2] = 1;
			st[page][3] = 1;
		}
	}
}

static void reset_arm9_page_tables(arm9_cpu *cpu);
static void map_dtcm_pages(
		arm9_cpu *cpu, u8 **pt, u8 (*tt)[4], u64 start, u64 end);
static void map_itcm_pages(arm9_cpu *cpu, u8 **pt, u8 (*tt)[4], u64 end);

void
update_arm9_page_tables(arm9_cpu *cpu)
{
	reset_arm9_page_tables(cpu);

	u64 dtcm_end = cpu->dtcm_base + (u64)~cpu->dtcm_addr_mask + 1;
	u64 itcm_end = (u64)~cpu->itcm_addr_mask + 1;

	if (cpu->read_dtcm) {
		map_dtcm_pages(cpu, cpu->load_pt, cpu->load_timings,
				cpu->dtcm_base, dtcm_end);
	}
	if (cpu->read_itcm) {
		map_itcm_pages(cpu, cpu->load_pt, cpu->load_timings, itcm_end);
		map_itcm_pages(cpu, cpu->fetch_pt, cpu->fetch_timings,
				itcm_end);
	}

	if (cpu->write_dtcm) {
		map_dtcm_pages(cpu, cpu->store_pt, cpu->store_timings,
				cpu->dtcm_base, dtcm_end);
	}
	if (cpu->write_itcm) {
		map_itcm_pages(cpu, cpu->store_pt, cpu->store_timings,
				itcm_end);
	}
}

static void
reset_arm9_page_tables(arm9_cpu *cpu)
{
	reset_page_table(cpu->nds, cpu->fetch_pt, 0, 0x100000000);
	reset_page_table(cpu->nds, cpu->load_pt, 0, 0x100000000);
	reset_page_table(cpu->nds, cpu->store_pt, 0, 0x100000000);
	reset_time_table(cpu->nds, cpu->fetch_timings, cpu->load_timings,
			cpu->store_timings, 0, 0x100000000);
}

static void
map_dtcm_pages(arm9_cpu *cpu, u8 **pt, u8 (*tt)[4], u64 start, u64 end)
{
	bool null_page = (end - start) < arm_cpu::PAGE_SIZE;

	for (u64 addr = start; addr < end; addr += arm_cpu::PAGE_SIZE) {
		u64 page = addr >> arm_cpu::PAGE_SHIFT;
		pt[page] = null_page ? nullptr
		                     : &cpu->dtcm[addr & cpu->dtcm_array_mask];
	}

	for (u64 addr = start; addr < end; addr += arm9_cpu::TIME_PAGE_SIZE) {
		u64 page = addr >> arm9_cpu::TIME_PAGE_SHIFT;
		tt[page][0] = 1;
		tt[page][1] = 1;
		tt[page][2] = 1;
		tt[page][3] = 1;
	}
}

static void
map_itcm_pages(arm9_cpu *cpu, u8 **pt, u8 (*tt)[4], u64 end)
{
	bool null_page = end < arm_cpu::PAGE_SIZE;

	for (u64 addr = 0; addr < end; addr += arm_cpu::PAGE_SIZE) {
		u64 page = addr >> arm_cpu::PAGE_SHIFT;
		pt[page] = null_page ? nullptr
		                     : &cpu->itcm[addr & cpu->itcm_array_mask];
	}

	for (u64 addr = 0; addr < end; addr += arm9_cpu::TIME_PAGE_SIZE) {
		u64 page = addr >> arm9_cpu::TIME_PAGE_SHIFT;
		tt[page][0] = 1;
		tt[page][1] = 1;
		tt[page][2] = 1;
		tt[page][3] = 1;
	}
}

u32
cp15_read(arm9_cpu *cpu, u32 reg)
{
	switch (reg) {
	/* cache type register */
	case 0x001:
		return 0x0F0D2112;
	case 0x100:
		return cpu->ctrl_reg;
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
		LOG("cp15 read %03X\n", reg);
		return 0;
	case 0x910:
		return cpu->dtcm_reg;
	case 0x911:
		return cpu->itcm_reg;
	default:
		LOG("cp15 read %03X\n", reg);
		throw twice_error("unhandled cp15 read");
	}
}

static void ctrl_reg_write(arm9_cpu *cpu, u32 value);

void
cp15_write(arm9_cpu *cpu, u32 reg, u32 value)
{
	bool tcm_changed = false;

	switch (reg) {
	case 0x100:
		ctrl_reg_write(cpu, value);
		break;
	case 0x910:
	{
		u32 shift = std::clamp<u32>(value >> 1 & 0x1F, 3, 23);
		u32 mask = ((u64)512 << shift) - 1;
		cpu->dtcm_base = value & ~mask;
		cpu->dtcm_addr_mask = ~mask;
		cpu->dtcm_array_mask = mask & arm9_cpu::DTCM_MASK;
		cpu->dtcm_reg = value & 0xFFFFF03E;
		tcm_changed = true;
		break;
	}
	case 0x911:
	{
		u32 shift = std::clamp<u32>(value >> 1 & 0x1F, 3, 23);
		u32 mask = ((u64)512 << shift) - 1;
		cpu->itcm_addr_mask = ~mask;
		cpu->itcm_array_mask = mask & arm9_cpu::ITCM_MASK;
		cpu->itcm_reg = value & 0x3E;
		tcm_changed = true;
		break;
	}
	case 0x704:
	case 0x782:
		halt_cpu(cpu, arm_cpu::CPU_HALT);
		break;
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
	case 0x601:
	case 0x611:
	case 0x621:
	case 0x631:
	case 0x641:
	case 0x651:
	case 0x661:
	case 0x671:
		LOGVV("[protection_unit] unhandled cp15 write to %03X\n", reg);
		break;
	case 0x750:
	case 0x751:
	case 0x752:
	case 0x754:
	case 0x756:
	case 0x757:
	case 0x760:
	case 0x761:
	case 0x762:
	case 0x770:
	case 0x771:
	case 0x772:
	case 0x7A1:
	case 0x7A2:
	case 0x7A4:
	case 0x7B1:
	case 0x7B2:
	case 0x7D1:
	case 0x7E1:
	case 0x7E2:
	case 0x7F1:
	case 0x7F2:
		LOGVV("[cache_control] unhandled cp15 write to %03X\n", reg);
		break;
	default:
		LOG("unhandled cp15 write to %03X\n", reg);
	}

	if (tcm_changed) {
		update_arm9_page_tables(cpu);
	}
}

static void
ctrl_reg_write(arm9_cpu *cpu, u32 value)
{
	u32 diff = cpu->ctrl_reg ^ value;
	u32 unhandled = BIT(0) | BIT(2) | BIT(7) | BIT(12) | BIT(14) | BIT(15);

	if (diff & unhandled) {
		LOG("unhandled bits in cp15 write %08X\n", value);
	}

	cpu->exception_base = value & BIT(13) ? 0xFFFF0000 : 0;

	bool tcm_changed = false;

	if (diff & (BIT(16) | BIT(17))) {
		cpu->read_dtcm = (value & BIT(16)) && !(value & BIT(17));
		cpu->write_dtcm = value & BIT(16);
		tcm_changed = true;
	}

	if (diff & (BIT(18) | BIT(19))) {
		cpu->read_itcm = (value & BIT(18)) && !(value & BIT(19));
		cpu->write_itcm = value & BIT(18);
		tcm_changed = true;
	}

	cpu->ctrl_reg = (cpu->ctrl_reg & ~0xFF085) | (value & 0xFF085);

	if (tcm_changed) {
		update_arm9_page_tables(cpu);
	}
}

} // namespace twice
