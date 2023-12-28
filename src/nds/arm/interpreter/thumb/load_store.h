#ifndef TWICE_THUMB_LOAD_STORE_H
#define TWICE_THUMB_LOAD_STORE_H

#include "nds/arm/interpreter/util.h"

namespace twice::arm::interpreter {

template <int OP, int OFFSET>
void
thumb_load_store_imm(arm_cpu *cpu)
{
	u32 rn = cpu->opcode >> 3 & 0x7;
	u32 rd = cpu->opcode & 0x7;
	u32 addr;

	switch (OP) {
	case 0xD:
		addr = cpu->gpr[rn] + OFFSET * 4;
		cpu->gpr[rd] = arm_do_ldr(cpu, addr);
		break;
	case 0xF:
		addr = cpu->gpr[rn] + OFFSET;
		cpu->gpr[rd] = arm_do_ldrb(cpu, addr);
		break;
	case 0x11:
		addr = cpu->gpr[rn] + OFFSET * 2;
		cpu->gpr[rd] = cpu->ldrh(addr);
		break;
	case 0xC:
		addr = cpu->gpr[rn] + OFFSET * 4;
		arm_do_str(cpu, addr, cpu->gpr[rd]);
		break;
	case 0xE:
		addr = cpu->gpr[rn] + OFFSET;
		arm_do_strb(cpu, addr, cpu->gpr[rd]);
		break;
	case 0x10:
		addr = cpu->gpr[rn] + OFFSET * 2;
		arm_do_strh(cpu, addr, cpu->gpr[rd]);
		break;
	}

	if (OP == 0xD || OP == 0xF || OP == 0x11) {
		cpu->add_cycles_cdi();
	} else {
		cpu->add_cycles_cd();
	}
}

template <int OP, int RM>
void
thumb_load_store_reg(arm_cpu *cpu)
{
	u32 rn = cpu->opcode >> 3 & 0x7;
	u32 rd = cpu->opcode & 0x7;
	u32 addr = cpu->gpr[rn] + cpu->gpr[RM];

	switch (OP) {
	case 0x2C:
		cpu->gpr[rd] = arm_do_ldr(cpu, addr);
		break;
	case 0x2E:
		cpu->gpr[rd] = arm_do_ldrb(cpu, addr);
		break;
	case 0x2D:
		cpu->gpr[rd] = cpu->ldrh(addr);
		break;
	case 0x2B:
		cpu->gpr[rd] = arm_do_ldrsb(cpu, addr);
		break;
	case 0x2F:
		cpu->gpr[rd] = cpu->ldrsh(addr);
		break;
	case 0x28:
		arm_do_str(cpu, addr, cpu->gpr[rd]);
		break;
	case 0x2A:
		arm_do_strb(cpu, addr, cpu->gpr[rd]);
		break;
	case 0x29:
		arm_do_strh(cpu, addr, cpu->gpr[rd]);
		break;
	}

	if (OP == 0x2C || OP == 0x2E || OP == 0x2D || OP == 0x2B ||
			OP == 0x2F) {
		cpu->add_cycles_cdi();
	} else {
		cpu->add_cycles_cd();
	}
}

template <int RD>
void
thumb_load_pc_relative(arm_cpu *cpu)
{
	u32 addr = (cpu->pc() & ~3) + ((cpu->opcode & 0xFF) << 2);
	cpu->gpr[RD] = cpu->load32n(addr);
	cpu->add_cycles_cdi();
}

template <int L, int RD>
void
thumb_load_store_sp_relative(arm_cpu *cpu)
{
	u32 addr = cpu->gpr[13] + ((cpu->opcode & 0xFF) << 2);

	if (L) {
		cpu->gpr[RD] = arm_do_ldr(cpu, addr);
		cpu->add_cycles_cdi();
	} else {
		arm_do_str(cpu, addr, cpu->gpr[RD]);
		cpu->add_cycles_cd();
	}
}

} // namespace twice::arm::interpreter

#endif
