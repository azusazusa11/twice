#ifndef TWICE_THUMB_BRANCH_H
#define TWICE_THUMB_BRANCH_H

#include "nds/arm/interpreter/thumb/exception.h"
#include "nds/arm/interpreter/util.h"

namespace twice::arm::interpreter {

template <int COND>
void
thumb_b1(arm_cpu *cpu)
{
	bool N = cpu->cpsr & BIT(31);
	bool Z = cpu->cpsr & BIT(30);
	bool C = cpu->cpsr & BIT(29);
	bool V = cpu->cpsr & BIT(28);

	bool jump;

	switch (COND) {
	case 0x0:
		jump = Z;
		break;
	case 0x1:
		jump = !Z;
		break;
	case 0x2:
		jump = C;
		break;
	case 0x3:
		jump = !C;
		break;
	case 0x4:
		jump = N;
		break;
	case 0x5:
		jump = !N;
		break;
	case 0x6:
		jump = V;
		break;
	case 0x7:
		jump = !V;
		break;
	case 0x8:
		jump = C && !Z;
		break;
	case 0x9:
		jump = !C || Z;
		break;
	case 0xA:
		jump = N == V;
		break;
	case 0xB:
		jump = N != V;
		break;
	case 0xC:
		jump = !Z && N == V;
		break;
	case 0xD:
		jump = Z || N != V;
		break;
	case 0xE:
		jump = true;
		break;
	case 0xF:
		jump = false;
	}

	cpu->add_cycles_c();

	if (jump) {
		u32 offset = (s16)(cpu->opcode << 8) >> 7;
		cpu->thumb_jump(cpu->pc() + offset);
	}
}

inline void
thumb_b2(arm_cpu *cpu)
{
	cpu->add_cycles_c();
	u32 offset = (s16)(cpu->opcode << 5) >> 4;
	cpu->thumb_jump(cpu->pc() + offset);
}

template <int H>
void
thumb_b_pair(arm_cpu *cpu)
{
	if (H == 2) {
		cpu->add_cycles_c();
		u32 offset = (s32)(cpu->opcode << 21) >> 9;
		cpu->gpr[14] = cpu->pc() + offset;
	} else if (H == 3) {
		cpu->add_cycles_c();
		u32 ret_addr = cpu->pc() - 2;
		u32 jump_addr = cpu->gpr[14] + ((cpu->opcode & 0x7FF) << 1);
		cpu->thumb_jump(jump_addr & ~1);
		cpu->gpr[14] = ret_addr | 1;
	} else {
		if (is_arm7(cpu)) {
			thumb_undefined(cpu);
			return;
		}

		cpu->add_cycles_c();
		u32 ret_addr = cpu->pc() - 2;
		u32 jump_addr = cpu->gpr[14] + ((cpu->opcode & 0x7FF) << 1);
		cpu->cpsr &= ~0x20;
		cpu->arm_jump(jump_addr & ~3);
		cpu->gpr[14] = ret_addr | 1;
	}
}

inline void
thumb_bx(arm_cpu *cpu)
{
	cpu->add_cycles_c();
	u32 addr = cpu->gpr[cpu->opcode >> 3 & 0xF];
	thumb_do_bx(cpu, addr);
}

inline void
thumb_blx2(arm_cpu *cpu)
{
	if (is_arm7(cpu)) {
		thumb_undefined(cpu);
		return;
	}

	cpu->add_cycles_c();
	u32 addr = cpu->gpr[cpu->opcode >> 3 & 0xF];
	cpu->gpr[14] = (cpu->pc() - 2) | 1;
	thumb_do_bx(cpu, addr);
}

} // namespace twice::arm::interpreter

#endif
