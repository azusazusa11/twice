#ifndef TWICE_ARM_DSP_MATH_H
#define TWICE_ARM_DSP_MATH_H

#include "nds/arm/interpreter/arm/exception.h"
#include "nds/arm/interpreter/util.h"

namespace twice::arm::interpreter {

inline u32
saturated_add(arm_cpu *cpu, u32 a, u32 b)
{
	u32 r = a + b;

	if (((a ^ r) & (b ^ r)) >> 31) {
		r = r >> 31 ? 0x7FFFFFFF : 0x80000000;
		set_q(cpu, 1);
	}

	return r;
}

inline u32
saturated_sub(arm_cpu *cpu, u32 a, u32 b)
{
	u32 r = a - b;

	if (((a ^ b) & (a ^ r)) >> 31) {
		r = r >> 31 ? 0x7FFFFFFF : 0x80000000;
		set_q(cpu, 1);
	}

	return r;
}

template <int OP>
void
arm_sat_add_sub(arm_cpu *cpu)
{
	if (is_arm7(cpu)) {
		arm_undefined(cpu);
		return;
	}

	u32 rn = cpu->opcode >> 16 & 0xF;
	u32 rd = cpu->opcode >> 12 & 0xF;
	u32 rm = cpu->opcode & 0xF;

	switch (OP) {
	case 0:
		cpu->gpr[rd] = saturated_add(cpu, cpu->gpr[rm], cpu->gpr[rn]);
		break;
	case 1:
		cpu->gpr[rd] = saturated_sub(cpu, cpu->gpr[rm], cpu->gpr[rn]);
		break;
	case 2:
	{
		u32 double_rn = saturated_add(cpu, cpu->gpr[rn], cpu->gpr[rn]);
		cpu->gpr[rd] = saturated_add(cpu, cpu->gpr[rm], double_rn);
		break;
	}
	case 3:
	{
		u32 double_rn = saturated_add(cpu, cpu->gpr[rn], cpu->gpr[rn]);
		cpu->gpr[rd] = saturated_sub(cpu, cpu->gpr[rm], double_rn);
		break;
	}
	}

	cpu->add_cycles_c();
}

inline u32
sticky_add(arm_cpu *cpu, u32 a, u32 b)
{
	u32 r = a + b;

	if (((a ^ r) & (b ^ r)) >> 31) {
		set_q(cpu, 1);
	}

	return r;
}

template <int OP, int Y, int X>
void
arm_dsp_multiply(arm_cpu *cpu)
{
	if (is_arm7(cpu)) {
		return;
	}

	u32 rd = cpu->opcode >> 16 & 0xF;
	u32 rn = cpu->opcode >> 12 & 0xF;
	u32 rs = cpu->opcode >> 8 & 0xF;
	u32 rm = cpu->opcode & 0xF;

	s32 a = X == 0 ? (s32)(cpu->gpr[rm] << 16) >> 16
	               : (s32)(cpu->gpr[rm]) >> 16;
	s32 b = Y == 0 ? (s32)(cpu->gpr[rs] << 16) >> 16
	               : (s32)(cpu->gpr[rs]) >> 16;

	switch (OP) {
	case 0x0:
		cpu->gpr[rd] = sticky_add(cpu, a * b, cpu->gpr[rn]);
		break;
	case 0x1:
	{
		u32 r = (s64)(s32)cpu->gpr[rm] * b >> 16;
		if (X) {
			cpu->gpr[rd] = r;
		} else {
			cpu->gpr[rd] = sticky_add(cpu, r, cpu->gpr[rn]);
		}
		break;
	}
	case 0x2:
	{
		u64 r = (s64)(a * b);
		r += (u64)cpu->gpr[rd] << 32 | cpu->gpr[rn];
		cpu->gpr[rd] = r >> 32;
		cpu->gpr[rn] = r;
		break;
	}
	case 0x3:
		cpu->gpr[rd] = a * b;
		break;
	}

	cpu->add_cycles_c(OP == 2);
}
} // namespace twice::arm::interpreter

#endif
