#ifndef TWICE_ARM_MULTIPLY_H
#define TWICE_ARM_MULTIPLY_H

#include "nds/arm/interpreter/util.h"

namespace twice::arm::interpreter {

template <int A, int S>
void
arm_multiply(arm_cpu *cpu)
{
	u32 rd = cpu->opcode >> 16 & 0xF;
	u32 rs = cpu->opcode >> 8 & 0xF;
	u32 rm = cpu->opcode & 0xF;

	u32 r = cpu->gpr[rm] * cpu->gpr[rs];

	if (A) {
		u32 rn = cpu->opcode >> 12 & 0xF;
		r += cpu->gpr[rn];
	}

	cpu->gpr[rd] = r;

	if (S) {
		set_nz(cpu, r >> 31, r == 0);
	}

	u32 internal_cycles;
	if (is_arm9(cpu)) {
		internal_cycles = S ? 3 : 1;
	} else {
		internal_cycles = rs & BIT(31) ? get_mul_internal_cycles(~rs)
		                               : get_mul_internal_cycles(rs);
		if (A) {
			internal_cycles += 1;
		}
	}

	cpu->add_cycles_c(internal_cycles);
}

template <int U, int A, int S>
void
arm_multiply_long(arm_cpu *cpu)
{
	u32 rdhi = cpu->opcode >> 16 & 0xF;
	u32 rdlo = cpu->opcode >> 12 & 0xF;
	u32 rs = cpu->opcode >> 8 & 0xF;
	u32 rm = cpu->opcode & 0xF;

	u64 r;
	/* U is flipped on purpose! */
	if (U) {
		r = (s64)(s32)cpu->gpr[rm] * (s32)cpu->gpr[rs];
	} else {
		r = (u64)cpu->gpr[rm] * cpu->gpr[rs];
	}

	if (A) {
		r += (u64)cpu->gpr[rdhi] << 32 | cpu->gpr[rdlo];
	}

	cpu->gpr[rdhi] = r >> 32;
	cpu->gpr[rdlo] = r;

	if (S) {
		set_nz(cpu, r >> 63, r == 0);
	}

	u32 internal_cycles;
	if (is_arm9(cpu)) {
		internal_cycles = S ? 4 : 2;
	} else {
		internal_cycles = get_mul_internal_cycles(
				(U && rs & BIT(31)) ? ~rs : rs);
		internal_cycles += A ? 2 : 1;
	}

	cpu->add_cycles_c(internal_cycles);
}

} // namespace twice::arm::interpreter

#endif
