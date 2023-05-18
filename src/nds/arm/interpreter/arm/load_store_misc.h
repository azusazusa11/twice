#ifndef TWICE_ARM_LOAD_STORE_MISC_H
#define TWICE_ARM_LOAD_STORE_MISC_H

#include "nds/arm/interpreter/util.h"

namespace twice {

template <int P, int U, int I, int W, int L, int S, int H>
void
arm_misc_dt(Arm *cpu)
{
	u32 rn = cpu->opcode >> 16 & 0xF;
	u32 rd = cpu->opcode >> 12 & 0xF;
	u32 imm_hi = cpu->opcode >> 8 & 0xF;
	u32 imm_lo = cpu->opcode & 0xF;
	u32 rm = cpu->opcode & 0xF;

	u32 offset;
	u32 address;

	if (I) {
		offset = imm_hi << 4 | imm_lo;
	} else {
		offset = cpu->gpr[rm];
	}

	if (!U) {
		offset = -offset;
	}

	if (P) {
		address = cpu->gpr[rn] + offset;
	} else {
		address = cpu->gpr[rn];
	}

	constexpr bool writeback = W == 1 || P == 0;

	if (L == 1) {
		if (writeback) {
			cpu->gpr[rn] += offset;
		}

		u32 value;
		if (S == 0 && H == 1) {
			value = cpu->ldrh(address);
		} else if (S == 1 && H == 0) {
			value = arm_do_ldrsb(cpu, address);
		} else if (S == 1 && H == 1) {
			value = cpu->ldrsh(address);
		}

		if (rd == 15) {
			cpu->arm_jump(value & ~3);
		} else {
			cpu->gpr[rd] = value;
		}
	} else if (L == 0 && S == 0 && H == 1) {
		u16 value = cpu->gpr[rd];

		if (rd == 15) {
			value += 4;
		}

		arm_do_strh(cpu, address, value);

		if (writeback) {
			cpu->gpr[rn] += offset;
		}
	} else if (L == 0 && S == 1 && H == 0) {
		if (writeback) {
			cpu->gpr[rn] += offset;
		}

		if (cpu->is_arm9()) {
			if (rd & 1) {
				fprintf(stderr, "ldrd odd reg\n");
				rd &= ~1;
			}

			cpu->gpr[rd] = cpu->load32(address & ~3);
			u32 value = cpu->load32((address & ~3) + 4);

			if (rd + 1 == 15) {
				cpu->arm_jump(value & ~3);
			} else {
				cpu->gpr[rd + 1] = value;
			}
		}
	} else if (L == 0 && S == 1 && H == 1) {
		if (cpu->is_arm7()) {
			return;
		}

		if (rd & 1) {
			fprintf(stderr, "strd odd reg\n");
			rd &= ~1;
		}

		cpu->store32(address & ~3, cpu->gpr[rd]);
		u32 value = cpu->gpr[rd + 1];

		if (rd + 1 == 15) {
			value += 4;
		}

		cpu->store32((address & ~3) + 4, value);

		if (writeback) {
			cpu->gpr[rn] += offset;
		}
	}
}
} // namespace twice

#endif
