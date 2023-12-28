#ifndef TWICE_ARM_LOAD_STORE_H
#define TWICE_ARM_LOAD_STORE_H

#include "nds/arm/interpreter/util.h"

namespace twice::arm::interpreter {

template <int P, int U, int B, int W, int L, int SHIFT>
void
arm_sdt(arm_cpu *cpu)
{
	u32 rn = cpu->opcode >> 16 & 0xF;
	u32 rd = cpu->opcode >> 12 & 0xF;

	u32 offset;
	u32 address;

	if (SHIFT == 4) {
		offset = cpu->opcode & 0xFFF;
	} else {
		u32 rm = cpu->opcode & 0xF;
		u32 shift_imm = cpu->opcode >> 7 & 0x1F;
		switch (SHIFT) {
		case 0:
			offset = cpu->gpr[rm] << shift_imm;
			break;
		case 1:
			if (shift_imm == 0) {
				offset = 0;
			} else {
				offset = cpu->gpr[rm] >> shift_imm;
			}
			break;
		case 2:
			if (shift_imm == 0) {
				if (cpu->gpr[rm] >> 31) {
					offset = 0xFFFFFFFF;
				} else {
					offset = 0;
				}
			} else {
				offset = (s32)cpu->gpr[rm] >> shift_imm;
			}
			break;
		default: /* case 3 */
			if (shift_imm == 0) {
				offset = (u32)get_c(cpu) << 31 |
				         cpu->gpr[rm] >> 1;
			} else {
				offset = std::rotr(cpu->gpr[rm], shift_imm);
			}
		}
	}

	if (!U) {
		offset = -offset;
	}

	if (P) {
		address = cpu->gpr[rn] + offset;
	} else {
		address = cpu->gpr[rn];
	}

	bool writeback = (W == 1 || P == 0) && rn != 15;
	if ((W == 1 || P == 0) && rn == 15) {
		LOG("writeback with pc as base\n");
	}

	if (L == 1) {
		if (writeback) {
			cpu->gpr[rn] += offset;
		}

		u32 value;
		if (B == 1) {
			value = arm_do_ldrb(cpu, address);
		} else {
			value = arm_do_ldr(cpu, address);
		}

		cpu->add_cycles_cdi();

		if (rd == 15) {
			if (is_arm9(cpu)) {
				arm_do_bx(cpu, value);
			} else {
				cpu->arm_jump(value & ~3);
			}
		} else {
			cpu->gpr[rd] = value;
		}
	} else {
		u32 value = cpu->gpr[rd];
		if (rd == 15) {
			value += 4;
		}

		if (B == 1) {
			arm_do_strb(cpu, address, value);
		} else {
			arm_do_str(cpu, address, value);
		}

		if (writeback) {
			cpu->gpr[rn] += offset;
		}

		cpu->add_cycles_cd();
	}
}
} // namespace twice::arm::interpreter

#endif
