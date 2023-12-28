#ifndef TWICE_ARM_COP_H
#define TWICE_ARM_COP_H

#include "nds/arm/arm9.h"
#include "nds/arm/interpreter/arm/exception.h"
#include "nds/arm/interpreter/util.h"

namespace twice::arm::interpreter {

template <int OP1, int L, int OP2>
void
arm_cop_reg(arm_cpu *cpu)
{
	u32 cn = cpu->opcode >> 16 & 0xF;
	u32 rd = cpu->opcode >> 12 & 0xF;
	u32 cp_num = cpu->opcode >> 8 & 0xF;
	u32 cm = cpu->opcode & 0xF;

	if (L == 0) {
		if (is_arm7(cpu)) {
			LOG("arm7 mcr\n");
			cpu->add_cycles_c();
			return;
		} else {
			if (cp_num != 15) {
				throw twice_error("arm9 mcr cp_num != 15");
			}

			if (OP1 != 0) {
				throw twice_error("arm9 mcr op1 != 0");
			}

			u32 reg = cn << 8 | cm << 4 | OP2;
			u32 value = cpu->gpr[rd];
			if (rd == 15) {
				value += 4;
			}
			cp15_write((arm9_cpu *)cpu, reg, value);
			cpu->add_cycles_c(2);
		}
	} else {
		u32 value;

		if (is_arm7(cpu)) {
			if (cp_num != 14) {
				LOG("arm7 mrc with cp_num %u\n", cp_num);
				arm_undefined(cpu);
				return;
			}

			value = cpu->pipeline[1];
		} else {
			if (cp_num != 15) {
				LOG("arm9 mrc with cp_num %u\n", cp_num);
				arm_undefined(cpu);
				return;
			}

			if (OP1 != 0) {
				throw twice_error("arm9 mrc op1 != 0");
			}

			if (!in_privileged_mode(cpu)) {
				throw twice_error(
						"arm9 mrc not in privileged mode");
			}

			u32 reg = cn << 8 | cm << 4 | OP2;
			value = cp15_read((arm9_cpu *)cpu, reg);
		}

		if (rd == 15) {
			u32 mask = 0xF0000000;
			cpu->cpsr = (cpu->cpsr & ~mask) | (value & mask);
		} else {
			cpu->gpr[rd] = value;
		}

		cpu->add_cycles_c(3);
	}
}

inline void
arm_cdp(arm_cpu *)
{
	throw twice_error("arm cdp");
}

inline void
arm_ldc(arm_cpu *)
{
	throw twice_error("arm ldc");
}

inline void
arm_stc(arm_cpu *)
{
	throw twice_error("arm stc");
}

inline void
arm_mcrr(arm_cpu *)
{
	throw twice_error("arm mcrr");
}

inline void
arm_mrrc(arm_cpu *)
{
	throw twice_error("arm mrrc");
}

} // namespace twice::arm::interpreter

#endif
