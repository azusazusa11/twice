#ifndef TWICE_ARM_COP_H
#define TWICE_ARM_COP_H

#include "nds/arm/interpreter/arm/exception.h"
#include "nds/arm/interpreter/util.h"

namespace twice {

template <int OP1, int L, int OP2>
void
arm_cop_reg(Arm *cpu)
{
	u32 cn = cpu->opcode >> 16 & 0xF;
	u32 rd = cpu->opcode >> 12 & 0xF;
	u32 cp_num = cpu->opcode >> 8 & 0xF;
	u32 cm = cpu->opcode & 0xF;

	if (L == 0) {
		if (cpu->is_arm7()) {
			fprintf(stderr, "arm7 mcr\n");
			return;
		}

		if (cp_num != 15) {
			throw TwiceError("arm9 mcr cp_num != 15");
		}

		if (OP1 != 0) {
			throw TwiceError("arm9 mcr op1 != 0");
		}

		u32 reg = cn << 8 | cm << 4 | OP2;
		((Arm9 *)cpu)->cp15_write(reg, cpu->gpr[rd]);
	} else {
		if (cpu->is_arm7()) {
			throw TwiceError("arm7 mrc");
		}

		if (cp_num != 15) {
			throw TwiceError("arm9 mrc cp_num != 15");
		}

		if (OP1 != 0) {
			throw TwiceError("arm9 mrc op1 != 0");
		}

		if (!cpu->in_privileged_mode()) {
			throw TwiceError("arm9 mrc not in privileged mode");
		}

		u32 reg = cn << 8 | cm << 4 | OP2;
		u32 value = ((Arm9 *)cpu)->cp15_read(reg);

		if (rd == 15) {
			u32 mask = 0xF0000000;
			cpu->cpsr = (cpu->cpsr & ~mask) | (value & mask);
		} else {
			cpu->gpr[rd] = value;
		}
	}
}

inline void
arm_cdp(Arm *cpu)
{
	(void)cpu;
	throw TwiceError("arm cdp");
}

inline void
arm_ldc(Arm *cpu)
{
	(void)cpu;
	throw TwiceError("arm ldc");
}

inline void
arm_stc(Arm *cpu)
{
	(void)cpu;
	throw TwiceError("arm stc");
}

inline void
arm_mcrr(Arm *cpu)
{
	(void)cpu;
	throw TwiceError("arm mcrr");
}

inline void
arm_mrrc(Arm *cpu)
{
	(void)cpu;
	throw TwiceError("arm mrrc");
}

} // namespace twice

#endif