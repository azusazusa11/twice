#ifndef TWICE_ARM_CLZ_H
#define TWICE_ARM_CLZ_H

#include "nds/arm/interpreter/arm/exception.h"
#include "nds/arm/interpreter/util.h"

namespace twice::arm::interpreter {

inline void
arm_clz(arm_cpu *cpu)
{
	if (is_arm7(cpu)) {
		arm_undefined(cpu);
		return;
	}

	cpu->add_cycles_c();
	u32 rd = cpu->opcode >> 12 & 0xF;
	u32 rm = cpu->opcode & 0xF;
	cpu->gpr[rd] = std::countl_zero(cpu->gpr[rm]);
}

} // namespace twice::arm::interpreter

#endif
