#ifndef TWICE_THUMB_EXCEPTION_H
#define TWICE_THUMB_EXCEPTION_H

#include "nds/arm/interpreter/util.h"

namespace twice {

inline void
thumb_undefined(arm_cpu *cpu)
{
	u32 old_cpsr = cpu->cpsr;

	cpu->cpsr &= ~0xBF;
	cpu->cpsr |= 0x9B;
	cpu->switch_mode(MODE_SVC);
	cpu->interrupt = false;

	cpu->gpr[14] = cpu->pc() - 2;
	cpu->spsr() = old_cpsr;
	cpu->arm_jump(cpu->exception_base + 0x4);
}

inline void
thumb_swi(arm_cpu *cpu)
{
	u32 old_cpsr = cpu->cpsr;

	cpu->cpsr &= ~0xBF;
	cpu->cpsr |= 0x93;
	cpu->switch_mode(MODE_SVC);
	cpu->interrupt = false;

	cpu->gpr[14] = cpu->pc() - 2;
	cpu->spsr() = old_cpsr;
	cpu->arm_jump(cpu->exception_base + 0x8);
}

inline void
thumb_bkpt(arm_cpu *cpu)
{
	(void)cpu;
	throw twice_error("thumb bkpt not implemented");
}

} // namespace twice

#endif
