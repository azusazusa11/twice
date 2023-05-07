#ifndef TWICE_ARM_EXCEPTION_H
#define TWICE_ARM_EXCEPTION_H

#include "nds/arm/interpreter/util.h"

namespace twice {

inline void
arm_undefined(Arm *cpu)
{
	throw TwiceError("arm undefined instruction");
}

inline void
arm_swi(Arm *cpu)
{
	u32 old_cpsr = cpu->cpsr;

	cpu->cpsr &= ~0xBF;
	cpu->cpsr |= 0x93;
	cpu->switch_mode(MODE_SVC);

	/* TODO: disable interrupts */

	cpu->gpr[14] = cpu->pc() - 4;
	cpu->spsr() = old_cpsr;
	cpu->arm_jump(cpu->exception_base + 0x8);
}

inline void
arm_bkpt(Arm *cpu)
{
	throw TwiceError("arm bkpt not implemented");
}

} // namespace twice

#endif