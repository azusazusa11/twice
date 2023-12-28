#ifndef TWICE_ARM_EXCEPTION_H
#define TWICE_ARM_EXCEPTION_H

#include "nds/arm/interpreter/util.h"

namespace twice::arm::interpreter {

inline void
arm_undefined(arm_cpu *cpu)
{
	u32 old_cpsr = cpu->cpsr;

	cpu->cpsr &= ~0xBF;
	cpu->cpsr |= 0x9B;
	arm_switch_mode(cpu, arm_cpu::MODE_UND);
	cpu->interrupt = false;

	cpu->gpr[14] = cpu->pc() - 4;
	cpu->spsr() = old_cpsr;
	cpu->add_cycles_c(1);
	cpu->arm_jump(cpu->exception_base + 0x4);
}

inline void
arm_swi(arm_cpu *cpu)
{
	u32 old_cpsr = cpu->cpsr;

	cpu->cpsr &= ~0xBF;
	cpu->cpsr |= 0x93;
	arm_switch_mode(cpu, arm_cpu::MODE_SVC);
	cpu->interrupt = false;

	cpu->gpr[14] = cpu->pc() - 4;
	cpu->spsr() = old_cpsr;
	cpu->cycles = cpu->code_cycles;
	cpu->add_cycles_c();
	cpu->arm_jump(cpu->exception_base + 0x8);
}

inline void
arm_bkpt(arm_cpu *)
{
	throw twice_error("arm bkpt not implemented");
}

} // namespace twice::arm::interpreter

#endif
