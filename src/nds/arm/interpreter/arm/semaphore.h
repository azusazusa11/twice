#ifndef TWICE_ARM_SEMAPHORE_H
#define TWICE_ARM_SEMAPHORE_H

#include "nds/arm/interpreter/util.h"

namespace twice::arm::interpreter {

template <int B>
void
arm_swap(arm_cpu *cpu)
{
	u32 rn = cpu->opcode >> 16 & 0xF;
	u32 rd = cpu->opcode >> 12 & 0xF;
	u32 rm = cpu->opcode & 0xF;

	u32 load_data_cycles;

	if (B) {
		u8 temp = arm_do_ldrb(cpu, cpu->gpr[rn]);
		load_data_cycles = cpu->data_cycles;
		arm_do_strb(cpu, cpu->gpr[rn], cpu->gpr[rm]);
		cpu->gpr[rd] = temp;
	} else {
		u32 temp = arm_do_ldr(cpu, cpu->gpr[rn]);
		load_data_cycles = cpu->data_cycles;
		arm_do_str(cpu, cpu->gpr[rn], cpu->gpr[rm]);
		cpu->gpr[rd] = temp;
	}

	cpu->data_cycles += load_data_cycles;
	cpu->add_cycles_cd(1);
}

} // namespace twice::arm::interpreter

#endif
