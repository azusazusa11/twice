#ifndef TWICE_ARM_LOAD_STORE_MULTIPLE_H
#define TWICE_ARM_LOAD_STORE_MULTIPLE_H

#include "nds/arm/interpreter/util.h"

namespace twice::arm::interpreter {

template <int P, int U, int S, int W, int L>
void
arm_block_dt(arm_cpu *cpu)
{
	u32 rn = cpu->opcode >> 16 & 0xF;
	u16 register_list = cpu->opcode & 0xFFFF;
	u32 offset = 4 * std::popcount(register_list);

	u32 addr;
	u32 jump_addr;
	u32 writeback_value;

	if (register_list == 0) {
		offset = 0x40;
	}

	if (P == 0 && U == 1) {
		addr = cpu->gpr[rn];
		writeback_value = cpu->gpr[rn] + offset;
	} else if (P == 1 && U == 1) {
		addr = cpu->gpr[rn] + 4;
		writeback_value = cpu->gpr[rn] + offset;
	} else if (P == 0 && U == 0) {
		addr = cpu->gpr[rn] - offset + 4;
		writeback_value = cpu->gpr[rn] - offset;
	} else if (P == 1 && U == 0) {
		addr = cpu->gpr[rn] - offset;
		writeback_value = cpu->gpr[rn] - offset;
	}

	addr &= ~3;

	bool writeback = W && rn != 15;
	if (W && rn == 15) {
		LOG("writeback with pc as base\n");
	}

	bool nonseq = true;

	if (L == 1) {
		bool cpsr_written = false;

		if (S == 0) {
			TWICE_ARM_LDM_(0, 14, cpu->gpr, 0);

			bool jump = (register_list >> 15) ||
			            (is_arm7(cpu) && register_list == 0);
			if (jump) {
				jump_addr = cpu->load32(addr, nonseq);
			}

			cpu->add_cycles_cdi();

			if (jump) {
				if (register_list >> 1) {
					if (!is_arm7(cpu)) {
						arm_do_bx(cpu, jump_addr);
					} else {
						cpu->arm_jump(jump_addr & ~3);
					}
				} else {
					cpu->arm_jump(jump_addr & ~3);
				}
			}
		} else if (!(register_list >> 15)) {
			TWICE_ARM_LDM_(0, 7, cpu->gpr, 0);

			if (in_fiq_mode(cpu)) {
				TWICE_ARM_LDM_(8, 12, cpu->fiqr, -8);
			} else {
				TWICE_ARM_LDM_(8, 12, cpu->gpr, 0);
			}

			if (in_sys_or_usr_mode(cpu)) {
				TWICE_ARM_LDM_(13, 14, cpu->gpr, 0);
			} else {
				TWICE_ARM_LDM_(13, 14, cpu->bankedr[0], -13);
			}

			bool jump = is_arm7(cpu) && register_list == 0;
			if (jump) {
				jump_addr = cpu->load32n(addr);
			}

			cpu->add_cycles_cdi();

			if (jump) {
				cpu->arm_jump(jump_addr & ~3);
			}
		} else {
			TWICE_ARM_LDM_(0, 14, cpu->gpr, 0);

			u32 value = cpu->load32(addr, nonseq);
			cpu->add_cycles_cdi();

			cpu->cpsr = cpu->spsr();
			if (cpu->cpsr & 0x20) {
				cpu->thumb_jump(value & ~1);
			} else {
				cpu->arm_jump(value & ~3);
			}
			cpsr_written = true;
		}

		if (writeback) {
			/* If rn is included in register_list:
			 * ARMv4: no writeback
			 * ARMv5: writeback if rn is only register,
			 *        or not the last register
			 */
			if (register_list & BIT(rn)) {
				bool only = register_list == BIT(rn);
				bool last = register_list >> rn == 1;
				if (is_arm9(cpu) && (only || !last)) {
					cpu->gpr[rn] = writeback_value;
				}
			} else {
				cpu->gpr[rn] = writeback_value;
			}
		}

		/* mode switch done after writeback */
		if (cpsr_written) {
			arm_on_cpsr_write(cpu);
		}
	} else {
		if (writeback) {
			/* If rn is included in register_list:
			 * ARMv4: if rn is first store old base
			 *        else store new base
			 * ARMv5: store old base
			 */
			bool in_rlist = register_list & BIT(rn);
			bool not_first = register_list & MASK(rn);
			if (in_rlist && is_arm7(cpu) && not_first) {
				cpu->gpr[rn] = writeback_value;
			}
		}

		if (S == 0) {
			TWICE_ARM_STM_(0, 14, cpu->gpr, 0);
		} else {
			TWICE_ARM_STM_(0, 7, cpu->gpr, 0);

			if (in_fiq_mode(cpu)) {
				TWICE_ARM_STM_(8, 12, cpu->fiqr, -8);
			} else {
				TWICE_ARM_STM_(8, 12, cpu->gpr, 0);
			}

			if (in_sys_or_usr_mode(cpu)) {
				TWICE_ARM_STM_(13, 14, cpu->gpr, 0);
			} else {
				TWICE_ARM_STM_(13, 14, cpu->bankedr[0], -13);
			}
		}

		if (register_list >> 15) {
			cpu->store32(addr, cpu->pc() + 4, nonseq);
		}

		if (is_arm7(cpu) && register_list == 0) {
			cpu->store32n(addr, cpu->pc() + 4);
		}

		if (writeback) {
			cpu->gpr[rn] = writeback_value;
		}

		cpu->add_cycles_cd();
	}
}

} // namespace twice::arm::interpreter

#endif
