#ifndef TWICE_ARM_LOAD_STORE_MULTIPLE_H
#define TWICE_ARM_LOAD_STORE_MULTIPLE_H

#include "nds/arm/interpreter/util.h"

namespace twice {

#define TWICE_ARM_LDM_(start_, end_, arr_, off_)                              \
	do {                                                                  \
		for (int i = (start_); i <= (end_); i++) {                    \
			if (register_list & (1 << i)) {                       \
				(arr_)[i + (off_)] = cpu->load32(addr);       \
				addr += 4;                                    \
			}                                                     \
		}                                                             \
	} while (0)

#define TWICE_ARM_STM_(start_, end_, arr_, off_)                              \
	do {                                                                  \
		for (int i = (start_); i <= (end_); i++) {                    \
			if (register_list & (1 << i)) {                       \
				cpu->store32(addr, (arr_)[i + (off_)]);       \
				addr += 4;                                    \
			}                                                     \
		}                                                             \
	} while (0)

template <int P, int U, int S, int W, int L>
void
arm_block_dt(Arm *cpu)
{
	u32 rn = cpu->opcode >> 16 & 0xF;
	u16 register_list = cpu->opcode & 0xFFFF;
	u32 offset = 4 * std::popcount(register_list);

	u32 addr;
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

	if (L == 1) {
		bool cpsr_written = false;

		if (S == 0) {
			TWICE_ARM_LDM_(0, 14, cpu->gpr, 0);

			if (register_list >> 15) {
				u32 value = cpu->load32(addr);
				if (!cpu->is_arm7()) {
					arm_do_bx(cpu, value);
				} else {
					cpu->arm_jump(value & ~3);
				}
			}

			if (cpu->is_arm7() && register_list == 0) {
				cpu->arm_jump(cpu->load32(addr) & ~3);
			}
		} else if (!(register_list >> 15)) {
			TWICE_ARM_LDM_(0, 7, cpu->gpr, 0);

			if (cpu->in_fiq_mode()) {
				TWICE_ARM_LDM_(8, 12, cpu->fiqr, -8);
			} else {
				TWICE_ARM_LDM_(8, 12, cpu->gpr, 0);
			}

			if (cpu->in_sys_or_usr_mode()) {
				TWICE_ARM_LDM_(13, 14, cpu->gpr, 0);
			} else {
				TWICE_ARM_LDM_(13, 14, cpu->bankedr[0], -13);
			}

			if (cpu->is_arm7() && register_list == 0) {
				cpu->arm_jump(cpu->load32(addr) & ~3);
			}
		} else {
			TWICE_ARM_LDM_(0, 14, cpu->gpr, 0);

			u32 value = cpu->load32(addr);
			cpu->cpsr = cpu->spsr();
			if (cpu->in_thumb()) {
				cpu->thumb_jump(value & ~1);
			} else {
				cpu->arm_jump(value & ~3);
			}
			cpsr_written = true;
		}

		if (W) {
			/* If rn is included in register_list:
			 * ARMv4: no writeback
			 * ARMv5: writeback if rn is only register,
			 *        or not the last register
			 */
			if (register_list & (1 << rn)) {
				bool only = register_list == 1 << rn;
				bool last = register_list >> rn == 1;
				if (cpu->is_arm9() && (only || !last)) {
					cpu->gpr[rn] = writeback_value;
				}
			} else {
				cpu->gpr[rn] = writeback_value;
			}
		}

		/* mode switch done after writeback */
		if (cpsr_written) {
			cpu->on_cpsr_write();
		}
	} else {
		if (W) {
			/* If rn is included in register_list:
			 * ARMv4: if rn is first store old base
			 *        else store new base
			 * ARMv5: store old base
			 */
			bool in_rlist = register_list & (1 << rn);
			bool not_first = register_list & ((1 << rn) - 1);
			if (in_rlist && cpu->is_arm7() && not_first) {
				cpu->gpr[rn] = writeback_value;
			}
		}

		if (S == 0) {
			TWICE_ARM_STM_(0, 14, cpu->gpr, 0);
		} else {
			TWICE_ARM_STM_(0, 7, cpu->gpr, 0);

			if (cpu->in_fiq_mode()) {
				TWICE_ARM_STM_(8, 12, cpu->fiqr, -8);
			} else {
				TWICE_ARM_STM_(8, 12, cpu->gpr, 0);
			}

			if (cpu->in_sys_or_usr_mode()) {
				TWICE_ARM_STM_(13, 14, cpu->gpr, 0);
			} else {
				TWICE_ARM_STM_(13, 14, cpu->bankedr[0], -13);
			}
		}

		if (register_list >> 15) {
			cpu->store32(addr, cpu->pc() + 4);
		}

		if (cpu->is_arm7() && register_list == 0) {
			cpu->store32(addr, cpu->pc() + 4);
		}

		if (W) {
			cpu->gpr[rn] = writeback_value;
		}
	}
}

} // namespace twice

#endif
