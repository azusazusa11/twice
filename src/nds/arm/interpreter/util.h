#ifndef TWICE_INTERPRETER_UTIL_H
#define TWICE_INTERPRETER_UTIL_H

#include "nds/arm/arm.h"

#include "common/logger.h"
#include "libtwice/exception.h"

namespace twice::arm::interpreter {

inline u32
get_mul_internal_cycles(u32 rs)
{
	int leading = std::countl_zero(rs);
	if (leading < 8) {
		return 4;
	} else if (leading < 16) {
		return 3;
	} else if (leading < 24) {
		return 2;
	} else {
		return 1;
	}
}

inline bool
in_privileged_mode(arm_cpu *cpu)
{
	return cpu->mode != arm_cpu::MODE_USR;
}

inline bool
in_sys_or_usr_mode(arm_cpu *cpu)
{
	return cpu->mode == arm_cpu::MODE_SYS ||
	       cpu->mode == arm_cpu::MODE_USR;
}

inline bool
in_fiq_mode(arm_cpu *cpu)
{
	return cpu->mode == arm_cpu::MODE_FIQ;
}

inline bool
current_mode_has_spsr(arm_cpu *cpu)
{
	return !in_sys_or_usr_mode(cpu);
}

inline bool
get_c(arm_cpu *cpu)
{
	return cpu->cpsr & BIT(29);
}

inline void
set_q(arm_cpu *cpu, bool q)
{
	cpu->cpsr = (cpu->cpsr & ~BIT(27)) | (q << 27);
}

inline void
set_nz(arm_cpu *cpu, bool n, bool z)
{
	cpu->cpsr &= ~0xC0000000;
	cpu->cpsr |= n << 31;
	cpu->cpsr |= z << 30;
}

inline void
set_nzc(arm_cpu *cpu, bool n, bool z, bool c)
{
	cpu->cpsr &= ~0xE0000000;
	cpu->cpsr |= n << 31;
	cpu->cpsr |= z << 30;
	cpu->cpsr |= c << 29;
}

inline void
set_nzcv(arm_cpu *cpu, bool n, bool z, bool c, bool v)
{
	cpu->cpsr &= ~0xF0000000;
	cpu->cpsr |= n << 31;
	cpu->cpsr |= z << 30;
	cpu->cpsr |= c << 29;
	cpu->cpsr |= v << 28;
}

inline bool
is_arm7(arm_cpu *cpu)
{
	return cpu->cpuid;
}

inline bool
is_arm9(arm_cpu *cpu)
{
	return !is_arm7(cpu);
}

inline void
arm_do_bx(arm_cpu *cpu, u32 addr)
{
	if (addr & 1) {
		cpu->cpsr |= 0x20;
		cpu->thumb_jump(addr & ~1);
	} else {
		cpu->arm_jump(addr & ~3);
	}
}

inline void
thumb_do_bx(arm_cpu *cpu, u32 addr)
{
	if (!(addr & 1)) {
		cpu->cpsr &= ~0x20;
		cpu->arm_jump(addr & ~3);
	} else {
		cpu->thumb_jump(addr & ~1);
	}
}

inline u32
arm_do_ldr(arm_cpu *cpu, u32 addr)
{
	return std::rotr(cpu->load32n(addr & ~3), (addr & 3) << 3);
}

inline void
arm_do_str(arm_cpu *cpu, u32 addr, u32 value)
{
	cpu->store32n(addr & ~3, value);
}

inline void
arm_do_strh(arm_cpu *cpu, u32 addr, u16 value)
{
	cpu->store16n(addr & ~1, value);
}

inline u8
arm_do_ldrb(arm_cpu *cpu, u32 addr)
{
	return cpu->load8n(addr);
}

inline void
arm_do_strb(arm_cpu *cpu, u32 addr, u8 value)
{
	cpu->store8n(addr, value);
}

inline s8
arm_do_ldrsb(arm_cpu *cpu, u32 addr)
{
	return cpu->load8n(addr);
}

#define TWICE_ARM_LDM_(start_, end_, arr_, off_)                              \
	do {                                                                  \
		for (int i = (start_); i <= (end_); i++) {                    \
			if (register_list & BIT(i)) {                         \
				(arr_)[i + (off_)] =                          \
						cpu->load32(addr, nonseq);    \
				nonseq = false;                               \
				addr += 4;                                    \
			}                                                     \
		}                                                             \
	} while (0)

#define TWICE_ARM_STM_(start_, end_, arr_, off_)                              \
	do {                                                                  \
		for (int i = (start_); i <= (end_); i++) {                    \
			if (register_list & BIT(i)) {                         \
				cpu->store32(addr, (arr_)[i + (off_)],        \
						nonseq);                      \
				nonseq = false;                               \
				addr += 4;                                    \
			}                                                     \
		}                                                             \
	} while (0)

#define SUB_FLAGS_(a_, b_)                                                    \
	do {                                                                  \
		carry = !((a_) < (b_));                                       \
		overflow = (((a_) ^ (b_)) & ((a_) ^ r)) >> 31;                \
	} while (0)

#define SBC_FLAGS_(a_, b_)                                                    \
	do {                                                                  \
		carry = !(r64 < 0);                                           \
		overflow = (((a_) ^ (b_)) & ((a_) ^ r)) >> 31;                \
	} while (0)

#define ADD_FLAGS_(a_, b_)                                                    \
	do {                                                                  \
		carry = r < (a_);                                             \
		overflow = (((a_) ^ r) & ((b_) ^ r)) >> 31;                   \
	} while (0)

#define ADC_FLAGS_(a_, b_)                                                    \
	do {                                                                  \
		carry = r64 >> 32;                                            \
		overflow = (((a_) ^ r) & ((b_) ^ r)) >> 31;                   \
	} while (0)

} // namespace twice::arm::interpreter

#endif
