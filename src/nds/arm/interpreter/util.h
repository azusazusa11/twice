#ifndef TWICE_INTERPRETER_UTIL_H
#define TWICE_INTERPRETER_UTIL_H

#include "nds/arm/arm.h"
#include "nds/arm/arm9.h"

#include "libtwice/exception.h"

namespace twice {

inline void
arm_do_bx(Arm *cpu, u32 addr)
{
	if (addr & 1) {
		cpu->set_t(1);
		cpu->thumb_jump(addr & ~1);
	} else {
		cpu->arm_jump(addr & ~3);
	}
}

inline u32
arm_do_ldr(Arm *cpu, u32 addr)
{
	return std::rotr(cpu->load32(addr & ~3), (addr & 3) << 3);
}

inline void
arm_do_str(Arm *cpu, u32 addr, u32 value)
{
	cpu->store32(addr & ~3, value);
}

inline void
arm_do_strh(Arm *cpu, u32 addr, u16 value)
{
	cpu->store16(addr & ~1, value);
}

inline u8
arm_do_ldrb(Arm *cpu, u32 addr)
{
	return cpu->load8(addr);
}

inline void
arm_do_strb(Arm *cpu, u32 addr, u8 value)
{
	cpu->store8(addr, value);
}

inline s8
arm_do_ldrsb(Arm *cpu, u32 addr)
{
	return cpu->load8(addr);
}

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

} // namespace twice

#endif
