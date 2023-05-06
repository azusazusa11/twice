#ifndef TWICE_ARM7_H
#define TWICE_ARM7_H

#include "nds/arm/arm.h"

namespace twice {

struct Arm7 final : Arm {
	Arm7(NDS *nds);

	void jump(u32 addr) override;
	void arm_jump(u32 addr) override;
	void thumb_jump(u32 addr) override;
	u32 fetch32(u32 addr) override;
	u16 fetch16(u32 addr) override;
	u32 load32(u32 addr) override;
	u16 load16(u32 addr) override;
	u8 load8(u32 addr) override;
	void store32(u32 addr, u32 value) override;
	void store16(u32 addr, u16 value) override;
	void store8(u32 addr, u8 value) override;

	void step();
};

} // namespace twice

#endif
