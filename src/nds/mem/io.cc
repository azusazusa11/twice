#include "nds/mem/io.h"

#include "common/logger.h"

#include "libtwice/exception.h"

namespace twice {

u8 gba_rom_nonseq_timings[4] = { 10, 8, 6, 18 };
u8 gba_rom_seq_timings[2] = { 6, 4 };

void
exmem_write(nds_ctx *nds, int cpuid, u16 value)
{
	nds->exmem[cpuid] = (nds->exmem[cpuid] & ~0x3F) | (value & 0x3F);
	if (cpuid == 0) {
		nds->exmem[0] = (nds->exmem[0] & ~0x8880) | (value & 0x8880);
		nds->exmem[1] = (nds->exmem[1] & ~0x8880) | (value & 0x8880);
		nds->gba_slot_cpu = nds->exmem[0] >> 7 & 1;
		nds->nds_slot_cpu = nds->exmem[0] >> 11 & 1;
	}
}

} // namespace twice
