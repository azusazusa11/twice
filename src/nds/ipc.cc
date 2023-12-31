#include "nds/arm/arm.h"
#include "nds/nds.h"

namespace twice {

u32
ipc_fifo_recv(nds_ctx *nds, int cpuid)
{
	auto& src = nds->ipcfifo[cpuid ^ 1];
	auto& dest = nds->ipcfifo[cpuid];

	if (!(dest.cnt & BIT(15))) {
		return dest.fifo[dest.read & 0xF];
	}

	if (dest.size == 0) {
		dest.cnt |= BIT(14);
		return dest.fifo[(dest.read - 1) & 0xF];
	}

	u32 ret = dest.fifo[dest.read++ & 0xF];
	dest.size--;

	if (dest.size == 15) {
		src.cnt &= ~BIT(1);
		dest.cnt &= ~BIT(9);
	}

	if (dest.size == 0) {
		src.cnt |= BIT(0);
		dest.cnt |= BIT(8);
		if (src.cnt & BIT(2)) {
			request_interrupt(nds->cpu[cpuid ^ 1], 17);
		}
	}

	return ret;
}

void
ipc_fifo_send(nds_ctx *nds, int cpuid, u32 value)
{
	auto& src = nds->ipcfifo[cpuid];
	auto& dest = nds->ipcfifo[cpuid ^ 1];

	if (!(src.cnt & BIT(15))) {
		return;
	}

	if (dest.size >= 16) {
		src.cnt |= BIT(14);
		return;
	}

	dest.fifo[dest.write++ & 0xF] = value;
	dest.size++;

	if (dest.size == 16) {
		src.cnt |= BIT(1);
		dest.cnt |= BIT(9);
	}

	if (dest.size == 1) {
		src.cnt &= ~BIT(0);
		dest.cnt &= ~BIT(8);
		if (dest.cnt & BIT(10)) {
			request_interrupt(nds->cpu[cpuid ^ 1], 18);
		}
	}
}

void
ipc_fifo_cnt_write(struct nds_ctx *nds, int cpuid, u16 value)
{
	auto& src = nds->ipcfifo[cpuid];
	auto& dest = nds->ipcfifo[cpuid ^ 1];

	bool send_irq_old = (src.cnt & BIT(2)) && (src.cnt & BIT(0));
	bool recv_irq_old = (src.cnt & BIT(10)) && !(src.cnt & BIT(8));

	src.cnt = (src.cnt & ~0x8404) | (value & 0x8404);
	src.cnt &= ~(value & BIT(14));

	if (value & BIT(3)) {
		dest.fifo.fill(0);
		dest.read = 0;
		dest.write = 0;
		dest.size = 0;

		src.cnt &= ~BIT(1);
		dest.cnt &= ~BIT(9);

		src.cnt |= BIT(0);
		dest.cnt |= BIT(8);
	}

	bool send_irq = (src.cnt & BIT(2)) && (src.cnt & BIT(0));
	bool recv_irq = (src.cnt & BIT(10)) && !(src.cnt & BIT(8));

	if (!send_irq_old && send_irq) {
		request_interrupt(nds->cpu[cpuid], 17);
	}
	if (!recv_irq_old && recv_irq) {
		request_interrupt(nds->cpu[cpuid], 18);
	}
}

void
ipcsync_write(nds_ctx *nds, int cpuid, u16 value)
{
	nds->ipcsync[cpuid ^ 1] &= ~0xF;
	nds->ipcsync[cpuid ^ 1] |= value >> 8 & 0xF;
	nds->ipcsync[cpuid] &= ~0x4F00;
	nds->ipcsync[cpuid] |= value & 0x4F00;

	if ((value & BIT(13)) && (nds->ipcsync[cpuid ^ 1] & BIT(14))) {
		request_interrupt(nds->cpu[cpuid ^ 1], 16);
	}
}

} // namespace twice
