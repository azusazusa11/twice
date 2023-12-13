#include "nds/timer.h"

#include "nds/arm/arm.h"
#include "nds/nds.h"

namespace twice {

static void update_timer_counter(nds_ctx *, int cpuid, int timer_id);
static void update_cascade_counter(nds_ctx *, int cpuid, int timer_id);
static void stop_timer(nds_ctx *, int cpuid, int timer_id);
static void reload_timer_counter(nds_ctx *, int cpuid, int timer_id);
static void schedule_timer_overflow(nds_ctx *, int cpuid, int timer_id);

static const int timer_scale_shift[4] = { 10, 4, 2, 0 };

u16
read_timer_counter(nds_ctx *nds, int cpuid, int timer_id)
{
	update_timer_counter(nds, cpuid, timer_id);
	return nds->tmr[cpuid][timer_id].counter >> 10;
}

void
write_timer_ctrl(nds_ctx *nds, int cpuid, int timer_id, u8 value)
{
	auto& t = nds->tmr[cpuid][timer_id];
	bool old_enabled = t.ctrl & BIT(7);
	bool enabled = value & BIT(7);
	bool cascade = value & BIT(2);

	if (!old_enabled && !enabled) {
		;
	} else if (!old_enabled) {
		reload_timer_counter(nds, cpuid, timer_id);
	} else if (!enabled) {
		update_timer_counter(nds, cpuid, timer_id);
		stop_timer(nds, cpuid, timer_id);
	} else {
		update_timer_counter(nds, cpuid, timer_id);
		if (cascade) {
			stop_timer(nds, cpuid, timer_id);
		}
	}

	t.ctrl = value & 0xC7;
	t.shift = timer_scale_shift[value & 3];

	if (enabled && !cascade) {
		schedule_timer_overflow(nds, cpuid, timer_id);
	}
}

static void
on_timer_overflow(nds_ctx *nds, int cpuid, int timer_id)
{
	auto& t = nds->tmr[cpuid][timer_id];
	bool irq_enabled = t.ctrl & BIT(6);
	bool cascade = t.ctrl & BIT(2);

	reload_timer_counter(nds, cpuid, timer_id);

	if (irq_enabled) {
		request_interrupt(nds->cpu[cpuid], 3 + timer_id);
	}

	if (timer_id != 3) {
		update_cascade_counter(nds, cpuid, timer_id + 1);
	}

	if (!cascade) {
		schedule_timer_overflow(nds, cpuid, timer_id);
	}
}

void
event_timer_overflow(nds_ctx *nds, int cpuid, intptr_t timer_id)
{
	on_timer_overflow(nds, cpuid, timer_id);
}

static void
update_timer_counter(nds_ctx *nds, int cpuid, int timer_id)
{
	auto& t = nds->tmr[cpuid][timer_id];
	bool enabled = t.ctrl & BIT(7);
	bool cascade = t.ctrl & BIT(2);

	if (enabled && !cascade) {
		timestamp current_time = nds->arm_cycles[cpuid];
		timestamp elapsed = current_time - t.last_update;
		if (cpuid == 0) {
			elapsed >>= 1;
		}

		t.counter += elapsed << t.shift;
		t.last_update = current_time;
	}
}

static void
update_cascade_counter(nds_ctx *nds, int cpuid, int timer_id)
{
	auto& t = nds->tmr[cpuid][timer_id];
	bool enabled = t.ctrl & BIT(7);
	bool cascade = t.ctrl & BIT(2);

	if (enabled && cascade) {
		t.counter += 1 << 10;

		if (t.counter >> 10 >= 0x10000) {
			on_timer_overflow(nds, cpuid, timer_id);
		}
	}
}

static void
stop_timer(nds_ctx *nds, int cpuid, int timer_id)
{
	cancel_cpu_event(nds, cpuid,
			event_scheduler::TIMER0_OVERFLOW + timer_id);
}

static void
reload_timer_counter(nds_ctx *nds, int cpuid, int timer_id)
{
	auto& t = nds->tmr[cpuid][timer_id];
	t.counter = (u32)t.reload_val << 10;
	t.last_update = nds->arm_cycles[cpuid];
}

static void
schedule_timer_overflow(nds_ctx *nds, int cpuid, int timer_id)
{
	auto& t = nds->tmr[cpuid][timer_id];
	timestamp dt = (((u32)0x10000 << 10) - (t.counter & MASK(26))) >>
	               t.shift;

	schedule_cpu_event_after(nds, cpuid,
			event_scheduler::TIMER0_OVERFLOW + timer_id, dt);
}

} // namespace twice
