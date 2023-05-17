#include "nds/nds.h"

namespace twice {

Scheduler::Scheduler()
{
	events[HBLANK_START].cb = gpu_on_hblank_start;
	events[HBLANK_END].cb = gpu_on_hblank_end;
}

void
Scheduler::schedule_event(int event, u64 t)
{
	events[event].enabled = true;
	events[event].time = t;
}

void
Scheduler::reschedule_event_after(int event, u64 t)
{
	events[event].enabled = true;
	events[event].time += t;
}

u64
Scheduler::get_next_event_time()
{
	u64 time = current_time + 32;

	for (int i = 0; i < NUM_EVENTS; i++) {
		if (events[i].enabled) {
			time = std::min(time, events[i].time);
		}
	}

	return time;
}

void
run_events(NDS *nds)
{
	auto& sc = nds->scheduler;

	for (int i = 0; i < Scheduler::NUM_EVENTS; i++) {
		if (sc.events[i].enabled &&
				sc.current_time >= sc.events[i].time) {
			sc.events[i].enabled = false;
			sc.events[i].cb(nds);
		}
	}
}

} // namespace twice