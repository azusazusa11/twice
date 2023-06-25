#include "nds/nds.h"

namespace twice {

Scheduler::Scheduler()
{
	events[HBLANK_START].cb = nds_event_hblank_start;
	events[HBLANK_END].cb = nds_event_hblank_end;
	arm_events[0][START_IMMEDIATE_DMAS].cb = start_immediate_dmas;
	arm_events[1][START_IMMEDIATE_DMAS].cb = start_immediate_dmas;
}

u64
get_next_event_time(NDS *nds)
{
	auto& sc = nds->scheduler;

	u64 time = sc.current_time + 64;

	for (int i = 0; i < Scheduler::NUM_EVENTS; i++) {
		if (sc.events[i].enabled) {
			time = std::min(time, sc.events[i].time);
		}
	}

	for (int i = 0; i < Scheduler::NUM_ARM_EVENTS; i++) {
		if (sc.arm_events[0][i].enabled) {
			time = std::min(time, sc.arm_events[0][i].time);
		}
		if (sc.arm_events[1][i].enabled) {
			time = std::min(time, sc.arm_events[1][i].time << 1);
		}
	}

	return time;
}

void
schedule_event(NDS *nds, int event, u64 t)
{
	nds->scheduler.events[event].enabled = true;
	nds->scheduler.events[event].time = t << 1;
}

void
reschedule_event_after(NDS *nds, int event, u64 dt)
{
	nds->scheduler.events[event].enabled = true;
	nds->scheduler.events[event].time += dt << 1;
}

void
schedule_arm_event_after(NDS *nds, int cpuid, int event, u64 dt)
{
	if (cpuid == 0) {
		dt <<= 1;
	}

	u64 event_time = nds->arm_cycles[cpuid] + dt;

	if (event_time < nds->arm_target_cycles[cpuid]) {
		nds->arm_target_cycles[cpuid] = event_time;
	}

	nds->scheduler.arm_events[cpuid][event].enabled = true;
	nds->scheduler.arm_events[cpuid][event].time = event_time;
}

void
run_events(NDS *nds)
{
	auto& sc = nds->scheduler;

	for (int i = 0; i < Scheduler::NUM_EVENTS; i++) {
		auto& event = sc.events[i];
		bool expired = (s64)(sc.current_time - event.time) >= 0;
		if (event.enabled && expired) {
			event.enabled = false;
			if (event.cb) {
				event.cb(nds);
			}
		}
	}
}

void
run_arm_events(NDS *nds, int cpuid)
{
	auto& sc = nds->scheduler;

	for (int i = 0; i < Scheduler::NUM_ARM_EVENTS; i++) {
		auto& event = sc.arm_events[cpuid][i];
		bool expired = (s64)(nds->arm_cycles[cpuid] - event.time) >= 0;
		if (event.enabled && expired) {
			event.enabled = false;
			if (event.cb) {
				event.cb(nds, cpuid);
			}
		}
	}
}

} // namespace twice
