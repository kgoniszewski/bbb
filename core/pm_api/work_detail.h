// BufferDetail (bench.h) and RenderInfo (engine.h) to the C API's
// pm_work_detail (pm_api.h): the same counts for a benchmark row, the live
// meter's worst buffer and an audio-thread scene run.
#pragma once

#include "pm_api/pm_api.h"
#include "pm_bench/bench.h"

namespace pm::bench {

inline void toWorkDetail(const BufferDetail& d, pm_work_detail& w) noexcept {
    w = pm_work_detail{};
    w.note_ons = d.noteOns;
    w.note_offs = d.noteOffs;
    w.pedal_events = d.pedalEvents;
    w.awake_modes = d.awakeModes;
    w.hammers_in_contact = d.hammersInContact;
    w.max_hammers_in_contact = d.maxHammersInContact;
    w.hammer_contact_samples = d.hammerContactSamples;
    w.hammer_samples = d.hammerSamples;
    w.hammer_damped_samples = d.hammerDampedSamples;
    w.newton_evals = d.newtonEvals;
    w.fallback_solves = d.fallbackSolves;
    w.damper_updates = d.damperUpdates;
    w.response_groups = d.responseGroups;
    w.kernel_rebuilds = d.kernelRebuilds;
    w.kernel_key_updates = d.kernelKeyUpdates;
    w.free_group_blocks = d.freeGroupBlocks;
    w.forced_group_blocks = d.forcedGroupBlocks;
    w.loop_members = d.loopMembers;
    w.loop_listeners = d.loopListeners;
    w.loop_entries = d.loopEntries;
    w.out_zones = d.outZones;
}

}  // namespace pm::bench
