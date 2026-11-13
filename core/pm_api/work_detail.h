// BufferDetail (bench.h) and RenderInfo (engine.h) to the C API's
// pm_work_detail (pm_api.h): the same counts for a benchmark row, the live
// meter's worst buffer and an audio-thread scene run.
#pragma once

#include "pm_api/pm_api.h"
#include "pm_bench/bench.h"
#include "pm_host/cpu_id.h"
#include "pm_host/load_meter.h"

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

// Per-CPU totals (cpu_id.h) to pm_cpu_usage: sums become means.
inline void toCpuUsage(const host::CpuUsage& c, pm_cpu_usage& u) noexcept {
    u = pm_cpu_usage{};
    u.buffers_tracked = c.tracked;
    u.buffers_unknown = c.unknown;
    u.migrations = c.migrations;
    u.worst_cpu_start = c.worstStart;
    u.worst_cpu_end = c.worstEnd;
    static_assert(PM_MAX_CPUS == host::kMaxCpus, "CPU table out of date");
    for (int i = 0; i < host::kMaxCpus; ++i) {
        u.buffers[i] = c.buffers[i];
        u.mean_render_us[i] = c.buffers[i] ? c.renderUs[i] / c.buffers[i] : 0.0;
        u.probe_runs[i] = c.probeRuns[i];
        u.probe_latency_us[i] = c.probeRuns[i] ? c.probeLatencyUs[i] / c.probeRuns[i] : 0.0;
        u.probe_throughput_us[i] = c.probeRuns[i] ? c.probeThroughputUs[i] / c.probeRuns[i] : 0.0;
    }
}

inline void toProbeSeries(const host::ProbeSeries& p, pm_probe_series& o) noexcept {
    o.runs = p.count;
    o.min_us = p.minUs;
    o.median_us = p.medianUs;
    o.mean_us = p.meanUs;
    o.max_us = p.maxUs;
}

inline void toProbeSeries(const ScenarioResult::ProbeStats& p, pm_probe_series& o) noexcept {
    o.runs = static_cast<std::uint64_t>(p.runs);
    o.min_us = p.minUs;
    o.median_us = p.medianUs;
    o.mean_us = p.meanUs;
    o.max_us = p.maxUs;
}

}  // namespace pm::bench
