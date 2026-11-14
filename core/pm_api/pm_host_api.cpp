// C API of the app host (pm_api.h, "App host"), over pm::host::Host.
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstring>
#include <ctime>
#include <new>
#include <vector>

#include "pm_api/pm_api.h"
#include "pm_api/pm_api_internal.h"
#include "pm_api/work_detail.h"
#include "pm_bench/bench.h"
#include "pm_host/cpu_id.h"
#include "pm_host/host.h"
#include "pm_host/speed_probe.h"

struct pm_host {
    pm_engine* engine = nullptr;
    pm::host::Host core;
    // Written by pm_host_prepare inside the gate, read by the audio thread
    // after the gate reopens (ordered by the gate's atomics).
    double sampleRate = 0.0;
    double ticksPerSample = 0.0;
    std::atomic<double> publishedRate{0.0};
    std::atomic<int32_t> lastFrames{0};
    std::atomic<bool> cpuTiming{false};
    std::uint32_t numer = 1, denom = 1;
    // Speed probe: every probeEvery-th render call (0: off); the counter
    // and the checksum sink belong to the audio thread.
    std::atomic<int32_t> probeEvery{0};
    int32_t probeCounter = 0;
    std::atomic<float> probeSink{0.0f};

    // Scene player (pm_host_scene_start). Set up inside the prepare gate;
    // from then on the audio thread owns 'scene' until it publishes the
    // result (sceneState DONE, release) and the main thread reads it
    // (acquire). The event list is allocated by pm_host_scene_start only.
    struct Scene {
        std::vector<pm::Event> events;
        std::size_t next = 0;
        double warmSeconds = 0.0, seconds = 0.0;
        int warmBuffers = -1, measureBuffers = 0;  // fixed at the first buffer (from its frame count)
        int buffer = 0;                            // buffers rendered since the start
        bool restoreTiming = false;
        bool restoreProbe = false;                 // the probe was switched on for the run
        // CPU trace totals (pm_scene_load) over the measured buffers.
        std::uint64_t traced = 0, migrated = 0;
        double subBlocks = 0.0, usBefore = 0.0, subBlocksBefore = 0.0;
        std::uint32_t endCpu[PM_MAX_CPUS] = {}, movedTo[PM_MAX_CPUS] = {};
        std::uint32_t leftAfter[PM_SCENE_LEFT_BINS] = {};
        // Totals over the measured buffers.
        int measured = 0, overruns = 0, minFrames = 0, maxFrames = 0;
        double awake = 0.0, freeBlocks = 0.0, forcedBlocks = 0.0, members = 0.0, listeners = 0.0;
        double rebuilds = 0.0, keyUpdates = 0.0, hammerUs = 0.0;
        long hammerSamples = 0, contactSamples = 0;
    } scene;
    std::atomic<int32_t> sceneState{PM_SCENE_IDLE};
    std::atomic<bool> sceneCancel{false};
    pm_host_scene_result sceneResult{};
    pm_cpu_usage sceneCpu{};            // published with sceneResult
    pm_speed_probe_split sceneProbe{};
    char sceneConfig[4096] = {};
    // PM_BENCH_SCENE_FRESH_ENGINE: the private engine (owned; replaced or
    // freed only inside the prepare gate) and whether the audio thread
    // renders it (audio-thread state, like 'scene').
    pm::Engine* sceneEngine = nullptr;
    bool sceneFresh = false;
    // Load multiplier: the other K - 1 engines (owned, benchmark-prepared;
    // replaced or freed only inside the prepare gate), their scratch output
    // and the number in use (audio-thread state, like 'scene').
    pm::Engine* loadEngines[PM_SCENE_MAX_LOAD - 1] = {};
    std::vector<float> loadScratch;  // 2 x kScratchFrames
    static constexpr int kScratchFrames = 1024;
    int sceneLoad = 1;
    std::uint64_t engineBytes = 0;   // of one benchmark-prepared engine (0: not measured)
    pm_scene_load sceneLoadResult{};  // published with sceneResult
    // CPU trace of the current buffer (audio thread): the CPU at the start
    // of the render call, the sub-blocks begun so far over all engines and
    // the first one found on another CPU.
    struct Trace {
        bool armed = false;
        int cpuStart = -1, index = 0, first = -1, movedTo = -1;
        std::chrono::steady_clock::time_point t0{};
        double usAtChange = 0.0;
    } trace;
};

namespace {
static_assert(sizeof(pm_event) == sizeof(pm::Event), "pm_event must mirror pm::Event");

// CPU time of the calling thread in seconds, or -1 where the clock does not
// exist. A plain kernel call (Darwin: a trap reading the thread's own
// timers; Linux: clock_gettime), no allocation and no lock it could wait on.
double threadCpuSeconds() noexcept {
#if defined(CLOCK_THREAD_CPUTIME_ID)
    timespec ts;
    if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts) == 0)
        return static_cast<double>(ts.tv_sec) + 1e-9 * static_cast<double>(ts.tv_nsec);
#endif
    return -1.0;
}

// Engine sub-block hook (audio thread): until the first change in the
// buffer, one CPU-number read per sub-block (cpu_id.h: a per-CPU register
// read) and, at the change, one clock read.
void traceSubBlock(void* ctx) noexcept {
    pm_host::Trace& t = static_cast<pm_host*>(ctx)->trace;
    if (!t.armed) return;
    const int i = t.index++;
    if (t.first >= 0) return;
    const int c = pm::host::currentCpu();
    if (c >= 0 && c != t.cpuStart) {
        t.first = i;
        t.movedTo = c;
        t.usAtChange = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t.t0).count();
    }
}

void toSplit(const pm::host::ProbeSnapshot& p, pm_speed_probe_split& o) noexcept {
    o = pm_speed_probe_split{};
    pm::bench::toProbeSeries(p.latency, o.latency);
    pm::bench::toProbeSeries(p.throughput, o.throughput);
}

void toWork(const pm::host::WorstBuffer& b, pm_work_detail& w) noexcept {
    w = pm_work_detail{};
    w.note_ons = b.noteOns;
    w.note_offs = b.noteOffs;
    w.pedal_events = b.pedalEvents;
    w.awake_modes = b.awakeModes;
    w.hammers_in_contact = b.hammersInContact;
    w.max_hammers_in_contact = b.maxHammersInContact;
    w.hammer_contact_samples = b.hammerContactSamples;
    w.hammer_samples = b.hammerSamples;
    w.hammer_damped_samples = b.hammerDampedSamples;
    w.newton_evals = b.newtonEvals;
    w.fallback_solves = b.fallbackSolves;
    w.damper_updates = b.damperUpdates;
    w.response_groups = b.responseGroups;
    w.kernel_rebuilds = b.kernelRebuilds;
    w.kernel_key_updates = b.kernelKeyUpdates;
    w.free_group_blocks = b.freeGroupBlocks;
    w.forced_group_blocks = b.forcedGroupBlocks;
    w.loop_members = b.loopMembers;
    w.loop_listeners = b.loopListeners;
    w.loop_entries = b.loopEntries;
    w.out_zones = b.outZones;
    w.groups_woken = b.groupsWoken;
    w.approx_responses = b.approxResponses;
}

// Audio thread, before the buffer's events are forwarded: starts the
// measurement after the warm-up and pushes the scene events due in this
// buffer (sample times from the reset, i.e. the engine's own clock).
// Audio thread, when the scene ends (done or cancelled): undoes what the
// start switched on, and returns the render path to the app's engine.
void sceneFinish(pm_host* host, pm::Engine& eng) noexcept {
    pm_host::Scene& sc = host->scene;
    if (sc.restoreTiming) eng.setPhaseTiming(false);
    if (sc.restoreProbe) host->probeEvery.store(0, std::memory_order_relaxed);
    if (pm::Engine* app = pmEngineOf(host->engine)) app->setSubBlockHook(nullptr, nullptr);  // this thread renders it
    host->trace.armed = false;
    host->sceneFresh = false;
    host->sceneLoad = 1;
}

void sceneBeforeRender(pm_host* host, pm::Engine& eng, int32_t frames) noexcept {
    pm_host::Scene& sc = host->scene;
    if (host->sceneCancel.exchange(false, std::memory_order_acq_rel)) {
        sceneFinish(host, eng);
        host->sceneState.store(PM_SCENE_IDLE, std::memory_order_release);
        return;
    }
    if (sc.warmBuffers < 0) {
        // As the benchmark: whole buffers of warm-up and of measurement.
        const double fs = eng.sampleRate();
        sc.warmBuffers = static_cast<int>(sc.warmSeconds * fs / frames);
        sc.measureBuffers = std::max(1, static_cast<int>(sc.seconds * fs / frames));
    }
    if (sc.buffer == sc.warmBuffers) host->core.meter().requestReset();  // applied at this buffer's first record
    const std::uint64_t t1 = eng.sampleTime() + static_cast<std::uint64_t>(frames);
    for (; sc.next < sc.events.size() && sc.events[sc.next].time < t1; ++sc.next) {
        eng.pushEvent(sc.events[sc.next]);
        for (int i = 0; i + 1 < host->sceneLoad; ++i) host->loadEngines[i]->pushEvent(sc.events[sc.next]);
    }
}

// Audio thread, after the buffer was recorded in the meter.
void sceneAfterRender(pm_host* host, pm::Engine& eng, int32_t frames, double renderUs, double periodUs,
                      int cpuEnd) noexcept {
    pm_host::Scene& sc = host->scene;
    if (host->sceneState.load(std::memory_order_relaxed) != PM_SCENE_RUNNING) return;
    const int b = sc.buffer++;
    if (b < sc.warmBuffers) {
        host->trace.armed = false;
        return;
    }
    const pm::RenderInfo& info = eng.lastRender();
    ++sc.measured;
    pm_host::Trace& tr = host->trace;
    if (tr.armed && tr.cpuStart >= 0 && tr.cpuStart < PM_MAX_CPUS && cpuEnd >= 0 && cpuEnd < PM_MAX_CPUS) {
        ++sc.traced;
        ++sc.endCpu[cpuEnd];
        sc.subBlocks += tr.index;
        int at = -1, to = -1;
        double us = 0.0;
        if (tr.first >= 0) {
            at = tr.first;
            to = tr.movedTo;
            us = tr.usAtChange;
        } else if (cpuEnd != tr.cpuStart) {
            at = tr.index;  // seen only at the end of the call
            to = cpuEnd;
            us = renderUs;
        }
        if (at >= 0) {
            ++sc.migrated;
            ++sc.leftAfter[std::min(at, PM_SCENE_LEFT_BINS - 1)];
            if (to >= 0 && to < PM_MAX_CPUS) ++sc.movedTo[to];
            sc.usBefore += us;
            sc.subBlocksBefore += at;
        }
    }
    if (renderUs > periodUs) ++sc.overruns;
    if (sc.minFrames == 0 || frames < sc.minFrames) sc.minFrames = frames;
    sc.maxFrames = std::max(sc.maxFrames, static_cast<int>(frames));
    sc.awake += info.awakeModes;
    sc.freeBlocks += info.freeGroupBlocks;
    sc.forcedBlocks += info.forcedGroupBlocks;
    sc.members += info.loopMembers;
    sc.listeners += info.loopListeners;
    sc.rebuilds += info.kernelRebuilds;
    sc.keyUpdates += info.kernelKeyUpdates;
    sc.hammerSamples += info.hammerSamples;
    sc.contactSamples += info.hammerContactSamples;
    if (eng.phaseTiming()) sc.hammerUs += eng.profile().us[pm::EngineProfile::Hammers];
    host->trace.armed = false;
    if (sc.measured < sc.measureBuffers) return;
    // Done: publish (the snapshot reads the meter's histograms; no
    // allocation, no lock).
    const pm::host::LoadSnapshot s = host->core.meter().snapshot();
    pm_host_scene_result& r = host->sceneResult;
    r = pm_host_scene_result{};
    r.state = PM_SCENE_DONE;
    r.buffers = sc.measured;
    r.sample_rate = eng.sampleRate();
    r.min_frames = sc.minFrames;
    r.max_frames = sc.maxFrames;
    r.period_us = periodUs;
    r.mean_us = s.meanUs;
    r.p99_us = s.p99Us;
    r.max_us = s.maxUs;
    r.mean_load = s.meanLoad;
    r.p99_load = s.p99Load;
    r.max_load = s.maxLoad;
    const double n = sc.measured;
    r.mean_awake_modes = sc.awake / n;
    r.free_group_blocks = sc.freeBlocks / n;
    r.forced_group_blocks = sc.forcedBlocks / n;
    r.loop_members = sc.members / n;
    r.loop_listeners = sc.listeners / n;
    r.kernel_rebuilds = sc.rebuilds / n;
    r.kernel_key_updates = sc.keyUpdates / n;
    if (sc.hammerSamples > 0) r.hammer_us_per_sample = sc.hammerUs / static_cast<double>(sc.hammerSamples);
    if (sc.contactSamples > 0) r.hammer_us_per_contact_sample = sc.hammerUs / static_cast<double>(sc.contactSamples);
    r.overruns = sc.overruns;
    r.has_phases = s.timedBuffers > 0 ? 1 : 0;
    r.top_count = static_cast<int32_t>(s.topCount);
    r.worst_timed = s.worst.timed ? 1 : 0;
    for (int p = 0; p < PM_PHASE_COUNT; ++p) {
        r.worst_phase_us[p] = s.worst.phaseUs[p];
        r.top_phase_us[p] = s.topPhaseUs[p];
        r.mean_phase_us[p] = s.meanPhaseUs[p];
    }
    toWork(s.worst, r.worst);
    r.worst_us = s.worst.renderUs;
    r.probe.interval = host->probeEvery.load(std::memory_order_relaxed);
    r.probe.runs = s.probe.count;
    r.probe.min_us = s.probe.minUs;
    r.probe.median_us = s.probe.medianUs;
    r.probe.mean_us = s.probe.meanUs;
    r.probe.max_us = s.probe.maxUs;
    r.fresh_engine = host->sceneFresh ? 1 : 0;
    pm::bench::toCpuUsage(s.cpu, host->sceneCpu);
    toSplit(s.probe, host->sceneProbe);
    pm_scene_load& L = host->sceneLoadResult;
    L = pm_scene_load{};
    L.load = host->sceneLoad;
    L.engine_bytes = host->engineBytes;
    L.cpu_tracked = sc.traced > 0 ? 1 : 0;
    L.buffers = sc.traced;
    L.migrated = sc.migrated;
    if (sc.traced > 0) L.mean_sub_blocks = sc.subBlocks / static_cast<double>(sc.traced);
    if (sc.migrated > 0) {
        L.mean_us_before_migration = sc.usBefore / static_cast<double>(sc.migrated);
        L.mean_sub_blocks_before_migration = sc.subBlocksBefore / static_cast<double>(sc.migrated);
    }
    for (int i = 0; i < PM_MAX_CPUS; ++i) {
        L.end_cpu[i] = sc.endCpu[i];
        L.moved_to[i] = sc.movedTo[i];
    }
    for (int i = 0; i < PM_SCENE_LEFT_BINS; ++i) L.left_after[i] = sc.leftAfter[i];
    sceneFinish(host, eng);
    host->sceneState.store(PM_SCENE_DONE, std::memory_order_release);
}

void toEntry(const pm::host::MonitorEntry& in, pm_midi_monitor_entry& out) {
    const pm::host::MidiMessage& m = in.message;
    out.host_time = in.hostTime;
    out.source_id = in.sourceId;
    out.kind = static_cast<uint8_t>(m.kind);
    out.channel = m.channel;
    out.status = m.status;
    out.data1 = m.data1;
    out.data2 = m.data2;
    out.group = m.group;
    out.protocol = m.protocol;
    out.word_count = m.wordCount;
    out.value = m.value;
    out.length = m.length;
    for (int i = 0; i < 4; ++i) out.words[i] = m.words[i];
}
}  // namespace

extern "C" {

pm_host* pm_host_create(void) {
    pm_host* h = new (std::nothrow) pm_host();
    if (!h) return nullptr;
    h->engine = pm_engine_create();
    if (!h->engine) {
        delete h;
        return nullptr;
    }
    return h;
}

void pm_host_destroy(pm_host* host) {
    if (!host) return;
    delete host->sceneEngine;
    for (pm::Engine* e : host->loadEngines) delete e;
    pm_engine_destroy(host->engine);
    delete host;
}

pm_engine* pm_host_engine(pm_host* host) { return host ? host->engine : nullptr; }

void pm_host_set_timebase(pm_host* host, uint32_t numer, uint32_t denom) {
    if (!host || numer == 0 || denom == 0) return;
    host->numer = numer;
    host->denom = denom;
}

double pm_host_ticks_per_sample(double sample_rate, uint32_t numer, uint32_t denom) {
    return pm::hostTicksPerSample(sample_rate, numer, denom);
}

int pm_host_prepare(pm_host* host, double sample_rate, int32_t max_frames) {
    if (!host || !(sample_rate > 0.0) || max_frames <= 0) return 0;
    return host->core.runExclusive([&] {
               if (!pm_engine_prepare(host->engine, sample_rate, max_frames)) return false;
               host->sampleRate = sample_rate;
               host->ticksPerSample = pm::hostTicksPerSample(sample_rate, host->numer, host->denom);
               host->publishedRate.store(sample_rate, std::memory_order_relaxed);
               return true;
           })
               ? 1
               : 0;
}

int pm_host_is_prepared(const pm_host* host) { return host && host->core.ready() ? 1 : 0; }

double pm_host_sample_rate(const pm_host* host) {
    return host ? host->publishedRate.load(std::memory_order_relaxed) : 0.0;
}

void pm_host_render(pm_host* host, float* left, float* right, int32_t frames, uint64_t host_time) {
    if (!host || !left || !right || frames <= 0) return;
    if (!host->core.enterRender()) {
        for (int32_t i = 0; i < frames; ++i) left[i] = right[i] = 0.0f;
        return;
    }
    pm_engine* engine = host->engine;
    pm::Engine* eng = pmEngineOf(engine);
    pm::Engine* sceneEngine = nullptr;
    if (host->sceneState.load(std::memory_order_acquire) == PM_SCENE_RUNNING) {
        sceneEngine = host->sceneFresh ? host->sceneEngine : eng;
        sceneBeforeRender(host, *sceneEngine, frames);
        if (!host->sceneFresh) sceneEngine = eng;  // cancelled just now
        else eng = sceneEngine;
    }
    // Speed probe, before the timed render (it is not part of the render
    // time, but of the callback's).
    if (const int32_t every = host->probeEvery.load(std::memory_order_relaxed); every > 0) {
        if (++host->probeCounter >= every) {
            host->probeCounter = 0;
            float sink = 0.0f;
            const int cpuNow = pm::host::currentCpu();
            const pm::host::ProbeTimes pt = pm::host::timeSpeedProbeSplit(&sink);
            host->probeSink.store(sink, std::memory_order_relaxed);
            host->core.meter().recordProbe(pt.latencyUs, pt.throughputUs, cpuNow);
        }
    }
    const bool cpu = host->cpuTiming.load(std::memory_order_relaxed);
    // CPU numbers while any diagnostic is on (two register reads).
    const bool trackCpu = cpu || eng->phaseTiming() || host->probeEvery.load(std::memory_order_relaxed) > 0;
    const int cpuStart = trackCpu ? pm::host::currentCpu() : -1;
    const double c0 = cpu ? threadCpuSeconds() : -1.0;
    const auto t0 = std::chrono::steady_clock::now();
    // Load multiplier and CPU trace: only while the scene runs (set up by
    // pm_host_scene_start_load inside the gate).
    const bool sceneOn = sceneEngine && host->sceneState.load(std::memory_order_relaxed) == PM_SCENE_RUNNING;
    const int extra = sceneOn ? host->sceneLoad - 1 : 0;
    if (sceneOn && trackCpu) {
        pm_host::Trace& tr = host->trace;
        tr.armed = true;
        tr.cpuStart = cpuStart;
        tr.index = 0;
        tr.first = tr.movedTo = -1;
        tr.t0 = t0;
        tr.usAtChange = 0.0;
    }
    host->core.forwardEvents([eng](const pm::Event& e) { eng->pushEvent(e); });  // a full engine queue counts the drop
    pm::ClockMapping clock;
    clock.hostTimeAtBufferStart = host_time;
    clock.hostTicksPerSample = host->ticksPerSample;
    eng->render(left, right, frames, host_time != 0 ? &clock : nullptr);
    if (extra > 0) {
        float* xl = host->loadScratch.data();
        float* xr = xl + pm_host::kScratchFrames;
        for (int i = 0; i < extra; ++i)
            for (int32_t done = 0; done < frames;) {
                const int32_t n = std::min<int32_t>(frames - done, pm_host::kScratchFrames);
                host->loadEngines[i]->render(xl, xr, n, nullptr);
                done += n;
            }
    }
    const auto t1 = std::chrono::steady_clock::now();
    const double c1 = cpu ? threadCpuSeconds() : -1.0;
    const int cpuEnd = trackCpu ? pm::host::currentCpu() : -1;
    pm::host::BufferRecord rec;
    rec.cpuStart = cpuStart;
    rec.cpuEnd = cpuEnd;
    rec.renderSeconds = std::chrono::duration<double>(t1 - t0).count();
    rec.periodSeconds = frames / host->sampleRate;
    rec.cpuSeconds = c0 >= 0.0 && c1 >= 0.0 ? c1 - c0 : -1.0;
    rec.frames = frames;
    if (const pm::Engine* e = eng) {
        const pm::RenderInfo& info = e->lastRender();
        rec.hammersInContact = info.hammersInContact;
        rec.hammersActive = info.hammersActive;
        rec.awakeModes = info.awakeModes;
        rec.events = info.events;
        rec.subBlocks = info.subBlocks;
        rec.idleSubBlocks = info.idleSubBlocks;
        rec.noteOns = info.noteOns;
        rec.noteOffs = info.noteOffs;
        rec.pedalEvents = info.pedalEvents;
        rec.damperUpdates = info.damperUpdates;
        rec.responseGroups = info.responseGroups;
        rec.kernelRebuilds = info.kernelRebuilds;
        rec.kernelKeyUpdates = info.kernelKeyUpdates;
        rec.maxHammersInContact = info.maxHammersInContact;
        rec.hammerContactSamples = info.hammerContactSamples;
        rec.newtonEvals = info.newtonEvals;
        rec.fallbackSolves = info.fallbackSolves;
        rec.hammerSamples = info.hammerSamples;
        rec.hammerDampedSamples = info.hammerDampedSamples;
        rec.freeGroupBlocks = info.freeGroupBlocks;
        rec.forcedGroupBlocks = info.forcedGroupBlocks;
        rec.loopMembers = info.loopMembers;
        rec.loopListeners = info.loopListeners;
        rec.loopEntries = info.loopEntries;
        rec.outZones = info.outZones;
        rec.groupsWoken = info.groupsWoken;
        rec.approxResponses = info.approxResponses;
        const pm::EngineProfile& p = e->profile();
        if (e->phaseTiming() && p.totalUs > 0.0) {
            static_assert(pm::EngineProfile::kPhases + 1 == pm::host::kLoadPhases, "phase table out of date");
            static_assert(PM_PHASE_COUNT == pm::host::kLoadPhases, "phase table out of date");
            rec.timed = true;
            double sum = 0.0;
            for (int i = 0; i < pm::EngineProfile::kPhases; ++i) {
                rec.phaseUs[i] = p.us[i];
                sum += p.us[i];
            }
            rec.phaseUs[pm::EngineProfile::kPhases] = p.totalUs > sum ? p.totalUs - sum : 0.0;
        }
    }
    host->core.meter().record(rec);
    if (sceneEngine)
        sceneAfterRender(host, *sceneEngine, frames, 1e6 * rec.renderSeconds, 1e6 * rec.periodSeconds, cpuEnd);
    host->lastFrames.store(frames, std::memory_order_relaxed);
    host->core.exitRender();
}

uint32_t pm_host_midi_event_list(pm_host* host, const void* event_list, uint32_t source_id) {
    if (!host || !event_list) return 0;
    return host->core.receiveEventList(event_list, source_id);
}

void pm_host_midi_ump(pm_host* host, const uint32_t* words, int32_t count, uint64_t host_time,
                      uint32_t source_id) {
    if (!host || !words || count <= 0) return;
    host->core.receiveUmp(words, static_cast<std::size_t>(count), host_time, source_id);
}

void pm_host_midi_bytes(pm_host* host, const uint8_t* bytes, int32_t count, uint64_t host_time,
                        uint32_t source_id) {
    if (!host || !bytes || count <= 0) return;
    host->core.receiveBytes(bytes, static_cast<std::size_t>(count), host_time, source_id);
}

int pm_host_push_event(pm_host* host, const pm_event* event) {
    if (!host || !event) return 0;
    pm::Event e;
    e.time = event->time;
    e.type = static_cast<pm::EventType>(event->type);
    e.timeBase = event->time_base == PM_TIME_HOST ? pm::TimeBase::HostTime : pm::TimeBase::SampleTime;
    e.note = event->note;
    e.value = event->value;
    return host->core.pushUi(e) ? 1 : 0;
}

int pm_host_monitor_pop(pm_host* host, pm_midi_monitor_entry* entry) {
    if (!host || !entry) return 0;
    pm::host::MonitorEntry m;
    if (!host->core.popMonitor(m)) return 0;
    toEntry(m, *entry);
    return 1;
}

void pm_host_set_monitor_enabled(pm_host* host, int enabled) {
    if (host) host->core.setMonitorEnabled(enabled != 0);
}

void pm_host_get_load(const pm_host* host, pm_host_load* load) {
    if (!host || !load) return;
    const pm::host::LoadSnapshot s = host->core.meter().snapshot();
    load->mean_load = s.meanLoad;
    load->recent_load = s.recentLoad;
    load->max_load = s.maxLoad;
    load->last_load = s.lastLoad;
    load->budget = s.budget;
    load->buffers = s.buffers;
    load->over_budget = s.overBudget;
    load->last_frames = host->lastFrames.load(std::memory_order_relaxed);
    load->reserved = 0;
}

void pm_host_get_diagnostics(const pm_host* host, pm_host_diagnostics* d) {
    if (!host || !d) return;
    *d = pm_host_diagnostics{};
    const pm::host::LoadSnapshot s = host->core.meter().snapshot();
    d->sample_rate = host->publishedRate.load(std::memory_order_relaxed);
    d->buffers = s.buffers;
    d->last_frames = s.lastFrames;
    d->min_frames = s.minFrames;
    d->max_frames = s.maxFrames;
    d->distinct_frame_counts = s.distinctFrames;
    d->last_period_us = s.lastPeriodUs;
    d->mean_us = s.meanUs;
    d->p99_us = s.p99Us;
    d->max_us = s.maxUs;
    d->mean_load = s.meanLoad;
    d->p99_load = s.p99Load;
    d->max_load = s.maxLoad;
    d->recent_load = s.recentLoad;
    d->budget = s.budget;
    d->over_budget = s.overBudget;
    d->worst_index = s.worst.index;
    d->worst_frames = s.worst.frames;
    d->worst_hammers_in_contact = s.worst.hammersInContact;
    d->worst_hammers_active = s.worst.hammersActive;
    d->worst_awake_modes = s.worst.awakeModes;
    d->worst_events = s.worst.events;
    d->worst_sub_blocks = s.worst.subBlocks;
    d->worst_period_us = s.worst.periodUs;
    d->worst_render_us = s.worst.renderUs;
    d->worst_load = s.worst.load;
    d->worst_cpu_us = s.worst.cpuUs;
    d->max_stall_us = s.maxStallUs;
    d->cpu_timing = host->cpuTiming.load(std::memory_order_relaxed) && threadCpuSeconds() >= 0.0 ? 1 : 0;
}

void pm_host_set_phase_timing(pm_host* host, int enabled) {
    if (!host) return;
    if (pm::Engine* e = pmEngineOf(host->engine)) e->setPhaseTiming(enabled != 0);
}

void pm_host_get_phase_diagnostics(const pm_host* host, pm_host_phase_diagnostics* d) {
    if (!host || !d) return;
    *d = pm_host_phase_diagnostics{};
    const pm::host::LoadSnapshot s = host->core.meter().snapshot();
    if (const pm::Engine* e = pmEngineOf(host->engine)) d->phase_timing = e->phaseTiming() ? 1 : 0;
    d->worst_timed = s.worst.timed ? 1 : 0;
    d->timed_buffers = s.timedBuffers;
    d->top_count = s.topCount;
    for (int p = 0; p < PM_PHASE_COUNT; ++p) {
        d->worst_phase_us[p] = s.worst.phaseUs[p];
        d->top_phase_us[p] = s.topPhaseUs[p];
        d->mean_phase_us[p] = s.meanPhaseUs[p];
    }
    d->worst_note_ons = s.worst.noteOns;
    d->worst_note_offs = s.worst.noteOffs;
    d->worst_pedal_events = s.worst.pedalEvents;
    d->worst_damper_updates = s.worst.damperUpdates;
    d->worst_response_groups = s.worst.responseGroups;
    d->worst_kernel_rebuilds = s.worst.kernelRebuilds;
    d->worst_idle_sub_blocks = s.worst.idleSubBlocks;
}

void pm_host_get_worst_detail(const pm_host* host, pm_host_worst_detail* d) {
    if (!host || !d) return;
    *d = pm_host_worst_detail{};
    const pm::host::LoadSnapshot s = host->core.meter().snapshot();
    d->hammers_in_contact = s.worst.hammersInContact;
    d->max_hammers_in_contact = s.worst.maxHammersInContact;
    d->hammer_contact_samples = s.worst.hammerContactSamples;
    d->newton_evals = s.worst.newtonEvals;
    d->fallback_solves = s.worst.fallbackSolves;
    d->kernel_rebuilds = s.worst.kernelRebuilds;
    d->kernel_key_updates = s.worst.kernelKeyUpdates;
}

void pm_host_get_worst_work(const pm_host* host, pm_work_detail* d) {
    if (!d) return;
    *d = pm_work_detail{};
    if (!host) return;
    toWork(host->core.meter().snapshot().worst, *d);
}

int pm_host_scene_start(pm_host* host, const pm_bench_config* config, int32_t item_index) {
    return pm_host_scene_start_load(host, config, item_index, 1);
}

int pm_host_scene_start_load(pm_host* host, const pm_bench_config* config, int32_t item_index, int32_t load) {
    if (!host || !config || !host->core.ready()) return 0;
    if (load < 1 || load > PM_SCENE_MAX_LOAD) return 0;
    const int32_t scene = pm_bench_item_scene(config, item_index);
    if (scene < 0) return 0;
    pm::Engine* eng = pmEngineOf(host->engine);
    if (!eng) return 0;
    // Built outside the gate (allocation), handed over inside it.
    const double fs = host->sampleRate > 0.0 ? host->sampleRate : eng->sampleRate();
    const double seconds = config->seconds > 0.0 ? config->seconds : 3.0;
    const pm::bench::Scene sc = static_cast<pm::bench::Scene>(scene);
    const double warm = sc == pm::bench::Scene::Worst ? 0.0 : 0.5;
    std::vector<pm::Event> events;
    pm::bench::sceneEvents(sc, fs, static_cast<std::uint64_t>((warm + seconds + 1.0) * fs), events);
    const bool timing = (config->flags & PM_BENCH_PHASE_TIMING) != 0;
    const bool probe = (config->flags & PM_BENCH_SPEED_PROBE) != 0;
    // Private engines, prepared as the benchmark prepares the item's: the
    // first one is heard (PM_BENCH_SCENE_FRESH_ENGINE, implied by load > 1),
    // the others are the load multiplier's.
    const bool wantFresh = load > 1 || (config->flags & PM_BENCH_SCENE_FRESH_ENGINE) != 0;
    pm::Engine* fresh[PM_SCENE_MAX_LOAD] = {};
    std::uint64_t bytes = 0;
    std::vector<float> scratch;
    auto freeAll = [&fresh] {
        for (pm::Engine*& e : fresh) {
            delete e;
            e = nullptr;
        }
    };
    if (wantFresh) {
        pm::bench::Config k;
        k.allTiers = (config->flags & PM_BENCH_ALL_TIERS) != 0;
        if (config->tier >= 0 && config->tier <= 2) k.tier = static_cast<pm::QualityTier>(config->tier);
        const pm::bench::Case c = pm::bench::caseAt(k, item_index);
        for (int i = 0; i < load; ++i) {
            fresh[i] = new (std::nothrow) pm::Engine;
            std::size_t b = 0;
            if (!fresh[i] || !pm::bench::prepareForCase(c, *fresh[i], fs, std::max(1, eng->maxFrames()), &b)) {
                freeAll();
                return 0;
            }
            if (i == 0) bytes = b;
        }
        if (load > 1) scratch.assign(2 * static_cast<std::size_t>(pm_host::kScratchFrames), 0.0f);
    }
    pm::Engine* old[PM_SCENE_MAX_LOAD] = {};
    const int ok = host->core.runExclusive([&] {
               if (!eng->prepared()) return false;
               // A scene still running loses its result: undo what it switched on.
               if (host->sceneState.load(std::memory_order_relaxed) == PM_SCENE_RUNNING) {
                   if (host->scene.restoreProbe) host->probeEvery.store(0, std::memory_order_relaxed);
                   if (host->scene.restoreTiming) (host->sceneFresh ? host->sceneEngine : eng)->setPhaseTiming(false);
               }
               old[0] = host->sceneEngine;
               for (int i = 0; i + 1 < PM_SCENE_MAX_LOAD; ++i) {
                   old[i + 1] = host->loadEngines[i];
                   host->loadEngines[i] = fresh[i + 1];
               }
               host->sceneEngine = fresh[0];
               host->sceneFresh = fresh[0] != nullptr;
               host->sceneLoad = load;
               host->engineBytes = bytes;
               host->loadScratch.swap(scratch);
               eng->setSubBlockHook(nullptr, nullptr);
               pm::Engine* run = fresh[0] ? fresh[0] : eng;
               pm_host::Scene& s = host->scene;
               s = pm_host::Scene{};
               s.events.swap(events);
               s.warmSeconds = warm;
               s.seconds = seconds;
               s.restoreTiming = timing && !run->phaseTiming();
               s.restoreProbe = probe && host->probeEvery.load(std::memory_order_relaxed) == 0;
               if (s.restoreProbe) host->probeEvery.store(8, std::memory_order_relaxed);
               host->trace = pm_host::Trace{};
               for (int i = 0; i < load; ++i) {
                   pm::Engine* e = i == 0 ? run : host->loadEngines[i - 1];
                   if (i > 0) e->setPhaseTiming(timing || run->phaseTiming());  // the same work as the first
                   e->setSubBlockHook(traceSubBlock, host);
                   e->reset();
               }
               if (timing) run->setPhaseTiming(true);
               run->describe(host->sceneConfig, static_cast<int>(sizeof host->sceneConfig));
               host->sceneResult = pm_host_scene_result{};
               host->sceneCpu = pm_cpu_usage{};
               host->sceneProbe = pm_speed_probe_split{};
               host->sceneLoadResult = pm_scene_load{};
               host->sceneResult.state = PM_SCENE_RUNNING;
               host->sceneCancel.store(false, std::memory_order_relaxed);
               host->sceneState.store(PM_SCENE_RUNNING, std::memory_order_release);
               return true;
           })
               ? 1
               : 0;
    if (ok) {
        for (pm::Engine* e : old) delete e;  // swapped out inside the gate: the audio thread no longer sees them
    } else {
        freeAll();  // refused before they were installed
    }
    return ok;
}

void pm_host_scene_cancel(pm_host* host) {
    if (host && host->sceneState.load(std::memory_order_acquire) == PM_SCENE_RUNNING)
        host->sceneCancel.store(true, std::memory_order_release);
}

void pm_host_scene_get_result(const pm_host* host, pm_host_scene_result* result) {
    if (!result) return;
    *result = pm_host_scene_result{};
    if (!host) return;
    const int32_t st = host->sceneState.load(std::memory_order_acquire);
    if (st == PM_SCENE_DONE) *result = host->sceneResult;
    result->state = st;
}

int32_t pm_host_scene_config(const pm_host* host, char* buffer, int32_t size) {
    if (!buffer || size <= 0) return 0;
    buffer[0] = 0;
    if (!host || host->sceneState.load(std::memory_order_acquire) == PM_SCENE_IDLE) return 0;
    const int32_t n = static_cast<int32_t>(std::min<std::size_t>(std::strlen(host->sceneConfig), static_cast<std::size_t>(size - 1)));
    std::memcpy(buffer, host->sceneConfig, static_cast<std::size_t>(n));
    buffer[n] = 0;
    return n;
}

void pm_host_set_speed_probe(pm_host* host, int32_t every_n_buffers) {
    if (host) host->probeEvery.store(every_n_buffers > 0 ? every_n_buffers : 0, std::memory_order_relaxed);
}

void pm_host_get_speed_probe(const pm_host* host, pm_speed_probe_stats* stats) {
    if (!host || !stats) return;
    *stats = pm_speed_probe_stats{};
    stats->interval = host->probeEvery.load(std::memory_order_relaxed);
    const pm::host::ProbeSnapshot p = host->core.meter().probeSnapshot();
    stats->runs = p.count;
    stats->min_us = p.minUs;
    stats->median_us = p.medianUs;
    stats->mean_us = p.meanUs;
    stats->max_us = p.maxUs;
}

double pm_speed_probe_run_us(int32_t runs, pm_speed_probe_stats* stats) {
    constexpr int kMax = 4096;
    double t[kMax];
    const int n = runs < 1 ? 1 : (runs > kMax ? kMax : runs);
    float sink = 0.0f;
    double sum = 0.0;
    for (int i = 0; i < n; ++i) {
        float r = 0.0f;
        t[i] = pm::host::timeSpeedProbe(&r);
        sink += r;
        sum += t[i];
    }
    std::sort(t, t + n);
    const double median = n % 2 ? t[n / 2] : 0.5 * (t[n / 2 - 1] + t[n / 2]);
    if (stats) {
        *stats = pm_speed_probe_stats{};
        stats->runs = static_cast<uint64_t>(n);
        stats->min_us = t[0];
        stats->median_us = median;
        stats->mean_us = sum / n;
        stats->max_us = t[n - 1];
        stats->checksum = sink;
    }
    return median;
}

void pm_host_get_speed_probe_split(const pm_host* host, pm_speed_probe_split* split) {
    if (!split) return;
    *split = pm_speed_probe_split{};
    if (host) toSplit(host->core.meter().probeSnapshot(), *split);
}

double pm_speed_probe_run_split(int32_t runs, pm_speed_probe_split* split) {
    constexpr int kMax = 4096;
    double lat[kMax], thr[kMax];
    const int n = runs < 1 ? 1 : (runs > kMax ? kMax : runs);
    for (int i = 0; i < n; ++i) {
        float r = 0.0f;  // the work is kept live inside timeSpeedProbeSplit
        const pm::host::ProbeTimes t = pm::host::timeSpeedProbeSplit(&r);
        lat[i] = t.latencyUs;
        thr[i] = t.throughputUs;
    }
    auto fill = [n](double* t, pm_probe_series& o) {
        double sum = 0.0;
        for (int i = 0; i < n; ++i) sum += t[i];
        std::sort(t, t + n);
        o.runs = static_cast<uint64_t>(n);
        o.min_us = t[0];
        o.max_us = t[n - 1];
        o.median_us = n % 2 ? t[n / 2] : 0.5 * (t[n / 2 - 1] + t[n / 2]);
        o.mean_us = sum / n;
    };
    pm_speed_probe_split s{};
    fill(lat, s.latency);
    fill(thr, s.throughput);
    if (split) *split = s;
    return s.throughput.median_us;
}

int32_t pm_current_cpu(void) { return pm::host::currentCpu(); }

void pm_host_get_cpu_usage(const pm_host* host, pm_cpu_usage* usage) {
    if (!usage) return;
    *usage = pm_cpu_usage{};
    usage->worst_cpu_start = usage->worst_cpu_end = -1;
    if (!host) return;
    pm::bench::toCpuUsage(host->core.meter().snapshot().cpu, *usage);
}

void pm_host_scene_get_cpu_usage(const pm_host* host, pm_cpu_usage* usage) {
    if (!usage) return;
    *usage = pm_cpu_usage{};
    usage->worst_cpu_start = usage->worst_cpu_end = -1;
    if (host && host->sceneState.load(std::memory_order_acquire) == PM_SCENE_DONE) *usage = host->sceneCpu;
}

void pm_host_scene_get_probe_split(const pm_host* host, pm_speed_probe_split* split) {
    if (!split) return;
    *split = pm_speed_probe_split{};
    if (host && host->sceneState.load(std::memory_order_acquire) == PM_SCENE_DONE) *split = host->sceneProbe;
}

void pm_host_scene_get_load(const pm_host* host, pm_scene_load* result) {
    if (!result) return;
    *result = pm_scene_load{};
    if (host && host->sceneState.load(std::memory_order_acquire) == PM_SCENE_DONE) *result = host->sceneLoadResult;
}

void pm_host_set_cpu_timing(pm_host* host, int enabled) {
    if (host) host->cpuTiming.store(enabled != 0, std::memory_order_relaxed);
}

void pm_host_reset_load(pm_host* host) {
    if (host) host->core.meter().requestReset();
}

void pm_host_set_load_budget(pm_host* host, double fraction) {
    if (host) host->core.meter().setBudget(fraction);
}

void pm_host_get_counters(const pm_host* host, pm_host_counters* counters) {
    if (!host || !counters) return;
    const pm::host::HostCounters c = host->core.counters();
    counters->midi_messages = c.midiMessages;
    counters->midi_malformed = c.midiMalformed;
    counters->midi_events = c.midiEvents;
    counters->dropped_events = c.droppedEvents;
    counters->monitor_dropped = c.monitorDropped;
}

}  // extern "C"
