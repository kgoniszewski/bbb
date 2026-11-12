// C API of the app host (pm_api.h, "App host"), over pm::host::Host.
#include <chrono>
#include <ctime>
#include <new>

#include "pm_api/pm_api.h"
#include "pm_api/pm_api_internal.h"
#include "pm_host/host.h"

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
    const bool cpu = host->cpuTiming.load(std::memory_order_relaxed);
    const double c0 = cpu ? threadCpuSeconds() : -1.0;
    const auto t0 = std::chrono::steady_clock::now();
    pm_engine* engine = host->engine;
    host->core.forwardEvents([engine](const pm::Event& e) {
        pm_event c;
        c.time = e.time;
        c.type = static_cast<uint8_t>(e.type);
        c.time_base = static_cast<uint8_t>(e.timeBase);
        c.note = e.note;
        c.reserved = 0;
        c.value = e.value;
        c.reserved2 = 0;
        pm_engine_push_event(engine, &c);  // a full engine queue counts the drop
    });
    pm_clock clock;
    clock.host_time_at_buffer_start = host_time;
    clock.host_ticks_per_sample = host->ticksPerSample;
    pm_engine_render(engine, left, right, frames, host_time != 0 ? &clock : nullptr);
    const auto t1 = std::chrono::steady_clock::now();
    const double c1 = cpu ? threadCpuSeconds() : -1.0;
    pm::host::BufferRecord rec;
    rec.renderSeconds = std::chrono::duration<double>(t1 - t0).count();
    rec.periodSeconds = frames / host->sampleRate;
    rec.cpuSeconds = c0 >= 0.0 && c1 >= 0.0 ? c1 - c0 : -1.0;
    rec.frames = frames;
    if (const pm::Engine* e = pmEngineOf(engine)) {
        const pm::RenderInfo& info = e->lastRender();
        rec.hammersInContact = info.hammersInContact;
        rec.hammersActive = info.hammersActive;
        rec.awakeModes = info.awakeModes;
        rec.events = info.events;
        rec.subBlocks = info.subBlocks;
    }
    host->core.meter().record(rec);
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
