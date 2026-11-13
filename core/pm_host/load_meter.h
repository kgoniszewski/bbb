// Render-load meter (plan §4.5): per-buffer render time as a fraction of the
// buffer period. record() runs on the audio thread (wait-free, no
// allocation); snapshot() and requestReset() from any other thread. Times
// are arguments, so the meter is clock-free and testable.
//
// Besides mean / max / smoothed load it keeps what is needed to explain a
// slow buffer without a profiler:
//  - fixed-bin histograms of the load (0.25 % bins up to 200 %) and of the
//    render time (log bins, 48 per octave, about 1.5 % wide, 1 us .. 1 s),
//    from which snapshot() reads the 99th percentiles (upper bin edges);
//    the audio thread only increments its own bins (relaxed store of
//    load + 1: one writer, so no read-modify-write is needed);
//  - the most expensive buffer (by load) since the reset, with its frame
//    count, period, render and thread-CPU time, and what the engine did in
//    it (hammers, awake modes, events, sub-blocks), published through a
//    sequence counter so that a reader never mixes two buffers;
//  - the frame counts seen: last, min, max and how many distinct values
//    (a render callback with varying frame counts, e.g. behind a sample-
//    rate converter, has more than one);
//  - when the engine's phase timing is on, the per-phase split of the
//    render time (events, hammers, free pass, forced pass, board, other):
//    for the worst buffer, as a mean over all timed buffers and as a mean
//    over the top 1 % by render time (phase sums kept per render-time bin,
//    so the top 1 % is resolved to a bin, about 1.5 %);
//  - for the worst buffer, the events by kind and the recomputations they
//    caused (damper updates, impulse responses, zone-kernel rebuilds and
//    in-place updates), and the hammer work (hammers that touched a string,
//    the most at the same time, hammer-samples in contact, contact-solver
//    Newton evaluations and fallbacks);
//  - the speed probe (recordProbe): the time of a fixed workload run on
//    the audio thread every Nth buffer, as min / median / mean / max, to
//    compare with the same workload on a benchmark thread (speed_probe.h);
//    since API 0.11 also its latency-bound and throughput-bound parts;
//  - which CPU each render call ran on (cpu_id.h), when the caller reads
//    it: render calls and summed render time per CPU (up to 64), the
//    render calls that ended on another CPU than they started on, the worst
//    buffer's CPUs, and the probe's two parts per CPU.
#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

#include "pm_host/cpu_id.h"

namespace pm::host {

// One rendered buffer. renderSeconds: wall time spent rendering;
// periodSeconds: frames / sample rate; cpuSeconds: thread CPU time spent
// rendering (< 0 when not measured).
inline constexpr int kLoadPhases = 6;  // events, hammers, free, forced, board, other

struct BufferRecord {
    double renderSeconds = 0.0;
    double periodSeconds = 0.0;
    double cpuSeconds = -1.0;
    int frames = 0;
    int hammersInContact = 0;
    int hammersActive = 0;
    int awakeModes = 0;
    int events = 0;
    int subBlocks = 0;
    int idleSubBlocks = 0;
    int noteOns = 0, noteOffs = 0, pedalEvents = 0;
    int damperUpdates = 0, responseGroups = 0, kernelRebuilds = 0;
    int kernelKeyUpdates = 0;
    int maxHammersInContact = 0, hammerContactSamples = 0, newtonEvals = 0, fallbackSolves = 0;
    // Work counts (engine RenderInfo): hammer-samples stepped / with the
    // damper on, bank group-blocks of the free / forced pass, soundboard-loop
    // members, listeners, zone-kernel entries, zones read.
    int hammerSamples = 0, hammerDampedSamples = 0, freeGroupBlocks = 0, forcedGroupBlocks = 0;
    int loopMembers = 0, loopListeners = 0, loopEntries = 0, outZones = 0;
    // Sleeping coupled groups woken by loop input, and coupled-group
    // responses taken from the damper expansion (engine RenderInfo).
    int groupsWoken = 0, approxResponses = 0;
    bool timed = false;                // phaseUs valid
    double phaseUs[kLoadPhases] = {};
    int cpuStart = -1, cpuEnd = -1;    // CPU at the start / end of the render call (-1: not read)
};

struct WorstBuffer {
    std::uint64_t index = 0;  // buffer number since the reset (1 = first)
    double load = 0.0;
    double renderUs = 0.0;
    double periodUs = 0.0;
    double cpuUs = -1.0;      // < 0: not measured
    int frames = 0;
    int hammersInContact = 0;
    int hammersActive = 0;
    int awakeModes = 0;
    int events = 0;
    int subBlocks = 0;
    int idleSubBlocks = 0;
    int noteOns = 0, noteOffs = 0, pedalEvents = 0;
    int damperUpdates = 0, responseGroups = 0, kernelRebuilds = 0;
    int kernelKeyUpdates = 0;
    int maxHammersInContact = 0, hammerContactSamples = 0, newtonEvals = 0, fallbackSolves = 0;
    // Work counts (engine RenderInfo): hammer-samples stepped / with the
    // damper on, bank group-blocks of the free / forced pass, soundboard-loop
    // members, listeners, zone-kernel entries, zones read.
    int hammerSamples = 0, hammerDampedSamples = 0, freeGroupBlocks = 0, forcedGroupBlocks = 0;
    int loopMembers = 0, loopListeners = 0, loopEntries = 0, outZones = 0;
    int groupsWoken = 0, approxResponses = 0;
    bool timed = false;
    double phaseUs[kLoadPhases] = {};
    int cpuStart = -1, cpuEnd = -1;
};

struct ProbeSeries {
    std::uint64_t count = 0;
    double minUs = 0.0, medianUs = 0.0, meanUs = 0.0, maxUs = 0.0;  // median: bin centre (about 1.5 %)
};

struct ProbeSnapshot {
    std::uint64_t count = 0;  // probe runs since the last reset
    double minUs = 0.0, medianUs = 0.0, meanUs = 0.0, maxUs = 0.0;  // the whole probe
    // The two parts (count 0 when only totals were recorded).
    ProbeSeries latency, throughput;
};

struct LoadSnapshot {
    double meanLoad = 0.0;    // mean of render time / period since the last reset
    double recentLoad = 0.0;  // exponentially smoothed, time constant about kRecentSeconds of audio
    double maxLoad = 0.0;     // worst single buffer since the last reset
    double lastLoad = 0.0;
    double p99Load = 0.0;     // upper edge of the histogram bin holding the 99th percentile
    std::uint64_t buffers = 0;     // since the last reset
    std::uint64_t overBudget = 0;  // buffers whose load exceeded the budget
    double budget = 0.0;           // the threshold for overBudget, a fraction of the period
    // Render time per buffer, microseconds.
    double meanUs = 0.0, p99Us = 0.0, maxUs = 0.0;
    // Largest wall-clock minus thread-CPU time of a buffer (time the thread
    // was not running: preemption, interrupts), when CPU time is measured.
    double maxStallUs = -1.0;
    // Frame counts since the last reset (0 when no buffer).
    int lastFrames = 0, minFrames = 0, maxFrames = 0, distinctFrames = 0;
    double lastPeriodUs = 0.0;
    WorstBuffer worst;
    // Phase timing: buffers timed since the reset, the mean split over them,
    // and the mean over the top 1 % by render time (topCount buffers).
    std::uint64_t timedBuffers = 0;
    std::uint64_t topCount = 0;
    double meanPhaseUs[kLoadPhases] = {};
    double topPhaseUs[kLoadPhases] = {};
    ProbeSnapshot probe;
    CpuUsage cpu;  // per-CPU totals (cpu.tracked + cpu.unknown == buffers whose CPU was read or not)
};

class LoadMeter {
public:
    static constexpr double kRecentSeconds = 0.5;
    static constexpr double kDefaultBudget = 0.7;  // worst-buffer limit on the target (CLAUDE.md)
    static constexpr int kLoadBins = 800;          // 0.25 % each, up to 200 %; then one overflow bin
    static constexpr double kLoadBinWidth = 0.0025;
    static constexpr int kUsBinsPerOctave = 48;
    static constexpr int kUsBins = 20 * kUsBinsPerOctave;  // 1 us .. 2^20 us; then one overflow bin
    static constexpr int kFrameBits = 8192;        // distinct frame counts tracked exactly below this

    // Audio thread. Ignores a non-positive period.
    void record(const BufferRecord& b) noexcept {
        if (!(b.periodSeconds > 0.0)) return;
        if (reset_.exchange(false, std::memory_order_acquire)) clearAudioState();
        const double load = b.renderSeconds > 0.0 ? b.renderSeconds / b.periodSeconds : 0.0;
        const double us = b.renderSeconds > 0.0 ? 1e6 * b.renderSeconds : 0.0;
        sum_ += load;
        sumUs_ += us;
        ++n_;
        if (load > budget_.load(std::memory_order_relaxed)) ++over_;
        // One-pole smoother with a time constant in seconds of audio, so the
        // reading does not depend on the buffer size.
        double a = b.periodSeconds / kRecentSeconds;
        if (a > 1.0) a = 1.0;
        recent_ = n_ == 1 ? load : recent_ + a * (load - recent_);
        bump(loadHist_[loadBin(load)]);
        const int ub = usBin(us);
        bump(usHist_[ub]);
        if (b.timed) {
            for (int p = 0; p < kLoadPhases; ++p) {
                addTo(phaseBin_[ub][p], b.phaseUs[p]);
                addTo(phaseAll_[p], b.phaseUs[p]);
            }
            bump(phaseCnt_[ub]);
        }
        if (us > maxUs_) maxUs_ = us;
        recordCpu(b.cpuStart, b.cpuEnd, us);
        if (b.cpuSeconds >= 0.0) {
            // The CPU clock is read just outside the wall-clock interval.
            const double stall = std::max(0.0, us - 1e6 * b.cpuSeconds);
            if (stall > maxStall_) maxStall_ = stall;
        }
        // Frame counts.
        const int f = b.frames;
        if (f > 0) {
            if (minFrames_ == 0 || f < minFrames_) minFrames_ = f;
            if (f > maxFrames_) maxFrames_ = f;
            const int bit = f < kFrameBits ? f : 0;  // every count >= kFrameBits shares bit 0
            std::uint64_t& w = seenFrames_[bit >> 6];
            const std::uint64_t mask = std::uint64_t{1} << (bit & 63);
            if (!(w & mask)) {
                w |= mask;
                ++distinct_;
            }
        }
        if (n_ == 1 || load > max_) {
            max_ = load;
            const std::uint32_t seq = worstSeq_.load(std::memory_order_relaxed);
            worstSeq_.store(seq + 1, std::memory_order_relaxed);  // odd: writing
            std::atomic_thread_fence(std::memory_order_release);
            wIndex_.store(n_, std::memory_order_relaxed);
            wLoad_.store(load, std::memory_order_relaxed);
            wUs_.store(us, std::memory_order_relaxed);
            wPeriodUs_.store(1e6 * b.periodSeconds, std::memory_order_relaxed);
            wCpuUs_.store(b.cpuSeconds >= 0.0 ? 1e6 * b.cpuSeconds : -1.0, std::memory_order_relaxed);
            wFrames_.store(f, std::memory_order_relaxed);
            wContact_.store(b.hammersInContact, std::memory_order_relaxed);
            wActive_.store(b.hammersActive, std::memory_order_relaxed);
            wModes_.store(b.awakeModes, std::memory_order_relaxed);
            wEvents_.store(b.events, std::memory_order_relaxed);
            wSub_.store(b.subBlocks, std::memory_order_relaxed);
            wIdle_.store(b.idleSubBlocks, std::memory_order_relaxed);
            wOn_.store(b.noteOns, std::memory_order_relaxed);
            wOff_.store(b.noteOffs, std::memory_order_relaxed);
            wPedal_.store(b.pedalEvents, std::memory_order_relaxed);
            wDamper_.store(b.damperUpdates, std::memory_order_relaxed);
            wResp_.store(b.responseGroups, std::memory_order_relaxed);
            wKernel_.store(b.kernelRebuilds, std::memory_order_relaxed);
            wKeyUpd_.store(b.kernelKeyUpdates, std::memory_order_relaxed);
            wMaxContact_.store(b.maxHammersInContact, std::memory_order_relaxed);
            wContactSamples_.store(b.hammerContactSamples, std::memory_order_relaxed);
            wNewton_.store(b.newtonEvals, std::memory_order_relaxed);
            wFallback_.store(b.fallbackSolves, std::memory_order_relaxed);
            wHamSamples_.store(b.hammerSamples, std::memory_order_relaxed);
            wHamDamped_.store(b.hammerDampedSamples, std::memory_order_relaxed);
            wFree_.store(b.freeGroupBlocks, std::memory_order_relaxed);
            wForced_.store(b.forcedGroupBlocks, std::memory_order_relaxed);
            wMembers_.store(b.loopMembers, std::memory_order_relaxed);
            wListeners_.store(b.loopListeners, std::memory_order_relaxed);
            wEntries_.store(b.loopEntries, std::memory_order_relaxed);
            wZones_.store(b.outZones, std::memory_order_relaxed);
            wWoken_.store(b.groupsWoken, std::memory_order_relaxed);
            wApprox_.store(b.approxResponses, std::memory_order_relaxed);
            wTimed_.store(b.timed, std::memory_order_relaxed);
            for (int p = 0; p < kLoadPhases; ++p) wPhase_[p].store(b.timed ? b.phaseUs[p] : 0.0, std::memory_order_relaxed);
            wCpuStart_.store(b.cpuStart, std::memory_order_relaxed);
            wCpuEnd_.store(b.cpuEnd, std::memory_order_relaxed);
            worstSeq_.store(seq + 2, std::memory_order_release);
        }
        mean_.store(sum_ / static_cast<double>(n_), std::memory_order_relaxed);
        meanUsPub_.store(sumUs_ / static_cast<double>(n_), std::memory_order_relaxed);
        recentPub_.store(recent_, std::memory_order_relaxed);
        maxPub_.store(max_, std::memory_order_relaxed);
        maxUsPub_.store(maxUs_, std::memory_order_relaxed);
        stallPub_.store(maxStall_, std::memory_order_relaxed);
        last_.store(load, std::memory_order_relaxed);
        lastFrames_.store(f, std::memory_order_relaxed);
        lastPeriodUs_.store(1e6 * b.periodSeconds, std::memory_order_relaxed);
        minFramesPub_.store(minFrames_, std::memory_order_relaxed);
        maxFramesPub_.store(maxFrames_, std::memory_order_relaxed);
        distinctPub_.store(distinct_, std::memory_order_relaxed);
        overPub_.store(over_, std::memory_order_relaxed);
        count_.store(n_, std::memory_order_release);
    }

    // Audio thread: the original two-argument form (frame count unknown).
    void record(double renderSeconds, double periodSeconds) noexcept {
        BufferRecord b;
        b.renderSeconds = renderSeconds;
        b.periodSeconds = periodSeconds;
        record(b);
    }

    // Any thread. Values are individually consistent; a snapshot taken while
    // record() runs may mix two consecutive buffers, which a display
    // tolerates (the worst-buffer record is always from one buffer).
    LoadSnapshot snapshot() const noexcept {
        LoadSnapshot s;
        s.budget = budget_.load(std::memory_order_relaxed);
        if (reset_.load(std::memory_order_acquire)) return s;  // reset pending: show zeros
        s.buffers = count_.load(std::memory_order_acquire);
        s.meanLoad = mean_.load(std::memory_order_relaxed);
        s.recentLoad = recentPub_.load(std::memory_order_relaxed);
        s.maxLoad = maxPub_.load(std::memory_order_relaxed);
        s.lastLoad = last_.load(std::memory_order_relaxed);
        s.overBudget = overPub_.load(std::memory_order_relaxed);
        s.meanUs = meanUsPub_.load(std::memory_order_relaxed);
        s.maxUs = maxUsPub_.load(std::memory_order_relaxed);
        s.maxStallUs = stallPub_.load(std::memory_order_relaxed);
        s.lastFrames = lastFrames_.load(std::memory_order_relaxed);
        s.lastPeriodUs = lastPeriodUs_.load(std::memory_order_relaxed);
        s.minFrames = minFramesPub_.load(std::memory_order_relaxed);
        s.maxFrames = maxFramesPub_.load(std::memory_order_relaxed);
        s.distinctFrames = distinctPub_.load(std::memory_order_relaxed);
        s.p99Load = percentile(loadHist_, kLoadBins + 1, 0.99, [](int i) { return (i + 1) * kLoadBinWidth; });
        s.p99Us = percentile(usHist_, kUsBins + 1, 0.99,
                             [](int i) { return std::exp2(static_cast<double>(i + 1) / kUsBinsPerOctave); });
        // The histograms are bin-resolution estimates: never above the max.
        if (s.p99Load > s.maxLoad) s.p99Load = s.maxLoad;
        if (s.p99Us > s.maxUs) s.p99Us = s.maxUs;
        for (int attempt = 0; attempt < 64; ++attempt) {
            const std::uint32_t a = worstSeq_.load(std::memory_order_acquire);
            if (a & 1u) continue;
            WorstBuffer w;
            w.index = wIndex_.load(std::memory_order_relaxed);
            w.load = wLoad_.load(std::memory_order_relaxed);
            w.renderUs = wUs_.load(std::memory_order_relaxed);
            w.periodUs = wPeriodUs_.load(std::memory_order_relaxed);
            w.cpuUs = wCpuUs_.load(std::memory_order_relaxed);
            w.frames = wFrames_.load(std::memory_order_relaxed);
            w.hammersInContact = wContact_.load(std::memory_order_relaxed);
            w.hammersActive = wActive_.load(std::memory_order_relaxed);
            w.awakeModes = wModes_.load(std::memory_order_relaxed);
            w.events = wEvents_.load(std::memory_order_relaxed);
            w.subBlocks = wSub_.load(std::memory_order_relaxed);
            w.idleSubBlocks = wIdle_.load(std::memory_order_relaxed);
            w.noteOns = wOn_.load(std::memory_order_relaxed);
            w.noteOffs = wOff_.load(std::memory_order_relaxed);
            w.pedalEvents = wPedal_.load(std::memory_order_relaxed);
            w.damperUpdates = wDamper_.load(std::memory_order_relaxed);
            w.responseGroups = wResp_.load(std::memory_order_relaxed);
            w.kernelRebuilds = wKernel_.load(std::memory_order_relaxed);
            w.kernelKeyUpdates = wKeyUpd_.load(std::memory_order_relaxed);
            w.maxHammersInContact = wMaxContact_.load(std::memory_order_relaxed);
            w.hammerContactSamples = wContactSamples_.load(std::memory_order_relaxed);
            w.newtonEvals = wNewton_.load(std::memory_order_relaxed);
            w.fallbackSolves = wFallback_.load(std::memory_order_relaxed);
            w.hammerSamples = wHamSamples_.load(std::memory_order_relaxed);
            w.hammerDampedSamples = wHamDamped_.load(std::memory_order_relaxed);
            w.freeGroupBlocks = wFree_.load(std::memory_order_relaxed);
            w.forcedGroupBlocks = wForced_.load(std::memory_order_relaxed);
            w.groupsWoken = wWoken_.load(std::memory_order_relaxed);
            w.approxResponses = wApprox_.load(std::memory_order_relaxed);
            w.loopMembers = wMembers_.load(std::memory_order_relaxed);
            w.loopListeners = wListeners_.load(std::memory_order_relaxed);
            w.loopEntries = wEntries_.load(std::memory_order_relaxed);
            w.outZones = wZones_.load(std::memory_order_relaxed);
            w.timed = wTimed_.load(std::memory_order_relaxed);
            for (int p = 0; p < kLoadPhases; ++p) w.phaseUs[p] = wPhase_[p].load(std::memory_order_relaxed);
            w.cpuStart = wCpuStart_.load(std::memory_order_relaxed);
            w.cpuEnd = wCpuEnd_.load(std::memory_order_relaxed);
            std::atomic_thread_fence(std::memory_order_acquire);
            if (worstSeq_.load(std::memory_order_relaxed) == a) {
                s.worst = w;
                break;
            }
        }
        phaseSummary(s);
        s.probe = probeSnapshot();
        s.cpu = cpuSnapshot();
        s.cpu.worstStart = s.worst.cpuStart;
        s.cpu.worstEnd = s.worst.cpuEnd;
        return s;
    }

    // Audio thread: one run of the speed probe, in microseconds (the whole
    // probe only).
    void recordProbe(double us) noexcept {
        if (reset_.exchange(false, std::memory_order_acquire)) clearAudioState();
        if (!(us >= 0.0)) return;
        probeTotal_.add(us);
        probeCount_.store(probeTotal_.n, std::memory_order_release);
    }

    // Audio thread: one run with its two parts and the CPU it ran on (-1:
    // unknown).
    void recordProbe(double latencyUs, double throughputUs, int cpu) noexcept {
        if (reset_.exchange(false, std::memory_order_acquire)) clearAudioState();
        if (!(latencyUs >= 0.0) || !(throughputUs >= 0.0)) return;
        probeLatency_.add(latencyUs);
        probeThroughput_.add(throughputUs);
        if (cpu >= 0 && cpu < kMaxCpus) {
            bump(cpuProbeRuns_[cpu]);
            addTo(cpuProbeLat_[cpu], latencyUs);
            addTo(cpuProbeThr_[cpu], throughputUs);
        }
        probeTotal_.add(latencyUs + throughputUs);
        probeCount_.store(probeTotal_.n, std::memory_order_release);
    }

    // Any thread (also part of snapshot()).
    ProbeSnapshot probeSnapshot() const noexcept {
        ProbeSnapshot p;
        if (reset_.load(std::memory_order_acquire)) return p;
        p.count = probeCount_.load(std::memory_order_acquire);
        if (p.count == 0) return p;
        const ProbeSeries t = probeTotal_.read();
        p.minUs = t.minUs;
        p.medianUs = t.medianUs;
        p.meanUs = t.meanUs;
        p.maxUs = t.maxUs;
        p.latency = probeLatency_.read();
        p.throughput = probeThroughput_.read();
        return p;
    }

    // Any thread (also part of snapshot()): per-CPU totals; worstStart /
    // worstEnd are filled by snapshot() only.
    CpuUsage cpuSnapshot() const noexcept {
        CpuUsage c;
        if (reset_.load(std::memory_order_acquire)) return c;
        c.tracked = cpuTracked_.load(std::memory_order_relaxed);
        c.unknown = cpuUnknown_.load(std::memory_order_relaxed);
        c.migrations = cpuMigrations_.load(std::memory_order_relaxed);
        for (int i = 0; i < kMaxCpus; ++i) {
            c.buffers[i] = cpuBuffers_[i].load(std::memory_order_relaxed);
            c.renderUs[i] = cpuUs_[i].load(std::memory_order_relaxed);
            c.probeRuns[i] = cpuProbeRuns_[i].load(std::memory_order_relaxed);
            c.probeLatencyUs[i] = cpuProbeLat_[i].load(std::memory_order_relaxed);
            c.probeThroughputUs[i] = cpuProbeThr_[i].load(std::memory_order_relaxed);
        }
        return c;
    }

    // Any thread. The audio thread clears its totals at its next record();
    // until then snapshot() reports zeros.
    void requestReset() noexcept { reset_.store(true, std::memory_order_release); }

    void setBudget(double fraction) noexcept {
        if (fraction > 0.0) budget_.store(fraction, std::memory_order_relaxed);
    }

private:
    // One series of probe times: audio-thread totals, a histogram for the
    // median, and the published min / max / mean.
    struct Series {
        std::uint64_t n = 0;
        double sum = 0.0, mn = 0.0, mx = 0.0;
        std::atomic<std::uint32_t> hist[kUsBins + 1] = {};
        std::atomic<std::uint64_t> count{0};
        std::atomic<double> minPub{0.0}, maxPub{0.0}, meanPub{0.0};
        void add(double us) noexcept {
            ++n;
            sum += us;
            if (n == 1 || us < mn) mn = us;
            if (us > mx) mx = us;
            bump(hist[usBin(us)]);
            minPub.store(mn, std::memory_order_relaxed);
            maxPub.store(mx, std::memory_order_relaxed);
            meanPub.store(sum / static_cast<double>(n), std::memory_order_relaxed);
            count.store(n, std::memory_order_release);
        }
        ProbeSeries read() const noexcept {
            ProbeSeries p;
            p.count = count.load(std::memory_order_acquire);
            if (p.count == 0) return p;
            p.minUs = minPub.load(std::memory_order_relaxed);
            p.maxUs = maxPub.load(std::memory_order_relaxed);
            p.meanUs = meanPub.load(std::memory_order_relaxed);
            // Bin centre (geometric) of the bin holding the median, clamped
            // to the observed range.
            const double edge = percentile(hist, kUsBins + 1, 0.5,
                                           [](int i) { return std::exp2(static_cast<double>(i + 1) / kUsBinsPerOctave); });
            p.medianUs = std::clamp(edge * std::exp2(-0.5 / kUsBinsPerOctave), p.minUs, p.maxUs);
            return p;
        }
        void clear() noexcept {
            n = 0;
            sum = mn = mx = 0.0;
            for (auto& c : hist) c.store(0, std::memory_order_relaxed);
            count.store(0, std::memory_order_relaxed);
        }
    };

    void recordCpu(int c0, int c1, double us) noexcept {
        if (c0 >= 0 && c0 < kMaxCpus) {
            bump(cpuBuffers_[c0]);
            addTo(cpuUs_[c0], us);
            cpuTracked_.store(cpuTracked_.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
        } else {
            cpuUnknown_.store(cpuUnknown_.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
        }
        if (c0 >= 0 && c1 >= 0 && c0 != c1)
            cpuMigrations_.store(cpuMigrations_.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
    }

    static int loadBin(double load) noexcept {
        const double x = load / kLoadBinWidth;
        return x < static_cast<double>(kLoadBins) ? static_cast<int>(x) : kLoadBins;
    }
    static int usBin(double us) noexcept {
        if (!(us > 1.0)) return 0;
        const double x = std::log2(us) * kUsBinsPerOctave;
        return x < static_cast<double>(kUsBins) ? static_cast<int>(x) : kUsBins;
    }
    static void bump(std::atomic<std::uint32_t>& c) noexcept {
        c.store(c.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
    }
    static void addTo(std::atomic<double>& c, double x) noexcept {
        c.store(c.load(std::memory_order_relaxed) + x, std::memory_order_relaxed);
    }
    // Mean phase split over all timed buffers and over the top 1 % by
    // render time: whole render-time bins from the top down until they hold
    // at least 1 % of the timed buffers.
    void phaseSummary(LoadSnapshot& s) const noexcept {
        std::uint64_t total = 0;
        for (int i = 0; i <= kUsBins; ++i) total += phaseCnt_[i].load(std::memory_order_relaxed);
        s.timedBuffers = total;
        if (total == 0) return;
        for (int p = 0; p < kLoadPhases; ++p)
            s.meanPhaseUs[p] = phaseAll_[p].load(std::memory_order_relaxed) / static_cast<double>(total);
        const std::uint64_t want = (total + 99) / 100;
        std::uint64_t n = 0;
        double sum[kLoadPhases] = {};
        for (int i = kUsBins; i >= 0 && n < want; --i) {
            const std::uint32_t c = phaseCnt_[i].load(std::memory_order_relaxed);
            if (c == 0) continue;
            n += c;
            for (int p = 0; p < kLoadPhases; ++p) sum[p] += phaseBin_[i][p].load(std::memory_order_relaxed);
        }
        s.topCount = n;
        for (int p = 0; p < kLoadPhases; ++p) s.topPhaseUs[p] = sum[p] / static_cast<double>(n);
    }
    template <typename Edge>
    static double percentile(const std::atomic<std::uint32_t>* h, int bins, double q, Edge edge) noexcept {
        std::uint64_t total = 0;
        for (int i = 0; i < bins; ++i) total += h[i].load(std::memory_order_relaxed);
        if (total == 0) return 0.0;
        const double target = q * static_cast<double>(total);
        std::uint64_t acc = 0;
        for (int i = 0; i < bins; ++i) {
            acc += h[i].load(std::memory_order_relaxed);
            if (static_cast<double>(acc) >= target) return edge(i);
        }
        return edge(bins - 1);
    }
    void clearAudioState() noexcept {
        sum_ = sumUs_ = max_ = maxUs_ = recent_ = 0.0;
        maxStall_ = -1.0;
        n_ = over_ = 0;
        minFrames_ = maxFrames_ = distinct_ = 0;
        for (std::uint64_t& w : seenFrames_) w = 0;
        for (auto& c : loadHist_) c.store(0, std::memory_order_relaxed);
        for (auto& c : usHist_) c.store(0, std::memory_order_relaxed);
        for (auto& c : phaseCnt_) c.store(0, std::memory_order_relaxed);
        for (auto& row : phaseBin_)
            for (auto& c : row) c.store(0.0, std::memory_order_relaxed);
        for (auto& c : phaseAll_) c.store(0.0, std::memory_order_relaxed);
        probeTotal_.clear();
        probeLatency_.clear();
        probeThroughput_.clear();
        probeCount_.store(0, std::memory_order_relaxed);
        cpuTracked_.store(0, std::memory_order_relaxed);
        cpuUnknown_.store(0, std::memory_order_relaxed);
        cpuMigrations_.store(0, std::memory_order_relaxed);
        for (int i = 0; i < kMaxCpus; ++i) {
            cpuBuffers_[i].store(0, std::memory_order_relaxed);
            cpuUs_[i].store(0.0, std::memory_order_relaxed);
            cpuProbeRuns_[i].store(0, std::memory_order_relaxed);
            cpuProbeLat_[i].store(0.0, std::memory_order_relaxed);
            cpuProbeThr_[i].store(0.0, std::memory_order_relaxed);
        }
    }

    // Audio-thread state.
    double sum_ = 0.0, sumUs_ = 0.0, max_ = 0.0, maxUs_ = 0.0, recent_ = 0.0, maxStall_ = -1.0;
    std::uint64_t n_ = 0, over_ = 0;
    int minFrames_ = 0, maxFrames_ = 0, distinct_ = 0;
    std::uint64_t seenFrames_[kFrameBits / 64] = {};
    // Published.
    std::atomic<std::uint32_t> loadHist_[kLoadBins + 1] = {};
    std::atomic<std::uint32_t> usHist_[kUsBins + 1] = {};
    // Phase timing: sums per render-time bin, over everything, and counts.
    std::atomic<std::uint32_t> phaseCnt_[kUsBins + 1] = {};
    std::atomic<double> phaseBin_[kUsBins + 1][kLoadPhases] = {};
    std::atomic<double> phaseAll_[kLoadPhases] = {};
    std::atomic<double> mean_{0.0}, recentPub_{0.0}, maxPub_{0.0}, last_{0.0};
    std::atomic<double> meanUsPub_{0.0}, maxUsPub_{0.0}, stallPub_{-1.0}, lastPeriodUs_{0.0};
    std::atomic<int> lastFrames_{0}, minFramesPub_{0}, maxFramesPub_{0}, distinctPub_{0};
    std::atomic<double> budget_{kDefaultBudget};
    std::atomic<std::uint64_t> count_{0}, overPub_{0};
    std::atomic<bool> reset_{false};
    // Worst buffer, under a sequence counter (odd while being written).
    std::atomic<std::uint32_t> worstSeq_{0};
    std::atomic<std::uint64_t> wIndex_{0};
    std::atomic<double> wLoad_{0.0}, wUs_{0.0}, wPeriodUs_{0.0}, wCpuUs_{-1.0};
    std::atomic<int> wFrames_{0}, wContact_{0}, wActive_{0}, wModes_{0}, wEvents_{0}, wSub_{0};
    std::atomic<int> wIdle_{0}, wOn_{0}, wOff_{0}, wPedal_{0}, wDamper_{0}, wResp_{0}, wKernel_{0};
    std::atomic<int> wKeyUpd_{0}, wMaxContact_{0}, wContactSamples_{0}, wNewton_{0}, wFallback_{0};
    std::atomic<int> wHamSamples_{0}, wHamDamped_{0}, wFree_{0}, wForced_{0};
    std::atomic<int> wMembers_{0}, wListeners_{0}, wEntries_{0}, wZones_{0};
    std::atomic<int> wWoken_{0}, wApprox_{0};
    // Speed probe: the whole probe and its two parts.
    Series probeTotal_, probeLatency_, probeThroughput_;
    std::atomic<std::uint64_t> probeCount_{0};
    std::atomic<bool> wTimed_{false};
    std::atomic<double> wPhase_[kLoadPhases] = {};
    std::atomic<int> wCpuStart_{-1}, wCpuEnd_{-1};
    // Per-CPU totals (one writer: relaxed load + store).
    std::atomic<std::uint64_t> cpuTracked_{0}, cpuUnknown_{0}, cpuMigrations_{0};
    std::atomic<std::uint32_t> cpuBuffers_[kMaxCpus] = {};
    std::atomic<double> cpuUs_[kMaxCpus] = {};
    std::atomic<std::uint32_t> cpuProbeRuns_[kMaxCpus] = {};
    std::atomic<double> cpuProbeLat_[kMaxCpus] = {}, cpuProbeThr_[kMaxCpus] = {};
};

}  // namespace pm::host
