// Render-load meter: mean, max, over-budget count, smoothing, reset, and
// concurrent reads while the audio thread records.
#include <atomic>
#include <cmath>
#include <thread>

#include "pm_host/load_meter.h"
#include "test_harness.h"

using pm::host::LoadMeter;

TEST(mean_max_and_budget) {
    LoadMeter m;
    const double period = 64.0 / 48000.0;
    // 90 buffers at 30 %, 10 at 80 %: mean 35 %, max 80 %, 10 over the 70 % budget.
    for (int i = 0; i < 100; ++i) m.record((i % 10 == 9 ? 0.8 : 0.3) * period, period);
    auto s = m.snapshot();
    CHECK(s.buffers == 100);
    CHECK_NEAR(s.meanLoad, 0.35, 1e-12);
    CHECK_NEAR(s.maxLoad, 0.8, 1e-12);
    CHECK_NEAR(s.lastLoad, 0.8, 1e-12);
    CHECK(s.overBudget == 10);
    CHECK_NEAR(s.budget, LoadMeter::kDefaultBudget, 0.0);
    // A buffer under the budget is not counted.
    m.record(0.69 * period, period);
    CHECK(m.snapshot().overBudget == 10);
    m.setBudget(0.25);
    m.record(0.3 * period, period);
    CHECK(m.snapshot().overBudget == 11);
    m.setBudget(-1.0);  // ignored
    CHECK_NEAR(m.snapshot().budget, 0.25, 0.0);
    // Zero or negative period: ignored; negative time counts as zero load.
    m.record(1.0, 0.0);
    CHECK(m.snapshot().buffers == 102);
    m.record(-1.0, period);
    CHECK(m.snapshot().buffers == 103 && m.snapshot().lastLoad == 0.0);
}

TEST(recent_load_time_constant) {
    // The smoothed load follows a step with a time constant in seconds of
    // audio, the same for 64- and 1024-frame buffers.
    for (int frames : {64, 1024}) {
        LoadMeter m;
        const double period = frames / 48000.0;
        const int perSecond = static_cast<int>(1.0 / period);
        for (int i = 0; i < perSecond; ++i) m.record(0.2 * period, period);
        CHECK_NEAR(m.snapshot().recentLoad, 0.2, 1e-9);
        // One time constant after a step to 60 %: about 63 % of the way.
        const int tau = static_cast<int>(LoadMeter::kRecentSeconds / period + 0.5);
        for (int i = 0; i < tau; ++i) m.record(0.6 * period, period);
        const double frac = (m.snapshot().recentLoad - 0.2) / 0.4;
        CHECK(frac > 0.58 && frac < 0.68);
    }
}

TEST(reset) {
    LoadMeter m;
    const double period = 1e-3;
    m.record(0.9e-3, period);
    m.requestReset();
    auto s = m.snapshot();  // pending: zeros
    CHECK(s.buffers == 0 && s.maxLoad == 0.0 && s.overBudget == 0);
    m.record(0.1e-3, period);
    s = m.snapshot();
    CHECK(s.buffers == 1);
    CHECK_NEAR(s.maxLoad, 0.1, 1e-12);
    CHECK_NEAR(s.meanLoad, 0.1, 1e-12);
    CHECK(s.overBudget == 0);
}

TEST(concurrent_reads) {
    LoadMeter m;
    std::atomic<bool> done{false};
    std::thread audio([&] {
        for (int i = 0; i < 200000; ++i) m.record((i % 100) * 1e-5, 1e-3);  // loads 0 .. 0.99
        done.store(true);
    });
    bool ok = true;
    int resets = 0;
    while (!done.load()) {
        const auto s = m.snapshot();
        if (s.meanLoad < 0.0 || s.meanLoad > 1.0 || s.maxLoad > 1.0 || s.recentLoad < 0.0) ok = false;
        if (++resets % 1000 == 0) m.requestReset();
    }
    audio.join();
    CHECK(ok);
    const auto s = m.snapshot();
    CHECK(s.maxLoad <= 0.99 + 1e-12);
}

TEST(percentiles_worst_buffer_and_frame_counts) {
    LoadMeter m;
    const double fs = 48000.0;
    // 990 buffers of 64 frames at 20 %, 10 buffers of 70 frames at 50 % (one
    // of them, the 500th buffer, at 90 % with its engine details), plus a
    // handful of odd frame counts.
    std::uint64_t worstAt = 0;
    for (int i = 0; i < 1000; ++i) {
        pm::host::BufferRecord b;
        b.frames = i % 100 == 7 ? 70 : 64;
        b.periodSeconds = b.frames / fs;
        double load = b.frames == 70 ? 0.5 : 0.2;
        if (i == 499) {
            b.frames = 70;
            b.periodSeconds = 70 / fs;
            load = 0.9;
            b.hammersInContact = 7;
            b.hammersActive = 12;
            b.awakeModes = 4321;
            b.events = 5;
            b.subBlocks = 9;
            b.cpuSeconds = 0.5 * load * b.periodSeconds;  // half of the wall time
            worstAt = static_cast<std::uint64_t>(i + 1);
        }
        b.renderSeconds = load * b.periodSeconds;
        m.record(b);
    }
    const auto s = m.snapshot();
    CHECK(s.buffers == 1000);
    // p99: 11 of 1000 buffers are above 20 %, so the 99th percentile is in
    // the 50 % bin (upper edge 50.25 %).
    CHECK(s.p99Load > 0.49 && s.p99Load <= 0.5025 + 1e-12);
    CHECK_NEAR(s.maxLoad, 0.9, 1e-12);
    // Render times: 20 % of 1333 us = 267 us; 50 % of 1458 us = 729 us.
    CHECK(s.p99Us >= 729.0 && s.p99Us <= 729.0 * 1.016);
    CHECK_NEAR(s.maxUs, 0.9 * 70 / fs * 1e6, 1e-6);
    CHECK(s.minFrames == 64 && s.maxFrames == 70 && s.distinctFrames == 2 && s.lastFrames == 64);
    CHECK(s.worst.index == worstAt && s.worst.frames == 70 && s.worst.hammersInContact == 7 &&
          s.worst.hammersActive == 12 && s.worst.awakeModes == 4321 && s.worst.events == 5 &&
          s.worst.subBlocks == 9);
    CHECK_NEAR(s.worst.periodUs, 70 / fs * 1e6, 1e-9);
    CHECK_NEAR(s.worst.cpuUs, 0.5 * s.worst.renderUs, 1e-9);
    CHECK_NEAR(s.maxStallUs, 0.5 * s.worst.renderUs, 1e-9);  // only that buffer had CPU time
    // Reset clears everything, histograms included.
    m.requestReset();
    pm::host::BufferRecord b;
    b.frames = 512;
    b.periodSeconds = 512 / fs;
    b.renderSeconds = 0.1 * b.periodSeconds;
    m.record(b);
    const auto r = m.snapshot();
    CHECK(r.buffers == 1 && r.distinctFrames == 1 && r.minFrames == 512 && r.maxFrames == 512);
    CHECK(r.p99Load <= 0.1 + 1e-12 && r.p99Load > 0.0975);
    CHECK(r.worst.index == 1 && r.worst.frames == 512 && r.worst.cpuUs < 0.0);
    CHECK(r.maxStallUs < 0.0);
}

// Phase timing: the split of the worst buffer, the mean over all timed
// buffers and the mean over the top 1 % by render time; untimed buffers
// count in the load statistics but not in the split. The worst buffer's
// event kinds and recompute counts are kept with it.
TEST(phase_split_worst_and_top_percent) {
    LoadMeter m;
    const double fs = 48000.0;
    // 1000 timed buffers: 980 at 100 us (events 10, free 90), 20 at 400 us
    // (events 300, free 100); one more, untimed, at 1000 us is the worst.
    auto rec = [&](double us, double ev, double fr, bool timed) {
        pm::host::BufferRecord b;
        b.frames = 64;
        b.periodSeconds = 64 / fs;
        b.renderSeconds = us * 1e-6;
        b.timed = timed;
        b.phaseUs[0] = ev;
        b.phaseUs[2] = fr;
        b.noteOffs = timed ? 0 : 10;
        b.damperUpdates = timed ? 0 : 10;
        b.responseGroups = timed ? 0 : 123;
        m.record(b);
    };
    for (int i = 0; i < 1000; ++i) {
        const bool slow = i % 50 == 3;
        rec(slow ? 400.0 : 100.0, slow ? 300.0 : 10.0, slow ? 100.0 : 90.0, true);
    }
    rec(1000.0, 0.0, 0.0, false);
    auto s = m.snapshot();
    CHECK(s.timedBuffers == 1000);
    CHECK(s.topCount == 20);  // the 1 % (10) rounded up to the whole 400 us bin
    CHECK_NEAR(s.topPhaseUs[0], 300.0, 1e-9);
    CHECK_NEAR(s.topPhaseUs[2], 100.0, 1e-9);
    CHECK_NEAR(s.meanPhaseUs[0], (980 * 10.0 + 20 * 300.0) / 1000, 1e-9);
    CHECK(!s.worst.timed && s.worst.noteOffs == 10 && s.worst.damperUpdates == 10 &&
          s.worst.responseGroups == 123);
    // A timed worst buffer carries its own split.
    rec(2000.0, 1500.0, 400.0, true);
    s = m.snapshot();
    CHECK(s.worst.timed && s.worst.phaseUs[0] == 1500.0 && s.worst.phaseUs[2] == 400.0);
    CHECK(s.worst.noteOffs == 0);
    m.requestReset();
    rec(100.0, 1.0, 2.0, false);
    s = m.snapshot();
    CHECK(s.timedBuffers == 0 && s.topCount == 0 && s.meanPhaseUs[0] == 0.0);
}

TEST(worst_buffer_is_never_torn) {
    // The worst-buffer fields always come from one buffer: frames and
    // awake modes are written together (awake = 10 * frames).
    LoadMeter m;
    std::atomic<bool> done{false};
    std::thread audio([&] {
        for (int i = 0; i < 300000; ++i) {
            pm::host::BufferRecord b;
            b.frames = 1 + i % 4000;
            b.awakeModes = 10 * b.frames;
            b.periodSeconds = 1e-3;
            b.renderSeconds = 1e-3 * (i % 5000) / 5000.0;
            m.record(b);
            if (i % 5000 == 4999) m.requestReset();
        }
        done.store(true);
    });
    bool ok = true;
    while (!done.load()) {
        const auto s = m.snapshot();
        if (s.worst.awakeModes != 10 * s.worst.frames) ok = false;
    }
    audio.join();
    CHECK(ok);
}

// Per-CPU tally (0.11): render calls and time by the CPU they started on,
// migrations (start != end), unknown CPUs, the worst buffer's CPUs, the
// probe's parts per CPU and overall; a reset clears all of it.
TEST(cpu_tally_and_probe_parts) {
    LoadMeter m;
    const double period = 512.0 / 44100.0;
    auto rec = [&](double us, int c0, int c1) {
        pm::host::BufferRecord b;
        b.renderSeconds = us * 1e-6;
        b.periodSeconds = period;
        b.cpuStart = c0;
        b.cpuEnd = c1;
        m.record(b);
    };
    for (int i = 0; i < 30; ++i) rec(3000.0, 1, 1);  // a slow core
    for (int i = 0; i < 70; ++i) rec(1000.0, 6, 6);  // a fast core
    rec(5000.0, 6, 2);                               // worst, migrated mid-buffer
    rec(500.0, -1, -1);                              // CPU not read
    rec(500.0, 64, 64);                              // out of the table
    for (int i = 0; i < 10; ++i) m.recordProbe(4.0, 30.0, 1);
    for (int i = 0; i < 10; ++i) m.recordProbe(2.0, 8.0, 6);
    m.recordProbe(2.0, 8.0, -1);
    m.recordProbe(20.0);  // whole probe only (no parts)
    const auto s = m.snapshot();
    const pm::host::CpuUsage& c = s.cpu;
    CHECK(s.buffers == 103);
    CHECK(c.tracked == 101 && c.unknown == 2 && c.migrations == 1);
    CHECK(c.buffers[1] == 30 && c.buffers[6] == 71 && c.buffers[2] == 0);
    CHECK_NEAR(c.renderUs[1], 30 * 3000.0, 1e-6);
    CHECK_NEAR(c.renderUs[6], 70 * 1000.0 + 5000.0, 1e-6);
    CHECK(c.worstStart == 6 && c.worstEnd == 2);
    CHECK(s.worst.cpuStart == 6 && s.worst.cpuEnd == 2);
    CHECK(c.probeRuns[1] == 10 && c.probeRuns[6] == 10);
    CHECK_NEAR(c.probeThroughputUs[1] / c.probeRuns[1], 30.0, 1e-9);
    CHECK_NEAR(c.probeLatencyUs[6] / c.probeRuns[6], 2.0, 1e-9);
    CHECK(s.probe.count == 22 && s.probe.latency.count == 21 && s.probe.throughput.count == 21);
    CHECK_NEAR(s.probe.throughput.minUs, 8.0, 1e-12);
    CHECK_NEAR(s.probe.throughput.maxUs, 30.0, 1e-12);
    CHECK_NEAR(s.probe.throughput.meanUs, (10 * 30.0 + 11 * 8.0) / 21.0, 1e-9);
    CHECK(std::fabs(s.probe.throughput.medianUs - 8.0) < 0.02 * 8.0);  // 11 of 21 at 8 us: bin centre
    CHECK_NEAR(s.probe.maxUs, 34.0, 1e-12);
    CHECK_NEAR(s.probe.minUs, 10.0, 1e-12);
    m.requestReset();
    rec(100.0, 3, 3);
    const auto r = m.snapshot();
    CHECK(r.cpu.tracked == 1 && r.cpu.buffers[3] == 1 && r.cpu.buffers[6] == 0 && r.cpu.migrations == 0);
    CHECK(r.probe.count == 0 && r.probe.latency.count == 0 && r.cpu.probeRuns[1] == 0);
}
