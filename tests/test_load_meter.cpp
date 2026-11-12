// Render-load meter: mean, max, over-budget count, smoothing, reset, and
// concurrent reads while the audio thread records.
#include <atomic>
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
