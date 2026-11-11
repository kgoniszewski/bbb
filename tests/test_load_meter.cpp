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
