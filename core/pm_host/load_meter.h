// Render-load meter (plan §4.5): per-buffer render time as a fraction of the
// buffer period. record() runs on the audio thread (wait-free, no
// allocation); snapshot() and requestReset() from any other thread. Times
// are arguments, so the meter is clock-free and testable.
#pragma once

#include <atomic>
#include <cstdint>

namespace pm::host {

struct LoadSnapshot {
    double meanLoad = 0.0;    // mean of render time / period since the last reset
    double recentLoad = 0.0;  // exponentially smoothed, time constant about kRecentSeconds of audio
    double maxLoad = 0.0;     // worst single buffer since the last reset
    double lastLoad = 0.0;
    std::uint64_t buffers = 0;     // since the last reset
    std::uint64_t overBudget = 0;  // buffers whose load exceeded the budget
    double budget = 0.0;           // the threshold for overBudget, a fraction of the period
};

class LoadMeter {
public:
    static constexpr double kRecentSeconds = 0.5;
    static constexpr double kDefaultBudget = 0.7;  // worst-buffer limit on the target (CLAUDE.md)

    // Audio thread. renderSeconds: time spent rendering the buffer;
    // periodSeconds: frames / sample rate. Ignores a non-positive period.
    void record(double renderSeconds, double periodSeconds) noexcept {
        if (!(periodSeconds > 0.0)) return;
        if (reset_.exchange(false, std::memory_order_acquire)) {
            sum_ = 0.0;
            n_ = 0;
            max_ = 0.0;
            over_ = 0;
            recent_ = 0.0;
        }
        const double load = renderSeconds > 0.0 ? renderSeconds / periodSeconds : 0.0;
        sum_ += load;
        ++n_;
        if (load > max_) max_ = load;
        if (load > budget_.load(std::memory_order_relaxed)) ++over_;
        // One-pole smoother with a time constant in seconds of audio, so the
        // reading does not depend on the buffer size.
        double a = periodSeconds / kRecentSeconds;
        if (a > 1.0) a = 1.0;
        recent_ = n_ == 1 ? load : recent_ + a * (load - recent_);
        mean_.store(sum_ / static_cast<double>(n_), std::memory_order_relaxed);
        recentPub_.store(recent_, std::memory_order_relaxed);
        maxPub_.store(max_, std::memory_order_relaxed);
        last_.store(load, std::memory_order_relaxed);
        overPub_.store(over_, std::memory_order_relaxed);
        count_.store(n_, std::memory_order_release);
    }

    // Any thread. Values are individually consistent; a snapshot taken while
    // record() runs may mix two consecutive buffers, which a display tolerates.
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
        return s;
    }

    // Any thread. The audio thread clears its totals at its next record();
    // until then snapshot() reports zeros.
    void requestReset() noexcept { reset_.store(true, std::memory_order_release); }

    void setBudget(double fraction) noexcept {
        if (fraction > 0.0) budget_.store(fraction, std::memory_order_relaxed);
    }

private:
    // Audio-thread state.
    double sum_ = 0.0, max_ = 0.0, recent_ = 0.0;
    std::uint64_t n_ = 0, over_ = 0;
    // Published.
    std::atomic<double> mean_{0.0}, recentPub_{0.0}, maxPub_{0.0}, last_{0.0};
    std::atomic<double> budget_{kDefaultBudget};
    std::atomic<std::uint64_t> count_{0}, overPub_{0};
    std::atomic<bool> reset_{false};
};

}  // namespace pm::host
