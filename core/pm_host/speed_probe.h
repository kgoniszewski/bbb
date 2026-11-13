// Speed probe: a small fixed workload whose run time tells how fast the
// calling thread is executing, independent of what the engine is doing.
// The host runs it on the audio thread every Nth buffer (pm_host_set_speed_probe)
// and the benchmark runs it on its own thread (pm_speed_probe_run_us); the
// ratio of the two is the speed at which the audio thread actually ran
// (core type and clock), which the render load alone cannot separate from
// the engine's work.
//
// Two parts, timed separately (API 0.11), because an efficiency core or a
// low clock slows them by very different factors:
//  - latency-bound: a scalar double Newton chain with square roots and
//    divisions (the hammer contact solve) and one dependent chain of vector
//    multiply-adds. Each operation waits for the previous one, so the time
//    is set by operation latencies and the clock; a narrow core with the
//    same latencies loses only its clock ratio.
//  - throughput-bound: the mode-bank kernel's shape, 1024 complex one-pole
//    modes (state and coefficients, 16 KiB, L1-resident) stepped 48 times,
//    with the output summed into four independent accumulators. Hundreds of
//    independent vector operations are in flight, so the time is set by the
//    core's issue width, vector units and load/store ports as well as the
//    clock: this part loses what the engine's wide inner loops lose.
// The arrays are initialised outside the timed parts. Deterministic, stack
// only (16 KiB; no allocation, no lock, no system call besides the clock),
// and the result is returned so that it cannot be optimised away. About
// 10-15 us on the x86 development container (SSE2 build).
#pragma once

#include <chrono>
#include <cmath>

#include "pm_base/simd.h"

namespace pm::host {

inline constexpr int kProbeNewton = 320;
inline constexpr int kProbeFmaChain = 1024;
inline constexpr int kProbeBankModes = 1024;  // a multiple of 4 x every SIMD width
inline constexpr int kProbeBankSteps = 48;
static_assert(kProbeBankModes % (4 * simd::kWidth) == 0, "four accumulators of whole vectors");

// Keeps the compiler from moving the workload out of the timed interval:
// the value passes through an empty asm with a memory clobber, so it is
// opaque and ordered against the clock calls (which may touch memory).
template <typename T>
inline void opaque(T& v) noexcept {
#if defined(__GNUC__) || defined(__clang__)
    __asm__ __volatile__("" : "+r"(v) : : "memory");
#else
    volatile T t = v;
    v = t;
#endif
}

// State and coefficients of the throughput part.
struct ProbeBank {
    alignas(64) float zr[kProbeBankModes];
    alignas(64) float zi[kProbeBankModes];
    alignas(64) float cr[kProbeBankModes];
    alignas(64) float ci[kProbeBankModes];
};

// Coefficients inside the unit circle (|c|^2 <= 0.9988), spread in angle,
// without trigonometry; state 1/(i+1).
inline void probeBankInit(ProbeBank& b) noexcept {
    for (int i = 0; i < kProbeBankModes; ++i) {
        b.cr[i] = 0.9990f - 0.0004f * static_cast<float>(i & 255) * (1.0f / 256.0f);
        b.ci[i] = 0.03f * static_cast<float>((i * 37) & 63) * (1.0f / 64.0f);
        b.zr[i] = 1.0f / static_cast<float>(i + 1);
        b.zi[i] = 0.0f;
    }
}

// Throughput part: z <- c z + drive per mode and step; returns the sum of
// the imaginary parts over all modes and steps.
inline float probeThroughputWork(ProbeBank& b, float drive) noexcept {
    using namespace simd;
    constexpr int W = kWidth;
    vf acc0 = zero(), acc1 = zero(), acc2 = zero(), acc3 = zero();
    const vf in = set1(drive);
    for (int n = 0; n < kProbeBankSteps; ++n)
        for (int i = 0; i < kProbeBankModes; i += 4 * W) {
            vf o[4];
            for (int k = 0; k < 4; ++k) {
                float* zr = b.zr + i + k * W;
                float* zi = b.zi + i + k * W;
                const vf r = load(zr), m = load(zi), a = load(b.cr + i + k * W), c = load(b.ci + i + k * W);
                const vf nr = fnmadd(c, m, fmadd(a, r, in));
                const vf ni = fmadd(c, r, mul(a, m));
                store(zr, nr);
                store(zi, ni);
                o[k] = ni;
            }
            acc0 = add(acc0, o[0]);
            acc1 = add(acc1, o[1]);
            acc2 = add(acc2, o[2]);
            acc3 = add(acc3, o[3]);
        }
    return hsum(add(add(acc0, acc1), add(acc2, acc3)));
}

// Latency part: Newton for x^2.5 = a with a changing every step (a
// dependent chain of sqrt and division, like the contact solver's), then a
// dependent vector multiply-add chain.
inline float probeLatencyWork(float drive) noexcept {
    double x = 1.0;
    for (int it = 0; it < kProbeNewton; ++it) {
        const double a = 1.0 + 0.0625 * (it & 15);
        const double s = std::sqrt(x);
        const double f = x * x * s - a;
        x -= f / (2.5 * x * s);
        if (!(x > 1e-3)) x = 1.0;
    }
    using namespace simd;
    vf v = set1(static_cast<float>(x));
    const vf g = set1(0.999f), in = set1(drive);
    for (int it = 0; it < kProbeFmaChain; ++it) v = fmadd(v, g, in);
    return hsum(v);
}

struct ProbeTimes {
    double latencyUs = 0.0;
    double throughputUs = 0.0;
    double totalUs() const noexcept { return latencyUs + throughputUs; }
};

// Runs both parts once on the calling thread; returns their wall times in
// microseconds and the checksum in *sink.
inline ProbeTimes timeSpeedProbeSplit(float* sink) noexcept {
    ProbeBank bank;
    probeBankInit(bank);
    float drive = 1e-3f;
    opaque(drive);
    const auto t0 = std::chrono::steady_clock::now();
    float a = probeLatencyWork(drive);
    opaque(a);
    const auto t1 = std::chrono::steady_clock::now();
    float b = probeThroughputWork(bank, drive);
    opaque(b);
    const auto t2 = std::chrono::steady_clock::now();
    if (sink) *sink = a + b;
    ProbeTimes t;
    t.latencyUs = std::chrono::duration<double, std::micro>(t1 - t0).count();
    t.throughputUs = std::chrono::duration<double, std::micro>(t2 - t1).count();
    return t;
}

// The whole probe (both parts), microseconds.
inline double timeSpeedProbe(float* sink) noexcept { return timeSpeedProbeSplit(sink).totalUs(); }

}  // namespace pm::host
