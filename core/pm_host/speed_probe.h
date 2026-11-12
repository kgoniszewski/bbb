// Speed probe: a small fixed workload whose run time tells how fast the
// calling thread is executing, independent of what the engine is doing.
// The host runs it on the audio thread every Nth buffer (pm_host_set_speed_probe)
// and the benchmark runs it on its own thread (pm_speed_probe_run_us); the
// ratio of the two is the speed at which the audio thread actually ran
// (core type and clock), which the render load alone cannot separate from
// the engine's work.
//
// The work resembles the engine's: a block of complex one-pole modes through
// the SIMD wrapper (the bank kernels' recursion) and a scalar double Newton
// chain with square roots and divisions (the hammer contact solve). It is
// deterministic, uses only the stack (no allocation, no locks, no system
// call besides the clock), and its result is returned so that it cannot be
// optimised away. A few microseconds on a fast core (about 8 us on the x86
// development container, SSE2 build).
#pragma once

#include <chrono>
#include <cmath>

#include "pm_base/simd.h"

namespace pm::host {

inline constexpr int kProbeModes = 128;   // a multiple of every SIMD width
inline constexpr int kProbeSteps = 96;
inline constexpr int kProbeNewton = 320;

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

// The workload; 'drive' is the modes' input (1e-3). Returns a checksum.
inline float speedProbeWork(float drive = 1e-3f) noexcept {
    alignas(64) float zr[kProbeModes], zi[kProbeModes], cr[kProbeModes], ci[kProbeModes];
    // Coefficients r e^{jw} by complex recurrence (no trigonometry), radius
    // just below 1, frequencies spread over the band.
    double ar = 0.9995, ai = 0.0;
    const double sr = std::cos(0.023), si = std::sin(0.023);
    for (int i = 0; i < kProbeModes; ++i) {
        const double nr = ar * sr - ai * si, ni = ar * si + ai * sr;
        ar = nr;
        ai = ni;
        cr[i] = static_cast<float>(ar);
        ci[i] = static_cast<float>(ai);
        zr[i] = 1.0f / static_cast<float>(i + 1);
        zi[i] = 0.0f;
    }
    using namespace simd;
    vf acc = zero();
    const vf in = set1(drive);
    for (int n = 0; n < kProbeSteps; ++n)
        for (int i = 0; i < kProbeModes; i += kWidth) {
            const vf r = load(zr + i), m = load(zi + i), a = load(cr + i), b = load(ci + i);
            const vf nr = fnmadd(b, m, fmadd(a, r, in));
            const vf ni = fmadd(b, r, mul(a, m));
            store(zr + i, nr);
            store(zi + i, ni);
            acc = add(acc, ni);
        }
    float sum = hsum(acc);
    // Newton for x^2.5 = a with a changing every step: a dependent chain of
    // sqrt and division, like the contact solver's.
    double x = 1.0;
    for (int it = 0; it < kProbeNewton; ++it) {
        const double a = 1.0 + 0.0625 * (it & 15);
        const double s = std::sqrt(x);
        const double f = x * x * s - a;
        x -= f / (2.5 * x * s);
        if (!(x > 1e-3)) x = 1.0;
    }
    return sum + static_cast<float>(x);
}

// Runs the workload once on the calling thread; returns its wall time in
// microseconds and the checksum in *sink.
inline double timeSpeedProbe(float* sink) noexcept {
    float drive = 1e-3f;
    const auto t0 = std::chrono::steady_clock::now();
    opaque(drive);
    float r = speedProbeWork(drive);
    opaque(r);
    const auto t1 = std::chrono::steady_clock::now();
    if (sink) *sink = r;
    return std::chrono::duration<double, std::micro>(t1 - t0).count();
}

}  // namespace pm::host
