// Room: image-source early reflections + 16-line FDN (plan §1.7).
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <new>
#include <random>
#include <string>
#include <vector>

#include "pm_room/room.h"
#include "test_harness.h"

// ---- counting allocator, armed only around the calls under test
namespace {
std::atomic<bool> gArmed{false};
std::atomic<long> gAllocs{0};
void* countedAlloc(std::size_t n, std::size_t align) {
    if (gArmed.load(std::memory_order_relaxed)) gAllocs.fetch_add(1, std::memory_order_relaxed);
    if (n == 0) n = 1;
    void* p = nullptr;
    if (align <= alignof(std::max_align_t)) p = std::malloc(n);
    else if (posix_memalign(&p, align, n) != 0) p = nullptr;
    return p;
}
}  // namespace
void* operator new(std::size_t n) {
    void* p = countedAlloc(n, 0);
    if (!p) std::abort();
    return p;
}
void* operator new[](std::size_t n) { return operator new(n); }
void* operator new(std::size_t n, const std::nothrow_t&) noexcept { return countedAlloc(n, 0); }
void* operator new[](std::size_t n, const std::nothrow_t&) noexcept { return countedAlloc(n, 0); }
void* operator new(std::size_t n, std::align_val_t a) {
    void* p = countedAlloc(n, static_cast<std::size_t>(a));
    if (!p) std::abort();
    return p;
}
void* operator new[](std::size_t n, std::align_val_t a) { return operator new(n, a); }
void* operator new(std::size_t n, std::align_val_t a, const std::nothrow_t&) noexcept {
    return countedAlloc(n, static_cast<std::size_t>(a));
}
void* operator new[](std::size_t n, std::align_val_t a, const std::nothrow_t&) noexcept {
    return countedAlloc(n, static_cast<std::size_t>(a));
}
void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }
void operator delete(void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t, std::align_val_t) noexcept { std::free(p); }
void operator delete(void* p, const std::nothrow_t&) noexcept { std::free(p); }
void operator delete[](void* p, const std::nothrow_t&) noexcept { std::free(p); }
void operator delete(void* p, std::align_val_t, const std::nothrow_t&) noexcept { std::free(p); }
void operator delete[](void* p, std::align_val_t, const std::nothrow_t&) noexcept { std::free(p); }

namespace {

constexpr double kFs = 48000.0;
constexpr int kMaxFrames = 512;
using Vec = std::vector<float>;

std::unique_ptr<pm::Room> makeRoom(double decay, double hf, double bal, double size = 1.0, double levelDb = 0.0) {
    auto r = std::make_unique<pm::Room>();
    r->setDecay(decay);
    r->setHfRatio(hf);
    r->setEarlyLateBalance(bal);
    r->setSize(size);
    r->setLevelDb(levelDb);
    CHECK(r->prepare(kFs, kMaxFrames));
    return r;
}

// Impulse response (unit impulse into the given channels), 64-frame blocks.
void impulse(pm::Room& r, int len, bool left, bool right, Vec& L, Vec& R) {
    Vec inL(len, 0.0f), inR(len, 0.0f);
    if (left) inL[0] = 1.0f;
    if (right) inR[0] = 1.0f;
    L.assign(len, 0.0f);
    R.assign(len, 0.0f);
    for (int i = 0; i < len; i += 64) {
        const int n = std::min(64, len - i);
        r.process(inL.data() + i, inR.data() + i, L.data() + i, R.data() + i, n);
    }
}

// RBJ band-pass, one octave wide, 0 dB peak.
Vec octaveBand(const Vec& x, double fc, double bw = 1.0) {
    const double w0 = 2 * M_PI * fc / kFs, sn = std::sin(w0);
    const double alpha = sn * std::sinh(std::log(2.0) / 2 * bw * w0 / sn);
    const double a0 = 1 + alpha;
    const double b0 = alpha / a0, b2 = -alpha / a0, a1 = -2 * std::cos(w0) / a0, a2 = (1 - alpha) / a0;
    Vec y(x.size());
    double x1 = 0, x2 = 0, y1 = 0, y2 = 0;
    for (std::size_t i = 0; i < x.size(); ++i) {
        const double v = b0 * x[i] + b2 * x2 - a1 * y1 - a2 * y2;
        x2 = x1;
        x1 = x[i];
        y2 = y1;
        y1 = v;
        y[i] = float(v);
    }
    return y;
}

// Schroeder backward integration of e = a^2 + b^2, least-squares fit -5..-35 dB.
double schroederT60(const Vec& a, const Vec& b) {
    const std::size_t n = a.size();
    std::vector<double> E(n + 1, 0.0);
    for (std::size_t i = n; i-- > 0;) E[i] = E[i + 1] + double(a[i]) * a[i] + double(b[i]) * b[i];
    double sx = 0, sy = 0, sxx = 0, sxy = 0;
    int m = 0;
    for (std::size_t i = 0; i < n; ++i) {
        const double db = 10 * std::log10(E[i] / E[0] + 1e-300);
        if (db > -5) continue;
        if (db < -35) break;
        const double t = i / kFs;
        sx += t;
        sy += db;
        sxx += t * t;
        sxy += t * db;
        ++m;
    }
    const double slope = (m * sxy - sx * sy) / (m * sxx - sx * sx);
    return -60.0 / slope;
}

TEST(stability_monotone_decay) {
    std::mt19937 rng(1234);
    std::uniform_real_distribution<double> U(0.0, 1.0);
    double worst = 0.0;
    for (int k = 0; k < 10; ++k) {
        const double decay = k == 0 ? 0.3 : k == 1 ? 6.0 : 0.3 + 5.7 * U(rng);
        const double hf = 0.1 + 0.9 * U(rng), size = 0.5 + 1.5 * U(rng), bal = U(rng);
        auto r = makeRoom(decay, hf, bal, size);
        Vec L, R;
        const int len = int((1.2 * decay + 0.6) * kFs);
        impulse(*r, len, true, true, L, R);
        int start = r->lateInputDelay() + r->lineLength(15);
        for (int c = 0; c < 2; ++c)
            for (int t = 0; t < pm::Room::kTaps; ++t) start = std::max(start, r->earlyTap(c, t, nullptr));
        // 50 ms windows down to -60 dB, and 250 ms windows (sums of five).
        const int win = int(0.05 * kFs);
        std::vector<double> e;
        for (int i = start; i + win <= len; i += win) {
            double v = 0;
            for (int j = i; j < i + win; ++j) v += double(L[j]) * L[j] + double(R[j]) * R[j];
            if (!e.empty() && v < e[0] * 1e-6) break;
            e.push_back(v);
        }
        double maxRatio = 0, at = 0, maxRatio5 = 0;
        for (std::size_t i = 1; i < e.size(); ++i)
            if (e[i] / e[i - 1] > maxRatio) maxRatio = e[i] / e[i - 1], at = i * 0.05;
        for (std::size_t i = 9; i < e.size(); i += 5)
            maxRatio5 = std::max(maxRatio5, (e[i] + e[i - 1] + e[i - 2] + e[i - 3] + e[i - 4]) /
                                                (e[i - 5] + e[i - 6] + e[i - 7] + e[i - 8] + e[i - 9]));
        worst = std::max(worst, maxRatio);
        std::printf("  T60 %.2f s hf %.2f size %.2f bal %.2f: max 50 ms window growth %+.2f dB (at %.2f s), "
                    "max 250 ms growth %+.2f dB\n",
                    decay, hf, size, bal, 10 * std::log10(maxRatio), at, 10 * std::log10(maxRatio5));
        // Loosened from "never grows": with T60 = 6 s the expected drop per
        // 50 ms window is only 0.5 dB, while the energy of a 50 ms window of a
        // diffuse (Gaussian) tail fluctuates by several tenths of a dB, more
        // once the high frequencies have died away (fewer degrees of freedom).
        // So 50 ms windows may grow by up to 1.5 dB (measured worst +0.93 dB);
        // 250 ms windows must not grow.
        CHECK_LE(10 * std::log10(maxRatio), 1.5);
        CHECK_LE(maxRatio5, 1.0);
    }
    std::printf("  worst %.3f\n", worst);
}

TEST(t60_bands) {
    for (double T : {0.5, 1.8, 4.0}) {
        const double hf = 0.5;
        auto r = makeRoom(T, hf, 1.0);
        Vec L, R;
        impulse(*r, int((1.6 * T + 0.4) * kFs), true, true, L, R);
        const double t500 = schroederT60(octaveBand(L, 500), octaveBand(R, 500));
        const double t1k = schroederT60(octaveBand(L, 1000), octaveBand(R, 1000));
        const double t8k = schroederT60(octaveBand(L, 8000, 1.0 / 3), octaveBand(R, 8000, 1.0 / 3));
        std::printf("  set %.2f s: T60 500 Hz %.3f (%+.1f %%), 1 kHz %.3f (%+.1f %%), 8 kHz %.3f (target %.3f, %+.1f %%)\n",
                    T, t500, 100 * (t500 / T - 1), t1k, 100 * (t1k / T - 1), t8k, hf * T,
                    100 * (t8k / (hf * T) - 1));
        CHECK_NEAR(t500, T, 0.10 * T);
        CHECK_NEAR(t1k, T, 0.10 * T);
        CHECK_NEAR(t8k, hf * T, 0.20 * hf * T);
    }
}

// Runs a 1.2 s noise burst + tail with a decay change, a size change and a
// level change at fixed sample positions, split into calls by `next`.
template <class Split>
void runScenario(Split next, Vec& L, Vec& R) {
    auto r = makeRoom(1.8, 0.5, 0.5, 1.0, -3.0);
    const int N = int(1.2 * kFs);
    std::mt19937 rng(7);
    std::normal_distribution<float> g(0.0f, 0.1f);
    Vec inL(N, 0.0f), inR(N, 0.0f);
    for (int i = 0; i < int(0.4 * kFs); ++i) inL[i] = g(rng), inR[i] = g(rng);
    L.assign(N, 0.0f);
    R.assign(N, 0.0f);
    const int ev[3] = {12000, 21000, 30001};
    int i = 0;
    while (i < N) {
        if (i == ev[0]) r->setDecay(4.0);
        if (i == ev[1]) r->setSize(1.6);
        if (i == ev[2]) r->setLevelDb(-12.0), r->setHfRatio(0.3);
        int n = std::min({next(), N - i, kMaxFrames});
        for (int e : ev)
            if (i < e && i + n > e) n = e - i;
        r->process(inL.data() + i, inR.data() + i, L.data() + i, R.data() + i, n);
        i += n;
    }
}

TEST(buffer_split_invariance) {
    Vec refL, refR;
    runScenario([] { return 64; }, refL, refR);
    std::mt19937 rng(99);
    std::uniform_int_distribution<int> d(1, kMaxFrames);
    const int sizes[] = {1, 7, 512, 0};
    for (int s : sizes) {
        Vec L, R;
        if (s > 0) runScenario([s] { return s; }, L, R);
        else runScenario([&] { return d(rng); }, L, R);
        const bool same = std::memcmp(L.data(), refL.data(), L.size() * 4) == 0 &&
                          std::memcmp(R.data(), refR.data(), R.size() * 4) == 0;
        std::printf("  split %s: %s\n", s ? std::to_string(s).c_str() : "random", same ? "bit-identical" : "DIFFERS");
        CHECK(same);
    }
}

TEST(no_allocation_in_process) {
    auto r = makeRoom(1.8, 0.5, 0.5);
    std::mt19937 rng(3);
    std::normal_distribution<float> g(0.0f, 0.1f);
    Vec inL(64), inR(64), oL(64), oR(64);
    const std::size_t live = pm::alignedLiveBytes();
    gAllocs = 0;
    gArmed = true;
    for (int b = 0; b < 3000; ++b) {
        if (b == 200) r->setDecay(5.0);
        if (b == 400) r->setSize(2.0);
        if (b == 800) r->setEnabled(false);
        if (b == 1000) r->setEnabled(true);
        for (int i = 0; i < 64; ++i) inL[i] = b < 1500 ? g(rng) : 0.0f, inR[i] = b < 1500 ? g(rng) : 0.0f;
        r->process(inL.data(), inR.data(), oL.data(), oR.data(), 64);
    }
    gArmed = false;
    std::printf("  allocations during process: %ld, aligned bytes change: %ld\n", gAllocs.load(),
                long(pm::alignedLiveBytes()) - long(live));
    CHECK(gAllocs.load() == 0);
    CHECK(pm::alignedLiveBytes() == live);
}

TEST(silence_and_restart) {
    auto r = makeRoom(1.8, 0.5, 0.5);
    const int len = int(1.0 * kFs);
    Vec ref1L, ref1R;
    impulse(*r, len, true, false, ref1L, ref1R);
    // Continue with zeros until silent.
    Vec z(64, 0.0f), oL(64), oR(64);
    long samples = 0;
    while (!r->silent() && samples < long(30 * kFs)) {
        r->process(z.data(), z.data(), oL.data(), oR.data(), 64);
        samples += 64;
    }
    std::printf("  silent after %.2f s of zeros (impulse + 1 s already processed)\n", samples / kFs);
    CHECK(r->silent());
    bool allZero = true;
    for (int b = 0; b < 750; ++b) {
        r->process(z.data(), z.data(), oL.data(), oR.data(), 64);
        for (int i = 0; i < 64; ++i) allZero &= oL[i] == 0.0f && oR[i] == 0.0f;
    }
    CHECK(allZero);
    // Shift the impulse off the control-block grid.
    r->process(z.data(), z.data(), oL.data(), oR.data(), 13);
    Vec L, R;
    impulse(*r, len, true, false, L, R);
    const bool same = std::memcmp(L.data(), ref1L.data(), len * 4) == 0 && std::memcmp(R.data(), ref1R.data(), len * 4) == 0;
    std::printf("  output exactly zero while silent: %s; response after silence %s\n", allZero ? "yes" : "no",
                same ? "bit-identical to response from reset" : "DIFFERS");
    CHECK(same);
}

TEST(early_reflection_taps) {
    pm::RoomGeometry g;
    g.dims[0] = 10, g.dims[1] = 8, g.dims[2] = 4;
    g.source[0] = 3, g.source[1] = 4, g.source[2] = 1;
    for (int c = 0; c < 2; ++c) g.mic[c][0] = 5, g.mic[c][1] = 4, g.mic[c][2] = 1;
    for (double& a : g.absorption) a = 0.19;  // reflection amplitude 0.9
    pm::Room r;
    CHECK(r.prepare(kFs, kMaxFrames));
    CHECK(r.configure(g));
    // Direct 2 m. Floor image (3, 4, -1): sqrt(8) m. Wall x = 0 image (-3, 4, 1): 8 m.
    const struct {
        const char* name;
        double dist;
    } img[] = {{"floor", std::sqrt(8.0)}, {"wall x=0", 8.0}};
    for (const auto& im : img) {
        const double delay = (im.dist - 2.0) / 343.0 * kFs, gain = 2.0 / im.dist * 0.9;
        int best = -1;
        for (int t = 0; t < pm::Room::kTaps; ++t)
            if (best < 0 || std::fabs(r.earlyTap(0, t, nullptr) - delay) < std::fabs(r.earlyTap(0, best, nullptr) - delay))
                best = t;
        float gt = 0;
        const int dt = r.earlyTap(0, best, &gt);
        const double dbErr = 20 * std::log10(gt / gain);
        std::printf("  %s: tap %d samples (exact %.2f), gain %.4f (exact %.4f, %+.3f dB)\n", im.name, dt, delay, gt, gain,
                    dbErr);
        CHECK_NEAR(dt, delay, 1.0);
        CHECK_NEAR(dbErr, 0.0, 0.1);
    }
}

TEST(late_stereo_correlation) {
    for (int mode = 0; mode < 2; ++mode) {
        auto r = makeRoom(1.8, 0.5, 1.0);
        Vec L, R;
        impulse(*r, int(1.6 * kFs), true, mode == 0, L, R);
        const int a = r->lateInputDelay() + r->lineLength(15) + int(0.05 * kFs), b = int(1.5 * kFs);
        double lr = 0, ll = 0, rr = 0;
        for (int i = a; i < b; ++i) lr += double(L[i]) * R[i], ll += double(L[i]) * L[i], rr += double(R[i]) * R[i];
        const double rho = lr / std::sqrt(ll * rr);
        std::printf("  impulse into %s: late L/R correlation %+.3f\n", mode == 0 ? "both channels" : "left only", rho);
        CHECK_LE(std::fabs(rho), 0.3);
    }
}

TEST(echo_density_kurtosis) {
    auto r = makeRoom(1.8, 0.5, 1.0);
    Vec L, R;
    impulse(*r, int(1.2 * kFs), true, true, L, R);
    const int win = int(0.02 * kFs);
    double sum = 0, mx = 0, mn = 1e9;
    int m = 0;
    for (int i = int(0.15 * kFs); i + win <= int(1.0 * kFs); i += win) {
        double s2 = 0, s4 = 0;
        for (int j = i; j < i + win; ++j) {
            const double x2 = double(L[j]) * L[j];
            s2 += x2;
            s4 += x2 * x2;
        }
        const double k = win * s4 / (s2 * s2);
        sum += k, mx = std::max(mx, k), mn = std::min(mn, k), ++m;
    }
    std::printf("  kurtosis in 20 ms windows, 150 ms..1 s: mean %.2f, min %.2f, max %.2f\n", sum / m, mn, mx);
    CHECK_NEAR(sum / m, 3.0, 1.0);
}

// The physical ratio is a mid-frequency quantity (the absorption filters
// shorten the high-frequency tail by design), so it is measured in the 1 kHz
// octave: band energy of the wet response over that of the unit impulse.
TEST(reverb_to_direct_ratio) {
    auto r = makeRoom(1.8, 0.5, 0.5);
    Vec L, R;
    const int len = int(4.0 * kFs);
    impulse(*r, len, true, false, L, R);
    Vec d(len, 0.0f);
    d[0] = 1.0f;
    const Vec bL = octaveBand(L, 1000), bD = octaveBand(d, 1000);
    double eE = 0, eAll = 0, eD = 0;
    const int lateStart = r->lateInputDelay() + r->lineLength(0);
    for (int i = 0; i < len; ++i) {
        eAll += double(bL[i]) * bL[i];
        eD += double(bD[i]) * bD[i];
        if (i < lateStart) eE += double(bL[i]) * bL[i];
    }
    const double meas = 10 * std::log10(eAll / eD), target = r->reverbToDirectDb();
    std::printf("  level 0 dB, 1 kHz octave: wet/direct %.2f dB (early part %.2f dB), physical (r/r_c)^2 %.2f dB; "
                "default level %.0f dB\n",
                meas, 10 * std::log10(eE / eD), target, pm::Room::kDefaultLevelDb);
    CHECK_NEAR(meas, target, 1.5);
}

TEST(cost) {
    auto r = makeRoom(1.8, 0.5, 0.5);
    std::mt19937 rng(5);
    std::normal_distribution<float> g(0.0f, 0.1f);
    const int N = int(kFs);
    Vec inL(N), inR(N), oL(64), oR(64);
    for (int i = 0; i < N; ++i) inL[i] = g(rng), inR[i] = g(rng);
    std::vector<double> ns;
    for (int run = 0; run < 7; ++run) {
        const auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i + 64 <= N; i += 64) r->process(inL.data() + i, inR.data() + i, oL.data(), oR.data(), 64);
        const auto t1 = std::chrono::steady_clock::now();
        ns.push_back(std::chrono::duration<double, std::nano>(t1 - t0).count() / N);
    }
    std::sort(ns.begin(), ns.end());
    const double med = ns[ns.size() / 2];
    std::printf("  process(): %.1f ns/sample (stereo frame), %.3f %% of a 64-frame buffer at 48 kHz\n", med,
                100.0 * med * 1e-9 * kFs);
}

}  // namespace
