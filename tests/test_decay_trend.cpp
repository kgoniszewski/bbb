// Key-to-key regularity of the prompt decay (plan §1.6 note (6)): the early
// decay of each key's first partial follows a smooth trend across the
// register in which the board's low-frequency regime lies, instead of
// jumping by large factors between neighbouring keys.
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

#include "decay_utils.h"
#include "piano_utils.h"
#include "test_harness.h"

using namespace pm_test;

namespace {

constexpr double kFs = 48000.0;

// Early T60 of the partial at f: least-squares slope of its envelope from
// 1 to 15 dB below its maximum (the prompt sound).
double earlyT60(const std::vector<float>& x, double f) {
    const double win = std::clamp(24.0 / f, 0.02, 0.06), hop = 0.005;
    const auto bb = baseband(x, kFs, f, win, hop);
    std::vector<double> db(bb.size());
    std::size_t im = 0;
    for (std::size_t k = 0; k < bb.size(); ++k) {
        db[k] = 20.0 * std::log10(std::abs(bb[k]) + 1e-30);
        if (db[k] > db[im]) im = k;
    }
    std::vector<double> t, y;
    for (std::size_t k = im; k < bb.size(); ++k) {
        const double d = db[im] - db[k];
        if (d > 15.0) break;
        if (d >= 1.0) {
            t.push_back(k * hop);
            y.push_back(db[k]);
        }
    }
    if (t.size() < 4) return 0.0;
    double s, c, e;
    lineFit(t, y, 0, static_cast<int>(t.size()), s, c, e);
    return s < 0.0 ? -60.0 / s : 0.0;
}

// Residual of log(v) about a least-squares cubic in the key number.
double cubicResidualRms(const std::vector<double>& key, const std::vector<double>& v) {
    const int n = static_cast<int>(key.size());
    double A[4][5] = {};
    const double k0 = 0.5 * (key.front() + key.back()), ks = 0.5 * (key.back() - key.front());
    for (int i = 0; i < n; ++i) {
        const double x = (key[i] - k0) / ks, y = std::log(v[i]);
        const double p[4] = {1.0, x, x * x, x * x * x};
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) A[r][c] += p[r] * p[c];
            A[r][4] += p[r] * y;
        }
    }
    for (int c = 0; c < 4; ++c)
        for (int r = c + 1; r < 4; ++r) {
            const double m = A[r][c] / A[c][c];
            for (int q = c; q < 5; ++q) A[r][q] -= m * A[c][q];
        }
    double coef[4];
    for (int r = 3; r >= 0; --r) {
        double s = A[r][4];
        for (int q = r + 1; q < 4; ++q) s -= A[r][q] * coef[q];
        coef[r] = s / A[r][r];
    }
    double ss = 0.0;
    for (int i = 0; i < n; ++i) {
        const double x = (key[i] - k0) / ks;
        const double e = std::log(v[i]) - (coef[0] + x * (coef[1] + x * (coef[2] + x * coef[3])));
        ss += e * e;
    }
    return std::sqrt(ss / n);
}

}  // namespace

// Keys 40-63 (C4-B5, first partials 262-990 Hz, inside the board's low
// regime) at MIDI velocity 80, held: the early T60 of the first partial on
// the engine output. Measured (g++, SSE2): with the v0 board the log-RMS
// residual about a cubic trend was a factor 1.82 and neighbouring keys
// differed by up to 8.4x (keys 50/51: Re Y at their fundamentals 2.5e-4 and
// 2.2e-3 s/kg, one board mode apart); with the low-regime board 1.40 and
// 2.6x, at the same median (2.0 s). Thresholds 1.6 and 3.5 sit between the
// two. Real boards scatter decay times too (modal overlap of about 1-2 in
// this range), so a smooth curve is not the target.
TEST(prompt_decay_follows_a_trend) {
    pm::Engine eng;
    eng.setNoiseLevels(0.0f, 0.0f);
    eng.prepare(kFs, 64);
    std::vector<double> keys, t60;
    for (int key = 40; key <= 63; ++key) {
        eng.prepare(kFs, 64);
        const double f1 = eng.table().keys[key - 1].f1;
        const Stereo s = renderEngine(eng, {noteOn(0, key + 20, 80)}, static_cast<int>(3.0 * kFs));
        std::vector<float> m(s.l.size());
        for (std::size_t i = 0; i < m.size(); ++i) m[i] = s.l[i] + s.r[i];
        const double t = earlyT60(m, f1);
        std::printf("  key %d (%.0f Hz): early T60 %.2f s\n", key, f1, t);
        CHECK(t > 0.0);
        keys.push_back(key);
        t60.push_back(std::max(t, 1e-3));
    }
    double worst = 1.0;
    for (std::size_t i = 1; i < t60.size(); ++i)
        worst = std::max(worst, std::max(t60[i] / t60[i - 1], t60[i - 1] / t60[i]));
    std::vector<double> sorted = t60;
    std::sort(sorted.begin(), sorted.end());
    const double resid = std::exp(cubicResidualRms(keys, t60));
    std::printf("  keys 40-63: residual about a cubic trend x%.2f, largest neighbour ratio %.2f, median %.2f s\n",
                resid, worst, sorted[sorted.size() / 2]);
    CHECK(resid <= 1.6);
    CHECK(worst <= 3.5);
}
