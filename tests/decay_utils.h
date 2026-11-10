// Partial envelopes, two-stage decay fits and beat rates for the unison
// tests (the same method as tools/analysis/analyze_decay.py).
#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <numbers>
#include <vector>

namespace pm_test {

// Complex baseband of the partial at fHz: heterodyne, then a Hann-windowed
// moving average of 'win' seconds, sampled every 'hop' seconds. Frame k is
// centred at t = k * hop + win / 2.
inline std::vector<std::complex<double>> baseband(const std::vector<float>& x, double fs, double fHz,
                                                   double win, double hop) {
    const int nw = std::max(4, static_cast<int>(win * fs));
    const int nh = std::max(1, static_cast<int>(hop * fs));
    std::vector<double> w(static_cast<std::size_t>(nw));
    double ws = 0.0;
    for (int i = 0; i < nw; ++i) ws += (w[i] = 0.5 - 0.5 * std::cos(2.0 * std::numbers::pi * (i + 0.5) / nw));
    std::vector<std::complex<double>> out;
    const double cyc = fHz / fs;
    for (int s = 0; s + nw <= static_cast<int>(x.size()); s += nh) {
        std::complex<double> acc = 0.0;
        for (int i = 0; i < nw; ++i) {
            const double ph = std::fmod(cyc * (s + i), 1.0);
            acc += w[i] * double(x[s + i]) * std::polar(1.0, -2.0 * std::numbers::pi * ph);
        }
        out.push_back(2.0 * acc / ws);
    }
    return out;
}

struct TwoStage {
    double promptT60 = 0.0, afterT60 = 0.0;  // s
    double breakTime = 0.0;                  // s
    double crossoverDb = 0.0;                // level where the two lines meet, re the start
};

// Least-squares line through (t, y) in [i0, i1).
inline void lineFit(const std::vector<double>& t, const std::vector<double>& y, int i0, int i1,
                    double& slope, double& icpt, double& sse) {
    double st = 0, sy = 0, stt = 0, sty = 0;
    const int n = i1 - i0;
    for (int i = i0; i < i1; ++i) {
        st += t[i];
        sy += y[i];
        stt += t[i] * t[i];
        sty += t[i] * y[i];
    }
    const double den = n * stt - st * st;
    slope = den != 0.0 ? (n * sty - st * sy) / den : 0.0;
    icpt = (sy - slope * st) / n;
    sse = 0.0;
    for (int i = i0; i < i1; ++i) {
        const double e = y[i] - (icpt + slope * t[i]);
        sse += e * e;
    }
}

// Two straight lines in dB against time (envelope from t0 until it falls
// rangeDb below its start), breakpoint chosen by least squares.
inline TwoStage twoStageFit(const std::vector<std::complex<double>>& bb, double hop, double win,
                            double t0, double rangeDb = 60.0) {
    std::vector<double> t, y;
    double ref = 0.0;
    for (std::size_t k = 0; k < bb.size(); ++k) {
        const double tk = k * hop + 0.5 * win;
        if (tk < t0) continue;
        const double db = 20.0 * std::log10(std::abs(bb[k]) + 1e-30);
        if (t.empty()) ref = db;
        if (db < ref - rangeDb) break;
        t.push_back(tk);
        y.push_back(db - ref);
    }
    TwoStage r;
    const int n = static_cast<int>(t.size());
    if (n < 8) return r;
    double best = 1e300;
    for (int b = 4; b <= n - 4; ++b) {
        double s1, c1, e1, s2, c2, e2;
        lineFit(t, y, 0, b, s1, c1, e1);
        lineFit(t, y, b, n, s2, c2, e2);
        if (e1 + e2 < best && s1 < 0.0 && s2 < 0.0) {
            best = e1 + e2;
            r.promptT60 = -60.0 / s1;
            r.afterT60 = -60.0 / s2;
            r.breakTime = t[b];
            const double tc = s1 != s2 ? (c2 - c1) / (s1 - s2) : t[b];
            r.crossoverDb = c1 + s1 * tc;
        }
    }
    return r;
}

// Beat rate: frequency difference of the two components of a second-order
// linear-prediction (Prony) model of the baseband over [t0, t0 + span),
// i.e. of the two dominant damped exponentials. Exact for a signal made of
// two damped sinusoids; with more (weaker) components it is the beat of the
// two strongest. Returns 0 if the weaker of the two components starts more
// than 30 dB below the stronger (no audible beat), or the poles are closer
// than 0.01 Hz.
inline double beatRate(const std::vector<std::complex<double>>& bb, double hop, double win, double t0,
                       double span) {
    using C = std::complex<double>;
    const int k0 = std::max(0, static_cast<int>(std::ceil((t0 - 0.5 * win) / hop)));
    const int n = std::min(static_cast<int>(span / hop), static_cast<int>(bb.size()) - k0);
    if (n < 8) return -1.0;
    // x[k] = a1 x[k-1] + a2 x[k-2], least squares (normal equations).
    C m11 = 0, m12 = 0, m22 = 0, r1 = 0, r2 = 0;
    for (int k = k0 + 2; k < k0 + n; ++k) {
        const C x1 = bb[k - 1], x2 = bb[k - 2], y = bb[k];
        m11 += std::conj(x1) * x1;
        m12 += std::conj(x1) * x2;
        m22 += std::conj(x2) * x2;
        r1 += std::conj(x1) * y;
        r2 += std::conj(x2) * y;
    }
    const C det = m11 * m22 - m12 * std::conj(m12);
    const C a1 = (r1 * m22 - m12 * r2) / det, a2 = (m11 * r2 - std::conj(m12) * r1) / det;
    // Roots of z^2 - a1 z - a2.
    const C disc = std::sqrt(a1 * a1 + 4.0 * a2);
    const C z1 = 0.5 * (a1 + disc), z2 = 0.5 * (a1 - disc);
    const double df = std::fabs(std::arg(z1) - std::arg(z2)) / (2.0 * std::numbers::pi * hop);
    // Amplitudes at the window start: least squares over x[k] = c1 z1^k + c2 z2^k.
    C g11 = 0, g12 = 0, g22 = 0, h1 = 0, h2 = 0, p1 = 1.0, p2 = 1.0;
    for (int k = 0; k < n; ++k) {
        g11 += std::conj(p1) * p1;
        g12 += std::conj(p1) * p2;
        g22 += std::conj(p2) * p2;
        h1 += std::conj(p1) * bb[k0 + k];
        h2 += std::conj(p2) * bb[k0 + k];
        p1 *= z1;
        p2 *= z2;
    }
    const C gd = g11 * g22 - g12 * std::conj(g12);
    const double c1 = std::abs((h1 * g22 - g12 * h2) / gd), c2 = std::abs((g11 * h2 - std::conj(g12) * h1) / gd);
    if (std::min(c1, c2) < 0.03 * std::max(c1, c2) || df < 0.01) return 0.0;
    return df;
}

}  // namespace pm_test
