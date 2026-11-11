// Shared helpers for the DSP tests: rendering a string and measuring the
// frequency and decay rate of one partial from the rendered signal.
#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <numbers>
#include <vector>

#include "pm_base/denormals.h"
#include "pm_string/mode_bank.h"
#include "pm_string/string_modes.h"

namespace pm_test {

inline std::vector<float> render(pm::ModeBank& bank, int nSamples, pm::Kernel kernel,
                                 const std::vector<float>* input = nullptr, int block = 64) {
    pm::ScopedFlushDenormals ftz;
    std::vector<float> out(static_cast<std::size_t>(nSamples), 0.0f);
    for (int pos = 0; pos < nSamples; pos += block) {
        const int n = std::min(block, nSamples - pos);
        bank.process(input ? input->data() + pos : nullptr, out.data() + pos, n, kernel);
    }
    return out;
}

struct PartialFit {
    double freqHz = 0.0;
    double sigma = 0.0;  // fitted decay rate, 1/s
    int frames = 0;      // frames used in the fit
};

// Projects the signal onto exp(-j 2 pi fExpected t) with a 4-term
// Blackman-Harris window (sidelobes < -92 dB, mainlobe half-width 4 bins)
// long enough that neighbouring partials 'spacingHz' away fall outside the
// mainlobe. For a damped sinusoid the projection is exactly
//   C * exp((-sigma + j 2 pi (f - fExpected)) t_frame),
// so a straight-line fit of unwrapped phase and log magnitude against frame
// time gives the frequency offset and the decay rate. Frames whose level is
// more than 'rangeDb' below the strongest frame are ignored.
inline PartialFit measurePartial(const std::vector<float>& x, double fs, double fExpected,
                                 double spacingHz, double rangeDb = 60.0) {
    constexpr double kPi = std::numbers::pi;
    const int nw = static_cast<int>(std::ceil(8.0 * fs / spacingHz));
    const int hop = std::max(1, nw / 4);
    std::vector<double> w(static_cast<std::size_t>(nw));
    for (int i = 0; i < nw; ++i) {
        const double a = 2.0 * kPi * i / (nw - 1);
        w[i] = 0.35875 - 0.48829 * std::cos(a) + 0.14128 * std::cos(2 * a) - 0.01168 * std::cos(3 * a);
    }
    std::vector<double> t, mag, ph;
    const double cyclesPerSample = fExpected / fs;
    for (int s = 0; s + nw <= static_cast<int>(x.size()); s += hop) {
        std::complex<double> acc = 0.0;
        for (int i = 0; i < nw; ++i) {
            const double cyc = std::fmod(cyclesPerSample * (s + i), 1.0);
            acc += w[i] * x[s + i] * std::polar(1.0, -2.0 * kPi * cyc);
        }
        t.push_back(s / fs);
        mag.push_back(std::abs(acc));
        ph.push_back(std::arg(acc));
    }
    PartialFit fit;
    if (t.size() < 3) return fit;
    const double peak = *std::max_element(mag.begin(), mag.end());
    const double floor = peak * std::pow(10.0, -rangeDb / 20.0);

    // Unwrap phase along all frames (adjacent frames differ by far less than
    // pi when the expected frequency is close), then fit on the strong ones.
    for (std::size_t k = 1; k < ph.size(); ++k) {
        double d = ph[k] - ph[k - 1];
        d -= 2.0 * kPi * std::round(d / (2.0 * kPi));
        ph[k] = ph[k - 1] + d;
    }
    double st = 0, sp = 0, sl = 0, stt = 0, stp = 0, stl = 0;
    int n = 0;
    for (std::size_t k = 0; k < t.size(); ++k) {
        if (mag[k] < floor) continue;
        const double l = std::log(mag[k]);
        st += t[k];
        sp += ph[k];
        sl += l;
        stt += t[k] * t[k];
        stp += t[k] * ph[k];
        stl += t[k] * l;
        ++n;
    }
    if (n < 3) return fit;
    const double den = n * stt - st * st;
    const double phaseSlope = (n * stp - st * sp) / den;
    const double logSlope = (n * stl - st * sl) / den;
    fit.freqHz = fExpected + phaseSlope / (2.0 * kPi);
    fit.sigma = -logSlope;
    fit.frames = n;
    return fit;
}

// Energy-weighted mean frequency of the components near fExpected (the
// partial as a tuner hears it, key_loader.h soundingPartial): the same
// windowed projection as measurePartial, frames every nw / 8 samples; each
// frame-to-frame phase increment is weighted by the product of the two
// frames' magnitudes, so a beating cluster gives the energy-weighted mean of
// its components and the phase jumps at beat minima carry no weight.
inline double measureSoundingPitch(const std::vector<float>& x, double fs, double fExpected, double spacingHz) {
    constexpr double kPi = std::numbers::pi;
    const int nw = static_cast<int>(std::ceil(8.0 * fs / spacingHz));
    const int hop = std::max(1, nw / 8);
    std::vector<double> w(static_cast<std::size_t>(nw));
    for (int i = 0; i < nw; ++i) {
        const double a = 2.0 * kPi * i / (nw - 1);
        w[i] = 0.35875 - 0.48829 * std::cos(a) + 0.14128 * std::cos(2 * a) - 0.01168 * std::cos(3 * a);
    }
    const double cyclesPerSample = fExpected / fs;
    std::complex<double> prev = 0.0;
    double num = 0.0, den = 0.0;
    bool first = true;
    for (int s = 0; s + nw <= static_cast<int>(x.size()); s += hop) {
        std::complex<double> acc = 0.0;
        for (int i = 0; i < nw; ++i) {
            const double cyc = std::fmod(cyclesPerSample * (s + i), 1.0);
            acc += w[i] * x[s + i] * std::polar(1.0, -2.0 * kPi * cyc);
        }
        if (!first) {
            const double wt = std::abs(prev) * std::abs(acc);
            num += wt * std::arg(acc * std::conj(prev));
            den += wt;
        }
        prev = acc;
        first = false;
    }
    return den > 0.0 ? fExpected + num / den * fs / (2.0 * kPi * hop) : 0.0;
}

inline double cents(double f, double ref) { return 1200.0 * std::log2(f / ref); }

}  // namespace pm_test
