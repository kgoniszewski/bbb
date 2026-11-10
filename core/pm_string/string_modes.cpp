#include "pm_string/string_modes.h"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace pm {

namespace {
constexpr double kPi = std::numbers::pi;

double freqLimit(const StringParams& p, double fs) noexcept {
    return std::min(0.45 * fs, p.maxFreqHz);
}

ModeSpec modeSpec(const StringParams& p, double fs, int n) noexcept {
    const double f = partialFrequency(p.f0, p.B, n);
    const double omega = 2.0 * kPi * f;
    const double modalMass = 0.5 * p.linearDensity * p.length;
    ModeSpec m;
    m.freqHz = f;
    m.sigma = partialDecayRate(p.b1, p.b3, f);
    // A force sample F acting for 1/fs gives the mode a velocity step
    // phi*F/(m fs), i.e. a step of phi*F/(m omega fs) in Re(z).
    m.wIn = std::sin(n * kPi * p.inputPos) / (modalMass * omega * fs);
    m.wOut = p.outputGain * std::sin(n * kPi * p.outputPos);
    return m;
}
}  // namespace

double partialFrequency(double f0, double B, int n) noexcept {
    const double nn = static_cast<double>(n);
    return nn * f0 * std::sqrt(1.0 + B * nn * nn);
}

double partialDecayRate(double b1, double b3, double freqHz) noexcept {
    const double w = 2.0 * kPi * freqHz;
    return b1 + b3 * w * w;
}

int countStringModes(const StringParams& p, double fs) noexcept {
    const double limit = freqLimit(p, fs);
    int n = 0;
    while (n < p.maxPartials && partialFrequency(p.f0, p.B, n + 1) < limit) ++n;
    return n;
}

int computeStringModes(const StringParams& p, double fs, ModeSpec* out, int maxModes) noexcept {
    const int count = std::min(countStringModes(p, fs), maxModes);
    for (int i = 0; i < count; ++i) out[i] = modeSpec(p, fs, i + 1);
    return count;
}

int loadString(ModeBank& bank, const StringParams& p, double fs) noexcept {
    const int count = countStringModes(p, fs);
    if (count > bank.capacity()) return 0;
    bank.resize(count);
    for (int i = 0; i < count; ++i) {
        const ModeSpec m = modeSpec(p, fs, i + 1);
        bank.setMode(i, m.freqHz, m.sigma, fs);
        bank.setWeights(i, static_cast<float>(m.wIn), static_cast<float>(m.wOut));
    }
    bank.clearState();
    return count;
}

void retuneString(ModeBank& bank, const StringParams& p, double fs) noexcept {
    const double limit = freqLimit(p, fs);
    constexpr double kFadeSigma = 1000.0;  // T60 ~ 7 ms
    for (int i = 0; i < bank.size(); ++i) {
        const double f = partialFrequency(p.f0, p.B, i + 1);
        if (f < limit)
            bank.setMode(i, f, partialDecayRate(p.b1, p.b3, f), fs);
        else
            bank.setMode(i, std::min(f, 0.49 * fs), kFadeSigma, fs);
    }
}

void pluck(ModeBank& bank, double xPluck, double heightM) noexcept {
    xPluck = std::clamp(xPluck, 1e-6, 1.0 - 1e-6);
    const double k = 2.0 * heightM / (kPi * kPi * xPluck * (1.0 - xPluck));
    for (int i = 0; i < bank.size(); ++i) {
        const double n = static_cast<double>(i + 1);
        bank.setState(i, 0.0f, static_cast<float>(k * std::sin(n * kPi * xPluck) / (n * n)));
    }
}

}  // namespace pm
