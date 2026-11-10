#include "pm_voicing/key_loader.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "pm_string/string_modes.h"

namespace pm {

namespace {
constexpr double kPi = std::numbers::pi;

StringParams stringParams(const KeyParams& k) {
    StringParams p;
    p.f0 = k.f0;
    p.B = k.B;
    p.b1 = k.b1;
    p.b3 = k.b3;
    p.length = k.length;
    p.linearDensity = k.linearDensity;
    p.inputPos = k.strikePos;
    p.maxPartials = k.maxPartials;
    p.maxFreqHz = k.maxFreqHz;
    return p;
}

// Mean of sin^2(n pi x) over [a, b].
double meanSin2(int n, double a, double b) {
    const double w = 2.0 * kPi * n;
    return 0.5 - (std::sin(w * b) - std::sin(w * a)) / (2.0 * w * (b - a));
}
}  // namespace

int keyModeCount(const KeyParams& k, double fs) { return countStringModes(stringParams(k), fs); }

KeyModeSet computeKeyModes(const KeyParams& k, double fs) {
    const StringParams p = stringParams(k);
    KeyModeSet s;
    s.count = countStringModes(p, fs);
    std::vector<ModeSpec> specs(static_cast<std::size_t>(s.count));
    computeStringModes(p, fs, specs.data(), s.count);
    s.modalMass = 0.5 * k.linearDensity * k.length;
    const double a = k.damperPos - 0.5 * k.damperWidth, b = k.damperPos + 0.5 * k.damperWidth;
    for (int i = 0; i < s.count; ++i) {
        const int n = i + 1;
        s.freq.push_back(specs[i].freqHz);
        s.sigma.push_back(specs[i].sigma);
        s.phi.push_back(std::sin(n * kPi * k.strikePos));
        s.wIn.push_back(specs[i].wIn);
        const double sign = (n & 1) ? 1.0 : -1.0;
        s.wOut.push_back(k.outputGain * k.tension * n * kPi / k.length * sign);
        const double w = k.damperFloor + (1.0 - k.damperFloor) * 2.0 * meanSin2(n, a, b);
        s.damperSigma.push_back(k.damperSigma * w);
    }
    // sum over omitted partials of phi^2 / (m omega_n^2); terms fall as n^-4
    // once stiffness dominates, so a few thousand terms converge.
    double res = 0.0;
    for (int n = s.count + 1; n <= s.count + 20000; ++n) {
        const double w = 2.0 * kPi * partialFrequency(k.f0, k.B, n);
        const double ph = std::sin(n * kPi * k.strikePos);
        const double term = ph * ph / (s.modalMass * w * w);
        res += term;
        if (term < 1e-12 * res) break;
    }
    s.residualCompliance = res;
    return s;
}

HammerParams hammerParams(const KeyParams& k) {
    HammerParams h;
    h.mass = k.hammerMass;
    h.K = k.feltK;
    h.p = k.feltP;
    h.mu = k.feltMu;
    return h;
}

void loadKey(ModeBank& bank, ContactTables& contact, int s, const KeyParams& k, double fs) {
    const KeyModeSet m = computeKeyModes(k, fs);
    const int m0 = bank.segmentFirstMode(s);
    const int R = k.contactOversampling;
    for (int i = 0; i < m.count; ++i) {
        bank.setMode(m0 + i, m.freq[i], m.sigma[i], fs);
        bank.setWeights(m0 + i, static_cast<float>(m.wIn[i]), static_cast<float>(m.wOut[i]));
        bank.setDamperRate(m0 + i, m.damperSigma[i], fs);
        contact.setMode(m0 + i, m.freq[i], m.sigma[i], m.phi[i], m.modalMass, fs, R);
    }
    // Multiple contacts (plan §1.4): the hammer is followed until it has been
    // off the string for longer than 1.5 round trips of the transverse wave.
    const double release = std::max(1.5 / k.f1, 0.003);
    contact.finishSegment(bank, s, fs, R, m.residualCompliance, release, 0.3);
}

}  // namespace pm
