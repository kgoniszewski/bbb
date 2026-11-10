#include "pm_voicing/key_loader.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "pm_string/string_modes.h"

namespace pm {

namespace {
constexpr double kPi = std::numbers::pi;
using cd = std::complex<double>;

// Mean of sin^2(n pi x) over [a, b].
double meanSin2(int n, double a, double b) {
    const double w = 2.0 * kPi * n;
    return 0.5 - (std::sin(w * b) - std::sin(w * a)) / (2.0 * w * (b - a));
}

double nominalPartial(const KeyParams& k, double f0, int n) { return partialFrequency(f0, k.B, n); }

UnisonPartialSpec partialSpec(const KeyParams& k, double f0, int n, const BridgeAdmittance& bridge) {
    UnisonPartialSpec s;
    const int S = std::clamp(k.strings, 1, kMaxStrings);
    s.strings = S;
    s.freqHz = nominalPartial(k, f0, n);
    for (int i = 0; i < S; ++i) s.detuneRatio[i] = std::pow(2.0, k.detuneCents[i] / 1200.0);
    s.horizontalRatio = std::pow(2.0, k.polarizationSplitCents / 1200.0);
    s.sigma = partialDecayRate(k.b1, k.b3, s.freqHz);
    const double stiff = 1.0 + k.B * n * n;
    s.kappa = k.bridgeCoupling ? k.tension / k.length * stiff : 0.0;
    s.Y = k.bridgeCoupling ? bridge.at(k.key, s.freqHz) : Admittance2{};
    s.phi = std::sin(n * kPi * k.strikePos);
    s.strikeHorizontal = k.strikeHorizontal;
    s.modalMass = 0.5 * k.linearDensity * k.length;
    const double sign = (n & 1) ? 1.0 : -1.0;
    s.outV = k.outputGain * k.tension * n * kPi / k.length * stiff * sign;
    s.outH = std::abs(s.Y.vv) > 0.0 ? s.outV * s.Y.vh / s.Y.vv : cd(0.0);
    return s;
}

// Excitation x output weighted mean frequency of the partial's modes.
double soundingFrequency(const UnisonPartial& u) {
    double num = 0.0, den = 0.0;
    for (int j = 0; j < u.oscillators; ++j) {
        num += u.mode[j].score * u.mode[j].lambda.imag();
        den += u.mode[j].score;
    }
    return den > 0.0 ? num / den / (2.0 * kPi) : u.mode[0].lambda.imag() / (2.0 * kPi);
}

StringParams stringParams(const KeyParams& k) {
    StringParams p;
    p.f0 = k.f0;
    p.B = k.B;
    p.maxPartials = k.maxPartials;
    p.maxFreqHz = k.maxFreqHz;
    return p;
}

}  // namespace

int keyPartialCount(const KeyParams& k, double fs) { return countStringModes(stringParams(k), fs); }

const BridgeAdmittance& defaultBridge() noexcept {
    static const PlaceholderBridgeAdmittance placeholder;
    return placeholder;
}

KeyModeSet computeKeyModes(const KeyParams& k, double fs) {
    static const RigidBridge rigid;
    return computeKeyModes(k, fs, k.bridgeCoupling ? defaultBridge() : static_cast<const BridgeAdmittance&>(rigid));
}

KeyModeSet computeKeyModes(const KeyParams& k, double fs, const BridgeAdmittance& bridge) {
    KeyModeSet s;
    const int S = std::clamp(k.strings, 1, kMaxStrings);
    s.strings = S;
    s.modalMass = 0.5 * k.linearDensity * k.length;
    const int nPartials = keyPartialCount(k, fs);

    // Pitch compensation of partial 1 (two fixed-point steps suffice: the
    // shift is a fraction of a cent).
    double f0 = k.f0;
    if (k.bridgeCoupling && nPartials > 0) {
        for (int it = 0; it < 3; ++it) {
            const UnisonPartial u = solveUnisonPartial(partialSpec(k, f0, 1, bridge));
            f0 *= k.f1 / soundingFrequency(u);
        }
    }
    s.pitchRatio = f0 / k.f0;

    const double a = k.damperPos - 0.5 * k.damperWidth, b = k.damperPos + 0.5 * k.damperWidth;
    s.partials.reserve(static_cast<std::size_t>(nPartials));
    for (int n = 1; n <= nPartials; ++n) {
        KeyPartial kp;
        kp.n = n;
        kp.firstMode = s.count;
        kp.unison = solveUnisonPartial(partialSpec(k, f0, n, bridge));
        const UnisonPartial& u = kp.unison;
        if (u.status == UnisonStatus::Perturbed) ++s.perturbedPartials;
        if (u.status == UnisonStatus::Uncoupled) ++s.uncoupledPartials;

        // Mode selection (plan §1.3): every excitable mode of the low
        // partials; above them the strongest by excitation x output.
        int order[kMaxOscillators];
        double maxExc = 0.0;
        for (int j = 0; j < u.oscillators; ++j) {
            order[j] = j;
            double e = 0.0;
            for (int st = 0; st < S; ++st) e += std::abs(u.mode[j].input[st]);
            maxExc = std::max(maxExc, e);
        }
        std::stable_sort(order, order + u.oscillators,
                         [&](int x, int y) { return u.mode[x].score > u.mode[y].score; });
        const int limit = n <= k.fullCoupledPartials
                              ? u.oscillators
                              : std::clamp(k.modesPerHighPartial, 1, u.oscillators);
        int kept = 0;
        for (int r = 0; r < limit; ++r) {
            const int j = order[r];
            double e = 0.0;
            for (int st = 0; st < S; ++st) e += std::abs(u.mode[j].input[st]);
            if (e <= 1e-9 * maxExc) continue;  // never excited by the hammer
            kp.keptIndex[kept++] = j;
        }
        // Keep the partial's modes in a fixed (eigen-solver) order.
        std::sort(kp.keptIndex, kp.keptIndex + kept);
        kp.kept = kept;

        const double w = k.damperFloor + (1.0 - k.damperFloor) * 2.0 * meanSin2(n, a, b);
        for (int r = 0; r < kept; ++r) {
            const UnisonMode& m = u.mode[kp.keptIndex[r]];
            s.partial.push_back(n);
            s.freq.push_back(m.lambda.imag() / (2.0 * kPi));
            s.sigma.push_back(-m.lambda.real());
            s.wOut.push_back(m.wOut);
            std::array<cd, kMaxStrings> pj{}, in{};
            for (int st = 0; st < S; ++st) {
                pj[st] = m.proj[st];
                in[st] = m.input[st];
            }
            s.proj.push_back(pj);
            s.input.push_back(in);
            s.damperSigma.push_back(k.damperSigma * w);
        }
        s.count += kept;
        s.partials.push_back(kp);
    }

    // Static compliance at the strike point of the partials not simulated:
    // sum of phi^2 / (m omega_n^2) per string; terms fall as n^-4 once
    // stiffness dominates, so a few thousand terms converge.
    double res = 0.0;
    for (int n = nPartials + 1; n <= nPartials + 20000; ++n) {
        const double wn = 2.0 * kPi * partialFrequency(f0, k.B, n);
        const double ph = std::sin(n * kPi * k.strikePos);
        const double term = ph * ph / (s.modalMass * wn * wn);
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
    h.strings = std::clamp(k.strings, 1, kMaxStrings);
    for (int s = 0; s < kMaxStrings; ++s) h.offset[s] = s < h.strings ? k.strikeOffset[s] : 0.0;
    return h;
}

HammerParams unaCordaHammerParams(const KeyParams& k, double u) {
    HammerParams h = hammerParams(k);
    u = std::clamp(u, 0.0, 1.0);
    if (k.unaCordaString >= 0 && k.unaCordaString < h.strings)
        h.offset[k.unaCordaString] += u * u * k.unaCordaMiss;
    h.K *= 1.0 - u * std::clamp(k.unaCordaSoften, 0.0, 0.9);
    return h;
}

void loadKeyModes(ModeBank& bank, ContactTables& contact, int seg, const KeyParams& k,
                  const KeyModeSet& m, double fs) {
    const int m0 = bank.segmentFirstMode(seg);
    const int R = k.contactOversampling;
    for (int i = 0; i < m.count; ++i) {
        cd wIn = 0.0;
        for (int st = 0; st < m.strings; ++st) wIn += m.input[i][st];
        wIn /= fs;
        bank.setMode(m0 + i, m.freq[i], m.sigma[i], fs);
        bank.setWeights(m0 + i, static_cast<float>(wIn.real()), static_cast<float>(wIn.imag()),
                        static_cast<float>(m.wOut[i]));
        bank.setDamperRate(m0 + i, m.damperSigma[i], fs);
        contact.setMode(m0 + i, cd(-m.sigma[i], 2.0 * kPi * m.freq[i]), m.strings, m.proj[i].data(),
                        m.input[i].data(), fs, R);
    }
    // Multiple contacts (plan §1.4): the hammer is followed until it has been
    // off the strings for longer than 1.5 round trips of the transverse wave.
    const double release = std::max(1.5 / k.f1, 0.003);
    contact.finishSegment(bank, seg, m.strings, fs, R, m.residualCompliance, release, 0.3);
}

void loadKey(ModeBank& bank, ContactTables& contact, int s, const KeyParams& k, double fs) {
    loadKeyModes(bank, contact, s, k, computeKeyModes(k, fs), fs);
}

double keyStringEnergy(const KeyModeSet& m, const float* zr, const float* zi) {
    double e = 0.0;
    for (const KeyPartial& p : m.partials) {
        const UnisonPartial& u = p.unison;
        const int n = u.oscillators;
        for (int k = 0; k < n; ++k) {
            cd z = 0.0;
            for (int r = 0; r < p.kept; ++r) {
                const int i = p.firstMode + r;
                z += u.V[k * n + p.keptIndex[r]] * cd(zr[i], zi[i]);
            }
            e += modalEnergy(z.real(), z.imag(), u.oscFreqHz[k], u.oscSigma[k], m.modalMass);
        }
    }
    return e;
}

}  // namespace pm
