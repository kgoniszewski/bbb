#include "pm_voicing/key_loader.h"

#include <algorithm>
#include <cmath>
#include <numbers>

#include "pm_string/string_modes.h"
#include "pm_voicing/complex_eigen.h"
#include "pm_voicing/soundboard_model.h"
#include "pm_voicing/tuning.h"

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
    s.outH = k.bridgeCoupling ? s.outV * bridge.crossRatio(k.key, s.freqHz) : cd(0.0);
    return s;
}

// The admittance the strings see in total (bake plus run-time loop).
class TotalAdmittance final : public BridgeAdmittance {
public:
    explicit TotalAdmittance(const BridgeAdmittance& b) : b_(b) {}
    Admittance2 at(int key, double f) const noexcept override { return b_.total(key, f); }
    Admittance2 atLambda(int key, std::complex<double> l) const noexcept override { return b_.totalLambda(key, l); }
    std::complex<double> crossRatio(int key, double f) const noexcept override { return b_.crossRatio(key, f); }

private:
    const BridgeAdmittance& b_;
};

// Complex frequency at which a partial's bake is evaluated: its dominant
// normal mode (by excitation x output) against the total admittance (one
// fixed-point step), not the uncoupled real frequency: where the board's
// admittance varies on the scale of the mode's frequency shift or decay
// rate, the two differ by several % in Re Y, and a run-time loop acts at
// the mode's actual (complex) frequency.
cd bakeLambda(const KeyParams& k, double f0, int n, const BridgeAdmittance& total) {
    const UnisonPartial pre = solveUnisonPartial(partialSpec(k, f0, n, total));
    int best = 0;
    for (int j = 1; j < pre.oscillators; ++j)
        if (pre.mode[j].score > pre.mode[best].score) best = j;
    return pre.mode[best].lambda;
}

UnisonPartial solveCoupled(const KeyParams& k, double f0, int n, const BridgeAdmittance& bridge,
                           const BridgeAdmittance& total) {
    UnisonPartialSpec spec = partialSpec(k, f0, n, bridge);
    if (k.bridgeCoupling) spec.Y = bridge.atLambda(k.key, bakeLambda(k, f0, n, total));
    return solveUnisonPartial(spec);
}

// Weight of a mode in the sounding pitch: its output energy over the tuning
// window, amplitude^2 * INT exp(-2 sigma t) dt over [kTuningStartS,
// kTuningStartS + kTuningWindowS].
double tuningWeight(double score, cd lambda) {
    const double s2 = -2.0 * lambda.real();
    const double e = std::fabs(s2) < 1e-9 ? kTuningWindowS
                                          : std::exp(-s2 * kTuningStartS) * -std::expm1(-s2 * kTuningWindowS) / s2;
    return score * score * e;
}

// Energy-weighted mean frequency of the partial's modes. A hammer drives
// the strings of a unison nearly in phase, so a mode's amplitude is its
// output weight times the coherent sum of its string inputs.
double soundingFrequency(const UnisonPartial& u, int strings) {
    double num = 0.0, den = 0.0;
    for (int j = 0; j < u.oscillators; ++j) {
        cd in = 0.0;
        for (int st = 0; st < strings; ++st) in += u.mode[j].input[st];
        const double w = tuningWeight(u.mode[j].wOut * std::abs(in), u.mode[j].lambda);
        num += w * u.mode[j].lambda.imag();
        den += w;
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

SoundingPartial soundingPartial(const KeyParams& k, double f0, int n, const BridgeAdmittance& bridge) {
    SoundingPartial r;
    const TotalAdmittance total(bridge);
    const UnisonPartial u = k.bridgeCoupling ? solveCoupled(k, f0, n, bridge, total)
                                             : solveUnisonPartial(partialSpec(k, f0, n, bridge));
    r.freqHz = soundingFrequency(u, std::clamp(k.strings, 1, kMaxStrings));
    {
        int lead = 0;
        for (int j = 1; j < u.oscillators; ++j)
            if (u.mode[j].score > u.mode[lead].score) lead = j;
        r.sigma = -u.mode[lead].lambda.real();
    }
    if (!k.bridgeCoupling) return r;
    UnisonPartialSpec spec = partialSpec(k, f0, n, bridge);
    const int S = spec.strings, no = 2 * S;
    // Not in a run-time loop: the bake is all the strings see, and u is exact.
    if (!(spec.freqHz < bridge.loopCutoffHz())) return r;
    cd pole[kMaxEigenN], res[kMaxEigenN];
    const int M = bridge.loopPoles(k.key, spec.freqHz, pole, res, std::min(kMaxLoopPoles, kMaxEigenN - no));

    // The partial's oscillators and the nearby board modes of the loop as one
    // linear system: board mode m is a state b_m with
    //   db_m/dt = pole_m b_m + c_m e^T z,   dz/dt = A_rest z - kappa e sum_m c_m b_m,
    // c_m^2 = residue_m, e = (1 vertical, r horizontal), which reproduces
    // -kappa e e^T sum residue / (s - pole) in the oscillators' equations.
    // A_rest carries the rest of the run-time admittance, evaluated at the
    // root being tracked (refined by fixed-point steps).
    const cd lb = bakeLambda(k, f0, n, total);
    const cd rc = bridge.crossRatio(k.key, spec.freqHz);
    const double beta = spec.phi / (spec.modalMass * 2.0 * kPi * spec.freqHz);
    int best = 0;
    for (int j = 1; j < u.oscillators; ++j)
        if (u.mode[j].score > u.mode[best].score) best = j;
    cd le = u.mode[best].lambda;
    const int N = no + M;
    Admittance2 baked = bridge.atLambda(k.key, lb);
    const Admittance2 load = bridge.loopLoading(k.key, spec.freqHz);
    baked = {baked.vv + load.vv, baked.vh + load.vh, baked.hh + load.hh};
    for (int it = 0; it < 8; ++it) {
        Admittance2 yrt = bridge.loopLambda(k.key, le);
        yrt = {baked.vv + yrt.vv, baked.vh + yrt.vh, baked.hh + yrt.hh};
        cd ex = 0.0;
        for (int m = 0; m < M; ++m) ex += res[m] / (le - pole[m]);
        spec.Y = {yrt.vv - ex, yrt.vh - rc * ex, yrt.hh - rc * rc * ex};
        cd A[kMaxEigenN * kMaxEigenN] = {}, Ar[kMaxOscillators * kMaxOscillators];
        unisonMatrix(spec, Ar);
        for (int a = 0; a < no; ++a)
            for (int b = 0; b < no; ++b) A[a * N + b] = Ar[a * no + b];
        for (int m = 0; m < M; ++m) {
            const cd c = std::sqrt(res[m]);
            for (int a = 0; a < no; ++a) {
                const cd e = a < S ? cd(1.0) : rc;
                A[a * N + no + m] = -spec.kappa * e * c;
                A[(no + m) * N + a] = c * e;
            }
            A[(no + m) * N + no + m] = pole[m];
        }
        cd lam[kMaxEigenN], V[kMaxEigenN * kMaxEigenN], Vi[kMaxEigenN * kMaxEigenN];
        if (!eigenDecompose(N, A, lam, V, Vi).ok) return r;
        // Participation of the strings in each mode (sums to 2S over the
        // modes); the sounding partial is the energy-weighted mean of the
        // string-dominated modes, the pitch a tuner settles on once the
        // board-dominated modes (board decay) have died away.
        double part[kMaxEigenN], score[kMaxEigenN];
        int top = 0;
        for (int j = 0; j < N; ++j) {
            cd p = 0.0, o = 0.0;
            for (int a = 0; a < no; ++a) {
                p += Vi[j * N + a] * V[a * N + j];
                o += (a < S ? spec.outV : spec.outH) * V[a * N + j];
            }
            cd in = 0.0;
            for (int st = 0; st < S; ++st) in += beta * (Vi[j * N + st] + spec.strikeHorizontal * Vi[j * N + S + st]);
            part[j] = p.real();
            score[j] = std::abs(o) * std::abs(in);
            if (part[j] > part[top]) top = j;
        }
        double num = 0.0, den = 0.0, sb = -1.0, other = 0.0, otherF = 0.0;
        int lead = top;
        for (int j = 0; j < N; ++j) {
            if (part[j] > 0.5 || j == top) {
                const double w = tuningWeight(score[j], lam[j]);
                num += w * lam[j].imag();
                den += w;
                if (score[j] > sb) {
                    sb = score[j];
                    lead = j;
                }
            } else if (part[j] > other) {
                other = part[j];
                otherF = lam[j].imag() / (2.0 * kPi);
            }
        }
        r.freqHz = den > 0.0 ? num / den / (2.0 * kPi) : lam[top].imag() / (2.0 * kPi);
        r.participation = part[lead];
        r.sigma = -lam[lead].real();
        r.boardFreqHz = otherF;
        r.boardParticipation = other;
        r.hybrid = M > 0;
        const cd next = lam[lead];
        const bool done = std::abs(next - le) < 1e-8 * std::abs(le);
        le = next;
        if (done) break;
    }
    return r;
}

double compensatedF0(const KeyParams& k, const BridgeAdmittance& bridge) {
    double f0 = k.f0;
    if (!k.bridgeCoupling) return f0;
    // Secant steps in log f0 (the first a plain ratio step).
    double x0 = 0.0, e0 = 0.0;
    for (int it = 0; it < 16; ++it) {
        const double f = soundingPartial(k, f0, 1, bridge).freqHz;
        if (!(f > 0.0)) break;
        const double x = std::log(f0), e = std::log(k.f1 / f);
        if (std::fabs(e) < 6e-8) break;
        double step = e;
        if (it > 0 && std::fabs(e - e0) > 1e-12)
            step = std::clamp(-e * (x - x0) / (e - e0), -4.0 * std::fabs(e), 4.0 * std::fabs(e));
        x0 = x;
        e0 = e;
        f0 = std::exp(x + step);
    }
    return f0;
}

void tuneToSoundingPartials(PianoTable& t, const BridgeAdmittance& bridge) {
    if (t.tuning.equalTemperament) return;
    // Only the partials the tuner compares (tuning.h): 4:2 at A3-A4; above
    // A4 partials 1, 2 of key k, 2, 4 of k - 12 and 4 of k - 24; below A3
    // partials 4, 6, 10 of key k and 2, 3, 5 of k + 12.
    bool need[kNumKeys + 1][kTunerPartials + 1] = {};
    need[37][4] = need[49][2] = true;
    for (int k = 50; k <= kNumKeys; ++k) {
        const OctaveWeights w = octaveWeights(k);
        if (w.w42 > 0.0) need[k][2] = need[k - 12][4] = true;
        if (w.w21 > 0.0) need[k - 12][2] = true;
        if (w.w41 > 0.0) need[k - 24][4] = true;
    }
    for (int k = 1; k <= 36; ++k) {
        const OctaveWeights w = octaveWeights(k);
        if (w.w42 > 0.0) need[k][4] = need[k + 12][2] = true;
        if (w.w63 > 0.0) need[k][6] = need[k + 12][3] = true;
        if (w.w105 > 0.0) need[k][10] = need[k + 12][5] = true;
    }
    // The ratios are taken with each string at its compensated pitch for the
    // current target, as the engine will bake it; a second pass takes them
    // again at the new targets (a partial near a board mode shifts with its
    // distance from it).
    // Applied as a correction of the table's own targets (which may carry
    // per-key edits): f1 *= (tuner on sounding ratios) / (tuner on r_k(n)).
    double B[kNumKeys], f1[kNumKeys], plain[kNumKeys], base[kNumKeys];
    for (int i = 0; i < kNumKeys; ++i) {
        B[i] = t.keys[i].B;
        base[i] = t.keys[i].f1;
    }
    virtualTuner(B, t.tuning, plain);
    for (int pass = 0; pass < 2; ++pass) {
        PartialRatios ratios;
        for (int i = 0; i < kNumKeys; ++i) {
            const KeyParams& k = t.keys[i];
            if (!k.bridgeCoupling) continue;
            double f0 = 0.0;
            for (int n = 2; n <= kTunerPartials; ++n) {
                if (!need[i + 1][n] || partialFrequency(k.f0, k.B, n) >= k.maxFreqHz) continue;
                if (f0 == 0.0) f0 = compensatedF0(k, bridge);
                ratios.r[i][n] = soundingPartial(k, f0, n, bridge).freqHz / k.f1;
            }
        }
        virtualTuner(B, t.tuning, f1, &ratios);
        for (int i = 0; i < kNumKeys; ++i) {
            KeyParams& k = t.keys[i];
            k.f1 = base[i] * f1[i] / plain[i];
            k.f0 = k.f1 / std::sqrt(1.0 + k.B);
            const double c = 2.0 * k.length * k.f0;
            k.linearDensity = k.tension / (c * c);
        }
    }
}

const BridgeAdmittance& defaultBridge() noexcept {
    static const SoundboardBridge board(defaultBoard(), 48000.0, 0.0);
    return board;
}

int keyCoupledGroups(const KeyModeSet& m) noexcept {
    return (m.coupledModes + ModeBank::kGroupSize - 1) / ModeBank::kGroupSize;
}
int keyBankModes(const KeyModeSet& m) noexcept {
    return keyCoupledGroups(m) * ModeBank::kGroupSize + (m.count - m.coupledModes);
}
int bankIndex(const KeyModeSet& m, int i) noexcept {
    return i < m.coupledModes ? i : keyCoupledGroups(m) * ModeBank::kGroupSize + (i - m.coupledModes);
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

    // Pitch compensation of partial 1: the sounding partial (soundingPartial,
    // which resolves a partial hybridised with a board mode of the run-time
    // loop) is brought to k.f1 to 1e-4 cent. Near a board resonance the
    // sounding frequency moves less than f0 (string-board repulsion), hence
    // secant steps.
    const double f0 = nPartials > 0 ? compensatedF0(k, bridge) : k.f0;
    s.pitchRatio = f0 / k.f0;

    const double a = k.damperPos - 0.5 * k.damperWidth, b = k.damperPos + 0.5 * k.damperWidth;
    const double cutoff = k.bridgeCoupling ? bridge.loopCutoffHz() : 0.0;
    const double gain = k.outputGain != 0.0 ? k.outputGain : 1.0;
    s.partials.reserve(static_cast<std::size_t>(nPartials));
    for (int n = 1; n <= nPartials; ++n) {
        KeyPartial kp;
        kp.n = n;
        kp.firstMode = s.count;
        kp.unison = solveCoupled(k, f0, n, bridge, TotalAdmittance(bridge));
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
        const UnisonPartialSpec spec = partialSpec(k, f0, n, bridge);
        const int no = u.oscillators;
        // Partials rise with n, so the coupled ones are the leading ones.
        const bool coupled = spec.freqHz < cutoff && s.coupledModes == s.count;
        const double phiD = std::fabs(std::sin(n * kPi * k.damperPos));
        for (int r = 0; r < kept; ++r) {
            const int j = kp.keptIndex[r];
            const UnisonMode& m = u.mode[j];
            cd sIn = 0.0;
            double vmax = 0.0;
            for (int q = 0; q < no; ++q) {
                const cd o = q < S ? spec.outV : spec.outH;
                sIn += u.Vinv[j * no + q] * (o / gain) / (s.modalMass * 2.0 * kPi * spec.freqHz);
                if (q < S) vmax = std::max(vmax, std::abs(u.V[q * no + j]));
            }
            s.sympIn.push_back(sIn);
            s.damperShape.push_back(phiD * vmax);
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
        if (coupled) s.coupledModes = s.count;
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
        const int b = m0 + bankIndex(m, i);
        cd wIn = 0.0;
        if (i < m.coupledModes) {
            const cd lambda(-m.sigma[i], 2.0 * kPi * m.freq[i]);
            // (c - 1) without cancellation, as in ModeBank::setMode.
            const double th = lambda.imag() / fs, rm1 = std::expm1(lambda.real() / fs), h = std::sin(0.5 * th);
            const cd cm1(rm1 * std::cos(th) - 2.0 * h * h, (1.0 + rm1) * std::sin(th));
            wIn = m.sympIn[i] * cm1 / lambda;
        }
        bank.setMode(b, m.freq[i], m.sigma[i], fs);
        bank.setWeights(b, static_cast<float>(wIn.real()), static_cast<float>(wIn.imag()),
                        static_cast<float>(m.wOut[i]));
        bank.setDamperRate(b, m.damperSigma[i], fs);
        contact.setMode(b, cd(-m.sigma[i], 2.0 * kPi * m.freq[i]), m.strings, m.proj[i].data(),
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
                const int i = bankIndex(m, p.firstMode + r);
                z += u.V[k * n + p.keptIndex[r]] * cd(zr[i], zi[i]);
            }
            e += modalEnergy(z.real(), z.imag(), u.oscFreqHz[k], u.oscSigma[k], m.modalMass);
        }
    }
    return e;
}

}  // namespace pm
