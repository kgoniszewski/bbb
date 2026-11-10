// Offline FDTD reference (tools/reference_fdtd): scheme sanity, and the
// cross-check of the real-time modal string + hammer against it (plan §5.4:
// partial levels of the first 30 partials within 1 dB).
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <numbers>
#include <vector>

#include "fdtd_string.h"
#include "piano_utils.h"
#include "test_harness.h"

using namespace pm_test;

namespace {

constexpr double kFs = 48000.0;
int kPoints = 400;

pm_fdtd::StringSpec specOf(const pm::KeyParams& k, double v) {
    pm_fdtd::StringSpec s;
    s.length = k.length;
    s.tension = k.tension;
    s.linearDensity = k.linearDensity;
    s.B = k.B;
    s.b1 = k.b1;
    s.b3 = k.b3;
    s.strikePos = k.strikePos;
    s.hammerMass = k.hammerMass;
    s.feltK = k.feltK;
    s.feltP = k.feltP;
    s.feltMu = k.feltMu;
    s.velocity = v;
    return s;
}

// One string of key 'key' with its share of the hammer (the real-time
// model's single-string configuration: rigid bridge, no mating offset, no
// horizontal component).
pm::KeyParams singleString(int key) {
    pm::KeyParams p = tableKey(key);
    p.hammerMass /= p.strings;
    p.strings = 1;
    p.bridgeCoupling = false;
    p.strikeHorizontal = 0.0;
    for (double& o : p.strikeOffset) o = 0.0;
    return p;
}

struct Energies {
    std::vector<double> e;  // per partial, extrapolated to t = 0
    double contactMs = 0.0;
    double toString = 0.0;  // fraction of the hammer's energy
};

Energies fdtd(const pm::KeyParams& p, double v, int points, int partials) {
    const pm_fdtd::StringSpec s = specOf(p, v);
    pm_fdtd::FdtdString str(s, pm_fdtd::stepForPoints(s, points));
    bool wasIn = false;
    Energies r;
    for (long long n = 0; n < 50000000; ++n) {
        str.step();
        const bool in = str.inContact();
        if (wasIn && !in) {
            // Leave the string alone for 1.5 round trips before deciding the
            // hammer is gone (multiple contacts in the bass).
            const double tEnd = str.time();
            bool again = false;
            const double wait = 1.5 / (p.f1);
            while (str.time() < tEnd + wait) {
                str.step();
                if (str.inContact()) {
                    again = true;
                    break;
                }
            }
            if (again) {
                wasIn = true;
                continue;
            }
            const pm_fdtd::ModalState m = str.modes(partials);
            double total = 0.0;
            for (int i = 0; i < static_cast<int>(m.energy.size()); ++i) {
                r.e.push_back(m.energy[i] * std::exp(2.0 * str.modeDecayRate(i + 1) * str.time()));
                total += m.energy[i];
            }
            r.contactMs = 1e3 * tEnd;
            r.toString = str.stringEnergy() / (0.5 * s.hammerMass * v * v);
            (void)total;
            return r;
        }
        wasIn = in;
    }
    return r;
}

Energies realTime(const pm::KeyParams& p, double v, int partials) {
    SingleKey k(p, kFs);
    const StrikeResult s = runStrike(k, v);
    Energies r;
    // State at the hammer's release (after its 1.5 round trips off the string).
    const double t = s.samples / kFs;
    for (int i = 0; i < std::min<int>(partials, static_cast<int>(k.modes.partials.size())); ++i)
        r.e.push_back(k.partialEnergy(i) * std::exp(2.0 * k.modes.sigma[static_cast<std::size_t>(i)] * t));
    r.contactMs = 1e3 * s.contactSpan;
    r.toString = k.stringEnergy() / s.ke0;
    return r;
}

}  // namespace

// Without loss and hammer the scheme's discrete energy is conserved to
// rounding; with loss it never increases.
TEST(scheme_energy) {
    for (bool lossy : {false, true}) {
        pm_fdtd::StringSpec s = specOf(tableKey(40), 1.0);
        if (!lossy) s.b1 = s.b3 = 0.0;
        pm_fdtd::FdtdString str(s, 1.0 / (8 * kFs));
        std::vector<double> y(static_cast<std::size_t>(str.grid().points + 1));
        const int N = str.grid().points;
        for (int i = 0; i <= N; ++i) {
            const double x = double(i) / N;
            y[i] = 1e-3 * (x < 0.12 ? x / 0.12 : (1.0 - x) / 0.88);
        }
        str.setDisplacement(y);
        str.step();
        const double e0 = str.stringEnergy();
        double drift = 0.0, rise = 0.0, last = e0;
        for (int n = 0; n < 40000; ++n) {
            str.step();
            const double e = str.stringEnergy();
            drift = std::max(drift, std::fabs(e / e0 - 1.0));
            rise = std::max(rise, (e - last) / e0);
            last = e;
        }
        std::printf("  C4 string, N = %d, %s, 40000 steps: max |E/E0 - 1| %.2e, max rise per step %.2e, "
                    "final %.4f E0\n",
                    N, lossy ? "lossy" : "lossless", drift, rise, last / e0);
        if (!lossy) CHECK_LE(drift, 1e-9);
        else CHECK_LE(rise, 1e-12);
    }
}

// The explicit hammer is not exactly energy-conserving: with a lossless
// string and an elastic felt, the hammer's kinetic energy after the strike
// plus the string's energy match the initial energy to within 0.5 %.
TEST(hammer_energy_sanity) {
    for (int key : {16, 40, 64}) {
        pm::KeyParams p = singleString(key);
        pm_fdtd::StringSpec s = specOf(p, kMF);
        s.b1 = s.b3 = 0.0;
        s.feltMu = 0.0;
        pm_fdtd::FdtdString str(s, pm_fdtd::stepForPoints(s, 400));
        bool wasIn = false;
        double err = 1.0;
        for (int n = 0; n < 5000000; ++n) {
            str.step();
            if (wasIn && !str.inContact()) {
                const double e0 = 0.5 * s.hammerMass * kMF * kMF;
                err = (str.hammerKineticEnergy() + str.stringEnergy()) / e0 - 1.0;
                break;
            }
            wasIn = str.inContact();
        }
        std::printf("  key %d: (hammer + string energy) / KE0 - 1 after the contact: %+.2e\n", key, err);
        CHECK_LE(std::fabs(err), 5e-3);
    }
}

// The FDTD grid is fine enough: doubling the grid moves partial energies by
// less than 0.2 dB (C4, mf).
TEST(fdtd_grid_converged) {
    const pm::KeyParams p = singleString(40);
    const Energies a = fdtd(p, kMF, 400, 30), b = fdtd(p, kMF, 800, 30);
    double emax = 0.0, worst = 0.0;
    for (double e : b.e) emax = std::max(emax, e);
    for (std::size_t i = 0; i < a.e.size(); ++i)
        if (b.e[i] > 1e-4 * emax) worst = std::max(worst, std::fabs(10.0 * std::log10(a.e[i] / b.e[i])));
    std::printf("  C4 mf, N = 400 vs 800: worst partial difference %.3f dB\n", worst);
    CHECK_LE(worst, 0.2);
}

// Real-time single string + hammer against the FDTD reference: energies of
// the first 30 partials at the end of the strike, extrapolated to the strike
// time with each model's own decay. Target (plan §5.4): 1 dB, checked for
// partials within 30 dB of the strongest from A0 to C7; within 40 dB and
// above C7 the numbers are printed (see README: the top octave receives
// only 1e-4..1e-6 of the hammer's energy in both models, and the dB
// difference there is in the far tail of the force spectrum).
// PM_FDTD_POINTS overrides the FDTD grid (default 400 intervals).
TEST(crosscheck_against_fdtd) {
    if (const char* e = std::getenv("PM_FDTD_POINTS")) kPoints = std::atoi(e);
    struct Case {
        int key;
        bool checked;  // inside the 1 dB target
    };
    const double vel[3] = {kPP, kMF, kFF};
    const char* vn[3] = {"pp", "mf", "ff"};
    for (const Case c : {Case{1, true}, Case{16, true}, Case{28, true}, Case{40, true}, Case{52, true},
                         Case{64, true}, Case{76, true}, Case{82, false}, Case{88, false}}) {
        const pm::KeyParams p = singleString(c.key);
        for (int iv = 0; iv < 3; ++iv) {
            const Energies rt = realTime(p, vel[iv], 30);
            const Energies fd = fdtd(p, vel[iv], kPoints, 30);
            double emax = 0.0, worst = 0.0, worst30 = 0.0, worstLevel = 0.0;
            int worstN = 0, used = 0;
            for (double e : fd.e) emax = std::max(emax, e);
            for (std::size_t i = 0; i < std::min(rt.e.size(), fd.e.size()); ++i) {
                if (fd.e[i] < 1e-4 * emax) continue;
                ++used;
                const double d = 10.0 * std::log10(rt.e[i] / fd.e[i]);
                if (fd.e[i] >= 1e-3 * emax && std::fabs(d) > std::fabs(worst30)) worst30 = d;
                if (std::fabs(d) > std::fabs(worst)) {
                    worst = d;
                    worstN = static_cast<int>(i) + 1;
                    worstLevel = 10.0 * std::log10(fd.e[i] / emax);
                }
            }
            std::printf("  key %2d %s: %2d partials within 40 dB, worst %+.2f dB (n=%d, at %.0f dB), within 30 dB "
                        "%+.2f dB; contact %.2f / %.2f ms; energy to string %.3g %% / %.3g %% (real-time / FDTD)\n",
                        c.key, vn[iv], used, worst, worstN, worstLevel, worst30, rt.contactMs, fd.contactMs,
                        100 * rt.toString, 100 * fd.toString);
            if (c.checked) CHECK_LE(std::fabs(worst30), 1.0);
        }
    }
}
