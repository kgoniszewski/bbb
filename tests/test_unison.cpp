// Coupled unison normal modes (plan §1.3) against a direct time-domain
// simulation, and limiting cases of the solver.
#include <cmath>
#include <complex>
#include <cstdio>
#include <numbers>
#include <vector>

#include "pm_voicing/key_loader.h"
#include "pm_voicing/unison.h"
#include "test_harness.h"

using cd = std::complex<double>;
constexpr double kPi = std::numbers::pi;

namespace {

// A real mechanical bridge: per polarization a mass, a dashpot and a
// spring, with a dashpot between the polarizations. Impedance
// Z(w) = R + j w M + K / (j w); Y = Z^-1 (2x2).
struct MechBridge final : pm::BridgeAdmittance {
    double M[2] = {0.05, 0.5}, K[2] = {2e6, 2e7}, R[2] = {1500.0, 15000.0}, Rvh = 900.0;
    pm::Admittance2 at(int, double f) const noexcept override {
        const double w = 2.0 * kPi * f;
        const cd zvv(R[0], w * M[0] - K[0] / w), zhh(R[1], w * M[1] - K[1] / w), zvh(Rvh, 0.0);
        const cd det = zvv * zhh - zvh * zvh;
        return {zhh / det, -zvh / det, zvv / det};
    }
};

struct Sim {
    // Strings: m (q'' + 2 sigma q' + w_k^2 q) = g u_p(k); bridge:
    // M u'' + R u' + K u + Rvh (u_other)' = F_p = g sum_{k in p} q_k.
    int S;
    double m, g, sigma;
    double w[6];
    MechBridge br;
    std::vector<double> x;  // q[2S], qd[2S], u[2], ud[2]

    std::vector<double> deriv(const std::vector<double>& s) const {
        const int n = 2 * S;
        std::vector<double> d(s.size());
        const double* q = s.data();
        const double* qd = q + n;
        const double* u = qd + n;
        const double* ud = u + 2;
        double F[2] = {0.0, 0.0};
        for (int k = 0; k < n; ++k) F[k >= S] += g * q[k];
        for (int k = 0; k < n; ++k) {
            d[k] = qd[k];
            d[n + k] = -2.0 * sigma * qd[k] - w[k] * w[k] * q[k] + g * u[k >= S] / m;
        }
        for (int p = 0; p < 2; ++p) {
            d[2 * n + p] = ud[p];
            d[2 * n + 2 + p] = (F[p] - br.R[p] * ud[p] - br.K[p] * u[p] - br.Rvh * ud[1 - p]) / br.M[p];
        }
        return d;
    }
    void rk4(double h) {
        auto add = [](const std::vector<double>& a, const std::vector<double>& b, double c) {
            std::vector<double> r(a.size());
            for (std::size_t i = 0; i < a.size(); ++i) r[i] = a[i] + c * b[i];
            return r;
        };
        const auto k1 = deriv(x);
        const auto k2 = deriv(add(x, k1, 0.5 * h));
        const auto k3 = deriv(add(x, k2, 0.5 * h));
        const auto k4 = deriv(add(x, k3, h));
        for (std::size_t i = 0; i < x.size(); ++i) x[i] += h / 6.0 * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]);
    }
    double outV() const {
        double f = 0.0;
        for (int s = 0; s < S; ++s) f += g * x[s];
        return f;
    }
};

}  // namespace

// The normal modes (eigen-solve of the first-order coupled system with Y at
// the partial frequency) reproduce a second-order time-domain simulation of
// the strings on a real mass-spring-dashpot bridge, integrated by RK4 with a
// 2 us step: one string struck (initial velocity), 4 s.
TEST(normal_modes_match_time_domain) {
    for (int S : {1, 2, 3}) {
        Sim sim;
        sim.S = S;
        sim.m = 0.002;
        const double f = 261.6, wn = 2.0 * kPi * f;
        const double kappa = 1081.0;  // C4: T / L
        sim.g = std::sqrt(2.0 * sim.m * wn * wn * kappa);
        sim.sigma = 0.35;
        const double cents[3] = {-0.9, 0.3, 0.6};
        pm::UnisonPartialSpec spec;
        spec.strings = S;
        spec.freqHz = f;
        spec.horizontalRatio = std::pow(2.0, 0.2 / 1200.0);
        for (int s = 0; s < S; ++s) {
            spec.detuneRatio[s] = std::pow(2.0, cents[s] / 1200.0);
            sim.w[s] = wn * spec.detuneRatio[s];
            sim.w[S + s] = sim.w[s] * spec.horizontalRatio;
        }
        spec.sigma = sim.sigma;
        spec.kappa = kappa;
        spec.Y = sim.br.at(40, f);
        spec.phi = 1.0;
        spec.modalMass = sim.m;
        spec.outV = sim.g;
        spec.outH = 0.0;
        const pm::UnisonPartial u = pm::solveUnisonPartial(spec);

        // String 0 vertical struck: q = 0, q' = v0, i.e. z = v0 / omega.
        const double v0 = 1e-3;
        sim.x.assign(static_cast<std::size_t>(4 * S + 4), 0.0);
        sim.x[2 * S] = v0;
        const int n = 2 * S;
        cd w0[6];
        for (int j = 0; j < n; ++j) w0[j] = u.Vinv[j * n + 0] * (v0 / wn);

        const double h = 2e-6, T = 4.0;
        const int steps = static_cast<int>(T / h), every = 25;  // 20 kHz output
        std::vector<double> yd, ym;
        for (int i = 0; i <= steps; ++i) {
            if (i % every == 0) {
                const double t = i * h;
                double y = 0.0;
                for (int j = 0; j < n; ++j) y += u.mode[j].wOut * std::imag(std::exp(u.mode[j].lambda * t) * w0[j]);
                yd.push_back(sim.outV());
                ym.push_back(y);
            }
            sim.rk4(h);
        }
        // Waveform error, and envelope (RMS over 50 ms) error in dB while the
        // level is within 40 dB of its start.
        double peak = 0.0, err = 0.0, envWorst = 0.0;
        for (std::size_t i = 0; i < yd.size(); ++i) {
            peak = std::max(peak, std::fabs(yd[i]));
            err = std::max(err, std::fabs(yd[i] - ym[i]));
        }
        const int win = 1000;
        double ref = -1.0;
        for (std::size_t s0 = 0; s0 + win <= yd.size(); s0 += win) {
            double a = 0.0, b = 0.0;
            for (int i = 0; i < win; ++i) {
                a += yd[s0 + i] * yd[s0 + i];
                b += ym[s0 + i] * ym[s0 + i];
            }
            if (ref < 0.0) ref = a;
            if (a < ref * 1e-4) break;
            envWorst = std::max(envWorst, std::fabs(10.0 * std::log10(a / b)));
        }
        std::printf("  S=%d: decay rates", S);
        for (int j = 0; j < n; ++j) std::printf(" %.2f", -u.mode[j].lambda.real());
        std::printf(" 1/s; max waveform error %.2e of peak, max envelope error %.3f dB\n", err / peak, envWorst);
        CHECK_LE(err / peak, 0.01);
        CHECK_LE(envWorst, 0.05);
    }
}

namespace {
pm::UnisonPartialSpec c4Spec(double width, double strikeHorizontal = 0.0) {
    pm::UnisonPartialSpec s;
    s.strings = 3;
    s.freqHz = 261.6;
    const double c[3] = {-0.5, 0.2, 0.3};
    for (int i = 0; i < 3; ++i) s.detuneRatio[i] = std::pow(2.0, width * c[i] / 1200.0);
    s.sigma = 0.35;
    s.kappa = 1081.0;
    // The smooth placeholder admittance: these are tests of the solver.
    s.Y = pm::PlaceholderBridgeAdmittance().at(40, 261.6);
    s.phi = 1.0;
    s.modalMass = 0.002;
    s.outV = 1.0;
    s.strikeHorizontal = strikeHorizontal;
    return s;
}
}  // namespace

// Identical strings in perfect unison: one vertical mode carries the whole
// bridge coupling (decay sigma + 3 kappa Re Yvv: the prompt sound), the two
// antisymmetric vertical modes do not move the bridge at all (decay sigma
// only, zero output), and a symmetric strike excites only the symmetric
// modes. Without coupling the modes are the strings themselves.
TEST(identical_strings_limits) {
    const pm::UnisonPartialSpec s = c4Spec(0.0);
    const pm::UnisonPartial u = pm::solveUnisonPartial(s);
    const double prompt = s.sigma + 3.0 * s.kappa * s.Y.vv.real();
    int sym = 0, anti = 0;
    // The antisymmetric modes are degenerate (horizontalRatio is 1 here), so
    // the solver may return any basis of that space: compare against the
    // largest input of any mode, not the mode's own.
    double maxIn = 0.0;
    for (int j = 0; j < u.oscillators; ++j) maxIn = std::max(maxIn, std::abs(u.mode[j].input[0]));
    for (int j = 0; j < u.oscillators; ++j) {
        const pm::UnisonMode& m = u.mode[j];
        const double sig = -m.lambda.real();
        const cd symExc = m.input[0] + m.input[1] + m.input[2];
        if (std::fabs(sig - prompt) < 0.02 * prompt) ++sym;
        if (std::fabs(sig - s.sigma) < 1e-6 && m.wOut < 1e-9 && std::abs(symExc) < 1e-9 * maxIn) ++anti;
    }
    std::printf("  identical strings: status %d, condition %.2f; modes at the prompt rate %d, "
                "silent antisymmetric modes at the internal rate %d\n",
                static_cast<int>(u.status), u.condition, sym, anti);
    CHECK(sym == 1);
    CHECK(anti >= 2);  // two vertical (plus horizontal) antisymmetric modes

    pm::UnisonPartialSpec r = c4Spec(1.0);
    r.kappa = 0.0;
    const pm::UnisonPartial v = pm::solveUnisonPartial(r);
    double worst = 0.0;
    for (int j = 0; j < v.oscillators; ++j)
        worst = std::max(worst, std::fabs(-v.mode[j].lambda.real() - r.sigma));
    std::printf("  no coupling: every mode at the internal decay rate (worst deviation %.2e 1/s)\n", worst);
    CHECK_LE(worst, 1e-12);
}

// Detuning large against the coupling: the modes become the individual
// strings again (each with its own share kappa Re Yvv of bridge loss);
// detuning small: the in-phase mode takes the whole bridge loss. In between,
// the slowest vertical decay rate falls monotonically as the strings are
// brought into tune (the aftersound gets longer).
TEST(detuning_sweep) {
    double last = 1e9;
    bool mono = true;
    for (double width : {50.0, 10.0, 3.0, 1.0, 0.3, 0.1, 0.0}) {
        const pm::UnisonPartialSpec s = c4Spec(width);
        const pm::UnisonPartial u = pm::solveUnisonPartial(s);
        double slow = 1e9, fast = 0.0;
        for (int j = 0; j < 6; ++j) {  // vertical-dominated modes
            double vert = 0.0;
            for (int k = 0; k < 3; ++k) vert += std::norm(u.V[k * 6 + j]);
            if (vert < 0.5) continue;
            slow = std::min(slow, -u.mode[j].lambda.real());
            fast = std::max(fast, -u.mode[j].lambda.real());
        }
        std::printf("  unison width x%-5.1f (spread %.2f cents): vertical decay rates %.3f .. %.3f 1/s, status %d\n",
                    width, width * 0.8, slow, fast, static_cast<int>(u.status));
        if (width == 50.0) {
            const double single = s.sigma + s.kappa * s.Y.vv.real();
            CHECK_LE(std::fabs(fast - single), 0.05 * single);
            CHECK_LE(std::fabs(slow - single), 0.05 * single);
        }
        mono = mono && slow <= last + 1e-9;
        last = slow;
    }
    CHECK(mono);
}

// Every partial of every key of the default table decomposes without
// perturbation or fallback, and the kept mode count is in the expected range.
TEST(table_decomposes_cleanly) {
    pm::PianoTable t;
    pm::buildPianoTable(t);
    int total = 0, pert = 0, unc = 0;
    double worstCond = 0.0;
    for (int k = 0; k < pm::kNumKeys; ++k) {
        const pm::KeyModeSet m = pm::computeKeyModes(t.keys[k], 48000.0);
        total += m.count;
        pert += m.perturbedPartials;
        unc += m.uncoupledPartials;
        for (const pm::KeyPartial& p : m.partials) worstCond = std::max(worstCond, p.unison.condition);
    }
    std::printf("  88 keys: %d normal modes kept, %d partials perturbed, %d uncoupled, worst condition %.1f\n",
                total, pert, unc, worstCond);
    CHECK(unc == 0);
    CHECK(total > 15000 && total < 30000);
}
