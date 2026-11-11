// No double counting (plan §1.6): the unison bake excludes exactly what the
// run-time soundboard loop feeds back. A key alone on the board, run with the
// real loop components (bank coupled stepping with its static correction,
// key-zone mixing, run-time board), decays like the eigen-solve with the full
// driving-point admittance predicts; with the loop disabled (bake of the
// full admittance, nothing fed back) it does so by construction.
//
// Method: identical strings and perfect hammer mating (one in-phase mode per
// partial carries the bridge coupling); the full-bake eigenvector of a
// partial's dominant normal mode is set as the strings' state (projected
// onto the run-time normal modes) and the decay of its own amplitude (its
// left eigenvector applied to the state) is measured over 0.5 s.
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <vector>

#include "pm_base/denormals.h"
#include "pm_string/string_modes.h"
#include "pm_voicing/key_loader.h"
#include "pm_voicing/soundboard_model.h"
#include "test_harness.h"

namespace {

constexpr double kFs = 48000.0;
using cd = std::complex<double>;

struct Rates {
    double eigen = 0.0;  // full-bake prediction of the dominant mode, 1/s
    double off = 0.0, on = 0.0;
    double boardSigma = 0.0;  // decay rate of the nearest board mode
};

double run(const pm::PianoTable& t, int key, int p, double g, const pm::KeyModeSet& full, int best) {
    const pm::KeyParams& k = t.keys[key - 1];
    const pm::BoardModel& b = pm::defaultBoard();
    const pm::SoundboardBridge bridge(b, kFs, g, &t);
    const pm::KeyModeSet m = pm::computeKeyModes(k, kFs, bridge);
    int nb = pm::keyBankModes(m);
    pm::ModeBank bank;
    bank.allocateSegments(&nb, 1);
    pm::ContactTables ct;
    ct.allocate(bank.capacity(), 1);
    pm::loadKeyModes(bank, ct, 0, k, m, kFs);
    bank.setSegmentDamper(0, 0.0f);
    bank.setSegmentCoupledGroups(0, pm::keyCoupledGroups(m));
    bank.computeStaticCorrection(0);
    bank.setCulling(false);
    const pm::KeyPartial& kp = m.partials[static_cast<std::size_t>(p)];
    const pm::KeyPartial& fp = full.partials[static_cast<std::size_t>(p)];
    const int no = fp.unison.oscillators;
    cd x[pm::kMaxOscillators];
    for (int q = 0; q < no; ++q) x[q] = fp.unison.V[q * no + best] * 1e-5;
    for (int r = 0; r < kp.kept; ++r) {
        cd w = 0.0;
        for (int q = 0; q < no; ++q) w += kp.unison.Vinv[kp.keptIndex[r] * no + q] * x[q];
        bank.setState(pm::bankIndex(m, kp.firstMode + r), static_cast<float>(w.real()), static_cast<float>(w.imag()));
    }
    pm::Soundboard sb;
    pm::loadSoundboard(sb, b, kFs);
    pm::KeyZone kz = b.keyZone[key - 1];
    kz.forceScale = static_cast<float>(1.0 / k.outputGain);
    auto amplitude = [&] {
        cd xs[pm::kMaxOscillators] = {};
        for (int r = 0; r < kp.kept; ++r) {
            const int i = pm::bankIndex(m, kp.firstMode + r);
            const cd w(bank.stateRe(i), bank.stateIm(i));
            for (int q = 0; q < no; ++q) xs[q] += kp.unison.V[q * no + kp.keptIndex[r]] * w;
        }
        cd a = 0.0;
        for (int q = 0; q < no; ++q) a += fp.unison.Vinv[best * no + q] * xs[q];
        return std::abs(a);
    };
    pm::SegmentIO io;
    float buf[pm::ModeBank::kCullInterval];
    io.out = buf;
    pm::CouplingLoop loop;
    loop.configure(&kz, 1, b.zones);
    loop.setGain(static_cast<float>(g));
    float zf[pm::Soundboard::kMaxZones], zd[pm::Soundboard::kMaxZones] = {}, pk[4];
    const int n0 = static_cast<int>(0.05 * kFs), n1 = static_cast<int>(0.55 * kFs);
    double a0 = 0.0;
    for (int s = 0; s < n1; s += 32) {
        bank.processSegmentsFree(&io, 32);
        loop.begin(bank, &io, 32);
        const int* oz = nullptr;
        const int no = loop.outZones(oz);
        for (int i = 0; i < 32; ++i) {
            for (int z = 0; z < b.zones; ++z) zf[z] = 0.0f;
            loop.forces(i, zf, nullptr);
            sb.step(zf, zd, pk, pm::Kernel::Simd, oz, no);
            loop.push(i, zd);
        }
        loop.end(bank, &io, 32);
        bank.endCoupled(&io, 32);
        if (s + 32 == n0) a0 = amplitude();
    }
    return std::log(a0 / amplitude()) / ((n1 - n0) / kFs);
}

}  // namespace

TEST(loop_on_matches_bake) {
    pm::ScopedFlushDenormals ftz;
    pm::TableOptions o;
    o.unisonWidth = 0.0;
    o.matingSpreadM = 0.0;
    pm::PianoTable t;
    pm::buildPianoTable(t, o);
    const pm::BoardModel& b = pm::defaultBoard();
    const pm::SoundboardBridge fullBridge(b, kFs, 0.0);
    int weak = 0, weakOk = 0, all = 0;
    double worstWeak = 0.0, worstAll = 0.0;
    std::vector<double> devs;
    for (int key : {16, 40, 64}) {
        const pm::KeyModeSet full = pm::computeKeyModes(t.keys[key - 1], kFs, fullBridge);
        for (int p = 0; p < 8 && p < static_cast<int>(full.partials.size()); ++p) {
            const pm::UnisonPartial& u = full.partials[static_cast<std::size_t>(p)].unison;
            if (u.mode[0].lambda.imag() / (2.0 * 3.141592653589793) >= b.params.maxFreqHz) break;
            int best = 0;
            for (int j = 1; j < u.oscillators; ++j)
                if (u.mode[j].score > u.mode[best].score) best = j;
            Rates r;
            r.eigen = -u.mode[best].lambda.real();
            const double f = u.mode[best].lambda.imag() / (2.0 * 3.141592653589793);
            double gap = 1e30;
            for (int i = 0; i < b.modes; ++i)
                if (std::fabs(b.freq[i] - f) < gap) {
                    gap = std::fabs(b.freq[i] - f);
                    r.boardSigma = b.sigma[i];
                }
            r.off = run(t, key, p, 0.0, full, best);
            r.on = run(t, key, p, 1.0, full, best);
            // Strong coupling: the bridge loss of the mode is comparable to
            // the bandwidth of the nearest board mode, where the narrowband
            // bake is only approximate (the loop is exact).
            const double share = r.eigen - pm::partialDecayRate(t.keys[key - 1].b1, t.keys[key - 1].b3, f);
            const bool strong = share > 0.3 * r.boardSigma;
            const double dev = std::fabs(r.on / r.off - 1.0);
            std::printf("  key %2d partial %d (%7.1f Hz): eigen %.3f  loop off %.3f  loop on %.3f 1/s (on/off %.3f)%s\n",
                        key, p + 1, f, r.eigen, r.off, r.on, r.on / r.off, strong ? "  strong coupling" : "");
            CHECK_LE(std::fabs(r.off / r.eigen - 1.0), 0.01);
            ++all;
            worstAll = std::max(worstAll, dev);
            devs.push_back(dev);
            if (!strong) {
                ++weak;
                weakOk += dev <= 0.03;
                worstWeak = std::max(worstWeak, dev);
            }
        }
    }
    std::sort(devs.begin(), devs.end());
    const double median = devs[devs.size() / 2];
    std::printf("  %d partials: median deviation loop on vs off %.2f %%, worst %.1f %%; weakly coupled: %d of %d "
                "within 3 %%\n",
                all, 100 * median, 100 * worstAll, weakOk, weak);
    CHECK(all >= 16);
    CHECK_LE(median, 0.01);
    CHECK_LE(worstAll, 0.04);
}
