// Passivity of the coupled strings + soundboard system (plan §1.6, §5.4,
// M4 acceptance gate): after an arbitrary initial excitation and with no
// input, the total energy never grows.
//
// Reduced model, real components: 2-4 random keys with 1-4 partials each
// (all strings, both polarizations, coupled normal modes), a random plate
// with a reduced modal range (400-1500 Hz, so a few dozen modes) and its
// zones; the bake (SoundboardBridge), the bank's coupled stepping with its
// static correction, the key/zone mixing and the run-time board are the
// production code, stepped exactly as the engine does (hammers excepted:
// they are an energy source). What is reduced: fewer keys, partials and
// board modes than the instrument, short runs.
//
// Randomised, within and beyond the user-reachable ranges: plate size,
// mass, stiffnesses, loss factor 0.3-6 %, zones per bridge 1-8, bridge
// contact length, horizontal ratio and cross coupling; key choice,
// partial count (some partials above the modal range: the uncoupled path),
// unison detuning 0-4x, internal string loss 0.02-2x (near-lossless strings
// stress passivity most), damper amount per key 0-1.5, sympathetic gain
// 0-1.6, culling on or off.
//
// Energy: physical string energy (all oscillators, key_loader.h) plus the
// board's modal energy. The interaction energy of the coupling is not
// included, so E may wobble by about the coupling ratio (~1e-2); growth is
// what is tested: E(t) <= 1.03 E(0) throughout, E(end) <= E(0), and E must
// fall on average: its mean over the last quarter of a run may not exceed
// its mean over the third quarter (by more than 0.1 %). Instability shows as
// a rising mean; the exchange with the uncounted interaction energy only as
// a wobble (sets flagged by a stricter pointwise test decayed steadily over
// 10 s).
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <vector>

#include "pm_base/denormals.h"
#include "pm_voicing/key_loader.h"
#include "pm_voicing/soundboard_model.h"
#include "test_harness.h"

namespace {

constexpr double kFs = 48000.0;

struct Rng {
    std::uint64_t s;
    std::uint64_t next() {
        std::uint64_t x = (s += 0x9E3779B97F4A7C15ull);
        x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ull;
        x = (x ^ (x >> 27)) * 0x94D049BB133111EBull;
        return x ^ (x >> 31);
    }
    double uni(double a, double b) { return a + (b - a) * double(next() >> 11) * (1.0 / 9007199254740992.0); }
    int range(int a, int b) { return a + static_cast<int>(next() % std::uint64_t(b - a + 1)); }
    double gauss() {
        double u = 0.0;
        for (int i = 0; i < 6; ++i) u += uni(-1.0, 1.0);
        return u * 0.7071;
    }
};

struct Result {
    double peak = 0.0;      // max E(t) / E(0)
    double final = 0.0;     // E(end) / E(0)
    double lateRise = 0.0;  // mean E over the last quarter / mean E over the third quarter
    int modes = 0, boardModes = 0;
    bool finite = true;
};

Result runSet(const pm::PianoTable& base, std::uint64_t seed, int samples) {
    Rng r{seed};
    pm::PlateParams pp;
    pp.lengthX = r.uni(1.6, 2.2);
    pp.widthY = r.uni(1.0, 1.5);
    pp.massPerArea = r.uni(4.0, 10.0);
    pp.Dx = r.uni(1000.0, 5000.0);
    pp.Dy = r.uni(8000.0, 40000.0);
    pp.Dxy = r.uni(0.2, 1.0) * std::sqrt(pp.Dx * pp.Dy);
    pp.etaLow = pp.etaHigh = r.uni(0.003, 0.06);
    pp.maxFreqHz = r.uni(400.0, 1500.0);
    pp.bass.zones = r.range(1, 5);
    pp.treble.zones = r.range(1, 8);
    pp.bridgeHalfLength = r.uni(0.0, 0.05);
    pp.horizontalRatio = r.uni(4.0, 30.0);
    pp.cross = r.uni(0.0, 0.6);
    pp.continuationPoints = 16;
    // Keep the bridges inside the plate.
    const double sx = pp.lengthX / 1.93, sy = pp.widthY / 1.25;
    for (pm::BridgeCurve* c : {&pp.bass, &pp.treble}) {
        c->x0 *= sx;
        c->cx *= sx;
        c->x1 *= sx;
        c->y0 *= sy;
        c->cy *= sy;
        c->y1 *= sy;
    }
    const pm::BoardModel board = pm::buildPlateBoard(pp);
    const double g = r.uni(0.0, 1.6);

    pm::PianoTable t = base;
    const int nk = r.range(2, 4);
    std::vector<int> keys;
    while (static_cast<int>(keys.size()) < nk) {
        const int k = r.range(1, 88);
        if (std::find(keys.begin(), keys.end(), k) == keys.end()) keys.push_back(k);
    }
    for (int k : keys) {
        pm::KeyParams& kp = t.keys[k - 1];
        kp.maxPartials = r.range(1, 4);
        const double loss = std::exp(r.uni(std::log(0.02), std::log(2.0)));
        kp.b1 *= loss;
        kp.b3 *= loss;
        const double w = r.uni(0.0, 4.0);
        for (double& d : kp.detuneCents) d *= w;
    }
    const pm::SoundboardBridge bridge(board, kFs, g, &t);

    std::vector<pm::KeyModeSet> sets(static_cast<std::size_t>(nk));
    std::vector<int> nb(static_cast<std::size_t>(nk));
    for (int i = 0; i < nk; ++i) {
        sets[i] = pm::computeKeyModes(t.keys[keys[i] - 1], kFs, bridge);
        nb[i] = pm::keyBankModes(sets[i]);
    }
    pm::ModeBank bank;
    bank.allocateSegments(nb.data(), nk);
    pm::ContactTables ct;
    ct.allocate(bank.capacity(), nk);
    std::vector<pm::KeyZone> kz(static_cast<std::size_t>(nk));
    std::vector<pm::SegmentIO> io(static_cast<std::size_t>(nk));
    std::vector<float> buf(static_cast<std::size_t>(nk) * pm::ModeBank::kCullInterval);
    Result res;
    for (int i = 0; i < nk; ++i) {
        const pm::KeyParams& kp = t.keys[keys[i] - 1];
        pm::loadKeyModes(bank, ct, i, kp, sets[i], kFs);
        bank.setSegmentCoupledGroups(i, pm::keyCoupledGroups(sets[i]));
        bank.computeStaticCorrection(i);
        bank.setSegmentDamper(i, r.uni(0.0, 1.0) < 0.5 ? 0.0f : static_cast<float>(r.uni(0.0, 1.5)));
        kz[i] = board.keyZone[keys[i] - 1];
        kz[i].forceScale = static_cast<float>(1.0 / kp.outputGain);
        io[i].out = buf.data() + std::size_t(i) * pm::ModeBank::kCullInterval;
        res.modes += sets[i].count;
    }
    bank.setCulling(r.uni(0.0, 1.0) < 0.5, -100.0f);
    pm::Soundboard sb;
    pm::loadSoundboard(sb, board, kFs);
    res.boardModes = board.modes;

    // Random initial state: 1e-4 m-scale string modes, board at a comparable energy.
    for (int i = 0; i < nk; ++i) {
        const int m0 = bank.segmentFirstMode(i);
        for (int j = 0; j < sets[i].count; ++j)
            bank.setState(m0 + pm::bankIndex(sets[i], j), static_cast<float>(1e-4 * r.gauss()),
                          static_cast<float>(1e-4 * r.gauss()));
    }
    auto stringEnergy = [&] {
        double e = 0.0;
        for (int i = 0; i < nk; ++i) {
            const int m0 = bank.segmentFirstMode(i);
            e += pm::keyStringEnergy(sets[i], bank.stateReData() + m0, bank.stateImData() + m0);
        }
        return e;
    };
    const double es = stringEnergy();
    for (int i = 0; i < board.modes; ++i) sb.setState(i, static_cast<float>(r.gauss()), static_cast<float>(r.gauss()));
    const double eb = sb.energy();
    const double scale = std::sqrt(es / std::max(eb, 1e-300));
    for (int i = 0; i < board.modes; ++i)
        sb.setState(i, static_cast<float>(sb.stateRe(i) * scale), static_cast<float>(sb.stateIm(i) * scale));

    std::vector<float> u(static_cast<std::size_t>(nk), 0.0f), F(static_cast<std::size_t>(nk)),
        Fc(static_cast<std::size_t>(nk));
    float zf[pm::Soundboard::kMaxZones], zd[pm::Soundboard::kMaxZones], pk[pm::Soundboard::kMaxPickups];
    const double e0 = stringEnergy() + sb.energy();
    double q3 = 0.0, q4 = 0.0;
    int n3 = 0, n4 = 0;
    for (int s = 0; s < samples; ++s) {
        const int sub = s % 32;
        if (sub == 0) {
            bank.processSegmentsUncoupled(io.data(), 32);
            bank.beginCoupled(io.data());
        }
        bank.stepCoupled(u.data(), Fc.data());
        for (int i = 0; i < nk; ++i) F[i] = Fc[i] + (io[i].wrote ? io[i].out[sub] : 0.0f);
        for (int z = 0; z < board.zones; ++z) zf[z] = 0.0f;
        pm::keysToZones(kz.data(), F.data(), nk, zf);
        sb.step(zf, zd, pk);
        if (g > 0.0) pm::zonesToKeys(kz.data(), zd, nk, static_cast<float>(g), u.data());
        if (sub == 31) {
            bank.endCoupled(io.data(), 32);
            const double e = stringEnergy() + sb.energy();
            if (!std::isfinite(e)) {
                res.finite = false;
                return res;
            }
            res.peak = std::max(res.peak, e / e0);
            if (4 * (s + 1) > 3 * samples) {
                q4 += e;
                ++n4;
            } else if (2 * (s + 1) > samples) {
                q3 += e;
                ++n3;
            }
            res.final = e / e0;
        }
    }
    res.lateRise = n3 > 0 && n4 > 0 && q3 > 0.0 ? (q4 / n4) / (q3 / n3) : 0.0;
    return res;
}

}  // namespace

TEST(energy_never_grows_random_sets) {
    pm::ScopedFlushDenormals ftz;
    pm::PianoTable base;
    pm::buildPianoTable(base);
    const int kSets = 10000;
    int bad = 0;
    double worstPeak = 0.0, worstFinal = 0.0, worstLate = 0.0;
    long modes = 0, boardModes = 0;
    for (int i = 0; i < kSets; ++i) {
        // Most runs short (25 ms); every 50th long (0.5 s), where slow growth would show.
        const int samples = i % 50 == 0 ? 24000 : 1216;
        const Result r = runSet(base, 1000 + std::uint64_t(i), samples);
        modes += r.modes;
        boardModes += r.boardModes;
        const bool ok = r.finite && r.peak <= 1.03 && r.final <= 1.0 && r.lateRise <= 1.001;
        if (!ok) {
            if (bad < 10)
                std::printf("  set %d: peak %.4f final %.4f late rise %.4f finite %d (%d string modes, %d board modes)\n",
                            i, r.peak, r.final, r.lateRise, r.finite ? 1 : 0, r.modes, r.boardModes);
            ++bad;
        }
        worstPeak = std::max(worstPeak, r.peak);
        worstFinal = std::max(worstFinal, r.final);
        worstLate = std::max(worstLate, r.lateRise);
    }
    std::printf("  %d random sets (%ld string modes, %ld board modes in total): worst E(t)/E(0) %.4f, worst "
                "E(end)/E(0) %.4f, worst mean(last quarter)/mean(third quarter) %.4f; %d violations\n",
                kSets, modes, boardModes, worstPeak, worstFinal, worstLate, bad);
    CHECK(bad == 0);
}
