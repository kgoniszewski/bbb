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
    long approx = 0, exactResponses = 0;  // group responses expanded / computed exactly
    bool finite = true;
};

// Deliberately broken variant, which the test must flag ('flipped'): the
// loop feeds back with the opposite sign while the bake still removes the
// loop's share, so the strings see the driving point minus twice the loop's
// self admittance, whose real part is negative at some board resonances.
// (A loop whose strings answer the board one sub-block late, what the exact
// block solve replaces, grows too slowly at these coupling strengths to be
// caught within 0.5 s and is not used as the detector.)
// moving: every key's damper amount takes a random step on every sub-block
// (within 0..A, A random in 0.02..1.5: near-lossless strings stress
// passivity most), and the bank's damper-response budget is 0-2 groups per
// sub-block, so the loop runs on approximate (expanded) and stale-amount
// responses most of the time (ModeBank::coupledResponse).
Result runSet(const pm::PianoTable& base, std::uint64_t seed, int samples, bool flipped = false, bool moving = false) {
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
    if (moving) bank.precomputeCoupledResponses();
    const int budget = moving ? r.range(0, 2) : -1;
    const double maxAmount = moving ? std::exp(r.uni(std::log(0.02), std::log(1.5))) : 0.0;
    std::vector<float> amount(static_cast<std::size_t>(nk));
    for (int i = 0; i < nk; ++i) amount[i] = bank.segmentDamper(i);
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

    pm::CouplingLoop loop;
    loop.configure(kz.data(), nk, board.zones);
    loop.setGain(static_cast<float>(flipped ? -g : g));
    float zf[pm::Soundboard::kMaxZones], zd[pm::Soundboard::kMaxZones] = {}, pk[pm::Soundboard::kMaxPickups];
    const double e0 = stringEnergy() + sb.energy();
    double q3 = 0.0, q4 = 0.0;
    int n3 = 0, n4 = 0;
    // The engine's sub-block sequence, production code throughout.
    for (int s = 0; s + 32 <= samples; s += 32) {
        if (moving) {
            for (int i = 0; i < nk; ++i) {
                amount[i] = static_cast<float>(std::clamp(amount[i] + maxAmount * r.uni(-0.25, 0.25), 0.0, maxAmount));
                bank.setSegmentDamper(i, r.uni(0.0, 1.0) < 0.1 ? 0.0f : amount[i]);
            }
            bank.beginResponseTick(budget);
            bank.refineResponses();
        }
        loop.begin(bank, sb, io.data());
        bank.processSegmentsFree(io.data(), 32);
        loop.collect(bank, io.data(), 32);
        const int* oz = nullptr;
        const int no = loop.outZones(oz);
        for (int i = 0; i < 32; ++i) {
            for (int z = 0; z < board.zones; ++z) zf[z] = 0.0f;
            loop.forces(i, zf, nullptr);
            sb.step(zf, zd, pk, pm::Kernel::Simd, oz, no);
            loop.push(i, zd);
        }
        loop.end(bank, io.data(), 32);
        bank.endCoupled(io.data(), 32);
        const double e = stringEnergy() + sb.energy();
        if (!std::isfinite(e)) {
            res.finite = false;
            return res;
        }
        res.peak = std::max(res.peak, e / e0);
        if (4 * (s + 32) > 3 * samples) {
            q4 += e;
            ++n4;
        } else if (2 * (s + 32) > samples) {
            q3 += e;
            ++n3;
        }
        res.final = e / e0;
    }
    res.lateRise = n3 > 0 && n4 > 0 && q3 > 0.0 ? (q4 / n4) / (q3 / n3) : 0.0;
    res.approx = static_cast<long>(bank.responseGroupsApproximated());
    res.exactResponses = static_cast<long>(bank.responseGroupsComputed());
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

// The same with the damper amounts moving on every sub-block and a tiny
// damper-response budget: the loop's responses are mostly the expansion in
// the amount, or exact at an amount the damper has since left (until the
// refinement catches up), while the strings always run on their exact
// coefficients. Such responses must not add energy either.
TEST(energy_never_grows_moving_dampers) {
    pm::ScopedFlushDenormals ftz;
    pm::PianoTable base;
    pm::buildPianoTable(base);
    const int kSets = 3000;
    int bad = 0;
    long approx = 0, exact = 0;
    double worstPeak = 0.0, worstFinal = 0.0, worstLate = 0.0;
    for (int i = 0; i < kSets; ++i) {
        const int samples = i % 50 == 0 ? 24000 : 1216;
        const Result r = runSet(base, 50000 + std::uint64_t(i), samples, false, true);
        approx += r.approx;
        exact += r.exactResponses;
        const bool ok = r.finite && r.peak <= 1.03 && r.final <= 1.0 && r.lateRise <= 1.001;
        if (!ok) {
            if (bad < 10)
                std::printf("  set %d: peak %.4f final %.4f late rise %.4f finite %d\n", i, r.peak, r.final, r.lateRise,
                            r.finite ? 1 : 0);
            ++bad;
        }
        worstPeak = std::max(worstPeak, r.peak);
        worstFinal = std::max(worstFinal, r.final);
        worstLate = std::max(worstLate, r.lateRise);
    }
    std::printf("  %d random sets, dampers moving (%ld group responses expanded, %ld exact): worst E(t)/E(0) %.4f, "
                "worst E(end)/E(0) %.4f, worst late rise %.4f; %d violations\n",
                kSets, approx, exact, worstPeak, worstFinal, worstLate, bad);
    CHECK(bad == 0);
    CHECK(approx > exact);
}

// The test detects a loop that is not passive (runSet, flipped) on the same
// kind of random sets, run for 0.5 s each.
TEST(broken_loop_is_flagged) {
    pm::ScopedFlushDenormals ftz;
    pm::PianoTable base;
    pm::buildPianoTable(base);
    const int kSets = 120;
    int bad = 0;
    for (int i = 0; i < kSets; ++i) {
        const Result r = runSet(base, 1000 + std::uint64_t(i), 24000, true);
        if (!(r.finite && r.peak <= 1.03 && r.final <= 1.0 && r.lateRise <= 1.001)) ++bad;
    }
    std::printf("  sign-flipped loop: %d of %d random sets violate passivity\n", bad, kSets);
    CHECK(bad >= kSets / 20);
}
