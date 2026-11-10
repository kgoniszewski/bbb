// Segmented mode bank: per-key processing, damper loss, skipping, culling.
#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdio>
#include <numbers>
#include <vector>

#include "signal_utils.h"
#include "test_harness.h"

using namespace pm;

namespace {

constexpr double kFs = 48000.0;

StringParams c2() {
    StringParams p;
    p.f0 = 65.41;
    p.B = 7.4e-5;
    p.length = 1.9;
    p.linearDensity = 0.012;
    return p;
}

// Loads string p into segment s of a segmented bank, damper rate sigmaD.
void loadInto(ModeBank& bank, int s, const StringParams& p, double sigmaD) {
    std::vector<ModeSpec> m(static_cast<std::size_t>(countStringModes(p, kFs)));
    computeStringModes(p, kFs, m.data(), static_cast<int>(m.size()));
    const int m0 = bank.segmentFirstMode(s);
    for (std::size_t i = 0; i < m.size(); ++i) {
        bank.setMode(m0 + int(i), m[i].freqHz, m[i].sigma, kFs);
        bank.setWeights(m0 + int(i), float(m[i].wIn), float(m[i].wOut));
        bank.setDamperRate(m0 + int(i), sigmaD, kFs);
    }
}

std::vector<float> noise(int n, unsigned seed) {
    std::vector<float> v(static_cast<std::size_t>(n));
    unsigned x = seed;
    for (auto& e : v) {
        x = x * 1664525u + 1013904223u;
        e = (static_cast<float>(x >> 8) / 16777216.0f - 0.5f) * 10.0f;
    }
    return v;
}

}  // namespace

// Two strings in two segments with their own inputs give exactly what two
// separate single-segment banks give.
TEST(segments_match_separate_banks) {
    const StringParams a = StringParams{}, b = c2();
    const int counts[2] = {countStringModes(a, kFs), countStringModes(b, kFs)};
    ModeBank seg;
    CHECK(seg.allocateSegments(counts, 2));
    CHECK(seg.segmentFirstMode(1) % ModeBank::kGroupSize == 0);
    CHECK(seg.segmentModeCount(0) >= counts[0]);
    loadInto(seg, 0, a, 0.0);
    loadInto(seg, 1, b, 0.0);
    ModeBank sa, sb;
    sa.allocate(256);
    sb.allocate(256);
    loadString(sa, a, kFs);
    loadString(sb, b, kFs);

    const int n = 4800, block = 37;
    const auto ua = noise(n, 1), ub = noise(n, 2);
    std::vector<float> ya(n), yb(n), ra(n, 0.0f), rb(n, 0.0f);
    SegmentIO io[2];
    for (int pos = 0; pos < n; pos += block) {
        const int len = std::min(block, n - pos);
        io[0].in = ua.data() + pos;
        io[0].out = ya.data() + pos;
        io[1].in = ub.data() + pos;
        io[1].out = yb.data() + pos;
        seg.processSegments(io, len);
        CHECK(io[0].wrote && io[1].wrote);
        sa.process(ua.data() + pos, ra.data() + pos, len);
        sb.process(ub.data() + pos, rb.data() + pos, len);
    }
    double d = 0.0;
    for (int i = 0; i < n; ++i) d = std::max({d, double(std::fabs(ya[i] - ra[i])), double(std::fabs(yb[i] - rb[i]))});
    std::printf("  max difference %.3g\n", d);
    CHECK(d == 0.0);
}

// The damper multiplies each coefficient by exp(-d sigmaD / fs) exactly (to
// float precision), at any amount, and returns to the free value at 0.
TEST(damper_scales_decay_rate) {
    const StringParams p;
    const int count = countStringModes(p, kFs);
    ModeBank bank;
    bank.allocateSegments(&count, 1);
    loadInto(bank, 0, p, 40.0);
    std::vector<std::complex<double>> free(static_cast<std::size_t>(count));
    for (int i = 0; i < count; ++i) free[i] = bank.coefficient(i);
    double worst = 0.0;
    for (float d : {0.25f, 1.0f, 4.0f}) {
        bank.setSegmentDamper(0, d);
        for (int i = 0; i < count; ++i) {
            const double sigma = -std::log(std::abs(bank.coefficient(i))) * kFs;
            const double sigma0 = -std::log(std::abs(free[i])) * kFs;
            worst = std::max(worst, std::fabs((sigma - sigma0) / (d * 40.0) - 1.0));
            CHECK_NEAR(std::arg(bank.coefficient(i)), std::arg(free[i]), 1e-6);
        }
    }
    bank.setSegmentDamper(0, 0.0f);
    bool back = true;
    for (int i = 0; i < count; ++i) back = back && bank.coefficient(i) == free[i];
    std::printf("  worst relative error of the extra decay rate %.2e\n", worst);
    CHECK_LE(worst, 2e-3);
    CHECK(back);
}

TEST(skipped_segment_is_untouched) {
    const StringParams p;
    const int counts[2] = {countStringModes(p, kFs), countStringModes(p, kFs)};
    ModeBank bank;
    bank.allocateSegments(counts, 2);
    loadInto(bank, 0, p, 0.0);
    loadInto(bank, 1, p, 0.0);
    for (int s = 0; s < 2; ++s)
        for (int i = 0; i < counts[s]; ++i) bank.setState(bank.segmentFirstMode(s) + i, 0.0f, 1e-4f);
    std::vector<float> y0(64), y1(64, 123.0f);
    SegmentIO io[2];
    io[0].out = y0.data();
    io[1].out = y1.data();
    io[1].skip = true;
    const float before = bank.stateIm(bank.segmentFirstMode(1));
    bank.processSegments(io, 64);
    CHECK(io[0].wrote);
    CHECK(!io[1].wrote);
    CHECK(y1[0] == 123.0f);
    CHECK(bank.stateIm(bank.segmentFirstMode(1)) == before);
    CHECK(bank.stateIm(bank.segmentFirstMode(0)) != before);
}

// Culling with segments, per-segment input and damper changes stays within
// -90 dB of the unculled result, and a damped, silent bank goes to sleep.
TEST(culling_with_segments_and_dampers) {
    const StringParams a = StringParams{}, b = c2();
    const int counts[2] = {countStringModes(a, kFs), countStringModes(b, kFs)};
    ModeBank x, y;
    for (ModeBank* m : {&x, &y}) {
        m->allocateSegments(counts, 2);
        loadInto(*m, 0, a, 30.0);
        loadInto(*m, 1, b, 15.0);
        m->setSegmentDamper(0, 0.0f);
        m->setSegmentDamper(1, 0.0f);
    }
    y.setCulling(true, -100.0f);
    const int n = static_cast<int>(4 * kFs);
    std::vector<float> u(n, 0.0f);
    u[100] = 20.0f;  // a hammer-sized force pulse
    u[101] = 20.0f;
    std::vector<float> xa(n), xb(n), ya(n), yb(n);
    SegmentIO iox[2], ioy[2];
    ScopedFlushDenormals ftz;
    double peak = 0.0, diff = 0.0;
    for (int pos = 0; pos < n; pos += 64) {
        if (pos == static_cast<int>(kFs)) {  // dampers land after 1 s
            for (ModeBank* m : {&x, &y}) {
                m->setSegmentDamper(0, 1.0f);
                m->setSegmentDamper(1, 1.0f);
            }
        }
        iox[0].in = ioy[0].in = u.data() + pos;
        iox[1].in = ioy[1].in = u.data() + pos;
        iox[0].out = xa.data() + pos;
        iox[1].out = xb.data() + pos;
        ioy[0].out = ya.data() + pos;
        ioy[1].out = yb.data() + pos;
        x.processSegments(iox, 64);
        y.processSegments(ioy, 64);
        for (int s = 0; s < 2; ++s) {
            const float* px = iox[s].wrote ? iox[s].out : nullptr;
            const float* py = ioy[s].wrote ? ioy[s].out : nullptr;
            for (int k = 0; k < 64; ++k) {
                const double vx = px ? px[k] : 0.0, vy = py ? py[k] : 0.0;
                peak = std::max(peak, std::fabs(vx));
                diff = std::max(diff, std::fabs(vx - vy));
            }
        }
    }
    std::printf("  peak %.3f, max |culled - unculled| %.2e (%.1f dBFS), awake groups at end %d of %d\n",
                peak, diff, 20 * std::log10(diff + 1e-30), y.activeGroups(), y.groupCount());
    CHECK(peak > 0.01);
    CHECK_LE(20 * std::log10(diff + 1e-30), -90.0);
    CHECK(y.activeGroups() == 0);
}
