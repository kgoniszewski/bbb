// Global voicing scales (API 0.19, params 30-37; 0.20, params 39-41): defaults leave the table and
// the output untouched, each scale moves its target, the C API clamps.
#include <cmath>
#include <complex>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "pm_api/pm_api.h"
#include "pm_engine/engine.h"
#include "pm_voicing/piano_table.h"
#include "pm_voicing/radiation_model.h"
#include "test_harness.h"

namespace {

constexpr int kC4 = 39;  // key 40

bool sameKey(const pm::KeyParams& a, const pm::KeyParams& b) {
    bool ok = a.key == b.key && a.f1 == b.f1 && a.f0 == b.f0 && a.length == b.length && a.tension == b.tension &&
              a.B == b.B && a.linearDensity == b.linearDensity && a.b1 == b.b1 && a.b3 == b.b3 &&
              a.maxPartials == b.maxPartials && a.maxFreqHz == b.maxFreqHz && a.strings == b.strings &&
              a.polarizationSplitCents == b.polarizationSplitCents && a.bridgeCoupling == b.bridgeCoupling &&
              a.fullCoupledPartials == b.fullCoupledPartials && a.modesPerHighPartial == b.modesPerHighPartial &&
              a.hammerMass == b.hammerMass && a.feltK == b.feltK && a.feltP == b.feltP && a.feltMu == b.feltMu &&
              a.strikePos == b.strikePos && a.contactOversampling == b.contactOversampling &&
              a.strikeHorizontal == b.strikeHorizontal && a.unaCordaString == b.unaCordaString &&
              a.unaCordaMiss == b.unaCordaMiss && a.unaCordaSoften == b.unaCordaSoften &&
              a.hasDamper == b.hasDamper && a.damperSigma == b.damperSigma && a.damperPos == b.damperPos &&
              a.damperWidth == b.damperWidth && a.damperFloor == b.damperFloor && a.outputGain == b.outputGain &&
              a.pan == b.pan;
    for (int s = 0; s < pm::kMaxStrings; ++s)
        ok = ok && a.detuneCents[s] == b.detuneCents[s] && a.strikeOffset[s] == b.strikeOffset[s];
    return ok;
}

pm::PianoTable build(const pm::TableOptions& o) {
    pm::PianoTable t;
    pm::buildPianoTable(t, o);
    return t;
}

double feltForce(const pm::KeyParams& k, double x) { return k.feltK * std::pow(x, k.feltP); }

std::vector<float> render(pm_engine* e) {
    const struct { uint64_t t; uint8_t note; uint16_t value; } ev[] = {
        {10, 60, 40000}, {300, 64, 52000}, {300, 33, 30000}, {600, 96, 45000}};
    for (const auto& x : ev) {
        pm_event p{};
        p.time = x.t;
        p.type = PM_EVENT_NOTE_ON;
        p.time_base = PM_TIME_SAMPLE;
        p.note = x.note;
        p.value = x.value;
        CHECK(pm_engine_push_event(e, &p) == 1);
    }
    std::vector<float> out;
    float l[64], r[64];
    for (int b = 0; b < 120; ++b) {
        pm_engine_render(e, l, r, 64, nullptr);
        out.insert(out.end(), l, l + 64);
        out.insert(out.end(), r, r + 64);
    }
    return out;
}

const int32_t kIds[] = {PM_PARAM_VOICING_FELT_HARDNESS, PM_PARAM_VOICING_FELT_EXPONENT,
                        PM_PARAM_VOICING_HAMMER_MASS,   PM_PARAM_VOICING_STRIKE_POSITION,
                        PM_PARAM_VOICING_STRING_LOSS,   PM_PARAM_VOICING_HF_LOSS,
                        PM_PARAM_VOICING_INHARMONICITY, PM_PARAM_VOICING_BOARD_DAMPING,
                        PM_PARAM_VOICING_RADIATION_TILT, PM_PARAM_VOICING_RADIATION_CORNER_HZ,
                        PM_PARAM_VOICING_FELT_DAMPING};

}  // namespace

TEST(defaults_leave_table_untouched) {
    pm::TableOptions explicitDefaults;
    explicitDefaults.feltHardness = 1.0;
    explicitDefaults.feltExponentOffset = 0.0;
    explicitDefaults.hammerMassScale = 1.0;
    explicitDefaults.strikePosScale = 1.0;
    explicitDefaults.stringLossScale = 1.0;
    explicitDefaults.hfLossScale = 1.0;
    explicitDefaults.inharmonicityScale = 1.0;
    explicitDefaults.feltDampingScale = 1.0;
    const pm::PianoTable a = build(pm::TableOptions{}), b = build(explicitDefaults);
    for (int i = 0; i < pm::kNumKeys; ++i) CHECK(sameKey(a.keys[i], b.keys[i]));
    // C4 is an anchor row: the plan §2.1 values come through unchanged
    // (feltK x0.7 after A/B listening, piano_table.cpp).
    const pm::KeyParams& k = a.keys[kC4];
    CHECK_NEAR(k.hammerMass, 9e-3, 1e-15);
    CHECK_NEAR(k.feltK / 3.15e9, 1.0, 1e-12);  // plan 4.5e9 x0.7 (A/B listening, 2026-11-19)
    CHECK_NEAR(k.feltP, 2.5, 1e-12);
    CHECK_NEAR(k.strikePos, 0.12, 1e-12);
    CHECK_NEAR(k.b1, 0.335, 1e-12);
    CHECK_NEAR(k.b3 / 3.5e-9, 1.0, 1e-12);
    CHECK_NEAR(k.B / 3.8e-4, 1.0, 1e-12);
}

TEST(defaults_render_bit_identical) {
    pm_engine* a = pm_engine_create();
    pm_engine* b = pm_engine_create();
    for (int32_t id : kIds) {
        pm_param_info_t d{};
        CHECK(pm_param_info(id, &d) == 1);
        CHECK(d.cls == PM_PARAM_CLASS_PREPARE);
        CHECK((d.flags & PM_PARAM_FLAG_HIDDEN) && (d.flags & PM_PARAM_FLAG_NON_REALTIME_SET));
        CHECK(pm_engine_set_param(b, id, d.default_value) == 1);
    }
    CHECK(pm_engine_prepare(a, 48000.0, 64) == 1);
    CHECK(pm_engine_prepare(b, 48000.0, 64) == 1);
    const std::vector<float> ra = render(a), rb = render(b);
    CHECK(ra == rb);
    // And a non-default value does reach the output.
    pm_engine* c = pm_engine_create();
    CHECK(pm_engine_set_param(c, PM_PARAM_VOICING_FELT_HARDNESS, 2.0) == 1);
    CHECK(pm_engine_prepare(c, 48000.0, 64) == 1);
    CHECK(render(c) != ra);
    pm_engine_destroy(a);
    pm_engine_destroy(b);
    pm_engine_destroy(c);
}

TEST(each_scale_moves_its_target) {
    const pm::PianoTable ref = build(pm::TableOptions{});
    for (int i = 0; i < pm::kNumKeys; ++i) {
        const pm::KeyParams& r = ref.keys[i];
        pm::TableOptions o;
        o.hammerMassScale = 2.0;
        o.hfLossScale = 0.5;
        o.stringLossScale = 4.0;
        const pm::KeyParams k = build(o).keys[i];
        CHECK_NEAR(k.hammerMass / r.hammerMass, 2.0, 1e-14);
        CHECK_NEAR(k.b3 / r.b3, 0.5, 1e-14);
        CHECK_NEAR(k.b1 / r.b1, 4.0, 1e-14);
        // Mass does not move the felt (its reference is the unscaled hammer).
        CHECK(k.feltK == r.feltK && k.feltP == r.feltP);
    }
    // Strike position: scaled, clamped at both ends, the damper follows.
    {
        pm::TableOptions o;
        o.strikePosScale = 1.5;
        const pm::PianoTable t = build(o);
        CHECK_NEAR(t.keys[kC4].strikePos, 0.18, 1e-12);
        CHECK_NEAR(t.keys[kC4].damperPos - t.keys[kC4].strikePos,
                   ref.keys[kC4].damperPos - ref.keys[kC4].strikePos, 1e-12);
        o.strikePosScale = 10.0;
        const pm::PianoTable hi = build(o);
        o.strikePosScale = 0.01;
        const pm::PianoTable lo = build(o);
        for (int i = 0; i < pm::kNumKeys; ++i) {
            CHECK(hi.keys[i].strikePos == 0.30);
            CHECK(lo.keys[i].strikePos == 0.02);
        }
    }
    // Inharmonicity: B scaled, the sounding f1 kept, f0 and density follow.
    for (double s : {0.0, 0.5, 2.0}) {
        pm::TableOptions o;
        o.inharmonicityScale = s;
        const pm::PianoTable t = build(o);
        for (int i = 0; i < pm::kNumKeys; ++i) {
            const pm::KeyParams &k = t.keys[i], &r = ref.keys[i];
            CHECK(k.f1 == r.f1);
            CHECK_NEAR(k.B, s * r.B, 1e-18);
            CHECK(k.f0 == k.f1 / std::sqrt(1.0 + k.B));
            const double c = 2.0 * k.length * k.f0;
            CHECK_NEAR(k.linearDensity / (k.tension / (c * c)), 1.0, 1e-14);
            if (s == 0.0) CHECK(k.f0 == k.f1);
        }
    }
    // Felt: hardness scales the force at the reference compression, the
    // exponent offset keeps it there and changes it elsewhere.
    for (int i = 0; i < pm::kNumKeys; ++i) {
        const pm::KeyParams& r = ref.keys[i];
        const double xRef = pm::voicingRefCompression(r.hammerMass, r.strings, r.feltK, r.feltP);
        CHECK(xRef > 1e-4 && xRef < 2e-3);  // 0.1-2 mm: the working range of the felt
        pm::TableOptions o;
        o.feltHardness = 3.0;
        const pm::KeyParams h = build(o).keys[i];
        CHECK_NEAR(feltForce(h, xRef) / feltForce(r, xRef), 3.0, 1e-12);
        CHECK(h.feltP == r.feltP);
        for (double dp : {-1.0, 0.7}) {
            pm::TableOptions oe;
            oe.feltExponentOffset = dp;
            const pm::KeyParams e = build(oe).keys[i];
            CHECK_NEAR(e.feltP, std::max(1.5, r.feltP + dp), 1e-15);
            CHECK_NEAR(feltForce(e, xRef) / feltForce(r, xRef), 1.0, 1e-12);
            if (e.feltP > r.feltP) CHECK(feltForce(e, 2.0 * xRef) > feltForce(r, 2.0 * xRef));
        }
    }
}

TEST(api_clamps_and_rejects_nan) {
    pm_engine* e = pm_engine_create();
    const struct { int32_t id; double lo, hi, def; } r[] = {
        {PM_PARAM_VOICING_FELT_HARDNESS, 0.25, 4.0, 1.0}, {PM_PARAM_VOICING_FELT_EXPONENT, -1.0, 1.0, 0.0},
        {PM_PARAM_VOICING_HAMMER_MASS, 0.5, 2.0, 1.0},    {PM_PARAM_VOICING_STRIKE_POSITION, 0.5, 2.0, 1.0},
        {PM_PARAM_VOICING_STRING_LOSS, 0.25, 4.0, 1.0},   {PM_PARAM_VOICING_HF_LOSS, 0.25, 4.0, 1.0},
        {PM_PARAM_VOICING_INHARMONICITY, 0.0, 2.0, 1.0},  {PM_PARAM_VOICING_BOARD_DAMPING, 0.25, 4.0, 1.0},
        {PM_PARAM_VOICING_RADIATION_TILT, -9.0, 0.0, 0.0}, {PM_PARAM_VOICING_RADIATION_CORNER_HZ, 250.0, 8000.0, 1000.0},
        {PM_PARAM_VOICING_FELT_DAMPING, 0.25, 8.0, 1.0}};
    for (const auto& p : r) {
        double v = -99.0;
        CHECK(pm_engine_get_param(e, p.id, &v) == 1 && v == p.def);
        CHECK(pm_engine_set_param(e, p.id, p.hi + 10.0) == 1);
        CHECK(pm_engine_get_param(e, p.id, &v) == 1 && v == p.hi);
        CHECK(pm_engine_set_param(e, p.id, p.lo - 10.0) == 1);
        CHECK(pm_engine_get_param(e, p.id, &v) == 1 && v == p.lo);
        CHECK(pm_engine_set_param(e, p.id, std::nan("")) == 0);
        CHECK(pm_engine_get_param(e, p.id, &v) == 1 && v == p.lo);
    }
    pm_engine_destroy(e);
}

TEST(board_damping_scales_board_loss) {
    pm::Engine a, b;
    pm::SoundboardConfig cfg;
    a.setSoundboardConfig(cfg);
    cfg.dampingScale = 2.0;
    b.setSoundboardConfig(cfg);
    CHECK(a.prepare(48000.0, 64));
    CHECK(b.prepare(48000.0, 64));
    const pm::PlateParams& pa = a.boardModel().params;
    const pm::PlateParams& pb = b.boardModel().params;
    CHECK(pb.etaLow == 2.0 * pa.etaLow && pb.etaHigh == 2.0 * pa.etaHigh && pb.etaBass == 2.0 * pa.etaBass);
    CHECK(a.boardModel().modes == b.boardModel().modes && a.boardModel().modes > 0);
    const int m = a.boardModel().modes / 2;
    CHECK(a.boardModel().freq[m] == b.boardModel().freq[m]);
    CHECK_NEAR(b.boardModel().sigma[m] / a.boardModel().sigma[m], 2.0, 0.02);
    // Through the C API the value reaches the output.
    pm_engine* r0 = pm_engine_create();
    pm_engine* r1 = pm_engine_create();
    CHECK(pm_engine_set_param(r1, PM_PARAM_VOICING_BOARD_DAMPING, 2.0) == 1);
    CHECK(pm_engine_prepare(r0, 48000.0, 64) == 1);
    CHECK(pm_engine_prepare(r1, 48000.0, 64) == 1);
    CHECK(render(r0) != render(r1));
    pm_engine_destroy(r0);
    pm_engine_destroy(r1);
}

TEST(felt_damping_scales_felt_mu) {
    const pm::PianoTable ref = build(pm::TableOptions{});
    pm::TableOptions o;
    o.feltDampingScale = 2.0;
    const pm::PianoTable t = build(o);
    for (int i = 0; i < pm::kNumKeys; ++i) {
        CHECK(t.keys[i].feltMu == 2.0 * ref.keys[i].feltMu);
        CHECK(t.keys[i].feltK == ref.keys[i].feltK && t.keys[i].feltP == ref.keys[i].feltP);
    }
    pm_engine* r0 = pm_engine_create();
    pm_engine* r1 = pm_engine_create();
    CHECK(pm_engine_set_param(r1, PM_PARAM_VOICING_FELT_DAMPING, 2.0) == 1);
    CHECK(pm_engine_prepare(r0, 48000.0, 64) == 1);
    CHECK(pm_engine_prepare(r1, 48000.0, 64) == 1);
    CHECK(render(r0) != render(r1));
    pm_engine_destroy(r0);
    pm_engine_destroy(r1);
}

TEST(radiation_tilt_scales_mic_gains_and_direct_path) {
    pm::Engine a, b;
    pm::SoundboardConfig cfg;
    a.setSoundboardConfig(cfg);
    cfg.radiationTiltDbPerOct = -6.0;
    cfg.radiationTiltCornerHz = 1000.0;
    b.setSoundboardConfig(cfg);
    CHECK(a.prepare(48000.0, 64));
    CHECK(b.prepare(48000.0, 64));
    const pm::BoardModel& bm = a.boardModel();
    const pm::MicGains& ga = a.micGains();
    const pm::MicGains& gb = b.micGains();
    CHECK(ga.modes == bm.modes && gb.modes == bm.modes && bm.modes > 0);
    auto tilt = [](double f) { return f <= 1000.0 ? 1.0 : std::pow(f / 1000.0, -6.0 / (20.0 * std::log10(2.0))); };
    int near2k = 0, below = 0;
    for (int i = 0; i < bm.modes; ++i) {
        if (std::fabs(bm.freq[i] - 2000.0) < std::fabs(bm.freq[near2k] - 2000.0)) near2k = i;
        for (int mic = 0; mic < 2; ++mic) {
            const std::complex<double> x = ga.gain[std::size_t(mic) * bm.modes + i];
            const std::complex<double> y = gb.gain[std::size_t(mic) * bm.modes + i];
            if (bm.freq[i] <= 1000.0) {
                CHECK(x == y);  // below the corner: untouched
                ++below;
            } else {
                CHECK(std::abs(y - tilt(bm.freq[i]) * x) <= 1e-12 * std::abs(x));  // real factor, phase kept
            }
        }
    }
    CHECK(below > 0);
    // The mode nearest 2 kHz: one octave above the corner at -6 dB/oct, |gain| halved.
    CHECK(std::fabs(bm.freq[near2k] - 2000.0) < 5.0);
    for (int mic = 0; mic < 2; ++mic) {
        const std::size_t j = std::size_t(mic) * bm.modes + near2k;
        CHECK_NEAR(std::abs(gb.gain[j]) / std::abs(ga.gain[j]), 0.5, 0.005);
    }
    // Direct path: its level is the mean |transfer| over 0.75 .. 0.95 of the
    // top modal frequency, now through the tilted gains. Continuity at the
    // junction: it must scale by the tilt of the modes it joins, i.e. lie
    // between g(0.95 fMax) and g(0.75 fMax).
    const double fMax = bm.params.maxFreqHz;
    CHECK(fMax >= 4000.0);  // the modal range ends at 5 kHz
    const double ratio = double(b.directPathGain()) / double(a.directPathGain());
    std::printf("tilt -6 dB/oct at 1 kHz: direct path x%.4f (g(0.75 fMax) %.4f, g(0.95 fMax) %.4f, fMax %.0f Hz)\n",
                ratio, tilt(0.75 * fMax), tilt(0.95 * fMax), fMax);
    CHECK(a.directPathGain() > 0.0f);
    CHECK(ratio <= tilt(0.75 * fMax) * 1.001 && ratio >= tilt(0.95 * fMax) * 0.999);
    // Through the C API the value reaches the output.
    pm_engine* r0 = pm_engine_create();
    pm_engine* r1 = pm_engine_create();
    CHECK(pm_engine_set_param(r1, PM_PARAM_VOICING_RADIATION_TILT, -4.0) == 1);
    CHECK(pm_engine_prepare(r0, 48000.0, 64) == 1);
    CHECK(pm_engine_prepare(r1, 48000.0, 64) == 1);
    CHECK(render(r0) != render(r1));
    pm_engine_destroy(r0);
    pm_engine_destroy(r1);
}

// Measured values on C4 at the default and the range ends (for the report).
TEST(report_c4) {
    const pm::KeyParams r = build(pm::TableOptions{}).keys[kC4];
    const double xRef = pm::voicingRefCompression(r.hammerMass, r.strings, r.feltK, r.feltP);
    std::printf("C4 x_ref = %.4g mm, F(x_ref) = %.4g N\n", xRef * 1e3, feltForce(r, xRef));
    auto at = [](void (*set)(pm::TableOptions&, double), double v) {
        pm::TableOptions o;
        set(o, v);
        return build(o).keys[kC4];
    };
    for (double v : {0.25, 1.0, 4.0}) {
        const pm::KeyParams k = at([](pm::TableOptions& o, double x) { o.feltHardness = x; }, v);
        std::printf("hardness %g: feltK %.4g F(x_ref) %.4g\n", v, k.feltK, feltForce(k, xRef));
    }
    for (double v : {-1.0, 0.0, 1.0}) {
        const pm::KeyParams k = at([](pm::TableOptions& o, double x) { o.feltExponentOffset = x; }, v);
        std::printf("exponent %+g: feltP %.4g feltK %.4g F(x_ref) %.4g\n", v, k.feltP, k.feltK, feltForce(k, xRef));
    }
    for (double v : {0.5, 1.0, 2.0})
        std::printf("mass %g: %.4g kg\n", v,
                    at([](pm::TableOptions& o, double x) { o.hammerMassScale = x; }, v).hammerMass);
    for (double v : {0.5, 1.0, 2.0})
        std::printf("strike %g: %.4g\n", v, at([](pm::TableOptions& o, double x) { o.strikePosScale = x; }, v).strikePos);
    for (double v : {0.25, 1.0, 4.0})
        std::printf("b1 %g: %.4g\n", v, at([](pm::TableOptions& o, double x) { o.stringLossScale = x; }, v).b1);
    for (double v : {0.25, 1.0, 4.0})
        std::printf("b3 %g: %.4g\n", v, at([](pm::TableOptions& o, double x) { o.hfLossScale = x; }, v).b3);
    for (double v : {0.0, 1.0, 2.0}) {
        const pm::KeyParams k = at([](pm::TableOptions& o, double x) { o.inharmonicityScale = x; }, v);
        std::printf("B %g: B %.4g f1 %.6f f0 %.6f\n", v, k.B, k.f1, k.f0);
    }
}
