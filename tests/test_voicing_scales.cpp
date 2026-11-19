// Global voicing scales (API 0.19, params 30-36): defaults leave the table and
// the output untouched, each scale moves its target, the C API clamps.
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "pm_api/pm_api.h"
#include "pm_voicing/piano_table.h"
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
                        PM_PARAM_VOICING_INHARMONICITY};

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
    const pm::PianoTable a = build(pm::TableOptions{}), b = build(explicitDefaults);
    for (int i = 0; i < pm::kNumKeys; ++i) CHECK(sameKey(a.keys[i], b.keys[i]));
    // C4 is an anchor row: the plan §2.1 values come through unchanged.
    const pm::KeyParams& k = a.keys[kC4];
    CHECK_NEAR(k.hammerMass, 9e-3, 1e-15);
    CHECK_NEAR(k.feltK / 4.5e9, 1.0, 1e-12);
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
        {PM_PARAM_VOICING_INHARMONICITY, 0.0, 2.0, 1.0}};
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
