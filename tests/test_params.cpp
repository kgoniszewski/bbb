// Parameter descriptor table (plan §5.3): completeness, consistency with the
// engine and the host defaults, and set_param range handling.
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <set>
#include <string>
#include <vector>

#include "pm_api/pm_api.h"
#include "pm_api/pm_api_internal.h"
#include "test_harness.h"

namespace {

pm_param_info_t info(int32_t index) {
    pm_param_info_t d{};
    pm_param_info(index, &d);
    return d;
}

bool snakeCase(const char* s) {
    if (!s || !std::islower(static_cast<unsigned char>(s[0]))) return false;
    for (const char* p = s; *p; ++p)
        if (!(std::islower(static_cast<unsigned char>(*p)) || std::isdigit(static_cast<unsigned char>(*p)) ||
              *p == '_'))
            return false;
    return s[std::strlen(s) - 1] != '_';
}

// What get_param returns after set_param(v) for an in-range v: most values
// are stored as float; the half-pedal width is stored at least 0.01.
double expectedReadback(const pm_param_info_t& d, double v) {
    if (d.curve == PM_PARAM_CURVE_BOOLEAN) return v >= 0.5 ? 1.0 : 0.0;
    if (d.curve == PM_PARAM_CURVE_INTEGER) return std::floor(v);
    if (d.id == PM_PARAM_HALF_PEDAL_WIDTH && v < 0.01) return 0.01;
    return v;
}

double get(const pm_engine* e, int32_t id) {
    double v = std::nan("");
    CHECK(pm_engine_get_param(e, id, &v) == 1);
    return v;
}

}  // namespace

TEST(params_every_id_has_one_descriptor) {
    CHECK(pm_param_count() == PM_PARAM_VOICING_FELT_DAMPING + 1);
    std::set<int32_t> ids;
    std::set<std::string> keys;
    for (int32_t i = 0; i < pm_param_count(); ++i) {
        pm_param_info_t d{};
        CHECK(pm_param_info(i, &d) == 1);
        CHECK(d.id >= 0 && d.id <= PM_PARAM_VOICING_FELT_DAMPING);
        CHECK(ids.insert(d.id).second);
        CHECK(d.key && keys.insert(d.key).second);
        if (!snakeCase(d.key)) std::printf("  key not snake_case: %s\n", d.key);
        CHECK(snakeCase(d.key));
        CHECK(d.name && d.name[0] != '\0');
        CHECK(d.unit != nullptr);
        CHECK(d.min <= d.default_value && d.default_value <= d.max);
        CHECK(d.min <= d.engine_default && d.engine_default <= d.max);
        CHECK(d.cls == PM_PARAM_CLASS_REALTIME || d.cls == PM_PARAM_CLASS_PREPARE);
        CHECK(d.curve >= PM_PARAM_CURVE_LINEAR && d.curve <= PM_PARAM_CURVE_INTEGER);
        if (d.curve == PM_PARAM_CURVE_LOG) CHECK(d.min > 0.0);
        if (d.curve == PM_PARAM_CURVE_BOOLEAN) CHECK(d.min == 0.0 && d.max == 1.0);
        // Automatable and smoothed only make sense for real-time parameters.
        if (d.flags & (PM_PARAM_FLAG_AUTOMATABLE | PM_PARAM_FLAG_SMOOTHED)) CHECK(d.cls == PM_PARAM_CLASS_REALTIME);
        if (d.flags & PM_PARAM_FLAG_AUTOMATABLE) CHECK(!(d.flags & PM_PARAM_FLAG_HIDDEN));
        if (d.enum_count > 0) {
            CHECK(d.curve == PM_PARAM_CURVE_INTEGER && d.enum_names != nullptr);
            CHECK(d.max - d.min + 1.0 == d.enum_count);
            for (int32_t k = 0; k < d.enum_count; ++k) CHECK(snakeCase(d.enum_names[k]));
        }
    }
    for (int32_t id = 0; id <= PM_PARAM_VOICING_FELT_DAMPING; ++id) CHECK(ids.count(id) == 1);
    pm_param_info_t d{};
    CHECK(pm_param_info(-1, &d) == 0);
    CHECK(pm_param_info(pm_param_count(), &d) == 0);
    CHECK(pm_param_info(0, nullptr) == 0);
    CHECK(info(PM_PARAM_QUALITY_TIER).enum_count == 3);
    CHECK(std::strcmp(info(PM_PARAM_QUALITY_TIER).enum_names[PM_QUALITY_ECO], "eco") == 0);
}

TEST(params_key_round_trip) {
    for (int32_t i = 0; i < pm_param_count(); ++i) {
        const pm_param_info_t d = info(i);
        CHECK(pm_param_id_from_key(d.key) == d.id);
    }
    CHECK(pm_param_id_from_key("room_decay_s") == PM_PARAM_ROOM_DECAY);
    CHECK(pm_param_id_from_key("no_such_parameter") == -1);
    CHECK(pm_param_id_from_key("") == -1);
    CHECK(pm_param_id_from_key(nullptr) == -1);
}

TEST(params_set_min_max_default_read_back) {
    pm_engine* e = pm_engine_create();
    CHECK(e != nullptr);
    for (int32_t i = 0; i < pm_param_count(); ++i) {
        const pm_param_info_t d = info(i);
        for (double v : {d.min, d.max, d.default_value, d.engine_default}) {
            CHECK(pm_engine_set_param(e, d.id, v) == 1);
            const double r = get(e, d.id);
            CHECK_NEAR(r, expectedReadback(d, v), 1e-6 * (1.0 + std::fabs(v)));
        }
    }
    pm_engine_destroy(e);
}

TEST(params_out_of_range_clamped_or_rejected) {
    pm_engine* e = pm_engine_create();
    for (int32_t i = 0; i < pm_param_count(); ++i) {
        const pm_param_info_t d = info(i);
        const bool rejects = (d.flags & PM_PARAM_FLAG_REJECTS_OUT_OF_RANGE) != 0;
        const double span = d.max - d.min;
        CHECK(pm_engine_set_param(e, d.id, d.default_value) == 1);
        CHECK(pm_engine_set_param(e, d.id, std::nan("")) == 0);
        CHECK_NEAR(get(e, d.id), expectedReadback(d, d.default_value), 1e-6 * (1.0 + std::fabs(d.default_value)));
        for (double v : {d.max + span + 1.0, d.min - span - 1.0, HUGE_VAL, -HUGE_VAL}) {
            const double limit = v > d.max ? d.max : d.min;
            if (rejects) {
                CHECK(pm_engine_set_param(e, d.id, v) == 0);
                CHECK_NEAR(get(e, d.id), expectedReadback(d, d.default_value), 1e-6 * (1.0 + std::fabs(d.default_value)));
            } else {
                CHECK(pm_engine_set_param(e, d.id, v) == 1);
                CHECK_NEAR(get(e, d.id), expectedReadback(d, limit), 1e-6 * (1.0 + std::fabs(limit)));
            }
            CHECK(pm_engine_set_param(e, d.id, d.default_value) == 1);
        }
    }
    double v = 0.0;
    CHECK(pm_engine_set_param(e, -1, 0.0) == 0);
    CHECK(pm_engine_set_param(e, pm_param_count(), 0.0) == 0);
    CHECK(pm_engine_get_param(e, -1, &v) == 0);
    CHECK(pm_engine_get_param(e, pm_param_count(), &v) == 0);
    pm_engine_destroy(e);
}

TEST(params_bare_engine_defaults_after_prepare) {
    pm_engine* e = pm_engine_create();
    CHECK(pm_engine_prepare(e, 48000.0, 64) == 1);
    for (int32_t i = 0; i < pm_param_count(); ++i) {
        const pm_param_info_t d = info(i);
        // The latency default 0 means "max_frames of prepare", which prepare stores.
        const double want = d.id == PM_PARAM_CONSTANT_LATENCY_FRAMES ? 64.0 : d.engine_default;
        const double r = get(e, d.id);
        if (std::fabs(r - want) > 1e-6 * (1.0 + std::fabs(want))) std::printf("  %s: %g vs %g\n", d.key, r, want);
        CHECK_NEAR(r, want, 1e-6 * (1.0 + std::fabs(want)));
    }
    pm_engine_destroy(e);
}

TEST(params_host_engine_has_product_defaults) {
    pm_host* h = pm_host_create();
    CHECK(h != nullptr);
    const pm_engine* e = pm_host_engine(h);
    for (int32_t i = 0; i < pm_param_count(); ++i) {
        const pm_param_info_t d = info(i);
        const double r = get(e, d.id);
        if (std::fabs(r - d.default_value) > 1e-6 * (1.0 + std::fabs(d.default_value)))
            std::printf("  %s: %g vs %g\n", d.key, r, d.default_value);
        CHECK_NEAR(r, d.default_value, 1e-6 * (1.0 + std::fabs(d.default_value)));
    }
    pm_host_destroy(h);
}

namespace {

std::vector<float> renderTouch(double touch) {  // touch < 0: never set
    pm_engine* e = pm_engine_create();
    if (touch >= 0.0) CHECK(pm_engine_set_param(e, PM_PARAM_TOUCH, touch) == 1);
    CHECK(pm_engine_prepare(e, 48000.0, 64) == 1);
    for (int n : {48, 60, 64, 67}) {
        pm_event ev{};
        ev.type = PM_EVENT_NOTE_ON;
        ev.time_base = PM_TIME_SAMPLE;
        ev.note = static_cast<uint8_t>(n);
        ev.value = static_cast<uint16_t>(65535 * (n - 20) / 100);
        CHECK(pm_engine_push_event(e, &ev) == 1);
    }
    std::vector<float> out;
    float l[64], r[64];
    for (int b = 0; b < 48000 / 2 / 64; ++b) {
        pm_engine_render(e, l, r, 64, nullptr);
        out.insert(out.end(), l, l + 64);
        out.insert(out.end(), r, r + 64);
    }
    pm_engine_destroy(e);
    return out;
}

double hammerSpeed(double touch, int velocity) {
    pm_engine* e = pm_engine_create();
    CHECK(pm_engine_set_param(e, PM_PARAM_TOUCH, touch) == 1);
    const double v = pmEngineOf(e)->hammerVelocityFor(velocity / 127.0);
    pm_engine_destroy(e);
    return v;
}

}  // namespace

TEST(params_touch_medium_is_bit_identical) {
    const std::vector<float> unset = renderTouch(-1.0);
    const std::vector<float> medium = renderTouch(1.0);
    CHECK(unset.size() == medium.size());
    CHECK(std::memcmp(unset.data(), medium.data(), unset.size() * sizeof(float)) == 0);
    CHECK(renderTouch(0.0) != unset);
}

TEST(params_touch_orders_hammer_speed) {
    pm_param_info_t d{};
    CHECK(pm_param_info(PM_PARAM_TOUCH, &d) == 1);
    CHECK(std::strcmp(d.key, "touch") == 0 && d.enum_count == 3);
    CHECK(d.default_value == 1.0 && d.engine_default == 1.0);
    for (int vel : {32, 64, 100}) {
        const double light = hammerSpeed(0, vel), medium = hammerSpeed(1, vel), strong = hammerSpeed(2, vel);
        std::printf("  velocity %3d: light %.3f medium %.3f strong %.3f m/s\n", vel, light, medium, strong);
        CHECK(light > medium && medium > strong);
    }
    // Same loudest and softest hammer: only the shape of the curve changes.
    // Tolerance: the engine stores the curve as a float table.
    CHECK_NEAR(hammerSpeed(0, 127), 5.5, 1e-5);
    CHECK_NEAR(hammerSpeed(1, 127), 5.5, 1e-5);
    CHECK_NEAR(hammerSpeed(2, 127), 5.5, 1e-5);
    for (double t : {0.0, 1.0, 2.0}) {
        const double v = hammerSpeed(t, 1);
        CHECK(v >= 0.3 - 1e-6 && v < 0.35);
    }
    // Touch sits on top of a custom exponent.
    pm_engine* e = pm_engine_create();
    CHECK(pm_engine_set_param(e, PM_PARAM_VELOCITY_EXPONENT, 3.0) == 1);
    CHECK(pm_engine_set_param(e, PM_PARAM_TOUCH, 0.0) == 1);
    CHECK_NEAR(pmEngineOf(e)->hammerVelocityFor(0.5), 0.3 + 5.2 * 0.25, 1e-3);
    double ex = 0.0;
    CHECK(pm_engine_get_param(e, PM_PARAM_VELOCITY_EXPONENT, &ex) == 1 && ex == 3.0);
    pm_engine_destroy(e);
}

TEST(params_master_gain_product_default) {
    pm_host* h = pm_host_create();
    double g = 0.0;
    CHECK(pm_engine_get_param(pm_host_engine(h), PM_PARAM_MASTER_GAIN_DB, &g) == 1);
    CHECK(g == 14.0);
    pm_host_destroy(h);
    pm_engine* e = pm_engine_create();
    CHECK(pm_engine_get_param(e, PM_PARAM_MASTER_GAIN_DB, &g) == 1);
    CHECK(g == 0.0);
    pm_engine_destroy(e);
}
