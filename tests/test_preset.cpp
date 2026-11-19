// Presets (plan §5.3): canonical JSON round trip, equivalence with direct
// set_param calls, parser robustness, migration and content tolerance.
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "pm_api/pm_api.h"
#include "test_harness.h"

namespace {

struct Lcg {
    std::uint64_t s;
    std::uint32_t next() {
        s = s * 6364136223846793005ull + 1442695040888963407ull;
        return static_cast<std::uint32_t>(s >> 33);
    }
    double uniform() { return next() / 2147483648.0; }  // [0, 1)
};

pm_param_info_t info(int32_t index) {
    pm_param_info_t d{};
    pm_param_info(index, &d);
    return d;
}

double get(const pm_engine* e, int32_t id) {
    double v = std::nan("");
    pm_engine_get_param(e, id, &v);
    return v;
}

std::string write(const pm_engine* e, int32_t flags, const char* name = "test") {
    const size_t n = pm_preset_write(e, name, flags, nullptr, 0);
    std::string s(n, '\0');
    std::vector<char> buf(n + 1);
    CHECK(pm_preset_write(e, name, flags, buf.data(), buf.size()) == n);
    s.assign(buf.data(), n);
    return s;
}

pm_preset_result_t read(pm_engine* e, const std::string& json, int32_t flags = 0) {
    pm_preset_result_t r{};
    // An exact-size copy: the reader must not rely on a terminating NUL.
    std::vector<char> exact(json.begin(), json.end());
    pm_preset_read(e, exact.empty() ? nullptr : exact.data(), exact.size(), flags, &r);
    if (exact.empty()) pm_preset_read(e, "", 0, flags, &r);
    return r;
}

int32_t visibleCount() {
    int32_t n = 0;
    for (int32_t i = 0; i < pm_param_count(); ++i)
        if (!(info(i).flags & PM_PARAM_FLAG_HIDDEN)) ++n;
    return n;
}

void randomize(pm_engine* e, Lcg& rng) {
    for (int32_t i = 0; i < pm_param_count(); ++i) {
        const pm_param_info_t d = info(i);
        const double u = rng.uniform();
        double v;
        if (d.curve == PM_PARAM_CURVE_BOOLEAN) v = u < 0.5 ? 0.0 : 1.0;
        else if (d.curve == PM_PARAM_CURVE_INTEGER) v = std::floor(d.min + u * (d.max - d.min + 1.0));
        else if (d.curve == PM_PARAM_CURVE_LOG) v = d.min * std::pow(d.max / d.min, u);
        else v = d.min + u * (d.max - d.min);
        if (v > d.max) v = d.max;
        CHECK(pm_engine_set_param(e, d.id, v) == 1);
    }
}

// A short scene with a note, a chord, the sustain pedal and a release.
std::vector<float> render(pm_engine* e) {
    const struct { uint64_t t; uint8_t type, note; uint16_t value; } ev[] = {
        {10, PM_EVENT_NOTE_ON, 60, 40000}, {400, PM_EVENT_SUSTAIN, 0, 65535}, {900, PM_EVENT_NOTE_ON, 64, 52000},
        {900, PM_EVENT_NOTE_ON, 67, 30000}, {4000, PM_EVENT_NOTE_OFF, 60, 0}, {6000, PM_EVENT_SUSTAIN, 0, 0}};
    for (const auto& x : ev) {
        pm_event p{};
        p.time = x.t;
        p.type = x.type;
        p.time_base = PM_TIME_SAMPLE;
        p.note = x.note;
        p.value = x.value;
        CHECK(pm_engine_push_event(e, &p) == 1);
    }
    std::vector<float> out;
    float l[64], r[64];
    for (int b = 0; b < 200; ++b) {
        pm_engine_render(e, l, r, 64, nullptr);
        out.insert(out.end(), l, l + 64);
        out.insert(out.end(), r, r + 64);
    }
    return out;
}

const char* kHeader = "{\"format\":\"pm-preset\",\"version\":1,";

}  // namespace

TEST(preset_round_trip_is_canonical) {
    Lcg rng{12345};
    for (int trial = 0; trial < 20; ++trial) {
        pm_engine* a = pm_engine_create();
        pm_engine* b = pm_engine_create();
        randomize(a, rng);
        const std::string j1 = write(a, PM_PRESET_WRITE_FULL_STATE);
        const pm_preset_result_t r = read(b, j1);
        CHECK(r.ok == 1 && r.version == 1 && r.error[0] == '\0');
        CHECK(r.applied == pm_param_count() && r.clamped == 0 && r.unknown == 0 && r.wrong_type == 0 &&
              r.missing == 0);
        CHECK(std::strcmp(r.name, "test") == 0);
        const std::string j2 = write(b, PM_PRESET_WRITE_FULL_STATE);
        CHECK(j1 == j2);
        for (int32_t i = 0; i < pm_param_count(); ++i) {
            const int32_t id = info(i).id;
            CHECK(get(a, id) == get(b, id));
        }
        if (trial == 0) std::printf("  %zu bytes: %.160s...\n", j1.size(), j1.c_str());
        pm_engine_destroy(a);
        pm_engine_destroy(b);
    }
    // A preset leaves out the hidden parameters; a full state has them.
    pm_engine* e = pm_engine_create();
    CHECK(write(e, 0).find("\"culling_enabled\"") == std::string::npos);
    CHECK(write(e, PM_PRESET_WRITE_FULL_STATE).find("\"culling_enabled\":1") != std::string::npos);
    CHECK(write(e, 0).find("\"room_enabled\":0,") != std::string::npos);
    // snprintf-style buffer contract.
    const std::string full = write(e, 0);
    char small[10];
    CHECK(pm_preset_write(e, "test", 0, small, sizeof small) == full.size());
    CHECK(std::strlen(small) == 9 && full.compare(0, 9, small) == 0);
    CHECK(pm_preset_write(nullptr, "x", 0, small, sizeof small) == 0);
    pm_engine_destroy(e);
}

TEST(preset_name_escapes_round_trip) {
    pm_engine* e = pm_engine_create();
    const char* name = "a\"b\\c\nd\x01 \xC3\xA9";
    pm_preset_result_t r = read(e, write(e, 0, name));
    CHECK(r.ok == 1 && std::strcmp(r.name, name) == 0);
    r = read(e, std::string(kHeader) + "\"name\":\"\\ud83c\\udfb9\\u00e9\\/\"}");
    CHECK(r.ok == 1 && std::strcmp(r.name, "\xF0\x9F\x8E\xB9\xC3\xA9/") == 0);
    // Truncated to the result buffer at a character boundary.
    std::string longName(62, 'x');
    longName += "\xC3\xA9\xC3\xA9";
    r = read(e, write(e, 0, longName.c_str()));
    CHECK(r.ok == 1 && std::strlen(r.name) == 62);
    pm_engine_destroy(e);
}

TEST(preset_apply_equals_direct_set_param) {
    // The reference values, including PREPARE-class ones (unison width, a4,
    // sympathetic, quality tier).
    pm_engine* ref = pm_engine_create();
    CHECK(pm_engine_set_param(ref, PM_PARAM_MASTER_GAIN_DB, -6.0) == 1);
    CHECK(pm_engine_set_param(ref, PM_PARAM_ROOM_ENABLED, 1.0) == 1);
    CHECK(pm_engine_set_param(ref, PM_PARAM_ROOM_DECAY, 2.5) == 1);
    CHECK(pm_engine_set_param(ref, PM_PARAM_TREBLE_DB, -1.5) == 1);
    CHECK(pm_engine_set_param(ref, PM_PARAM_UNISON_WIDTH, 1.7) == 1);
    CHECK(pm_engine_set_param(ref, PM_PARAM_A4_HZ, 442.0) == 1);
    CHECK(pm_engine_set_param(ref, PM_PARAM_SYMPATHETIC, 0.6) == 1);
    CHECK(pm_engine_set_param(ref, PM_PARAM_QUALITY_TIER, 1.0) == 1);
    const std::string preset = write(ref, 0);

    // Direct: the same set_param calls the preset makes (every non-hidden
    // parameter, table order), then the re-prepare.
    pm_engine* a = pm_engine_create();
    CHECK(pm_engine_prepare(a, 48000.0, 64) == 1);
    for (int32_t i = 0; i < pm_param_count(); ++i) {
        const pm_param_info_t d = info(i);
        if (!(d.flags & PM_PARAM_FLAG_HIDDEN)) CHECK(pm_engine_set_param(a, d.id, get(ref, d.id)) == 1);
    }
    CHECK(pm_engine_prepare(a, 48000.0, 64) == 1);

    pm_engine* b = pm_engine_create();
    CHECK(pm_engine_prepare(b, 48000.0, 64) == 1);
    const pm_preset_result_t r = read(b, preset, PM_PRESET_READ_REPREPARE);
    CHECK(r.ok == 1 && r.reprepared == 1 && r.needs_prepare == 0 && r.applied == visibleCount());

    const std::vector<float> ya = render(a), yb = render(b);
    float peak = 0.0f;
    for (float v : ya) peak = std::fmax(peak, std::fabs(v));
    CHECK(peak > 1e-3f);
    CHECK(ya.size() == yb.size() && std::memcmp(ya.data(), yb.data(), ya.size() * sizeof(float)) == 0);
    std::printf("  %zu samples bit-identical, peak %.4f\n", ya.size(), peak);

    // Without the flag the change is reported, not acted on; an unchanged
    // PREPARE value does not ask for a prepare.
    pm_engine* c = pm_engine_create();
    CHECK(pm_engine_prepare(c, 48000.0, 64) == 1);
    pm_preset_result_t rc = read(c, preset);
    CHECK(rc.ok == 1 && rc.needs_prepare == 1 && rc.reprepared == 0);
    rc = read(c, preset);
    CHECK(rc.ok == 1 && rc.needs_prepare == 0);
    pm_engine_destroy(ref);
    pm_engine_destroy(a);
    pm_engine_destroy(b);
    pm_engine_destroy(c);
}

TEST(preset_host_read_reprepares) {
    pm_host* h = pm_host_create();
    CHECK(pm_host_prepare(h, 44100.0, 128) == 1);
    const std::string json = std::string(kHeader) + "\"params\":{\"unison_width\":2,\"treble_db\":-4}}";
    pm_preset_result_t r{};
    CHECK(pm_host_preset_read(h, json.data(), json.size(), PM_PRESET_READ_REPREPARE, &r) == 1);
    CHECK(r.reprepared == 1 && r.applied == 2 && r.missing == visibleCount() - 2);
    CHECK(pm_host_is_prepared(h) == 1 && pm_host_sample_rate(h) == 44100.0);
    CHECK(get(pm_host_engine(h), PM_PARAM_UNISON_WIDTH) == 2.0);
    char buf[4096];
    CHECK(pm_host_preset_write(h, "h", 0, buf, sizeof buf) < sizeof buf);
    CHECK(std::strstr(buf, "\"unison_width\":2,") != nullptr);
    pm_host_destroy(h);
}

TEST(preset_malformed_inputs_rejected) {
    const std::string deep = std::string(kHeader) + "\"x\":" + std::string(40, '[') + std::string(40, ']') + "}";
    const std::string syntax[] = {
        "", "   ", "{", std::string(kHeader), "{\"a\":1,}", "{\"a\":[1,2,]}", "{\"a\":\"\\x\"}",
        "{\"a\":\"\\u12\"}", "{\"a\":\"\\u12g4\"}", "{\"a\":\"\\ud800\"}", "{\"a\":\"\\udc00\"}",
        "{\"a\":\"\\ud800\\u0041\"}", "{\"a\":\"\\u0000\"}", "{\"a\":1e999}", "{\"a\":-1e400}", "{\"a\":01}",
        "{\"a\":1.}", "{\"a\":.5}", "{\"a\":+1}", "{\"a\":1e}", "{\"a\":-}", "{\"a\":1} x", "{'a':1}",
        "{\"a\":tru}", "nul", "{\"a\" 1}", "{\"a\":1 \"b\":2}", "{1:2}", "{\"a\":\"x", "{\"a\":\"\xff\"}",
        "{\"a\":\"\xC0\xAF\"}", "{\"a\":\"\xED\xA0\x80\"}", "{\"a\":\"\t\"}", std::string("{\"a\":\0}", 7),
        std::string("{\"a\":\"x\0y\"}", 11), std::string("{\"a\":1}\0", 8), deep};
    const std::string content[] = {
        "[]", "1", "\"pm-preset\"", "{}", "{\"format\":\"other\",\"version\":1}", "{\"format\":\"pm-preset\"}",
        "{\"format\":\"pm-preset\",\"version\":\"1\"}", "{\"format\":\"pm-preset\",\"version\":-1}",
        "{\"format\":\"pm-preset\",\"version\":1.5}", std::string(kHeader) + "\"params\":[]}",
        std::string(kHeader) + "\"params\":5}", "{\"format\":\"pm-preset\",\"version\":0,\"params\":{}}"};
    pm_engine* e = pm_engine_create();
    CHECK(pm_engine_set_param(e, PM_PARAM_MASTER_GAIN_DB, -7.0) == 1);
    const std::string before = write(e, PM_PRESET_WRITE_FULL_STATE);
    int n = 0;
    for (const std::string& s : syntax) {
        const pm_preset_result_t r = read(e, s);
        if (r.ok || r.error[0] == '\0' || r.error_offset < 0 || r.error_offset > static_cast<int64_t>(s.size()))
            std::printf("  accepted or no offset: case %d (%s)\n", n, r.error);
        CHECK(r.ok == 0 && r.error[0] != '\0' && r.error_offset >= 0 &&
              r.error_offset <= static_cast<int64_t>(s.size()));
        ++n;
    }
    for (const std::string& s : content) {
        const pm_preset_result_t r = read(e, s);
        CHECK(r.ok == 0 && r.error[0] != '\0');
        ++n;
    }
    CHECK(pm_preset_read(e, nullptr, 10, 0, nullptr) == 0);
    CHECK(pm_preset_read(nullptr, "{}", 2, 0, nullptr) == 0);
    // Nothing was applied by a rejected document.
    CHECK(write(e, PM_PRESET_WRITE_FULL_STATE) == before);
    // The nesting limit is a limit, not a bug: 30 levels are fine.
    const pm_preset_result_t ok = read(e, std::string(kHeader) + "\"x\":" + std::string(30, '[') +
                                              std::string(30, ']') + "}");
    CHECK(ok.ok == 1 && ok.unknown == 1);
    std::printf("  %d malformed documents rejected\n", n);
    pm_engine_destroy(e);
}

TEST(preset_fuzz_never_crashes) {
    pm_engine* src = pm_engine_create();
    Lcg rng{777};
    randomize(src, rng);
    const std::string base = write(src, PM_PRESET_WRITE_FULL_STATE, "fuzz \xC3\xA9\\n");
    pm_engine* e = pm_engine_create();
    int accepted = 0;
    for (int it = 0; it < 2000; ++it) {
        std::string s = base;
        const int edits = 1 + static_cast<int>(rng.next() % 4);
        for (int k = 0; k < edits && !s.empty(); ++k) {
            const size_t pos = rng.next() % s.size();
            switch (rng.next() % 6) {
                case 0: s[pos] = static_cast<char>(rng.next() & 0xFF); break;
                case 1: s.erase(pos, 1 + rng.next() % 8); break;
                case 2: s.insert(pos, 1, "{}[]\",:\\0e-.u"[rng.next() % 13]); break;
                case 3: s.insert(pos, s.substr(rng.next() % s.size(), 1 + rng.next() % 16)); break;
                case 4: s.resize(pos); break;
                default: s[pos] = "0123456789"[rng.next() % 10]; break;
            }
        }
        const pm_preset_result_t r = read(e, s);
        if (r.ok) ++accepted;
        else CHECK(r.error[0] != '\0');
    }
    std::printf("  2000 mutations: %d accepted, %d rejected\n", accepted, 2000 - accepted);
    pm_engine_destroy(src);
    pm_engine_destroy(e);
}

TEST(preset_migrates_v0_and_refuses_newer) {
    const std::string v0 =
        "{\"format\":\"pm-preset\",\"version\":0,\"name\":\"old\",\"params\":[{\"id\":0,\"value\":-6},"
        "{\"id\":12,\"value\":2.5},{\"id\":9999,\"value\":1},{\"value\":3},7]}";
    pm_engine* a = pm_engine_create();
    const pm_preset_result_t r = read(a, v0);
    CHECK(r.ok == 1 && r.version == 0 && std::strcmp(r.name, "old") == 0);
    CHECK(r.applied == 2 && r.unknown == 3 && r.needs_prepare == 1);
    CHECK(get(a, PM_PARAM_MASTER_GAIN_DB) == -6.0 && get(a, PM_PARAM_UNISON_WIDTH) == 2.5);
    // The same content as v1 gives the same engine state.
    pm_engine* b = pm_engine_create();
    const pm_preset_result_t r1 =
        read(b, "{\"format\":\"pm-preset\",\"version\":1,\"name\":\"old\",\"params\":"
                "{\"master_gain_db\":-6,\"unison_width\":2.5}}");
    CHECK(r1.ok == 1 && r1.version == 1 && r1.applied == 2 && r1.unknown == 0);
    CHECK(write(a, PM_PRESET_WRITE_FULL_STATE) == write(b, PM_PRESET_WRITE_FULL_STATE));
    // v0 with params missing entirely is fine (all missing).
    CHECK(read(a, "{\"format\":\"pm-preset\",\"version\":0}").missing == visibleCount());

    const pm_preset_result_t rn = read(a, "{\"format\":\"pm-preset\",\"version\":2,\"params\":{}}");
    CHECK(rn.ok == 0 && std::strstr(rn.error, "newer") != nullptr);
    std::printf("  newer: %s\n", rn.error);
    pm_engine_destroy(a);
    pm_engine_destroy(b);
}

TEST(preset_content_tolerance) {
    pm_engine* e = pm_engine_create();
    CHECK(pm_engine_set_param(e, PM_PARAM_STEREO_WIDTH, 0.8) == 1);
    const std::string doc = std::string(kHeader) +
                            "\"name\":\"t\",\"extra\":{\"x\":[1,2]},\"params\":{\"master_gain_db\":100,"
                            "\"stereo_width\":\"wide\",\"bogus\":1,\"room_enabled\":true,\"a4_hz\":1000,"
                            "\"quality_tier\":7,\"treble_db\":null,\"room_size\":0.1,\"room_size\":1.5}}";
    const pm_preset_result_t r = read(e, doc);
    std::printf("  warnings: %s\n", r.warnings);
    CHECK(r.ok == 1 && r.unknown == 2 && r.wrong_type == 2 && r.clamped == 4 && r.applied == 5);
    // stereo_width and treble_db are present but of the wrong type: not missing, kept.
    CHECK(r.missing == visibleCount() - 7);
    CHECK(r.needs_prepare == 1);
    CHECK(get(e, PM_PARAM_MASTER_GAIN_DB) == 24.0 && get(e, PM_PARAM_A4_HZ) == 600.0);
    CHECK(get(e, PM_PARAM_QUALITY_TIER) == 2.0 && get(e, PM_PARAM_ROOM_ENABLED) == 1.0);
    CHECK(get(e, PM_PARAM_ROOM_SIZE) == 1.5);  // the last duplicate wins
    CHECK(std::fabs(get(e, PM_PARAM_STEREO_WIDTH) - 0.8) < 1e-6);
    CHECK(std::strstr(r.warnings, "bogus") && std::strstr(r.warnings, "extra") && std::strstr(r.warnings, "wide") == nullptr);

    // Missing parameters keep their value, or get the product default.
    CHECK(pm_engine_set_param(e, PM_PARAM_MASTER_GAIN_DB, -20.0) == 1);
    const std::string empty = std::string(kHeader) + "\"params\":{}}";
    CHECK(read(e, empty).ok == 1 && get(e, PM_PARAM_MASTER_GAIN_DB) == -20.0);
    const pm_preset_result_t rd = read(e, empty, PM_PRESET_READ_MISSING_TO_DEFAULT);
    CHECK(rd.ok == 1 && rd.applied == visibleCount() && rd.missing == visibleCount());
    // The product master volume is +14 dB since API 0.20.
    CHECK(get(e, PM_PARAM_MASTER_GAIN_DB) == 14.0 && get(e, PM_PARAM_ROOM_ENABLED) == 1.0);
    pm_engine_destroy(e);
}
