#include "pm_api/pm_api.h"

#include <algorithm>
#include <cmath>
#include <new>

#include "pm_api/param_table.h"
#include "pm_api/pm_api_internal.h"
#include "pm_engine/engine.h"

struct pm_engine {
    pm::Engine engine;
    pm::VelocityCurve curve;      // as set through PM_PARAM_VELOCITY_*
    pm::VelocityCurve effective;  // curve with the touch applied (the engine reads this)
    int touch = 1;                // PM_PARAM_TOUCH
    pm::TableOptions voicing;
    pm::SoundboardConfig board;
};

pm::Engine* pmEngineOf(pm_engine* engine) noexcept { return engine ? &engine->engine : nullptr; }

namespace {
// Touch scales the exponent of x^exponent: below 1 a given key velocity gives
// a faster hammer (light), above 1 a slower one (strong); x = 0 and 1 keep vMin
// and vMax. Medium multiplies by exactly 1, so it is bit-identical.
constexpr double kTouchExponentScale[3] = {2.0 / 3.0, 1.0, 1.5};

void applyVelocityCurve(pm_engine* e) noexcept {
    e->effective = e->curve;
    e->effective.exponent = e->curve.exponent * kTouchExponentScale[e->touch];
    e->engine.setVelocityFunction(pm::velocityCurveFn, &e->effective);
}

static_assert(sizeof(pm_event) == sizeof(pm::Event), "pm_event must mirror pm::Event");
static_assert(sizeof(pm_clock) == sizeof(pm::ClockMapping), "pm_clock must mirror ClockMapping");
}  // namespace

extern "C" {

uint32_t pm_api_version(void) {
    return (PM_API_VERSION_MAJOR << 16) | (PM_API_VERSION_MINOR << 8) | PM_API_VERSION_PATCH;
}

#define PM_STR_(x) #x
#define PM_STR(x) PM_STR_(x)
// The number comes from the version macros, so the two cannot drift apart.
const char* pm_version_string(void) {
    return PM_STR(PM_API_VERSION_MAJOR) "." PM_STR(PM_API_VERSION_MINOR) "." PM_STR(PM_API_VERSION_PATCH)
        " (M4, app host, diagnostics, phase timing, speed probe, benchmark, pacing, audio-thread scenes, "
        "per-CPU statistics, headroom test, damper-response budget)";
}

pm_engine* pm_engine_create(void) {
    pm_engine* e = new (std::nothrow) pm_engine();
    if (e) applyVelocityCurve(e);
    return e;
}

void pm_engine_destroy(pm_engine* engine) { delete engine; }

int pm_engine_prepare(pm_engine* engine, double sample_rate, int32_t max_frames) {
    if (!engine) return 0;
    pm::PianoTable* table = new (std::nothrow) pm::PianoTable;
    if (!table) return 0;
    pm::buildPianoTable(*table, engine->voicing);
    engine->engine.setSoundboardConfig(engine->board);
    const bool ok = engine->engine.prepare(sample_rate, max_frames, *table);
    delete table;
    return ok ? 1 : 0;
}

int pm_engine_push_event(pm_engine* engine, const pm_event* event) {
    if (!engine || !event) return 0;
    pm::Event e;
    e.time = event->time;
    e.type = static_cast<pm::EventType>(event->type);
    e.timeBase = event->time_base == PM_TIME_HOST ? pm::TimeBase::HostTime : pm::TimeBase::SampleTime;
    e.note = event->note;
    e.value = event->value;
    return engine->engine.pushEvent(e) ? 1 : 0;
}

void pm_engine_render(pm_engine* engine, float* left, float* right, int32_t frames,
                      const pm_clock* clock) {
    if (!engine || frames <= 0) return;
    pm::ClockMapping c;
    if (clock) {
        c.hostTimeAtBufferStart = clock->host_time_at_buffer_start;
        c.hostTicksPerSample = clock->host_ticks_per_sample;
    }
    engine->engine.render(left, right, frames, clock ? &c : nullptr);
}

int pm_engine_set_param(pm_engine* engine, int32_t id, double value) {
    if (!engine) return 0;
    const pm_param_info_t* d = pm::paramDescriptor(id);
    if (!d || !pm::paramAccept(*d, &value)) return 0;
    // Range checks and clamping come from the descriptor table.
    pm::Engine& e = engine->engine;
    switch (id) {
        case PM_PARAM_MASTER_GAIN_DB: e.setMasterGainDb(static_cast<float>(value)); return 1;
        case PM_PARAM_STEREO_WIDTH: e.setStereoWidth(static_cast<float>(value)); return 1;
        case PM_PARAM_ROOM_ENABLED: e.setRoomEnabled(value >= 0.5); return 1;
        case PM_PARAM_ROOM_LEVEL_DB: e.setRoomLevelDb(static_cast<float>(value)); return 1;
        case PM_PARAM_ROOM_DECAY: e.setRoomDecay(static_cast<float>(value)); return 1;
        case PM_PARAM_ROOM_HF_RATIO: e.setRoomHfRatio(static_cast<float>(value)); return 1;
        case PM_PARAM_ROOM_SIZE: e.setRoomSize(static_cast<float>(value)); return 1;
        case PM_PARAM_ROOM_EARLY_LATE: e.setRoomEarlyLate(static_cast<float>(value)); return 1;
        case PM_PARAM_TREBLE_DB: e.setTrebleDb(static_cast<float>(value)); return 1;
        case PM_PARAM_BASS_DB: e.setBassDb(static_cast<float>(value)); return 1;
        case PM_PARAM_MID_DB: e.setMidDb(static_cast<float>(value)); return 1;
        case PM_PARAM_SCHEDULING_MODE:
            e.setSchedulingMode(value >= 0.5 ? pm::SchedulingMode::ConstantLatency
                                             : pm::SchedulingMode::LowestLatency);
            return 1;
        case PM_PARAM_CONSTANT_LATENCY_FRAMES:
            e.setConstantLatencyFrames(static_cast<int>(value));
            return 1;
        case PM_PARAM_CULL_THRESHOLD_DB:
            e.setCulling(e.cullingEnabled(), static_cast<float>(value));
            return 1;
        case PM_PARAM_CULLING_ENABLED:
            e.setCulling(value >= 0.5, e.cullThresholdDb());
            return 1;
        case PM_PARAM_VELOCITY_MIN:
        case PM_PARAM_VELOCITY_MAX:
        case PM_PARAM_VELOCITY_EXPONENT: {
            if (id == PM_PARAM_VELOCITY_MIN) engine->curve.vMin = value;
            else if (id == PM_PARAM_VELOCITY_MAX) engine->curve.vMax = value;
            else engine->curve.exponent = value;
            applyVelocityCurve(engine);
            return 1;
        }
        case PM_PARAM_TOUCH:
            engine->touch = std::clamp(static_cast<int>(std::lround(value)), 0, 2);
            applyVelocityCurve(engine);
            return 1;
        case PM_PARAM_A4_HZ: engine->voicing.a4Hz = value; return 1;
        case PM_PARAM_TUNING_STRETCH: engine->voicing.stretch = value; return 1;
        case PM_PARAM_EQUAL_TEMPERAMENT: engine->voicing.equalTemperament = value >= 0.5; return 1;
        case PM_PARAM_UNISON_WIDTH: engine->voicing.unisonWidth = value; return 1;
        case PM_PARAM_VOICING_FELT_HARDNESS: engine->voicing.feltHardness = value; return 1;
        case PM_PARAM_VOICING_FELT_EXPONENT: engine->voicing.feltExponentOffset = value; return 1;
        case PM_PARAM_VOICING_HAMMER_MASS: engine->voicing.hammerMassScale = value; return 1;
        case PM_PARAM_VOICING_STRIKE_POSITION: engine->voicing.strikePosScale = value; return 1;
        case PM_PARAM_VOICING_STRING_LOSS: engine->voicing.stringLossScale = value; return 1;
        case PM_PARAM_VOICING_HF_LOSS: engine->voicing.hfLossScale = value; return 1;
        case PM_PARAM_VOICING_INHARMONICITY: engine->voicing.inharmonicityScale = value; return 1;
        case PM_PARAM_VOICING_BOARD_DAMPING: engine->board.dampingScale = value; return 1;
        case PM_PARAM_VOICING_RADIATION_TILT: engine->board.radiationTiltDbPerOct = value; return 1;
        case PM_PARAM_VOICING_RADIATION_CORNER_HZ: engine->board.radiationTiltCornerHz = value; return 1;
        case PM_PARAM_VOICING_FELT_DAMPING: engine->voicing.feltDampingScale = value; return 1;
        case PM_PARAM_SOUNDBOARD_ENABLED: engine->board.enabled = value >= 0.5; return 1;
        case PM_PARAM_SYMPATHETIC: engine->board.sympathetic = value; return 1;
        case PM_PARAM_KNOCK_LEVEL: e.setNoiseLevels(static_cast<float>(value), e.damperNoiseLevel()); return 1;
        case PM_PARAM_DAMPER_NOISE_LEVEL: e.setNoiseLevels(e.knockLevel(), static_cast<float>(value)); return 1;
        case PM_PARAM_HALF_PEDAL_P0_TREBLE:
        case PM_PARAM_HALF_PEDAL_P0_BASS:
        case PM_PARAM_HALF_PEDAL_WIDTH: {
            pm::HalfPedalCurve c = e.halfPedalCurve();
            if (id == PM_PARAM_HALF_PEDAL_P0_TREBLE) c.p0Treble = static_cast<float>(value);
            else if (id == PM_PARAM_HALF_PEDAL_P0_BASS) c.p0Bass = static_cast<float>(value);
            else c.width = static_cast<float>(value);
            e.setHalfPedalCurve(c);
            return 1;
        }
        case PM_PARAM_DAMPED_CULL_OFFSET_DB: e.setDampedCullOffsetDb(static_cast<float>(value)); return 1;
        case PM_PARAM_QUALITY_TIER: {
            const int t = static_cast<int>(value);
            engine->board.tier = static_cast<pm::QualityTier>(t);
            return 1;
        }
        case PM_PARAM_LOOP_CUTOFF_HZ: engine->board.loopCutoffHz = value; return 1;
        default: return 0;
    }
}

int pm_engine_get_param(const pm_engine* engine, int32_t id, double* value) {
    if (!engine || !value) return 0;
    const pm::Engine& e = engine->engine;
    switch (id) {
        case PM_PARAM_MASTER_GAIN_DB: *value = e.masterGainDb(); return 1;
        case PM_PARAM_STEREO_WIDTH: *value = e.stereoWidth(); return 1;
        case PM_PARAM_ROOM_ENABLED: *value = e.roomEnabled() ? 1.0 : 0.0; return 1;
        case PM_PARAM_ROOM_LEVEL_DB: *value = e.roomLevelDb(); return 1;
        case PM_PARAM_ROOM_DECAY: *value = e.roomDecay(); return 1;
        case PM_PARAM_ROOM_HF_RATIO: *value = e.roomHfRatio(); return 1;
        case PM_PARAM_ROOM_SIZE: *value = e.roomSize(); return 1;
        case PM_PARAM_ROOM_EARLY_LATE: *value = e.roomEarlyLate(); return 1;
        case PM_PARAM_TREBLE_DB: *value = e.trebleDb(); return 1;
        case PM_PARAM_BASS_DB: *value = e.bassDb(); return 1;
        case PM_PARAM_MID_DB: *value = e.midDb(); return 1;
        case PM_PARAM_SCHEDULING_MODE: *value = static_cast<double>(e.schedulingMode()); return 1;
        case PM_PARAM_CONSTANT_LATENCY_FRAMES: *value = e.constantLatencyFrames(); return 1;
        case PM_PARAM_CULL_THRESHOLD_DB: *value = e.cullThresholdDb(); return 1;
        case PM_PARAM_CULLING_ENABLED: *value = e.cullingEnabled() ? 1.0 : 0.0; return 1;
        case PM_PARAM_VELOCITY_MIN: *value = engine->curve.vMin; return 1;
        case PM_PARAM_VELOCITY_MAX: *value = engine->curve.vMax; return 1;
        case PM_PARAM_VELOCITY_EXPONENT: *value = engine->curve.exponent; return 1;
        case PM_PARAM_TOUCH: *value = engine->touch; return 1;
        case PM_PARAM_A4_HZ: *value = engine->voicing.a4Hz; return 1;
        case PM_PARAM_TUNING_STRETCH: *value = engine->voicing.stretch; return 1;
        case PM_PARAM_EQUAL_TEMPERAMENT: *value = engine->voicing.equalTemperament ? 1.0 : 0.0; return 1;
        case PM_PARAM_UNISON_WIDTH: *value = engine->voicing.unisonWidth; return 1;
        case PM_PARAM_VOICING_FELT_HARDNESS: *value = engine->voicing.feltHardness; return 1;
        case PM_PARAM_VOICING_FELT_EXPONENT: *value = engine->voicing.feltExponentOffset; return 1;
        case PM_PARAM_VOICING_HAMMER_MASS: *value = engine->voicing.hammerMassScale; return 1;
        case PM_PARAM_VOICING_STRIKE_POSITION: *value = engine->voicing.strikePosScale; return 1;
        case PM_PARAM_VOICING_STRING_LOSS: *value = engine->voicing.stringLossScale; return 1;
        case PM_PARAM_VOICING_HF_LOSS: *value = engine->voicing.hfLossScale; return 1;
        case PM_PARAM_VOICING_INHARMONICITY: *value = engine->voicing.inharmonicityScale; return 1;
        case PM_PARAM_VOICING_BOARD_DAMPING: *value = engine->board.dampingScale; return 1;
        case PM_PARAM_VOICING_RADIATION_TILT: *value = engine->board.radiationTiltDbPerOct; return 1;
        case PM_PARAM_VOICING_RADIATION_CORNER_HZ: *value = engine->board.radiationTiltCornerHz; return 1;
        case PM_PARAM_VOICING_FELT_DAMPING: *value = engine->voicing.feltDampingScale; return 1;
        case PM_PARAM_SOUNDBOARD_ENABLED: *value = engine->board.enabled ? 1.0 : 0.0; return 1;
        case PM_PARAM_SYMPATHETIC: *value = engine->board.sympathetic; return 1;
        case PM_PARAM_QUALITY_TIER: *value = static_cast<double>(engine->board.tier); return 1;
        case PM_PARAM_LOOP_CUTOFF_HZ: *value = engine->board.loopCutoffHz; return 1;
        case PM_PARAM_KNOCK_LEVEL: *value = e.knockLevel(); return 1;
        case PM_PARAM_DAMPER_NOISE_LEVEL: *value = e.damperNoiseLevel(); return 1;
        case PM_PARAM_HALF_PEDAL_P0_TREBLE: *value = e.halfPedalCurve().p0Treble; return 1;
        case PM_PARAM_HALF_PEDAL_P0_BASS: *value = e.halfPedalCurve().p0Bass; return 1;
        case PM_PARAM_HALF_PEDAL_WIDTH: *value = e.halfPedalCurve().width; return 1;
        case PM_PARAM_DAMPED_CULL_OFFSET_DB: *value = e.dampedCullOffsetDb(); return 1;
        default: return 0;
    }
}

int32_t pm_engine_describe(const pm_engine* engine, char* buffer, int32_t size) {
    if (!buffer || size <= 0) return 0;
    buffer[0] = 0;
    if (!engine) return 0;
    return engine->engine.describe(buffer, size);
}

void pm_engine_get_stats(const pm_engine* engine, pm_stats* stats) {
    if (!engine || !stats) return;
    const pm::EngineStats s = engine->engine.stats();
    stats->total_modes = s.totalModes;
    stats->awake_groups = s.awakeGroups;
    stats->awake_modes = s.awakeModes;
    stats->active_hammers = s.activeHammers;
    stats->peak_active_hammers = s.peakActiveHammers;
    stats->dropped_events = s.droppedEvents;
    stats->deferred_strikes = s.deferredStrikes;
    stats->late_events = s.lateEvents;
    stats->limited_samples = s.limitedSamples;
    stats->peak_pre_limiter = s.peakPreLimiter;
}

}  // extern "C"
