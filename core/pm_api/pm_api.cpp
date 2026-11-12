#include "pm_api/pm_api.h"

#include <new>

#include "pm_api/pm_api_internal.h"
#include "pm_engine/engine.h"

struct pm_engine {
    pm::Engine engine;
    pm::VelocityCurve curve;
    pm::TableOptions voicing;
    pm::SoundboardConfig board;
};

pm::Engine* pmEngineOf(pm_engine* engine) noexcept { return engine ? &engine->engine : nullptr; }

namespace {
static_assert(sizeof(pm_event) == sizeof(pm::Event), "pm_event must mirror pm::Event");
static_assert(sizeof(pm_clock) == sizeof(pm::ClockMapping), "pm_clock must mirror ClockMapping");
}  // namespace

extern "C" {

uint32_t pm_api_version(void) {
    return (PM_API_VERSION_MAJOR << 16) | (PM_API_VERSION_MINOR << 8) | PM_API_VERSION_PATCH;
}

const char* pm_version_string(void) { return "0.9.0 (M4, app host, diagnostics, phase timing, speed probe, benchmark)"; }

pm_engine* pm_engine_create(void) {
    pm_engine* e = new (std::nothrow) pm_engine();
    if (e) e->engine.setVelocityFunction(pm::velocityCurveFn, &e->curve);
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
    pm::Engine& e = engine->engine;
    switch (id) {
        case PM_PARAM_MASTER_GAIN_DB: e.setMasterGainDb(static_cast<float>(value)); return 1;
        case PM_PARAM_STEREO_WIDTH: e.setStereoWidth(static_cast<float>(value)); return 1;
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
            e.setVelocityFunction(pm::velocityCurveFn, &engine->curve);
            return 1;
        }
        case PM_PARAM_A4_HZ:
            if (!(value >= 300.0 && value <= 600.0)) return 0;
            engine->voicing.a4Hz = value;
            return 1;
        case PM_PARAM_TUNING_STRETCH: engine->voicing.stretch = value < 0.0 ? 0.0 : (value > 1.5 ? 1.5 : value); return 1;
        case PM_PARAM_EQUAL_TEMPERAMENT: engine->voicing.equalTemperament = value >= 0.5; return 1;
        case PM_PARAM_UNISON_WIDTH: engine->voicing.unisonWidth = value < 0.0 ? 0.0 : (value > 5.0 ? 5.0 : value); return 1;
        case PM_PARAM_SOUNDBOARD_ENABLED: engine->board.enabled = value >= 0.5; return 1;
        case PM_PARAM_SYMPATHETIC: engine->board.sympathetic = value < 0.0 ? 0.0 : (value > 1.5 ? 1.5 : value); return 1;
        case PM_PARAM_KNOCK_LEVEL: e.setNoiseLevels(static_cast<float>(value), e.damperNoiseLevel()); return 1;
        case PM_PARAM_DAMPER_NOISE_LEVEL: e.setNoiseLevels(e.knockLevel(), static_cast<float>(value)); return 1;
        case PM_PARAM_HALF_PEDAL_P0_TREBLE:
        case PM_PARAM_HALF_PEDAL_P0_BASS:
        case PM_PARAM_HALF_PEDAL_WIDTH: {
            if (!(value >= 0.0 && value <= 1.0)) return 0;
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
            if (!(value >= 0.0) || t > PM_QUALITY_ECO) return 0;
            engine->board.tier = static_cast<pm::QualityTier>(t);
            return 1;
        }
        case PM_PARAM_LOOP_CUTOFF_HZ: engine->board.loopCutoffHz = value > 0.0 ? value : 0.0; return 1;
        default: return 0;
    }
}

int pm_engine_get_param(const pm_engine* engine, int32_t id, double* value) {
    if (!engine || !value) return 0;
    const pm::Engine& e = engine->engine;
    switch (id) {
        case PM_PARAM_MASTER_GAIN_DB: *value = e.masterGainDb(); return 1;
        case PM_PARAM_STEREO_WIDTH: *value = e.stereoWidth(); return 1;
        case PM_PARAM_SCHEDULING_MODE: *value = static_cast<double>(e.schedulingMode()); return 1;
        case PM_PARAM_CONSTANT_LATENCY_FRAMES: *value = e.constantLatencyFrames(); return 1;
        case PM_PARAM_CULL_THRESHOLD_DB: *value = e.cullThresholdDb(); return 1;
        case PM_PARAM_CULLING_ENABLED: *value = e.cullingEnabled() ? 1.0 : 0.0; return 1;
        case PM_PARAM_VELOCITY_MIN: *value = engine->curve.vMin; return 1;
        case PM_PARAM_VELOCITY_MAX: *value = engine->curve.vMax; return 1;
        case PM_PARAM_VELOCITY_EXPONENT: *value = engine->curve.exponent; return 1;
        case PM_PARAM_A4_HZ: *value = engine->voicing.a4Hz; return 1;
        case PM_PARAM_TUNING_STRETCH: *value = engine->voicing.stretch; return 1;
        case PM_PARAM_EQUAL_TEMPERAMENT: *value = engine->voicing.equalTemperament ? 1.0 : 0.0; return 1;
        case PM_PARAM_UNISON_WIDTH: *value = engine->voicing.unisonWidth; return 1;
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
