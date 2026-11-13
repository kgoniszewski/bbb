// C API of the offline benchmark (pm_api.h, "Offline benchmark") and the
// build information string.
#include <cstdio>
#include <memory>
#include <new>

#include "pm_api/pm_api.h"
#include "pm_base/simd.h"
#include "pm_bench/bench.h"
#include "pm_api/work_detail.h"

namespace {

pm::bench::Config toConfig(const pm_bench_config* c) {
    pm::bench::Config k;
    if (!c) return k;
    if (c->sample_rate > 0.0) k.sampleRate = c->sample_rate;
    if (c->seconds > 0.0) k.seconds = c->seconds;
    if (c->block_frames > 0) k.block = c->block_frames;
    if (c->tier >= 0 && c->tier <= 2) k.tier = static_cast<pm::QualityTier>(c->tier);
    if (c->hammer_strikes > 0) k.hammerStrikes = c->hammer_strikes;
    k.allTiers = (c->flags & PM_BENCH_ALL_TIERS) != 0;
    k.phaseTiming = (c->flags & PM_BENCH_PHASE_TIMING) != 0;
    k.paced = (c->flags & (PM_BENCH_PACED | PM_BENCH_PACED_EVICT)) != 0;
    k.evictBytes = (c->flags & PM_BENCH_PACED_EVICT) != 0 ? pm::bench::kDefaultEvictBytes : 0;
    return k;
}

// The last scenario's engine configuration, per thread (the app runs the
// benchmark on one background thread).
thread_local char tLastConfig[4096] = {};

bool withHammer(const pm_bench_config* c) { return !c || (c->flags & PM_BENCH_NO_HAMMER) == 0; }

}  // namespace

extern "C" {

int32_t pm_bench_item_count(const pm_bench_config* config) {
    const pm::bench::Config k = toConfig(config);
    return pm::bench::caseCount(k) + (withHammer(config) ? pm::bench::kHammerKeyCount : 0);
}

const char* pm_bench_item_name(const pm_bench_config* config, int32_t index) {
    const pm::bench::Config k = toConfig(config);
    const int cases = pm::bench::caseCount(k);
    if (index >= 0 && index < cases) return pm::bench::caseAt(k, index).name;
    if (withHammer(config) && index >= cases && index < cases + pm::bench::kHammerKeyCount)
        return pm::bench::hammerName(index - cases);
    return "";
}

int pm_bench_run_item(const pm_bench_config* config, int32_t index, pm_bench_result* result) {
    if (!result) return 0;
    *result = pm_bench_result{};
    const pm::bench::Config k = toConfig(config);
    const int cases = pm::bench::caseCount(k);
    result->period_us = 1e6 * k.block / k.sampleRate;
    if (index >= 0 && index < cases) {
        result->kind = PM_BENCH_SCENARIO;
        std::unique_ptr<pm::Engine> eng(new (std::nothrow) pm::Engine);
        if (!eng) return 0;
        pm::bench::ScenarioResult r;
        if (!pm::bench::runScenario(k, pm::bench::caseAt(k, index), *eng, r)) return 0;
        result->mean_us = r.meanUs;
        result->p99_us = r.p99Us;
        result->max_us = r.maxUs;
        result->mean_load = r.avgPercent / 100.0;
        result->p99_load = r.p99Percent / 100.0;
        result->max_load = r.maxPercent / 100.0;
        result->first_100ms_load = r.first100msPercent / 100.0;
        result->mean_awake_modes = r.meanAwakeModes;
        result->awake_modes_end = r.awakeModesEnd;
        result->has_phases = r.hasPhases ? 1 : 0;
        result->top_count = r.topCount;
        for (int p = 0; p < PM_PHASE_COUNT && p < pm::bench::kPhaseCount; ++p) {
            result->worst_phase_us[p] = r.maxBuf.us[p];
            result->top_phase_us[p] = r.top1.us[p];
            result->mean_phase_us[p] = r.all.us[p];
        }
        result->hammer_us_per_sample = r.hammerUsPerSample;
        result->hammer_us_per_contact_sample = r.hammerUsPerContactSample;
        result->free_group_blocks = r.freeGroupBlocksPerBuffer;
        result->forced_group_blocks = r.forcedGroupBlocksPerBuffer;
        result->loop_members = r.loopMembersMean;
        result->loop_listeners = r.loopListenersMean;
        result->kernel_rebuilds = r.kernelRebuildsPerBuffer;
        result->kernel_key_updates = r.kernelKeyUpdatesPerBuffer;
        result->overruns = r.overruns;
        result->paced = r.paced ? (r.evictBytes > 0 ? 2 : 1) : 0;
        pm::bench::toWorkDetail(r.maxDetail, result->worst);
        eng->describe(tLastConfig, static_cast<int>(sizeof tLastConfig));
        result->ok = 1;
        return 1;
    }
    if (withHammer(config) && index >= cases && index < cases + pm::bench::kHammerKeyCount) {
        result->kind = PM_BENCH_HAMMER;
        pm::bench::HammerResult h;
        if (!pm::bench::runHammer(k, pm::bench::kHammerKeys[index - cases], h)) return 0;
        result->key = h.key;
        result->strings = h.strings;
        result->modes = h.modes;
        result->oversampling = h.oversampling;
        result->contact_us = h.contactUs;
        result->life_us = h.lifeUs;
        result->ok = 1;
        return 1;
    }
    return 0;
}

int32_t pm_bench_last_config(char* buffer, int32_t size) {
    if (!buffer || size <= 0) return 0;
    int32_t n = 0;
    while (n < size - 1 && tLastConfig[n]) {
        buffer[n] = tLastConfig[n];
        ++n;
    }
    buffer[n] = 0;
    return n;
}

int32_t pm_bench_item_scene(const pm_bench_config* config, int32_t index) {
    const pm::bench::Config k = toConfig(config);
    if (index < 0 || index >= pm::bench::caseCount(k)) return -1;
    return static_cast<int32_t>(pm::bench::caseAt(k, index).scene);
}

const char* pm_build_info(void) {
    // Describes this translation unit's flags, which the build applies to
    // the whole engine (CMake: pm_options; SwiftPM: the target's settings).
    static const char* const kInfo =
        "simd "
#if defined(PM_SIMD_NEON)
        "neon"
#elif defined(PM_SIMD_AVX2)
        "avx2+fma"
#elif defined(PM_SIMD_SSE2)
        "sse2"
#else
        "scalar-fallback"
#endif
#if defined(__OPTIMIZE__)
        ", optimized"
#else
        ", NOT OPTIMIZED"
#endif
#if defined(__OPTIMIZE_SIZE__)
        " (size)"
#endif
#if defined(NDEBUG)
        ", NDEBUG"
#else
        ", asserts on"
#endif
#if defined(__FAST_MATH__)
        ", fast-math"
#endif
#if defined(__ARM_FEATURE_FMA) || defined(__FMA__)
        ", hardware fma"
#endif
#if defined(__has_feature)
#if __has_feature(address_sanitizer)
        ", ASAN"
#endif
#if __has_feature(thread_sanitizer)
        ", TSAN"
#endif
#endif
#if defined(__aarch64__) || defined(__arm64__)
        ", arm64"
#elif defined(__x86_64__) || defined(_M_X64)
        ", x86-64"
#endif
#if defined(__clang__)
        ", clang " __clang_version__
#elif defined(__GNUC__)
        ", gcc " __VERSION__
#endif
        ;
    return kInfo;
}

}  // extern "C"
