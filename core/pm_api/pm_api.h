/* Stable C interface to the piano engine (plan §5.2), for Swift and Python.
 *
 * Threads: create/destroy/prepare and the velocity-curve parameters
 * (PM_PARAM_VELOCITY_*) from a non-real-time thread; pm_engine_push_event
 * from one producer thread; pm_engine_render from the audio thread (real-time
 * safe); other parameters from any thread. Every function is noexcept and
 * tolerates a null engine pointer. */
#ifndef PM_API_H
#define PM_API_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PM_API_VERSION_MAJOR 0
#define PM_API_VERSION_MINOR 2
#define PM_API_VERSION_PATCH 0

typedef struct pm_engine pm_engine;

enum pm_event_type {
    PM_EVENT_NOTE_ON = 1,       /* note, value = 16-bit velocity (0 = note-off) */
    PM_EVENT_NOTE_OFF = 2,      /* note, value = 16-bit release velocity (0 = default) */
    PM_EVENT_SUSTAIN = 3,       /* value = 16-bit pedal position (CC64) */
    PM_EVENT_ALL_NOTES_OFF = 4,
    PM_EVENT_PANIC = 5
};

enum pm_time_base {
    PM_TIME_HOST = 0,   /* host clock, mapped by pm_clock on each render call */
    PM_TIME_SAMPLE = 1  /* absolute engine sample index since prepare */
};

typedef struct pm_event {
    uint64_t time;
    uint8_t type;       /* pm_event_type */
    uint8_t time_base;  /* pm_time_base */
    uint8_t note;       /* MIDI note number, 21..108 */
    uint8_t reserved;
    uint16_t value;
    uint16_t reserved2;
} pm_event;

typedef struct pm_clock {
    uint64_t host_time_at_buffer_start;
    double host_ticks_per_sample;
} pm_clock;

enum pm_param {
    PM_PARAM_MASTER_GAIN_DB = 0,          /* -96 .. 24, default 0 */
    PM_PARAM_STEREO_WIDTH = 1,            /* 0 .. 1, default 0.35 */
    PM_PARAM_SCHEDULING_MODE = 2,         /* 0 lowest latency (default), 1 constant latency */
    PM_PARAM_CONSTANT_LATENCY_FRAMES = 3, /* default: max_frames from prepare */
    PM_PARAM_CULL_THRESHOLD_DB = 4,       /* default -100 */
    PM_PARAM_CULLING_ENABLED = 5,         /* 0 / 1, default 1 */
    PM_PARAM_VELOCITY_MIN = 6,            /* hammer m/s at velocity 0+, default 0.3 */
    PM_PARAM_VELOCITY_MAX = 7,            /* hammer m/s at velocity 127, default 5.5 */
    PM_PARAM_VELOCITY_EXPONENT = 8        /* power curve exponent, default 1.5 */
};

typedef struct pm_stats {
    int32_t total_modes;
    int32_t awake_groups;
    int32_t active_hammers;
    int32_t peak_active_hammers;
    uint64_t dropped_events;
    uint64_t deferred_strikes;
    uint64_t late_events;
    uint64_t limited_samples;
    float peak_pre_limiter;
} pm_stats;

uint32_t pm_api_version(void); /* (major << 16) | (minor << 8) | patch */
const char* pm_version_string(void);

pm_engine* pm_engine_create(void);  /* NULL on allocation failure */
void pm_engine_destroy(pm_engine* engine);

/* Returns 1 on success, 0 on failure. Resets all sound. */
int pm_engine_prepare(pm_engine* engine, double sample_rate, int32_t max_frames);

/* Returns 1 if queued, 0 if the queue was full (the event is dropped). */
int pm_engine_push_event(pm_engine* engine, const pm_event* event);

/* Renders frames of stereo audio (non-interleaved). clock may be NULL. */
void pm_engine_render(pm_engine* engine, float* left, float* right, int32_t frames,
                      const pm_clock* clock);

/* Return 1 on success, 0 for an unknown id. */
int pm_engine_set_param(pm_engine* engine, int32_t id, double value);
int pm_engine_get_param(const pm_engine* engine, int32_t id, double* value);

void pm_engine_get_stats(const pm_engine* engine, pm_stats* stats);

#ifdef __cplusplus
}
#endif

#endif /* PM_API_H */
