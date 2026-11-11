/* Stable C interface to the piano engine (plan §5.2), for Swift and Python.
 *
 * Threads: create/destroy/prepare, the velocity-curve parameters
 * (PM_PARAM_VELOCITY_*) and the tuning parameters from a non-real-time thread; pm_engine_push_event
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
#define PM_API_VERSION_MINOR 5
#define PM_API_VERSION_PATCH 0

typedef struct pm_engine pm_engine;

enum pm_event_type {
    PM_EVENT_NOTE_ON = 1,       /* note, value = 16-bit velocity (0 = note-off) */
    PM_EVENT_NOTE_OFF = 2,      /* note, value = 16-bit release velocity (0 = default) */
    PM_EVENT_SUSTAIN = 3,       /* value = 16-bit pedal position (CC64) */
    PM_EVENT_ALL_NOTES_OFF = 4,
    PM_EVENT_PANIC = 5,
    PM_EVENT_SOSTENUTO = 6,     /* value = 16-bit pedal position (CC66), on at half travel */
    PM_EVENT_UNA_CORDA = 7,     /* value = 16-bit pedal position (CC67), continuous */
    PM_EVENT_KEY_DOWN_SILENT = 8 /* note: key held down without sounding (damper up); NOTE_OFF releases */
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
    PM_PARAM_VELOCITY_EXPONENT = 8,       /* power curve exponent, default 1.5 */
    /* Tuning and voicing: take effect at the next pm_engine_prepare. */
    PM_PARAM_A4_HZ = 9,                   /* default 440 */
    PM_PARAM_TUNING_STRETCH = 10,         /* 0 .. 1.5 of the computed stretch, default 1 */
    PM_PARAM_EQUAL_TEMPERAMENT = 11,      /* 0 / 1: plain 12-TET, default 0 */
    PM_PARAM_UNISON_WIDTH = 12,           /* scales the unison detuning, default 1 */
    /* Soundboard (M4): take effect at the next pm_engine_prepare (the bake depends on them). */
    PM_PARAM_SOUNDBOARD_ENABLED = 13,     /* 0 / 1, default 1; 0 = no board, M3 output (A/B) */
    PM_PARAM_SYMPATHETIC = 14,            /* 0 .. 1.5, loop gain of the sympathetic return, default 1 */
    /* Real-time (any thread): */
    PM_PARAM_KNOCK_LEVEL = 15,            /* hammer knock, N per m/s of hammer speed, default 0.02; 0 = off */
    PM_PARAM_DAMPER_NOISE_LEVEL = 16,     /* damper-lift noise, N per pedal travel/s, default 0.004; 0 = off */
    PM_PARAM_HALF_PEDAL_P0_TREBLE = 17,   /* pedal position where the treble dampers start to lift, 0.30 */
    PM_PARAM_HALF_PEDAL_P0_BASS = 18,     /* the same for the bass, default 0.40 */
    PM_PARAM_HALF_PEDAL_WIDTH = 19,       /* width of the transition zone, default 0.25 */
    PM_PARAM_DAMPED_CULL_OFFSET_DB = 20,  /* seated dampers woken by the bridge only this far above the
                                             culling threshold, default 120 (never) */
    /* Soundboard loop quality: take effect at the next pm_engine_prepare. */
    PM_PARAM_QUALITY_TIER = 21,           /* pm_quality_tier, default PM_QUALITY_FULL */
    PM_PARAM_LOOP_CUTOFF_HZ = 22          /* > 0 overrides the tier's loop band (Hz), default 0 */
};

/* Partials below the loop band are fed back by the string-board loop
 * (sympathetic resonance, bloom); the others only drive the board. Loop
 * cost is proportional to the fed-back partials. */
enum pm_quality_tier {
    PM_QUALITY_FULL = 0,      /* the board's whole modal range (5 kHz) */
    PM_QUALITY_BALANCED = 1,  /* 2 kHz: half the fed-back modes, bloom within 0.7 dB */
    PM_QUALITY_ECO = 2        /* 1 kHz: a quarter, bloom within about 1.5-3 dB */
};

typedef struct pm_stats {
    int32_t total_modes;
    int32_t awake_groups;
    int32_t awake_modes;
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
