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
#define PM_API_VERSION_MINOR 11
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

/* ------------------------------------------------------------------------
 * App host (API 0.6). A pm_host owns an engine and connects it to the
 * platform's threads (core/pm_host/host.h):
 *   MIDI thread:  pm_host_midi_event_list / _ump / _bytes (one thread at a time)
 *   UI thread:    pm_host_push_event, pm_host_monitor_pop, meters, parameters
 *   audio thread: pm_host_render
 *   any non-real-time thread: pm_host_prepare (blocks until the audio
 *   thread has left pm_host_render; the audio thread never waits)
 * Do not call pm_engine_prepare, pm_engine_push_event or pm_engine_render on
 * the engine of a host; pm_engine_set_param / get_param / get_stats are fine.
 *
 * Time. MIDI timestamps are host-clock ticks (Core MIDI: mach_absolute_time);
 * 0 means "now". pm_host_render takes the host time of the buffer's first
 * frame (AudioTimeStamp.mHostTime; 0 if unknown). With the tick-to-
 * nanosecond ratio from pm_host_set_timebase (mach_timebase_info: ns =
 * ticks * numer / denom), ticks per sample = 1e9 / fs * denom / numer, and
 * an event at host time t lands at frame floor((t - buffer start) / ticks
 * per sample) of the buffer: events in the past at frame 0 (lowest-latency
 * mode), events beyond the buffer stay queued for a later one. */
typedef struct pm_host pm_host;

enum pm_midi_kind {
    PM_MIDI_NOTE_OFF = 1,
    PM_MIDI_NOTE_ON = 2,
    PM_MIDI_POLY_PRESSURE = 3,
    PM_MIDI_CONTROL_CHANGE = 4,
    PM_MIDI_PROGRAM_CHANGE = 5,
    PM_MIDI_CHANNEL_PRESSURE = 6,
    PM_MIDI_PITCH_BEND = 7,
    PM_MIDI_SYSEX = 8,
    PM_MIDI_SYSTEM_COMMON = 9,
    PM_MIDI_REALTIME = 10,
    PM_MIDI_UTILITY = 11,
    PM_MIDI_OTHER = 12
};

/* One received MIDI message, for a MIDI monitor. status/data1/data2 are the
 * MIDI 1.0 bytes (for MIDI 2.0 messages: the 1.0 equivalent); value is the
 * full-resolution value (MIDI 1.0: data2, or the 14-bit pitch bend; MIDI
 * 2.0: 16-bit velocity, 32-bit controller); words are the UMP words
 * (word_count 0 for byte streams); length is a SysEx payload length. */
typedef struct pm_midi_monitor_entry {
    uint64_t host_time;
    uint32_t source_id;
    uint8_t kind;        /* pm_midi_kind */
    uint8_t channel;     /* 0..15 */
    uint8_t status;
    uint8_t data1;
    uint8_t data2;
    uint8_t group;       /* UMP group */
    uint8_t protocol;    /* 1 or 2 */
    uint8_t word_count;
    uint32_t value;
    uint32_t length;
    uint32_t words[4];
} pm_midi_monitor_entry;

typedef struct pm_host_load {
    double mean_load;    /* render time / buffer period, mean since the last reset */
    double recent_load;  /* smoothed over about 0.5 s */
    double max_load;     /* worst buffer since the last reset */
    double last_load;
    double budget;       /* over_budget counts buffers above this fraction (default 0.7) */
    uint64_t buffers;
    uint64_t over_budget;
    int32_t last_frames; /* frames of the last rendered buffer */
    int32_t reserved;
} pm_host_load;

typedef struct pm_host_counters {
    uint64_t midi_messages;     /* decoded, all kinds */
    uint64_t midi_malformed;
    uint64_t midi_events;       /* engine events produced from MIDI */
    uint64_t dropped_events;    /* host queues full */
    uint64_t monitor_dropped;   /* monitor ring full */
} pm_host_counters;

pm_host* pm_host_create(void);  /* NULL on allocation failure; not prepared */
void pm_host_destroy(pm_host* host);
pm_engine* pm_host_engine(pm_host* host);

/* Tick-to-nanosecond ratio of the host clock (default 1/1: nanoseconds).
 * Takes effect at the next pm_host_prepare. */
void pm_host_set_timebase(pm_host* host, uint32_t numer, uint32_t denom);
double pm_host_ticks_per_sample(double sample_rate, uint32_t numer, uint32_t denom);

/* Non-real-time. Returns 1 on success. Also applies prepare-time parameters
 * (tuning, soundboard, quality tier). Queued events are discarded. */
int pm_host_prepare(pm_host* host, double sample_rate, int32_t max_frames);
int pm_host_is_prepared(const pm_host* host);
double pm_host_sample_rate(const pm_host* host);

/* Audio thread. Forwards queued events, renders, and records the render
 * time in the load meter. Silence while not prepared or while preparing. */
void pm_host_render(pm_host* host, float* left, float* right, int32_t frames, uint64_t host_time);

/* MIDI thread. event_list points to a Core MIDI MIDIEventList (MIDI 1.0 or
 * 2.0 protocol UMP); returns the number of packets. */
uint32_t pm_host_midi_event_list(pm_host* host, const void* event_list, uint32_t source_id);
void pm_host_midi_ump(pm_host* host, const uint32_t* words, int32_t count, uint64_t host_time,
                      uint32_t source_id);
void pm_host_midi_bytes(pm_host* host, const uint8_t* bytes, int32_t count, uint64_t host_time,
                        uint32_t source_id);

/* UI thread (a second producer, independent of the MIDI thread). Returns 1
 * if queued. Use time_base PM_TIME_SAMPLE with time 0 for "now". */
int pm_host_push_event(pm_host* host, const pm_event* event);

/* UI thread. Returns 1 and fills entry while the monitor ring has entries. */
int pm_host_monitor_pop(pm_host* host, pm_midi_monitor_entry* entry);
void pm_host_set_monitor_enabled(pm_host* host, int enabled);

void pm_host_get_load(const pm_host* host, pm_host_load* load);
void pm_host_reset_load(pm_host* host);
void pm_host_set_load_budget(pm_host* host, double fraction);
void pm_host_get_counters(const pm_host* host, pm_host_counters* counters);

/* ------------------------------------------------------------------------
 * Load diagnostics (API 0.7): enough to explain a slow buffer without a
 * profiler. Render times are wall-clock time spent in pm_host_render;
 * loads are render time / the buffer's own period (frames / sample rate).
 * p99 values come from fixed-bin histograms (0.25 % load bins; render-time
 * bins about 1.5 % wide) and are the upper edge of the bin holding the 99th
 * percentile. Everything is since the last pm_host_reset_load. */
typedef struct pm_host_diagnostics {
    double sample_rate;          /* prepared rate */
    uint64_t buffers;
    int32_t last_frames;         /* frames of the last render call */
    int32_t min_frames;          /* smallest / largest frame count seen */
    int32_t max_frames;
    int32_t distinct_frame_counts; /* > 1: the callback is not called with a fixed size */
    double last_period_us;       /* last_frames / sample rate */
    double mean_us, p99_us, max_us;        /* render time per buffer */
    double mean_load, p99_load, max_load;  /* fractions of the buffer period */
    double recent_load;
    double budget;
    uint64_t over_budget;
    /* The buffer with the highest load. */
    uint64_t worst_index;        /* its number since the reset (1 = first) */
    int32_t worst_frames;
    int32_t worst_hammers_in_contact; /* hammers that touched a string in it, including those
                                         released before its end (API 0.9; before: only those
                                         still active at its end) */
    int32_t worst_hammers_active;     /* active at its end */
    int32_t worst_awake_modes;
    int32_t worst_events;        /* events applied in it */
    int32_t worst_sub_blocks;    /* segments the engine cut it into */
    double worst_period_us;
    double worst_render_us;
    double worst_load;
    double worst_cpu_us;         /* thread CPU time of that buffer; < 0 when not measured */
    /* Thread CPU time (pm_host_set_cpu_timing): the largest wall-clock minus
     * CPU time of any buffer, i.e. time the audio thread was not running
     * (preemption); < 0 when not measured. */
    double max_stall_us;
    int32_t cpu_timing;          /* 1 while thread CPU time is measured */
    int32_t reserved;
} pm_host_diagnostics;

void pm_host_get_diagnostics(const pm_host* host, pm_host_diagnostics* diagnostics);

/* Off by default. When on, pm_host_render also reads the audio thread's CPU
 * time (POSIX clock_gettime(CLOCK_THREAD_CPUTIME_ID), two calls per buffer:
 * a kernel call that neither allocates nor waits on a lock, but a kernel
 * call all the same, hence opt-in). Comparing it with the wall-clock time
 * separates preemption from slow execution. Has no effect where the clock
 * does not exist. Takes effect at the next buffer. */
void pm_host_set_cpu_timing(pm_host* host, int enabled);

/* ------------------------------------------------------------------------
 * Phase timing and event detail (API 0.8): where the render time of a slow
 * buffer went. Off by default. When on, the engine reads a steady clock at
 * about five points per 32-frame sub-block (16 sub-blocks in a 512-frame
 * buffer) and twice per sample while a hammer is active, a user-space
 * counter read (no lock, no allocation, no system call on Apple platforms);
 * the split is then kept for the worst buffer, as a mean over all timed
 * buffers and as a mean over the top 1 % by render time. Phases: events
 * (event handling, control ticks, damper updates, loop set-up including
 * impulse-response and zone-kernel recomputes), hammers (contact solves),
 * free (the strings' block pass), forced (bridge input added to the
 * strings), board (zone kernel and soundboard step), other (output stage,
 * cull passes, the rest). The event kinds and recompute counts of the worst
 * buffer are recorded whether or not timing is on. Since the last
 * pm_host_reset_load. */
enum pm_phase {
    PM_PHASE_EVENTS = 0,
    PM_PHASE_HAMMERS = 1,
    PM_PHASE_FREE = 2,
    PM_PHASE_FORCED = 3,
    PM_PHASE_BOARD = 4,
    PM_PHASE_OTHER = 5,
    PM_PHASE_COUNT = 6
};

typedef struct pm_host_phase_diagnostics {
    int32_t phase_timing;        /* 1 while phase timing is on */
    int32_t worst_timed;         /* 1 if the worst buffer was timed (worst_phase_us valid) */
    uint64_t timed_buffers;
    uint64_t top_count;          /* buffers in the top-1 % mean (whole render-time bins, about 1.5 % wide) */
    double worst_phase_us[PM_PHASE_COUNT];
    double top_phase_us[PM_PHASE_COUNT];
    double mean_phase_us[PM_PHASE_COUNT];
    /* The worst buffer of pm_host_diagnostics: */
    int32_t worst_note_ons;
    int32_t worst_note_offs;     /* note-offs, including note-ons with velocity 0 */
    int32_t worst_pedal_events;  /* sustain, sostenuto and una corda changes */
    int32_t worst_damper_updates;   /* keys whose damper amount was applied to their strings */
    int32_t worst_response_groups;  /* string mode groups whose loop impulse response was recomputed */
    int32_t worst_kernel_rebuilds;  /* full soundboard-loop zone-kernel rebuilds (API 0.9: only
                                       when the loop's member set changes; see pm_host_worst_detail) */
    int32_t worst_idle_sub_blocks;  /* sub-blocks skipped as silent */
    int32_t reserved;
} pm_host_phase_diagnostics;

void pm_host_set_phase_timing(pm_host* host, int enabled);
void pm_host_get_phase_diagnostics(const pm_host* host, pm_host_phase_diagnostics* diagnostics);

/* ------------------------------------------------------------------------
 * Hammer and kernel detail of the worst buffer (API 0.9), recorded whether
 * or not phase timing is on. */
typedef struct pm_host_worst_detail {
    int32_t hammers_in_contact;      /* hammers that touched a string in it (= pm_host_diagnostics) */
    int32_t max_hammers_in_contact;  /* most hammers touching at the same sample (board off: sub-block) */
    int32_t hammer_contact_samples;  /* sum over hammers of the base samples each spent in contact */
    int32_t newton_evals;            /* contact-solver Newton evaluations (each S felt forces) */
    int32_t fallback_solves;         /* contact substeps that needed the bracketed fallback */
    int32_t kernel_rebuilds;         /* full zone-kernel rebuilds (member set changed) */
    int32_t kernel_key_updates;      /* keys whose zone-kernel columns were updated in place */
    int32_t reserved;
} pm_host_worst_detail;

void pm_host_get_worst_detail(const pm_host* host, pm_host_worst_detail* detail);

/* ------------------------------------------------------------------------
 * Work counts of one buffer (API 0.10), the same for the live meter's worst
 * buffer (pm_host_get_worst_work), a benchmark scenario's most expensive
 * buffer (pm_bench_result.worst) and an audio-thread scene run
 * (pm_host_scene_result.worst), so that two instances or two kinds of run
 * can be compared count by count. */
typedef struct pm_work_detail {
    int32_t note_ons, note_offs, pedal_events;
    int32_t awake_modes;             /* at the end of the buffer, excluding group padding */
    int32_t hammers_in_contact, max_hammers_in_contact, hammer_contact_samples;
    int32_t hammer_samples;          /* hammer-samples stepped: every active hammer (in flight, in contact or
                                        waiting to be released), one pass over its key's modes per sample */
    int32_t hammer_damped_samples;   /* of those, with the key's damper on (two more passes per sample) */
    int32_t newton_evals, fallback_solves;
    int32_t damper_updates, response_groups, kernel_rebuilds, kernel_key_updates;
    int32_t free_group_blocks;       /* bank group x sub-block invocations of the free pass */
    int32_t forced_group_blocks;     /* ... of the forced pass (soundboard-loop input) */
    int32_t loop_members;            /* keys with awake coupled groups (largest over the sub-blocks) */
    int32_t loop_listeners;          /* keys taking loop input (largest over the sub-blocks) */
    int32_t loop_entries;            /* zone-kernel entries (largest over the sub-blocks) */
    int32_t out_zones;               /* board zones read back per sample (largest over the sub-blocks) */
    int32_t reserved;
} pm_work_detail;

void pm_host_get_worst_work(const pm_host* host, pm_work_detail* detail);

/* Effective engine configuration as text (sample rate, block limits, tier,
 * loop band, board, culling, parameters, mode counts, SIMD kernel, array
 * placement modulo 16 KiB), one "key value" per line, for diffing the app's
 * engine against the benchmark's (pm_bench_last_config). Writes at most
 * size bytes including the terminating zero; returns the length. Main
 * thread; prepare-time values are read without synchronisation (stable
 * between prepares). */
int32_t pm_engine_describe(const pm_engine* engine, char* buffer, int32_t size);

/* ------------------------------------------------------------------------
 * Speed probe (API 0.9): how fast the audio thread actually executes. A
 * fixed workload (core/pm_host/speed_probe.h: SIMD one-pole modes and a
 * scalar Newton chain, a few microseconds on a fast core; no allocation,
 * no lock) is timed on the audio thread at the start of every Nth
 * pm_host_render call (before the render's own timing), and the same
 * workload on any other thread with pm_speed_probe_run_us. The ratio
 * reference / audio-thread time is the speed the audio thread ran at
 * relative to the reference thread (core type, clock). Off by default. */
typedef struct pm_speed_probe_stats {
    int32_t interval;    /* pm_host_get_speed_probe: N (0 = off) */
    int32_t reserved;
    uint64_t runs;       /* probe runs since the last pm_host_reset_load (or measured) */
    double min_us, median_us, mean_us, max_us;  /* host median: histogram bin centre, about 1.5 % */
    float checksum;      /* pm_speed_probe_run_us: the workload's result (consumed) */
    float reserved2;
} pm_speed_probe_stats;

/* Any thread; takes effect at the next buffer. every_n_buffers <= 0: off. */
void pm_host_set_speed_probe(pm_host* host, int32_t every_n_buffers);
void pm_host_get_speed_probe(const pm_host* host, pm_speed_probe_stats* stats);
/* Runs the workload 'runs' times (1..4096) on the calling thread; returns
 * the median in microseconds and fills stats (may be NULL). Non-real-time. */
double pm_speed_probe_run_us(int32_t runs, pm_speed_probe_stats* stats);

/* ------------------------------------------------------------------------
 * Speed probe parts and CPU numbers (API 0.11).
 *
 * The probe has two parts, timed separately: latency-bound (a dependent
 * scalar sqrt/divide chain and a dependent vector multiply-add chain) and
 * throughput-bound (the mode-bank kernel's shape: 1024 complex one-pole
 * modes, 16 KiB, many independent vector operations in flight). A core or
 * clock change slows them by different factors: an efficiency core loses
 * little more than its clock on the first and much more on the second, as
 * the engine's wide inner loops do. pm_speed_probe_stats keeps reporting the
 * whole probe (both parts; not comparable with the API 0.9-0.10 workload).
 * Times in microseconds; host medians are histogram bin centres (1.5 %). */
typedef struct pm_probe_series {
    uint64_t runs;
    double min_us, median_us, mean_us, max_us;
} pm_probe_series;

typedef struct pm_speed_probe_split {
    pm_probe_series latency;
    pm_probe_series throughput;
} pm_speed_probe_split;

/* The audio thread's probe parts since the last pm_host_reset_load. */
void pm_host_get_speed_probe_split(const pm_host* host, pm_speed_probe_split* split);
/* Runs the probe 'runs' times (1..4096) on the calling thread; fills split
 * (may be NULL) and returns the throughput part's median. Non-real-time. */
double pm_speed_probe_run_split(int32_t runs, pm_speed_probe_split* split);

/* Which CPU the calling thread runs on now (Apple: pthread_cpu_number_np;
 * Linux: sched_getcpu), or -1 where it cannot be read. */
int32_t pm_current_cpu(void);

/* Per-CPU usage of the render calls. pm_host_render reads the CPU number at
 * the start and at the end of each render call (outside the timed interval)
 * while any of the run-time diagnostics is on: thread CPU time
 * (pm_host_set_cpu_timing), phase timing or the speed probe. Two reads of a
 * per-CPU register value (no system call, no lock; see core/pm_host/cpu_id.h
 * for the platforms and what was verified). A call is counted on the CPU it
 * started on. CPU numbers are the kernel's logical CPU numbers; which of them
 * are efficiency or performance cores is not part of this API (on current
 * Apple silicon the efficiency cores have the lowest numbers, but nothing
 * documents that). */
#define PM_MAX_CPUS 64
typedef struct pm_cpu_usage {
    uint64_t buffers_tracked;  /* render calls whose CPU number was read (and < PM_MAX_CPUS) */
    uint64_t buffers_unknown;  /* render calls without a CPU number (diagnostics off, or none) */
    uint64_t migrations;       /* render calls that ended on another CPU than they started on */
    int32_t worst_cpu_start;   /* the worst buffer's CPUs (pm_host_diagnostics' worst; -1: unknown) */
    int32_t worst_cpu_end;
    uint32_t buffers[PM_MAX_CPUS];           /* render calls per CPU */
    double mean_render_us[PM_MAX_CPUS];      /* their mean render time */
    uint32_t probe_runs[PM_MAX_CPUS];        /* speed-probe runs per CPU */
    double probe_latency_us[PM_MAX_CPUS];    /* their mean latency-bound part */
    double probe_throughput_us[PM_MAX_CPUS]; /* their mean throughput-bound part */
} pm_cpu_usage;

/* Live meter, since the last pm_host_reset_load. */
void pm_host_get_cpu_usage(const pm_host* host, pm_cpu_usage* usage);

/* ------------------------------------------------------------------------
 * Build information (API 0.7): a static string describing how the engine
 * was compiled (SIMD kernel, optimisation, NDEBUG, FMA contraction,
 * compiler), for diagnostics. */
const char* pm_build_info(void);

/* ------------------------------------------------------------------------
 * Offline benchmark (API 0.7): the scenarios of tools/bench/bench_engine
 * (core/pm_bench/bench.h), run on the calling thread with a private engine
 * as fast as it can, for measuring a device. Non-real-time: each item
 * prepares its own engine (about a second) and then renders for 'seconds';
 * call from a background thread. Items: the engine scenarios, then the
 * hammer micro-benchmark per key (unless PM_BENCH_NO_HAMMER). */
enum pm_bench_flags {
    PM_BENCH_ALL_TIERS = 1,  /* also worst/cluster/bloom at Balanced and Eco */
    PM_BENCH_NO_HAMMER = 2,
    PM_BENCH_PHASE_TIMING = 4, /* API 0.9: phase timing on during the scenarios (its cost included) */
    /* API 0.10: sleep to each buffer's real-time deadline between render
     * calls, as an audio thread does (only the render call is timed); with
     * EVICT also walk a 32 MiB scratch buffer between buffers (other work on
     * the core). EVICT implies PACED. */
    PM_BENCH_PACED = 8,
    PM_BENCH_PACED_EVICT = 16,
    /* API 0.10, pm_host_scene_start only: render the scene on the audio
     * thread with a private engine prepared exactly as the benchmark
     * prepares the item's (instead of the app's engine instance), to tell
     * the instance from the thread. */
    PM_BENCH_SCENE_FRESH_ENGINE = 32,
    /* API 0.11, scenarios: the speed probe (both parts) right before every
     * 8th render call, untimed, as the host runs it on the audio thread;
     * see pm_bench_last_probe_split and pm_bench_last_cpu_usage. */
    PM_BENCH_SPEED_PROBE = 64
};

enum pm_bench_kind {
    PM_BENCH_SCENARIO = 0,
    PM_BENCH_HAMMER = 1
};

typedef struct pm_bench_config {
    double sample_rate;      /* <= 0: 48000 */
    double seconds;          /* measured per scenario; <= 0: 3 */
    int32_t block_frames;    /* frames per render call; <= 0: 64 */
    int32_t tier;            /* pm_quality_tier of the board-on scenarios */
    int32_t hammer_strikes;  /* per key; <= 0: 200 */
    int32_t flags;           /* pm_bench_flags */
} pm_bench_config;

typedef struct pm_bench_result {
    int32_t kind;            /* pm_bench_kind */
    int32_t ok;              /* 0: the item could not be run */
    double period_us;        /* block_frames / sample_rate */
    /* PM_BENCH_SCENARIO: render time per buffer. */
    double mean_us, p99_us, max_us;
    double mean_load, p99_load, max_load; /* fractions of the period */
    double first_100ms_load; /* mean over the first 100 ms measured */
    double mean_awake_modes;
    int32_t awake_modes_end;
    /* PM_BENCH_HAMMER: one hammer alone, us per buffer (medians). */
    int32_t key, strings, modes, oversampling;
    double contact_us;       /* first buffer after the launch: in contact */
    double life_us;          /* mean per buffer until the hammer is released */
    /* API 0.10, PM_BENCH_SCENARIO: */
    int32_t has_phases;      /* PM_BENCH_PHASE_TIMING: the phase splits below are valid */
    int32_t top_count;       /* buffers in the top-1 % mean */
    double worst_phase_us[PM_PHASE_COUNT];  /* the most expensive buffer (by the engine's own total) */
    double top_phase_us[PM_PHASE_COUNT];
    double mean_phase_us[PM_PHASE_COUNT];
    double hammer_us_per_sample;          /* hammers phase per hammer-sample stepped (0: no hammer work) */
    double hammer_us_per_contact_sample;  /* ... per hammer-sample in contact */
    double free_group_blocks, forced_group_blocks;  /* means per buffer */
    double loop_members, loop_listeners;            /* means per buffer */
    double kernel_rebuilds, kernel_key_updates;     /* means per buffer */
    int32_t overruns;        /* buffers whose render took longer than the period */
    int32_t paced;           /* 1: paced to real time (2: with the eviction walk) */
    pm_work_detail worst;    /* the most expensive buffer by wall time */
} pm_bench_result;

int32_t pm_bench_item_count(const pm_bench_config* config);
/* Static string; "" for an index out of range. */
const char* pm_bench_item_name(const pm_bench_config* config, int32_t index);
/* Returns 1 and fills result on success. */
int pm_bench_run_item(const pm_bench_config* config, int32_t index, pm_bench_result* result);
/* API 0.10: the effective configuration (pm_engine_describe format) of the
 * engine of the last scenario run on the calling thread; "" before any. */
int32_t pm_bench_last_config(char* buffer, int32_t size);
/* API 0.10: the scene (pm_bench_scene) of scenario item 'index', or -1. */
int32_t pm_bench_item_scene(const pm_bench_config* config, int32_t index);
/* API 0.11: of the last scenario run on the calling thread, the CPUs its
 * measured render calls ran on (always recorded) and, with
 * PM_BENCH_SPEED_PROBE, the probe's parts (runs 0 otherwise). Zeros before
 * any. */
void pm_bench_last_cpu_usage(pm_cpu_usage* usage);
void pm_bench_last_probe_split(pm_speed_probe_split* split);

/* API 0.11: pacing hook. A paced scenario (PM_BENCH_PACED) run on the
 * calling thread calls hook(ctx, event, ns) on that thread: PM_PACE_BEGIN
 * once after the engine is prepared and before the first buffer (ns = the
 * buffer period), PM_PACE_BUFFER before each render call (ns = time left
 * until that buffer's real-time deadline, may be negative after an
 * overrun), PM_PACE_BUFFER_DONE after it (before the sleep; ns = 0) and
 * PM_PACE_END after the last buffer. Lets the app run the paced loop in the
 * scheduling class of an audio thread (time-constraint policy, an audio
 * work interval with each buffer's deadline) without platform code in the
 * engine. The hook's own time is outside the timed render call. NULL
 * removes it. Per thread. */
enum pm_pace_event {
    PM_PACE_BEGIN = 0,
    PM_PACE_BUFFER = 1,
    PM_PACE_BUFFER_DONE = 2,
    PM_PACE_END = 3
};
typedef void (*pm_pace_hook)(void* ctx, int32_t event, int64_t ns);
void pm_bench_set_pace_hook(pm_pace_hook hook, void* ctx);

/* ------------------------------------------------------------------------
 * Scene on the audio thread (API 0.10): the events of a benchmark scenario
 * (the same sceneEvents, at the host's sample rate) played through the real
 * host and audio render path, in real time and audible, so that identical
 * material can be measured three ways: back to back on the benchmark
 * thread, paced on the benchmark thread (PM_BENCH_PACED) and on the real
 * audio thread with the app's engine instance.
 *
 * pm_host_scene_start (non-real-time; briefly closes the prepare gate, so
 * the audio thread outputs one buffer or so of silence): builds the event
 * list, resets the engine to its just-prepared state (silence, pedals up,
 * sample time 0; the app's parameters and tables are kept) and arms the
 * player; with PM_BENCH_PHASE_TIMING in config->flags phase timing is on for
 * the run. The audio thread then pushes each buffer's events into the
 * engine before rendering it (no allocation, no lock), resets the load
 * meter when the warm-up is over (0.5 s, none for the worst scene, as the
 * benchmark), measures config->seconds and publishes the result: the load
 * meter's statistics over exactly the measured buffers (mean exact; p99 as
 * the meter's histogram, about 1.5 %), the phase splits, the work counts of
 * the worst buffer and means per buffer, and the speed probe over the run
 * (if the probe is on). MIDI and UI events keep arriving and are rendered
 * too; leave the keyboard alone. The engine keeps playing from the scene's
 * end state afterwards (send a panic or all-notes-off). Returns 0 if the
 * host is not prepared or the item is not a scenario. */
enum pm_scene_state {
    PM_SCENE_IDLE = 0,
    PM_SCENE_RUNNING = 1,
    PM_SCENE_DONE = 2
};

typedef struct pm_host_scene_result {
    int32_t state;           /* pm_scene_state */
    int32_t buffers;         /* measured */
    double sample_rate;
    int32_t min_frames, max_frames;
    double period_us;        /* of the last buffer */
    double mean_us, p99_us, max_us;
    double mean_load, p99_load, max_load;
    double mean_awake_modes;
    double free_group_blocks, forced_group_blocks, loop_members, loop_listeners;  /* means per buffer */
    double kernel_rebuilds, kernel_key_updates;  /* means per buffer */
    double hammer_us_per_sample, hammer_us_per_contact_sample;
    int32_t overruns;        /* buffers whose render took longer than their period */
    int32_t has_phases;
    int32_t top_count;
    int32_t worst_timed;
    double worst_phase_us[PM_PHASE_COUNT];
    double top_phase_us[PM_PHASE_COUNT];
    double mean_phase_us[PM_PHASE_COUNT];
    pm_work_detail worst;
    double worst_us;
    pm_speed_probe_stats probe;  /* audio-thread probe over the measured buffers (runs = 0: off) */
    int32_t fresh_engine;    /* 1: rendered with a private, benchmark-prepared engine */
    int32_t reserved;
} pm_host_scene_result;

int pm_host_scene_start(pm_host* host, const pm_bench_config* config, int32_t item_index);
/* Any thread: stops a running scene (no result). */
void pm_host_scene_cancel(pm_host* host);
/* Main thread: the state, and once PM_SCENE_DONE the result. */
void pm_host_scene_get_result(const pm_host* host, pm_host_scene_result* result);
/* With PM_BENCH_SCENE_FRESH_ENGINE: a private engine is prepared first (on
 * the calling thread, about a second) for the item's case at the host's
 * sample rate and maximum frame count, and the audio thread renders it, and
 * feeds it the MIDI and UI events, until the scene is done or cancelled;
 * then it returns to the app's engine, which kept its state. The private
 * engine is freed at the next pm_host_scene_start or pm_host_destroy. */
/* The engine configuration captured when the scene started (pm_engine_describe). */
int32_t pm_host_scene_config(const pm_host* host, char* buffer, int32_t size);
/* API 0.11, once PM_SCENE_DONE: the CPUs of the measured buffers (read when
 * phase timing, thread CPU time or the speed probe is on, as the live meter)
 * and the audio-thread probe's parts over the run. Zeros otherwise. */
void pm_host_scene_get_cpu_usage(const pm_host* host, pm_cpu_usage* usage);
void pm_host_scene_get_probe_split(const pm_host* host, pm_speed_probe_split* split);

#ifdef __cplusplus
}
#endif

#endif /* PM_API_H */
