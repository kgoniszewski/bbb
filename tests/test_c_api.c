/* The C API compiles as C and drives the engine end to end. */
#include <math.h>
#include <stdio.h>

#include "pm_api/pm_api.h"

static int failures = 0;
#define EXPECT(c)                                                     \
    do {                                                              \
        if (!(c)) {                                                   \
            printf("  FAILED %s:%d: %s\n", __FILE__, __LINE__, #c); \
            ++failures;                                               \
        }                                                             \
    } while (0)

int main(void) {
    float left[256], right[256];
    double v = 0.0;
    float peak = 0.0f;
    int i, b;
    pm_stats st;
    pm_event e = {0};
    pm_engine* eng = pm_engine_create();

    EXPECT(pm_api_version() == ((PM_API_VERSION_MAJOR << 16) | (PM_API_VERSION_MINOR << 8) |
                                PM_API_VERSION_PATCH));
    EXPECT(eng != NULL);
    EXPECT(pm_engine_prepare(eng, 48000.0, 256) == 1);
    EXPECT(pm_engine_set_param(eng, PM_PARAM_MASTER_GAIN_DB, -6.0) == 1);
    EXPECT(pm_engine_get_param(eng, PM_PARAM_MASTER_GAIN_DB, &v) == 1 && v == -6.0);
    EXPECT(pm_engine_set_param(eng, PM_PARAM_VELOCITY_MAX, 5.0) == 1);
    EXPECT(pm_engine_get_param(eng, PM_PARAM_VELOCITY_MAX, &v) == 1 && v == 5.0);
    EXPECT(pm_engine_set_param(eng, 9999, 1.0) == 0);
    /* Tuning parameters apply at the next prepare. */
    EXPECT(pm_engine_set_param(eng, PM_PARAM_TUNING_STRETCH, 0.5) == 1);
    EXPECT(pm_engine_get_param(eng, PM_PARAM_TUNING_STRETCH, &v) == 1 && v == 0.5);
    EXPECT(pm_engine_set_param(eng, PM_PARAM_UNISON_WIDTH, 2.0) == 1);
    EXPECT(pm_engine_set_param(eng, PM_PARAM_A4_HZ, 442.0) == 1);
    EXPECT(pm_engine_set_param(eng, PM_PARAM_A4_HZ, 10.0) == 0);
    EXPECT(pm_engine_set_param(eng, PM_PARAM_EQUAL_TEMPERAMENT, 1.0) == 1);
    EXPECT(pm_engine_get_param(eng, PM_PARAM_EQUAL_TEMPERAMENT, &v) == 1 && v == 1.0);
    /* Soundboard (prepare-time) and real-time M4 parameters. */
    EXPECT(pm_engine_set_param(eng, PM_PARAM_SYMPATHETIC, 0.7) == 1);
    EXPECT(pm_engine_get_param(eng, PM_PARAM_SYMPATHETIC, &v) == 1 && fabs(v - 0.7) < 1e-12);
    EXPECT(pm_engine_set_param(eng, PM_PARAM_SOUNDBOARD_ENABLED, 1.0) == 1);
    EXPECT(pm_engine_set_param(eng, PM_PARAM_QUALITY_TIER, PM_QUALITY_BALANCED) == 1);
    EXPECT(pm_engine_get_param(eng, PM_PARAM_QUALITY_TIER, &v) == 1 && v == PM_QUALITY_BALANCED);
    EXPECT(pm_engine_set_param(eng, PM_PARAM_QUALITY_TIER, 3.0) == 0);
    EXPECT(pm_engine_set_param(eng, PM_PARAM_LOOP_CUTOFF_HZ, 1500.0) == 1);
    EXPECT(pm_engine_get_param(eng, PM_PARAM_LOOP_CUTOFF_HZ, &v) == 1 && v == 1500.0);
    EXPECT(pm_engine_set_param(eng, PM_PARAM_LOOP_CUTOFF_HZ, 0.0) == 1);
    EXPECT(pm_engine_set_param(eng, PM_PARAM_QUALITY_TIER, PM_QUALITY_FULL) == 1);
    EXPECT(pm_engine_set_param(eng, PM_PARAM_KNOCK_LEVEL, 0.0) == 1);
    EXPECT(pm_engine_get_param(eng, PM_PARAM_KNOCK_LEVEL, &v) == 1 && v == 0.0);
    EXPECT(pm_engine_set_param(eng, PM_PARAM_DAMPER_NOISE_LEVEL, 0.01) == 1);
    EXPECT(pm_engine_set_param(eng, PM_PARAM_HALF_PEDAL_WIDTH, 0.3) == 1);
    EXPECT(pm_engine_get_param(eng, PM_PARAM_HALF_PEDAL_WIDTH, &v) == 1 && fabs(v - 0.3) < 1e-6);
    EXPECT(pm_engine_set_param(eng, PM_PARAM_HALF_PEDAL_P0_BASS, 2.0) == 0);
    EXPECT(pm_engine_prepare(eng, 48000.0, 256) == 1);

    e.type = PM_EVENT_NOTE_ON;
    e.time_base = PM_TIME_SAMPLE;
    e.time = 100;
    e.note = 60;
    e.value = 50000;
    EXPECT(pm_engine_push_event(eng, &e) == 1);
    e.type = PM_EVENT_UNA_CORDA;
    e.time = 50;
    e.value = 65535;
    EXPECT(pm_engine_push_event(eng, &e) == 1);
    e.type = PM_EVENT_SOSTENUTO;
    e.time = 200;
    EXPECT(pm_engine_push_event(eng, &e) == 1);
    e.type = PM_EVENT_KEY_DOWN_SILENT;
    e.note = 48;
    e.time = 10;
    EXPECT(pm_engine_push_event(eng, &e) == 1);
    e.type = PM_EVENT_SUSTAIN;
    e.time = 300;
    e.value = 30000; /* half pedal */
    EXPECT(pm_engine_push_event(eng, &e) == 1);
    for (b = 0; b < 40; ++b) {
        pm_engine_render(eng, left, right, 256, NULL);
        for (i = 0; i < 256; ++i)
            if (fabsf(left[i]) > peak) peak = fabsf(left[i]);
    }
    pm_engine_get_stats(eng, &st);
    printf("  %s: %d modes (%d awake), peak %.4f\n", pm_version_string(), st.total_modes, st.awake_modes, peak);
    EXPECT(peak > 0.001f && peak < 1.0f);
    EXPECT(st.total_modes > 15000);
    EXPECT(st.awake_modes > 0 && st.awake_modes <= st.total_modes);
    pm_engine_destroy(eng);
    pm_engine_destroy(NULL);

    /* App host (API 0.6), from C: MIDI bytes in, audio out, meters. */
    {
        pm_host* h = pm_host_create();
        pm_host_load load;
        pm_host_counters hc;
        pm_midi_monitor_entry me;
        const uint8_t on[3] = {0x90, 60, 100};
        const uint32_t ump = 0x20B04000u; /* CC64 = 0 */
        EXPECT(h != NULL);
        pm_host_set_timebase(h, 125, 3);
        EXPECT(pm_engine_set_param(pm_host_engine(h), PM_PARAM_SOUNDBOARD_ENABLED, 0.0) == 1);
        EXPECT(pm_host_prepare(h, 48000.0, 256) == 1 && pm_host_is_prepared(h) == 1);
        pm_host_midi_bytes(h, on, 3, 0, 1);
        pm_host_midi_ump(h, &ump, 1, 0, 1);
        EXPECT(pm_host_monitor_pop(h, &me) == 1 && me.kind == PM_MIDI_NOTE_ON && me.data1 == 60);
        peak = 0.0f;
        for (b = 0; b < 20; ++b) {
            pm_host_render(h, left, right, 256, 0);
            for (i = 0; i < 256; ++i)
                if (fabsf(left[i]) > peak) peak = fabsf(left[i]);
        }
        EXPECT(peak > 0.001f);
        pm_host_get_load(h, &load);
        EXPECT(load.buffers == 20 && load.budget > 0.0);
        pm_host_get_counters(h, &hc);
        EXPECT(hc.midi_messages == 2 && hc.midi_events == 2);
        /* Diagnostics (API 0.7). */
        {
            pm_host_diagnostics d;
            pm_host_set_cpu_timing(h, 1);
            pm_host_render(h, left, right, 100, 0);
            pm_host_get_diagnostics(h, &d);
            EXPECT(d.buffers == 21 && d.last_frames == 100 && d.min_frames == 100 && d.max_frames == 256);
            EXPECT(d.distinct_frame_counts == 2 && d.sample_rate == 48000.0);
            EXPECT(d.max_us >= d.p99_us && d.p99_us > 0.0 && d.mean_us > 0.0);
            EXPECT(d.worst_index >= 1 && d.worst_frames > 0 && d.worst_load == d.max_load);
            EXPECT(d.worst_hammers_active >= 0 && d.worst_sub_blocks >= 1);
            printf("  diagnostics: mean %.1f us, p99 %.1f us, max %.1f us (%d frames, %d sub-blocks), "
                   "cpu timing %d, stall %.1f us\n",
                   d.mean_us, d.p99_us, d.max_us, d.worst_frames, d.worst_sub_blocks, d.cpu_timing,
                   d.max_stall_us);
            pm_host_reset_load(h);
            pm_host_get_diagnostics(h, &d);
            EXPECT(d.buffers == 0);
        }
        pm_host_destroy(h);
    }

    /* Build information and the offline benchmark (API 0.7). */
    {
        pm_bench_config bc = {0};
        pm_bench_result br;
        const char* info = pm_build_info();
        EXPECT(info != NULL && info[0] != 0);
        printf("  build: %s\n", info);
        EXPECT(pm_bench_item_count(&bc) == 15);
        bc.flags = PM_BENCH_ALL_TIERS;
        EXPECT(pm_bench_item_count(&bc) == 21);
        bc.flags = PM_BENCH_NO_HAMMER;
        EXPECT(pm_bench_item_count(&bc) == 7);
        EXPECT(pm_bench_item_name(&bc, 7)[0] == 0);
        bc.flags = 0;
        EXPECT(pm_bench_item_name(&bc, 3)[0] == 'c');  /* cluster */
        EXPECT(pm_bench_item_name(&bc, 10)[0] == 'h'); /* hammer, key 40 */
        bc.hammer_strikes = 3;
        EXPECT(pm_bench_run_item(&bc, 10, &br) == 1);
        EXPECT(br.ok == 1 && br.kind == PM_BENCH_HAMMER && br.key == 40 && br.strings == 3 &&
               br.oversampling == 2 && br.contact_us > 0.0 && br.life_us > 0.0);
        bc.seconds = 0.05;
        EXPECT(pm_bench_run_item(&bc, 6, &br) == 1); /* cluster, board off */
        EXPECT(br.ok == 1 && br.kind == PM_BENCH_SCENARIO && br.max_us >= br.p99_us && br.p99_us > 0.0);
        EXPECT(br.mean_load > 0.0 && fabs(br.period_us - 1e6 * 64 / 48000.0) < 1e-9);
        printf("  bench item 6: mean %.1f us, p99 %.1f us, max %.1f us\n", br.mean_us, br.p99_us, br.max_us);
        EXPECT(pm_bench_run_item(&bc, 99, &br) == 0);
    }
    printf("%d failure(s)\n", failures);
    return failures == 0 ? 0 : 1;
}
