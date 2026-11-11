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
    printf("%d failure(s)\n", failures);
    return failures == 0 ? 0 : 1;
}
