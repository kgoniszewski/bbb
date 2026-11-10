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

    e.type = PM_EVENT_NOTE_ON;
    e.time_base = PM_TIME_SAMPLE;
    e.time = 100;
    e.note = 60;
    e.value = 50000;
    EXPECT(pm_engine_push_event(eng, &e) == 1);
    for (b = 0; b < 40; ++b) {
        pm_engine_render(eng, left, right, 256, NULL);
        for (i = 0; i < 256; ++i)
            if (fabsf(left[i]) > peak) peak = fabsf(left[i]);
    }
    pm_engine_get_stats(eng, &st);
    printf("  %s: %d modes, peak %.4f\n", pm_version_string(), st.total_modes, peak);
    EXPECT(peak > 0.001f && peak < 1.0f);
    EXPECT(st.total_modes > 1000);
    pm_engine_destroy(eng);
    pm_engine_destroy(NULL);
    printf("%d failure(s)\n", failures);
    return failures == 0 ? 0 : 1;
}
