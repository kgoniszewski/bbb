# Offline benchmark scenarios (non-real-time), for the C API (app) and
# bench_engine. pm_bench_profile is the same code against the profiling
# engine (PM_PROFILE, per-phase breakdown); never linked by the app.
add_library(pm_bench STATIC bench.cpp)
target_include_directories(pm_bench PUBLIC ${PROJECT_SOURCE_DIR}/core)
target_link_libraries(pm_bench PUBLIC pm_engine pm_voicing)

add_library(pm_bench_profile STATIC EXCLUDE_FROM_ALL bench.cpp)
target_include_directories(pm_bench_profile PUBLIC ${PROJECT_SOURCE_DIR}/core)
target_link_libraries(pm_bench_profile PUBLIC pm_engine_profile pm_voicing)
