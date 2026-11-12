# Offline benchmark scenarios (non-real-time), for the C API (app) and
# bench_engine (which turns the engine's run-time phase timing on).
add_library(pm_bench STATIC bench.cpp)
target_include_directories(pm_bench PUBLIC ${PROJECT_SOURCE_DIR}/core)
target_link_libraries(pm_bench PUBLIC pm_engine pm_voicing)
