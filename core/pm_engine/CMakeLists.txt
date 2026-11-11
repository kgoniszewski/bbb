add_library(pm_engine STATIC engine.cpp)
target_include_directories(pm_engine PUBLIC ${PROJECT_SOURCE_DIR}/core)
target_link_libraries(pm_engine PUBLIC pm_voicing pm_hammer pm_string)
# Real-time code: no exceptions, no RTTI.
if(NOT MSVC)
  target_compile_options(pm_engine PRIVATE -fno-exceptions -fno-rtti)
endif()

# Profiling variant for bench_engine only: the same engine with per-phase
# timers (PM_PROFILE, EngineProfile in engine.h). Never linked by the app,
# the C API or the tests, so the real-time library carries no timing calls.
add_library(pm_engine_profile STATIC EXCLUDE_FROM_ALL engine.cpp)
target_include_directories(pm_engine_profile PUBLIC ${PROJECT_SOURCE_DIR}/core)
target_link_libraries(pm_engine_profile PUBLIC pm_voicing pm_hammer pm_string)
target_compile_definitions(pm_engine_profile PUBLIC PM_PROFILE)
if(NOT MSVC)
  target_compile_options(pm_engine_profile PRIVATE -fno-exceptions -fno-rtti)
endif()
