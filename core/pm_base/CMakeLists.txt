# Header-only: aligned storage, SIMD wrappers, denormal guard, SPSC queue.
add_library(pm_base INTERFACE)
target_include_directories(pm_base INTERFACE ${PROJECT_SOURCE_DIR}/core)
target_link_libraries(pm_base INTERFACE pm_options)
