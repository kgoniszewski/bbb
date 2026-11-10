# Offline finite-difference reference (not real-time).
add_library(pm_fdtd STATIC fdtd_string.cpp)
target_include_directories(pm_fdtd PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(pm_fdtd PUBLIC pm_options)

if(PM_BUILD_TOOLS)
  add_executable(reference_fdtd reference_fdtd.cpp)
  target_link_libraries(reference_fdtd PRIVATE pm_fdtd pm_voicing)
endif()
