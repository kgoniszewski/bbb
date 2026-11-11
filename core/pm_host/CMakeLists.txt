# App host glue (MIDI parsing, event queues, prepare gate, load meter):
# real-time code, portable, no platform dependencies.
find_package(Threads REQUIRED)
add_library(pm_host STATIC midi.cpp host.cpp)
target_include_directories(pm_host PUBLIC ${PROJECT_SOURCE_DIR}/core)
target_link_libraries(pm_host PUBLIC pm_engine Threads::Threads)
if(NOT MSVC)
  target_compile_options(pm_host PRIVATE -fno-exceptions -fno-rtti)
endif()
