add_library(pm_engine STATIC engine.cpp)
target_include_directories(pm_engine PUBLIC ${PROJECT_SOURCE_DIR}/core)
target_link_libraries(pm_engine PUBLIC pm_voicing pm_hammer pm_string)
# Real-time code: no exceptions, no RTTI.
if(NOT MSVC)
  target_compile_options(pm_engine PRIVATE -fno-exceptions -fno-rtti)
endif()
