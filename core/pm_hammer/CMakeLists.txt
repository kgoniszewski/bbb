add_library(pm_hammer STATIC hammer.cpp)
target_include_directories(pm_hammer PUBLIC ${PROJECT_SOURCE_DIR}/core)
target_link_libraries(pm_hammer PUBLIC pm_string)
# Real-time code: no exceptions, no RTTI.
if(NOT MSVC)
  target_compile_options(pm_hammer PRIVATE -fno-exceptions -fno-rtti)
endif()
