add_library(pm_board STATIC soundboard.cpp)
target_include_directories(pm_board PUBLIC ${PROJECT_SOURCE_DIR}/core)
target_link_libraries(pm_board PUBLIC pm_string)
# Real-time code: no exceptions, no RTTI.
if(NOT MSVC)
  target_compile_options(pm_board PRIVATE -fno-exceptions -fno-rtti)
endif()
