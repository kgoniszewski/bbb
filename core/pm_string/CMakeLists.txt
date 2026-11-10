add_library(pm_string STATIC
  mode_bank.cpp
  modal_kernels.cpp
  string_modes.cpp)
target_include_directories(pm_string PUBLIC ${PROJECT_SOURCE_DIR}/core)
target_link_libraries(pm_string PUBLIC pm_base)
# Real-time code: no exceptions, no RTTI.
if(NOT MSVC)
  target_compile_options(pm_string PRIVATE -fno-exceptions -fno-rtti)
endif()
