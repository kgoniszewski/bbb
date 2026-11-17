add_library(pm_room STATIC room.cpp)
target_include_directories(pm_room PUBLIC ${PROJECT_SOURCE_DIR}/core)
target_link_libraries(pm_room PUBLIC pm_base)
# Real-time code: no exceptions, no RTTI.
if(NOT MSVC)
  target_compile_options(pm_room PRIVATE -fno-exceptions -fno-rtti)
endif()
