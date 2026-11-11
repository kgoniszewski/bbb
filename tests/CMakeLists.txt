set(PM_TESTS
  test_spsc_queue
  test_eigen
  test_unison
  test_double_decay
  test_pedals
  test_tuning
  test_denormals
  test_string_accuracy
  test_kernels
  test_culling
  test_coefficient_change
  test_bank_segments
  test_hammer
  test_engine_sound
  test_damper
  test_scheduling
  test_engine_stress
  test_rt_alloc
  test_soundboard_model
  test_soundboard_coupling
  test_passivity
  test_sympathetic
  test_half_pedal)

find_package(Threads REQUIRED)
foreach(t IN LISTS PM_TESTS)
  add_executable(${t} ${t}.cpp)
  target_link_libraries(${t} PRIVATE pm_engine Threads::Threads)
  add_test(NAME ${t} COMMAND ${t})
endforeach()

# The C API must compile as C.
add_executable(test_c_api test_c_api.c)
target_link_libraries(test_c_api PRIVATE pm_api)
set_target_properties(test_c_api PROPERTIES LINKER_LANGUAGE CXX)
add_test(NAME test_c_api COMMAND test_c_api)

# Cross-check against the offline FDTD reference.
add_executable(test_fdtd test_fdtd.cpp)
target_link_libraries(test_fdtd PRIVATE pm_engine pm_fdtd Threads::Threads)
add_test(NAME test_fdtd COMMAND test_fdtd)

# App host: MIDI parsing, load meter, host glue through the C API.
foreach(t test_midi_parser test_load_meter)
  add_executable(${t} ${t}.cpp)
  target_link_libraries(${t} PRIVATE pm_host Threads::Threads)
  add_test(NAME ${t} COMMAND ${t})
endforeach()
add_executable(test_host test_host.cpp)
target_link_libraries(test_host PRIVATE pm_api Threads::Threads)
add_test(NAME test_host COMMAND test_host)
