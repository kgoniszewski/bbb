set(PM_TESTS
  test_spsc_queue
  test_denormals
  test_string_accuracy
  test_kernels
  test_culling
  test_coefficient_change)

find_package(Threads REQUIRED)
foreach(t IN LISTS PM_TESTS)
  add_executable(${t} ${t}.cpp)
  target_link_libraries(${t} PRIVATE pm_string Threads::Threads)
  add_test(NAME ${t} COMMAND ${t})
endforeach()
