add_executable(bench_modal bench_modal.cpp)
target_link_libraries(bench_modal PRIVATE pm_string)
add_executable(bench_engine bench_engine.cpp)
target_link_libraries(bench_engine PRIVATE pm_engine)
