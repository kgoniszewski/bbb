# Non-real-time: parameter tables -> coefficient tables.
add_library(pm_voicing STATIC piano_table.cpp key_loader.cpp)
target_include_directories(pm_voicing PUBLIC ${PROJECT_SOURCE_DIR}/core)
target_link_libraries(pm_voicing PUBLIC pm_string pm_hammer)
