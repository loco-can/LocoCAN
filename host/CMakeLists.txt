add_library(loco_sim
    src/can_frame.cpp
    src/virtual_bus.cpp
    src/protocol.cpp
    src/serialcan.cpp
    src/modules.cpp
    src/scenarios.cpp
)

target_include_directories(loco_sim
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/src
)

target_compile_options(loco_sim PRIVATE -Wall -Wextra -Wpedantic)

add_executable(loco-sim src/sim_main.cpp)
target_link_libraries(loco-sim PRIVATE loco_sim)
target_compile_options(loco-sim PRIVATE -Wall -Wextra -Wpedantic)

add_executable(loco_host_tests
    tests/test_main.cpp
    tests/test_headers.cpp
    tests/test_protocol.cpp
    tests/test_bus.cpp
    tests/test_serialcan.cpp
    tests/test_scenarios.cpp
)
target_link_libraries(loco_host_tests PRIVATE loco_sim)
target_include_directories(loco_host_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/tests)
target_compile_options(loco_host_tests PRIVATE -Wall -Wextra -Wpedantic)

add_test(NAME loco_host_tests COMMAND loco_host_tests)
add_test(NAME loco_sim_all_scenarios COMMAND loco-sim --scenario all --quiet)
