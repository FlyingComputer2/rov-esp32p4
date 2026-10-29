# Host tests for components/rover_sample/core (portable C++, no ESP-IDF).
cmake_minimum_required(VERSION 3.16)
project(rover_sample_tests CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CORE ${CMAKE_CURRENT_LIST_DIR}/../components/rover_sample/core)
add_executable(rover_sample_test
    rover_sample_test.cpp
    ${CORE}/msp.cpp
    ${CORE}/protocol.cpp
    ${CORE}/params.cpp
    ${CORE}/controller.cpp
    ${CORE}/ahrs.cpp
    ${CORE}/rc_link.cpp)
target_include_directories(rover_sample_test PRIVATE ${CORE})
target_compile_options(rover_sample_test PRIVATE -Wall -Wextra -Werror -fsanitize=address,undefined)
target_link_options(rover_sample_test PRIVATE -fsanitize=address,undefined)
