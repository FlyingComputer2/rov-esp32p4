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

set(RELAY ${CMAKE_CURRENT_LIST_DIR}/../components/mplane_relay)
add_executable(mplane_relay_test mplane_relay_test.cpp ${RELAY}/mplane_relay.cpp)
target_include_directories(mplane_relay_test PRIVATE ${RELAY}/include)
target_compile_options(mplane_relay_test PRIVATE -Wall -Wextra -Werror -fsanitize=address,undefined)
target_link_options(mplane_relay_test PRIVATE -fsanitize=address,undefined)
find_package(Threads REQUIRED)
target_link_libraries(mplane_relay_test PRIVATE Threads::Threads)

# udp_ota core: unit tests, and a host server for tools/udp_ota.py.
set(OTA_CORE ${CMAKE_CURRENT_LIST_DIR}/../components/udp_ota/core)
foreach(t udp_ota_test udp_ota_host_server)
    add_executable(${t} ${t}.cpp ${OTA_CORE}/server.cpp ${OTA_CORE}/fs.cpp)
    target_include_directories(${t} PRIVATE ${OTA_CORE})
    target_compile_options(${t} PRIVATE -Wall -Wextra -Werror -fsanitize=address,undefined)
    target_link_options(${t} PRIVATE -fsanitize=address,undefined)
endforeach()
