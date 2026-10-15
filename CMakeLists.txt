# Rover remote on the ESP32-P4: the winject SDIO proxy (C6 radio on UDP
# 22 / 9000) and winject-l3 (winject-manager) talking to it over loopback.
# Local checkouts: -DFETCHCONTENT_SOURCE_DIR_WINJECT_L3=<dir>,
# -DFETCHCONTENT_SOURCE_DIR_WINJECT_SDIO_PROXY=<dir>.
cmake_minimum_required(VERSION 3.16)
if(NOT DEFINED PROJECT_VER)
    set(PROJECT_VER "0.1.0")
endif()

include(FetchContent)
FetchContent_Declare(winject_l3
    GIT_REPOSITORY https://github.com/winject-wireless/winject-l3.git
    GIT_TAG 82c338152326040dcd381083c947cb3417b53e2d
)
FetchContent_Declare(winject_sdio_proxy
    GIT_REPOSITORY https://github.com/winject-wireless/winject-radio-esp32-sdio-proxy.git
    GIT_TAG cf4afeb760ba2ac0ce9b75addd5ecac068e74d11
)
foreach(dep winject_l3 winject_sdio_proxy)
    FetchContent_GetProperties(${dep})
    if(NOT ${dep}_POPULATED)
        FetchContent_Populate(${dep})
    endif()
endforeach()
include(${winject_sdio_proxy_SOURCE_DIR}/cmake/winject_sdio_proxy.cmake)

set(EXTRA_COMPONENT_DIRS
    ${winject_l3_SOURCE_DIR}/idf/winject_l3
    ${WINJECT_SDIO_PROXY_COMPONENT_DIRS})
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rover-remote-esp32-p4)
