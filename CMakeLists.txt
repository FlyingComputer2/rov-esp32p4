# Rover remote on the ESP32-P4: the winject SDIO proxy (C6 radio on UDP
# 22 / 9000), winject-l3 (winject-manager) talking to it over loopback, and
# the vstreamer component (video pipeline; not started yet).
# Local checkouts: -DFETCHCONTENT_SOURCE_DIR_WINJECT_L3=<dir>,
# -DFETCHCONTENT_SOURCE_DIR_WINJECT_SDIO_PROXY=<dir>,
# -DFETCHCONTENT_SOURCE_DIR_VSTREAMER=<dir>.
cmake_minimum_required(VERSION 3.16)
if(NOT DEFINED PROJECT_VER)
    set(PROJECT_VER "0.1.0")
endif()

include(FetchContent)
FetchContent_Declare(winject_l3
    GIT_REPOSITORY https://github.com/winject-wireless/winject-l3.git
    GIT_TAG c8e97486b82f72b6d818833d676477b01d83181c
)
FetchContent_Declare(winject_sdio_proxy
    GIT_REPOSITORY https://github.com/winject-wireless/winject-radio-esp32-sdio-proxy.git
    GIT_TAG cf4afeb760ba2ac0ce9b75addd5ecac068e74d11
)
# vstreamer and winject-l3 share the bfc and isa-l FetchContent names; whichever
# is declared first wins, so keep their pins equal.
FetchContent_Declare(vstreamer
    GIT_REPOSITORY https://github.com/FlyingComputer2/vstreamer.git
    GIT_TAG f8ad5eeaf7511c0311e0a5dc7b15f2ee52432f05
)
foreach(dep winject_l3 winject_sdio_proxy vstreamer)
    FetchContent_GetProperties(${dep})
    if(NOT ${dep}_POPULATED)
        FetchContent_Populate(${dep})
    endif()
endforeach()
include(${winject_sdio_proxy_SOURCE_DIR}/cmake/winject_sdio_proxy.cmake)

set(EXTRA_COMPONENT_DIRS
    ${winject_l3_SOURCE_DIR}/idf/winject_l3
    ${vstreamer_SOURCE_DIR}/idf/vstreamer
    ${WINJECT_SDIO_PROXY_COMPONENT_DIRS})
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rover-remote-esp32p4)
