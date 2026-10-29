# Rover remote on the ESP32-P4: the winject SDIO proxy (C6 radio), winject-l3
# (winject-manager) and vstreamer live TX (camera -> H.264 -> stream_sender),
# joined by in-process bfc queues (main/main.cpp).
# Local checkouts: -DFETCHCONTENT_SOURCE_DIR_WINJECT_L3=<dir>,
# -DFETCHCONTENT_SOURCE_DIR_WINJECT_SDIO_PROXY=<dir>,
# -DFETCHCONTENT_SOURCE_DIR_VSTREAMER=<dir>, -DFETCHCONTENT_SOURCE_DIR_KRONO=<dir>.
cmake_minimum_required(VERSION 3.16)
if(NOT DEFINED PROJECT_VER)
    set(PROJECT_VER "0.1.0")
endif()

include(FetchContent)
# The three share bfc's queue and reactor registries, so they must build
# against one bfc: FetchContent keeps the first declaration, and this one
# comes before theirs. Keep it equal to their pins.
FetchContent_Declare(bfc
    GIT_REPOSITORY https://github.com/therooftopprinz/BFC.git
    GIT_TAG f6fe9784bb8379eaa72a1597fe9e9638b97fb5fe
)
FetchContent_Declare(winject_l3
    GIT_REPOSITORY https://github.com/winject-wireless/winject-l3.git
    GIT_TAG ffc3f403f00d2fb82969ec91b4faf9d6ba283faa
)
FetchContent_Declare(winject_sdio_proxy
    GIT_REPOSITORY https://github.com/winject-wireless/winject-radio-esp32-sdio-proxy.git
    GIT_TAG 0dc783a1b0fd1aadd9c88620af47dde5e8150630
)
# vstreamer and winject-l3 also share the isa-l FetchContent name; whichever
# is declared first wins, so keep their pins equal.
FetchContent_Declare(vstreamer
    GIT_REPOSITORY https://github.com/FlyingComputer2/vstreamer.git
    GIT_TAG df5de95e9b79d8d99582e2dbae5fd0eabb400cee
)
# Clock sync (rover = SynchClient on L3 upstream 0).
FetchContent_Declare(krono
    GIT_REPOSITORY https://github.com/FlyingComputer2/krono.git
    GIT_TAG c7dc5caaca89d93467ddb2dfb349a8f7ef30f005
)
foreach(dep winject_l3 winject_sdio_proxy vstreamer krono)
    FetchContent_GetProperties(${dep})
    if(NOT ${dep}_POPULATED)
        FetchContent_Populate(${dep})
    endif()
endforeach()
include(${winject_sdio_proxy_SOURCE_DIR}/cmake/winject_sdio_proxy.cmake)

set(EXTRA_COMPONENT_DIRS
    ${winject_l3_SOURCE_DIR}/idf/winject_l3
    ${winject_l3_SOURCE_DIR}/idf/app_bwtest
    ${krono_SOURCE_DIR}/idf/krono
    ${vstreamer_SOURCE_DIR}/idf/vstreamer
    ${WINJECT_SDIO_PROXY_COMPONENT_DIRS})
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rov-esp32p4)
