# Rover remote on the ESP32-P4: the winject SDIO proxy (C6 radio), winject-l3
# (winject-manager) and vstreamer live TX (camera -> H.264 -> stream_sender),
# joined by in-process bfc queues (main/main.cpp).
# Local checkouts: -DFETCHCONTENT_SOURCE_DIR_WINJECT_L3=<dir>,
# -DFETCHCONTENT_SOURCE_DIR_WINJECT_SDIO_PROXY=<dir>,
# -DFETCHCONTENT_SOURCE_DIR_VSTREAMER=<dir>, -DFETCHCONTENT_SOURCE_DIR_KRONO=<dir>,
# -DFETCHCONTENT_SOURCE_DIR_SYS_APPLET=<dir>.
cmake_minimum_required(VERSION 3.16)
if(NOT DEFINED PROJECT_VER)
    set(PROJECT_VER "0.1.0")
endif()

include(FetchContent)
# One bfc for the whole firmware: the dependencies share its queue and reactor
# registries. BFC_PIN is checked against each dependency's own pin
# (BfcFetch.cmake), and BFC_CONFIG_DIR holds this firmware's bfc_config.hpp
# (function type, allocator, poll reactor), which every bfc user compiles with.
set(BFC_PIN 8dc02add56771bf05f60519b3915c8fc9464d24b)
set(BFC_CONFIG_DIR ${CMAKE_CURRENT_LIST_DIR}/bfc_config)
FetchContent_Declare(bfc
    GIT_REPOSITORY https://github.com/therooftopprinz/BFC.git
    GIT_TAG ${BFC_PIN}
)
FetchContent_Declare(winject_l3
    GIT_REPOSITORY https://github.com/winject-wireless/winject-l3.git
    GIT_TAG 4255053470eb485356b9623e578ecd384165a03a
)
FetchContent_Declare(winject_sdio_proxy
    GIT_REPOSITORY https://github.com/winject-wireless/winject-radio-esp32-sdio-proxy.git
    GIT_TAG 1b2411580697b4fb36e5d4611f3b918620c496d6
)
# vstreamer and winject-l3 also share the isa-l FetchContent name; whichever
# is declared first wins, so keep their pins equal.
FetchContent_Declare(vstreamer
    GIT_REPOSITORY https://github.com/FlyingComputer2/vstreamer.git
    GIT_TAG a5004e1bf8157d5b33f3b2ef00dfaca4dac7b138
)
# Clock sync (rover = SynchClient on L3 upstream 0).
FetchContent_Declare(krono
    GIT_REPOSITORY https://github.com/FlyingComputer2/krono.git
    GIT_TAG c7dc5caaca89d93467ddb2dfb349a8f7ef30f005
)
# The system applet (get sys cpu / heap / stack / ...; console 5130, m-plane
# upstream 10). It uses this firmware's bfc and pins none.
FetchContent_Declare(sys_applet
    GIT_REPOSITORY https://github.com/FlyingComputer2/sys-applet-esp32p4.git
    GIT_TAG 00c16e1fe60f359a7f25eb68839de60d2fa16f65
)
foreach(dep winject_l3 winject_sdio_proxy vstreamer krono sys_applet)
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
    ${sys_applet_SOURCE_DIR}/idf/sys_applet
    ${vstreamer_SOURCE_DIR}/idf/vstreamer
    ${WINJECT_SDIO_PROXY_COMPONENT_DIRS})
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rov-esp32p4)
