# Rover remote on the ESP32-P4: the winject SDIO proxy (C6 radio), winject-l3
# (winject-manager) and vstreamer live TX (camera -> H.264 -> stream_sender),
# joined by in-process bfc queues (main/main.cpp).
# Local checkouts: -DFETCHCONTENT_SOURCE_DIR_WINJECT_L3=<dir>,
# -DFETCHCONTENT_SOURCE_DIR_WINJECT_SDIO_PROXY=<dir>,
# -DFETCHCONTENT_SOURCE_DIR_VSTREAMER=<dir>, -DFETCHCONTENT_SOURCE_DIR_KRONO=<dir>,
# -DFETCHCONTENT_SOURCE_DIR_SYS_APPLET=<dir>, -DFETCHCONTENT_SOURCE_DIR_WINJECT_FEC=<dir>,
# -DFETCHCONTENT_SOURCE_DIR_UDPFS=<dir>.
cmake_minimum_required(VERSION 3.16)
if(NOT DEFINED PROJECT_VER)
    set(PROJECT_VER "0.1.0")
endif()

include(FetchContent)
# One bfc for the whole firmware: the dependencies share its queue and reactor
# registries. BFC_PIN is checked against each dependency's own pin
# (BfcFetch.cmake), and BFC_CONFIG_DIR holds this firmware's bfc_config.hpp
# (function type, allocator, poll reactor), which every bfc user compiles with.
set(BFC_PIN 804b986b10d142c2f6ed0cd93bfbfcad8c73c8a6)
set(BFC_CONFIG_DIR ${CMAKE_CURRENT_LIST_DIR}/bfc_config)
FetchContent_Declare(bfc
    GIT_REPOSITORY https://github.com/therooftopprinz/BFC.git
    GIT_TAG ${BFC_PIN}
)
FetchContent_Declare(winject_l3
    GIT_REPOSITORY https://github.com/winject-wireless/winject-l3.git
    GIT_TAG 70d3532fb19c127639d4a078456ea36dd37cb265
)
FetchContent_Declare(winject_sdio_proxy
    GIT_REPOSITORY https://github.com/winject-wireless/winject-radio-esp32-sdio-proxy.git
    GIT_TAG e625385fd5034d56551b137d5622373ecee54dd6
)
# vstreamer sends the video; FEC is winject-l3's (winject-fec, below).
FetchContent_Declare(vstreamer
    GIT_REPOSITORY https://github.com/FlyingComputer2/vstreamer.git
    GIT_TAG 07ef91818a25afeb5d83a1a27215e14a7c96b294
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
    GIT_TAG 0c34abc0b6b11ed8a1387e50ae376d71de89033d
)
# Remote file access (udpfs: upstream 11, UDP 2440). It uses this firmware's
# bfc and pins none; udpfs_config/ is this firmware's errno / flag map.
FetchContent_Declare(udpfs
    GIT_REPOSITORY https://github.com/FlyingComputer2/udpfs.git
    GIT_TAG f755977a1400ecd9a7e59266f990ddd7b9299e3e
)
set(UDPFS_CONFIG_DIR ${CMAKE_CURRENT_LIST_DIR}/udpfs_config)
foreach(dep winject_l3 winject_sdio_proxy vstreamer krono sys_applet udpfs)
    FetchContent_GetProperties(${dep})
    if(NOT ${dep}_POPULATED)
        FetchContent_Populate(${dep})
    endif()
endforeach()
include(${winject_sdio_proxy_SOURCE_DIR}/cmake/winject_sdio_proxy.cmake)
# winject-fec, the FEC behind winject-l3's upstreams, at l3's pin (checked there).
set(WINJECT_FEC_PIN c4c9b4c7685a9a7d88ea26d9c0b31ab52b57da55)
include(${winject_l3_SOURCE_DIR}/cmake/WinjectFecFetch.cmake)

set(EXTRA_COMPONENT_DIRS
    ${winject_l3_SOURCE_DIR}/idf/winject_l3
    ${WINJECT_FEC_SOURCE_DIR}/idf/winject_fec
    ${winject_l3_SOURCE_DIR}/idf/app_bwtest
    ${krono_SOURCE_DIR}/idf/krono
    ${sys_applet_SOURCE_DIR}/idf/sys_applet
    ${udpfs_SOURCE_DIR}/idf/udpfs
    ${vstreamer_SOURCE_DIR}/idf/vstreamer
    ${WINJECT_SDIO_PROXY_COMPONENT_DIRS})
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(rov-esp32p4)
