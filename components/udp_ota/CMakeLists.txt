# udp_ota: P4 app update and config-file access over UDP (Ethernet, or the
# air via an L3 upstream).
# core/ is portable C++ and host-tested (tests/); src/ is the ESP-IDF side.
idf_component_register(
    SRCS core/server.cpp
         core/fs.cpp
         src/udp_ota.cpp
    INCLUDE_DIRS include
    PRIV_INCLUDE_DIRS core
    PRIV_REQUIRES app_update esp_partition esp_app_format esp_timer lwip log esp_system freertos
)
target_compile_options(${COMPONENT_LIB} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>)
