# mplane_relay: rover half of the multi-client m-plane relay (one loopback
# socket per ground client id). mplane_relay.cpp is portable POSIX and
# host-tested (tests/); src/applet.cpp attaches it to a named app reactor.
idf_component_register(
    SRCS mplane_relay.cpp
         src/applet.cpp
    INCLUDE_DIRS include
    REQUIRES app_kit
    PRIV_REQUIRES lwip log esp_timer
)
target_compile_options(${COMPONENT_LIB} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>)
