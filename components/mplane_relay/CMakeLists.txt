# mplane_relay: rover half of the multi-client m-plane relay (one loopback
# socket per ground client id). Portable POSIX sockets; host-tested (tests/).
idf_component_register(
    SRCS mplane_relay.cpp
    INCLUDE_DIRS include
    PRIV_REQUIRES lwip
)
target_compile_options(${COMPONENT_LIB} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17>)
