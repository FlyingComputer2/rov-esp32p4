# applets: app_bwtest and the krono client on a named app reactor (app_kit),
# driving the libraries' classes instead of their *_main loops.
idf_component_register(
    SRCS applets.cpp
    INCLUDE_DIRS include
    REQUIRES app_kit winject_l3 app_bwtest krono
    PRIV_REQUIRES log
)
target_compile_options(${COMPONENT_LIB} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17 -fexceptions>)
