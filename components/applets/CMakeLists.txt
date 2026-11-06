# applets: app_bwtest and the krono client / server on a named reactor
# (bfc::default_reactor<>), driving the libraries' classes instead of their
# *_main loops. bfc is the project's FetchContent declaration; not populated
# during IDF's early requirement scan.
if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    include(FetchContent)
    FetchContent_GetProperties(bfc)
    if(NOT bfc_POPULATED)
        FetchContent_Populate(bfc)
    endif()
endif()

idf_component_register(
    SRCS applets.cpp
    INCLUDE_DIRS include compat
    REQUIRES winject_l3 app_bwtest krono pthread vfs
    PRIV_REQUIRES lwip log
)
if(NOT CMAKE_BUILD_EARLY_EXPANSION)
    target_include_directories(${COMPONENT_LIB} PUBLIC ${bfc_SOURCE_DIR}/src)
endif()
target_compile_options(${COMPONENT_LIB} PRIVATE $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17 -fexceptions>)
