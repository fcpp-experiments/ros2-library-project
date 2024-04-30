cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
option(FCPP_INTERNAL_TESTS "Build internal tests for FCPP." OFF)
add_subdirectory(./fcpp/src)

macro(fcpp_setupx)
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED True)
    set(CMAKE_CXX_FLAGS_DEBUG   "${CMAKE_CXX_FLAGS_DEBUG}   -g")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG")
    if(NOT CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "arm64")
        # -march=native is not supported by arm64
        set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native -pthread")
    endif()
    if(WIN32)
        set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Os")
        set(CMAKE_FIND_LIBRARY_SUFFIXES ".a")
        set(BUILD_SHARED_LIBS OFF)
        set(CMAKE_EXE_LINKER_FLAGS "-static")
        if(FCPP_BUILD_GL)
            set(CMAKE_DISABLE_FIND_PACKAGE_BrotliDec True)
        endif(FCPP_BUILD_GL)
    endif(WIN32)
    if(FCPP_BUILD_TESTS)
        file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test)
    endif(FCPP_BUILD_TESTS)
    file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/run)
endmacro()
fcpp_setupx()

project(
    AP_Engine
    VERSION 0.0.0
    DESCRIPTION "AP engine for the PoC demo."
)

function(fcpp_targetx target_executable enable_gui extra_sources)
    get_filename_component(target_name ${target_executable} NAME_WE)
    add_executable(${target_name} ${target_executable} ${extra_sources})
    target_link_libraries(${target_name} PRIVATE fcpp)
    target_include_directories(${target_name} PRIVATE .)
    set_target_properties(${target_name} PROPERTIES PREFIX "run/")
    if(${enable_gui} STREQUAL ON)
        if(FCPP_BUILD_GL)
            fcpp_copy(${target_name} ${PROJECT_SOURCE_DIR} ${CMAKE_BINARY_DIR} textures)
            target_compile_options(${target_name} PUBLIC -DFCPP_CONFIGURATION=FCPP_CONFIGURATION_DEPENDENT -DFCPP_GUI)
        endif()
    endif()
    fcpp_copy(${target_name} ${PROJECT_SOURCE_DIR} ${CMAKE_BINARY_DIR} input)
endfunction()

set(
    sources
    lib/common_data.cpp
    lib/feedback_parser.cpp
    lib/goal_parser.cpp
    lib/action_writer.cpp
    lib/poc_file_watcher.cpp
    lib/file_watcher.hpp
    lib/file_utils.cpp
    lib/poc_utils.cpp
)

set(CMAKE_CXX_STANDARD 17)

# AP engine (with file-watcher).
fcpp_targetx("./run/ap_engine.cpp" ON "${sources}")

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
