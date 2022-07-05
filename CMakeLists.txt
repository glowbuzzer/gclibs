# stop if cmake version below 3.12
cmake_minimum_required(VERSION 3.12)

project(gclibs)

message(STATUS "GB: This is a dummy build of clibs, we don't use the output from this build!")
message(STATUS "GB: Using cmake version [${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION}.${CMAKE_PATCH_VERSION}]")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -Wsign-conversion -fPIC -std=gnu99 -DGB_APP_LINUX -DLOG_LEVEL=0 -DENABLE_LOGGING=0")

set(CMAKE_C_FLAGS_DEBUG "-Og -g3")
set(CMAKE_C_FLAGS_RELEASE "-O3")

message(STATUS "GB: PROJECT_BINARY_DIR [${PROJECT_BINARY_DIR} ]")
message(STATUS "GB: CURRENT_SOURCE_DIR [${CMAKE_CURRENT_SOURCE_DIR} ]")

add_subdirectory("common-misc")
add_subdirectory("bcm2835")
add_subdirectory("logging")
add_subdirectory("pid")
add_subdirectory("cia")
add_subdirectory("dpm")
add_subdirectory("linux-shm")
add_subdirectory("sleep")
add_subdirectory("gberror")
add_subdirectory("minini")
add_subdirectory("state_machine")
add_subdirectory("unity")

if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR “GB: In-source builds not allowed! Create a build directory and run CMake from there.”)
endif ()
