cmake_minimum_required(VERSION 3.5)

project(dpm C)

file(GLOB SOURCES "src/*.c")


add_library(dpm STATIC ${SOURCES})
# dpm-status is found in status/ecm or status/sm
# (only one can be active in the project)
target_link_libraries(dpm cia dpm-status)
target_include_directories(dpm PUBLIC inc dpm-status)


