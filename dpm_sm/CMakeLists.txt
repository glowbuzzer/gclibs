cmake_minimum_required(VERSION 3.5)

project(dpm C)

file(GLOB SOURCES "src/*.c")


add_library(dpm STATIC ${SOURCES})
target_link_libraries(dpm sm_status cia)
target_include_directories(dpm PUBLIC inc)


