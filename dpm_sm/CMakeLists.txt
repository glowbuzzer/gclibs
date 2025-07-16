cmake_minimum_required(VERSION 3.5)

project(dpm_sm C)

file(GLOB SOURCES "src/*.c")


add_library(dpm_sm STATIC ${SOURCES})
target_link_libraries(dpm_sm sm_status cia)
target_include_directories(dpm_sm PUBLIC inc)


