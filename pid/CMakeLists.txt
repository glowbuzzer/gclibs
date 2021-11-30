cmake_minimum_required(VERSION 3.5)

project(pid C)

add_library(pid ./src/pid.c)

target_link_libraries(pid Logging common-misc)

target_include_directories(pid PUBLIC inc)


