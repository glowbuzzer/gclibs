cmake_minimum_required(VERSION 3.5)

project(sleep C)

add_library(sleep STATIC ./src/sleep.c)

target_link_libraries(sleep Logging common-misc)

target_include_directories(sleep PUBLIC inc)





