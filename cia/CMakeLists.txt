cmake_minimum_required(VERSION 3.5)

project(cia C)

file(GLOB SOURCES "src/*.c")


add_library(cia STATIC ${SOURCES})
#target_link_libraries(cia shared)
target_include_directories(cia PUBLIC inc)


