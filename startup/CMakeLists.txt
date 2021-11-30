cmake_minimum_required(VERSION 3.5)

project(startup C)

file(GLOB SOURCES "src/*.c")

add_library(startup STATIC ${SOURCES})
target_link_libraries(startup common-misc minini Logging)
target_include_directories(startup PUBLIC inc)
