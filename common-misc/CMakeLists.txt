cmake_minimum_required(VERSION 3.5)

project(common-misc C)
file(GLOB SOURCES "src/*.c")

add_library(common-misc STATIC ${SOURCES})
target_include_directories(common-misc PUBLIC inc)