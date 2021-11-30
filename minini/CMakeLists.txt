cmake_minimum_required(VERSION 3.5)

project(minini C)

file(GLOB SOURCES "src/*.c")


add_library(minini STATIC ${SOURCES})

target_include_directories(minini PUBLIC inc)