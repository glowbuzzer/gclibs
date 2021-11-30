cmake_minimum_required(VERSION 3.5)

project(gberror C)

file(GLOB SOURCES "src/*.c")

add_library(gberror STATIC ${SOURCES})

target_include_directories(gberror PUBLIC inc)