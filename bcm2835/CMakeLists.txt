cmake_minimum_required(VERSION 3.5)

project(bcm2835 C)

file(GLOB SOURCES "src/*.c")

add_library(bcm2835 STATIC ${SOURCES})

target_include_directories(bcm2835 PUBLIC inc)