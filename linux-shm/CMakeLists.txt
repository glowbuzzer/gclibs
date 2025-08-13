cmake_minimum_required(VERSION 3.5)

project(linux-shm C)

file(GLOB SOURCES "src/*.c")

add_library(linux-shm STATIC ${SOURCES})
target_link_libraries(linux-shm gberror Logging pid dpm dpm-status)
target_include_directories(linux-shm PUBLIC inc)
