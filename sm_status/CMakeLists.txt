cmake_minimum_required(VERSION 3.5)

project(ecm_status C)

file(GLOB SOURCES "src/*.c")


add_library(sm_status INTERFACE)

target_link_libraries(sm_status INTERFACE cia)
target_include_directories(sm_status INTERFACE inc)