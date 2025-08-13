cmake_minimum_required(VERSION 3.5)

project(dpm-status C)

add_library(dpm-status INTERFACE)

target_link_libraries(dpm-status INTERFACE cia)
target_include_directories(dpm-status INTERFACE inc)