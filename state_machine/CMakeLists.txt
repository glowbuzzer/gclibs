cmake_minimum_required(VERSION 3.5)

project(state_machine C)

add_library(state_machine ./src/stateMachine.c)

target_include_directories(state_machine PUBLIC inc)