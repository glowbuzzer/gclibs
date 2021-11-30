cmake_minimum_required(VERSION 3.5)

project(Logging C)

add_library(Logging STATIC src/log.c src/user_message.c miniprintf/src/printf.c)

target_link_libraries(Logging gberror)

target_include_directories(Logging PUBLIC include miniprintf/inc gberror)




