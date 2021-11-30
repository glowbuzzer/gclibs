/**
 ******************************************************************************
 * @file           :  user_message.h
 * @brief          :  user message logging functions
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */


#ifndef GB_MONOREPO_USER_MESSAGE_H
#define GB_MONOREPO_USER_MESSAGE_H

#include "printf.h"
#include "gberror.h"
#include <stdbool.h>

void gb_fatal_release_error(unsigned long line, const char *file, const char* fmt, ...);

extern const char *um_level_strings[];

extern bool um_disable_logging;


enum {
    UM_LVL_FATAL, // 0
    UM_LVL_ERROR, // 1
    UM_LVL_WARNING, // 2
    UM_LVL_INFO, // 3
};


void log_generic(int level, const char* fmt, ...);
gberror_t logger_set_log_file(const char* filename, int um_en);
gberror_t logger_set_syslog(char * log_ident);
gberror_t logger_set_stdout(void);

//#define UM(enabled, level, fmt, ...) do {    \
//    if ( (enabled >= 1)) { \
//        printf("[%s] " fmt "\n", um_level_strings[level], ##__VA_ARGS__); \
//    } \
//    if ( level == UM_LVL_FATAL ) { \
//        UFATAL; \
//    } \
//} while(0)

#define UM(enabled, level, fmt, ...) do {    \
    if ( (enabled >= 1 && level != UM_LVL_FATAL)) {                   \
        log_generic(level, fmt, ##__VA_ARGS__); \
    } \
    if ( level == UM_LVL_FATAL ) { \
        UFATAL(fmt, ##__VA_ARGS__); \
    } \
} while(0)



//#define LOG(enabled, level, fmt, ...) do {    \
//if ( (enabled >= 1) && LOG_SHOULD_I(level) ) { \
//printf("[%s] (%s) " fmt " (%s:%d)\n", log_level_strings[level],CORE_DEBUG_PREFIX, ##__VA_ARGS__, __FILE__,__LINE__); \
//} \
//if ( level == LOG_LVL_FATAL ) { \
//FATAL; \
//} \
//} while(0)


#define UM_INFO(enabled, fmt, ...) UM( enabled, UM_LVL_INFO, fmt, ##__VA_ARGS__ )
#define UM_WARN(enabled, fmt, ...) UM( enabled, UM_LVL_WARNING, fmt, ##__VA_ARGS__ )
#define UM_ERROR(enabled, fmt, ...) UM( enabled, UM_LVL_ERROR, fmt, ##__VA_ARGS__ )
#define UM_FATAL(fmt, ...) UM( 1, UM_LVL_FATAL, fmt, ##__VA_ARGS__ )

#define UFATAL(fmt, ...) gb_fatal_release_error(__LINE__, __FILE__, fmt, ##__VA_ARGS__ )

#endif //GB_MONOREPO_USER_MESSAGE_H
