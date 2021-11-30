/**
 ******************************************************************************
 * @file           :  log.h
 * @brief          :  simple logging framework
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */


#ifndef __LOG_H
#define __LOG_H

#if defined(GB_APP_LINUX)

#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include "printf.h"
#else
#include "printf.h"
#endif

#include <stdlib.h>
#include <stdarg.h>

/*
 * defines for strings used to prefix the log output to let you know which core is logging
 */
#ifdef CORE_CM4
#define CORE_DEBUG_PREFIX "CM4"
#endif

#ifdef CORE_CM7
#define CORE_DEBUG_PREFIX "CM7"
#endif






void gb_fatal_error(unsigned long line, const char *file);

void printf_log_function(const char *fmt, ...);

void LL_HEX(char *addr, int len);


enum {
    LOG_LVL_FATAL, // 0
    LOG_LVL_ERROR, // 1
    LOG_LVL_WARNING, // 2
    LOG_LVL_INFO, // 3
    LOG_LVL_DEBUG, // 4
    LOG_LVL_TRACE // 5
};

extern int log_run_level;

extern const char *log_level_strings[];

#define LOG_SHOULD_I(level) ( (ENABLE_LOGGING && ((level) <= log_run_level))	 )

//static char time_buf[32] __attribute__((unused));

/*
#if defined(GB_APP_LINUX)
static char *current_time() {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    struct tm *tp = localtime(&tv.tv_sec);
    sprintf(time_buf, "%02d:%02d:%02d.%03.0f", tp->tm_hour, tp->tm_min, tp->tm_sec, tv.tv_usec / 1000.0);
    return time_buf;
}

#define LOG(enabled, level, fmt, ...) do {    \
    if ( (enabled >= 1) && LOG_SHOULD_I(level) ) { \
        printf("\033[0;3%dm%s [%s] " fmt " (%s:%d)\033[0m\n", enabled-1, current_time(), log_level_strings[level], ##__VA_ARGS__, __FILE__,__LINE__); \
        fflush(stdout); \
    } \
    if ( level == LOG_LVL_FATAL ) { \
        FATAL; \
    } \
} while(0)
#else
*/

#if defined(GB_APP_LINUX)
#define DEBUG_PREFIX "LL"
#endif
//for poverty stricken embedded developers who can't afford fancy coloured terminals like proper developers...
#define LOG(enabled, level, fmt, ...) do {    \
    if ( (enabled >= 1) && LOG_SHOULD_I(level) ) { \
        printf("[%s] [%s] " fmt " (%s:%d)\n", log_level_strings[level],DEBUG_PREFIX, ##__VA_ARGS__, __FILE__,__LINE__); \
    } \
    if ( level == LOG_LVL_FATAL ) { \
        FATAL; \
    } \
} while(0)



//#endif

#define EVERYN 1000

#define LL_TRACE(enabled, fmt, ...) LOG( enabled, LOG_LVL_TRACE, fmt, ##__VA_ARGS__ )
#define LL_DEBUG(enabled, fmt, ...) LOG( enabled, LOG_LVL_DEBUG, fmt, ##__VA_ARGS__ )
#define LL_INFO(enabled, fmt, ...) LOG( enabled, LOG_LVL_INFO, fmt, ##__VA_ARGS__ )
#define LL_WARN(enabled, fmt, ...) LOG( enabled ? 4: 0, LOG_LVL_WARNING, fmt, ##__VA_ARGS__ )
#define LL_ERROR(enabled, fmt, ...) LOG( enabled ? 2 : 0, LOG_LVL_ERROR, fmt, ##__VA_ARGS__ )
#define LL_FATAL(fmt, ...) LOG( 2, LOG_LVL_FATAL, fmt, ##__VA_ARGS__ )
#define LL_TRACE_EVERYN(enabled, mod_variable, fmt, ...) do {    \
		if ((mod_variable % EVERYN) == 0){ \
			LOG( enabled, LOG_LVL_TRACE, fmt, ##__VA_ARGS__ ); \
		} \
} while(0)



/*
 * not quite a global assert enable/disable - see mbed.h for details
 */
#ifndef ASSERT_EN
#define ASSERT_EN 0
#endif

#define FATAL gb_fatal_error(__LINE__, __FILE__)
#define FATAL_IF(x) if ((x)==1) gb_fatal_error(__LINE__, __FILE__)
#define FATAL_UNLESS(x) if ((x)==0) gb_fatal_error(__LINE__, __FILE__)
#define ASSERT(x) if (ASSERT_EN && (x)==0) gb_fatal_error(__LINE__, __FILE__)

#endif
