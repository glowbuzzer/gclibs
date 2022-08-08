
/**
 ******************************************************************************
 * @file           :  user_message.c
 * @brief          :  user message logging functions
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */

#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <syslog.h>
#include <stdio.h>
#include <stdbool.h>
#include <errno.h>
#include "user_message.h"

/* global variable used to disable user messages */
bool um_disable_logging = false;

/* enum whose states represent the different output channels for user messages */
typedef enum {LOG_TO_STDOUT, LOG_TO_SYSLOG, LOG_TO_FILE} logger_output_type_t;


/** struct to hole user message state */
typedef struct {
    logger_output_type_t op;
    FILE* out_file;
    void (*logger_func) (const int level, const char*);
}logger_t;

logger_t log_global_set;
void print_to_stdout(int level, const char* message);
void print_to_syslog(int level, const char* message);
void print_to_file(int level, const char* message);
void cleanup_internal(void);


/**
 * @brief Reset internal state and set stdout as defaul
 */
gberror_t logger_set_stdout(void)
{
    cleanup_internal();
    log_global_set.op = LOG_TO_STDOUT;
    log_global_set.logger_func = print_to_stdout;
    return E_SUCCESS;

}

/**
 * @brief Close remaining file descriptor and reset global params
 */
void cleanup_internal(void)
{
    if (log_global_set.out_file){
        fclose(log_global_set.out_file);
    }

    log_global_set.out_file = NULL;
    log_global_set.op = LOG_TO_STDOUT;
    log_global_set.logger_func = NULL;
}

/**
 * @brief set logger output to syslog
 * @param log_ident string to use in syslog as prefix to identify output
 * @return
 */
gberror_t logger_set_syslog(char * log_ident){
    cleanup_internal();
    openlog(log_ident, LOG_NDELAY, LOG_USER);
    log_global_set.op = LOG_TO_SYSLOG;
    log_global_set.logger_func = print_to_syslog;
    return E_SUCCESS;
}

/**
 * @brief set logger output to a file
 * @param filename
 * @return
 */
gberror_t logger_set_log_file(const char* filename, int um_en)
{
    cleanup_internal();
    if ((filename != NULL) && (filename[0] == '\0')) {
        UM_ERROR(um_en, "Filename not valid");
        return E_FAILED_TO_OPEN_FILE;
    }
    log_global_set.out_file = fopen(filename, "a");

    if (log_global_set.out_file == NULL) {
        UM_ERROR(um_en, "Failed to open file %s error %s", filename, strerror(errno));
        return E_FAILED_TO_OPEN_FILE;
    }
    log_global_set.op = LOG_TO_FILE;
    log_global_set.logger_func = print_to_file;

    return E_SUCCESS;
}

/* syslog.h priorities
* #define	LOG_EMERG	0	 system is unusable
* #define	LOG_ALERT	1	 action must be taken immediately
* #define	LOG_CRIT	2	 critical conditions
* #define	LOG_ERR		3	 error conditions
* #define	LOG_WARNING	4	 warning conditions
* #define	LOG_NOTICE	5	 normal but significant condition
* #define	LOG_INFO	6	 informational
* #define	LOG_DEBUG	7	 debug-level messages
*/

void print_to_syslog(const int level, const char* message){

    switch (level) {
        case UM_LVL_INFO:
            syslog(LOG_INFO, "%s", message);
            break;
        case UM_LVL_WARNING:
            syslog(LOG_ALERT, "%s", message);
            break;
        case UM_LVL_ERROR:
            syslog(LOG_ERR, "%s", message);
            break;
        case UM_LVL_FATAL:
            syslog(LOG_EMERG, "%s", message);
            break;
        default:
            break;
    }
}

/**
 * @brief outputs message to stdout
 * @param level
 * @param message
 */
void print_to_stdout(const int level, const char* message){
    printf("[%s] %s\n", um_level_strings[level], message);
}

/**
 * @brief outputs message to file
 * @param level
 * @param message
 */
void print_to_file(const int level, const char* message){
    int res = fprintf(log_global_set.out_file,
    "[%s] %s\n"
            , um_level_strings[level],
            message);
    if (res == -1) {
        print_to_syslog(UM_LVL_ERROR, "Unable to write to log file!");
        return;
    }

    fflush(log_global_set.out_file);
}

/**
 * @brief called log functions then calls right output function based on config.
 * @param level
 * @param fmt
 * @param ...
 */
void log_generic(const int level, const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    char buffer[1024];
    if (!um_disable_logging) {
        vsnprintf(buffer, 1024, fmt, args);
        if (log_global_set.logger_func != NULL) {
            log_global_set.logger_func(level, buffer);
        }
    }
}



const char *um_level_strings[] = { "FATAL    ", "ERROR    ", "WARN     ", "INFO     " };

void gb_fatal_release_error(unsigned long line, const char *file, const char* fmt, ...) {

#if GB_APP_LINUX == 1
    va_list args;
    va_start(args, fmt);
    printf("[FATAL    ] ");
    vprintf(fmt, args);
    printf("\n");
#if DEBUG_BUILD == 1
    printf("[FATAL    ] Error handler called from %s:%ld", file, line);
#endif
    exit(-1);
#else
    volatile uint32_t setToNonZeroInDebuggerToContinue = 0;
//    __ASM volatile("BKPT #01")
    while (setToNonZeroInDebuggerToContinue == 0) {
    }
#endif
//    strcpy(error_file, file);
//    error_line = line;
}