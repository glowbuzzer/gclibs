/**
 ******************************************************************************
 * @file           :  log.c
 * @brief          :
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */


#include "log.h"
#include <string.h>
#include "stdint.h"

#if !defined(GB_APP_LINUX)  && !defined(CORE_CM4)
#include "FreeRTOS.h"
#include "task.h"
#endif


int log_run_level = LOG_LEVEL;

//int log_run_level = 0;

#if !defined(GB_APP_LINUX)
/**
 * @brief function to output formatted strings with [DEBUG ] prefix
 * Used by third party libs that can't integrate with the LOG macro
 * Works with single newline terminated strings and also
 * Uses thread local storage array element [0] to store whether the string is expecting the newline
 * @note hard codes the log level to DEBUG as the libs don't have our concept of log level
 * @param fmt - formatted string to log
 * @return void
 */

void printf_log_function(const char *fmt, ...) {
	va_list ap;
	va_start(ap, fmt);
	uint32_t newline_on_end = 0;
	uint32_t no_newline_string_count = ( uint32_t ) pvTaskGetThreadLocalStoragePointer(NULL, 0);

	uint32_t string_length = strlen(fmt);
	if (fmt[string_length - 1] == '\n') {
		newline_on_end = 1;
	}

	if ((newline_on_end && (no_newline_string_count == 0)) || (no_newline_string_count == 0)) {
		printf("[DEBUG ] (%s) ", CORE_DEBUG_PREFIX);
	}
	vprintf(fmt, ap);
	va_end(ap);

	if (!newline_on_end) {
		no_newline_string_count++;
		vTaskSetThreadLocalStoragePointer(NULL, 0, (void*) no_newline_string_count);
	}
	if (newline_on_end) {
		vTaskSetThreadLocalStoragePointer(NULL, 0, (void*) 0);
	}
}
#endif
const char *log_level_strings[] = { "FATAL ", "ERROR ", "WARN  ", "INFO  ", "DEBUG ", "TRACE " };

void LL_HEX(char *addr, int len) {
	if (!LOG_SHOULD_I(LOG_LVL_DEBUG)) {
		return;
	}

	int i;
	unsigned char buff[17];
	unsigned char *pc = (unsigned char *) addr;

	// Process every byte in the data.
	for (i = 0; i < len; i++) {
		// Multiple of 16 means new line (with line offset).

		if ((i % 16) == 0) {
			// Just don't print ASCII for the zeroth line.
			if (i != 0)
				printf("  %s\n", buff);

			// Output the offset.
			printf("  %04x ", i);
		}

		// Now the hex code for the specific character.
		printf(" %02x", pc[i]);

		// And store a printable ASCII character for later.
		if ((pc[i] < 0x20) || (pc[i] > 0x7e))
			buff[i % 16] = '.';
		else
			buff[i % 16] = pc[i];
		buff[(i % 16) + 1] = '\0';
	}

	// Pad out last line if not exactly 16 characters.
	while ((i % 16) != 0) {
		printf("   ");
		i++;
	}

	// And print the final ASCII bit.
	printf("  %s\n", buff);
}

// store error location for debugging purposes
char error_file[256];
unsigned long error_line;

//extern void vAssertCalled(unsigned long ulLine, const char *pcFileName);

//Make this the function that duktape, freertos etc. call
void gb_fatal_error(unsigned long line, const char *file) {

#if !defined(GB_APP_LINUX)
	printf("[FATAL ] Error handler called from [%s:%ld]", file, line);
	exit(-1);
#else
	volatile uint32_t setToNonZeroInDebuggerToContinue = 0;
//    __ASM volatile("BKPT #01")
	while (setToNonZeroInDebuggerToContinue == 0) {
	}
#endif
	strcpy(error_file, file);
	error_line = line;
}

