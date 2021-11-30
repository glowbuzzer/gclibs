/**
 ******************************************************************************
 * @file           :  gberror.h
 * @brief          :
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */


#ifndef GB_MONOREPO_GBERROR_H
#define GB_MONOREPO_GBERROR_H

#define NUMECODES (int)(sizeof(gberrordesc)/sizeof(gberrordesc[0]))

/** enum for all gberror return codes */
typedef enum {
    E_SUCCESS =0,
    E_INVALID_INPUT = -1,
    E_FILE_NOT_FOUND = -2,
    E_SYSCALL = -3,
    E_NUMARGS = -4,
    E_NOINPUT = -5,
    E_TOO_LONG = -6,
    E_MALLOC = -7,
    E_SDO_READ_FAILURE = -8,
    E_SDO_WRITE_FAILURE = -9,
    E_SUB_DRIVE_OUT_OF_RANGE = -10,
    E_ETHERCAT_ERROR_DETECTED = -11,
    E_NVRAM_WRITE_FAILURE = -12,
    E_GENERAL_FAILURE = -13,
    E_NO_FUNCTION_FOUND = -14,
    E_SHARED_MEM_INIT_FAILURE = -15,
    E_TIMEOOUT = -16,
    E_ARRAY_OVERFLOW = -17,
    E_INVALID_MAP = -18,
    E_LOOKUP_FAILED = -19,
    E_NOT_IMPLEMENTED = -20,
    E_FAILED_TO_OPEN_FILE = -21,
    E_THREAD_CREATE_FAIL = -22,
    E_USER_CANCELLED_OPERATION = -23,
    E_OPERATION_FAILED = -24,
    E_UNKNOWN_ERROR = -26,
    E_ENTRY_NOT_FOUND = -27,
    E_INVALID_CONFIG = -28,
    E_REGISTER_READ_FAILURE = -29,
    E_INVALID_PARAMETER = -30,
    E_INIT_FAILURE = -31,
    E_NOT_STOPPED = -32,
    E_NOT_FOUND = -33,
    E_GPIO_FAILURE = -34
} gberror_t;

char * gb_strerror(int return_value);
int gb_perror(int return_value);

#endif //GB_MONOREPO_GBERROR_H
