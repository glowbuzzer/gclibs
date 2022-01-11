
/**
 ******************************************************************************
 * @file           :  gberror.c
 * @brief          :
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */

#include "gberror.h"
#include <stdio.h>

/** defines the gberror messages*/
struct _gberrordesc {
    int  code;
    char *message;
} gberrordesc[] = {
        { E_SUCCESS, "No error" },
        { E_INVALID_INPUT, "Invalid input" },
        { E_FILE_NOT_FOUND, "File not found" },
        { E_SYSCALL, "System call failed" },
        { E_NUMARGS, "Wrong number of arguments" },
        { E_NOINPUT, "No input or invalid input -- type help for more"},
        { E_TOO_LONG, "Input longer than the maximum allowable characters (256)" },
        { E_MALLOC, "Failed to allocate memory" },
        {E_SDO_READ_FAILURE, "Failed to read SDO object"},
        {E_SDO_WRITE_FAILURE, "Failed to write SDO object"},
        {E_SUB_DRIVE_OUT_OF_RANGE, "A function has tried to find a sub-drive that is not handled"},
        {E_ETHERCAT_ERROR_DETECTED, "EtherCAT error detected during processing"},
        {E_NVRAM_WRITE_FAILURE, "Failure to write data to NVRAM"},
        {E_GENERAL_FAILURE, "General failure"},
        {E_NO_FUNCTION_FOUND, "No function to perform the operation was found"},
        {E_SHARED_MEM_INIT_FAILURE, "Shared memory initialisation or connection establishment faiure"},
        {E_TIMEOOUT, "A timeout has occured connecting to something to executing something"},
        {E_ARRAY_OVERFLOW, "An array overflow (would) have occured"},
        {E_INVALID_MAP, "The map provided for processing is invalid"},
        {E_LOOKUP_FAILED, "A lookup of a value failed to find a match"},
        {E_NOT_IMPLEMENTED, "The function has not been implemented"},
        {E_FAILED_TO_OPEN_FILE, "Failed to open requested file"},
        {E_THREAD_CREATE_FAIL, "Failed to create a thread"},
        {E_USER_CANCELLED_OPERATION, "User chose to cancel the operation"},
        {E_OPERATION_FAILED, "Operation failed"},
        {E_UNKNOWN_ERROR, "Uknown error"},
        {E_ENTRY_NOT_FOUND, "A matching entry was not found"},
        {E_INVALID_CONFIG, "The configuration provided is invalid"},
        {E_REGISTER_READ_FAILURE, "EtherCAT register read failure"},
        {E_INVALID_PARAMETER, "Invalid parameter passed to function"},
        {E_INIT_FAILURE, "Initialisation failure"},
        {E_NOT_STOPPED, "An action has been requested whilst motion is occurring"},
        {E_NOT_FOUND, "The searched for entity was not found"},
        {E_GPIO_FAILURE, "GPIO failure"},
        {E_CHIP_MATCH_FAILURE,  "Chip does not match the requested one"},
        {E_INVALID_COOKIE, "Cookie is not valid"},
        {E_DEVICE_IS_NOT_SLAVE, "Device is not a slave"},
        {E_INVALID_PROTOCOL, "Protocol is not valid"},
        {E_NO_SPACE, "No space for data"},
        {E_NO_MESSAGES, "There are no messages available"},
        {E_REG_FAILED, "Registration for the service failed"},
};

int gb_perror(int return_value){

    if (return_value < 0 && return_value >= (-1*NUMECODES)) {
        printf("%s", gberrordesc[-1*return_value].message);

    }else if (return_value == 0) {
        return 0;
    }
    else {
        printf("Unknown error %d: \n", return_value);
    }
    return 0;
}


char * gb_strerror(int return_value){

    if (return_value < 0 && return_value >= (-1*NUMECODES)) {
        return gberrordesc[-1*return_value].message;

    }else if (return_value == 0) {
        return "";
    }
    else {
        printf("Unknown error %d: \n", return_value);
    }
    return "";
}
