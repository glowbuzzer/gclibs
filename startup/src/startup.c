
/**
 ******************************************************************************
 * @file           :  startup.c
 * @brief          :  functions for boot up
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */

#include "startup.h"
#include "minIni.h"
#include "string.h"
#include "std_defs_and_macros.h"
#include "user_message.h"
#include "log.h"

////platform definition is read from ini file
//os_platform_t os_platform = PLATFORM_PI;
//const char gb_inifile_name[] = "gbem.ini";

/**
 * @brief reads ini file and pulls out platform
 * @return returns enum of type of platform from ini file {PLATFORM_LINUX, PLATFORM_PI}
 * @param gb_inifile_name filename of ini file
 * @param
 */
//os_platform_t read_platform_from_ini(char * gb_inifile_name,os_platform_t default_platform, gb_program_t gb_program, int um_en) {
//    char str[100];
////todo-crit ??
//    if (gb_program == GB_PROGRAM_GBC){
//
//    } else if (gb_program == GB_PROGRAM_GBEM){
//
//    } else if (gb_program == GB_PROGRAM_GBSM){
//
//    }
//    else {
//        LL_FATAL("Invalid gb_program name");
//    }
//
//
//    //section name in ini file is "Platform" (upper case p)
//    //key in ini file is platform (lower case p)
//
//    int n = ini_gets("Platform", "platform", "none", str, sizearray(str), gb_inifile_name);
////n = number of chars read
//
////printf("str:%s\n",str);
//    if (strcmp(str, "generic_linux") == 0) {
//        UM_INFO(um_en, "STARTUP: IMPORTANT - the platform has been set to [generic_linux]");
//        return PLATFORM_LINUX;
//    } else if (strcmp(str, "rpi") == 0) {
//        UM_INFO(um_en, "STARTUP: IMPORTANT - the platform has been set to [rpi]");
//        return PLATFORM_PI;
//    } else {
//        if (default_platform == PLATFORM_PI){
//            UM_ERROR(um_en, "STARTUP: We could not read value from: [%s] file, platform will default to [PLATFORM_PI]",
//                 gb_inifile_name);
//        return PLATFORM_PI;
//        }
//        else if (default_platform == PLATFORM_LINUX){
//            UM_ERROR(um_en, "STARTUP: We could not read value from: [%s] file, platform will default to [PLATFORM_GENERIC_LINUX]",
//                     gb_inifile_name);
//            return PLATFORM_LINUX;
//
//        }
//        else {
//            LL_FATAL("STARTUP: Invalid default platform provided to read platform from ini");
//        }
//    }
//}
//
