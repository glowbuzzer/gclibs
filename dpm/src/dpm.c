/**
 ******************************************************************************
 * @file           :  dpm.c
 * @brief          :
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */


#include "dpm.h"
//#include "shared.h"

//allows for double buffering but not used
#ifdef CORE_CM7
dpm_in_t *dpm_in = (dpm_in_t*)inA;
dpm_out_t *dpm_out = (dpm_out_t*)outA;
#endif
#if defined(CORE_CM4) || defined(GB_APP_LINUX)
dpm_in_t *dpm_in = (dpm_in_t*)inA;
dpm_out_t *dpm_out = (dpm_out_t*)outA;

#endif


