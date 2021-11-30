
/**
 ******************************************************************************
 * @file           :  sim.c
 * @brief          :
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */

#include <stdint.h>
#include "cia402.h"

/**
 * @brief Simulates the CIA402 state machine for sim mode
 * @param [In] nextControlWord
 * @param [Out] machineState
 * @return simulated status word
 *
 *
 */
uint32_t cia_sim_control_word(const uint32_t nextControlWord, uint32_t *machineState) {

    uint32_t simulatedStatusWord = 0;
    //Fault reset
//    if (nextControlWord & 0b01000000) {
//        simulatedStatusWord &= 0b10110000;
//        simulatedStatusWord |= 0b1111;
//        return;
//    }

//this shouldnt be here really
//    if (cia_ctrlwrd_to_command(nextControlWord) == CIA_FAULT_RESET) {
//        simulatedStatusWord = CIA_SWITCH_ON_DISABLED_STATWRD;
//        return;
//    }

    switch (*machineState) {
        case CIA_FAULT_REACTION_ACTIVE:
            if (cia_ctrlwrd_to_command(nextControlWord) == CIA_FAULT_RESET) {
                simulatedStatusWord = CIA_SWITCH_ON_DISABLED_STATWRD;
                *machineState = CIA_SWITCH_ON_DISABLED;
            }
            break;
        case CIA_FAULT:
            if (cia_ctrlwrd_to_command(nextControlWord) == CIA_FAULT_RESET) {
                simulatedStatusWord = CIA_SWITCH_ON_DISABLED_STATWRD;
                *machineState = CIA_SWITCH_ON_DISABLED;
            } else {
                simulatedStatusWord = CIA_FAULT_STATWRD;
            }

            break;
        case CIA_NOT_READY_TO_SWITCH_ON:
            simulatedStatusWord = CIA_SWITCH_ON_DISABLED_STATWRD;
            *machineState = CIA_SWITCH_ON_DISABLED;
            break;
        case CIA_SWITCH_ON_DISABLED:
            if (cia_ctrlwrd_to_command(nextControlWord) == CIA_SHUTDOWN) {
                simulatedStatusWord = CIA_READY_TO_SWITCH_ON_STATWRD;
                *machineState = CIA_READY_TO_SWITCH_ON;
            } else {
                simulatedStatusWord = CIA_SWITCH_ON_DISABLED_STATWRD;
            }

            break;
        case CIA_READY_TO_SWITCH_ON:
            if (cia_ctrlwrd_to_command(nextControlWord) == CIA_SWITCH_ON) {
                simulatedStatusWord = CIA_SWITCHED_ON_STATWRD;
                *machineState = CIA_SWITCHED_ON;
            } else if (cia_ctrlwrd_to_command(nextControlWord) == CIA_DISABLE_VOLTAGE) {
                simulatedStatusWord = CIA_SWITCH_ON_DISABLED_STATWRD;
                *machineState = CIA_SWITCH_ON_DISABLED;
            } else {
                simulatedStatusWord = CIA_READY_TO_SWITCH_ON_STATWRD;
            }
            break;
        case CIA_SWITCHED_ON:
            if (cia_ctrlwrd_to_command(nextControlWord) == CIA_ENABLE_OPERATION) {
                simulatedStatusWord = CIA_OPERATION_ENABLED_STATWRD;
                *machineState = CIA_OPERATION_ENABLED;
            } else if (cia_ctrlwrd_to_command(nextControlWord) == CIA_DISABLE_VOLTAGE) {
                simulatedStatusWord = CIA_SWITCH_ON_DISABLED_STATWRD;
                *machineState = CIA_SWITCH_ON_DISABLED;
            } else if (cia_ctrlwrd_to_command(nextControlWord) == CIA_SHUTDOWN) {
                simulatedStatusWord = CIA_READY_TO_SWITCH_ON_STATWRD;
                *machineState = CIA_READY_TO_SWITCH_ON;
            } else {
                simulatedStatusWord = CIA_SWITCHED_ON_STATWRD;
            }
            break;
        case CIA_OPERATION_ENABLED:
            if (cia_ctrlwrd_to_command(nextControlWord) == CIA_DISABLE_VOLTAGE) {
                simulatedStatusWord = CIA_SWITCH_ON_DISABLED_STATWRD;
                *machineState = CIA_SWITCH_ON_DISABLED;
            } else if (cia_ctrlwrd_to_command(nextControlWord) == CIA_DISABLE_OPERATION) {
                simulatedStatusWord = CIA_SWITCHED_ON_STATWRD;
                *machineState = CIA_SWITCHED_ON;
            } else if (cia_ctrlwrd_to_command(nextControlWord) == CIA_SHUTDOWN) {
                simulatedStatusWord = CIA_READY_TO_SWITCH_ON_STATWRD;
                *machineState = CIA_READY_TO_SWITCH_ON;
            } else if (cia_ctrlwrd_to_command(nextControlWord) == CIA_QUICK_STOP) {
                simulatedStatusWord = CIA_QUICK_STOP_ACTIVE_STATWRD;
                *machineState = CIA_QUICK_STOP_ACTIVE_STATWRD;
            } else {
                simulatedStatusWord = CIA_OPERATION_ENABLED_STATWRD;
            }
            break;
        case CIA_QUICK_STOP_ACTIVE:
            if (cia_ctrlwrd_to_command(nextControlWord) == CIA_DISABLE_VOLTAGE) {
                simulatedStatusWord = CIA_SWITCH_ON_DISABLED_STATWRD;
                *machineState = CIA_SWITCH_ON_DISABLED;
            } else if (cia_ctrlwrd_to_command(nextControlWord) == CIA_ENABLE_OPERATION) {
                simulatedStatusWord = CIA_OPERATION_ENABLED_STATWRD;
                *machineState = CIA_OPERATION_ENABLED;
            } else {
                simulatedStatusWord = CIA_QUICK_STOP_ACTIVE_STATWRD;
            }
            break;
        default:
            break;
    }
    // not sure this is appropriate - simulatedStatusWord will be init value of 0
    // (modified as part of removing compiler warnings)
    return simulatedStatusWord;
}