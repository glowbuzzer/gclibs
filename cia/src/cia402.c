/**
 ******************************************************************************
 * @file           :  cia402.c
 * @brief          :
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy, Copyright (c) 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */



/*
 * ORIENTAL MOTOR:
 *
 * CTRLWRD
 * Says Bit 7 (fault reset) can be in any state whereas we require it to be 0 in all other states
 *
 */


#include "cia402.h"

const char *cia_state_names[CIA_NUM_STATE_NAMES] = {"Not ready to switch on", "Switch on disabled",
                                                    "Ready to switch on", "Switched on",
                                                    "Operation enabled", "Quick stop active", "Fault reaction active",
                                                    "Fault"};


const char *cia_command_names[CIA_NUM_COMMAND_NAMES] = {"Shutdown", "Switch on", "Disable voltage", "Quick stop",
                                                        "Disable operation",
                                                        "Enable operation", "Fault reset"};


/**
 * @brief converts an enum command  to a ctrlwrd (unit16)
 * @param ctrlwrd
 * @return
 */

uint16_t cia_command_to_ctrlwrd(const cia_commands_t command) {
    switch (command) {
        case CIA_SHUTDOWN:
            return CIA_SHUTDOWN_CTRLWRD;
        case CIA_SWITCH_ON:
            return CIA_SWITCH_ON_CTRLWRD;
        case CIA_DISABLE_VOLTAGE:
            return CIA_DISABLE_VOLTAGE_CTRLWRD;
        case CIA_QUICK_STOP:
            return CIA_QUICK_STOP_CTRLWRD;
        case CIA_DISABLE_OPERATION:
            return CIA_DISABLE_OPERATION_CTRLWRD;
        case CIA_ENABLE_OPERATION:
            return CIA_ENABLE_OPERATION_CTRLWRD;
        case CIA_FAULT_RESET:
            return CIA_FAULT_RESET_CTRLWRD;
        default:
            return 99;
    }
}


/**
 * @brief converts a ctrlwrd (unit16) to an enum command
 * @param controlWord
 * @return
 */
cia_commands_t cia_ctrlwrd_to_command(const uint16_t controlWord) {

    if ((controlWord & CIA_SHUTDOWN_BIT_MASK) == CIA_SHUTDOWN_CTRLWRD) {
        return CIA_SHUTDOWN;
    }
    if ((controlWord & CIA_SWITCH_ON_BIT_MASK) == CIA_SWITCH_ON_CTRLWRD) {
        return CIA_SWITCH_ON;
    }
//    if ((controlWord & CIA_SWITCH_ON_AND_ENABLE_OPERATION_BIT_MASK) == CIA_SWITCH_ON_AND_ENABLE_OPERATION_CTRLWRD) {
//        return CIA_SWITCH_ON_AND_ENABLE_OPERATION;
//    }
    if ((controlWord & CIA_DISABLE_VOLTAGE_BIT_MASK) == CIA_DISABLE_VOLTAGE_CTRLWRD) {
        return CIA_DISABLE_VOLTAGE;
    }
    if ((controlWord & CIA_QUICK_STOP_BIT_MASK) == CIA_QUICK_STOP_CTRLWRD) {
        return CIA_QUICK_STOP;
    }
    if ((controlWord & CIA_DISABLE_OPERATION_BIT_MASK) == CIA_DISABLE_OPERATION_CTRLWRD) {
        return CIA_DISABLE_OPERATION;
    }
    if ((controlWord & CIA_ENABLE_OPERATION_BIT_MASK) == CIA_ENABLE_OPERATION_CTRLWRD) {
        return CIA_ENABLE_OPERATION;
    }
    if ((controlWord & CIA_FAULT_RESET_BIT_MASK) == CIA_FAULT_RESET_CTRLWRD) {
        return CIA_FAULT_RESET;
    }
    return 99;
}

/**
 * @brief converts a state (enum) to a statwrd (uint16)
 * @param state
 * @return
 */

uint16_t cia_state_to_statwrd(const cia_state_t state) {

    switch (state) {
        case CIA_NOT_READY_TO_SWITCH_ON:
            return CIA_NOT_READY_TO_SWITCH_ON_STATWRD;
        case CIA_SWITCH_ON_DISABLED:
            return CIA_SWITCH_ON_DISABLED_STATWRD;
        case CIA_READY_TO_SWITCH_ON:
            return CIA_READY_TO_SWITCH_ON_STATWRD;
        case CIA_SWITCHED_ON:
            return CIA_SWITCHED_ON_STATWRD;
        case CIA_OPERATION_ENABLED:
            return CIA_OPERATION_ENABLED_STATWRD;
        case CIA_QUICK_STOP_ACTIVE:
            return CIA_QUICK_STOP_ACTIVE_STATWRD;
        case CIA_FAULT_REACTION_ACTIVE:
            return CIA_FAULT_REACTION_ACTIVE_STATWRD;
        case CIA_FAULT:
            return CIA_FAULT_STATWRD;
        default:
            return 99;
    }

}

/**
 * @brief converts a statwrd (uint16) to a state (enum)
 * @param statusWord
 * @return
 */
cia_state_t cia_statwrd_to_state(const uint16_t statusWord) {
//    volatile uint16_t sw = statusWord;
    if ((statusWord & CIA_NOT_READY_TO_SWITCH_ON_BIT_MASK) == CIA_NOT_READY_TO_SWITCH_ON_STATWRD) {
        return CIA_NOT_READY_TO_SWITCH_ON;
    }
    if ((statusWord & CIA_SWITCH_ON_DISABLED_BIT_MASK) == CIA_SWITCH_ON_DISABLED_STATWRD) {
        return CIA_SWITCH_ON_DISABLED;
    }
    if ((statusWord & CIA_READY_TO_SWITCH_ON_BIT_MASK) == CIA_READY_TO_SWITCH_ON_STATWRD) {
        return CIA_READY_TO_SWITCH_ON;
    }
    if ((statusWord & CIA_SWITCHED_ON_BIT_MASK) == CIA_SWITCHED_ON_STATWRD) {
        return CIA_SWITCHED_ON;
    }
    if ((statusWord & CIA_OPERATION_ENABLED_BIT_MASK) == CIA_OPERATION_ENABLED_STATWRD) {
        return CIA_OPERATION_ENABLED;
    }
    if ((statusWord & CIA_QUICK_STOP_ACTIVE_BIT_MASK) == CIA_QUICK_STOP_ACTIVE_STATWRD) {
        return CIA_QUICK_STOP_ACTIVE;
    }
    if ((statusWord & CIA_FAULT_REACTION_ACTIVE_BIT_MASK) == CIA_FAULT_REACTION_ACTIVE_STATWRD) {
        return CIA_FAULT_REACTION_ACTIVE;
    }
    if ((statusWord & CIA_FAULT_BIT_MASK) == CIA_FAULT_STATWRD) {
        return CIA_FAULT;
    }
    return 99;
}


const char *cia_moo_strings[CIA_HIGHEST_MOO_DEFINED] = {
        "CIA_MOO_OP_DISABLED", //0
        "CIA_MOO_PROFILE_POS", //1
        "CIA_MOO_PROFILE_VEL", //2
        "", // Placeholder for 3 (not defined)
        "", // Placeholder for 4 (not defined)
        "", // Placeholder for 5 (not defined)
        "CIA_MOO_HOMING", //6
        "", // Placeholder for 7 (not defined)
        "CIA_MOO_CSP", //8
        "CIA_MOO_CSV", //9
        "CIA_MOO_CST" //10
};