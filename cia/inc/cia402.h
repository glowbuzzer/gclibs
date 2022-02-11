/**
 ******************************************************************************
 * @file           :  cia402.h
 * @brief          :
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */

#ifndef GCLIB_CIA402_H_
#define GCLIB_CIA402_H_

#include <stdint.h>
#include <stdbool.h>

#define CTRL_MACHINE_CTRL_WRD_REQUEST_FAULT_BIT_NUM     16
#define CTRL_MOVE_NOT_OP_ENABLED_FAULT_REQ_BIT_NUM     17

#define CIA_NUM_STATE_NAMES     8
#define CIA_NUM_COMMAND_NAMES   7

typedef enum {
    CIA_NOT_READY_TO_SWITCH_ON,
    CIA_SWITCH_ON_DISABLED,
    CIA_READY_TO_SWITCH_ON,
    CIA_SWITCHED_ON,
    CIA_OPERATION_ENABLED,
    CIA_QUICK_STOP_ACTIVE,
    CIA_FAULT_REACTION_ACTIVE,
    CIA_FAULT
} cia_state_t;

extern const char* cia_state_names[CIA_NUM_STATE_NAMES];


typedef enum {
    CIA_SHUTDOWN,
    CIA_SWITCH_ON,
//    CIA_SWITCH_ON_AND_ENABLE_OPERATION,
    CIA_DISABLE_VOLTAGE,
    CIA_QUICK_STOP,
    CIA_DISABLE_OPERATION,
    CIA_ENABLE_OPERATION,
    CIA_FAULT_RESET
} cia_commands_t;

extern const char* cia_command_names[CIA_NUM_COMMAND_NAMES];

//so we are use bit 16 here - bit confusing
#define GB_FAULT_PLC_CTRLWRD                      0b10000000000000000

/* Control words to command a transition to a state - e.g. send to a drive to change its state */
/* The switch on ctrl word and the disable operation control words are identical */

#define CIA_SHUTDOWN_CTRLWRD                            0b00000110
#define CIA_SWITCH_ON_CTRLWRD                           0b00000111
//#define CIA_SWITCH_ON_AND_ENABLE_OPERATION_CTRLWRD      0b00001111
#define CIA_DISABLE_VOLTAGE_CTRLWRD                     0b00000000
#define CIA_QUICK_STOP_CTRLWRD                          0b00000010
#define CIA_DISABLE_OPERATION_CTRLWRD                   0b00000111
#define CIA_ENABLE_OPERATION_CTRLWRD                    0b00001111
#define CIA_FAULT_RESET_CTRLWRD                         0b10000000 //0->1 on bit7


/* Statuswords that reflect a state - e.g. when in a state send this status word to report it */

#define CIA_NOT_READY_TO_SWITCH_ON_STATWRD              0b00000000
#define CIA_SWITCH_ON_DISABLED_STATWRD                  0b01000000
#define CIA_READY_TO_SWITCH_ON_STATWRD                  0b00100001
#define CIA_SWITCHED_ON_STATWRD                         0b00100011
#define CIA_OPERATION_ENABLED_STATWRD                   0b00100111
#define CIA_QUICK_STOP_ACTIVE_STATWRD                   0b00000111
#define CIA_FAULT_REACTION_ACTIVE_STATWRD               0b00001111
#define CIA_FAULT_STATWRD                               0b00001000



/* controlword bit masks - 1 indicates that it is a bit we care about */


#define CIA_SHUTDOWN_BIT_MASK                           0b10000111
#define CIA_SWITCH_ON_BIT_MASK                          0b10001111
//#define CIA_SWITCH_ON_AND_ENABLE_OPERATION_BIT_MASK     0b10001111
#define CIA_DISABLE_VOLTAGE_BIT_MASK                    0b10000010
#define CIA_QUICK_STOP_BIT_MASK                         0b10000110
#define CIA_DISABLE_OPERATION_BIT_MASK                  0b10011111
#define CIA_ENABLE_OPERATION_BIT_MASK                   0b10001111
#define CIA_FAULT_RESET_BIT_MASK                        0b10000000



/* statusword bit masks  - 1 indicates that it is a bit we care about*/
#define CIA_NOT_READY_TO_SWITCH_ON_BIT_MASK             0b01001111
#define CIA_SWITCH_ON_DISABLED_BIT_MASK                 0b01001111
#define CIA_READY_TO_SWITCH_ON_BIT_MASK                 0b01101111
#define CIA_SWITCHED_ON_BIT_MASK                        0b01101111
#define CIA_OPERATION_ENABLED_BIT_MASK                  0b01101111
#define CIA_QUICK_STOP_ACTIVE_BIT_MASK                  0b01101111
#define CIA_FAULT_REACTION_ACTIVE_BIT_MASK              0b01001111
#define CIA_FAULT_BIT_MASK                              0b01001111



#define CIA_WARNING_BIT_NUM                     (7)
//bit 8 can be STO or isMoving
#define CIA_REMOTE_BIT_NUM                      (9)
#define CIA_TARGET_REACHED                      (10)
#define CIA_INTERNAL_LIMIT_BIT_NUM              (11)
#define CIA_FOLLOW_ERROR_BIT_NUM                (13)


/*  modes of operation (MOO) */
#define CIA_MOO_OP_DISABLED                        		0
#define CIA_MOO_PROFILE_POS                        		1
#define CIA_MOO_PROFILE_VEL                       	 	3
#define CIA_MOO_HOMING                            		6
#define CIA_MOO_CSP                                		8 //we use this
#define CIA_MOO_CSV                                		9




cia_commands_t cia_ctrlwrd_to_command(uint16_t controlWord);
cia_state_t cia_statwrd_to_state(uint16_t statusWord);
uint16_t cia_command_to_ctrlwrd(cia_commands_t command);
uint16_t cia_state_to_statwrd(cia_state_t state);



#endif /* GCLIB__CIA402_H_ */
