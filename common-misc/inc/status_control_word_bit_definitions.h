/**
 ******************************************************************************
 * @file           :  status_control_word_bit_definitions
 * @brief          :
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */
#ifndef GBEM_STATUS_CONTROL_WORD_BIT_DEFINITIONS_H
#define GBEM_STATUS_CONTROL_WORD_BIT_DEFINITIONS_H

//#define STATUS_WORD_GBEM_ALIVE_BIT_NUM                      (16)
//#define STATUS_WORD_GBEM_BOOT_IN_PROGRESS_BIT_NUM           (17)
//#define STATUS_WORD_GBEM_BOOTED_BIT_NUM                     (18)
//#define STATUS_WORD_GBEM_HOMING_NEEDED_BIT_NUM              (19)
//#define STATUS_WORD_GBEM_WAITING_FOR_START_HOMING_BIT_NUM   (20)
//#define STATUS_WORD_GBEM_HOMING_IN_PROGRESS_BIT_NUM         (21)
//#define STATUS_WORD_GBEM_HOMING_ERROR_BIT_NUM               (23)
//#define STATUS_WORD_GBEM_HOMING_ATTAINED_BIT_NUM            (24)

// TODO: M: update gbem with new define, CONTROL_WORD_GBC_INTERNAL_FAULT_REQ_BIT_NUM is deprecated
#define CONTROL_WORD_GBC_INTERNAL_FAULT_REQ_BIT_NUM         (CONTROL_WORD_GBC_OPERATION_ERROR_BIT_NUM)
// GBEM responds with FAULT_CAUSE_GBC_INTERNAL_ERROR_BIT_NUM
// old def #define CTRL_GBC_INTERNAL_FAULT_REQ_BIT_NUM                 (16)

#define CONTROL_WORD_GBEM_START_HOMING_BIT_NUM              (17)


//#define CONTROL_WORD_GBC_REQUEST_FAULT_BIT_NUM              (18)
// GBEM responds with FAULT_CAUSE_GBC_FAULT_REQUEST_BIT_NUM
//old def #define CTRL_MACHINE_CTRL_WRD_REQUEST_FAULT_BIT_NUM         (18)

//#define CONTROL_WORD_MOVE_NOT_OP_ENABLED_FAULT_REQ_BIT_NUM  (19)
// GBEM responds with FAULT_CAUSE_MOVE_NOT_OP_EN_BIT_NUM
//old def #define CTRL_MOVE_NOT_OP_ENABLED_FAULT_REQ_BIT_NUM          (19)

//#define CONTROL_WORD_GBEM_REBOOT_BIT_NUM                    (20)


#endif //GBEM_STATUS_CONTROL_WORD_BIT_DEFINITIONS_H
