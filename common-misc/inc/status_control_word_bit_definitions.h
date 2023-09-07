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

#define STATUS_WORD_GBEM_ALIVE_BIT_NUM                   (16)
#define STAUS_WORD_GBEM_BOOT_IN_PROGRESS_BIT_NUM         (17)
#define STATUS_WORD_GBEM_BOOTED_BIT_NUM                  (18)
#define STATUS_WORD_GBEM_HOMING_NEEDED_BIT_NUM           (19)
#define STATUS_WORD_GBEM_WAITING_FOR_START_HOMING_BIT_NUM (20)
#define STATUS_WORD_GBEM_HOMING_IN_PROGRESS_BIT_NUM      (21)
#define STATUS_WORD_GBEM_HOMING_ERROR_BIT_NUM            (23)
#define STATUS_WORD_GBEM_HOMING_ATTAINED_BIT_NUM         (24)

#define CONTROL_WORD_GBEM_REBOOT_BIT_NUM                 (16)
#define CONTROL_WORD_GBEM_START_HOMING_BIT_NUM           (17)


#endif //GBEM_STATUS_CONTROL_WORD_BIT_DEFINITIONS_H
