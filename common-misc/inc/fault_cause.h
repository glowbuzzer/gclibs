/**
 ******************************************************************************
 * @file           :  fault_cause.h
 * @brief          :
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */


#ifndef GCLIB_FAULT_CAUSE_H
#define GCLIB_FAULT_CAUSE_H


/* fault cause bit numbers in machine status word */
#define FAULT_CAUSE_ESTOP_BIT_NUM                    (0)
#define FAULT_CAUSE_DRIVE_FAULT_BIT_NUM                (1)
#define FAULT_CAUSE_GBC_FAULT_REQUEST_BIT_NUM        (2)
#define FAULT_CAUSE_HEARTBEAT_LOST_BIT_NUM            (3)
#define FAULT_CAUSE_LIMIT_REACHED_BIT_NUM                (4)
#define FAULT_CAUSE_DRIVE_STATE_CHANGE_TIMEOUT_BIT_NUM    (5)
#define FAULT_CAUSE_DRIVE_FOLLOW_ERROR_BIT_NUM            (6)
#define FAULT_CAUSE_DRIVE_NO_REMOTE_BIT_NUM            (7)
#define FAULT_CAUSE_ECAT_BIT_NUM                        (8)
#define FAULT_CAUSE_DRIVE_ALARM_BIT_NUM                 (9)
#define FAULT_CAUSE_GBC_INTERNAL_ERROR_BIT_NUM          (10)
#define FAULT_CAUSE_DRIVE_MOOERROR_BIT_NUM              (11)
#define FAULT_CAUSE_ECAT_SLAVE_ERROR_BIT_NUM            (12)
#define FAULT_CAUSE_PLC_SIGNALLED_ERROR_BIT_NUM         (13)
#define FAULT_CAUSE_HOMING_ERROR_BIT_NUM                (14)
#define FAULT_CAUSE_GBC_TO_PLC_CON_ERROR_BIT_NUM        (15)
#define FAULT_CAUSE_MOVE_NOT_OP_EN_BIT_NUM              (16)
#define FAULT_CAUSE_CST_CSV_POSITION_LIMIT_ERROR_BIT_NUM (17)
#endif //GCLIB_FAULT_CAUSE_H
