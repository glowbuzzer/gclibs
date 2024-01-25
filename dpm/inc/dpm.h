/**
 ******************************************************************************
 * @file           :  dpm.h
 * @brief          :
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */


#ifndef GCLIB__DPM_H
#define GCLIB__DPM_H


//#include "std_headers.h"
//#include "shared.h"
#include "ecm_status.h"

#include <stdint.h>

#define DPM_NUM_JOINTS 10
#define DPM_NUM_ANALOGS 6
#define DPM_NUM_INT32S 2
#define DPM_NUM_UINT32S 0
#define DPM_SIZE_OF_EXTERNAL 32
// ensure digitals is multiple of 64
// NOTE: if more than a single uint64_t is used, order will be confusing - be warned!
#define DPM_NUM_DIGITALS 64
#define DPM_NUM_SAFETY_DIGITALS 64
#define SIZE_OF_GBC_OFFLINE 20000

typedef struct {
    uint32_t machine_word; //the machine's overall status (CiA-402)
    uint32_t active_fault_word; //bits that define any active faults with the machine
    uint32_t fault_history_word; //bits that define any fault events that resulted in an error
    uint32_t heartbeat; // a periodically increasing heartbeat used to detect we are connected
    uint16_t joint_statusword[DPM_NUM_JOINTS]; //the CiA-402 status for drives
    int32_t joint_actual_position[DPM_NUM_JOINTS]; //actual position of the drives
    int32_t joint_actual_velocity[DPM_NUM_JOINTS]; //actual velocity of the drives
    int32_t joint_actual_torque[DPM_NUM_JOINTS]; //actual torque applied by the drives
    int32_t joint_actual_follow_error[DPM_NUM_JOINTS]; //actual follow error of the drives
    uint64_t digital[DPM_NUM_DIGITALS / 64]; // state of digital ins
    uint64_t safetyDigital[DPM_NUM_SAFETY_DIGITALS / 64]; // state of digital ins
    float analog[DPM_NUM_ANALOGS]; //state of float ins
    int32_t integer32[DPM_NUM_INT32S]; //status of signed integers ins
    uint8_t reserved[4];
    uint32_t unsigned32[DPM_NUM_UINT32S]; //status of unsigned integer ins
    uint8_t external[DPM_SIZE_OF_EXTERNAL]; //status of external ins
}__attribute__((packed)) dpm_in_t;

typedef struct {
    uint32_t machine_word; // commanded (CiA-402) state of machine
    uint32_t hlc_control_word; // we dont what this is for
    uint32_t gbc_control_word; // move gbc fault and move not op en
    uint32_t heartbeat; // a periodically increasing heartbeat used to detect we are connected
    uint16_t joint_controlword[DPM_NUM_JOINTS]; // CiA-402 control word for the drives (not used if say GBEM is commanding teh drives)
    int32_t joint_set_position[DPM_NUM_JOINTS]; // set position for drives (usually only this is used not velocity and torque)
    int32_t joint_set_velocity[DPM_NUM_JOINTS]; // set velocity for drives
    int32_t joint_set_torque[DPM_NUM_JOINTS]; // set torque for drives
    int32_t joint_set_torque_offset[DPM_NUM_JOINTS]; // set torque offset for drives
    uint64_t digital[DPM_NUM_DIGITALS / 64]; // state of digital ins
    uint64_t safetyDigital[DPM_NUM_SAFETY_DIGITALS / 64]; // state of digital ins
    float analog[DPM_NUM_ANALOGS]; // commanded values for float outs
    uint32_t unsigned32[DPM_NUM_UINT32S]; // commandsed values for unsigned integer outs
    uint8_t reserved[4];
    int32_t integer32[DPM_NUM_INT32S]; // commanded values for signed integer outs
    uint8_t external[DPM_SIZE_OF_EXTERNAL]; //status of external outs
}__attribute__((packed)) dpm_out_t;

#define SIZE_OF_GBC_PDO (sizeof(dpm_in_t))

extern uint8_t inA[SIZE_OF_GBC_PDO];
extern uint8_t outA[SIZE_OF_GBC_PDO];
extern uint8_t inB[SIZE_OF_GBC_PDO];
extern uint8_t outB[SIZE_OF_GBC_PDO];

extern dpm_in_t *dpm_in;
extern dpm_out_t *dpm_out;

//gbem->gbc
typedef struct {
    ecm_status_t ecm_status;
}__attribute__((packed)) dpm_offline_in_t;

//gbc->gbem
typedef struct {
    uint32_t tbd;
}__attribute__((packed)) dpm_offline_out_t;


#endif