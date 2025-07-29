/**
******************************************************************************
 * @file           :  sm_status.h
 * @brief          :
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */

#ifndef SM_STATUS_H
#define SM_STATUS_H

#include "cia402.h"
#include "shared_mem_types.h"

/** used in sm_status to track the progress of the main boot*/
typedef struct {
    bool init_done;
    bool io_initialised;
    bool all_drives_initialised;
    bool all_drives_configured;
    bool all_drives_homed;
    bool all_drives_operational;
    bool boot_successful;
} sm_boot_state_t;

typedef struct {
    cia_commands_t command;
    cia_state_t state;
    int8_t cmd_moo;
    int8_t act_moo;
    bool active_fault;
    bool historic_fault;
    bool active_stall;
    bool historic_stop_on_stall;
    bool active_encoder_fail;
    bool historic_encoder_fail;
    bool active_overtemperature_prewarning;
    bool historic_overtemperature_prewarning;
    bool active_overtemperature;
    bool historic_overtemperature;
    bool active_overcurrent_a;
    bool historic_overcurrent_a;
    bool active_overcurrent_b;
    bool historic_overcurrent_b;
    bool active_open_load_a;
    bool historic_open_load_a;
    bool active_open_load_b;
    bool historic_open_load_b;
    bool historic_home_error;
    bool historic_cl_max; //closed loop commutation angle has reach maximum angle
    bool historic_cl_fit; //closed loop deviation angle has reached inner limit
    bool historic_motor_flag_set;
    bool historic_reset_triggered;
    bool active_stallguard;
    bool active_undervoltage;
    bool historic_undervoltage;
    bool active_short_to_ground_a;
    bool historic_short_to_ground_a;
    bool active_short_to_ground_b;
    bool historic_short_to_ground_b;
    bool active_l_stop;
    bool historic_l_stop;
    bool active_r_stop;
    bool historic_r_stop;
    bool active_l_vstop;
    bool historic_l_vstop;
    bool active_r_vstop;
    bool historic_r_vstop;
    bool frozen;
    bool active_51x0_error;
    bool historic_51x0_error;
} sm_status_drive_t;


 typedef struct {
     bool active;
     bool complete;
     bool error;
}sm_status_homing_t;

typedef struct {
    // ecm_cyclic_state_t cyclic_state;
    sm_boot_state_t boot_state;
    sm_status_homing_t homing_status[6];
    uint64_t cycle_count; //linked
    uint8_t drive_count; // linked
    // bool gbc_connected;
    sm_status_drive_t drives[6];
    cia_state_t machine_state; // linked
    cia_state_t commanded_machine_state; // linked
    uint64_t shared_mem_busy_count; // linked
} sm_status_t;



#endif //SM_STATUS_H
