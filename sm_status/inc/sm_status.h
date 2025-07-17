//
// Created by david on 16/07/2025.
//

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

    // bool slaves_found;
    // bool all_slaves_pre_op;
    // bool error_check_ok;
    // bool all_slaves_safe_op;
    // bool pdo_remap_done;
    // bool apply_standard_sdos_done;
    // bool wkc_check_ok;
    // bool slaves_match_ok;
    // bool all_slaves_op;
    bool boot_successful;
} sm_boot_state_t;

typedef struct {
    // char error_message[MAX_DRIVE_ERROR_MSG_LENGTH];
    // char historical_error_message[MAX_DRIVE_ERROR_MSG_LENGTH];
    cia_commands_t command;
    cia_state_t state;
    // char name[MAX_DRIVE_NAME_LENGTH];
    // char secondary_name[MAX_DRIVE_NAME_LENGTH];
    int8_t cmd_moo;
    int8_t act_moo;
    bool active_internal_limit;
    bool historic_internal_limit;

    bool active_follow_error;
    bool historic_follow_error;

    bool active_fault;
    bool historic_fault;

    bool active_stall;
    bool historic_stall;

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

    bool active_undervoltage;
    bool historic_undervoltage;

    bool active_short_to_ground_a;
    bool historic_short_to_ground_a;
    bool active_short_to_ground_b;
    bool historic_short_to_ground_b;



    // uint32_t error_code; //todo crit - add error code
    // uint32_t historical_error_code; //todo crit - add historical error code
    // uint32_t error_count; //todo crit - add error count
    // uint32_t historical_error_count; //todo crit - add historical error count



} sm_status_drive_t;


 typedef struct {
     bool active;
     bool complete;
     bool error;
}sm_status_homing_t;

typedef struct {
    // bool safety_state;
    // ecm_cyclic_state_t cyclic_state;
    sm_boot_state_t boot_state;
    sm_status_homing_t homin_status[6];
    // ecm_net_scan_state_t net_scan_state;
    // ecm_active_program_t active_program;
    //ec_map is the struct copies into from soem slave struct array
    // ecm_status_map_t map[EC_MAXSLAVE]; //this is key bits of the slave state struct
    uint64_t cycle_count;
    uint8_t drive_count; //const
    bool gbc_connected;
    // ec_circular_slave_error_message_t slave_error_messages;
    // bool ec_check_found_error;
    // uint8_t slavecount;
    sm_status_drive_t drives[6];
    cia_state_t machine_state;
    cia_state_t commanded_machine_state;
    uint64_t shared_mem_busy_count;
    // ecm_status_fsoe_t fsoe;
} sm_status_t;



#endif //SM_STATUS_H
