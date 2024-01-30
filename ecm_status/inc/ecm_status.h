/**
 ******************************************************************************
 * @file           :  ecm_status.h
 * @brief          :
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */


#ifndef GBEM_ECM_STATUS_H
#define GBEM_ECM_STATUS_H

#include "cia402.h"
#include "shared_mem_types.h"

//todo these are plucked from the gbem code base
#define EC_MAXNAME                                      40
#define EC_MAXERRORNAME                                 127
#define MAX_NUM_SLAVE_ERROR_MESSAGES                    10
#define MAX_DRIVE_ERROR_MSG_LENGTH                      300
#define MAX_DRIVE_NAME_LENGTH                           30
#define EC_MAXSLAVE                                     30
#define MAP_MAX_NUM_DRIVES                              10
#define MAP_MAX_NUM_FSOE_SLAVES                         10


/* enums for the state of the different programs that can be run */
typedef enum {
    ECM_PRE_BOOT,
    ECM_BOOT_FINISHED,
    ECM_CYCLIC_RUNNING,
    ECM_ERROR
} ecm_cyclic_state_t;

typedef enum {
    ECM_NO_PROG,
    ECM_NET_SCAN_PROG,
    ECM_CYCLIC_PROG,
    ECM_PRINT_CONFIG_PROG,
    ECM_WRITE_NVRAM_PROG,
    ECM_NET_SCAN_PDO_PROG
} ecm_active_program_t;

typedef enum {
    ECM_NET_SCAN_PRE_START,
    ECM_NET_SCAN_START,
    ECM_NET_SCAN_ERROR,
    ECM_NET_SCAN_NO_SLAVES_FOUND,
    ECM_NET_SCAN_FINISHED
} ecm_net_scan_state_t;

/** used in ecm_status to track the progress of the main boot*/
typedef struct {
    bool init_done;
    bool slaves_found;
    bool all_slaves_pre_op;
    bool error_check_ok;
    bool all_slaves_safe_op;
    bool pdo_remap_done;
    bool apply_standard_sdos_done;
    bool wkc_check_ok;
    bool slaves_match_ok;
    bool all_slaves_op;
    bool boot_sucessful;
} ecm_boot_state_t;


/** struct used in ecm_status to hold a copy of key slave state info*/
typedef struct {
    char name[EC_MAXNAME + 1]; //const
    uint16_t Obits; //const
    uint16_t Ibits; //const
    uint16_t Obytes; //const
    uint16_t Ibytes; //const
    bool hasdc; //const
    //this is the ethercat state
    uint16_t state;
    uint16_t configadr; //const
    uint16_t ALstatuscode;
} ecm_status_map_t;


typedef struct {
    uint8_t slave_error_message[MAX_NUM_SLAVE_ERROR_MESSAGES][EC_MAXERRORNAME];
    int head;
    int tail;
    uint8_t num_slots_full;
} ec_circular_slave_error_message_t;

/** nested in ecm_status and holds drive status info */
typedef struct {
    char error_message[MAX_DRIVE_ERROR_MSG_LENGTH];
    char historical_error_message[MAX_DRIVE_ERROR_MSG_LENGTH];
    cia_commands_t command;
    cia_state_t state;
    char name[MAX_DRIVE_NAME_LENGTH];
    char secondary_name[MAX_DRIVE_NAME_LENGTH];
    int8_t cmd_moo;
    int8_t act_moo;
    bool active_internal_limit;
    bool historic_internal_limit;
    bool active_follow_error;
    bool historic_follow_error;
} ecm_status_drive_t;


// typedef enum {
//     FSOE_SLAVE_TYPE_NONE,
//     FSOE_SLAVE_TYPE_SYNAPTICON,
//     FSOE_SLAVE_TYPE_EL1904,
//     FSOE_SLAVE_TYPE_EL2904,
//     FSOE_SLAVE_TYPE_SCU_1_EC,
//     FSOE_SLAVE_TYPE_EL6900,
//     FSOE_SLAVE_TYPE_EL6910,
//     FSOE_SLAVE_TYPE_SICK_MICROSCAN3
// } ecm_fsoe_slave_type_t;


typedef enum {
    BBH_SCU_MODE_NONE,
    //0
    BBH_SCU_MODE_START_UP,
    //1
    BBH_SCU_MODE_SENDCONFIG,
    //2
    BBH_SCU_MODE_STARTUP_BUS,
    //3
    BBH_SCU_MODE_RUN,
    //4
    BBH_SCU_MODE_STOP,
    //5
    BBH_SCU_MODE_ERROR,
    //6
    BBH_SCU_MODE_ALARM,
    //7
    BBH_SCU_MODE_LOCAL_MODE,
    //8
} bbh_scu_mode_t;


// typedef enum {
//     FSOE_SLAVE_HIGH_LEVEL_STATE_NONE,
//     FSOE_SLAVE_HIGH_LEVEL_STATE_ERROR,
//     FSOE_SLAVE_HIGH_LEVEL_STATE_ACK_REQ,
//     FSOE_SLAVE_HIGH_LEVEL_STATE_ERROR_AND_ACK_REQ,
// } fsoe_slave_high_level_state_t;

// typedef enum {
//     FSOE_MASTER_HIGH_LEVEL_STATE_NONE,
//     //0
//     FSOE_MASTER_HIGH_LEVEL_STATE_START_UP,
//     //1
//     FSOE_MASTER_HIGH_LEVEL_STATE_SENDCONFIG,
//     //2
//     FSOE_MASTER_HIGH_LEVEL_STATE_STARTUP_BUS,
//     //3
//     FSOE_MASTER_HIGH_LEVEL_STATE_RUN,
//     //4
//     FSOE_MASTER_HIGH_LEVEL_STATE_STOP,
//     //5
//     FSOE_MASTER_HIGH_LEVEL_STATE_ERROR,
//     //6
//     FSOE_MASTER_HIGH_LEVEL_STATE_ALARM,
//     //7
//     FSOE_MASTER_HIGH_LEVEL_STATE_NO_NETWORK //8
// } fsoe_master_high_level_state_t;


typedef struct {
    uint8_t master_slave_no; //set in control.c
    uint8_t slave_count; // set in ec_functions.c
    enum FSOE_SLAVE_TYPE slave_type[EC_MAXSLAVE]; // set in ec_functions.c
    uint32_t master_state;
    uint32_t master_error_code;
    uint32_t slave_state[EC_MAXSLAVE];
    uint16_t slave_connection_id[EC_MAXSLAVE];
    enum FSOE_SLAVE_HIGH_LEVEL_STATE slave_high_level_state[EC_MAXSLAVE];
    enum FSOE_MASTER_HIGH_LEVEL_STATE master_high_level_state;
} ecm_status_fsoe_t;

/** This struct holds the config and status of whole machine
 *  It is used by the status writing program
 *  Status runs on another core so it doesnt have access to everything
 *  The other core does have access to dpm so there should be no overlap
*/
typedef struct {
    ecm_cyclic_state_t cyclic_state;
    ecm_boot_state_t boot_state;
    ecm_net_scan_state_t net_scan_state;
    ecm_active_program_t active_program;
    //ec_map is the struct copies into from soem slave struct array
    ecm_status_map_t map[EC_MAXSLAVE]; //this is key bits of the slave state struct
    uint64_t cycle_count;
    uint8_t drive_count; //const
    bool gbc_connected;
    ec_circular_slave_error_message_t slave_error_messages;
    bool ec_check_found_error;
    uint8_t slavecount;
    ecm_status_drive_t drives[MAP_MAX_NUM_DRIVES];
    cia_state_t machine_state;
    cia_state_t commanded_machine_state;
    uint64_t shared_mem_busy_count;
    ecm_status_fsoe_t fsoe;
} ecm_status_t;

//#ifdef GB_APP_LINUX
extern ecm_status_t ecm_status;
//#endif

extern const char *ecm_cyclic_state_names[];
extern const char *ecm_active_program_names[];
extern const char *ecm_net_scan_names[];


#endif //GBEM_ECM_STATUS_H
