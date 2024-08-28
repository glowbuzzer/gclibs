/**
 ******************************************************************************
 * @file           :  linux_shm.h
 * @brief          :
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */


#ifndef GCLIB__LINUX_SHM_H
#define GCLIB__LINUX_SHM_H

#include "stdint.h"
#include "gberror.h"
#include <stdbool.h>
#include <semaphore.h>

/**The size of the shared memory buffer for offline NON real-time comms */
#define SHM_OFFLINE_BUF_SIZE    20000

/**The size of the shared memory buffer for real-time cyclic comms */
#define SHM_BUF_SIZE            305


extern char gbc_shared_mem_name[100];

extern sem_t *gbc_named_trigger_semaphore;
extern sem_t *gbc_named_mem_protection_semaphore;
extern sem_t *gbc_named_offline_mem_protection_semaphore;


struct shm_msg {
    int gbc_alive;
    int in_busy;
    int out_busy;
    uint8_t sm_buf_in[SHM_BUF_SIZE];
    uint8_t sm_buf_out[SHM_BUF_SIZE];
    uint8_t sm_offline_buf_in[SHM_OFFLINE_BUF_SIZE];
    uint8_t sm_offline_buf_out[SHM_OFFLINE_BUF_SIZE];
};


gberror_t establish_shared_mem_and_signal_con(struct shm_msg **shared_mem, int um_en, bool gbem_or_gbc);

gberror_t establish_shared_mem_con(struct shm_msg **shared_mem, int um_en);

sem_t *create_named_semaphore(const char *name, int value);


#endif //GCLIB__LINUX_SHM_H
