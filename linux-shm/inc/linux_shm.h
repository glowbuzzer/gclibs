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

#define SHM_BUF_SIZE            200
//#define SHM_KEY                 0x1234

/*magic number used to detect if gbc is alive */
#define SHM_MAGIC_NUMBER        0x13a


extern char gbc_shared_mem_name[100];


struct shm_msg {
    int gbc_alive;
    int in_busy;
    int out_busy;
    uint8_t sm_buf_in[SHM_BUF_SIZE];
    uint8_t sm_buf_out[SHM_BUF_SIZE];
};
//extern struct shm_msg *shmp;

gberror_t establish_shared_mem_and_signal_con(struct shm_msg **shared_mem, char *proc, const bool retry, int *pid, int um_en);
gberror_t establish_shared_mem_con(struct shm_msg **shared_mem, int um_en);


#endif //GCLIB__LINUX_SHM_H
