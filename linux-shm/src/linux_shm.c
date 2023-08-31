
/**
 ******************************************************************************
 * @file           :  linux_shm.c
 * @brief          :
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */

#include "linux_shm.h"
#include "user_message.h"
#include <errno.h>
#include "pid.h"
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <string.h>


#define GBC_SHARED_MEMORY_NAME "gbc_shared_memory"

char gbc_shared_mem_name[100] = GBC_SHARED_MEMORY_NAME;


#define ALIVE_TIMEOUT_SECS 10

/**
 * @brief establishes a shared mem connection to and works out the pid to send signal to
 * @param proc (process name (of GBC)
 * @return
 */
gberror_t
establish_shared_mem_and_signal_con(struct shm_msg **shared_mem, char *proc, const bool retry, int *pid, int um_en) {
    int gbc_alive_timeout = 0;
    bool connected = false;
    gberror_t grc = 0;

    UM_INFO(um_en,
            "LINUX_SHM: Check if other process is running and has notified us to start processing messages from the shared memory");

    while (1) {

        grc = establish_shared_mem_con(shared_mem, um_en);


        if (*shared_mem == NULL) {
            UM_FATAL("LINUX_SHM: Null shared mem pointer");
        }

        if (grc == E_SUCCESS) {
            //we have a shared mem connection to GBC
            *pid = find_matching_pid(proc, um_en);

            //no process matching name found
            if (*pid == -1) {
                UM_ERROR(um_en,
                         "LINUX_SHM: No [%s] process found - we need a matching PID",
                         proc);
                *pid = 0;
            }

            //multiple processes matching name found
            if (*pid == -2) {
                UM_ERROR(um_en,
                         "LINUX_SHM: Multiple processes found, please kill the superfluous one");
                *pid = 0;
            }

            //single process matching name found
            if (*pid > 0) {
                UM_INFO(um_en, "LINUX_SHM: [%s] is running as process id [%d]", proc, *pid);
                //magic number is useful but persists after gbc is closed and rerun
                if (1) {
//                if ((*shared_mem)->gbc_alive == SHM_MAGIC_NUMBER) {
                    UM_INFO(um_en,
                            "LINUX_SHM: The shared mem has had the correct magic number written to it [0x%02X]",
                            SHM_MAGIC_NUMBER);
                    connected = true;
                    break;
                } else {
                    UM_INFO(um_en,
                            "LINUX_SHM: The shared mem does NOT have the correct magic number written to it");
                }
            }
        } else {
            UM_ERROR(um_en, "LINUX_SHM: Could not connect to shared memory [%s]", gb_strerror(grc));
        }
        if (retry) {
            sleep(5);
            gbc_alive_timeout++;
            if (gbc_alive_timeout > ALIVE_TIMEOUT_SECS / 5) {
                break;
            }
        } else {
            break;
        }
    } //end while(1) loop


    if (!connected) {
        UM_ERROR(um_en,
                 "LINUX_SHM: We failed to respond over shared memory or no process found");
        return E_TIMEOOUT;
    } else {
        UM_INFO(um_en, "LINUX_SHM: We have a shared memory and signal connection to the other process");
    }
    return E_SUCCESS;

}


gberror_t establish_shared_mem_con(struct shm_msg **shared_mem, int um_en) {
    int rc;
//    int shm_open_fd = shm_open(gbc_shared_mem_name, O_CREAT | O_RDWR, S_IRWXU | S_IRWXG);
    int shm_open_fd = shm_open(gbc_shared_mem_name, O_CREAT | O_RDWR, 0777);


    //O_CREAT Create the shared memory object if it does not exist
    //O_RDWR Open the object for read-write access
//    S_IRWXU user rw permission bit
//    S_IRWXG group rw permission bit

    if (shm_open_fd < 0) {
        if (shm_open_fd == EACCES) {
            UM_ERROR(um_en, "LINUX_SHM: Could not open shared memory region. Permission was denied");
            return E_SHARED_MEM_INIT_FAILURE;
        } else if (shm_open_fd == EINVAL) {
            UM_ERROR(um_en, "LINUX_SHM: Could not open shared memory region. Invalid name for shared memory region");
            return E_SHARED_MEM_INIT_FAILURE;
        } else if (shm_open_fd == EMFILE) {
            UM_ERROR(um_en,
                     "LINUX_SHM: Could not open the shared memory region. The per-process limit on the number of open file descriptors has been reached");
            return E_SHARED_MEM_INIT_FAILURE;
        } else if (shm_open_fd == ENAMETOOLONG) {
            UM_ERROR(um_en, "LINUX_SHM: Could not open shared memory region. The length of name exceeds PATH_MAX");
            return E_SHARED_MEM_INIT_FAILURE;
        } else if (shm_open_fd == ENFILE) {
            UM_ERROR(um_en,
                     "LINUX_SHM: Could not open shared memory region. The system-wide limit on the total number of open files has been reached. This is fatal and we can't continue");
            return E_SHARED_MEM_INIT_FAILURE;
        } else {
            UM_ERROR(um_en, "LINUX_SHM: Could not open shared memory region");
            return E_SHARED_MEM_INIT_FAILURE;
        }
    }

    UM_INFO(um_en, "LINUX_SHM: Shared Memory opened successfully. Name [%s], size [%d]", gbc_shared_mem_name,
            sizeof(struct shm_msg));


    rc = ftruncate(shm_open_fd, sizeof(struct shm_msg));
    if (rc != 0) {
        UM_ERROR(um_en, "LINUX_SHM: Could not truncate shared memory region [%s]", strerror(errno));
        return E_SHARED_MEM_INIT_FAILURE;
    }

    *shared_mem = mmap(0, sizeof(struct shm_msg), PROT_READ | PROT_WRITE, MAP_SHARED, shm_open_fd, 0);

    if (*shared_mem == NULL) {
        UM_ERROR(um_en, "LINUX_SHM: The shared memory region was a NULL pointer. This is very bad");
        return E_SHARED_MEM_INIT_FAILURE;
    }


    if (*shared_mem == MAP_FAILED) {
        UM_ERROR(um_en, "LINUX_SHM: Could not map to the shared memory region [%s]", strerror(errno));
        return E_SHARED_MEM_INIT_FAILURE;
    }
    close(shm_open_fd);
    return E_SUCCESS;
}

