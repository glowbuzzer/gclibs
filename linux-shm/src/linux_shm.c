
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
#include <sys/sem.h>

#define GBC_SHARED_MEMORY_NAME "gbc_shared_memory"
#define GBC_NAMED_TRIGGER_SEMAPHORE_NAME "/gbc_named_trigger_semaphore"
#define GBC_NAMED_MEM_PROTECTION_SEMAPHORE_NAME "/gbc_named_mem_protection_semaphore"
#define GBC_NAMED_OFFLINE_MEM_PROTECTION_SEMAPHORE_NAME "/gbc_named_offline_mem_protection_semaphore"

char gbc_shared_mem_name[100] = GBC_SHARED_MEMORY_NAME;


/**
 * @brief establishes a shared mem connection to and works out the pid to send signal to
 * @param proc (process name (of GBC)
 * @return
 */
gberror_t
establish_shared_mem_and_signal_con(struct shm_msg **shared_mem, int um_en, bool gbem_or_gbc) {

    gberror_t grc_sm = 0;

    UM_INFO(um_en,
            "LINUX_SHM: Create shared memory and semaphores");


    grc_sm = establish_shared_mem_con(shared_mem, um_en);

    if (*shared_mem == NULL) {
        UM_FATAL("LINUX_SHM: Null shared mem pointer");
    }

    if (grc_sm == E_SUCCESS) {

        if (gbem_or_gbc == true) {
            //we have been called from gbem

            gbc_named_trigger_semaphore = create_named_semaphore(GBC_NAMED_TRIGGER_SEMAPHORE_NAME, 1);
            gbc_named_mem_protection_semaphore = create_named_semaphore(GBC_NAMED_MEM_PROTECTION_SEMAPHORE_NAME, 1);
            gbc_named_offline_mem_protection_semaphore = create_named_semaphore(
                    GBC_NAMED_OFFLINE_MEM_PROTECTION_SEMAPHORE_NAME, 1);

            if (sem_init(gbc_named_trigger_semaphore, 1, 1) == -1) {
                UM_FATAL("LINUX_SHM: Could not reset gbc_named_trigger_semaphore semaphore [%s]", strerror(errno));
            }

            if (sem_init(gbc_named_mem_protection_semaphore, 1, 1) == -1) {
                UM_FATAL("LINUX_SHM: Could not reset gbc_named_mem_protection_semaphore semaphore [%s]",
                         strerror(errno));
            }

            if (sem_init(gbc_named_offline_mem_protection_semaphore, 1, 1) == -1) {
                UM_FATAL("LINUX_SHM: Could not reset gbc_named_offline_mem_protection_semaphore semaphore [%s]",
                         strerror(errno));
            }


        } else {
            //we have been called from gbc
            gbc_named_trigger_semaphore = create_named_semaphore(GBC_NAMED_TRIGGER_SEMAPHORE_NAME, 1);
            gbc_named_mem_protection_semaphore = create_named_semaphore(GBC_NAMED_MEM_PROTECTION_SEMAPHORE_NAME, 1);
            gbc_named_offline_mem_protection_semaphore = create_named_semaphore(
                    GBC_NAMED_OFFLINE_MEM_PROTECTION_SEMAPHORE_NAME, 1);


        }


    }


    return E_SUCCESS;

}


gberror_t establish_shared_mem_con(struct shm_msg **shared_mem, int um_en) {
    int rc;

    int shm_open_fd = shm_open(gbc_shared_mem_name, O_CREAT | O_RDWR, 0777);


    //O_CREAT Create the shared memory object if it does not exist
    //O_RDWR Open the object for read-write access
//    S_IRWXU user rw permission bit
//    S_IRWXG group rw permission bit

//    0666 (rw-rw-rw-): This permission mode allows both read and write access to all users on the system. It's quite permissive and should be used with caution. If you use 0666, any process on the system can potentially access and modify the semaphore, which might not be suitable for all use cases.
//  0777 (rwxrwxrwx): This permission mode grants full read, write, and execute permissions to all users. It's even more permissive than 0666 and is generally discouraged for most situations because it allows any user to execute the semaphore as if it were a program.


    if (shm_open_fd < 0) {
        if (errno == EACCES) {
            UM_ERROR(um_en, "LINUX_SHM: Could not open shared memory region. Permission was denied");
            return E_SHARED_MEM_INIT_FAILURE;
        } else if (errno == EINVAL) {
            UM_ERROR(um_en, "LINUX_SHM: Could not open shared memory region. Invalid name for shared memory region");
            return E_SHARED_MEM_INIT_FAILURE;
        } else if (errno == EMFILE) {
            UM_ERROR(um_en,
                     "LINUX_SHM: Could not open the shared memory region. The per-process limit on the number of open file descriptors has been reached");
            return E_SHARED_MEM_INIT_FAILURE;
        } else if (errno == ENAMETOOLONG) {
            UM_ERROR(um_en, "LINUX_SHM: Could not open shared memory region. The length of name exceeds PATH_MAX");
            return E_SHARED_MEM_INIT_FAILURE;
        } else if (errno == ENFILE) {
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


/** create a named sempahore */
sem_t *create_named_semaphore(const char *name, const int value) {
    sem_t *semaphore = sem_open(name, O_CREAT, 0777, value);
    if (semaphore == SEM_FAILED) {
        UM_FATAL("LINUX_SHM: Could not create semaphore [%s]", strerror(errno));
        return NULL;
    }
    return semaphore;
}

