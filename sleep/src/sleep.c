
/**
 ******************************************************************************
 * @file           :  sleep.c
 * @brief          :
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */
#include <stdint.h>
#include <time.h>
#include <unistd.h>

int nsleep_micro(uint64_t microseconds) {
    struct timespec req, rem;

    if (microseconds > 999) {
        req.tv_sec = (int) (microseconds / 1000000);                            /* Must be Non-Negative */
        req.tv_nsec = (__syscall_slong_t) (microseconds - ((uint64_t) req.tv_sec * 1000)) * 1000; /* Must be in range of 0 to 999999999 */
    } else {
        req.tv_sec = 0;                         /* Must be Non-Negative */
        req.tv_nsec = (__syscall_slong_t) microseconds * 1000;    /* Must be in range of 0 to 999999999 */
    }

    return nanosleep(&req, &rem);
}






/**
 * @brief sleeps for a number of seconds - should be immune to signal interruption
 * @param sec
 */
void true_sleep(int sec) {
    struct timespec ts_start;
    struct timespec ts_end;

    clock_gettime(CLOCK_MONOTONIC, &ts_start);

    ts_end = ts_start;
    ts_end.tv_sec += sec;

    for (;;) {
        struct timespec ts_current;
        struct timespec ts_remaining;

        clock_gettime(CLOCK_MONOTONIC, &ts_current);

        ts_remaining.tv_sec = ts_end.tv_sec - ts_current.tv_sec;
        ts_remaining.tv_nsec = ts_end.tv_nsec - ts_current.tv_nsec;
        while (ts_remaining.tv_nsec > 1000000000) {
            ts_remaining.tv_sec++;
            ts_remaining.tv_nsec -= 1000000000;
        }
        while (ts_remaining.tv_nsec < 0) {
            ts_remaining.tv_sec--;
            ts_remaining.tv_nsec += 1000000000;
        }
        if (ts_remaining.tv_sec < 0) {
            break;
        }
        if (ts_remaining.tv_sec > 0) {
            sleep(ts_remaining.tv_sec);
        } else {
            usleep(ts_remaining.tv_nsec / 1000);
        }
    }
}

