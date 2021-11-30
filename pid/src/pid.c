
/**
 ******************************************************************************
 * @file           :  pid.c
 * @brief          :
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */

#include "log.h"
#include <string.h>
#include "std_headers.h"
#include "user_message.h"
#include <stdio.h>

/**
 * @brief scans the process list for a pid matching a process name
 * @param proc_name_concat process name
 * @return pid (int) matching (-1 if no mathcing pid, -2 if multiple matching pids)
 */
int find_matching_pid(char *proc_name, int um_en) {
    char pidline[1024] = {0};
    char *pid;
    int pid_array = 0;
    int pidno[64];
    FILE *fp;

    char proc_name_concat[1024];


    strcpy(proc_name_concat, "pidof ");


    strcat(proc_name_concat, proc_name);


    fp = popen(proc_name_concat, "r");


    if ( !fgets(pidline, 1024, fp) ) {
        // keep compiler happy about unused return from fgets

            UM_WARN(um_en, "PID: Failed to read line using popen (we can't find a Linux process that matches the name [%s])", proc_name);

    }


    pid = strtok(pidline, " ");
    while (pid != NULL) {
        pidno[pid_array] = atoi(pid);

        UM_INFO(um_en, "PID: We found a PID (%d) matching process name: %s", pidno[pid_array],
                proc_name);
        pid = strtok(NULL, " ");
        pid_array++;
    }
    pclose(fp);


    if (pid_array == 0) {
        //no pid matching the process name found
        return (-1);
    } else if (pid_array > 1) {
        //multiple pids matching the process name found
        return (-2);
    } else {
        // single matching pid found returning the pid
        return (pidno[0]);
    }
}
