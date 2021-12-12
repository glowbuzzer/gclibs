/**
 ******************************************************************************
 * @file           :  sim.h
 * @brief          :
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */


#ifndef GCLIB__SIM_H
#define GCLIB__SIM_H

uint32_t cia_sim_control_word(uint32_t nextControlWord, uint32_t *machineState);

#endif //GCLIB__SIM_H
