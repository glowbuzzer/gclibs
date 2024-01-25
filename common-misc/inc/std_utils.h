/**
 ******************************************************************************
 * @file           : std_utils.h
 * @brief          : miscellaneous helper functions
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */

#ifndef GCLIB__STD_UTILS_H
#define GCLIB__STD_UTILS_H

#include <stdint.h>
#include <stddef.h>

#include <sys/types.h>

typedef int32_t fixed_point_t;
#define FIXED_POINT_FRACTIONAL_BITS 8

/// Converts 24.8 format -> double
double fixed_to_float(fixed_point_t input);

/// Converts double to 24.8 format
fixed_point_t float_to_fixed(double input);



typedef enum  {GB_PROGRAM_GBC, GB_PROGRAM_GBEM, GB_PROGRAM_GBSM} gb_program_t;

size_t	sltoa(char *s, long int n);
size_t	ultoa(char *s, unsigned long int n);

void upcase(char *s);

void remove_spaces(char* s);

void printBits(int size, const void *ptr);

void print_register_32(const void *ptr);

        uint32_t set_bits_32 (uint32_t data, uint8_t offset, uint8_t n);

uint32_t unset_bits_32 (uint32_t data, uint8_t offset, uint8_t n);

uint32_t copy_set_bits_32(uint32_t destination, uint32_t source, uint32_t at, uint32_t numbits);

void print_current_time_with_ms(void);

#endif //GCLIB__STD_UTILS_H