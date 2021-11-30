/**
 ******************************************************************************
 * @file           : std_utils.c
 * @brief          : miscellaneous helper functions
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */


#include "std_utils.h"
#include <stdio.h>
#include <stdint.h>
#include <ctype.h>
#include <time.h>
#include <math.h>
#include <inttypes.h>

void print_register_32(void const *const ptr) {
    unsigned char *b = (unsigned char *) ptr;
    unsigned char byte;
    int i, j;

    printf("31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 09 08 07 06 05 04 03 02 01 00\n");
    for (i = 4 - 1; i >= 0; i--) {
        for (j = 7; j >= 0; j--) {
            byte = (b[i] >> j) & 1;
            printf(" %u ", byte);
        }
    }
    puts("");
    printf("uint32 [%u]\n", (*(uint32_t *) ptr));
    printf("int32 [%d]\n", (*(int32_t *) ptr));
    printf("hex [%#08x]\n", (*(int32_t *) ptr));
    printf("fixed point [%f]\n", fixed_to_float((*(int32_t *) ptr)));

}

void printBits(size_t const size, void const * const ptr)
{
    unsigned char *b = (unsigned char*) ptr;
    unsigned char byte;
    int i, j;

    for (i=size-1;i>=0;i--)
        for (j=7;j>=0;j--)
        {
            byte = (b[i] >> j) & 1;
            printf("%u", byte);
        }

    puts("");
}



double fixed_to_float(fixed_point_t input) {
    return ((double) input / (double) (1 << FIXED_POINT_FRACTIONAL_BITS));
}

fixed_point_t float_to_fixed(double input) {
    return (fixed_point_t) (round(input * (1 << FIXED_POINT_FRACTIONAL_BITS)));
}



uint32_t set_bits_32 (uint32_t data, uint8_t offset, uint8_t n)
{
    uint32_t mask = 0xFFFFFFFF >> (32-n);
    return data | (mask << offset);
}

uint32_t unset_bits_32 (uint32_t data, uint8_t offset, uint8_t n)
{
    uint32_t mask =  (uint32_t) ((1 << n) - 1);
    return data & (~(mask << offset));
}


//at position to copy to
uint32_t copy_set_bits_32(uint32_t destination, uint32_t source, uint32_t at, uint32_t numbits)
{
    uint32_t mask = ((~0u)>>(sizeof(int)*8-numbits))<<at; // 2nd aproach
    return (destination&~mask)|((source<<at)&mask);
}


void upcase(char *s)
{
    while (*s)
    {
        *s = toupper(*s);
        s++;
    }
}


/**
 * @brief removes white space
 * @param s
 * @warning don't call on string literals (char * p = "pig" =  ro mem (string literal) whereas char p[] = "sheep" = on stack)
 */
void remove_spaces(char* s) {
    const char* d = s;
    do {
        while (*d == ' ') {
            ++d;
        }
    } while (*s++ = *d++);
}






/*
* Copyright (c) 2010 Przemo Nowaczyk <pnowaczyk.mail@gmail.com>
*
* Permission to use, copy, modify, and distribute this software for any
        * purpose with or without fee is hereby granted, provided that the above
* copyright notice and this permission notice appear in all copies.
*
* THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
* WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
* MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
        * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
* WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
* ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
        * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
*/
#include <sys/types.h>



//these are int to string conversion utils

//char my_char[10];
//sltoa(&my_char, -1999);
//printf("my num: %s\n", my_char);
//$ my num: -1999


static void	reverse(char *, size_t);


/**
 * @brief Convert signed integer to string
 * @param[out] s (array to store converted string)
 * @param[in] n  (int value to be converted to a string)
 * @return size of string
 *
 * Converts an integer value to a null-terminated string - base 10
 */
size_t sltoa(char *s, long int n) {
    size_t i = 0;
    long int sign_mask;
    unsigned long int nn;

    sign_mask = n >> sizeof(long int) * 8 - 1;
    nn = (n + sign_mask) ^ sign_mask;
    do {
        s[i++] = nn % 10 + '0';
    } while (nn /= 10);

    s[i] = '-';
    i += sign_mask & 1;
    s[i] = '\0';

    reverse(s, i);
    return (i);
}


/**
 * @brief Convert unsigned integer to string
 * @param[out] s (array to store converted string)
 * @param[in] n  (int value to be converted to a string)
 * @return size of string
 *
 * Converts an integer value to a null-terminated string - base 10
 */
size_t ultoa(char *s, unsigned long int n) {
    size_t i = 0;

    do {
        s[i++] = n % 10 + '0';
    } while (n /= 10);
    s[i] = '\0';

    reverse(s, i);
    return (i);
}

static void reverse(char *s, size_t s_len) {
    size_t i, j;
    char swap;

    for (i = 0, j = s_len - 1; i < j; ++i, --j) {
        swap = s[i];
        s[i] = s[j];
        s[j] = swap;
    }
}



void print_current_time_with_ms(void) {
    long ms; // Milliseconds
    time_t s;  // Seconds
    struct timespec spec;

    clock_gettime(CLOCK_REALTIME, &spec);

    s = spec.tv_sec;
    ms = round(spec.tv_nsec / 1.0e6); // Convert nanoseconds to milliseconds
    if (ms > 999) {
        s++;
        ms = 0;
    }
    printf("%"PRIdMAX".%03ld\n", (intmax_t) s, ms);
    //    printf("Current time: %"PRIdMAX".%03ld seconds since the Epoch\n", (intmax_t)s, ms);
}