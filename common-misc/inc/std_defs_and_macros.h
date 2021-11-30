/**
 ******************************************************************************
 * @file           :  std_defs_and_macros.h
 * @brief          :
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2022 Glowbuzzer.
 * All rights reserved.</center></h2>
 *
 ******************************************************************************
 */



#ifndef _STD_DEFS_AND_MACROS
#define _STD_DEFS_AND_MACROS


#include <setjmp.h>

/* For the full documentation and explanation of the code below, please refer to
 * http://www.di.unipi.it/~nids/docs/longjump_try_trow_catch.html
 */

#define TRY do { jmp_buf ex_buf__; switch( setjmp(ex_buf__) ) { case 0: while(1) {
#define CATCH(x) break; case x:
#define FINALLY break; } default: {
#define ETRY break; } } }while(0)
#define THROW(x) longjmp(ex_buf__, x)

#define sizearray(a)  (sizeof(a) / sizeof((a)[0]))

//macro to test one bit from array of uint8_t - returns 1 if set
#define TestBitInArray(A, k)    (((A)[(k / 8u)] & (1u << (k % 8u))) >> (k % 8u))
//macro to set one bit
#define SetBitInArray(A, k)     ((A)[(k / 8u)] |= (1u << (k % 8u)) )
//macro to clear on bit
#define ClearBitInArray(A, k)   ((A)[(k / 8u)] &= ~(1u << (k % 8u)) )


#define NSEC_PER_SEC 1000000000
#define DIFF_NS(A, B) (((B).tv_sec - (A).tv_sec) * NSEC_PER_SEC + \
    (B).tv_nsec - (A).tv_nsec)

///check the nth bit from the right end
//#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))
/* a=target variable, b=bit number to act upon 0-n */
#define BIT_SET(a,b) ((a) |= (1ULL<<(b)))
#define BIT_CLEAR(a,b) ((a) &= ~(1ULL<<(b)))
#define BIT_FLIP(a,b) ((a) ^= (1ULL<<(b)))
#define BIT_CHECK(a,b) (!!((a) & (1ULL<<(b))))        // '!!' to make sure this returns 0 or 1

/* x=target variable, y=mask */
#define BITMASK_SET(x,y) ((x) |= (y))
#define BITMASK_CLEAR(x,y) ((x) &= (~(y)))
#define BITMASK_FLIP(x,y) ((x) ^= (y))
#define BITMASK_CHECK_ALL(x,y) (((x) & (y)) == (y))   // warning: evaluates y twice
#define BITMASK_CHECK_ANY(x,y) ((x) & (y))

#ifndef ABS
#define ABS(x) ((x) < 0 ? -(x) : (x))
#endif

//extract set of bits
#define LAST(k,n) ((k) & ((1<<(n))-1))
//n>m
#define MID(k,m,n) LAST((k)>>(m),((n)-(m)))


/*  Obtain the number of elements in the given C array */
#define GET_ARRAY_LEN( arrayName ) (sizeof( arrayName ) / sizeof(( arrayName)[ 0 ] ))

/* Return min of two numbers. Commonly used but never defined as part of standard headers */
#ifndef MIN
#define MIN( n1, n2 )   ((n1) > (n2) ? (n2) : (n1))
#endif

/* Return max of two numbers. Commonly used but never defined as part of standard headers */
#ifndef MAX
#define MAX( n1, n2 )   ((n1) > (n2) ? (n1) : (n2))
#endif


/* Determine whether the given signed or unsigned integer is odd.*/
#define IS_ODD( num )   ((num) & 1)

/* Determine whether the given signed or unsigned integer is even. */
#define IS_EVEN( num )  (!IS_ODD( (num) ))

/* used for functions that are going to be called from C++ code */
//#if defined(__cplusplus)
//#define EXTERN_C extern "C"
//#else
//#define EXTERN_C extern
//#endif

/* printf %lu and %lu will work here on 64 bit linux but not pi(32 bit) or mbed so these are better for xplatform (if you have uclibc in mbedland) */
#define PRINT_INT64(t) printf("%" PRId64 "\n", t);
#define PRINT_UINT64(t) printf("%" PRIu64 "\n", t);


#define M_ZERO 1e-10
#define M_SMALL 1e-4

#define IS_ZERO(A) ( (A < M_ZERO) && (A > -M_ZERO) )
#define IS_SMALL(A) ( (A < M_SMALL) && (A > -M_SMALL) )
#define IS_SAME(A, B) ( ((A-B) < M_ZERO) && ((A-B) > -M_ZERO) )
#define IS_SIMILAR(A, B) ( ((A-B) < M_SMALL) && ((A-B) > -M_SMALL) )
#define IS_BETWEEN (A, B, C) ( ((A-B) > -M_ZERO) && ((A-C) < M_ZERO) )


//stringizing macros
#define XSTR(x) STR(x)
#define STR(x) #x

/** Macro to make a word from 2 bytes */
#define MK_WORD(msb, lsb)   ((((uint16)(msb))<<8) | (lsb))
/** Macro to get hi byte of a word */
#define HI_BYTE(w)          ((w) >> 8)
/** Macro to get low byte of a word */
#define LO_BYTE(w)          ((w) & 0x00ff)
/** Macro to swap hi and low byte of a word */
#define SWAP(w)             ((((w)& 0xff00) >> 8) | (((w) & 0x00ff) << 8))
/** Macro to get hi word of a dword */
#define LO_WORD(l)          ((l) & 0xffff)
/** Macro to get hi word of a dword */
#define HI_WORD(l)          ((l) >> 16)



#define strdupa(src) (__extension__ ({			\
	size_t len_	= strlen(src);			\
	char *dst_	= __builtin_alloca(len_ + 1);	\
	dst_[len_]	= '\0';				\
	(char *)memcpy(dst_, src, len_);		\
}))


#define DUMP(varname) printf("%s, %d", #varname, (varname).index);


// Generic mask/shift macros
#define REGISTER_GET(data, mask, shift) \
	(((data) & (mask)) >> (shift))

#define REGISTER_SET(data, mask, shift, value) \
	(((data) & (~(mask))) | (((value) << (shift)) & (mask)))


#endif
