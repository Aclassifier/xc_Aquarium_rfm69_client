/*
 * _globals.h
 *
 *  Created on: 15. aug. 2018
 *      Author: teig
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_

#ifdef GLOBALS_H_ // To show that the below may also be defined in library space

    // BOOLEAN #include <stdbool.h> if C99
    // See http://www.teigfam.net/oyvind/home/technology/165-xc-code-examples/#bool
    typedef enum {false,true} bool; // 0,1 This typedef matches any integer-type type like long, int, unsigned, char, bool

    #define min(a,b) (((a)<(b))?(a):(b))
    #define max(a,b) (((a)>(b))?(a):(b))

    #define t_swap(type,a,b) {type t = a; a = b; b = t;}

    #define NUM_ELEMENTS(array) (sizeof(array) / sizeof(array[0])) // Kernighan & Pike p22

    typedef signed int time32_t; // signed int (=signed) or unsigned int (=unsigned) both ok, as long as they are monotoneously increasing
                                 // XC/XMOS 100 MHz increment every 10 ns for max 2exp32 = 4294967296,
                                 // ie. divide by 100 mill = 42.9.. seconds

    #define AFTER_32(a,b) ((a-b)>0)

    #ifdef DO_ASSERTS
        #define ASSERT_DELAY_32(d) do {if (d > INT_MAX) fail("Overflow");} while (0) // Needs <so646.h<, <limits.h> and <xassert.h>
        // INT_MAX is 2147483647 is what fits into 31 bits or last value before a signed 32 bits wraps around
    #else
        #define ASSERT_DELAY_32(d)
    #endif

    #define NOW_32(tmr,time) do {tmr :> time;} while(0) // A tick is 10ns
    // “Programming XC on XMOS Devices” (Douglas Watt)
    //     If the delay between the two input values fits in 31 bits, timerafter is guaranteed to behave correctly,
    //     otherwise it may behave incorrectly due to overlow or underflow. This means that a timer can be used to
    //     measure up to a total of 2exp31 / (100 mill) = 21s.

#endif

// FOR ONE-ONE connection MASTER/SLAVE
//                ### MUST BE UNIQUE
#define MASTER_ID  99

#define IS_MYTARGET_VOID               0
#define IS_MYTARGET_STARTKIT           1
#define IS_MYTARGET_XCORE_200_EXPLORER 2

#if (MYTARGET==STARTKIT)
    #define IS_MYTARGET IS_MYTARGET_STARTKIT
#elif (MYTARGET==XCORE-200-EXPLORER)
    #define IS_MYTARGET IS_MYTARGET_XCORE_200_EXPLORER
#else
    #error NO TARGET DEFINED
#endif

#if (ISMASTER==1) // Doesn't seem like words like SLAVE or MASTER or _SLAVE or _MASTER work. Using 0 or 1 instead
    #define IS_MYTARGET_MASTER 1
    #define IS_MYTARGET_SLAVE  0

    #define NODEID    MASTER_ID
    #define GATEWAYID SHARED_ID // Since MASTER it knows this SENDTO_ADDRESS
#elif (ISMASTER==0)
    #define IS_MYTARGET_SLAVE  1
    #define IS_MYTARGET_MASTER 0

    // It is the SLAVE that has to get its RETURNTO/SENDTO address from an established connection
    // The MASTER/SLAVE role rationale is also to avoid application level deadlock or oscillation,
    // we would not want both parts to spontaneously send
    #define NODEID SHARED_ID // RECEIVER_ADDRESS is MASTER'S SENDTO_ADDRESS
                             // defined in ... /_Aquarium_1_x/src/_rfm69_commprot.h
    // ANY SENDTO/RETURNTO address picked out of message
#else
    #error NO ROLE DEFINED
#endif

#ifdef _USERMAKEFILE_LIB_RFM69_XC_TRANS // AQU=073 new
    #define CLIENT_ALLOW_SESSION_TYPE_TRANS _USERMAKEFILE_LIB_RFM69_XC_TRANS

    #ifdef _USERMAKEFILE_LIB_RFM69_XC_TRANS_ASYNCH_WRAPPED
        #define TRANS_ASYNCH_WRAPPED _USERMAKEFILE_LIB_RFM69_XC_TRANS_ASYNCH_WRAPPED
    #else
        #define TRANS_ASYNCH_WRAPPED 0
    #endif
#else
    #define CLIENT_ALLOW_SESSION_TYPE_TRANS 0
    #define TRANS_ASYNCH_WRAPPED 0
#endif

#ifdef _USERMAKEFILE_LIB_RFM69_XC_GET_RADIO_DEBUG_REGS
    #define GET_RADIO_DEBUG_REGS _USERMAKEFILE_LIB_RFM69_XC_GET_RADIO_DEBUG_REGS
#else
    #define GET_RADIO_DEBUG_REGS 0
#endif

#ifdef _USERMAKEFILE_LIB_RFM69_XC_DEBUG_SHARED_LOG_VALUE
    #define DEBUG_SHARED_LOG_VALUE _USERMAKEFILE_LIB_RFM69_XC_DEBUG_SHARED_LOG_VALUE
#else
    #define DEBUG_SHARED_LOG_VALUE 0
#endif

#define DEBUG_PRINT_GLOBAL_APP 0 // 0: all printf off
                                 // 1: controlled locally in each xc file

#endif /* GLOBALS_H_ */
