/*
 * _globals.h
 *
 *  Created on: 15. aug. 2018
 *      Author: teig
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_

#ifdef GLOBALS_H_ // To show that the below may also be defined in library space

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

    #ifdef __XC__
        // BOOLEAN #include <stdbool.h> if C99
        // See http://www.teigfam.net/oyvind/home/technology/165-xc-code-examples/#bool
        typedef enum {false,true} bool; // 0,1 This typedef matches any integer-type type like long, int, unsigned, char, bool
    #endif

#endif

#define DEBUG_PRINT_GLOBAL_APP 1 // 0: all printf off
                                 // 1: controlled locally in each xc file

#endif /* GLOBALS_H_ */
