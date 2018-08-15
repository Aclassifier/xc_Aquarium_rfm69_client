/*
 * _globals.h
 *
 *  Created on: 15. aug. 2018
 *      Author: teig
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_

#ifdef GLOBALS_H_ // To show that the below may also be defined in library space

    // See http://www.teigfam.net/oyvind/home/technology/109-know-your-timers-type/#monotonic_timer
    // ABOUT SIGNED OR NOT SIGNED:
    typedef signed int time32_t; // signed int (=signed) or unsigned int (=unsigned) both ok, as long as they are monotoneously increasing
                                 // XC/XMOS 100 MHz increment every 10 ns for max 2exp32 = 4294967296, ie. 42.9.. seconds

    // See http://www.teigfam.net/oyvind/home/technology/109-know-your-timers-type/#the_after_macro_8211_platform-independent8217ish
    // ABOUT THE AFTER_32 macro:
    #define AFTER_32(a,b) ((a-b)>0)
    #define NOW_32(tmr,time) do {tmr :> time;} while(0) // A tick is 10ns (100 MHz timer in XC). 32 bits wide
    // 2exp32 / (100 mill) = 42.94967296 second
    //
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

typedef struct { // Size must be modulo 4
    uint8_t payload_1;
    uint8_t payload_2;
    uint8_t payload_3;
    uint8_t payload_4;
} payload_u0_t;

typedef struct {  // Size must be modulo 4
    union {
        payload_u0_t payload_u0;
        uint8_t      payload_u1_uint8_arr[_USERMAKEFILE_LIB_RFM69_XC_PAYLOAD_LEN08]; // Size must be modulo 4
    } u;
} payload_t;

#endif /* GLOBALS_H_ */
