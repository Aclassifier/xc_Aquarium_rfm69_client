/*
 * beep_blink.xc
 *
 *  Created on: 20. sep. 2018
 *      Author: teig
 */

#include <xs1.h>
#include <platform.h> // slice
#include <timer.h>    // delay_milliseconds(200), XS1_TIMER_HZ etc
#include <stdint.h>   // uint8_t
#include <stdio.h>    // printf
#include <iso646.h>   // not etc.

#define DO_ASSERTS
#include <limits.h>   // for ASSERT_DELAY_32 if DO_ASSERTS
#include <xassert.h>  // for ASSERT_DELAY_32 if DO_ASSERTS

#include "_version.h"
#include "_globals.h"
#include "blink_and_watchdog.h"

#define DEBUG_PRINT_BLINK_AND_WATCHDOG_RFM69 0 // DON'T SET 1 HERE IF TESTING HOW printf AND XTAG-3 BLOCKS THE CALL!
                                               // See http://www.teigfam.net/oyvind/home/technology/098-my-xmos-notes/#xtag-3_debug_log_hanging
                                               // Use is_watchdog_blinking instead!
//
#define debug_print(fmt, ...) do { if(DEBUG_PRINT_BLINK_AND_WATCHDOG_RFM69 and (DEBUG_PRINT_GLOBAL_APP==1)) printf(fmt, __VA_ARGS__); } while (0)

#define BLINK_RESOLUTION_MS 10

[[combinable]] // Cannot be [[distributable]] since timer case in select
void blink_and_watchdog_task (
        server blink_and_watchdog_if_t i_beep_blink [BEEP_BLINK_TASK_NUM_CLIENTS],
        out port               p_port)
{
    unsigned         port_pins = 0;


    timer            tmr;
    time32_t         blink_resolution_timeout_tics;

    time32_t         single_pulse_timeout_tics;
    bool             single_pulse_off_next = false;
    port_pins_mask_t single_pulse_restore_port_pins_mask;

    bool             do_watchdog_feed_next = false;
    time32_t         watchdog_feed_timeout_tics;

    bool             does_watchdog_blinking = false;

    time32_t         watchdog_blink_timeout_tics;
    port_pins_mask_t watchdog_port_pins_mask;
    bool             is_watchdog_port_pins_on;


    bool             enabled_watchdog = false;
    unsigned         watchdog_blink_on_ms;
    unsigned         watchdog_silent_for_ms;

    p_port <: port_pins;

    tmr :> blink_resolution_timeout_tics;
    blink_resolution_timeout_tics += (BLINK_RESOLUTION_MS * XS1_TIMER_KHZ);

    while (1) {
        select {
            case tmr when timerafter (blink_resolution_timeout_tics) :> void: {

                blink_resolution_timeout_tics += (BLINK_RESOLUTION_MS * XS1_TIMER_KHZ);

                if (single_pulse_off_next) {
                    if (AFTER_32 (blink_resolution_timeout_tics, single_pulse_timeout_tics)) {
                        port_pins and_eq (compl single_pulse_restore_port_pins_mask);
                        p_port <: port_pins;

                        single_pulse_off_next = false;
                    } else {}
                } else {}

                if (do_watchdog_feed_next) {
                    watchdog_feed_timeout_tics = blink_resolution_timeout_tics + (watchdog_silent_for_ms * XS1_TIMER_KHZ);
                    do_watchdog_feed_next = false;
                } else if (not enabled_watchdog) {
                    // Do nothing, no code
                } else if (does_watchdog_blinking) {
                    // Do nothing, no code
                } else if (AFTER_32 (blink_resolution_timeout_tics, watchdog_feed_timeout_tics)) {
                    does_watchdog_blinking = true; // Once on, never off

                    port_pins or_eq watchdog_port_pins_mask;
                    p_port <: port_pins;
                    is_watchdog_port_pins_on = true;

                    watchdog_blink_timeout_tics = blink_resolution_timeout_tics + (watchdog_blink_on_ms * XS1_TIMER_KHZ);
                    debug_print ("%s\n", "WATCHDOG STARTING");
                } else {}

                if (does_watchdog_blinking) { // Continuously on when first on
                    if (AFTER_32 (blink_resolution_timeout_tics, watchdog_blink_timeout_tics)) {
                        if (is_watchdog_port_pins_on) {
                            port_pins and_eq (compl watchdog_port_pins_mask); // was on now off
                        } else {
                            port_pins or_eq watchdog_port_pins_mask; // was off now on
                        }
                        is_watchdog_port_pins_on = not is_watchdog_port_pins_on;
                        p_port <: port_pins;
                        watchdog_blink_timeout_tics = blink_resolution_timeout_tics + (watchdog_blink_on_ms * XS1_TIMER_KHZ);
                    } else {}
                } else {}
            } break;

            case (single_pulse_off_next == false) => i_beep_blink[int index_of_client].blink_pulse_ok (
                    const port_pins_mask_t port_pins_mask,
                    const unsigned         blink_on_ms) // Max about 21 seconds
                    -> bool success : {

                ASSERT_DELAY_32 (blink_on_ms * XS1_TIMER_KHZ);

                if (does_watchdog_blinking) {
                    success = false; // Watchdog may have used same pins!
                } else {
                    port_pins or_eq port_pins_mask;
                    p_port <: port_pins;

                    single_pulse_restore_port_pins_mask = port_pins_mask;

                    tmr :> single_pulse_timeout_tics;
                    single_pulse_timeout_tics += (blink_on_ms * XS1_TIMER_KHZ);
                    single_pulse_off_next = true;
                    do_watchdog_feed_next = true;
                    success = true;
                }
                debug_print ("blink_pulse_ok success %u\n", success);
            } break;

            case i_beep_blink[int index_of_client].blink_on_ok (const port_pins_mask_t port_pins_mask) -> bool success : {

                if (does_watchdog_blinking) {
                    success = false; // Watchdog may have used same pins!
                } else {
                    port_pins or_eq port_pins_mask;
                    p_port <: port_pins;
                    do_watchdog_feed_next = true;
                    success = true;
                }
                debug_print ("blink_on_ok success %u\n", success);
            } break;

            case i_beep_blink[int index_of_client].blink_off_ok (const port_pins_mask_t port_pins_mask) -> bool success : {

                if (does_watchdog_blinking) {
                    success = false; // Watchdog may have used same pins!
                } else {
                    port_pins and_eq (compl port_pins_mask);
                    p_port <: port_pins;

                    do_watchdog_feed_next = true;
                    success = true;
                }
                debug_print ("blink_off_ok success %u\n", success);
            } break;

            case i_beep_blink[int index_of_client].enable_watchdog_ok (
                    const port_pins_mask_t port_pins_mask, // May overlap other pins
                    const unsigned         silent_for_ms,  // Max about 21 seconds
                    const unsigned         blink_on_ms)    // Max about 21 seconds
                    -> bool success: { // off is same time


                ASSERT_DELAY_32 (silent_for_ms * XS1_TIMER_KHZ);
                ASSERT_DELAY_32 (blink_on_ms * XS1_TIMER_KHZ);

                if (enabled_watchdog) {
                    success = false; // Already called, don't set up the params another time
                } else {
                    watchdog_silent_for_ms = silent_for_ms;
                    watchdog_port_pins_mask = port_pins_mask;

                    tmr :> watchdog_feed_timeout_tics;
                    watchdog_feed_timeout_tics += (watchdog_silent_for_ms * XS1_TIMER_KHZ);

                    watchdog_blink_on_ms = blink_on_ms;
                    enabled_watchdog = true;
                    success = true;
                }
                debug_print ("enable_watchdog_ok success %u\n", success);
            } break;

            case i_beep_blink[int index_of_client].is_watchdog_blinking () -> bool yes: {
                yes = does_watchdog_blinking;
            } break;
        }
    }
}
