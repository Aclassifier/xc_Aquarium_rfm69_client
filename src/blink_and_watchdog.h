/*
 * beep_blink.h
 *
 *  Created on: 20. sep. 2018
 *      Author: teig
 */

#ifndef BLINK_AND_WATCHDOG_H_
#define BLINK_AND_WATCHDOG_H_

#define XCORE_200_EXPLORER_LED_GREEN_BIT_MASK     0x01
#define XCORE_200_EXPLORER_LED_RGB_BLUE_BIT_MASK  0x02
#define XCORE_200_EXPLORER_LED_RGB_GREEN_BIT_MASK 0x04
#define XCORE_200_EXPLORER_LED_RGB_RED_BIT_MASK   0x08
#define XCORE_200_EXPLORER_LEDS_ALL_BITS_MASK     0x0f // Looks blue'ish

typedef unsigned port_pins_mask_t; // Zero (no change), one or any number of bits. A high bit is light on

typedef interface beep_blink_if_t {

    // Each pin or group of pins may be set individually. The only protection that's done here is during blink_pulse,
    // even if it may be called with different pin-(groups). There is no guarantee of the visibility of any blink.
    // We could have guarded all calls, but this might cause unwanted blockings on the client side.
    // Remember that blocking may not even be possible since blocking per pin is not possible!
    // So I guess it's client usage that defines how this would look

    [[guarded]] bool blink_pulse_ok  ( // returns success if watchdog not timed out
                                      const port_pins_mask_t port_pins_mask, // [[guarded]] cost 344 bytes
                                      const unsigned         blink_ms);      // Assumed to be shorter than until next call
                bool blink_on_ok     ( // returns success if watchdog not timed out
                                      const port_pins_mask_t port_pins_mask);
                bool blink_off_ok    ( // returns success if watchdog not timed out
                                      const port_pins_mask_t port_pins_mask);

                // If none of the above not called within silent_for_ms then port_pins_mask will continuously blink off and on
                bool enable_watchdog_ok ( // returns success if called fir the first time (can only be called once)
                                         const port_pins_mask_t port_pins_mask, // May overlap pins above
                                         const unsigned         silent_for_ms,  // Max about 42 seconds
                                         const unsigned         blink_on_ms);   // off is same time

                bool is_watchdog_blinking (void);
} beep_blink_if_t;

#define BEEP_BLINK_TASK_NUM_CLIENTS 1 // Making it multi-client cost 120 bytes

// ------------------------------------------------------------------------------------------
// Observe placement! On the xCore-200 eXplorerKIT and printing to JTAG and XTAG-3 this task
// should run on another core than the client if the client does the printing! If not the
// timing with blink_pulse would be on as long as the log calls take (no scheduling!)
// ------------------------------------------------------------------------------------------

[[combinable]] // Cannot be [[distributable]] since timer case in select
void blink_and_watchdog_task (
        server beep_blink_if_t i_beep_blink [BEEP_BLINK_TASK_NUM_CLIENTS],
        out port               p_explorer_leds);

#endif /* BLINK_AND_WATCHDOG_H_ */
