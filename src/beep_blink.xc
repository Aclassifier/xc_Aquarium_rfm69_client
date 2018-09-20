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

#include "_version.h"
#include "_globals.h"
#include "beep_blink.h"

[[combinable]] // Cannot be [[distributable]] since timer case in select
void beep_blink_task (
        server beep_blink_if_t i_beep_blink [BEEP_BLINK_TASK_NUM_CLIENTS],
        out port               p_port)
{

    timer            tmr;
    time32_t         blink_timeout_ticks;
    bool             do_timeout = false; // This also guards blink_pulse calls while the past call is not finished
    unsigned         port_pins = 0;
    port_pins_mask_t port_pins_mask_blink_end;

    p_port <: port_pins;

    while (1) {
        select {
            case (do_timeout) => tmr when timerafter (blink_timeout_ticks) :> void: {

                port_pins and_eq (compl port_pins_mask_blink_end);
                p_port <: port_pins;

                do_timeout = false;

            } break;

            case (do_timeout == false) => i_beep_blink[int index_of_client].blink_pulse (
                    const port_pins_mask_t port_pins_mask,
                    const time32_t         timeout_ticks) : {

                port_pins or_eq port_pins_mask;
                p_port <: port_pins;

                port_pins_mask_blink_end = port_pins_mask;

                tmr :> blink_timeout_ticks;
                blink_timeout_ticks += timeout_ticks;
                do_timeout = true;

            } break;

            case i_beep_blink[int index_of_client].blink_on (const port_pins_mask_t port_pins_mask) : {

                port_pins or_eq port_pins_mask;
                p_port <: port_pins;

            } break;

            case i_beep_blink[int index_of_client].blink_off (const port_pins_mask_t port_pins_mask) : {

                port_pins and_eq (compl port_pins_mask);
                p_port <: port_pins;

            } break;
        }
    }
}
