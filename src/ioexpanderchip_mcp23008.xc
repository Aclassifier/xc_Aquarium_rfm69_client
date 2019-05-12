/*
 * ioexpanderchip_mcp23008.xc
 *
 *  Created on: 12. mai 2019
 *      Author: teig
 */

#define INCLUDES
#ifdef INCLUDES
#include <platform.h>
#include <xs1.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <xccompat.h> // REFERENCE_PARAM
#include <iso646.h>
#include <string.h>   // memset.
#include <timer.h>    // For delay_milliseconds (but it compiles without?)

#include "_version.h" // First this..
#include "_globals.h" // ..then this
#include "param.h"

#include "i2c.h"

#include "defines_adafruit.h"
#include "i2c_internal_task.h"
#include "ioexpanderchip_mcp23008.h"

#endif

void internal_i2c_mcp23008_init (
          client   i2c_internal_commands_if i_i2c_internal_commands,
          unsigned                          &mcp23008_err_cnt)
{
    const uint8_t iodir = MCP23008_IODIR_ALL_PINS_DIR_OUTPUT bitor MY_MPC23008_IN_BUTTON_PRESS_WHENLOW_MASK;
    // bitor above since MY_MPC23008_IN_BUTTON_PRESS_WHENLOW_MASK has bit high as MCP23008_PIN_DIR_INPUT
    const unsigned char reg_data [LEN_I2C_REG+1] = {MCP23008_IODIR, iodir};

    mcp23008_err_cnt = 0; // Counting from init

    if (not i_i2c_internal_commands.write_ok (I2C_ADDRESS_OF_PORT_EXPANDER, reg_data, sizeof reg_data)) {
        mcp23008_err_cnt++;
    } else {
        const unsigned char reg_data [LEN_I2C_REG+1] = {MCP23008_GPIO, MY_MCP23008_ALL_OFF};

        if (not i_i2c_internal_commands.write_ok (I2C_ADDRESS_OF_PORT_EXPANDER, reg_data, sizeof reg_data)) {
            mcp23008_err_cnt++;
        }
    }
}

bool // relay_button_pressed
internal_i2c_mcp23008_poll_button (
          client   i2c_internal_commands_if i_i2c_internal_commands,
          unsigned                          &mcp23008_err_cnt,
          const bool                        relay_button_pressed_prev,
          relay_button_ustate_t             &relay_button_ustate)
{
    uint8_t the_register;
    bool    relay_button_pressed;

    if  (not i_i2c_internal_commands.read_reg_ok (I2C_ADDRESS_OF_PORT_EXPANDER, MCP23008_GPIO, the_register)) {
        mcp23008_err_cnt++;
        relay_button_pressed = false;
    } else {
        relay_button_pressed = ((the_register bitand MY_MPC23008_IN_BUTTON_PRESS_WHENLOW_MASK) == 0);
        if ((relay_button_pressed != relay_button_pressed_prev) and relay_button_pressed) { // Next state
            relay_button_ustate.u.cnt++;
            if (relay_button_ustate.u.state == RELAYBUTT_ROOF) {
                relay_button_ustate.u.state = RELAYBUTT_0;
            } else {}
        } else {}

    }
    return relay_button_pressed;
}


