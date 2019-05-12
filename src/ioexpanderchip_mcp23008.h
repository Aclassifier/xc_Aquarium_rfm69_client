/*
 * ioexpanderchip_mcp23008.h
 *
 *  Created on: 3. mai 2019
 *      Author: teig
 */

#ifndef IOEXPANDERCHIP_MCP23008_H_
#define IOEXPANDERCHIP_MCP23008_H_

    typedef enum i2c_reg_addr_port_expander_mcp23008_t {
        MCP23008_IODIR     = 0x00,
        MCP23008_IPOL      = 0x01,
        MCP23008_GPINTEN   = 0x02,
        MCP23008_DEFVAL    = 0x03,
        MCP23008_INTCON    = 0x04,
        MCP23008_IOCON     = 0x05,
        MCP23008_GPPU      = 0x06,
        MCP23008_INTF      = 0x07,
        MCP23008_INTCAP_RO = 0x08, // Read only
        MCP23008_GPIO      = 0x09,
        MCP23008_OLAT      = 0x0A
    } i2c_reg_addr_port_expander_mcp23008_e;

    // 1.6.1 I/O DIRECTION (IODIR) REGISTER
    // Controls the direction of the data I/O.
    // When a bit is set, the corresponding pin becomes an input. When a bit is clear, the corresponding pin becomes an output.
    typedef enum {MCP23008_PIN_DIR_OUTPUT, MCP23008_PIN_DIR_INPUT} mcp23008_direction_e;
    #define MCP23008_IODIR_ALL_PINS_DIR_OUTPUT 0x00
    //
    typedef enum {MCP23008_PIN_INPUT_FLOATING, MCP23008_PIN_INPUT_PULLUP} mcp23008_pullup_e;
    typedef enum {MCP23008_PIN_LOW,            MCP23008_PIN_HIGH}         mcp23008_value_e;

#endif /* IOEXPANDERCHIP_MCP23008_H_ */
