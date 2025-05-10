/*
 * iochip_mcp23008.h
 *
 *  Created on: 3. mai 2019
 *      Author: teig
 */

#ifndef IOEXPANDERCHIP_MCP23008_H_
#define IOEXPANDERCHIP_MCP23008_H_

    typedef enum i2c_reg_addr_port_expander_mcp23008_e {
        MCP23008_IODIR     =  0,
        MCP23008_IPOL      =  1,
        MCP23008_GPINTEN   =  2,
        MCP23008_DEFVAL    =  3,
        MCP23008_INTCON    =  4,
        MCP23008_IOCON     =  5,
        MCP23008_GPPU      =  6,
        MCP23008_INTF      =  7,
        MCP23008_INTCAP_RO =  8, // Read only
        MCP23008_GPIO      =  9,
        MCP23008_OLAT      = 10
    } i2c_reg_addr_port_expander_mcp23008_e;

    // 1.6.1 I/O DIRECTION (IODIR) REGISTER
    // Controls the direction of the data I/O.
    // When a bit is set, the corresponding pin becomes an input. When a bit is clear, the corresponding pin becomes an output.
    typedef enum {MCP23008_PIN_DIR_OUTPUT, MCP23008_PIN_DIR_INPUT} mcp23008_direction_e;
    #define MCP23008_IODIR_ALL_PINS_DIR_OUTPUT 0x00
    //
    typedef enum {MCP23008_PIN_INPUT_FLOATING, MCP23008_PIN_INPUT_PULLUP} mcp23008_pullup_e;
    typedef enum {MCP23008_PIN_LOW,            MCP23008_PIN_HIGH}         mcp23008_value_e;


    //                                                                #
    #define MY_MCP23008_OUT_RELAY2_ON_BIT               7 // ON=1 OFF=0
    //                                                  6 //    .     . not used
    #define MY_MCP23008_OUT_RELAY1_ON_BIT               5 // ON=1 OFF=0
    //                                                  4 //    .     . not used
    #define MY_MCP23008_OUT_WATCHDOG_LOWTOHIGH_EDGE_BIT 3 //    .     . low-to-high on pin resets watchdog
    #define MY_MPC23008_IN_BUTTON_PRESS_WHENLOW_BIT     2 //    .     . input, low when buttton pressed
    #define MY_MCP23008_OUT_RED_LED_OFF_BIT             1 // ON=0 OFF=1
    #define MY_MCP23008_OUT_GREEN_LED_OFF_BIT           0 // ON=0 OFF=1
    //                                                                #
    #define MY_MCP23008_ALL_OFF                      0x03 //          # 0.0...11 as 00000011

    #define MY_MCP23008_OUT_RELAY2_ON_MASK               (1<<MY_MCP23008_OUT_RELAY2_ON_BIT)
    #define MY_MCP23008_OUT_RELAY1_ON_MASK               (1<<MY_MCP23008_OUT_RELAY1_ON_BIT)
    #define MY_MCP23008_OUT_WATCHDOG_LOWTOHIGH_EDGE_MASK (1<<MY_MCP23008_OUT_WATCHDOG_LOWTOHIGH_EDGE_BIT)
    #define MY_MPC23008_IN_BUTTON_PRESS_WHENLOW_MASK     (1<<MY_MPC23008_IN_BUTTON_PRESS_WHENLOW_BIT) // Bit high as MCP23008_PIN_DIR_INPUT
    #define MY_MCP23008_OUT_RED_LED_OFF_MASK             (1<<MY_MCP23008_OUT_RED_LED_OFF_BIT)
    #define MY_MCP23008_OUT_GREEN_LED_OFF_MASK           (1<<MY_MCP23008_OUT_GREEN_LED_OFF_BIT)

    typedef enum {
        RELAYBUTT_0,
        RELAYBUTT_1,
        RELAYBUTT_2,
        RELAYBUTT_3,
        RELAYBUTT_4,
        RELAYBUTT_ROOF // Not used
    } relay_button_state_e;

    typedef struct relay_button_ustate_t {
        union {
            relay_button_state_e state;
            unsigned             cnt;
        } u;
    } relay_button_ustate_t;

    typedef interface i2c_general_commands_if {

        bool write_reg_ok (
                const i2c_dev_address_t dev_addr,
                const unsigned char     reg_data[],    // reg_addr followed by data
                const static unsigned   len_reg_data); // must include space for LEN_I2C_REG

        bool read_reg_ok (
                const i2c_dev_address_t dev_addr,
                const unsigned char     reg_addr,
                      uint8_t           &the_register);

    } i2c_general_commands_if;

    void i2c_general_mcp23008_init (
              client   i2c_general_commands_if i_i2c_general_commands,
              unsigned                         &iochip_err_cnt);

    bool // relay_button_pressed
    i2c_general_mcp23008_poll_button (
              client   i2c_general_commands_if i_i2c_general_commands,
              unsigned                         &iochip_err_cnt,
              const bool                       relay_button_pressed_prev,
              relay_button_ustate_t            &relay_button_ustate);

#endif /* IOEXPANDERCHIP_MCP23008_H_ */
