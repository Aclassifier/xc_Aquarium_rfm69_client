/*
 * I2C_internal_task.h
 *
 *  Created on: 27. feb. 2015
 *      Author: Teig
 */

#ifndef I2C_INTERNAL_SERVER_H_
#define I2C_INTERNAL_SERVER_H_

typedef enum i2c_dev_address_internal_t {
                                     // NO SOLDERING NEEDED ON ANY OF THESE BOARDS
    I2C_ADDRESS_OF_DISPLAY   = 0x3C, // UG-2832HSWEG02 with chip SSD1306 from Univision Technology Inc.
                                     // ALL BELOW AS USED IN AQUARIUM BOX:
    I2C_ADDRESS_OF_FRAM      = 0x50, //     Fujitsu MB85RC256V (MB85RC)
    I2C_ADDRESS_OF_FRAM_F8   = 0xF8, //     Fujitsu MB85RC256V Device ID first address
    I2C_ADDRESS_OF_FRAM_F9   = 0xF9, //     Fujitsu MB85RC256V Device ID second address
    I2C_ADDRESS_OF_CHRONODOT = 0x68  //     DS3231 Extremely Accurate I2C-Integrated RTC/TCXO/Crystal by Maxim
} i2c_dev_address_internal_t; // i2c_dev_address_t                          2

typedef interface i2c_internal_commands_if {

    bool write_display_ok (const i2c_dev_address_t dev_addr, const i2c_reg_address_t reg_addr, unsigned char data[], unsigned nbytes);

} i2c_internal_commands_if;

#define I2C_INTERNAL_NUM_CLIENTS 1

[[combinable]]
void I2C_Internal_Task (server i2c_internal_commands_if i_i2c_internal_commands[I2C_INTERNAL_NUM_CLIENTS]);

#else
    #error Nested include I2C_INTERNAL_SERVER_H_
#endif

