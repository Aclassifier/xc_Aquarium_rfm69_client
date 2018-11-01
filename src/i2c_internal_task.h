/*
 * i2c_internal_task.h
 *
 *  Created on: 27. feb. 2015
 *      Author: Teig
 */

#ifndef I2C_INTERNAL_SERVER_H_
#define I2C_INTERNAL_SERVER_H_

#if (IS_MYTARGET==IS_MYTARGET_XCORE_200_EXPLORER)

    typedef enum i2c_dev_address_internal_t {
        I2C_ADDRESS_OF_ACCELEROMETER_AND_MAGNETOMETER = 0x1E, // FXOS8700CQ BMG160 3-axis gyroscope sensor
                                                              // NOT modifiable plus hardwired on XMOS XCORE-200 EXPLORERKIT
        I2C_ADDRESS_OF_DISPLAY                        = 0x3C, // UG-2832HSWEG02 with chip SSD1306 from Univision Technology Inc.
        I2C_ADDRESS_OF_GYROSCOPE_BMG160               = 0x68  // BMG160  FXOS8700CQ Digital Sensor - 3D Accelerometer (±2g/±4g/±8g) + 3D Magnetometer
                                                              // NOT modifiable plus hardwired on XMOS XCORE-200 EXPLORERKIT
                                                              // Observe that CHRONODOT has same address 0x68, also hard wired! So cannot coexist on same I2C bus
    } i2c_dev_address_internal_t; // i2c_dev_address_t

#elif (IS_MYTARGET==IS_MYTARGET_STARTKIT)
    typedef enum i2c_dev_address_internal_t {
                                         // NO SOLDERING NEEDED ON ANY OF THESE BOARDS
        I2C_ADDRESS_OF_DISPLAY   = 0x3C, // UG-2832HSWEG02 with chip SSD1306 from Univision Technology Inc.
                                         // ALL BELOW AS USED IN AQUARIUM BOX:
        I2C_ADDRESS_OF_FRAM      = 0x50, //     Fujitsu MB85RC256V (MB85RC)
        I2C_ADDRESS_OF_FRAM_F8   = 0xF8, //     Fujitsu MB85RC256V Device ID first address
        I2C_ADDRESS_OF_FRAM_F9   = 0xF9, //     Fujitsu MB85RC256V Device ID second address
        I2C_ADDRESS_OF_CHRONODOT = 0x68  //     DS3231 Extremely Accurate I2C-Integrated RTC/TCXO/Crystal by Maxim. NOT modifiable!
    } i2c_dev_address_internal_t; // i2c_dev_address_t
#else
    #error TARGET NOT DEFINED
#endif

typedef interface i2c_internal_commands_if {

    bool write_display_ok (
            const i2c_dev_address_t dev_addr,
            const i2c_reg_address_t reg_addr,
            const unsigned char     data[],
            const unsigned          nbytes);

} i2c_internal_commands_if;

#define I2C_INTERNAL_NUM_CLIENTS 1

[[combinable]]
void I2C_Internal_Task (
        server i2c_internal_commands_if i_i2c_internal_commands[I2C_INTERNAL_NUM_CLIENTS],
        client  i2c_master_if           i_i2c); // synchronous

#else
    #error Nested include I2C_INTERNAL_SERVER_H_
#endif

