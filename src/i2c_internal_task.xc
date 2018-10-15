/*
 * i2c_internal_task.xc
 *
 *  Created on: 27. feb.
 *      Author: Teig
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

#include "_globals.h"
#include "param.h"
#include "button_press.h"

#include "i2c.h"

#include "defines_adafruit.h"
#include "core_graphics_adafruit_GFX.h"

#include "I2C_Internal_Task.h"
#include "display_ssd1306.h"
#endif

#define DEBUG_PRINT_DISPLAY 0 // Cost 0.3k
#define debug_print(fmt, ...) do { if(DEBUG_PRINT_DISPLAY and (DEBUG_PRINT_GLOBAL_APP==1)) printf(fmt, __VA_ARGS__); } while (0)

#ifdef CODETHIS

r_i2c i2c_internal_config = { // For display and ChronoDot
    on tile[0]:XS1_PORT_1E, // I_SCL SCL is at startKIT GPIO header (J7.4) port P1E0, processor pin X0D12
    on tile[0]:XS1_PORT_1F, // I_SDA SDA is at startKIT GPIO header (J7.1) port P1F0, processor pin X0D13
    300                     // clockTics, smaller is faster
                            // Length of I2C clock period in reference clock ticks(10ns)
                            // 1000 * 10ns = 10.000 ns = 10 us -> 100.00 kbit/s operation
                            //  300 * 10ns =  3.000 ns =  3 us -> 333.33 kbit/s operation
};

// Internal i2c matters (not display matters)
[[combinable]]
void I2C_Internal_Task (server i2c_internal_commands_if i_i2c_internal_commands[I2C_INTERNAL_NUM_CLIENTS]) {

    #ifdef DEBUG_PRINT_DISPLAY
        unsigned long int num_chars = 0;
    #endif

    i2c_master_init (i2c_internal_config); // XMOS library

    // PRINT
    debug_print("%s", "I2C_Internal_Task started\n");

    while (1) {
        select {

            case i_i2c_internal_commands[int index_of_client].write_display_ok (const i2c_dev_address_t dev_addr, const i2c_reg_address_t reg_addr, unsigned char data[], unsigned nbytes) -> bool ok: {

                i2c_result_t i2c_result;

                if (nbytes <= SSD1306_WRITE_CHUNK_SIZE) {
                    unsigned      write_nbytes = nbytes;
                    unsigned char write_data[SSD1306_WRITE_CHUNK_SIZE];

                    debug_print ("i2c-i dev:%02x reg:%02x len:%d:", (int)dev_addr, reg_addr, (int)write_nbytes);

                    for (uint8_t x=0; x<write_nbytes; x++) {
                        write_data[x] = data[x];

                        #ifdef DEBUG_PRINT_DISPLAY // Keep it
                            if (x==(write_nbytes-1)) {
                                debug_print("%02x",data[x]); // Last, no comma
                            }
                            else {
                                debug_print("%02x ",data[x]);
                            }
                        #endif
                    }
                    i2c_result = i2c_master_write_reg ((int)dev_addr, reg_addr, write_data, (int)write_nbytes, i2c_internal_config);

                    #ifdef DEBUG_PRINT_DISPLAY // Keep it
                        num_chars += write_nbytes;
                        debug_print(" #%u\n", num_chars);
                    #endif

                } else {
                    i2c_result = I2C_PARAM_ERR; // qwe handle later or just do crash or truncate and let i be visible in the dislay
                }
                ok = (i2c_result == I2C_OK); // 1 = (1==1), all OK when 1
            } break;
        }
    }
}
#endif
