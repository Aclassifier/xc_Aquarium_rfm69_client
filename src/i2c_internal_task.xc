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
#include "i2c_internal_task.h"
#include "display_ssd1306.h"
#include "core_graphics_adafruit_gfx.h"
#include "_texts_and_constants.h"

#endif

#define DEBUG_PRINT_DISPLAY 0
#define debug_print(fmt, ...) do { if(DEBUG_PRINT_DISPLAY and (DEBUG_PRINT_GLOBAL_APP==1)) printf(fmt, __VA_ARGS__); } while (0)

// Internal i2c matters (not display matters)
[[combinable]]
void I2C_Internal_Task (
        server i2c_internal_commands_if i_i2c_internal_commands[I2C_INTERNAL_NUM_CLIENTS],
        client  i2c_master_if           i_i2c) { // synchronous

    #ifdef DEBUG_PRINT_DISPLAY
        unsigned long int num_chars = 0;
    #endif

    // PRINT
    debug_print("%s", "I2C_Internal_Task started\n");

    while (1) {
        select {

            case i_i2c_internal_commands[int index_of_client].write_display_ok (
                    const i2c_dev_address_t dev_addr,
                    const i2c_reg_address_t reg_addr,
                    const unsigned char     data[], // SSD1306_WRITE_CHUNK_SIZE always is n:
                    const unsigned          nbytes) -> bool ok: {

                #define REG_SIZE 1 // register address is one bytes. It's sent after dev_addr
                #define SSD1306_WRITE_ARR_SIZE (REG_SIZE + SSD1306_WRITE_CHUNK_SIZE)

                unsigned char write_data[SSD1306_WRITE_ARR_SIZE];

                write_data[0] = reg_addr;

                i2c_result_t i2c_result;

                if (nbytes <= SSD1306_WRITE_CHUNK_SIZE) {
                    unsigned  write_nbytes = nbytes + REG_SIZE; // Now including reg_addr as first byte

                    debug_print ("i2c-i dev:%02x reg:%02x r-len:%d:", (int)dev_addr, reg_addr, (int)write_nbytes);

                    for (uint8_t x=REG_SIZE; x<write_nbytes; x++) {
                        write_data[x] = data[x-REG_SIZE];

                        #ifdef DEBUG_PRINT_DISPLAY // Keep it
                            if (x==(write_nbytes-1)) {
                                debug_print("%02x",write_data[x]); // Last, no comma
                            }
                            else {
                                debug_print("%02x ",write_data[x]);
                            }
                        #endif
                    }
                    // lib_i2c:
                    size_t    num_bytes_sent;
                    int       send_stop_bit = 1;
                    i2c_res_t i2c_res;

                    i2c_res= i_i2c.write ((uint8_t)dev_addr, write_data, (size_t) write_nbytes, num_bytes_sent, send_stop_bit);

                    if ((i2c_res == I2C_NACK) or (num_bytes_sent != write_nbytes)) {
                        i2c_result = I2C_ERR;
                    } else {
                        // ==I2C_ACK
                        i2c_result = I2C_OK;
                    }

                    // module_i2c_master:
                    // i2c_result = i2c_master_write_reg ((int)dev_addr, reg_addr, write_data, (int)write_nbytes, i2c_internal_config);
                    /* The code in module_i2c_master:
                    int i2c_master_write_reg(int device, int addr, unsigned char s_data[], int nbytes, struct r_i2c &i2c) {
                        startBit(i2c, 1);
                        if (!tx8(i2c, device<<1)) return floatWires(i2c);
                        if (!tx8(i2c, addr)) return floatWires(i2c);
                        for(int j = 0; j < nbytes; j++) {
                           i f (!tx8(i2c, s_data[j])) return floatWires(i2c);
                        }
                        stopBit(i2c);
                        return 1;
                    } */

                    #ifdef DEBUG_PRINT_DISPLAY // Keep it
                        debug_print(" r-sent %d\n", num_bytes_sent); // Including reg_addr
                        num_chars += write_nbytes;
                        debug_print(" #%u\n", num_chars); // For a typical display at least 3KB are written
                    #endif

                } else {
                    i2c_result = I2C_PARAM_ERR; // qwe handle later or just do crash or truncate and let i be visible in the dislay
                }
                ok = (i2c_result == I2C_OK); // 1 = (1==1), all OK when 1
            } break;
        }
    }
}
