/*
 * main.xc
 *
 *  Created on: 15. aug. 2018
 *      Author: teig
 */

#define INCLUDES
#ifdef INCLUDES
#include <xs1.h>
#include <platform.h> // slice
#include <timer.h>    // delay_milliseconds(200), XS1_TIMER_HZ etc
#include <stdint.h>   // uint8_t
#include <stdio.h>    // printf
#include <string.h>   // memcpy
#include <xccompat.h> // REFERENCE_PARAM(my_app_ports_t, my_app_ports) -> my_app_ports_t &my_app_ports
#include <iso646.h>   // not etc.
#include <spi.h>
#include <xassert.h>
#include <i2c.h>

#include "_version.h" // First this..
#include "_globals.h" // ..then this
#include "blink_and_watchdog.h"

#include "param.h"
#include "defines_adafruit.h"
#include "i2c_internal_task.h"
#include "display_ssd1306.h"
#include "core_graphics_adafruit_gfx.h"
#include "_texts_and_constants.h"

#include <rfm69_globals.h>
#include <rfm69_crc.h>
#include <rfm69_commprot.h>
#include <rfm69_xc.h>
#include "button_press.h"

#include "_Aquarium_rfm69_client.h"
#endif

#define DEBUG_PRINT_RFM69 1
#define debug_print(fmt, ...) do { if((DEBUG_PRINT_RFM69==1) and (DEBUG_PRINT_GLOBAL_APP==1)) printf(fmt, __VA_ARGS__); } while (0)

#define SEMANTICS_DO_RSSI_IN_IRQ_DETECT_TASK 0 // # chanends ---MEM---  (relative values)
                                               // 1 :     2    700 bytes  Does it faster after IRQ line (good if much logging in RFM69_driver)
                                               //                         DOES NOT WORK WITH xTIMEcomposer 13.3.3, see XMOS ticket 31286
                                               // 0 :     0      0        RSSI may be measured too late if much logging in RFM69_driver

// http://www.xcore.com/viewtopic.php?f=26&t=6331
#define CAT3(a,b,c) a##b##c
#define XS1_PORT(WIDTH,LETTER) CAT3(XS1_PORT_,WIDTH,LETTER) // XS1_PORT_ is a string here, not some #define from the woods!

                                              //               StartKIT                  eXplorerKIT - BUT NOT AS PREDEFINED SPI in their Portmaps
                                              //                                         as WiFi sliceCARD My breakpot board
#define SPI_MOSI                XS1_PORT(1,K) // XS1_PORT_1K   X0D34 P1K        PCIe-B10 GPIO-PIN19                                (also on J12 servo connector)
#define SPI_CLK                 XS1_PORT(1,J) // XS1_PORT_1J   X0D25 P1J        PCIe-A8  GPIO-PIN21
#define SPI_MISO                XS1_PORT(1,I) // XS1_PORT_1I   X0D24 P1I        PCIe-B15 GPIO-PIN23
#define SPI_CS_EN               XS1_PORT(4,C) // XS1_PORT_4C   X0D14 P4C0       PCIe-B6  GPIO-PIN47  MASKOF_SPI_SLAVE0_CS          CS/SS Chip select is port BIT0 low
                                              // XS1_PORT_4C   X0D15 P4C1       PCIe-B7  GPIO-PIN45  MASKOF_SPI_SLAVE0_EN          nPWR_EN SPI_EN Power enable is port BIT1 high
                                              // XS1_PORT_4C   X0D20 P4C2       PCIe-A6  GPIO-PIN43  MASKOF_SPI_SLAVE0_PROBE1_INNER
                                              // XS1_PORT_4C   X0D21 P4C3       PCIe-A7  GPIO-PIN42  MASKOF_SPI_SLAVE0_PROBE2_OUTER
#define SPI_AUX                 XS1_PORT(4,D) // XS1_PORT_4D   X0D16 P4D0       PCIe-B9  GPIO-PIN31  MASKOF_SPI_AUX0_RST           RST Restart is port BIT0
                                              // XS1_PORT_4D   X0D17 P4D1       PCIe-B11 GPIO-PIN29  MASKOF_SPI_AUX0_PROBE3_IRQ
#define SPI_IRQ                 XS1_PORT(1,L) // XS1_PORT_1L   X0D35 P1L        PCIe-A15 GPIO-PIN17  IRQ, "G0", "GPIO 0", DIO0     (also on J10 servo connector)
#define PROBE5                  XS1_PORT(1,D) // XS1_PORT_1D   X0D11 P1D  J3.21 LED-D2   SPI-MOSI    "PROBE1", "PROBE2" & "PROBE3" are in bitmasks
#define BUTTON_LEFT             XS1_PORT(1,N) // XS1_PORT_1N                             GPIO-PIN61 With pull-up of 9.1k           (also on J9 servo connector)
#define BUTTON_CENTER           XS1_PORT(1,O) // XS1_PORT_1O                             GPIO-PIN59 With pull-up of 9.1k
#define BUTTON_RIGHT            XS1_PORT(1,P) // XS1_PORT_1P                             GPIO-PIN57 With pull-up of 9.1k
#define XCORE_200_EXPLORER_LEDS XS1_PORT(4,F) // XS1_PORT_4F
#define I2C_SCL                 XS1_PORT(1,E) // XS1_PORT_1E                             GPIO-PIN39
#define I2C_SDA                 XS1_PORT(1,F) // XS1_PORT_1F                             GPIO-PIN37
#define DISPLAY_NRES            XS1_PORT(1,G) // XS1_PORT_1G                             GPIO-PIN35

// From spi_lib spi.pdf
//                32 bits over 1 bit:                                  New as above | As spi_master_interface in main.xc in _app_tiwisl_simple_webserver
in  buffered port:32 p_miso             = on tile[0]: SPI_MISO;     // New as above | Was XS1_PORT_1A, but that's for sliceKIT
out buffered port:32 p_sclk             = on tile[0]: SPI_CLK;      // New as above | Was XS1_PORT_1C, but that's for sliceKIT (Was 22 in spi.pdf but that must be a typo)
out buffered port:32 p_mosi             = on tile[0]: SPI_MOSI;     // New as above | Was XS1_PORT_1D, but that's for sliceKIT
//
clock                clk_spi            = on tile[0]: XS1_CLKBLK_1; // See USE_CLOCK_BLOCK
port                 p_scl              = on tile[0]: I2C_SCL;
port                 p_sda              = on tile[0]: I2C_SDA;
out port             p_display_notReset = on tile[0]: DISPLAY_NRES; // was outP_display_notReset
                                        // on adafruit monochrome 128x32 I2C OLED graphic display PRODUCT ID: 931, containing
                                        // module UG-2832HSWEG02 with chip SSD1306 from Univision Technology Inc. Data sheet often says 128 x 64 bits
                                        // as it looks like much of the logic is the same as for 128 z 32 bits.
                                        // At least 3 us low to reset

#if (SEMANTICS_DO_RSSI_IN_IRQ_DETECT_TASK==1)
    #define SPI_NUM_CLIENTS 2 // Number of users per board
#else
    #define SPI_NUM_CLIENTS 1 // Number of users per board
#endif

#define                     SPI_CLIENT_0    0 // BOTH HERE: Remember a call to i_spi.await_spi_port_init_by_all_clients(); before use of spi_master_if (by i_spi)
#define                     SPI_CLIENT_1    1 // AND HERE:  --"--
#define                     SPI_CLIENT_VOID 0 // Any value
#define NUM_SPI_CS_SETS SPI_NUM_CLIENTS  // See (*) below. Actually number of different SPI boards, but since both clients use the same chip, the sets are equal

// https://learn.adafruit.com/adafruit-rfm69hcw-and-rfm96-rfm95-rfm98-lora-packet-padio-breakouts/pinouts
//      SPI_CS_EN bits:
#define MASKOF_SPI_SLAVE0_CS          0x01  // Chip select SS/CS is port SPI_CS_EN BIT0 low (Called "NSS pin" in Semtech manual)
#define MASKOF_SPI_SLAVE0_EN          0x02  // Power enable EN   is port SPI_CS_EN BIT1 high
                                            // EN - connected to the enable pin of the regulator. Pulled high to Vin by default,
                                            //      pull low to completely cut power to the RX_some_rfm69_internals.
#define MASKOF_SPI_SLAVE0_PROBE1_INNER 0x04 // Inner scope pin on piggy-board
#define MASKOF_SPI_SLAVE0_PROBE2_OUTER 0x08 // Outer scope pin on piggy-board


#define MASKOF_SPI_AND_PROBE_PINS_INIT { MASKOF_SPI_SLAVE0_CS, MASKOF_SPI_SLAVE0_EN, MASKOF_SPI_SLAVE0_PROBE1_INNER, MASKOF_SPI_SLAVE0_PROBE2_OUTER }
//#define MASKOF_SPI_AND_PROBE_PINS_INIT {(0x01),(0x02),0x04,0x08} // error: parse error before numeric constant
//#define MASKOF_SPI_AND_PROBE_PINS_INIT {0x01,(0x02),0x04,0x08} // NOT: error: parse error before numeric constant TODO

maskof_spi_and_probe_pins_t maskof_spi_and_probe_pins [NUM_SPI_CS_SETS] = // (*)
{
    MASKOF_SPI_AND_PROBE_PINS_INIT
    #if (NUM_SPI_CS_SETS == 2)
        // (*) Observe that compiler will not warn if there's too little in init field, only too much
        //     This caused (Running)CDTDebugModelPresentation.12=signal that I wasn't able to trace, but I found it.
        //     It was solved with the array overflow check and "fail" call in spi_sync.xc in lib_spi
        , MASKOF_SPI_AND_PROBE_PINS_INIT
    #endif
};

#define USE_CLOCK_BLOCK 0
//
// From spi.pdf
//     The final parameter of the spi_master task is an optional clock block. If the clock block is supplied then the
//     maximum transfer rate of the SPI bus is increased (see Table 3). If null is supplied instead then the performance
//     is less but no clock block is used.
//
#if (USE_CLOCK_BLOCK==1)
    #define SPI_CLOCK clk_spi // As speed increases the 8 clock cycles take less and less time in the CS-window
#else
    #define SPI_CLOCK null // timers and chans same, but saved 1456 bytes! Observe speed limit, see above
                           // The 8 clock cycles always "fill in" the CS-window
#endif

// From spi.pdf
//    Connecting to the xCORE SPI master
//        If only one data direction is required then the MOSI or MISO line need not be connected.
//        However, asynchronous mode is only supported if the MISO line is connected.
//    Connecting to the xCORE SPI slave
//        If the MISO line is not required then it need not be connected.
//        The MOSI line must always be connected.

#ifdef COMMENT_BLOCK

// XMOS Programming Guide. Publication Date: 2014/9/18

// The pins on the device are accessed via the hardware-response ports.
// The port is responsible for driving output on the pins or sampling input data.
// Different ports have different widths: there are 1-bit, 4-bit, 8-bit, 16-bit and 32-bit ports.
// An n-bit port will simultaneously drive or sample n bits at once.
// In current devices, each tile has 29 ports.

// Port width  Number of ports  Port names
// 1           16               1A, 1B, 1C, 1D, 1E, 1F, 1G, 1H, 1I, 1J, 1K, 1L, 1M, 1N, 1O ,1P
// 4           6                4A, 4B, 4C, 4D, 4E ,4F
// 8           4                8A, 8B, 8C ,8D
// 16          2                16A, 16B
// 32          1                32A

// All ports are clocked - they are attached to a clock block in the device to control reading and writing
// from the port. A clock block provides a regular clock signal to the port.

// Each port has a register called the shift register within it that holds either data to output or data
// just input depending on whether the port is in input or output mode. At each clock tick, the port
// samples the external pins into the shift register or drives the external pins based on the contents
// of the shift register. When a program “inputs” or “outputs” to a port it is actually reads or writes
// the shift register.

// There are six clock blocks per tile. Any port can be connected to any of these six clock blocks.
// Each port can be set to one of two modes:
// - Divide             The clock runs at a rate which is an integer divide of the core clock rate of the
//                      chip (e.g. a divide of 500MHz for a 500MHz part).
// - Externally driven  The clock runs at a rate governed by an port input.

// By default, all ports are connected to clock block 0 which is designated the reference
// clock block and always runs at 100MHz.

// All ports must be declared as global variables, and no two ports may be initialized with the same port
// identifier. After initialization, a port may not be assigned to. Passing a port to a function is allowed
// as long as the port does not appear in more than one of a function’s arguments, which would create an
// illegal alias

#endif


#define DO_PLACED 1 // 1 is least code

// Observe that I have no control of the ports during xTIMEcomposer downloading
// I have observed a 700-800 ms low on signal pins before my code starts

out port p_spi_cs_en     = on tile[0]:SPI_CS_EN;
out port p_spi_aux       = on tile[0]:SPI_AUX;
in  port p_spi_irq       = on tile[0]:SPI_IRQ;
out port p_explorer_leds = on tile[0]:XCORE_200_EXPLORER_LEDS;

port inP_button_left   = on tile[0]: XS1_PORT_1N; // P1N0, X0D37 B_Left
port inP_button_center = on tile[0]: XS1_PORT_1O; // P1O0, X0D38 B_Center
port inP_button_right  = on tile[0]: XS1_PORT_1P; // P11P, X0D39 B_Right

#define I2C_MASTER_NUM_CLIENTS          1
#define I2C_MASTER_SPEED_KBPS           333 // 333 is same speed as used in the aquarium in i2c_internal_task.xc,
                                             // i2c_internal_config.clockTicks 300 for older XMOS code struct r_i2c in i2c.h and module_i2c_master
#define I2C_MASTER_TRANSACTION_MAX_NUMB (SSD1306_WRITE_CHUNK_SIZE * 2) // Just more. Only if asynchnronous mode with i2c_master_async and i2c_master_async_if

#define I2C_INTERNAL_NUM_CLIENTS        1

#define IRQ_HIGH_MAX_TIME_MILLIS        2000 // Longer than debugger with prints time (about 1550 ms)

int main() {

    button_if                i_buttons[BUTTONS_NUM_CLIENTS];
    spi_master_if            i_spi[SPI_NUM_CLIENTS];
    radio_if_t               i_radio;
    chan                     c_irq_update;
    blink_and_watchdog_if_t  i_blink_and_watchdog[BEEP_BLINK_TASK_NUM_CLIENTS];
    i2c_internal_commands_if i_i2c_internal_commands [I2C_INTERNAL_NUM_CLIENTS];
    i2c_master_if            i_i2c[I2C_MASTER_NUM_CLIENTS];

    // Observe http://www.teigfam.net/oyvind/home/technology/098-my-xmos-notes/#xtag-3_debug_log_hanging!

    par {
        // RFM69=003 giving the two first their own cores. Basically want i2c not to take cycles from spi (which they didn't do anyhow)
        //           i2c_master is synchronous and called from RFM69_client so RFM69_client should then not be on some core as spi_master_2 (?)

        on tile[0].core[0]: spi_master_2            (i_spi, SPI_NUM_CLIENTS, p_sclk, p_mosi, p_miso, SPI_CLOCK, p_spi_cs_en, maskof_spi_and_probe_pins, NUM_SPI_CS_SETS); // Is [[distributable]]
        on tile[0].core[1]: RFM69_driver            (i_radio, p_spi_aux, i_spi[SPI_CLIENT_0], SPI_CLIENT_0); // Is [[combineable]]
        on tile[0].core[2]: RFM69_client            (c_irq_update, i_radio, i_blink_and_watchdog[0], SEMANTICS_DO_RSSI_IN_IRQ_DETECT_TASK, i_buttons, i_i2c_internal_commands[0], p_display_notReset);
        on tile[0].core[3]: blink_and_watchdog_task (i_blink_and_watchdog, p_explorer_leds);

        #if (SEMANTICS_DO_RSSI_IN_IRQ_DETECT_TASK==1)
            // Does not work, see XMOS ticket 31286
            IRQ_detect_task (i_irq, p_spi_irq, null, i_spi[SPI_CLIENT_1], SPI_CLIENT_1);
        #else
            on tile[0].core[4]: IRQ_detect_and_follow_task_2 (c_irq_update, p_spi_irq, null, IRQ_HIGH_MAX_TIME_MILLIS); // null since IRQ has a separate LED
        #endif

        on tile[0].core[5]: Button_Task (IOF_BUTTON_LEFT,   inP_button_left,   i_buttons[IOF_BUTTON_LEFT]);   // [[combinable]]
        on tile[0].core[5]: Button_Task (IOF_BUTTON_CENTER, inP_button_center, i_buttons[IOF_BUTTON_CENTER]); // [[combinable]]
        on tile[0].core[5]: Button_Task (IOF_BUTTON_RIGHT,  inP_button_right,  i_buttons[IOF_BUTTON_RIGHT]);  // [[combinable]]

        on tile[0].core[6]: I2C_Internal_Task (i_i2c_internal_commands, i_i2c[0]);
        on tile[0].core[6]: i2c_master (i_i2c, I2C_MASTER_NUM_CLIENTS, p_scl, p_sda, I2C_MASTER_SPEED_KBPS); // Synchronous==distributable
    }

    return 0;
}
