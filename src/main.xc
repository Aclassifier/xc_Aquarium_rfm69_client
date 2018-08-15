/*
 * main.xc
 *
 *  Created on: 15. aug. 2018
 *      Author: teig
 */

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

#include "_version.h"
#include "_globals.h"

#include <rfm69_globals.h>
#include <rfm69_crc.h>
#include <rfm69_commprot.h>
#include <rfm69_xc.h>

#include "_Aquarium_rfm69_client.h"

#define DEBUG_PRINT_RFM69 1
#define debug_print(fmt, ...) do { if(DEBUG_PRINT_RFM69 and (DEBUG_PRINT_GLOBAL_APP==1)) printf(fmt, __VA_ARGS__); } while (0)

#define DEBUG_PRINT_BUFFER    0
#define DEBUG_PRINT_TIME_USED 0


#define SEMANTICS_DO_RSSI_IN_IRQ_DETECT_TASK 0 // # chanends ---MEM---  (relative values)
                                               // 1 :     2    700 bytes  Does it faster after IRQ line (good if much logging in RFM69_driver)
                                               //                         DOES NOT WORK WITH xTIMEcomposer 13.3.3, see XMOS ticket 31286
                                               // 0 :     0      0        RSSI may be measured too late if much logging in RFM69_driver

// http://www.xcore.com/viewtopic.php?f=26&t=6331
#define CAT3(a,b,c) a##b##c
#define XS1_PORT(WIDTH,LETTER) CAT3(XS1_PORT_,WIDTH,LETTER) // XS1_PORT_ is a string here, not some #define from the woods!

                               //                StartKIT                 eXplorerKIT - BUT NOT AS PREDEFINED SPI in their Portmaps
                               //                                         as WiFi sliceCARD My breakpot board
#define SPI_MOSI  XS1_PORT(1,K) // XS1_PORT_1K   X0D34 P1K       PCIe-B10 GPIO-PIN19
#define SPI_CLK   XS1_PORT(1,J) // XS1_PORT_1J   X0D25 P1J       PCIe-A8  GPIO-PIN21
#define SPI_MISO  XS1_PORT(1,I) // XS1_PORT_1I   X0D24 P1I       PCIe-B15 GPIO-PIN23
#define SPI_CS_EN XS1_PORT(4,C) // XS1_PORT_4C   X0D14 P4C0      PCIe-B6  GPIO-PIN47  MASKOF_SPI_SLAVE0_CS          CS/SS Chip select is port BIT0 low
                                // XS1_PORT_4C   X0D15 P4C1      PCIe-B7  GPIO-PIN45  MASKOF_SPI_SLAVE0_EN          nPWR_EN SPI_EN Power enable is port BIT1 high
                                // XS1_PORT_4C   X0D20 P4C2      PCIe-A6  GPIO-PIN43  MASKOF_SPI_SLAVE0_PROBE1_INNER
                                // XS1_PORT_4C   X0D21 P4C3      PCIe-A7  GPIO-PIN42  MASKOF_SPI_SLAVE0_PROBE2_OUTER
#define SPI_AUX   XS1_PORT(4,D) // XS1_PORT_4D   X0D16 P4D0      PCIe-B9  GPIO-PIN31  MASKOF_SPI_AUX0_RST           RST Restart is port BIT0
                                // XS1_PORT_4D   X0D17 P4D1      PCIe-B11 GPIO-PIN29  MASKOF_SPI_AUX0_PROBE3_IRQ
#define SPI_IRQ   XS1_PORT(1,L) // XS1_PORT_1L   X0D35 P1L       PCIe-A15 GPIO-PIN17  IRQ, "GPIO 0", DIO0
#define PROBE4    XS1_PORT(1,F) // XS1_PORT_1F   X0D13 P1F  J7.1 PCIe-B2  GPIO-PIN37  "PROBE1", "PROBE2" & "PROBE3" are in bitmasks
#define PROBE5    XS1_PORT(1,D) // XS1_PORT_1D   X0D11 P1D  J3.21 LED-D2   SPI-MOSI   "PROBE1", "PROBE2" & "PROBE3" are in bitmasks

// From spi_lib spi.pdf
//                            32 bits over 1 bit:        // New as above | As spi_master_interface in main.xc in _app_tiwisl_simple_webserver
in  buffered port:32 p_miso  = on tile[0]: SPI_MISO;     // New as above | Was XS1_PORT_1A, but that's for sliceKIT
out buffered port:32 p_sclk  = on tile[0]: SPI_CLK;      // New as above | Was XS1_PORT_1C, but that's for sliceKIT (Was 22 in spi.pdf but that must be a typo)
out buffered port:32 p_mosi  = on tile[0]: SPI_MOSI;     // New as above | Was XS1_PORT_1D, but that's for sliceKIT
clock                clk_spi = on tile[0]: XS1_CLKBLK_1; // See USE_CLOCK_BLOCK

#if (SEMANTICS_DO_RSSI_IN_IRQ_DETECT_TASK==1)
    #define NUM_SPI_CLIENT_USERS 2 // Number of users per board
#else
    #define NUM_SPI_CLIENT_USERS 1 // Number of users per board
#endif

#define                     SPI_CLIENT_0    0 // BOTH HERE: Remember a call to i_spi.await_spi_port_init_by_all_clients(); before use of spi_master_if (by i_spi)
#define                     SPI_CLIENT_1    1 // AND HERE:  --"--
#define                     SPI_CLIENT_VOID 0 // Any value
#define NUM_SPI_CS_SETS NUM_SPI_CLIENT_USERS  // See (*) below. Actually number of different SPI boards, but since both clients use the same chip, the sets are equal

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

out port p_spi_cs_en = on tile[0]:SPI_CS_EN;
out port p_spi_aux   = on tile[0]:SPI_AUX;
in  port p_spi_irq   = on tile[0]:SPI_IRQ;

// Another way of doing it. Used as nullable parameter, so may be dropped
probe_pins_t probe_config = {
    on tile[0]:PROBE4
};

int main() {

    spi_master_if i_spi[NUM_SPI_CLIENT_USERS];
    radio_if_t    i_radio;
    irq_if_t      i_irq;

    #if (DO_PLACED == 0) // [[combinable] RFM69_driver
        par {
            on tile[0].core[0]: spi_master_2 (i_spi, NUM_SPI_CLIENT_USERS, p_sclk, p_mosi, p_miso, SPI_CLOCK, p_spi_cs_en, maskof_spi_and_probe_pins, NUM_SPI_CS_SETS); // Is [[distributable]]
            on tile[0].core[0]: RFM69_driver (i_radio, p_spi_aux, i_spi[SPI_CLIENT_0], SPI_CLIENT_0); // Is [[combineable]]
            on tile[0].core[0]: RFM69_client (i_irq, i_radio);

            #if (SEMANTICS_DO_RSSI_IN_IRQ_DETECT_TASK==1)
                // Does not work, see XMOS ticket 31286
                on tile[0].core[0]: IRQ_detect_task (i_irq, p_spi_irq, probe_config, i_spi[SPI_CLIENT_1], SPI_CLIENT_1);
            #else
                on tile[0].core[0]: IRQ_detect_task (i_irq, p_spi_irq, probe_config, null, SPI_CLIENT_VOID);
            #endif
        }
        /*
        Constraint check for tile[0]:
          Cores available:            8,   used:          1 .  OKAY
          Timers available:          10,   used:          1 .  OKAY
          Chanends available:        32,   used:          0 .  OKAY
          Memory available:       65536,   used:      12532 .  OKAY
            (Stack: 1112, Code: 10518, Data: 902)
        Constraints checks PASSED.
        */
    #elif (DO_PLACED == 1)
        [[combine]]
        par {
            spi_master_2 (i_spi, NUM_SPI_CLIENT_USERS, p_sclk, p_mosi, p_miso, SPI_CLOCK, p_spi_cs_en, maskof_spi_and_probe_pins, NUM_SPI_CS_SETS); // Is [[distributable]]
            RFM69_driver (i_radio, p_spi_aux, i_spi[SPI_CLIENT_0], SPI_CLIENT_0); // Is [[combineable]]
            RFM69_client (i_irq, i_radio, SEMANTICS_DO_RSSI_IN_IRQ_DETECT_TASK);

            #if (SEMANTICS_DO_RSSI_IN_IRQ_DETECT_TASK==1)
                // Does not work, see XMOS ticket 31286
                IRQ_detect_task (i_irq, p_spi_irq, probe_config, i_spi[SPI_CLIENT_1], SPI_CLIENT_1);
            #else
                IRQ_detect_task (i_irq, p_spi_irq, probe_config, null, SPI_CLIENT_VOID);
            #endif
        }
        /* // [[combinable] RFM69_driver
        Constraint check for tile[0]:
          Cores available:            8,   used:          1 .  OKAY
          Timers available:          10,   used:          1 .  OKAY
          Chanends available:        32,   used:          0 .  OKAY
          Memory available:       65536,   used:      12456 .  OKAY
            (Stack: 1108, Code: 10452, Data: 896)
        Constraints checks PASSED.
        */

        /* // [[distributable] RFM69_driver
        Constraint check for tile[0]:
          Cores available:            8,   used:          1 .  OKAY
          Timers available:          10,   used:          1 .  OKAY
          Chanends available:        32,   used:          0 .  OKAY
          Memory available:       65536,   used:      12508 .  OKAY
            (Stack: 1116, Code: 10496, Data: 896)
        Constraints checks PASSED
        */

    #elif (DO_PLACED == 2) // [[distributable]] RFM69_driver. No way to do that except edit there (see Issue 31429)
        [[combine]]
        par {
            spi_master_2 (i_spi, NUM_SPI_CLIENT_USERS, p_sclk, p_mosi, p_miso, SPI_CLOCK, p_spi_cs_en, maskof_spi_and_probe_pins, NUM_SPI_CS_SETS); // Is [[distributable]]
            RFM69_driver (i_radio, p_spi_aux, i_spi[SPI_CLIENT_0], SPI_CLIENT_0); // Is [[distributable]]
            RFM69_client (i_irq, i_radio);

            #if (SEMANTICS_DO_RSSI_IN_IRQ_DETECT_TASK==1)
                // Does not work, see XMOS ticket 31286
                IRQ_detect_task (i_irq, p_spi_irq, probe_config, i_spi[SPI_CLIENT_1], SPI_CLIENT_1);
            #else
                IRQ_detect_task (i_irq, p_spi_irq, probe_config, null, SPI_CLIENT_VOID);
            #endif
        }
        /*
        Constraint check for tile[0]:
          Cores available:            8,   used:          1 .  OKAY
          Timers available:          10,   used:          1 .  OKAY
          Chanends available:        32,   used:          0 .  OKAY
          Memory available:       65536,   used:      12508 .  OKAY
            (Stack: 1116, Code: 10496, Data: 896)
        Constraints checks PASSED.
        */
    #elif (DO_PLACED == 3) // [[distributable]] RFM69_driver. No way to do that except edit there (see Issue 31429)
        par {
            spi_master_2 (i_spi, NUM_SPI_CLIENT_USERS, p_sclk, p_mosi, p_miso, SPI_CLOCK, p_spi_cs_en, maskof_spi_and_probe_pins, NUM_SPI_CS_SETS); // Is [[distributable]]
            RFM69_driver (i_radio, p_spi_aux, i_spi[SPI_CLIENT_0], SPI_CLIENT_0);
            RFM69_client (i_irq, i_radio);

            #if (SEMANTICS_DO_RSSI_IN_IRQ_DETECT_TASK==1)
                // Does not work, see XMOS ticket 31286
                IRQ_detect_task (i_irq, p_spi_irq, probe_config, i_spi[SPI_CLIENT_1], SPI_CLIENT_1);
            #else
                IRQ_detect_task (i_irq, p_spi_irq, probe_config, null, SPI_CLIENT_VOID);
            #endif
        }
        /*
        Constraint check for tile[0]:
          Cores available:            8,   used:          2 .  OKAY
          Timers available:          10,   used:          2 .  OKAY
          Chanends available:        32,   used:          2 .  OKAY
          Memory available:       65536,   used:      11684 .  OKAY
            (Stack: 1476, Code: 9332, Data: 876)
        Constraints checks PASSED.
        */
    #elif (DO_PLACED == 4) // [[distributable]] RFM69_driver. No way to do that except edit there (see Issue 31429)
        par {
            [[combine]]
            par {
                RFM69_client (i_irq, i_radio);

                #if (SEMANTICS_DO_RSSI_IN_IRQ_DETECT_TASK==1)
                    // Does not work, see XMOS ticket 31286
                    IRQ_detect_task (i_irq, p_spi_irq, probe_config, i_spi[SPI_CLIENT_1], SPI_CLIENT_1);
                #else
                    IRQ_detect_task (i_irq, p_spi_irq, probe_config, null, SPI_CLIENT_VOID);
                #endif
            }
            [[distribute]]
            par {
                // ^ *** error: distributed statement must be a call to a distributable function. WRONG ERROR MESSAGE
                spi_master_2 (i_spi, NUM_SPI_CLIENT_USERS, p_sclk, p_mosi, p_miso, SPI_CLOCK, p_spi_cs_en, maskof_spi_and_probe_pins, NUM_SPI_CS_SETS); // Is [[distributable]]
                RFM69_driver (i_radio, p_spi_aux, i_spi[SPI_CLIENT_0], SPI_CLIENT_0);
            }
        }

        /*

        */
    #endif
    return 0;
}
