/*
 * _version.h
 *
 *  Created on: 15. aug. 2018
 *      Author: teig
 */


#ifndef VERSION_H_
#define VERSION_H_

// SHOULD THE LENGTH OF THESE NEED TO CHANGE THEN THE STRING THEY ARE COPIED INTO MUST BE MODIFIED
//
#define XTIMECOMPOSER_VERSION_STR "14.3.3"

#define RFM69_CLIENT_VERSION_STR "0.8.7"
    // "0.8.7"  19Nov2018 feed_watchdog is new etc. Plus a blank screen
    // "0.8.6"  19Nov2018 RX_context.appSeqCnt_prev and calculations around this fixed (had been compromised by some code cut and paste)
    // "0.8.5"  18Nov2018 SCREEN_4_MAX_NA_MIN is new (with Watt etc.)
    // "0.8.4"  18Nov2018 Helper functions to convert and print pluss _USERMAKEFILE_LIB_RFM69_XC_SEMANTICS_DO_INTERMEDIATE_RECEIVEDONE=0 again
    // "0.8.2"  13Nov2018 IRQ and timeout handling now separate functions in _Aquarium_rfm69_client.xc
    // "0.8.1"  02Nov2018 DEG C and WATT ALSO DISPLAYED
    // "0.7.2"  29Oct2018 THIS VERSION SHOWS AQUARIUM TIME AND WATER TEMP IN THE DISPLAY! YESS!
    //          RFM69=001 The IRQ line that keeps high after IRQ 7 (messagePacketLenErr_IRQ) is worked on. Testing at the moment
    //          --"--     This tag is also seen in the "rfm69_commprot.h" code
    // "0.7.1"  07Jun2018 _commprot.h -> renamed to rfm69_commprot.h
    //                    _globals.h  -> renamed to rfm69_globals.h and new _globals.h made
    // "0.7.0"  06Jun2018 File changes before moving to lib_fsm69_xc
    //                    RFM69.h           -> contents moved to rfm69_xc.xc then renamed to rfm69_xc.h
    //                    RFM69_interface.h -> contents moved to rfm69_xc.h then file obsoleted
    //                    RFM69.xc          -> renamed to rfm69_xc.xc
    //                    RFM69.xc          -> renamed to rfm69_xc.xc
    //                    spi_driver.h      -> renamed to rfm69_spi_driver.h
    //                    spi_driver.xc     -> renamed to rfm69_spi_driver.xc
    //                    CRC.h             -> renamed to rfm69_crc.h and uint32_t moved go _globals.h
    //                    CRC.xc            -> renamed to rfm69_crc.xc
    //                    RFM69_registers.h -> renamed to rfm69_registers.h
    // "0.6.0"  05May2018 SEMANTICS_DO_LOOP_FOR_RF_IRQFLAGS2_PACKETSENT is new
    // "0.5.5"  18Apr2018 This is a long-live number during development
    // "0.5.3"  03Apr2018 SHARED_ID etc are new
    // "0.5.2"            File _commprot.h is new etc
    // "0.5.1"  28Feb2018 more advanced handling
    // "0.5.0"  26Feb2018 testing some changes that Maxim initiated
    //          RFM69=000 First testing with two boards

#endif /* VERSION_H_ */

