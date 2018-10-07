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

#define RFM69_CLIENT_VERSION_STR "0.7.1"
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

// FOR ONE-ONE connection MASTER/SLAVE
//                ### MUST BE UNIQUE
#define MASTER_ID  99

#define IS_MYTARGET_VOID               0
#define IS_MYTARGET_STARTKIT           1
#define IS_MYTARGET_XCORE_200_EXPLORER 2

#if (MYTARGET==STARTKIT)
    #define IS_MYTARGET IS_MYTARGET_STARTKIT
#elif (MYTARGET==XCORE-200-EXPLORER)
    #define IS_MYTARGET IS_MYTARGET_XCORE_200_EXPLORER
#else
    #error NO TARGET DEFINED
#endif

#if (ISMASTER==1) // Doesn't seem like words like SLAVE or MASTER or _SLAVE or _MASTER work. Using 0 or 1 instead
    #define IS_MYTARGET_MASTER 1
    #define IS_MYTARGET_SLAVE  0

    #define NODEID    MASTER_ID
    #define GATEWAYID SHARED_ID // Since MASTER it knows this SENDTO_ADDRESS
#elif (ISMASTER==0)
    #define IS_MYTARGET_SLAVE  1
    #define IS_MYTARGET_MASTER 0

    // It is the SLAVE that has to get its RETURNTO/SENDTO address from an established connection
    // The MASTER/SLAVE role rationale is also to avoid application level deadlock or oscillation,
    // we would not want both parts to spontaneously send
    #define NODEID SHARED_ID // RECEIVER_ADDRESS is MASTER'S SENDTO_ADDRESS
    // ANY SENDTO/RETURNTO address picked out of message
#else
    #error NO ROLE DEFINED
#endif

#endif /* VERSION_H_ */

