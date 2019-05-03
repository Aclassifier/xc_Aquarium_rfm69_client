/*
 * _rfm69_commprot.h
 *
 *  Created on: 30. aug. 2018
 *      Author: teig
 */

#ifndef _RFM69_COMMPROT_H_
#define _RFM69_COMMPROT_H_

#define USE_AQUARIUM__RFM69_COMMPROT 1

#if (USE_AQUARIUM__RFM69_COMMPROT==1)
    #undef _RFM69_COMMPROT_H_ // Since it's also used by this:
    #include "../../_Aquarium_1_x/src/_rfm69_commprot.h" // Defines structs like payload_t
    // Also defines SHARED_ID // SENDTO_ADDRESS == RECEIVER_ADDRESS
#else
    // Same contents as in the above inlcuded file here, but now I have removed it
#endif

#endif /* RFM69_COMMPROT_H_ */
