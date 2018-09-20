/*
 * _Aquarium_rfm69_client.h
 *
 *  Created on: 15. aug. 2018
 *      Author: teig
 */

#ifndef AQUARIUM_RFM69_CLIENT_H_
#define AQUARIUM_RFM69_CLIENT_H_

#define XCORE_200_EXPLORER_LEDS                   XS1_PORT_4F
#define XCORE_200_EXPLORER_LED_GREEN_BIT_MASK     0x01
#define XCORE_200_EXPLORER_LED_RGB_BLUE_BIT_MASK  0x02
#define XCORE_200_EXPLORER_LED_RGB_GREEN_BIT_MASK 0x04
#define XCORE_200_EXPLORER_LED_RGB_RED_BIT_MASK   0x08
#define XCORE_200_EXPLORER_LEDS_ALL_BIT_MASK      0x0f // Looks blue'ish

#define LED

[[combinable]] // Cannot be [[distributable]] since timer case in select
void RFM69_client (
        server irq_if_t   i_irq,
        client radio_if_t i_radio,
        const  bool       semantics_do_rssi_in_irq_detect_task,
        out port          p_explorer_leds);

#endif /* AQUARIUM_RFM69_CLIENT_H_ */
