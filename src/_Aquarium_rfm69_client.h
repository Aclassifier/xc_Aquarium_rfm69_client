/*
 * _Aquarium_rfm69_client.h
 *
 *  Created on: 15. aug. 2018
 *      Author: teig
 */

#ifndef AQUARIUM_RFM69_CLIENT_H_
#define AQUARIUM_RFM69_CLIENT_H_

[[combinable]] // Cannot be [[distributable]] since timer case in select
void RFM69_client (
        server  irq_if_t                i_irq,
        client  radio_if_t              i_radio,
        client  blink_and_watchdog_if_t i_beep_blink,
        const   bool                    semantics_do_rssi_in_irq_detect_task,
        server  button_if               i_button_in[BUTTONS_NUM_CLIENTS]);

#endif /* AQUARIUM_RFM69_CLIENT_H_ */
