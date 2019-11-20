/*
 * _Aquarium_rfm69_client.h
 *
 *  Created on: 15. aug. 2018
 *      Author: teig
 */

#ifndef AQUARIUM_RFM69_CLIENT_H_
#define AQUARIUM_RFM69_CLIENT_H_

void RFM69_client (
        chanend                          c_irq_update,
        client  radio_if_t               i_radio,
        client  blink_and_watchdog_if_t  i_beep_blink,
        server  button_if                i_button_in[BUTTONS_NUM_CLIENTS],
        client  i2c_internal_commands_if i_i2c_internal_commands,
        client  i2c_general_commands_if  i_i2c_general_commands,
        out port                         p_display_notReset);

#endif /* AQUARIUM_RFM69_CLIENT_H_ */
