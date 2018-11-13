/*
 * _app_rfm69_on_xmos_native.xc
 *
 *  Created on: 15. aug. 2018
 *      Author: teig
 *      INSPIRED FROM AND PORTED FROM THE RFM69 LIBRARY
 *      See https://lowpowerlab.com/2013/06/20/rfm69-library/
 *      And LICENCE info here:
 */
#ifdef SHOW_RFM69_LICENCE
    // Copyright Felix Rusu 2016, http://www.LowPowerLab.com/contact
    // **********************************************************************************
    // License
    // **********************************************************************************
    // This program is free software; you can redistribute it
    // and/or modify it under the terms of the GNU General
    // Public License as published by the Free Software
    // Foundation; either version 3 of the License, or
    // (at your option) any later version.
    //
    // This program is distributed in the hope that it will
    // be useful, but WITHOUT ANY WARRANTY; without even the
    // implied warranty of MERCHANTABILITY or FITNESS FOR A
    // PARTICULAR PURPOSE. See the GNU General Public
    // License for more details.
    //
    // Licence can be viewed at
    // http://www.gnu.org/licenses/gpl-3.0.txt
    //
    // Please maintain this license information along with authorship
    // and copyright notices in any redistribution of this code
    // **********************************************************************************
#endif

#define INCLUDES
#ifdef INCLUDES
#include <xs1.h>
#include <platform.h> // slice
#include <timer.h>    // delay_milliseconds(200), XS1_TIMER_HZ etc
#include <stdint.h>   // uint8_t
#include <stdio.h>    // printf
#include <string.h>   // memcpy, memcmp
#include <xccompat.h> // REFERENCE_PARAM(my_app_ports_t, my_app_ports) -> my_app_ports_t &my_app_ports
#include <iso646.h>   // not etc.
#include <spi.h>
#include <xassert.h>
#include <limits.h>
#include <i2c.h>

#include "_version.h"
#include "_globals.h"
#include "blink_and_watchdog.h"

#include "param.h"
#include "defines_adafruit.h"
#include "i2c_internal_task.h"
#include "display_ssd1306.h"
#include "core_graphics_adafruit_gfx.h"
#include "core_graphics_font5x8.h"
#include "_texts_and_constants.h"

#include <rfm69_globals.h>
#include <rfm69_crc.h>
#include <rfm69_commprot.h>
#include <rfm69_xc.h>

#include "_rfm69_commprot.h"
#include "button_press.h"

#include "_Aquarium_rfm69_client.h"
#endif

#define DEBUG_PRINT_RFM69 1
#define debug_print(fmt, ...) do { if(DEBUG_PRINT_RFM69 and (DEBUG_PRINT_GLOBAL_APP==1)) printf(fmt, __VA_ARGS__); } while (0)

#define DEBUG_PRINT_BUFFER    0
#define DEBUG_PRINT_TIME_USED 0

#if (WARNINGS==1)
    #if (IS_MYTARGET==ISMYTARGET_STARTKIT)
        #warning IS STARTKIT
    #elif (IS_MYTARGET==IS_MYTARGET_XCORE_200_EXPLORER)
        #warning IS XCORE_200_EXPLORER
    #endif
    #if (IS_MYTARGET_MASTER==1)
        #warning IS MASTER
    #elif (IS_MYTARGET_SLAVE==1)
        #warning IS SLAVE
    #endif
#endif


#define SEMANTICS_DO_INTERMEDIATE_RECEIVEDONE _USERMAKEFILE_LIB_RFM69_XC_SEMANTICS_DO_INTERMEDIATE_RECEIVEDONE

#define SEMANTICS_DEBUG_CALC_RF_FRF_REGISTERS 1 // 1 : never while real TX->RX!. Also set ALLOW_FLOAT_FREQ_CALC 1 for calculations
                                                // 0 : Standard usage

// 433 MHz band shared with our (1) indoor/outdoor thermometer and ("433") (2) kitchen light switch over the table and ("433.92")
// (3) car key ("433.92") - so it's a good idea to avoid 433.92 MHz at least
#define TEST_CAR_KEY 0
//
#if (TEST_CAR_KEY == 1)
    #define MY_RFM69_FREQ_HZ     RF69_INIT_FREQUENCY_433919921_HZ // 433.92 MHz same as my car key
    #define MY_RFM69_FREQ_REGS            RF_FRF_433_433919921    // Is 0x006C7AE0
            // See http://www.teigfam.net/oyvind/home/technology/164-my-aquariums-data-radioed-through-the-shelf/#frequency_crash_with_my_car_key
            // and https://forum.allaboutcircuits.com/threads/radio-board-interferes-with-my-car-key-system.151059/
            //   The hypothesis there seems to be that my key must send a unique on/off pattern (one for lock and one for unlock),
            //   typically modulated at 10 MHz with a bit rate of 40 kbps, repeated every 20 ms for 500 ms before the receiver in the car
            //   will accept it. There probably is no packetisation or cryptographic key, just that repeated stream of bits.
            //   So when I sent my packets every 200 ms I destroyed that data stream. This seems very plausible.
            //   Thanks to sghioto and DickCappels! See Wikipedia article On-off keying (https://en.wikipedia.org/wiki/On-off_keying) and
            //   Application note 4439 from Maxim integrated at IÕm OOK. YouÕre OOK? (https://www.maximintegrated.com/en/app-notes/index.mvp/id/4439)
#else
    #define MY_RFM69_FREQ_HZ     RF69_INIT_FREQUENCY_433705993_HZ // Frequency is according to calculator 0x006C6D2F * (RF69_FSTEP_FLOAT32 as 61.03515625)
                                                                  // Only needed for printouts and to check calculator-accurate calculation of these hex values:
    #define MY_RFM69_FREQ_REGS            RF_FRF_433_433705993    // Is 0x006C6D2F
#endif

//      SPI_AUX bits:
#define MASKOF_SPI_AUX0_RST        0x01 // RST is port SPI_AUX BIT0. Search for SPI_AUX0_RST to see HW-defined timing
#define MASKOF_SPI_AUX0_PROBE3_IRQ 0x02 // Test pin for IRQ. "LED1"

//           ##                Same number then related
#define TEST_01_FOLLOW_ADDRESS 1 // Testet OK 04Apr2018
#define TEST_01_LISTENTOALL    0 // Tested OK 04Apr2018

#define ONE_SECOND_TICKS                        (1000 * XS1_TIMER_KHZ) // Expands to clock frequency = 100 mill = 100 * 1000 * 1000
#define SEND_PACKET_ON_NO_CHANGE_TIMOEUT_SECONDS 1                     // MINIMUM 1! For sendPacket_seconds_cntdown

#define KEY            MY_KEY // From *_commprot.h TODO move it into aquarium _rfm69_commprot.h away from the library files
#define IS_RFM69HW_HCW true   // 1 for Adafruit RFM69HCW (high power)

#define CHAR_EQ_STR     "="
#define CHAR_CHANGE_STR "#"

typedef struct {
    time32_t   max_diffTime_ms;
    time32_t   sum_diffTime_ms;
    time32_t   mean_diffTime_ms;
    time32_t   prev_mean_diffTime_ms;
    bool       changed_mean_diffTime_ms;
    unsigned   num_diffTime_ms;
    //
} diffTime_t;

typedef struct {
    rfm69_params_t         radio_init;
    int16_t                irq_value;
    uint8_t                device_type;
    bool                   receiveDone;
    is_error_e             is_new_error;
    uint32_t               interruptCnt;
    some_rfm69_internals_t some_rfm69_internals;
    packet_t               PACKET;
    //
} RXTX_context_t;

#define RX_PACKET_U RXTX_context.PACKET
#define TX_PACKET_U RXTX_context.PACKET // Same

typedef struct {
    bool      doListenToAll;
    unsigned  num_totLost;
    unsigned  num_radioCRC16errs;
    unsigned  num_appCRC32errs;
    unsigned  seconds_since_last_received;
    bool      first_debug_print_received_done;
    uint32_t  lastReceivedAppSeqCnt;
    payload_t RX_radio_payload_prev;
    payload_t RX_radio_payload_max;
    payload_t RX_radio_payload_min;
    #if (_USERMAKEFILE_LIB_RFM69_XC_GETDEBUG==1)
        uint8_t debug_data_prev[NUM_DEBUG_BYTES];
    #endif
    //
} RX_context_t; // RX same as SLAVE same as ISMASTER==0

typedef struct {
    uint8_t                    TX_appPowerLevel_dBm;
    uint8_t                    TX_gatewayid;
    uint32_t                   TX_appSeqCnt;
    unsigned                   sendPacket_seconds_cntdown; // Down from SEND_PACKET_ON_NO_CHANGE_TIMOEUT_SECONDS
    waitForIRQInterruptCause_e waitForIRQInterruptCause;
    //
} TX_context_t; // TX same as MASTER same as ISMASTER==1

typedef struct {
    char display_ts1_chars [SSD1306_TS1_DISPLAY_VISIBLE_CHAR_LEN]; // 84 chars for display needs 85 char buffer (with NUL) when sprintf is use (use SSD1306_TS1_DISPLAY_ALL_CHAR_LEN for full flexibility)
    int  sprintf_numchars;
} display_context_t;

void RFM69_handle_irq (
         RX_context_t                     &?RX_context,
         TX_context_t                     &?TX_context,
         RXTX_context_t                   &RXTX_context,
         display_context_t                &display_context,
         client  radio_if_t                i_radio,
         client  blink_and_watchdog_if_t   i_blink_and_watchdog,
         const   bool                      semantics_do_rssi_in_irq_detect_task,
         client  i2c_internal_commands_if  i_i2c_internal_commands)
{
    i_blink_and_watchdog.blink_pulse_ok (XCORE_200_EXPLORER_LED_RGB_BLUE_BIT_MASK, 50);

    int16_t nowRSSI;
    if (semantics_do_rssi_in_irq_detect_task) {
        nowRSSI = RXTX_context.irq_value;
    } else {
        nowRSSI = i_radio.readRSSI_dBm (FORCETRIGGER_OFF);
    }

    // c_irq_rising is delivered from IRQ_detect_task also if pin change was over while _this_ task
    // was in an i_radio call, which might last longer than the IRQ pulse! We saw that this
    // construct used here was never taken:
    //     case p_spi_irq when pinsneq(spi_irq_current_val) :> spi_irq_current_val: {
    // So we offloaded to IRQ_detect_task instead
    interruptAndParsingResult_e interruptAndParsingResult;

    RXTX_context.interruptCnt++;

    i_radio.do_spi_aux_pin (MASKOF_SPI_AUX0_PROBE3_IRQ, high); // For scope

    {RXTX_context.some_rfm69_internals, RX_PACKET_U, interruptAndParsingResult} = i_radio.handleSPIInterrupt();


    #if (DEBUG_PRINT_BUFFER==1)
        #if (IS_MYTARGET_MASTER == 1)
            debug_print ("%sIRQ %u:\n", char_leading_space_str, interruptAndParsingResult);
        #endif

        for (unsigned index = 0; index < PACKET_LEN32; index++) {
            unsigned value = RX_PACKET_U.u.packet_u2_uint32_arr[index];
            debug_print ("%s[%u]: %08X\n",
                    (interruptAndParsingResult == messageReceivedOk_IRQ)   ? "RX" :
                    (interruptAndParsingResult == messagePacketSentOk_IRQ) ? "TX" : "?X",
                    index,
                    value);
        }

        debug_print ("%s", char_leading_space_str);
    #endif

    RXTX_context.receiveDone = i_radio.receiveDone(); // For any interruptAndParsingResult (30Aug2018, 12Sept2018 TODO works?)

    switch (interruptAndParsingResult) {

        #if (IS_MYTARGET_SLAVE == 1)
            case messageReceivedOk_IRQ: {
                // if (i_radio.receiveDone()) {
                if (RXTX_context.receiveDone) {

                    payload_t RX_radio_payload; // Copy it out and use rather than typecast
                    int degC_dp1;
                    int degC_Unary_Part;
                    int degC_Decimal_Part;
                    int Volt_dp1;
                    int Volt_Unary_Part;
                    int Volt_Decimal_Part;

                    RX_context.seconds_since_last_received = 0;

                    int32_t num_messages_lost_since_last_success; // May be negative if sender restarts

                    if (RX_context.first_debug_print_received_done) {
                        debug_print ("\nRSSI %d, P %u, ",
                                nowRSSI,
                                RX_PACKET_U.u.packet_u3.appPowerLevel_dBm);
                    } else {
                        debug_print ("\nSENDERID %d, RSSI %d, P %u, ",
                               RXTX_context.some_rfm69_internals.SENDERID,
                               nowRSSI,
                               RX_PACKET_U.u.packet_u3.appPowerLevel_dBm);
                    }

                    if (RX_context.doListenToAll) {
                        debug_print ("to %d, ", RXTX_context.some_rfm69_internals.TARGETID);
                    } else {}

                    num_messages_lost_since_last_success = RX_PACKET_U.u.packet_u3.appSeqCnt - RX_context.lastReceivedAppSeqCnt - 1;

                    if (RX_context.first_debug_print_received_done) {

                        debug_print ("RXappSeqCnt %u ", RX_PACKET_U.u.packet_u3.appSeqCnt);
                        if (num_messages_lost_since_last_success < 0) {
                            if (RX_context.num_totLost != 0) {
                                debug_print ("Sender restarted? (RX_context.num_totLost %u kept)\n", RX_context.num_totLost);
                            } else {
                                debug_print ("%s", "\n");
                            }
                        } else if (num_messages_lost_since_last_success == 0) {
                            debug_print ("%s", "\n");
                        } else { // num_messages_lost_since_last_success > 0
                            RX_context.num_totLost += num_messages_lost_since_last_success;
                            debug_print ("num_messages_lost_since_last_success %d, RX_context.num_totLost %u\n", num_messages_lost_since_last_success, RX_context.num_totLost);
                            // 03Apr2018: one lost in 140, then in 141 (of tenths of thousands!), one in 263
                        }
                    } else {
                        debug_print ("numbytes %u, from NODEID %u, RXappSeqCnt %u\n",
                               RXTX_context.some_rfm69_internals.PACKETLEN,
                               RX_PACKET_U.u.packet_u3.appNODEID,
                               RX_PACKET_U.u.packet_u3.appSeqCnt);
                        num_messages_lost_since_last_success = 0; // Testing on it for diff. To avoid diff both on first and second after start.
                    }

                    RX_context.lastReceivedAppSeqCnt = RX_PACKET_U.u.packet_u3.appSeqCnt;

                    for (unsigned index = 0; index < _USERMAKEFILE_LIB_RFM69_XC_PAYLOAD_LEN08; index++) {
                        RX_radio_payload.u.payload_u1_uint8_arr [index] = RX_PACKET_U.u.packet_u3.appPayload_uint8_arr[index]; // Received now
                    }

                    RX_context.RX_radio_payload_max.u.payload_u0.heater_on_percent                        = max (RX_context.RX_radio_payload_max.u.payload_u0.heater_on_percent,                        RX_radio_payload.u.payload_u0.heater_on_percent);
                    RX_context.RX_radio_payload_max.u.payload_u0.heater_on_watt                           = max (RX_context.RX_radio_payload_max.u.payload_u0.heater_on_watt,                           RX_radio_payload.u.payload_u0.heater_on_watt);
                    RX_context.RX_radio_payload_max.u.payload_u0.i2c_temp_heater_onetenthDegC             = max (RX_context.RX_radio_payload_max.u.payload_u0.i2c_temp_heater_onetenthDegC,             RX_radio_payload.u.payload_u0.i2c_temp_heater_onetenthDegC);
                    RX_context.RX_radio_payload_max.u.payload_u0.i2c_temp_ambient_onetenthDegC            = max (RX_context.RX_radio_payload_max.u.payload_u0.i2c_temp_ambient_onetenthDegC,            RX_radio_payload.u.payload_u0.i2c_temp_ambient_onetenthDegC);
                    RX_context.RX_radio_payload_max.u.payload_u0.i2c_temp_water_onetenthDegC              = max (RX_context.RX_radio_payload_max.u.payload_u0.i2c_temp_water_onetenthDegC,              RX_radio_payload.u.payload_u0.i2c_temp_water_onetenthDegC);
                    RX_context.RX_radio_payload_max.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC = max (RX_context.RX_radio_payload_max.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC, RX_radio_payload.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC);
                    RX_context.RX_radio_payload_max.u.payload_u0.internal_box_temp_onetenthDegC           = max (RX_context.RX_radio_payload_max.u.payload_u0.internal_box_temp_onetenthDegC,           RX_radio_payload.u.payload_u0.internal_box_temp_onetenthDegC);

                    RX_context.RX_radio_payload_min.u.payload_u0.heater_on_percent                        = min (RX_context.RX_radio_payload_min.u.payload_u0.heater_on_percent,                        RX_radio_payload.u.payload_u0.heater_on_percent);
                    RX_context.RX_radio_payload_min.u.payload_u0.heater_on_watt                           = min (RX_context.RX_radio_payload_min.u.payload_u0.heater_on_watt,                           RX_radio_payload.u.payload_u0.heater_on_watt);
                    RX_context.RX_radio_payload_min.u.payload_u0.i2c_temp_heater_onetenthDegC             = min (RX_context.RX_radio_payload_min.u.payload_u0.i2c_temp_heater_onetenthDegC,             RX_radio_payload.u.payload_u0.i2c_temp_heater_onetenthDegC);
                    RX_context.RX_radio_payload_min.u.payload_u0.i2c_temp_ambient_onetenthDegC            = min (RX_context.RX_radio_payload_min.u.payload_u0.i2c_temp_ambient_onetenthDegC,            RX_radio_payload.u.payload_u0.i2c_temp_ambient_onetenthDegC);
                    RX_context.RX_radio_payload_min.u.payload_u0.i2c_temp_water_onetenthDegC              = min (RX_context.RX_radio_payload_min.u.payload_u0.i2c_temp_water_onetenthDegC,              RX_radio_payload.u.payload_u0.i2c_temp_water_onetenthDegC);
                    RX_context.RX_radio_payload_min.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC = min (RX_context.RX_radio_payload_min.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC, RX_radio_payload.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC);
                    RX_context.RX_radio_payload_min.u.payload_u0.internal_box_temp_onetenthDegC           = min (RX_context.RX_radio_payload_min.u.payload_u0.internal_box_temp_onetenthDegC,           RX_radio_payload.u.payload_u0.internal_box_temp_onetenthDegC);

                    {
                        unsigned rest;
                        unsigned thousands;
                        unsigned hundreds;
                        unsigned tens;

                        rest      = RX_radio_payload.u.payload_u0.application_version_num;
                        thousands = rest / 1000;
                        rest      = rest - (thousands * 1000);
                        hundreds  = rest / 100;
                        rest      = rest - (hundreds * 100);
                        tens      = rest;
                        debug_print ("Version %01u.%01u.%02u\n", thousands, hundreds, tens); // 1110 -> 1.1.10

                        debug_print ("version_of_full_payload %u, num_of_this_app_payload %u\n",
                                RX_PACKET_U.u.packet_u3.appHeading.version_of_full_payload,
                                RX_PACKET_U.u.packet_u3.appHeading.num_of_this_app_payload);
                    }

                    { // DISPLAY
                        debug_print ("%s", "I2C ");

                        bool i2c_ok;
                        const char char_degC_circle_str[] = DEGC_CIRCLE_STR;

                        Clear_All_Pixels_In_Buffer();
                        for (int index_of_char = 0; index_of_char < NUM_ELEMENTS(display_context.display_ts1_chars); index_of_char++) {
                            display_context.display_ts1_chars [index_of_char] = ' ';
                        }

                        setTextSize(2);
                        setTextColor(WHITE);
                        setCursor(0,0);

                        degC_dp1          = RX_radio_payload.u.payload_u0.i2c_temp_water_onetenthDegC;
                        degC_Unary_Part   = degC_dp1/10;
                        degC_Decimal_Part = degC_dp1 - (degC_Unary_Part*10);

                        display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars, "%02u:%02u:%02u",
                                RX_radio_payload.u.payload_u0.hour,
                                RX_radio_payload.u.payload_u0.minute,
                                RX_radio_payload.u.payload_u0.second);
                        setTextSize(2);
                        display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL

                        display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars, "%s%u",
                                RX_radio_payload.u.payload_u0.num_days_since_start <= 999 ? " " : "", // To get it on that line
                                RX_radio_payload.u.payload_u0.num_days_since_start);
                        setTextSize(1);
                        display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL

                        display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars, "\n");
                        setTextSize(2);
                        display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL

                        display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars, "%u.%u%sC %uW",
                                degC_Unary_Part, degC_Decimal_Part,
                                char_degC_circle_str,
                                RX_radio_payload.u.payload_u0.heater_on_watt);
                        setTextSize(2);
                        display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL

                        i2c_ok = writeToDisplay_i2c_all_buffer(i_i2c_internal_commands);
                        debug_print ("%s\n", i2c_ok ? "ok2" : "err2");
                    }

                    debug_print ("num_days_since_start%s%04u at %02u:%02u:%02u\n",
                            (RX_radio_payload.u.payload_u0.num_days_since_start == RX_context.RX_radio_payload_prev.u.payload_u0.num_days_since_start) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                             RX_radio_payload.u.payload_u0.num_days_since_start,
                             RX_radio_payload.u.payload_u0.hour,
                             RX_radio_payload.u.payload_u0.minute,
                             RX_radio_payload.u.payload_u0.second);

                    debug_print ("error_bits_now%s0x%04x (error_bits_history%s0x%04x)\n",
                            (RX_radio_payload.u.payload_u0.error_bits_now == RX_context.RX_radio_payload_prev.u.payload_u0.error_bits_now) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                             RX_radio_payload.u.payload_u0.error_bits_now,
                            (RX_radio_payload.u.payload_u0.error_bits_history == RX_context.RX_radio_payload_prev.u.payload_u0.error_bits_history) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                             RX_radio_payload.u.payload_u0.error_bits_history);

                    degC_dp1          = RX_radio_payload.u.payload_u0.i2c_temp_water_onetenthDegC;
                    degC_Unary_Part   = degC_dp1/10;
                    degC_Decimal_Part = degC_dp1 - (degC_Unary_Part*10);
                    //
                    debug_print ("Water %u.%udegC - ", degC_Unary_Part, degC_Decimal_Part);

                    degC_dp1          = RX_radio_payload.u.payload_u0.i2c_temp_ambient_onetenthDegC;
                    degC_Unary_Part   = degC_dp1/10;
                    degC_Decimal_Part = degC_dp1 - (degC_Unary_Part*10);
                    //
                    debug_print ("ambient %u.%udegC\n", degC_Unary_Part, degC_Decimal_Part);

                    degC_dp1          = RX_radio_payload.u.payload_u0.i2c_temp_heater_onetenthDegC;
                    degC_Unary_Part   = degC_dp1/10;
                    degC_Decimal_Part = degC_dp1 - (degC_Unary_Part*10);
                    //
                    debug_print ("Heater now_regulating_at%s%u temp %u.%udegC, ",
                            (RX_radio_payload.u.payload_u0.now_regulating_at == RX_context.RX_radio_payload_prev.u.payload_u0.now_regulating_at) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                             RX_radio_payload.u.payload_u0.now_regulating_at,
                             degC_Unary_Part,
                             degC_Decimal_Part);

                    degC_dp1          = RX_radio_payload.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC;
                    degC_Unary_Part   = degC_dp1/10;
                    degC_Decimal_Part = degC_dp1 - (degC_Unary_Part*10);
                    debug_print ("mean %u.%udegC heater_on_percent%s%u%% heater_on_watt%s%uW\n", degC_Unary_Part, degC_Decimal_Part,
                            (RX_radio_payload.u.payload_u0.heater_on_percent == RX_context.RX_radio_payload_prev.u.payload_u0.heater_on_percent) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                             RX_radio_payload.u.payload_u0.heater_on_percent,
                            (RX_radio_payload.u.payload_u0.heater_on_watt == RX_context.RX_radio_payload_prev.u.payload_u0.heater_on_watt) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                             RX_radio_payload.u.payload_u0.heater_on_watt);

                    const char light_control_scheme_strings [][LIGHT_CONTROL_SCHEME_TEXT_TOTLEN] = LIGHT_CONTROL_SCHEME_STRINGS;
                    debug_print ("Light light_control_scheme%s%s with light_composition%s%02u gives FCB %u/3 %u/3 %u/3 full%s%u/3 day%s%uh (%u-%u)\n",
                            (RX_radio_payload.u.payload_u0.light_control_scheme == RX_context.RX_radio_payload_prev.u.payload_u0.light_control_scheme) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                             light_control_scheme_strings[RX_radio_payload.u.payload_u0.light_control_scheme],
                            (RX_radio_payload.u.payload_u0.light_composition == RX_context.RX_radio_payload_prev.u.payload_u0.light_composition) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                             RX_radio_payload.u.payload_u0.light_composition,
                             RX_radio_payload.u.payload_u0.light_intensity_thirds_front,
                             RX_radio_payload.u.payload_u0.light_intensity_thirds_center,
                             RX_radio_payload.u.payload_u0.light_intensity_thirds_back,
                            (RX_radio_payload.u.payload_u0.light_amount_full_or_two_thirds == RX_context.RX_radio_payload_prev.u.payload_u0.light_amount_full_or_two_thirds) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                             RX_radio_payload.u.payload_u0.light_amount_full_or_two_thirds - NORMAL_LIGHT_THIRDS_OFFSET,
                            (RX_radio_payload.u.payload_u0.light_daytime_hours == RX_context.RX_radio_payload_prev.u.payload_u0.light_daytime_hours) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                             RX_radio_payload.u.payload_u0.light_daytime_hours,
                             RX_radio_payload.u.payload_u0.day_start_light_hour,
                             RX_radio_payload.u.payload_u0.night_start_dark_hour);

                    Volt_dp1          = RX_radio_payload.u.payload_u0.rr_24V_heat_onetenthV;
                    Volt_Unary_Part   = Volt_dp1/10;
                    Volt_Decimal_Part = Volt_dp1 - (Volt_Unary_Part*10);
                    debug_print ("Voltage at heater %02u.%uV, ", Volt_Unary_Part, Volt_Decimal_Part);

                    Volt_dp1          = RX_radio_payload.u.payload_u0.rr_12V_LEDlight_onetenthV;
                    Volt_Unary_Part   = Volt_dp1/10;
                    Volt_Decimal_Part = Volt_dp1 - (Volt_Unary_Part*10);
                    debug_print ("light %02u.%uV\n", Volt_Unary_Part, Volt_Decimal_Part);

                    degC_dp1          = RX_radio_payload.u.payload_u0.internal_box_temp_onetenthDegC;
                    degC_Unary_Part   = degC_dp1/10;
                    degC_Decimal_Part = degC_dp1 - (degC_Unary_Part*10);
                    debug_print ("Box %02u.%udegC\n", degC_Unary_Part, degC_Decimal_Part);

                    debug_print ("Debug%s%02X\n",
                            (RX_radio_payload.u.payload_u0.debug == RX_context.RX_radio_payload_prev.u.payload_u0.debug) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                            RX_radio_payload.u.payload_u0.debug);

                    RX_context.first_debug_print_received_done = true;

                    // RFM69 had a call to receiveDone(); here, only needed if setMode(RF69_MODE_STANDBY) case 1 in receiveDone
                    // Reinserted RFM69=001
                    #if (SEMANTICS_DO_INTERMEDIATE_RECEIVEDONE == 1) // TODO remove
                       if (i_radio.receiveDone()) {                          // This is needed even if..
                           debug_print ("%s\n", "receiveDone in polling!");  // ..it never gets here! (TODO?)
                       } else {}
                    #endif

                    i_blink_and_watchdog.blink_pulse_ok (XCORE_200_EXPLORER_LED_RGB_RED_BIT_MASK, 50); // Looks orange

                    if (num_messages_lost_since_last_success == 0) {
                        // BOTH 40 5Oct2018: debug_print ("sizeof %u, LEN %u\n", sizeof RX_radio_payload.u.payload_u1_uint8_arr, _USERMAKEFILE_LIB_RFM69_XC_PAYLOAD_LEN08);
                        for (unsigned index = 0; index < _USERMAKEFILE_LIB_RFM69_XC_PAYLOAD_LEN08; index++) {
                            // Take a copy of last received into "previous"
                            RX_context.RX_radio_payload_prev.u.payload_u1_uint8_arr[index] = RX_radio_payload.u.payload_u1_uint8_arr [index]; // Received last
                        }
                    } else {} // Don't restore or set to PACKET_INIT_VAL08, I get to many CHAR_CHANGE_STR ('#')

                    debug_print ("NOW: On:%3u%% @ Watt:%2u - Heater:%3d Ambient:%3d Water:%3d Mean:%3d Box:%3d\n",
                        RX_radio_payload.u.payload_u0.heater_on_percent,
                        RX_radio_payload.u.payload_u0.heater_on_watt,
                        RX_radio_payload.u.payload_u0.i2c_temp_heater_onetenthDegC,
                        RX_radio_payload.u.payload_u0.i2c_temp_ambient_onetenthDegC,
                        RX_radio_payload.u.payload_u0.i2c_temp_water_onetenthDegC,
                        RX_radio_payload.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC,
                        RX_radio_payload.u.payload_u0.internal_box_temp_onetenthDegC);
                    debug_print ("MAX: On:%3u%% @ Watt:%2u - Heater:%3d Ambient:%3d Water:%3d Mean:%3d Box:%3d\n",
                        RX_context.RX_radio_payload_max.u.payload_u0.heater_on_percent,
                        RX_context.RX_radio_payload_max.u.payload_u0.heater_on_watt,
                        RX_context.RX_radio_payload_max.u.payload_u0.i2c_temp_heater_onetenthDegC,
                        RX_context.RX_radio_payload_max.u.payload_u0.i2c_temp_ambient_onetenthDegC,
                        RX_context.RX_radio_payload_max.u.payload_u0.i2c_temp_water_onetenthDegC,
                        RX_context.RX_radio_payload_max.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC,
                        RX_context.RX_radio_payload_max.u.payload_u0.internal_box_temp_onetenthDegC);
                    debug_print ("MIN: On:%3u%% @ Watt:%2u - Heater:%3d Ambient:%3d Water:%3d Mean:%3d Box:%3d\n",
                        RX_context.RX_radio_payload_min.u.payload_u0.heater_on_percent,
                        RX_context.RX_radio_payload_min.u.payload_u0.heater_on_watt,
                        RX_context.RX_radio_payload_min.u.payload_u0.i2c_temp_heater_onetenthDegC,
                        RX_context.RX_radio_payload_min.u.payload_u0.i2c_temp_ambient_onetenthDegC,
                        RX_context.RX_radio_payload_min.u.payload_u0.i2c_temp_water_onetenthDegC,
                        RX_context.RX_radio_payload_min.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC,
                        RX_context.RX_radio_payload_min.u.payload_u0.internal_box_temp_onetenthDegC);
                } else {
                    debug_print ("Max %u %u \n", "IRQ but not receiveDone!");
                }
            } break;

            #if (TEST_01_FOLLOW_ADDRESS==1)
                case messageNotForThisNode_IRQ: {
                    #if (TEST_01_LISTENTOALL==1)
                        debug_print ("\nStarting to receive on any address, RX_context.num_totLost %u kept, RXappSeqCnt %u\n", RX_context.num_totLost, RX_PACKET_U.u.packet_u1.appSeqCnt);
                        i_radio.RX_context.doListenToAll (true);
                    #else
                        uint8_t previous_NODEID = i_radio.setNODEID (RXTX_context.some_rfm69_internals.TARGETID); // Follow who sender wants to send to
                        debug_print ("\nStarting (from #%03u) to receive on address #%03u, RX_context.num_totLost %u, RXappSeqCnt %u\n",
                                previous_NODEID,
                                RXTX_context.some_rfm69_internals.TARGETID,
                                RX_context.num_totLost,
                                RX_PACKET_U.u.packet_u3.appSeqCnt);
                    #endif
                } break;
            #endif

            case messageRadioCRC16Err_IRQ:
            case messageAppCRC32Err_IRQ:
            case messageRadioCRC16AppCRC32Errs_IRQ:{

                const bool CRC16err = (interruptAndParsingResult == messageRadioCRC16Err_IRQ);
                const bool CRC32err = (interruptAndParsingResult == messageAppCRC32Err_IRQ);
                const bool bothErr  = (interruptAndParsingResult == messageRadioCRC16AppCRC32Errs_IRQ);

                if (CRC16err) {RX_context.num_radioCRC16errs++;}
                if (CRC32err) {RX_context.num_appCRC32errs++;}
                if (bothErr)  {RX_context.num_radioCRC16errs++; RX_context.num_appCRC32errs++;}

                debug_print ("RSSI %d, CRC-fail@%u radioCRC16@%u %u, appCRC32@%u %u with PACKETLEN %u\n",
                        nowRSSI,
                        bothErr,
                        bothErr ? 1 : CRC16err,
                        RX_context.num_radioCRC16errs,
                        bothErr ? 1 : CRC32err,
                        RX_context.num_appCRC32errs,
                        RXTX_context.some_rfm69_internals.PACKETLEN);
            } break;

            case messagePacketSentOk_IRQ:
            case messagePacketSentOk2_IRQ:
            case messagePacketSentOkMissing_IRQ:
            case messagePacketSentOkMissing2_IRQ:
            {
                // No code. We haven't sent anything so we can't do anything
            } break;

            case void_IRQ:
            case messageNotExpectedBy_IRQ:
            case messagePacketLenErr_IRQ:
            #if (TEST_01_FOLLOW_ADDRESS==0)
                case messageNotForThisNode_IRQ:
            #endif

            default: // none left for default
            {
                debug_print ("RSSI %d, fail IRQ %u, RX_context.num_radioCRC16errs %u, RX_context.num_appCRC32errs %u with PACKETLEN %u\n",
                        nowRSSI, interruptAndParsingResult, RX_context.num_radioCRC16errs, RX_context.num_appCRC32errs, RXTX_context.some_rfm69_internals.PACKETLEN);

                // TODO
                // 30Aug2018 hang after this. 29Oct2018 now called RFM69=001
                // RSSI -48, fail IRQ 7, RX_context.num_radioCRC16errs 0, RX_context.num_appCRC32errs 0 with PACKETLEN 1
                //                    7 messagePacketLenErr_IRQ
                // 12Sept2018 same hanging, see file "2018 09 12 A fail IRQ 7 still not solved.txt"
                // RSSI -44, fail IRQ 7, RX_context.num_radioCRC16errs 0, RX_context.num_appCRC32errs 0 with PACKETLEN 1
            } break;

        #elif (IS_MYTARGET_MASTER == 1)
            case messageReceivedOk_IRQ:
            case messagePacketLenErr_IRQ:
            case messageNotForThisNode_IRQ:
            case messageRadioCRC16Err_IRQ:
            case messageAppCRC32Err_IRQ:
            case messageRadioCRC16AppCRC32Errs_IRQ:
            {
                // Just let it pass, not interesting, at least not if only sending
            } break;

            case messagePacketSentOk_IRQ:
            {
                if (TX_context.waitForIRQInterruptCause == messagePacketSent_IRQExpected) {
                    TX_context.waitForIRQInterruptCause = no_IRQExpected; // Ending normal send with this IRQ
                } else {}
            } break;

            case void_IRQ:
            case messagePacketSentOk2_IRQ:
            case messagePacketSentOkMissing_IRQ:
            case messagePacketSentOkMissing2_IRQ:
            case messageNotExpectedBy_IRQ:
            default: // none left for default
            {
                debug_print ("fail IRQ %u\n", interruptAndParsingResult);
            } break;
        #endif
    }

    {RXTX_context.some_rfm69_internals.error_bits, RXTX_context.is_new_error} = i_radio.getAndClearErrorBits();
    if (RXTX_context.some_rfm69_internals.error_bits != ERROR_BITS_NONE) {
        debug_print ("RFM69 err2 new %u code %04X\n", RXTX_context.is_new_error, RXTX_context.some_rfm69_internals.error_bits);
    } else {}

    i_radio.do_spi_aux_pin (MASKOF_SPI_AUX0_PROBE3_IRQ, low); // For scope
}

void RFM69_handle_timeout (
        diffTime_t                       &diffTime,
        const time32_t                   startTime_ticks,
        RX_context_t                     &?RX_context,
        TX_context_t                     &?TX_context,
        RXTX_context_t                   &RXTX_context,
        display_context_t                &display_context,
        client  radio_if_t               i_radio,
        client  blink_and_watchdog_if_t  i_blink_and_watchdog,
        const   bool                     semantics_do_rssi_in_irq_detect_task,
        client  i2c_internal_commands_if i_i2c_internal_commands)
{

    // i_blink_and_watchdog.blink_pulse_ok (XCORE_200_EXPLORER_LED_GREEN_BIT_MASK, 50);

    #if (DEBUG_PRINT_TIME_USED == 1)
        debug_print ("..After %08X is time %08X ticks\n", time_ticks, startTime_ticks);
    #endif

    #if (IS_MYTARGET_SLAVE == 1)
        RX_context.seconds_since_last_received++; // about, anyhow, since we don't reset time_ticks in pin_rising
        //
        if (RX_context.seconds_since_last_received > ((AQUARIUM_RFM69_REPEAT_SEND_EVERY_SEC * 5)/2)) { // 2.5 times 4 seconds
            i_radio.receiveDone();
            RX_context.seconds_since_last_received = 0;
        } else {}

        if (i_blink_and_watchdog.is_watchdog_blinking()) {
            debug_print ("WATCHDOG BLINKING T %u ", RX_context.seconds_since_last_received); // no nl, added below
        } else {
            debug_print ("T %u ", RX_context.seconds_since_last_received);  // no nl, added below
        }

        #if (_USERMAKEFILE_LIB_RFM69_XC_GETDEBUG==1)
        {
            uint8_t debug_data[NUM_DEBUG_BYTES];
            bool    are_equal;

            i_radio.getDebug (debug_data);

            are_equal = (memcmp (RX_context.debug_data_prev, debug_data, NUM_DEBUG_BYTES) == 0);

            for (unsigned i = 0; i < NUM_DEBUG_BYTES; i++) {
                RX_context.debug_data_prev[i] = debug_data[i];
            }

            debug_print ("DEB%s", are_equal ? CHAR_EQ_STR : CHAR_CHANGE_STR);

            for (unsigned i = 0; i < NUM_DEBUG_BYTES; i++) {
                debug_print ("%02X%s", debug_data[i], ((i == (NUM_DEBUG_BYTES-1)) ? "\n" : " "));
            }
        }
        #else
            debug_print ("%s", "\n"); // Add missing nl from above
        #endif

    #elif (IS_MYTARGET_MASTER == 1)
        if (TX_context.sendPacket_seconds_cntdown == 0) {
            const char char_leading_space_str[] = "       ";
            TX_context.TX_appSeqCnt++;

            if (TX_context.waitForIRQInterruptCause != no_IRQExpected) {
                // Normal send failed: no messagePacketSentOk_IRQ seen
                #if (SEMANTICS_DO_LOOP_FOR_RF_IRQFLAGS2_PACKETSENT == 0)
                    debug_print ("fail IRQ waitForIRQInterruptCause %u\n", TX_context.waitForIRQInterruptCause); // Report and continue
                    TX_context.waitForIRQInterruptCause = no_IRQExpected; // Clear it here even if i_radio.send will overwrite it
                #endif
            } else {} // No code

            if (TX_context.TX_appSeqCnt == 10) {
                #if (TEST_CAR_KEY == 1)
                    // No code, kep full power always
                #else
                TX_context.TX_appPowerLevel_dBm = APPPOWERLEVEL_MIN_DBM;
                    i_radio.setPowerLevel_dBm (TX_context.TX_appPowerLevel_dBm); // Should stop the sound in my speakers!
                #endif
            } else {}

            #if ((TEST_01_FOLLOW_ADDRESS==1) or (TEST_01_LISTENTOALL==1))
                if (TX_context.TX_appSeqCnt == 10) {
                    debug_print ("\nKEY2\n", TX_context.TX_gatewayid);
                    debug_print ("%s", char_leading_space_str);
                    #define KEY2 "OM11-Aquarium-2"
                    i_radio.encrypt16 (KEY2, KEY_LEN);
                } else if (TX_context.TX_appSeqCnt == 20) {
                    debug_print ("\nKEY again\n", "KEY again");
                    debug_print ("%s", char_leading_space_str);
                    i_radio.encrypt16 (RXTX_context.radio_init.key, KEY_LEN);
                } else if (TX_context.TX_appSeqCnt == 25) {
                    TX_context.TX_gatewayid = 58;
                    debug_print ("\ngatewayid %u(%02X)\n", TX_context.TX_gatewayid, TX_context.TX_gatewayid);
                    debug_print ("%s", char_leading_space_str);
                } else if (TX_context.TX_appSeqCnt == 26) {
                    debug_print ("%s\n", "=== ZEROED: Time max 0, mean-1");
                    debug_print ("%s", char_leading_space_str);
                    diffTime.max_diffTime_ms  = 0;
                    diffTime.mean_diffTime_ms = 0;
                    #if (RADIO_IF_FULL == 1)
                        // i_radio.readAllRegs();
                    #endif
                } else {}
            #endif

            for (unsigned index = 0; index < PACKET_LEN32; index++) {
                 TX_PACKET_U.u.packet_u2_uint32_arr[index] = PACKET_INIT_VAL32;
            }

            TX_PACKET_U.u.packet_u3.appHeading.numbytes_of_full_payload = PACKET_LEN08;
            TX_PACKET_U.u.packet_u3.appHeading.version_of_full_payload  = VERSION_OF_APP_PAYLOAD_01;
            TX_PACKET_U.u.packet_u3.appHeading.num_of_this_app_payload  = NUM_OF_THIS_APP_PAYLOAD_01;

            TX_PACKET_U.u.packet_u3.appNODEID = NODEID;
            TX_PACKET_U.u.packet_u3.appPowerLevel_dBm = TX_context.TX_appPowerLevel_dBm;

            TX_PACKET_U.u.packet_u3.appSeqCnt = TX_context.TX_appSeqCnt;

            debug_print("TXappSeqCnt %u\n", TX_context.TX_appSeqCnt);

            TX_context.waitForIRQInterruptCause = i_radio.send (
                    TX_context.TX_gatewayid,
                    TX_PACKET_U); // element CommHeaderRFM69 is not taken from here, so don't fill it in

            // delay_milliseconds(500); // I can hear the sending in my speakers when high power since delays time to IRQ

            {RXTX_context.some_rfm69_internals.error_bits, RXTX_context.is_new_error} = i_radio.getAndClearErrorBits();
            if (RXTX_context.some_rfm69_internals.error_bits != ERROR_BITS_NONE) {
                debug_print ("RFM69 err3 new %u code %04X\n", RXTX_context.is_new_error, RXTX_context.some_rfm69_internals.error_bits);
            } else {}

            TX_context.sendPacket_seconds_cntdown = SEND_PACKET_ON_NO_CHANGE_TIMOEUT_SECONDS - 1;

        } else {
            TX_context.sendPacket_seconds_cntdown--; // To zero
        }

        {
            time32_t endTime_tics;
            time32_t diffTime_tics;
            time32_t diffTime_ms;

            timer tmr_now; // Not need to parameterise this as it doesn't take an extra hw timer
            tmr_now :> endTime_tics; // NOW TIME

            diffTime_tics = endTime_tics - startTime_ticks;
            diffTime_ms   = diffTime_tics / XS1_TIMER_KHZ;

            diffTime.max_diffTime_ms = max (diffTime.max_diffTime_ms, diffTime_ms);
            diffTime.num_diffTime_ms++;

            // IIR very slow moving average::
            diffTime.sum_diffTime_ms          = diffTime.sum_diffTime_ms + diffTime_ms; // running total or partial sum
            diffTime.mean_diffTime_ms         = diffTime.sum_diffTime_ms / diffTime.num_diffTime_ms;
            diffTime.changed_mean_diffTime_ms = (diffTime.prev_mean_diffTime_ms != diffTime.mean_diffTime_ms);
            diffTime.prev_mean_diffTime_ms    = diffTime.mean_diffTime_ms;

            #if (DEBUG_PRINT_TIME_USED == 1)
                // I can hear sending in my speakers when full power when this printing is going on!
                // Probably because this delays IRQ handling!
                debug_print ("Time used %06d ms (%08X - %08X). Time max %d, mean-%u %d ms\n",
                        diffTime_ms, endTime_tics, startTime_ticks,
                        diffTime.max_diffTime_ms,            // 345
                        diffTime.changed_mean_diffTime_ms,
                        diffTime.mean_diffTime_ms);          // 255
            #endif
        }
    #endif
}

[[combinable]] // Cannot be [[distributable]] since timer case in select
void RFM69_client (
          server  irq_if_t                 i_irq,
          client  radio_if_t               i_radio,
          client  blink_and_watchdog_if_t  i_blink_and_watchdog,
          const   bool                     semantics_do_rssi_in_irq_detect_task,
          server  button_if                i_button_in[BUTTONS_NUM_CLIENTS],
          client  i2c_internal_commands_if i_i2c_internal_commands,
          out port                         p_display_notReset)
{
    timer    tmr;
    time32_t time_ticks;

    // All declared, used or not, since nullable references not allowed for structs

    RXTX_context_t    RXTX_context;
    display_context_t display_context;

    RXTX_context.interruptCnt = 0;
    RXTX_context.radio_init.nodeID    = NODEID;
    RXTX_context.radio_init.RegFrf    = MY_RFM69_FREQ_REGS;
    RXTX_context.radio_init.isRFM69HW = IS_RFM69HW_HCW; // Must be true or else my Adafruit high power module won't work!
    //
    for (unsigned i=0; i < KEY_LEN; i++) {
        RXTX_context.radio_init.key[i] = KEY[i];
    }

    #if (IS_MYTARGET_MASTER==1)
        TX_context_t       TX_context;
        #define TX_CONTEXT TX_context
        #define RX_CONTEXT null
        //
        TX_context.TX_appPowerLevel_dBm = APPPPOWERLEVEL_MAX_DBM;
        TX_context.TX_gatewayid = GATEWAYID;
        TX_context.TX_appSeqCnt = 0;
        TX_context.sendPacket_seconds_cntdown = 0; // Down from SEND_PACKET_ON_NO_CHANGE_TIMOEUT_SECONDS
        TX_context.waitForIRQInterruptCause = no_IRQExpected;

    #elif (IS_MYTARGET_SLAVE==1)
        RX_context_t       RX_context;
        #define RX_CONTEXT RX_context
        #define TX_CONTEXT null

        RX_context.doListenToAll = false; // Set to 'true' to sniff all packets on the same network
        RX_context.num_totLost = 0;
        RX_context.num_radioCRC16errs = 0;
        RX_context.num_appCRC32errs = 0;
        RX_context.seconds_since_last_received = 0;
        RX_context.first_debug_print_received_done = false;
        RX_context.lastReceivedAppSeqCnt = 0;
        #if (_USERMAKEFILE_LIB_RFM69_XC_GETDEBUG==1)
            for (unsigned i = 0; i < NUM_DEBUG_BYTES; i++) {
                RX_context.debug_data_prev[i] = 0;
            }
        #endif
    #else
        #error MUST BE ONE of them! To code for both, recode somewhat
    #endif

    diffTime_t diffTime;
    //
    diffTime.max_diffTime_ms = 0;
    diffTime.sum_diffTime_ms = 0;
    diffTime.mean_diffTime_ms = 0;
    diffTime.prev_mean_diffTime_ms = 0;
    diffTime.changed_mean_diffTime_ms = false;
    diffTime.num_diffTime_ms = 0;

    #if ((PACKET_LEN_FACIT % 4) != 0)
        #error sizeof packet_u1_t must be word aligned (12, 16, 20 ...)
    #endif

    if (PACKET_LEN08 > MAX_SX1231H_PACKET_LEN) {
        fail ("SX1231H LIMIT"); // Stops the code also when flashed, but there's no restart
    } else if (PACKET_LEN08 != PACKET_LEN_FACIT) {
        fail ("PACKET_LEN_FACIT"); // Stops the code also when flashed, but there's no restart
    } else {
        debug_print ("packet_t %u bytes\n", PACKET_LEN08);
    }

    // One example, opposite from aquarium:
    // startKIT as SLAVE[98]: RFM69-driver[0.5.2], RFM69-client[0.5.2]
    // xplorKIT as MASTER[99] sendsto [98] every 10000 ms: RFM69-driver[0.5.2], RFM69-client[0.5.2]
    //
    #if (IS_MYTARGET_MASTER==1)
        debug_print ("\n%s as MASTER[%u] sendsto [%d] every %u sec: RFM69-driver[%s], RFM69-client[%s]\n",
                (IS_MYTARGET == IS_MYTARGET_STARTKIT)           ? "startKIT"              :
                (IS_MYTARGET == IS_MYTARGET_XCORE_200_EXPLORER) ? "xplorKIT" : "none!",
                NODEID,
                GATEWAYID,
                SEND_PACKET_ON_NO_CHANGE_TIMOEUT_SECONDS,
                RFM69_DRIVER_VERSION_STR,
                RFM69_CLIENT_VERSION_STR);
    #elif (IS_MYTARGET_SLAVE==1)
        debug_print ("\n%s as SLAVE[%u]: RFM69-driver[%s], RFM69-client[%s]\n",
                (IS_MYTARGET == IS_MYTARGET_STARTKIT)           ? "startKIT"              :
                (IS_MYTARGET == IS_MYTARGET_XCORE_200_EXPLORER) ? "xplorKIT" : "none!",
                NODEID,
                RFM69_DRIVER_VERSION_STR,
                RFM69_CLIENT_VERSION_STR);
    #endif

    debug_print ("Built %s [%s] with read RSSI %s and radio CRC %s IRQ and %s sent\n\n",
            __DATE__,
            __TIME__,
            (semantics_do_rssi_in_irq_detect_task) ? "in IRQ_detect_task" : "by RFM69_client",
            (SEMANTICS_DO_CRC_ERR_NO_IRQ == 1) ? "no" : "with",
            (SEMANTICS_DO_LOOP_FOR_RF_IRQFLAGS2_PACKETSENT == 1) ? "loop for" : "state for");

    // Display matters
    {
        Adafruit_GFX_constructor (SSD1306_LCDWIDTH, SSD1306_LCDHEIGHT);
        Adafruit_SSD1306_i2c_begin (i_i2c_internal_commands, p_display_notReset);

        Clear_All_Pixels_In_Buffer();
        writeToDisplay_i2c_all_buffer(i_i2c_internal_commands);

        for (int index_of_char = 0; index_of_char < NUM_ELEMENTS(display_context.display_ts1_chars); index_of_char++) {
            display_context.display_ts1_chars [index_of_char] = ' ';
        }

        display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars,
                "\n Ver %s rx data\n fra akvariet hvert\n %u sek..", RFM69_CLIENT_VERSION_STR, AQUARIUM_RFM69_REPEAT_SEND_EVERY_SEC);
        setTextSize(1);
        setTextColor(WHITE);
        setCursor(0,0);
        display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL
        writeToDisplay_i2c_all_buffer(i_i2c_internal_commands);
        delay_milliseconds (4000);
    }

    // Radio matters

    i_radio.do_spi_aux_adafruit_rfm69hcw_RST_pulse (MASKOF_SPI_AUX0_RST);
    i_radio.initialize (RXTX_context.radio_init);

    RXTX_context.device_type = i_radio.getDeviceType(); // ERROR_BITNUM_DEVICE_TYPE if not 0x24
    debug_print ("\n---> DEVICE TYPE 0x%02X <---\n\n", RXTX_context.device_type);

    {RXTX_context.some_rfm69_internals.error_bits, RXTX_context.is_new_error} = i_radio.getAndClearErrorBits();

    if (RXTX_context.some_rfm69_internals.error_bits == ERROR_BITS_NONE) {

        i_radio.setHighPower (RXTX_context.radio_init.isRFM69HW);
        i_radio.encrypt16 (RXTX_context.radio_init.key, KEY_LEN);
        #if (IS_MYTARGET_SLAVE==1)
            i_radio.setListenToAll (RX_context.doListenToAll);
        #endif
        i_radio.setFrequencyRegister (MY_RFM69_FREQ_REGS); // Only needed if different from MY_RFM69_FREQ_REGS, since that done in radio_init

        #if (DEBUG_PRINT_RFM69 == 1)
            #define TEST_FLOAT_FREQ 0 // =1 would cause the linker to kick in the floating point library!
            //
            #if (TEST_FLOAT_FREQ == 1)
                // Code really "for fun" (to see how XC handles float). A calculator using RF69_FSTEP_FLOAT32 = 61.03515625 and not
                // float (or even double) value of 61.035156 is needed to get MY_RFM69_FREQ_REGS right
                #if (WARNINGS == 1)
                    #warning COMPILING TO CALC FREQ WITH FLOAT (FOR FUN)
                #endif
                {
                    unsigned frequencyHz    = freq_register_value_to_Hz (MY_RFM69_FREQ_REGS);
                    uint32_t register_value = freq_Hz_to_register_value (frequencyHz); // Just for comparing and printing
                    debug_print ("TX/RX at %u/%u Hz with reg %06X/%06X\n", frequencyHz, MY_RFM69_FREQ_HZ, MY_RFM69_FREQ_REGS, register_value);
                }
            #else
                {
                    uint32_t frequencyHz = MY_RFM69_FREQ_HZ;
                    debug_print ("TX/RX at %u Hz with reg %06X\n", frequencyHz, MY_RFM69_FREQ_REGS);
                }
            #endif
        #endif

        i_radio.receiveDone(); // To have setMode(RF69_MODE_RX) done (via receiveBegin)
    } else {}

    {RXTX_context.some_rfm69_internals.error_bits, RXTX_context.is_new_error} = i_radio.getAndClearErrorBits();

    if (RXTX_context.some_rfm69_internals.error_bits != ERROR_BITS_NONE) {
        debug_print ("RFM69 err1 new %u code %04X\n", RXTX_context.is_new_error, RXTX_context.some_rfm69_internals.error_bits);
    } else {}

    #if (IS_MYTARGET_SLAVE==1)
        for (unsigned index = 0; index < _USERMAKEFILE_LIB_RFM69_XC_PAYLOAD_LEN08; index++) {
            RX_context.RX_radio_payload_prev.u.payload_u1_uint8_arr[index] = PACKET_INIT_VAL08;
        }

        RX_context.RX_radio_payload_max.u.payload_u0.heater_on_percent                        = HEATER_ON_PERCENT_R_MIN;
        RX_context.RX_radio_payload_max.u.payload_u0.heater_on_watt                           = HEATER_ON_WATT_R_MIN;
        RX_context.RX_radio_payload_max.u.payload_u0.i2c_temp_heater_onetenthDegC             = ONETENTHDEGC_R_MIN;
        RX_context.RX_radio_payload_max.u.payload_u0.i2c_temp_ambient_onetenthDegC            = ONETENTHDEGC_R_MIN;
        RX_context.RX_radio_payload_max.u.payload_u0.i2c_temp_water_onetenthDegC              = ONETENTHDEGC_R_MIN;
        RX_context.RX_radio_payload_max.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC = ONETENTHDEGC_R_MIN;
        RX_context.RX_radio_payload_max.u.payload_u0.internal_box_temp_onetenthDegC           = ONETENTHDEGC_R_MIN;

        RX_context.RX_radio_payload_min.u.payload_u0.heater_on_percent                        = HEATER_ON_PERCENT_R_MAX;
        RX_context.RX_radio_payload_min.u.payload_u0.heater_on_watt                           = HEATER_ON_WATT_R_MAX;
        RX_context.RX_radio_payload_min.u.payload_u0.i2c_temp_heater_onetenthDegC             = ONETENTHDEGC_R_MAX;
        RX_context.RX_radio_payload_min.u.payload_u0.i2c_temp_ambient_onetenthDegC            = ONETENTHDEGC_R_MAX;
        RX_context.RX_radio_payload_min.u.payload_u0.i2c_temp_water_onetenthDegC              = ONETENTHDEGC_R_MAX;
        RX_context.RX_radio_payload_min.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC = ONETENTHDEGC_R_MAX;
        RX_context.RX_radio_payload_min.u.payload_u0.internal_box_temp_onetenthDegC           = ONETENTHDEGC_R_MAX;
    #endif

    i_blink_and_watchdog.enable_watchdog_ok (
            XCORE_200_EXPLORER_LED_GREEN_BIT_MASK bitor XCORE_200_EXPLORER_LED_RGB_GREEN_BIT_MASK,
            ((AQUARIUM_RFM69_REPEAT_SEND_EVERY_SEC*5)/2) * 1000, // 10 seconds. May lose two ok. Max 21 secs
            200);

    tmr :> time_ticks; // First sending now

    while (1) {
        select {
            case i_irq.pin_rising (const int16_t value) : { // PROTOCOL: int16_t chan_value

                RXTX_context.irq_value = value;

                RFM69_handle_irq (
                        RX_CONTEXT,
                        TX_CONTEXT,
                        RXTX_context,
                        display_context,
                        i_radio,
                        i_blink_and_watchdog,
                        semantics_do_rssi_in_irq_detect_task,
                        i_i2c_internal_commands);

            } break;

            case tmr when timerafter (time_ticks) :> time32_t startTime_ticks: {

                RFM69_handle_timeout (
                        diffTime,
                        startTime_ticks,
                        RX_CONTEXT,
                        TX_CONTEXT,
                        RXTX_context,
                        display_context,
                        i_radio,
                        i_blink_and_watchdog,
                        semantics_do_rssi_in_irq_detect_task,
                        i_i2c_internal_commands);

                #if (TEST_CAR_KEY == 1)
                    time_ticks += ONE_SECOND_TICKS/5; // Every 200 ms will destroy for car's requirement of seein the pulse train for 500 ms
                #else
                    time_ticks += ONE_SECOND_TICKS; // FUTURE TIMEOUT
                    // observe AQUARIUM_RFM69_REPEAT_SEND_EVERY_SEC
                #endif

                // If diffTime_ms is larger than ONE_SEC_TICS the select will be timed out immediately N times until it's AFTER again

            } break;

            case i_button_in[int iof_button].button (const button_action_t button_action) : {
                switch (iof_button) {
                    case IOF_BUTTON_LEFT: {
                        i_blink_and_watchdog.reset_watchdog_ok();
                    } break;
                    case IOF_BUTTON_CENTER: {
                        //
                    } break;
                    case IOF_BUTTON_RIGHT: {
                        //
                    } break;
                }
            } break;
        }
    }
}

