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

#include "_version.h" // First this..
#include "_globals.h" // ..then this
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

#include "f_conversions.h"
#include "_rfm69_commprot.h"
#include "button_press.h"

#include "_Aquarium_rfm69_client.h"
#endif

#define DEBUG_PRINT_RFM69 1
#define debug_print(fmt, ...) do { if((DEBUG_PRINT_RFM69==1) and (DEBUG_PRINT_GLOBAL_APP==1)) printf(fmt, __VA_ARGS__); } while (0)

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

#ifdef _USERMAKEFILE_LIB_RFM69_XC_SEMANTICS_DO_INTERMEDIATE_RECEIVEDONE
    #define SEMANTICS_DO_INTERMEDIATE_RECEIVEDONE _USERMAKEFILE_LIB_RFM69_XC_SEMANTICS_DO_INTERMEDIATE_RECEIVEDONE
#else
    #define SEMANTICS_DO_INTERMEDIATE_RECEIVEDONE 0
#endif

#define SEMANTICS_DEBUG_CALC_RF_FRF_REGISTERS 1 // 1 : never while real TX->RX!. Also set ALLOW_FLOAT_FREQ_CALC 1 for calculations
                                                // 0 : Standard usage

// 433 MHz band shared with our
//    (1) indoor/outdoor thermometer and ("433")
//    (2) kitchen light switch over the table and ("433.92")
//    (3) car key ("433.92") - so it's a good idea to avoid 433.92 MHz at least
//        For the CAR_KEY case (code removed with 0858)
//            #define MY_RFM69_FREQ_HZ     RF69_INIT_FREQUENCY_433919921_HZ // 433.92 MHz same as my car key
//            #define MY_RFM69_FREQ_REGS            RF_FRF_433_433919921    // Is 0x006C7AE0
//            // See http://www.teigfam.net/oyvind/home/technology/164-my-aquariums-data-radioed-through-the-shelf/#frequency_crash_with_my_car_key
//            // and https://forum.allaboutcircuits.com/threads/radio-board-interferes-with-my-car-key-system.151059/
//            //   The hypothesis there seems to be that my key must send a unique on/off pattern (one for lock and one for unlock),
//            //   typically modulated at 10 MHz with a bit rate of 40 kbps, repeated every 20 ms for 500 ms before the receiver in the car
//            //   will accept it. There probably is no packetisation or cryptographic key, just that repeated stream of bits.
//            //   So when I sent my packets every 200 ms I destroyed that data stream. This seems very plausible.
//            //   Thanks to sghioto and DickCappels! See Wikipedia article On-off keying (https://en.wikipedia.org/wiki/On-off_keying) and
//            //   Application note 4439 from Maxim integrated at I’m OOK. You’re OOK? (https://www.maximintegrated.com/en/app-notes/index.mvp/id/4439)


#define MY_RFM69_FREQ_HZ   RF69_INIT_FREQUENCY_433705993_HZ // Frequency is according to calculator 0x006C6D2F * (RF69_FSTEP_FLOAT32 as 61.03515625)
                                                            // Only needed for printouts and to check calculator-accurate calculation of these hex values:
#define MY_RFM69_FREQ_REGS          RF_FRF_433_433705993    // Is 0x006C6D2F

//      SPI_AUX bits:
#define MASKOF_SPI_AUX0_RST        0x01 // RST is port SPI_AUX BIT0. Search for SPI_AUX0_RST to see HW-defined timing
#define MASKOF_SPI_AUX0_PROBE3_IRQ 0x02 // Test pin for IRQ. "LED1"

//           ##                Same number then related
#define TEST_01_FOLLOW_ADDRESS 1 // Testet OK 04Apr2018
#define TEST_01_LISTENTOALL    0 // Tested OK 04Apr2018

#define ONE_SECOND_TICKS                        (1000 * XS1_TIMER_KHZ) // Expands to clock frequency = 100 mill = 100 * 1000 * 1000
#define SEND_PACKET_ON_NO_CHANGE_TIMOEUT_SECONDS 4                     // MINIMUM 1! For sendPacket_seconds_cntdown

#define KEY            MY_KEY // From *_commprot.h TODO move it into aquarium _rfm69_commprot.h away from the library files
#define IS_RFM69HW_HCW true   // 1 for Adafruit RFM69HCW (high power)

#define CHAR_EQ_STR     "="
#define CHAR_CHANGE_STR "#"

typedef struct {
    time32_t   time_ticks;
    time32_t   max_diffTime_ms;
    time32_t   sum_diffTime_ms;
    time32_t   mean_diffTime_ms;
    time32_t   prev_mean_diffTime_ms;
    bool       changed_mean_diffTime_ms;
    unsigned   num_diffTime_ms;
    //
} divTime_t;

typedef struct {
    rfm69_params_t         radio_init;
    int16_t                irq_value;
    uint8_t                device_type;
    bool                   receiveDone;
    is_error_e             is_new_error;
    uint32_t               interruptCnt;
    some_rfm69_internals_t some_rfm69_internals;
    packet_t               PACKET;
    error_bits_e           error_bits_history;
    #if (CLIENT_ALLOW_SESSION_TYPE_TRANS==1)
        timing_transx_t    timing_transx;
        return_trans3_t    return_trans3;
        unsigned           radio_log_value;
    #endif
} RXTX_context_t;

typedef struct {
    bool      doListenToAll;
    unsigned  num_received;
    int32_t   num_lost_since_last_success; // May be negative if sender restarts
    unsigned  num_appSeqCnt_notSeen; // Earlier num_toLost but they are only counted when we see a message again
    unsigned  num_appSeqCnt_notSeen_inDisplay;
    unsigned  num_radioCRC16errs;
    unsigned  num_appCRC32errs;
    unsigned  seconds_since_last_received;
    uint32_t  appSeqCnt;
    uint32_t  appSeqCnt_prev;
    payload_t RX_radio_payload_prev;
    payload_t RX_radio_payload;
    payload_t RX_radio_payload_max;
    payload_t RX_radio_payload_min;
    int16_t   nowRSSI;           // -80  dB
    int16_t   nowRSSI_weakest;   // -100 dB
    int16_t   nowRSSI_strongest; // -60  dB
    bool      allow_10_sek_timeout; // AQUARIUM_RFM69_RECEIVE_TIMOUT_SEC or none
    bool      is_watchdog_blinking;

    #if (GET_RADIO_DEBUG_REGS==1)
        debug_data_t  debug_data;
        debug_data_t  debug_data_prev;
        debug_state_e debug_state;
    #endif
} RX_context_t; // RX same as SLAVE same as ISMASTER==0

#define AQUARIUM_RFM69_RECEIVE_TIMOUT_SEC ((AQUARIUM_RFM69_REPEAT_SEND_EVERY_SEC * 5)/2) // 10 or 4 times 2.5 (test with 3 secs)

typedef enum {USE_NONE, USE_THIS, USE_PREV} use_t;

typedef struct {
    uint8_t                    TX_appPowerLevel_dBm;
    uint8_t                    TX_gatewayid;
    uint32_t                   TX_appSeqCnt;
    unsigned                   sendPacket_seconds_cntdown; // Down from SEND_PACKET_ON_NO_CHANGE_TIMOEUT_SECONDS
    waitForIRQInterruptCause_e waitForIRQInterruptCause;
    //
} TX_context_t; // TX same as MASTER same as ISMASTER==1

typedef enum display_screen_name_t {
    // English-Norwegian here because the screens are in Norwegian
    // Sequence defines NEXT with IOF_BUTTON_CENTER:
    SCREEN_RX_MAIN_TIME_TEMP_ETC, // Keep as first  since   0 display_screen_name_str "1 " is not shown (allow_auto_switch_to_screen_rx_main_time_temp_etc)
    SCREEN_STATISTICS_DB_ETC,     // Keep as second since   1 display_screen_name_str "2 " is not shown
    SCREEN_STATISTICS_OBS,                              //  2 display_screen_name_str "3 "
    SCREEN_TEMPS_ETC,                                   //  3 display_screen_name_str "4 "
    SCREEN_WATT_ETC,                                    //  4 display_screen_name_str "5 "
    SCREEN_LIGHT,                                       //  5 display_screen_name_str "6 "
    SCREEN_TX_SEQ_CNT,                                  //  6 display_screen_name_str "7 "
    SCREEN_AQUARIUM_BOX_INTERNALS,                          //  7 display_screen_name_str "8 "
    SCREEN_AQUARIUM_ERROR_BITS,                         //  8 display_screen_name_str "9 "
    SCREEN_WELCOME,                                     //  9 display_screen_name_str "10"
    SCREEN_HJELP,                                       // 10 display_screen_name_str "11"
    SCREEN_RX_DISPLAY_OVERSIKT,                         // 11 display_screen_name_str "12"
    SCREEN_RADIO_DEBUG_TRANS,                           // 12 display_screen_name_str "13"
    #if (GET_RADIO_DEBUG_REGS==1)
        SCREEN_RADIO_DEBUG_REGS,                        // 13 display_screen_name_str "14"
    #endif
    SCREEN_DARK // Must be last
} display_screen_name_t;

typedef enum {is_on, is_off} display_state_e;

typedef struct {
    display_state_e       state;
    char                  display_ts1_chars [SSD1306_TS1_DISPLAY_VISIBLE_CHAR_LEN]; // 84 chars for display needs 85 char buffer (with NUL) when sprintf is use (use SSD1306_TS1_DISPLAY_ALL_CHAR_LEN for full flexibility)
    int                   sprintf_numchars;
    display_screen_name_t display_screen_name;
    display_screen_name_t display_screen_name_last_on;
    bool                  allow_auto_switch_to_screen_rx_main_time_temp_etc; // SCREEN_RX_MAIN_TIME_TEMP_ETC
    bool                  display_screen_direction_up;
    bool                  debug_r_button;
    button_state_t        buttons_state [BUTTONS_NUM_CLIENTS];
    unsigned              ultimateIRQclearCnt;
    unsigned              ultimateIRQclearCnt_notSeen_inDisplay;
    unsigned              senderid_not_displayed_cnt;  // New RFM69=005. Typically if received from MASTER_ID_BLACK_BOX
    uint8_t               senderid_not_displayed_now;  // New RFM69=006
    uint8_t               senderid_displayed_now;      // New RFM69=006
} display_context_t;

typedef enum {
    DEBUG_PRINT_RX_1_SEQCNT_ETC,
    DEBUG_PRINT_RX_2_TEMPS_ETC,
    DEBUG_PRINT_RX_2_NOW_MAX_MIN
} debug_print_state_e;

typedef struct {
    bool debug_print_rx_2_done; // true when DEBUG_PRINT_RX_2_TEMPS_ETC done once
} debug_print_context_t;


// MUST NOT MODIFY ANY STATE VALUES!
bool // i2c_ok
    Display_screen (
        display_context_t                 &display_context,
                RX_context_t              &?RX_context,  // #if (IS_MYTARGET_SLAVE == 1)
                RXTX_context_t            &RXTX_context,
        const   use_t                     use,
        client  i2c_internal_commands_if  i_i2c_internal_commands) {

    bool i2c_ok = true;

    if (display_context.state == is_on) {

        const char char_aa_str         [] = CHAR_AA_STR;          // Å
        const char char_triple_bar_str [] = CHAR_TRIPLE_BAR_STR;  // ≡
        const char char_right_arrow_str[] = CHAR_RIGHT_ARROW_STR; // →
        const char char_up_arrow_str   [] = CHAR_UP_ARROW_STR;    // ↑
        const char char_down_arrow_str [] = CHAR_DOWN_ARROW_STR;  // ↓
        const char char_degC_circle_str[] = DEGC_CIRCLE_STR;      // °

        #define SCREEN_NUMBER_WIDTH 2
        char display_screen_name_str [SCREEN_NUMBER_WIDTH+1];
        u_to_str_lm (display_context.display_screen_name + 1, display_screen_name_str, sizeof display_screen_name_str); // Starts at "1 "

        #if (IS_MYTARGET_SLAVE == 1)
            const bool alive = (RX_context.appSeqCnt % 2) == 0;
        #else
            const bool alive = false;
        #endif

        Clear_All_Pixels_In_Buffer();

        for (int index_of_char = 0; index_of_char < NUM_ELEMENTS(display_context.display_ts1_chars); index_of_char++) {
            display_context.display_ts1_chars [index_of_char] = ' ';
        }

        debug_print ("display %u\n", display_context.display_screen_name);

        setTextColor(WHITE);
        setCursor(0,0);
        setTextSize(1);

        switch (display_context.display_screen_name) {

            case SCREEN_RX_MAIN_TIME_TEMP_ETC: {
                #if (IS_MYTARGET_SLAVE == 1)
                    if (!isnull(RX_context.RX_radio_payload)) {

                        // display_screen_name_str "1 " is not shown

                        // ##########. Most setTextSize(2)
                        // 12:43:04 3
                        // 25.3°C 4W

                        display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars, "%02u:%02u:%02u",
                                RX_context.RX_radio_payload.u.payload_u0.hour,
                                RX_context.RX_radio_payload.u.payload_u0.minute,
                                RX_context.RX_radio_payload.u.payload_u0.second);

                        setTextSize(2);
                        display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL

                        // num_days_since_start also shown (calculated) in  SCREEN_TX_SEQ_CNT

                        display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars, "%s%u",
                                RX_context.RX_radio_payload.u.payload_u0.num_days_since_start <= 999 ? " " : "", // To get it on that line
                                RX_context.RX_radio_payload.u.payload_u0.num_days_since_start);

                        setTextSize(1);
                        display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL

                        display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars, "\n");

                        setTextSize(2);
                        display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL

                        {
                            const dp1_t dp1 = Parse_i16_dp1 (RX_context.RX_radio_payload.u.payload_u0.i2c_temp_water_onetenthDegC);
                            display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars, "%u.%u%sC %uW",
                                    dp1.unary, dp1.decimal,
                                    char_degC_circle_str,
                                    RX_context.RX_radio_payload.u.payload_u0.heater_on_watt);
                        }
                    } else {
                        display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars, "RADIO? RX?");
                    }

                    setTextSize(2);

                #elif (IS_MYTARGET_SLAVE == 0) // MASTER
                    display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars, "%s", display_screen_name_str);
                #endif
                display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL
            } break;

            case SCREEN_STATISTICS_DB_ETC: {
                #if (IS_MYTARGET_SLAVE == 1)

                    // display_screen_name_str "2 " is not shown

                    // ..........----------. Some setTextSize(2)
                    // -100dB         ↑ -96
                    //                ↓ -101
                    // RX? 2 av 300
                    // RX? 1/150 (+2)

                    display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars, "%ddB", RX_context.nowRSSI);

                    setTextSize(2);
                    display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL

                    setTextSize(1);

                    setCursor(80,0);
                    display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars, "%s %d",
                            char_up_arrow_str, RX_context.nowRSSI_strongest);
                    display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL

                    setCursor(80,7);
                    display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars, "%s %d",
                            char_down_arrow_str, RX_context.nowRSSI_weakest);
                    display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL

                    setCursor(0,16);
                    {
                        unsigned lost_one_per;
                        if (RX_context.num_appSeqCnt_notSeen == 0) {
                            lost_one_per = 0;
                        } else {
                            lost_one_per = RX_context.num_received / RX_context.num_appSeqCnt_notSeen;
                        }
                        const signed diff = RX_context.num_appSeqCnt_notSeen - RX_context.num_appSeqCnt_notSeen_inDisplay; // Increasing, so alwasy positive. Still keep signed and %d to detect errors
                        display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars,
                                "RX? %u av %d\nRX? 1/%u (%s%u)",
                                RX_context.num_appSeqCnt_notSeen,
                                RX_context.num_received,
                                lost_one_per,
                                (diff > 0) ? "+" : "", diff);
                    }
                #elif (IS_MYTARGET_SLAVE == 0) // MASTER
                    display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars, "%s", display_screen_name_str);
                #endif
                display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL
            } break;

            case SCREEN_STATISTICS_OBS: {
                #if (IS_MYTARGET_SLAVE == 1)

                    // ..........----------.
                    // 3  *  OBS
                    // CRC16 123
                    // CRC32 0
                    // IRQ↑  123 (+2)

                    const signed diff = display_context.ultimateIRQclearCnt - display_context.ultimateIRQclearCnt_notSeen_inDisplay;
                    display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars,
                            "%s %s  OBS\nCRC16 %u\nCRC32 %u\nIRQ%s  %u (%s%d)",
                            display_screen_name_str,
                            alive ? "*" : "+",
                            RX_context.num_radioCRC16errs,
                            RX_context.num_appCRC32errs,
                            char_up_arrow_str,
                            display_context.ultimateIRQclearCnt,
                            (diff > 0) ? "+" : "", diff);

                #elif (IS_MYTARGET_SLAVE == 0) // MASTER
                    display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars, "%s", display_screen_name_str);
                #endif
                display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL
            } break;

            case SCREEN_TEMPS_ETC: {
                #if (IS_MYTARGET_SLAVE == 1)

                    const dp1_t max_water_dp1   = Parse_i16_dp1 (RX_context.RX_radio_payload_max.u.payload_u0.i2c_temp_water_onetenthDegC);
                    const dp1_t max_ambient_dp1 = Parse_i16_dp1 (RX_context.RX_radio_payload_max.u.payload_u0.i2c_temp_ambient_onetenthDegC);
                    const dp1_t max_heater_dp1  = Parse_i16_dp1 (RX_context.RX_radio_payload_max.u.payload_u0.i2c_temp_heater_onetenthDegC);

                    dp1_t now_water_dp1;
                    dp1_t now_ambient_dp1;
                    dp1_t now_heater_dp1;
                    if (use == USE_THIS) {
                        now_water_dp1   = Parse_i16_dp1 (RX_context.RX_radio_payload.u.payload_u0.i2c_temp_water_onetenthDegC);
                        now_ambient_dp1 = Parse_i16_dp1 (RX_context.RX_radio_payload.u.payload_u0.i2c_temp_ambient_onetenthDegC);
                        now_heater_dp1  = Parse_i16_dp1 (RX_context.RX_radio_payload.u.payload_u0.i2c_temp_heater_onetenthDegC);
                    } else if (use == USE_PREV) {
                        now_water_dp1   = Parse_i16_dp1 (RX_context.RX_radio_payload_prev.u.payload_u0.i2c_temp_water_onetenthDegC);
                        now_ambient_dp1 = Parse_i16_dp1 (RX_context.RX_radio_payload_prev.u.payload_u0.i2c_temp_ambient_onetenthDegC);
                        now_heater_dp1  = Parse_i16_dp1 (RX_context.RX_radio_payload_prev.u.payload_u0.i2c_temp_heater_onetenthDegC);
                    } else {
                        fail();
                    }

                    const dp1_t min_water_dp1   = Parse_i16_dp1 (RX_context.RX_radio_payload_min.u.payload_u0.i2c_temp_water_onetenthDegC);
                    const dp1_t min_ambient_dp1 = Parse_i16_dp1 (RX_context.RX_radio_payload_min.u.payload_u0.i2c_temp_ambient_onetenthDegC);
                    const dp1_t min_heater_dp1  = Parse_i16_dp1 (RX_context.RX_radio_payload_min.u.payload_u0.i2c_temp_heater_onetenthDegC);

                    // ..........----------.
                    // 4  * VANN LUFT VARME
                    // MAX  25.2 26.1 23.2
                    // NÅ.  25.2 26.1 23.2   '.' when awating data
                    // MIN  25.2 26.1 23.2

                    display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars,
                            "%s %s VANN LUFT VARME\nMAX  %2d.%1d %2d.%1d %2d.%1d\nN%s%s  %2d.%1d %2d.%1d %2d.%1d\nMIN  %2d.%1d %2d.%1d %2d.%1d",
                            display_screen_name_str,
                            alive ? "*" : "+",
                            max_water_dp1.unary, max_water_dp1.decimal, max_ambient_dp1.unary, max_ambient_dp1.decimal, max_heater_dp1.unary, max_heater_dp1.decimal,
                            char_aa_str, (use == USE_THIS) ? " " : ".",
                            now_water_dp1.unary, now_water_dp1.decimal, now_ambient_dp1.unary, now_ambient_dp1.decimal, now_heater_dp1.unary, now_heater_dp1.decimal,
                            min_water_dp1.unary, min_water_dp1.decimal, min_ambient_dp1.unary, min_ambient_dp1.decimal, min_heater_dp1.unary, min_heater_dp1.decimal);

                #elif (IS_MYTARGET_SLAVE == 0) // MASTER
                    display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars, "%s", display_screen_name_str);
                #endif
                display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL
            } break;

            case SCREEN_WATT_ETC: {
                #if (IS_MYTARGET_SLAVE == 1)
                    const heater_on_watt_r    max_heater_watt     =                RX_context.RX_radio_payload_max.u.payload_u0.heater_on_watt;
                    const heater_on_percent_r max_heater_percent  =                RX_context.RX_radio_payload_max.u.payload_u0.heater_on_percent;
                    const dp1_t               max_heater_mean_dp1 = Parse_i16_dp1 (RX_context.RX_radio_payload_max.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC);

                    heater_on_watt_r    now_heater_watt;
                    heater_on_percent_r now_heater_percent;
                    dp1_t               now_heater_mean_dp1;
                    if (use == USE_THIS) {
                        now_heater_watt     =                RX_context.RX_radio_payload.u.payload_u0.heater_on_watt;
                        now_heater_percent  =                RX_context.RX_radio_payload.u.payload_u0.heater_on_percent;
                        now_heater_mean_dp1 = Parse_i16_dp1 (RX_context.RX_radio_payload.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC);
                    } else if (use == USE_PREV) {
                        now_heater_watt     =                RX_context.RX_radio_payload_prev.u.payload_u0.heater_on_watt;
                        now_heater_percent  =                RX_context.RX_radio_payload_prev.u.payload_u0.heater_on_percent;
                        now_heater_mean_dp1 = Parse_i16_dp1 (RX_context.RX_radio_payload_prev.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC);
                    } else {
                        fail();
                    }

                    const heater_on_watt_r    min_heater_watt     =                RX_context.RX_radio_payload_min.u.payload_u0.heater_on_watt;
                    const heater_on_percent_r min_heater_percent  =                RX_context.RX_radio_payload_min.u.payload_u0.heater_on_percent;
                    const dp1_t               min_heater_mean_dp1 = Parse_i16_dp1 (RX_context.RX_radio_payload_min.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC);

                    const char now_regulating_at_char[][2] = NOW_REGULATING_AT_CHAR_TEXTS;

                    // ..........----------.
                    // 5  * R W  %   VARME≡  Heater tray mean temp
                    // MAX    48 100 40.4
                    // NÅ.  = 24 50  25.3    '.' when awating data. '=' in a white square
                    // MIN    0  0   24.2

                    #define WATT_NUMBER_WIDTH 2
                    char max_heater_watt_str[WATT_NUMBER_WIDTH+1];
                    char now_heater_watt_str[WATT_NUMBER_WIDTH+1];
                    char min_heater_watt_str[WATT_NUMBER_WIDTH+1];

                    u_to_str_lm (max_heater_watt, max_heater_watt_str, sizeof max_heater_watt_str);
                    u_to_str_lm (now_heater_watt, now_heater_watt_str, sizeof now_heater_watt_str);
                    u_to_str_lm (min_heater_watt, min_heater_watt_str, sizeof min_heater_watt_str);

                    #define PERCENT_NUMBER_WIDTH 3
                    char max_heater_percent_str[PERCENT_NUMBER_WIDTH+1];
                    char now_heater_percent_str[PERCENT_NUMBER_WIDTH+1];
                    char min_heater_percent_str[PERCENT_NUMBER_WIDTH+1];

                    u_to_str_lm (max_heater_percent, max_heater_percent_str, sizeof max_heater_percent_str);
                    u_to_str_lm (now_heater_percent, now_heater_percent_str, sizeof now_heater_percent_str);
                    u_to_str_lm (min_heater_percent, min_heater_percent_str, sizeof min_heater_percent_str);

                    display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars,
                            "%s %s R W  %%   VARME%s\nMAX    %s %s %2d.%1d\nN%s%s    %s %s %2d.%1d\nMIN    %s %s %2d.%1d",
                            display_screen_name_str,
                            alive ? "*" : "+",
                            char_triple_bar_str,
                            max_heater_watt_str,
                            max_heater_percent_str,
                            max_heater_mean_dp1.unary, max_heater_mean_dp1.decimal,
                            char_aa_str, (use == USE_THIS) ? " " : ".",
                            now_heater_watt_str,
                            now_heater_percent_str,
                            now_heater_mean_dp1.unary, now_heater_mean_dp1.decimal,
                            min_heater_watt_str,
                            min_heater_percent_str,
                            min_heater_mean_dp1.unary, min_heater_mean_dp1.decimal);

                    display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL

                    #define POS_X 27
                    drawRoundRect(POS_X, 14, 11, 11, 1, WHITE); // x,y,w,h,r,color x,y=0,0 is left top BORDERS ONLY
                    fillRoundRect(POS_X, 14, 11, 11, 1, WHITE); // x,y,w,h,r,color x,y=0,0 is left top FILL ONLY

                    setTextColor(BLACK);
                    setCursor(POS_X+3,16);

                    display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars, "%s",
                            now_regulating_at_char[RX_context.RX_radio_payload.u.payload_u0.now_regulating_at]);

                #elif (IS_MYTARGET_SLAVE == 0) // MASTER
                    display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars, "%s", display_screen_name_str);
                #endif
                display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL
            } break;

            case SCREEN_LIGHT: {
                #if (IS_MYTARGET_SLAVE == 1)
                    const char light_control_scheme_strings [][LIGHT_CONTROL_SCHEME_CHAR_TEXTS_LENGTH] = LIGHT_CONTROL_SCHEME_CHAR_TEXTS_LA;

                    unsigned num_light_amount = 0;
                    unsigned den_light_amount = 0;

                    // ..........----------.
                    // 6  * LYS 1/2
                    // LEDfmb   2/3 1/3 1/3
                    // NÅ.      DAG  = 10       '.' when awating data. Aquarium display also uses '='. Obs space after "DAG " but not "NATT" (in LIGHT_CONTROL_SCHEME_CHAR_TEXTS_LA)
                    // TIMER    10t 10-20

                    if (use == USE_PREV) {
                        // Wait for valid data. TODO have I copied enough over, is this filter necessary?
                    } else if (RXTX_context.PACKET.u.packet_u3.appHeading.version_of_full_payload == VERSION_OF_APP_PAYLOAD_01) {

                        if (RX_context.num_received == 0) {
                            RX_context.RX_radio_payload.u.payload_u0.light_amount.u.with_offset_30 = NORMAL_LIGHT_THIRDS_OFFSET;
                        } else {
                            // For this test always receives (from AQUARIUM) 32d which means "2/3"
                        }

                        num_light_amount = RX_context.RX_radio_payload.u.payload_u0.light_amount.u.with_offset_30 - NORMAL_LIGHT_THIRDS_OFFSET; // 32-30=2
                        den_light_amount = NORMAL_LIGHT_THIRDS_OFFSET/10; // 30/10=3

                    } else if (RXTX_context.PACKET.u.packet_u3.appHeading.version_of_full_payload == VERSION_OF_APP_PAYLOAD_02) {
                        if (RX_context.num_received == 0) {
                            RX_context.RX_radio_payload.u.payload_u0.light_amount.u.fraction_2_nibbles = 0x11; // "1/1"
                        } else {
                            // For this test it always receives (from BLACK_BAORD) 0x12 which means "1/2"
                        }

                        num_light_amount = GET_NUMERATOR   (RX_context.RX_radio_payload.u.payload_u0.light_amount.u.fraction_2_nibbles); // [1..9]
                        den_light_amount = GET_DENOMINATOR (RX_context.RX_radio_payload.u.payload_u0.light_amount.u.fraction_2_nibbles); // [1..9]

                    } else {} // Should not happen

                    display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars,
                            "%s %s LYS %u/%u\nLEDfmb   %u/3 %u/3 %u/3\nN%s%s      %s = %u\nTIMER    %ut %u-%u",
                            display_screen_name_str,
                            alive ? "*" : "+",
                            num_light_amount, den_light_amount,
                            RX_context.RX_radio_payload.u.payload_u0.light_intensity_thirds_front,
                            RX_context.RX_radio_payload.u.payload_u0.light_intensity_thirds_center,
                            RX_context.RX_radio_payload.u.payload_u0.light_intensity_thirds_back,
                            char_aa_str, (use == USE_THIS) ? " " : ".",
                            light_control_scheme_strings[RX_context.RX_radio_payload.u.payload_u0.light_control_scheme],
                            RX_context.RX_radio_payload.u.payload_u0.light_composition,
                            RX_context.RX_radio_payload.u.payload_u0.light_daytime_hours,
                            RX_context.RX_radio_payload.u.payload_u0.day_start_light_hour,
                            RX_context.RX_radio_payload.u.payload_u0.night_start_dark_hour
                    );

                #elif (IS_MYTARGET_SLAVE == 0) // MASTER
                    display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars, "%s", display_screen_name_str);
                #endif
                display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL
            } break;

            case SCREEN_TX_SEQ_CNT: {
                #if (IS_MYTARGET_SLAVE == 1)

                    // ..........----------.
                    // 7  * TX 242091
                    // TIMER   268
                    // DAGER   11

                    const unsigned hours = (RX_context.appSeqCnt * AQUARIUM_RFM69_REPEAT_SEND_EVERY_SEC) / 3600;
                    const unsigned days  = hours / 24; // Also shown in SCREEN_RX_MAIN_TIME_TEMP_ETC (as payload num_days_since_start)

                    display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars,
                            "%s %s TX %u\nTIMER   %u\nDAGER   %u",
                            display_screen_name_str,
                            alive ? "*" : "+",
                            RX_context.appSeqCnt,
                            hours, days);

                    #elif (IS_MYTARGET_SLAVE == 0) // MASTER
                        display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars, "%s", display_screen_name_str);
                    #endif
                    display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL
            } break;

            case SCREEN_AQUARIUM_BOX_INTERNALS: {
                #if (IS_MYTARGET_SLAVE == 1)
                    const dp1_t rr_12V_LEDlight_onetenthV      = Parse_i16_dp1 (RX_context.RX_radio_payload.u.payload_u0.rr_12V_LEDlight_onetenthV);
                    const dp1_t rr_24V_heat_onetenthV          = Parse_i16_dp1 (RX_context.RX_radio_payload.u.payload_u0.rr_24V_heat_onetenthV);
                    const dp1_t internal_box_temp_onetenthDegC = Parse_i16_dp1 (RX_context.RX_radio_payload.u.payload_u0.internal_box_temp_onetenthDegC);

                    // ..........----------.
                    // 8  *       AKVA      or KORT
                    //        LYS 11.9V
                    //      VARME 24.1V
                    // TEMPERATUR 24.5°C     Also used with max/min with debug_print

                    const bool is_aquarium = (display_context.senderid_displayed_now == MASTER_ID_AQUARIUM); // Opposte is MASTER_ID_BLACK_BOARD

                    display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars,
                            "%s %s       %s\n       LYS %02u.%uV\n     VARME %02u.%uV\nTEMPERATUR %02u.%u%sC",
                            display_screen_name_str,
                            alive ? "*" : "+",
                            (is_aquarium) ? "AKVA" : "KORT", // Displayed now
                            rr_12V_LEDlight_onetenthV.unary, rr_12V_LEDlight_onetenthV.decimal,
                            rr_24V_heat_onetenthV.unary, rr_24V_heat_onetenthV.decimal,
                            internal_box_temp_onetenthDegC.unary, internal_box_temp_onetenthDegC.decimal,
                            char_degC_circle_str);

                #elif (IS_MYTARGET_SLAVE == 0) // MASTER
                    display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars, "%s", display_screen_name_str);
                #endif
                display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL
            } break;

            case SCREEN_AQUARIUM_ERROR_BITS: {
                #if (IS_MYTARGET_SLAVE == 1)

                    // ..........----------.
                    // 9  *       FEIL
                    // NÅ         0000     '.' when awating data
                    // HISTORIE   0000
                    // FJERN #IRQ 255

                    display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars,
                            "%s %s       FEIL\nN%s%s        %04X\nHISTORIE   %04X\nFJERN #IRQ %u",
                            display_screen_name_str,
                            alive ? "*" : "+",
                            char_aa_str, (use == USE_THIS) ? " " : ".",
                            RX_context.RX_radio_payload.u.payload_u0.error_bits_now,
                            RX_context.RX_radio_payload.u.payload_u0.error_bits_history,
                            RX_context.RX_radio_payload.u.payload_u0.debug); // 8 bits here, full 32 bits value in display of aquarium (context.ultimateIRQclearCnt)

                    display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL
                #elif (IS_MYTARGET_SLAVE == 0) // MASTER
                    display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars, "%s", display_screen_name_str);
                #endif
            } break;

            case SCREEN_WELCOME: {
                #if (IS_MYTARGET_SLAVE == 1)

                    // ..........----------.
                    // 10 B:0.8.09 R:0.9.13
                    // RX AKVA               eller "RX KORT"
                    // HVERT 4. SEKUND (*)
                    // MED TIMEOUT 10 SEK    eller "UTEN TIMEOUT" or "TIMET UT 10 SEK" or "TIMET UT" a short period

                    char timeout_str [3];
                    sprintf (timeout_str, "%u", AQUARIUM_RFM69_RECEIVE_TIMOUT_SEC);

                    const bool is_aquarium = (display_context.senderid_displayed_now == MASTER_ID_AQUARIUM); // Opposte is MASTER_ID_BLACK_BOARD

                    display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars,
                            "%s B:%s R:%s\nRX %s\nHVERT %u. SEKUND (%s)\n%s %s %s",
                            display_screen_name_str,
                            RFM69_CLIENT_VERSION_STR,
                            RFM69_DRIVER_VERSION_STR,
                            (is_aquarium) ? "AKVA" : "KORT", // Displayed now
                            AQUARIUM_RFM69_REPEAT_SEND_EVERY_SEC,
                            alive ? "*" : "+",
                            RX_context.is_watchdog_blinking ?     "TIMET UT"    :
                                RX_context.allow_10_sek_timeout ? "MED TIMEOUT" : "UTEN TIMEOUT",
                            RX_context.allow_10_sek_timeout ? timeout_str : "",
                            RX_context.allow_10_sek_timeout ? "SEK"       : "");

                #elif (IS_MYTARGET_SLAVE == 0) // MASTER
                    display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars, "%s", display_screen_name_str);
                #endif
                display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL
            } break;

            case SCREEN_HJELP: {

                // ..........----------.
                // 11 H: AVSTILL P/10S   Høyre knapp:   Avstill feil-blinking (puls) eller nullstill statistikkdata (10 sekunder) eller neste verdi
                //    S: SKJERM ↑↓       Senterknapp:   Neste eller forrige skjermbilde
                //    V: SKJERM AV/PÅ    Venstre knapp: Skjerm av eller på
                //  S+V: SNU ↑↓          Hold inne senterknapp og trykk venstre knapp inn og ut toggler til neste eller forrige skjermbilde

                display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars,
                        "%s H: AVSTILL P/10S\n   S: SKJERM %s%s\n   V: SKJERM AV/P%s\n S+V: SNU %s%s",
                        display_screen_name_str,
                        char_up_arrow_str, char_down_arrow_str,
                        char_aa_str,
                        char_up_arrow_str, char_down_arrow_str);

                display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL
            } break;

            case SCREEN_RX_DISPLAY_OVERSIKT: {
                #if (IS_MYTARGET_SLAVE == 1)
                    // ..........----------.
                    // 12 * DISPLAY: AKVA   12 * DISPLAY: KORT
                    // VISES ADR 98         VISES ADR 99
                    // IKKE  ADR 99→        IKKE  ADR 98→
                    //       #RX 1234             #RX 1234

                    const bool is_aquarium = (display_context.senderid_displayed_now == MASTER_ID_AQUARIUM); // Opposte is MASTER_ID_BLACK_BOARD

                    display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars,
                            "%s %s DISPLAY: %s\nVISES ADR %u\nIKKE  ADR %u%s\n      #RX %u",
                            display_screen_name_str,
                            alive ? "*" : "+",
                            (is_aquarium) ? "AKVA" : "KORT", // Displayed now
                            display_context.senderid_displayed_now,
                            display_context.senderid_not_displayed_now,
                            char_right_arrow_str,
                            display_context.senderid_not_displayed_cnt);
                #elif (IS_MYTARGET_SLAVE == 0) // MASTER
                    display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars, "%s", display_screen_name_str);
                #endif
                display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL
            } break;

            case SCREEN_RADIO_DEBUG_TRANS: {
                #if (IS_MYTARGET_SLAVE == 1)
                    #if (CLIENT_ALLOW_SESSION_TYPE_TRANS==1)
                        // ..........----------.
                        // 13 * RADIO OK          RADIO FEIL?
                        //   MAX 820 us
                        //   LOG 87654321
                        //
                        display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars,
                                "%s %s RADIO %s\nMAX %u us\nLOG %08X",
                                display_screen_name_str,
                                alive ? "*" : "+",
                               (RXTX_context.timing_transx.timed_out_trans1to2) ? "FEIL?" : "OK",
                                RXTX_context.timing_transx.maxtime_used_us_trans1to2,
                                RXTX_context.radio_log_value);
                    #else
                        // ..........----------.
                        // 13 * RADIO
                        //      SYNC CALLS
                        //
                        display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars,
                                "%s %s RADIO\n     SYNC CALLS",
                                display_screen_name_str,
                                alive ? "*" : "+");
                    #endif

                #elif (IS_MYTARGET_SLAVE == 0) // MASTER
                    display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars, "%s", display_screen_name_str);
                #endif
                display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL
            } break;

            #if (GET_RADIO_DEBUG_REGS==1)
                case SCREEN_RADIO_DEBUG_REGS: {
                    display_context.sprintf_numchars = sprintf (display_context.display_ts1_chars,
                        "%s %s RADIO REGS\n  MANGLER", // Hent fra tidligere commit
                        display_screen_name_str);
                    display_print (display_context.display_ts1_chars, display_context.sprintf_numchars); // num chars not including NUL
                } break;
            #endif

            default:
            case SCREEN_DARK: {
                // No code
            } break;
        }

        i2c_ok = writeToDisplay_i2c_all_buffer(i_i2c_internal_commands);
        debug_print ("%s\n", i2c_ok ? "ok2" : "err2");

    } else {
        // is_off : no code
    }

    return i2c_ok;
}

// MUST NOT MODIFY ANY STATE VALUES!

#if (DEBUG_PRINT_GLOBAL_APP == 0)

    #define DEBUG_PRINT_VALUES(state_in,debug_print_context,RX_context,RXTX_context) // No code

#else
    #define DEBUG_PRINT_VALUES(state_in,debug_print_context,RX_context,RXTX_context) \
            Debug_print_values(state_in,debug_print_context,RX_context,RXTX_context)

    void Debug_print_values (
            const debug_print_state_e state_in,
            debug_print_context_t     &debug_print_context,
            RX_context_t              &RX_context,
            RXTX_context_t            &RXTX_context)
    {
        dp1_t dp1;

        switch (state_in) {
            case DEBUG_PRINT_RX_1_SEQCNT_ETC: {
                if (not debug_print_context.debug_print_rx_2_done) {
                    debug_print ("numbytes %u, from NODEID %u, RXappSeqCnt %u\n",
                           RXTX_context.some_rfm69_internals.PACKETLEN,
                           RXTX_context.PACKET.u.packet_u3.appNODEID,
                           RXTX_context.PACKET.u.packet_u3.appSeqCnt);
                } else {
                    debug_print ("RXappSeqCnt %u ", RXTX_context.PACKET.u.packet_u3.appSeqCnt);
                    if (RX_context.num_lost_since_last_success < 0) {
                        if (RX_context.num_appSeqCnt_notSeen != 0) {
                            debug_print ("Sender restarted? (RX_context.num_appSeqCnt_notSeen %u kept)\n", RX_context.num_appSeqCnt_notSeen);
                        } else {
                            debug_print ("%s", "\n");
                        }
                    } else if (RX_context.num_lost_since_last_success == 0) {
                        debug_print ("%s", "\n");
                    } else { // RX_context.num_lost_since_last_success > 0
                        debug_print ("num_lost_since_last_success %d, RX_context.num_appSeqCnt_notSeen %u\n", RX_context.num_lost_since_last_success, RX_context.num_appSeqCnt_notSeen);
                    }
                }
            } break;
            case DEBUG_PRINT_RX_2_TEMPS_ETC: {

                if (debug_print_context.debug_print_rx_2_done) {
                    debug_print ("\nRSSI %d, P %u, ",
                            RX_context.nowRSSI,
                            RXTX_context.PACKET.u.packet_u3.appPowerLevel_dBm);
                } else {
                    debug_print ("\nSENDERID %d, RSSI %d, P %u, ",
                           RXTX_context.some_rfm69_internals.SENDERID, // Received here
                           RX_context.nowRSSI,
                           RXTX_context.PACKET.u.packet_u3.appPowerLevel_dBm);
                }

                if (RX_context.doListenToAll) {
                    debug_print ("to %d, ", RXTX_context.some_rfm69_internals.TARGETID);
                } else {}

                {
                    const version_t version = Parse_packed_version (RX_context.RX_radio_payload.u.payload_u0.application_version_num);
                    debug_print ("Version %01u.%01u.%02u\n", version.major, version.minor, version.build); // 1110 -> 1.1.10
                }

                debug_print ("version_of_full_payload %u (0x%0x), num_of_this_app_payload %u\n",
                        RXTX_context.PACKET.u.packet_u3.appHeading.version_of_full_payload,
                        RX_context.RX_radio_payload.u.payload_u0.light_amount.u.fraction_2_nibbles,
                        RXTX_context.PACKET.u.packet_u3.appHeading.num_of_this_app_payload);

                debug_print ("num_days_since_start%s%04u at %02u:%02u:%02u\n",
                        (RX_context.RX_radio_payload.u.payload_u0.num_days_since_start == RX_context.RX_radio_payload_prev.u.payload_u0.num_days_since_start) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                         RX_context.RX_radio_payload.u.payload_u0.num_days_since_start,
                         RX_context.RX_radio_payload.u.payload_u0.hour,
                         RX_context.RX_radio_payload.u.payload_u0.minute,
                         RX_context.RX_radio_payload.u.payload_u0.second);

                debug_print ("error_bits_now%s0x%04x (error_bits_history%s0x%04x)\n",
                        (RX_context.RX_radio_payload.u.payload_u0.error_bits_now == RX_context.RX_radio_payload_prev.u.payload_u0.error_bits_now) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                         RX_context.RX_radio_payload.u.payload_u0.error_bits_now,
                        (RX_context.RX_radio_payload.u.payload_u0.error_bits_history == RX_context.RX_radio_payload_prev.u.payload_u0.error_bits_history) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                         RX_context.RX_radio_payload.u.payload_u0.error_bits_history);

                dp1 = Parse_i16_dp1 (RX_context.RX_radio_payload.u.payload_u0.i2c_temp_water_onetenthDegC); // dp1.sign never used here since it's per def POS
                debug_print ("Water %u.%udegC - ", dp1.unary, dp1.decimal);

                dp1 = Parse_i16_dp1 (RX_context.RX_radio_payload.u.payload_u0.i2c_temp_ambient_onetenthDegC);
                debug_print ("ambient %u.%udegC\n", dp1.unary, dp1.decimal);

                dp1 = Parse_i16_dp1 (RX_context.RX_radio_payload.u.payload_u0.i2c_temp_heater_onetenthDegC);
                debug_print ("ambient %u.%udegC\n", dp1.unary, dp1.decimal);
                debug_print ("Heater now_regulating_at%s%u temp %u.%udegC, ",
                        (RX_context.RX_radio_payload.u.payload_u0.now_regulating_at == RX_context.RX_radio_payload_prev.u.payload_u0.now_regulating_at) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                         RX_context.RX_radio_payload.u.payload_u0.now_regulating_at,
                         dp1.unary, dp1.decimal);

                dp1 = Parse_i16_dp1 (RX_context.RX_radio_payload.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC);
                debug_print ("mean %u.%udegC heater_on_percent%s%u%% heater_on_watt%s%uW\n", dp1.unary, dp1.decimal,
                        (RX_context.RX_radio_payload.u.payload_u0.heater_on_percent == RX_context.RX_radio_payload_prev.u.payload_u0.heater_on_percent) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                         RX_context.RX_radio_payload.u.payload_u0.heater_on_percent,
                        (RX_context.RX_radio_payload.u.payload_u0.heater_on_watt == RX_context.RX_radio_payload_prev.u.payload_u0.heater_on_watt) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                         RX_context.RX_radio_payload.u.payload_u0.heater_on_watt);

                if (RXTX_context.PACKET.u.packet_u3.appHeading.version_of_full_payload == VERSION_OF_APP_PAYLOAD_01) {
                    const char light_control_scheme_strings [][LIGHT_CONTROL_SCHEME_CHAR_TEXTS_LENGTH] = LIGHT_CONTROL_SCHEME_CHAR_TEXTS;
                    debug_print ("Light light_control_scheme%s%s with light_composition%s%02u gives FCB %u/3 %u/3 %u/3 full%s%u/3 day%s%uh (%u-%u)\n",
                            (RX_context.RX_radio_payload.u.payload_u0.light_control_scheme == RX_context.RX_radio_payload_prev.u.payload_u0.light_control_scheme) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                             light_control_scheme_strings[RX_context.RX_radio_payload.u.payload_u0.light_control_scheme],
                            (RX_context.RX_radio_payload.u.payload_u0.light_composition == RX_context.RX_radio_payload_prev.u.payload_u0.light_composition) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                             RX_context.RX_radio_payload.u.payload_u0.light_composition,
                             RX_context.RX_radio_payload.u.payload_u0.light_intensity_thirds_front,
                             RX_context.RX_radio_payload.u.payload_u0.light_intensity_thirds_center,
                             RX_context.RX_radio_payload.u.payload_u0.light_intensity_thirds_back,
                            (RX_context.RX_radio_payload.u.payload_u0.light_amount.u.with_offset_30 ==
                                    RX_context.RX_radio_payload_prev.u.payload_u0.light_amount.u.with_offset_30) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                             RX_context.RX_radio_payload.u.payload_u0.light_amount.u.with_offset_30 - NORMAL_LIGHT_THIRDS_OFFSET,
                            (RX_context.RX_radio_payload.u.payload_u0.light_daytime_hours == RX_context.RX_radio_payload_prev.u.payload_u0.light_daytime_hours) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                             RX_context.RX_radio_payload.u.payload_u0.light_daytime_hours,
                             RX_context.RX_radio_payload.u.payload_u0.day_start_light_hour,
                             RX_context.RX_radio_payload.u.payload_u0.night_start_dark_hour);
                } else if (RXTX_context.PACKET.u.packet_u3.appHeading.version_of_full_payload == VERSION_OF_APP_PAYLOAD_02) {
                    const char light_control_scheme_strings [][LIGHT_CONTROL_SCHEME_CHAR_TEXTS_LENGTH] = LIGHT_CONTROL_SCHEME_CHAR_TEXTS;
                    const unsigned num_light_amount = GET_NUMERATOR   (RX_context.RX_radio_payload.u.payload_u0.light_amount.u.fraction_2_nibbles ); // [1..9]
                    const unsigned den_light_amount = GET_DENOMINATOR (RX_context.RX_radio_payload.u.payload_u0.light_amount.u.fraction_2_nibbles ); // [1..9]
                    debug_print ("Light light_control_scheme%s%s with light_composition%s%02u gives FCB %u/3 %u/3 %u/3 amount%s%u/%u day%s%uh (%u-%u)\n",
                            (RX_context.RX_radio_payload.u.payload_u0.light_control_scheme == RX_context.RX_radio_payload_prev.u.payload_u0.light_control_scheme) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                             light_control_scheme_strings[RX_context.RX_radio_payload.u.payload_u0.light_control_scheme],
                            (RX_context.RX_radio_payload.u.payload_u0.light_composition == RX_context.RX_radio_payload_prev.u.payload_u0.light_composition) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                             RX_context.RX_radio_payload.u.payload_u0.light_composition,
                             RX_context.RX_radio_payload.u.payload_u0.light_intensity_thirds_front,
                             RX_context.RX_radio_payload.u.payload_u0.light_intensity_thirds_center,
                             RX_context.RX_radio_payload.u.payload_u0.light_intensity_thirds_back,
                            (RX_context.RX_radio_payload.u.payload_u0.light_amount.u.fraction_2_nibbles ==
                             RX_context.RX_radio_payload_prev.u.payload_u0.light_amount.u.fraction_2_nibbles) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                             num_light_amount, den_light_amount,
                            (RX_context.RX_radio_payload.u.payload_u0.light_daytime_hours == RX_context.RX_radio_payload_prev.u.payload_u0.light_daytime_hours) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                             RX_context.RX_radio_payload.u.payload_u0.light_daytime_hours,
                             RX_context.RX_radio_payload.u.payload_u0.day_start_light_hour,
                             RX_context.RX_radio_payload.u.payload_u0.night_start_dark_hour);
                } else {} // Should not happen

                dp1 = Parse_i16_dp1 (RX_context.RX_radio_payload.u.payload_u0.rr_24V_heat_onetenthV);
                debug_print ("Voltage at heater %02u.%uV, ",  dp1.unary, dp1.decimal);

                dp1 = Parse_i16_dp1 (RX_context.RX_radio_payload.u.payload_u0.rr_12V_LEDlight_onetenthV);
                debug_print ("light %02u.%uV\n", dp1.unary, dp1.decimal);

                dp1 = Parse_i16_dp1 (RX_context.RX_radio_payload.u.payload_u0.internal_box_temp_onetenthDegC);
                debug_print ("Box %02u.%udegC\n", dp1.unary, dp1.decimal);

                debug_print ("Debug%s%1u.%01X\n", // Debug= or Debug#
                        (RX_context.RX_radio_payload.u.payload_u0.debug == RX_context.RX_radio_payload_prev.u.payload_u0.debug) ? CHAR_EQ_STR : CHAR_CHANGE_STR,
                        (RX_context.RX_radio_payload.u.payload_u0.debug >> 4) bitand 0x0f,
                         RX_context.RX_radio_payload.u.payload_u0.debug       bitand 0x0f);

                debug_print_context.debug_print_rx_2_done = true;
            } break;

            case DEBUG_PRINT_RX_2_NOW_MAX_MIN: {
                {
                    const dp1_t heater_dp1       = Parse_i16_dp1 (RX_context.RX_radio_payload_max.u.payload_u0.i2c_temp_heater_onetenthDegC);
                    const dp1_t ambient_dp1      = Parse_i16_dp1 (RX_context.RX_radio_payload_max.u.payload_u0.i2c_temp_ambient_onetenthDegC);
                    const dp1_t water_dp1        = Parse_i16_dp1 (RX_context.RX_radio_payload_max.u.payload_u0.i2c_temp_water_onetenthDegC);
                    const dp1_t heater_mean_dp1  = Parse_i16_dp1 (RX_context.RX_radio_payload_max.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC);
                    const dp1_t internal_dp1     = Parse_i16_dp1 (RX_context.RX_radio_payload_max.u.payload_u0.internal_box_temp_onetenthDegC);
                    debug_print ("MAX: On:%3u%% @ Watt:%2u - Heater:%2d.%1d Ambient:%2d.%1d Water:%2d.%1d Mean:%2d.%1d Box:%2d.%1d\n",
                        RX_context.RX_radio_payload_max.u.payload_u0.heater_on_percent,
                        RX_context.RX_radio_payload_max.u.payload_u0.heater_on_watt,
                        heater_dp1.unary, heater_dp1.decimal,
                        ambient_dp1.unary, ambient_dp1.decimal,
                        water_dp1.unary, water_dp1.decimal,
                        heater_mean_dp1.unary, heater_mean_dp1.decimal,
                        internal_dp1.unary, internal_dp1.decimal);
                }
                {
                    const dp1_t heater_dp1       = Parse_i16_dp1 (RX_context.RX_radio_payload.u.payload_u0.i2c_temp_heater_onetenthDegC);
                    const dp1_t ambient_dp1      = Parse_i16_dp1 (RX_context.RX_radio_payload.u.payload_u0.i2c_temp_ambient_onetenthDegC);
                    const dp1_t water_dp1        = Parse_i16_dp1 (RX_context.RX_radio_payload.u.payload_u0.i2c_temp_water_onetenthDegC);
                    const dp1_t heater_mean_dp1  = Parse_i16_dp1 (RX_context.RX_radio_payload.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC);
                    const dp1_t internal_dp1     = Parse_i16_dp1 (RX_context.RX_radio_payload.u.payload_u0.internal_box_temp_onetenthDegC);
                    debug_print ("NOW: On:%3u%% @ Watt:%2u - Heater:%2d.%1d Ambient:%2d.%1d Water:%2d.%1d Mean:%2d.%1d Box:%2d.%1d\n",
                        RX_context.RX_radio_payload.u.payload_u0.heater_on_percent,
                        RX_context.RX_radio_payload.u.payload_u0.heater_on_watt,
                        heater_dp1.unary, heater_dp1.decimal,
                        ambient_dp1.unary, ambient_dp1.decimal,
                        water_dp1.unary, water_dp1.decimal,
                        heater_mean_dp1.unary, heater_mean_dp1.decimal,
                        internal_dp1.unary, internal_dp1.decimal);
                }
                {
                    const dp1_t heater_dp1       = Parse_i16_dp1 (RX_context.RX_radio_payload_min.u.payload_u0.i2c_temp_heater_onetenthDegC);
                    const dp1_t ambient_dp1      = Parse_i16_dp1 (RX_context.RX_radio_payload_min.u.payload_u0.i2c_temp_ambient_onetenthDegC);
                    const dp1_t water_dp1        = Parse_i16_dp1 (RX_context.RX_radio_payload_min.u.payload_u0.i2c_temp_water_onetenthDegC);
                    const dp1_t heater_mean_dp1  = Parse_i16_dp1 (RX_context.RX_radio_payload_min.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC);
                    const dp1_t internal_dp1     = Parse_i16_dp1 (RX_context.RX_radio_payload_min.u.payload_u0.internal_box_temp_onetenthDegC);
                    debug_print ("MIN: On:%3u%% @ Watt:%2u - Heater:%2d.%1d Ambient:%2d.%1d Water:%2d.%1d Mean:%2d.%1d Box:%2d.%1d\n",
                        RX_context.RX_radio_payload_min.u.payload_u0.heater_on_percent,
                        RX_context.RX_radio_payload_min.u.payload_u0.heater_on_watt,
                        heater_dp1.unary, heater_dp1.decimal,
                        ambient_dp1.unary, ambient_dp1.decimal,
                        water_dp1.unary, water_dp1.decimal,
                        heater_mean_dp1.unary, heater_mean_dp1.decimal,
                        internal_dp1.unary, internal_dp1.decimal);
                }
            } break;

            default: {} break;
        }
    }
#endif

void RFM69_handle_irq (
         RX_context_t                      &?RX_context,
         TX_context_t                      &?TX_context,
         RXTX_context_t                    &RXTX_context,
         display_context_t                 &display_context,
         client  radio_if_t                i_radio,
         client  blink_and_watchdog_if_t   i_blink_and_watchdog,
         client  i2c_internal_commands_if  i_i2c_internal_commands,
         debug_print_context_t             &debug_print_context)
{
    // IRQ LED mounted RFM69 board's IRQ/G0 pin will lit now (via a transistor). Same as SPI_IRQ port

    // c_irq_rising is delivered from IRQ_detect_task also if pin change was over while _this_ task
    // was in an i_radio call, which might last longer than the IRQ pulse! We saw that this
    // construct used here was never taken:
    //     case p_spi_irq when pinsneq(spi_irq_current_val) :> spi_irq_current_val: {
    // So we offloaded to IRQ_detect_task instead
    interruptAndParsingResult_e interruptAndParsingResult;

    RXTX_context.interruptCnt++;

    #if (CLIENT_ALLOW_SESSION_TYPE_TRANS==1)
       // CALL IFF, BUT IN ANY CASE NO SPI (so no _trans1)
       do_spi_aux_pin_iff (RXTX_context.timing_transx.timed_out_trans1to2, i_radio, MASKOF_SPI_AUX0_PROBE3_IRQ, high);
    #else
       i_radio.do_spi_aux_pin (MASKOF_SPI_AUX0_PROBE3_IRQ, high); // For scope
    #endif

    #if (CLIENT_ALLOW_SESSION_TYPE_TRANS==1)

        // ASYNCH CALL AND BACKGROUND ACTION WITH TIMEOUT
        #if (TRANS_ASYNCH_WRAPPED==1)
            interruptAndParsingResult = _handleSPIInterrupt_iff_asynch (i_radio, RXTX_context.timing_transx, RXTX_context.some_rfm69_internals, RXTX_context.PACKET); // TODO ta bort _
        #else
            RXTX_context.timing_transx.start_time_trans1 = handleSPIInterrupt_iff_trans1 (RXTX_context.timing_transx.timed_out_trans1to2, i_radio);
            // MUST be run now:
            do_sessions_trans2to3 (i_radio, RXTX_context.timing_transx, RXTX_context.return_trans3);

            RXTX_context.some_rfm69_internals = RXTX_context.return_trans3.u_out.handleSPIInterrupt.return_some_rfm69_internals;
            RXTX_context.PACKET               = RXTX_context.return_trans3.u_out.handleSPIInterrupt.return_PACKET;
            interruptAndParsingResult         = RXTX_context.return_trans3.u_out.handleSPIInterrupt.return_interruptAndParsingResult;
        #endif
        RXTX_context.radio_log_value = RXTX_context.timing_transx.radio_log_value;
    #else
        {RXTX_context.some_rfm69_internals, RXTX_context.PACKET, interruptAndParsingResult} = i_radio.uspi_handleSPIInterrupt(); // DO IT and GET DATA
        // Now values like RXTX_context.some_rfm69_internals.SENDERID has a value
    #endif

    #if (DEBUG_PRINT_BUFFER==1)
        const char char_leading_space_str[] = CHAR_LEADING_SPACE_STR;
        #if (IS_MYTARGET_MASTER == 1)
            debug_print ("%sIRQ %u:\n", char_leading_space_str, interruptAndParsingResult);
        #endif

        for (unsigned index = 0; index < PACKET_LEN32; index++) {
            unsigned value = RXTX_context.PACKET.u.packet_u2_uint32_arr[index];
            debug_print ("%s[%u]: %08X\n",
                    (interruptAndParsingResult == messageReceivedOk_IRQ)   ? "RX" :
                    (interruptAndParsingResult == messagePacketSentOk_IRQ) ? "TX" : "?X",
                    index,
                    value);
        }

        debug_print ("%s", char_leading_space_str);
    #endif

    #if (CLIENT_ALLOW_SESSION_TYPE_TRANS==1)
        // ASYNCH CALL AND BACKGROUND ACTION WITH TIMEOUT
        #if (TRANS_ASYNCH_WRAPPED==1)
            RXTX_context.receiveDone = receiveDone_iff_asynch (i_radio, RXTX_context.timing_transx);
        #else
            RXTX_context.timing_transx.start_time_trans1 = receiveDone_iff_trans1 (RXTX_context.timing_transx.timed_out_trans1to2, i_radio);
            // MUST be run now:
            do_sessions_trans2to3 (i_radio, RXTX_context.timing_transx, RXTX_context.return_trans3);
            RXTX_context.receiveDone = RXTX_context.return_trans3.u_out.receiveDone;
        #endif
        RXTX_context.radio_log_value = RXTX_context.timing_transx.radio_log_value;
    #else
        RXTX_context.receiveDone = i_radio.uspi_receiveDone(); // For any interruptAndParsingResult (30Aug2018, 12Sept2018 TODO works?)
    #endif

    switch (interruptAndParsingResult) {

        #if (IS_MYTARGET_SLAVE == 1)
            case messageReceivedOk_IRQ: {
                // if (i_radio.uspi_receiveDone()) {
                if (RXTX_context.receiveDone) {

                    if (RXTX_context.some_rfm69_internals.SENDERID == display_context.senderid_displayed_now) { // From this address
                        if (display_context.state == is_on) {
                            i_blink_and_watchdog.blink_pulse_ok (XCORE_200_EXPLORER_LED_RGB_BLUE_BIT_MASK, 25);
                        } else {}

                        i_blink_and_watchdog.feed_watchdog();

                        if (display_context.state == is_on) {
                            i_blink_and_watchdog.blink_pulse_ok (XCORE_200_EXPLORER_LED_RGB_RED_BIT_MASK, 25); // Looks orange
                        } else {}

                        RX_context.seconds_since_last_received = 0;

                        RX_context.appSeqCnt = RXTX_context.PACKET.u.packet_u3.appSeqCnt; // Now. Probably is a very high number also on the first message we see. So:

                        if (RX_context.num_received == 0) { // First message we see
                            RX_context.num_lost_since_last_success = 0;
                        } else {
                            RX_context.num_lost_since_last_success = RX_context.appSeqCnt - RX_context.appSeqCnt_prev - 1; // So that 123 - 122 - 1 is zero for none lost
                        }
                        RX_context.num_received++;

                        if (RX_context.num_lost_since_last_success > 0) {
                            RX_context.num_appSeqCnt_notSeen += RX_context.num_lost_since_last_success;
                            // 03Apr2018: one lost in 140, then in 141 (of tenths of thousands!), one in 263
                        } else {}

                        for (unsigned index = 0; index < _USERMAKEFILE_LIB_RFM69_XC_PAYLOAD_LEN08; index++) {
                            RX_context.RX_radio_payload.u.payload_u1_uint8_arr [index] = RXTX_context.PACKET.u.packet_u3.appPayload_uint8_arr[index]; // Received now
                        }

                        if (display_context.allow_auto_switch_to_screen_rx_main_time_temp_etc) {
                            display_context.allow_auto_switch_to_screen_rx_main_time_temp_etc = false; // Once
                            display_context.display_screen_name = SCREEN_RX_MAIN_TIME_TEMP_ETC;
                        } else {}

                        // ONLY THOSE PRINTED OR DISPLAYED ARE CALCULATED ON:
                        //
                        RX_context.RX_radio_payload_max.u.payload_u0.heater_on_percent                        = max (RX_context.RX_radio_payload_max.u.payload_u0.heater_on_percent,                        RX_context.RX_radio_payload.u.payload_u0.heater_on_percent);
                        RX_context.RX_radio_payload_max.u.payload_u0.heater_on_watt                           = max (RX_context.RX_radio_payload_max.u.payload_u0.heater_on_watt,                           RX_context.RX_radio_payload.u.payload_u0.heater_on_watt);
                        RX_context.RX_radio_payload_max.u.payload_u0.i2c_temp_heater_onetenthDegC             = max (RX_context.RX_radio_payload_max.u.payload_u0.i2c_temp_heater_onetenthDegC,             RX_context.RX_radio_payload.u.payload_u0.i2c_temp_heater_onetenthDegC);
                        RX_context.RX_radio_payload_max.u.payload_u0.i2c_temp_ambient_onetenthDegC            = max (RX_context.RX_radio_payload_max.u.payload_u0.i2c_temp_ambient_onetenthDegC,            RX_context.RX_radio_payload.u.payload_u0.i2c_temp_ambient_onetenthDegC);
                        RX_context.RX_radio_payload_max.u.payload_u0.i2c_temp_water_onetenthDegC              = max (RX_context.RX_radio_payload_max.u.payload_u0.i2c_temp_water_onetenthDegC,              RX_context.RX_radio_payload.u.payload_u0.i2c_temp_water_onetenthDegC);
                        RX_context.RX_radio_payload_max.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC = max (RX_context.RX_radio_payload_max.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC, RX_context.RX_radio_payload.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC);
                        RX_context.RX_radio_payload_max.u.payload_u0.internal_box_temp_onetenthDegC           = max (RX_context.RX_radio_payload_max.u.payload_u0.internal_box_temp_onetenthDegC,           RX_context.RX_radio_payload.u.payload_u0.internal_box_temp_onetenthDegC);
                        //
                        RX_context.RX_radio_payload_min.u.payload_u0.heater_on_percent                        = min (RX_context.RX_radio_payload_min.u.payload_u0.heater_on_percent,                        RX_context.RX_radio_payload.u.payload_u0.heater_on_percent);
                        RX_context.RX_radio_payload_min.u.payload_u0.heater_on_watt                           = min (RX_context.RX_radio_payload_min.u.payload_u0.heater_on_watt,                           RX_context.RX_radio_payload.u.payload_u0.heater_on_watt);
                        RX_context.RX_radio_payload_min.u.payload_u0.i2c_temp_heater_onetenthDegC             = min (RX_context.RX_radio_payload_min.u.payload_u0.i2c_temp_heater_onetenthDegC,             RX_context.RX_radio_payload.u.payload_u0.i2c_temp_heater_onetenthDegC);
                        RX_context.RX_radio_payload_min.u.payload_u0.i2c_temp_ambient_onetenthDegC            = min (RX_context.RX_radio_payload_min.u.payload_u0.i2c_temp_ambient_onetenthDegC,            RX_context.RX_radio_payload.u.payload_u0.i2c_temp_ambient_onetenthDegC);
                        RX_context.RX_radio_payload_min.u.payload_u0.i2c_temp_water_onetenthDegC              = min (RX_context.RX_radio_payload_min.u.payload_u0.i2c_temp_water_onetenthDegC,              RX_context.RX_radio_payload.u.payload_u0.i2c_temp_water_onetenthDegC);
                        RX_context.RX_radio_payload_min.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC = min (RX_context.RX_radio_payload_min.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC, RX_context.RX_radio_payload.u.payload_u0.temp_heater_mean_last_cycle_onetenthDegC);
                        RX_context.RX_radio_payload_min.u.payload_u0.internal_box_temp_onetenthDegC           = min (RX_context.RX_radio_payload_min.u.payload_u0.internal_box_temp_onetenthDegC,           RX_context.RX_radio_payload.u.payload_u0.internal_box_temp_onetenthDegC);

                        RX_context.nowRSSI_weakest   = min (RX_context.nowRSSI_weakest,   RX_context.nowRSSI); // Not in DEBUG_PRINT_RX_2_NOW_MAX_MIN
                        RX_context.nowRSSI_strongest = max (RX_context.nowRSSI_strongest, RX_context.nowRSSI); // Not in DEBUG_PRINT_RX_2_NOW_MAX_MIN

                        DEBUG_PRINT_VALUES (DEBUG_PRINT_RX_2_NOW_MAX_MIN, debug_print_context, RX_context, RXTX_context);

                        Display_screen (display_context, RX_context, RXTX_context, USE_THIS, i_i2c_internal_commands);

                        DEBUG_PRINT_VALUES (DEBUG_PRINT_RX_1_SEQCNT_ETC, debug_print_context, RX_context, RXTX_context);
                        DEBUG_PRINT_VALUES (DEBUG_PRINT_RX_2_TEMPS_ETC, debug_print_context, RX_context, RXTX_context);

                        // RFM69 had a call to receiveDone(); here, only needed if setMode(RF69_MODE_STANDBY) case 1 in receiveDone
                        // Reinserted RFM69=001
                        #if (SEMANTICS_DO_INTERMEDIATE_RECEIVEDONE == 1)         // TODO remove
                            #if (CLIENT_ALLOW_SESSION_TYPE_TRANS==1)
                                // ASYNCH CALL AND BACKGROUND ACTION WITH TIMEOUT
                                #if (TRANS_ASYNCH_WRAPPED==1)
                                    RXTX_context.receiveDone = receiveDone_iff_asynch (i_radio, RXTX_context.timing_transx);
                                #else
                                    RXTX_context.timing_transx.start_time_trans1 = receiveDone_iff_trans1 (RXTX_context.timing_transx.timed_out_trans1to2, i_radio);
                                    // MUST be run now:
                                    do_sessions_trans2to3 (i_radio, RXTX_context.timing_transx, RXTX_context.return_trans3);
                                    RXTX_context.receiveDone = RXTX_context.return_trans3.u_out.receiveDone;
                                #endif
                                RXTX_context.radio_log_value = RXTX_context.timing_transx.radio_log_value;
                            #else
                                RXTX_context.receiveDone = i_radio.uspi_receiveDone(); // For any interruptAndParsingResult (30Aug2018, 12Sept2018 TODO works?)
                            #endif
                            if (RXTX_context.receiveDone) {                     // This is needed even if..
                               debug_print ("%s\n", "receiveDone in polling!");  // ..it never gets here! (TODO?)
                            } else {}
                        #endif

                        if (RX_context.num_lost_since_last_success == 0) {
                            // BOTH 40 5Oct2018: debug_print ("sizeof %u, LEN %u\n", sizeof RX_context.RX_radio_payload.u.payload_u1_uint8_arr, _USERMAKEFILE_LIB_RFM69_XC_PAYLOAD_LEN08);
                            for (unsigned index = 0; index < _USERMAKEFILE_LIB_RFM69_XC_PAYLOAD_LEN08; index++) {
                                // Take a copy of last received into "previous"
                                RX_context.RX_radio_payload_prev.u.payload_u1_uint8_arr[index] = RX_context.RX_radio_payload.u.payload_u1_uint8_arr [index]; // Received last
                            }
                        } else {} // Don't restore or set to PACKET_INIT_VAL08, I get to many CHAR_CHANGE_STR ('#')

                        RX_context.appSeqCnt_prev = RX_context.appSeqCnt;
                    } else {
                        display_context.senderid_not_displayed_cnt++;
                        if (display_context.display_screen_name == SCREEN_RX_DISPLAY_OVERSIKT) {
                            Display_screen (display_context, RX_context, RXTX_context, USE_THIS, i_i2c_internal_commands);
                        } else {}
                    }

                } else {
                    debug_print ("Max %u %u \n", "IRQ but not receiveDone!");
                }
            } break;

            // No unconditional Display_screen after this point in this function! Then it may show the wrong data set
            // When a button is pressed it's fine, but then with USE_PREV

            #if (TEST_01_FOLLOW_ADDRESS==1)
                case messageNotForThisNode_IRQ: { // Both MASTER_ID_AQUARIUM and MASTER_ID_BLACK_BOX may send TO this address
                    #if (TEST_01_LISTENTOALL==1)
                        debug_print ("%s\n", "Starting to receive on any address");
                        #if (CLIENT_ALLOW_SESSION_TYPE_TRANS==1)
                            // CALL IFF, BUT IN ANY CASE NO SPI (so no _trans1)
                            setListenToAll_iff (RXTX_context.timing_transx.timed_out_trans1to2, i_radio, true);
                        #else
                            i_radio.setListenToAll (true);
                        #endif
                    #else
                        uint8_t

                        #if (CLIENT_ALLOW_SESSION_TYPE_TRANS==1)
                            // CALL IFF, BUT IN ANY CASE NO SPI (so no _trans1)
                            previous_NODEID = setNODEID_iff (RXTX_context.timing_transx.timed_out_trans1to2, i_radio, RXTX_context.some_rfm69_internals.TARGETID);
                        #else
                            previous_NODEID = i_radio.setNODEID (RXTX_context.some_rfm69_internals.TARGETID); // Follow who sender wants to send to
                        #endif
                        debug_print ("\nStarting (from #%03u) to receive on address #%03u, RX_context.num_appSeqCnt_notSeen %u, RXappSeqCnt %u\n",
                                previous_NODEID,
                                RXTX_context.some_rfm69_internals.TARGETID,
                                RX_context.num_appSeqCnt_notSeen,
                                RXTX_context.PACKET.u.packet_u3.appSeqCnt);
                    #endif
                } break;
            #endif

            case messageRadioCRC16Err_IRQ:
            case messageAppCRC32Err_IRQ: {

                const bool CRC16err = (interruptAndParsingResult == messageRadioCRC16Err_IRQ);
                const bool CRC32err = (interruptAndParsingResult == messageAppCRC32Err_IRQ);

                if (CRC16err) {RX_context.num_radioCRC16errs++;}
                if (CRC32err) {RX_context.num_appCRC32errs++;}

                debug_print ("RSSI %d, CRC-fail radioCRC16@%u %u, appCRC32@%u %u with PACKETLEN %u\n",
                        RX_context.nowRSSI,
                        CRC16err,
                        RX_context.num_radioCRC16errs,
                        CRC32err,
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
                        RX_context.nowRSSI, interruptAndParsingResult, RX_context.num_radioCRC16errs, RX_context.num_appCRC32errs, RXTX_context.some_rfm69_internals.PACKETLEN);

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

    #if (CLIENT_ALLOW_SESSION_TYPE_TRANS==1)
        // CALL IFF, BUT IN ANY CASE NO SPI (so no _trans1)
        {RXTX_context.some_rfm69_internals.error_bits, RXTX_context.is_new_error} = getAndClearErrorBits_iff (RXTX_context.timing_transx.timed_out_trans1to2, i_radio);
    #else
        {RXTX_context.some_rfm69_internals.error_bits, RXTX_context.is_new_error} = i_radio.getAndClearErrorBits();
    #endif

    if (RXTX_context.some_rfm69_internals.error_bits != ERROR_BITS_NONE) {
        debug_print ("RFM69 err2 new %u code %04X\n", RXTX_context.is_new_error, RXTX_context.some_rfm69_internals.error_bits);
    } else {}

    #if (DEBUG_SHARED_LOG_VALUE==1)
        if (not RXTX_context.timing_transx.timed_out_trans1to2) {
            clr_radio_log_value(); // But RXTX_context.radio_log_value is not touched now, it will be updated with new value of not timed out
            debug_print ("%s\n", "clr_radio_log_value");
        } else {}
    #endif

    #if (CLIENT_ALLOW_SESSION_TYPE_TRANS==1)
       // CALL IFF, BUT IN ANY CASE NO SPI (so no _trans1)
       do_spi_aux_pin_iff (RXTX_context.timing_transx.timed_out_trans1to2, i_radio, MASKOF_SPI_AUX0_PROBE3_IRQ, low);
    #else
       i_radio.do_spi_aux_pin (MASKOF_SPI_AUX0_PROBE3_IRQ, low); // For scope
    #endif
}

void RFM69_handle_timeout (
        divTime_t                        &divTime,
        const time32_t                   startTime_ticks,
        RX_context_t                     &?RX_context,
        TX_context_t                     &?TX_context,
        RXTX_context_t                   &RXTX_context,
        display_context_t                &display_context,
        const   unsigned                 seconds_since_last_call,
        client  radio_if_t               i_radio,
        client  blink_and_watchdog_if_t  i_blink_and_watchdog,
        client  i2c_internal_commands_if i_i2c_internal_commands)
{

    // i_blink_and_watchdog.blink_pulse_ok (XCORE_200_EXPLORER_LED_GREEN_BIT_MASK, 50);

    #if (DEBUG_PRINT_TIME_USED == 1)
        debug_print ("..After %08X is time %08X ticks\n", divTime.time_ticks, startTime_ticks);
    #endif

    #if (IS_MYTARGET_SLAVE == 1)

        #if (GET_RADIO_DEBUG_REGS==1)
            if (display_context.display_screen_name == SCREEN_RADIO_DEBUG_REGS) {
                // SCREEN_RADIO_DEBUG_REGS is empty
                #if (CLIENT_ALLOW_SESSION_TYPE_TRANS==1)
                     // ASYNCH CALL AND BACKGROUND ACTION WITH TIMEOUT
                     #if (TRANS_ASYNCH_WRAPPED==1)
                         RX_context.debug_data = getDebug_iff_asynch (i_radio, RXTX_context.timing_transx, RX_context.debug_state);
                     #else
                         RXTX_context.timing_transx.start_time_trans1 = getDebug_iff_trans1 (RXTX_context.timing_transx.timed_out_trans1to2, i_radio, RX_context.debug_state);
                         // MUST be run now:
                         do_sessions_trans2to3 (i_radio, RXTX_context.timing_transx, RXTX_context.return_trans3);
                         RX_context.debug_data = RXTX_context.return_trans3.u_out.debug_data;
                     #endif
                     RXTX_context.radio_log_value = RXTX_context.timing_transx.radio_log_value;
                 #else
                     RX_context.debug_data = i_radio.uspi_getDebug (RX_context.debug_state);
                 #endif

                bool are_equal = (memcmp (RX_context.debug_data_prev.data, RX_context.debug_data.data, NUM_DEBUG_BYTES) == 0);

                for (unsigned i = 0; i < NUM_DEBUG_BYTES; i++) {
                    RX_context.debug_data_prev.data[i] = RX_context.debug_data.data[i];
                }

                debug_print ("DEB%s\n", are_equal ? CHAR_EQ_STR : CHAR_CHANGE_STR);

                for (unsigned i = 0; i < NUM_DEBUG_BYTES; i++) {
                    debug_print ("%02X%s", RX_context.debug_data.data[i], ((i == (NUM_DEBUG_BYTES-1)) ? "\n" : " "));
                }
                debug_print ("%s", "\n");
            } else {}
        #else
            // No code
        #endif

    #elif (IS_MYTARGET_MASTER == 1)
        if (TX_context.sendPacket_seconds_cntdown == 0) {
            const char char_leading_space_str[] = CHAR_LEADING_SPACE_STR;
            TX_context.TX_appSeqCnt++;

            if (TX_context.waitForIRQInterruptCause != no_IRQExpected) {
                // Normal send failed: no messagePacketSentOk_IRQ seen
                #if (SEMANTICS_DO_LOOP_FOR_RF_IRQFLAGS2_PACKETSENT == 0)
                    debug_print ("fail IRQ waitForIRQInterruptCause %u\n", TX_context.waitForIRQInterruptCause); // Report and continue
                    TX_context.waitForIRQInterruptCause = no_IRQExpected; // Clear it here even if i_radio.uspi_send will overwrite it
                #endif
            } else {} // No code

            if (TX_context.TX_appSeqCnt == 10) {
                #if (TEST_CAR_KEY == 1)
                    // No code, kep full power always
                #else
                    TX_context.TX_appPowerLevel_dBm = APPPOWERLEVEL_MIN_DBM;

                    #if (CLIENT_ALLOW_SESSION_TYPE_TRANS==1)

                        // ASYNCH CALL AND BACKGROUND ACTION WITH TIMEOUT
                        #if (TRANS_ASYNCH_WRAPPED==1)
                            setPowerLevel_dBm_iff_asynch (i_radio, RXTX_context.timing_transx, RXTX_context.some_rfm69_internals, TX_context.TX_appPowerLevel_dB);
                        #else
                            RXTX_context.timing_transx.start_time_trans1 = setPowerLevel_dBm_iff_trans1 (RXTX_context.timing_transx.timed_out_trans1to2, i_radio, TX_context.TX_appPowerLevel_dBm);
                            // MUST be run now:
                            do_sessions_trans2to3 (i_radio, RXTX_context.timing_transx, RXTX_context.return_trans3);
                        #endif
                        RXTX_context.radio_log_value = RXTX_context.timing_transx.radio_log_value;
                    #else
                        i_radio.uspi_setPowerLevel_dBm (TX_context.TX_appPowerLevel_dBm); // Should stop the sound in my speakers!
                    #endif
                #endif
            } else {}

            #if ((TEST_01_FOLLOW_ADDRESS==1) or (TEST_01_LISTENTOALL==1))
                if (TX_context.TX_appSeqCnt == 10) {
                    debug_print ("\nKEY2\n", TX_context.TX_gatewayid);
                    debug_print ("%s", char_leading_space_str);
                    #define KEY2 "OM11-Aquarium-2"
                    #if (CLIENT_ALLOW_SESSION_TYPE_TRANS==1)
                        // ASYNCH CALL AND BACKGROUND ACTION WITH TIMEOUT
                        #if (TRANS_ASYNCH_WRAPPED==1)
                            encrypt16_iff_asynch (i_radio, RXTX_context.timing_transx, RXTX_context.some_rfm69_internals, KEY2);
                        #else
                            RXTX_context.timing_transx.start_time_trans1 = encrypt16_iff_trans1 (RXTX_context.timing_transx.timed_out_trans1to2, i_radio, KEY2);
                            // MUST be run now:
                            do_sessions_trans2to3 (i_radio, RXTX_context.timing_transx, RXTX_context.return_trans3);
                        #endif
                        RXTX_context.radio_log_value = RXTX_context.timing_transx.radio_log_value;
                    #else
                        i_radio.uspi_encrypt16 (KEY2, KEY_LEN);
                    #endif

                } else if (TX_context.TX_appSeqCnt == 20) {
                    debug_print ("\nKEY again\n", "KEY again");
                    debug_print ("%s", char_leading_space_str);
                    #if (CLIENT_ALLOW_SESSION_TYPE_TRANS==1)
                        // ASYNCH CALL AND BACKGROUND ACTION WITH TIMEOUT
                        #if (TRANS_ASYNCH_WRAPPED==1)
                            encrypt16_iff_asynch (i_radio, RXTX_context.timing_transx, RXTX_context.some_rfm69_internals, RXTX_context.radio_init.key);
                        #else
                            RXTX_context.timing_transx.start_time_trans1 = encrypt16_iff_trans1 (RXTX_context.timing_transx.timed_out_trans1to2, i_radio, RXTX_context.radio_init.key);
                            // MUST be run now:
                            do_sessions_trans2to3 (i_radio, RXTX_context.timing_transx, RXTX_context.return_trans3);
                        #endif
                        RXTX_context.radio_log_value = RXTX_context.timing_transx.radio_log_value;
                    #else
                        i_radio.uspi_encrypt16 (RXTX_context.radio_init.key, KEY_LEN);
                    #endif
                } else if (TX_context.TX_appSeqCnt == 25) {
                    TX_context.TX_gatewayid = 58;
                    debug_print ("\ngatewayid %u(%02X)\n", TX_context.TX_gatewayid, TX_context.TX_gatewayid);
                    debug_print ("%s", char_leading_space_str);
                } else if (TX_context.TX_appSeqCnt == 26) {
                    debug_print ("%s\n", "=== ZEROED: Time max 0, mean-1");
                    debug_print ("%s", char_leading_space_str);
                    divTime.max_diffTime_ms  = 0;
                    divTime.mean_diffTime_ms = 0;
                    #if (RADIO_IF_FULL == 1)
                        // i_radio.uspi_readAllRegs();
                    #endif
                } else {}
            #endif

            for (unsigned index = 0; index < PACKET_LEN32; index++) {
                 RXTX_context.PACKET.u.packet_u2_uint32_arr[index] = PACKET_INIT_VAL32;
            }

            RXTX_context.PACKET.u.packet_u3.appHeading.numbytes_of_full_payload = PACKET_LEN08;
            RXTX_context.PACKET.u.packet_u3.appHeading.version_of_full_payload  = VERSION_OF_APP_PAYLOAD_02;
            RXTX_context.PACKET.u.packet_u3.appHeading.num_of_this_app_payload  = NUM_OF_THIS_APP_PAYLOAD_01;

            RXTX_context.PACKET.u.packet_u3.appNODEID = NODEID; // Comes from MASTER_ID
            RXTX_context.PACKET.u.packet_u3.appPowerLevel_dBm = TX_context.TX_appPowerLevel_dBm;

            RXTX_context.PACKET.u.packet_u3.appSeqCnt = TX_context.TX_appSeqCnt;

            debug_print("TXappSeqCnt %u\n", TX_context.TX_appSeqCnt);

            #if (CLIENT_ALLOW_SESSION_TYPE_TRANS==1)
                // ASYNCH CALL AND BACKGROUND ACTION WITH TIMEOUT
                #if (TRANS_ASYNCH_WRAPPED==1)
                    interruptAndParsingResult = send_iff_asynch (i_radio, RXTX_context.timing_transx, TX_context.TX_gatewayid, RXTX_context.PACKET);
                #else
                    RXTX_context.timing_transx.start_time_trans1 = i_radio.send_iff_trans1 (RXTX_context.timing_transx.timed_out_trans1to2, i_radio, TX_context.TX_gatewayid, RXTX_context.PACKET);
                    // MUST be run now:
                    do_sessions_trans2to3 (i_radio, RXTX_context.timing_transx, RXTX_context.return_trans3);
                #endif
                RXTX_context.radio_log_value = RXTX_context.timing_transx.radio_log_value;
             #else
                 TX_context.waitForIRQInterruptCause = i_radio.uspi_send (TX_context.TX_gatewayid, RXTX_context.PACKET); // element CommHeaderRFM69 is not taken from here, so don't fill it in
             #endif

            #if (CLIENT_ALLOW_SESSION_TYPE_TRANS==1)
                // CALL IFF, BUT IN ANY CASE NO SPI (so no _trans1)
                {RXTX_context.some_rfm69_internals.error_bits, RXTX_context.is_new_error} = getAndClearErrorBits_iff (RXTX_context.timing_transx.timed_out_trans1to2, i_radio);
            #else
                {RXTX_context.some_rfm69_internals.error_bits, RXTX_context.is_new_error} = i_radio.getAndClearErrorBits();
            #endif

            if (RXTX_context.some_rfm69_internals.error_bits != ERROR_BITS_NONE) {
                debug_print ("RFM69 err3 new %u code %04X\n", RXTX_context.is_new_error, RXTX_context.some_rfm69_internals.error_bits);
            } else {}

            TX_context.sendPacket_seconds_cntdown = SEND_PACKET_ON_NO_CHANGE_TIMOEUT_SECONDS - 1;

        } else {
            if (TX_context.sendPacket_seconds_cntdown > seconds_since_last_call) {
                TX_context.sendPacket_seconds_cntdown -= seconds_since_last_call; // Down
            } else {
                TX_context.sendPacket_seconds_cntdown = 0;
            }
        }

        {
            time32_t endTime_tics;
            time32_t diffTime_tics;
            time32_t diffTime_ms;

            timer tmr_now; // Not need to parameterise this as it doesn't take an extra hw timer
            tmr_now :> endTime_tics; // NOW TIME

            diffTime_tics = endTime_tics - startTime_ticks;
            diffTime_ms   = diffTime_tics / XS1_TIMER_KHZ;

            divTime.max_diffTime_ms = max (divTime.max_diffTime_ms, diffTime_ms);
            divTime.num_diffTime_ms++;

            // IIR very slow moving average::
            divTime.sum_diffTime_ms          = divTime.sum_diffTime_ms + diffTime_ms; // running total or partial sum
            divTime.mean_diffTime_ms         = divTime.sum_diffTime_ms / divTime.num_diffTime_ms;
            divTime.changed_mean_diffTime_ms = (divTime.prev_mean_diffTime_ms != divTime.mean_diffTime_ms);
            divTime.prev_mean_diffTime_ms    = divTime.mean_diffTime_ms;

            #if (DEBUG_PRINT_TIME_USED == 1)
                // I can hear sending in my speakers when full power when this printing is going on!
                // Probably because this delays IRQ handling!
                debug_print ("Time used %06d ms (%08X - %08X). Time max %d, mean-%u %d ms\n",
                        diffTime_ms, endTime_tics, startTime_ticks,
                        divTime.max_diffTime_ms,            // 345
                        divTime.changed_mean_diffTime_ms,
                        divTime.mean_diffTime_ms);          // 255
            #endif
        }
    #endif
}

void display_screen_store_RX_context_values (
        display_context_t &display_context,
        RX_context_t      &?RX_context)
{
    #if (IS_MYTARGET_SLAVE==1)
        RX_context.num_appSeqCnt_notSeen_inDisplay       = RX_context.num_appSeqCnt_notSeen;
        display_context.ultimateIRQclearCnt_notSeen_inDisplay = display_context.ultimateIRQclearCnt;
    #endif
}

void reset_values (
        display_context_t &display_context,
        RX_context_t      &?RX_context,
        RXTX_context_t    &RXTX_context) {

    #if (IS_MYTARGET_SLAVE==1)
        for (unsigned index = 0; index < _USERMAKEFILE_LIB_RFM69_XC_PAYLOAD_LEN08; index++) {
            RX_context.RX_radio_payload_prev.u.payload_u1_uint8_arr[index] = PACKET_INIT_VAL08;
            RX_context.RX_radio_payload.u.payload_u1_uint8_arr[index]      = PACKET_INIT_VAL08; // So that SCREEN_RX_MAIN_TIME_TEMP_ETC is zeroed
        }
                                                                                              // RFM69=002 redefined in _Aquarium_1_x/src/_rfm69_commprot.h
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

        RX_context.nowRSSI_weakest   = RSSI_DB_STRONGEST;  // -100 is weaker
        RX_context.nowRSSI_strongest = RSSI_DB_WEAKEST;    // -80 is stronger

        RX_context.num_appSeqCnt_notSeen = 0;
        RX_context.num_received = 0;

        RX_context.num_radioCRC16errs = 0;
        RX_context.num_appCRC32errs = 0;

        #if (GET_RADIO_DEBUG_REGS==1)
            RX_context.debug_state         = debug_void; // So that the debug_none is the initial use
            display_context.debug_r_button = false;
        #endif

        display_context.ultimateIRQclearCnt = 0;
        display_context.ultimateIRQclearCnt_notSeen_inDisplay = 0;
        display_context.senderid_not_displayed_cnt = 0;
    #endif

    RXTX_context.error_bits_history = 0;
}

void RFM69_client (
                  chanend                  c_irq_update,
          client  radio_if_t               i_radio,
          client  blink_and_watchdog_if_t  i_blink_and_watchdog,
          server  button_if                i_button_in[BUTTONS_NUM_CLIENTS],
          client  i2c_internal_commands_if i_i2c_internal_commands,
          out port                         p_display_notReset)
{
    timer tmr;

    // All declared, used or not, since nullable references not allowed for structs

    RXTX_context_t        RXTX_context;
    display_context_t     display_context;
    debug_print_context_t debug_print_context;
    divTime_t             divTime;
    unsigned              seconds_since_last_call = 0;

    debug_print_context.debug_print_rx_2_done = false;

    RXTX_context.interruptCnt         = 0;
    RXTX_context.radio_init.nodeID    = NODEID; // Comes from SHARED_ID from _Aquarium. Will cause messageReceivedOk_IRQ if received this or RF69_BROADCAST_ADDR
    RXTX_context.radio_init.RegFrf    = MY_RFM69_FREQ_REGS;
    RXTX_context.radio_init.isRFM69HW = IS_RFM69HW_HCW; // Must be true or else my Adafruit high power module won't work!
    //
    for (unsigned i=0; i < KEY_LEN; i++) {
        RXTX_context.radio_init.key[i] = KEY[i];
    }

    #if (CLIENT_ALLOW_SESSION_TYPE_TRANS==1)
        RXTX_context.timing_transx.timed_out_trans1to2          = false; // Set       by do_sessions_trans2to3, but we need to clear it first
        RXTX_context.timing_transx.maxtime_used_us_trans1to2    = 0;     // Increased by do_sessions_trans2to3, but we need to zero it first
        RXTX_context.timing_transx.maxtime_allowed_ms_trans1to2 = CLIENT_WAIT_FOR_RADIO_MAX_MS; // Set only here
        RXTX_context.timing_transx.radio_log_value              = 0; // Overwritten later on
    #endif

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
        RX_context.num_appSeqCnt_notSeen = 0;
        RX_context.num_appSeqCnt_notSeen_inDisplay = 0;
        RX_context.num_received = 0;
        RX_context.num_radioCRC16errs = 0;
        RX_context.num_appCRC32errs = 0;
        RX_context.seconds_since_last_received = 0;
        RX_context.appSeqCnt = 0;
        RX_context.appSeqCnt_prev = 0;
        RX_context.nowRSSI = 0; // Observe that it's signed so "0" and "-82" would align with print %d. This is fine!
        #if (GET_RADIO_DEBUG_REGS==1)
            for (unsigned i = 0; i < NUM_DEBUG_BYTES; i++) {
                RX_context.debug_data.data[i] = 0;
                RX_context.debug_data_prev.data[i] = 0;
            }
            RX_context.debug_state = debug_just_read_some_registers;
        #endif

        for (unsigned index = 0; index < _USERMAKEFILE_LIB_RFM69_XC_PAYLOAD_LEN08; index++) {
            RX_context.RX_radio_payload.u.payload_u1_uint8_arr[index] = PACKET_INIT_VAL08;
        }

        RX_context.allow_10_sek_timeout = true;
        RX_context.is_watchdog_blinking = false;
        display_context.senderid_not_displayed_cnt = 0;
        display_context.senderid_displayed_now = MASTER_ID_AQUARIUM;
        display_context.senderid_not_displayed_now = MASTER_ID_BLACK_BOARD;

    #else
        #error MUST BE ONE of them! To code for both, recode somewhat
    #endif

    reset_values (display_context, RX_CONTEXT, RXTX_context);

    divTime.max_diffTime_ms = 0;
    divTime.sum_diffTime_ms = 0;
    divTime.mean_diffTime_ms = 0;
    divTime.prev_mean_diffTime_ms = 0;
    divTime.changed_mean_diffTime_ms = false;
    divTime.num_diffTime_ms = 0;

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
                (IS_MYTARGET == IS_MYTARGET_STARTKIT)           ? "startKIT" :
                (IS_MYTARGET == IS_MYTARGET_XCORE_200_EXPLORER) ? "xplorKIT" : "none!",
                NODEID,
                GATEWAYID,
                SEND_PACKET_ON_NO_CHANGE_TIMOEUT_SECONDS,
                RFM69_DRIVER_VERSION_STR,
                RFM69_CLIENT_VERSION_STR);
    #elif (IS_MYTARGET_SLAVE==1)
        // xplorKIT as SLAVE[90]: RFM69-driver[0.8.4], RFM69-client[0.8.36]
        debug_print ("\n%s as SLAVE[%u]: RFM69-driver[%s], RFM69-client[%s]\n",
                (IS_MYTARGET == IS_MYTARGET_STARTKIT)           ? "startKIT" :
                (IS_MYTARGET == IS_MYTARGET_XCORE_200_EXPLORER) ? "xplorKIT" : "none!",
                NODEID,
                RFM69_DRIVER_VERSION_STR,
                RFM69_CLIENT_VERSION_STR);
    #endif

    debug_print ("Built %s [%s] with radio CRC %s IRQ and %s sent\n\n",
            __DATE__,
            __TIME__,
            (SEMANTICS_DO_CRC_ERR_NO_IRQ == 1) ? "no" : "with",
            (SEMANTICS_DO_LOOP_FOR_RF_IRQFLAGS2_PACKETSENT == 1) ? "loop for" : "state for");

    // Display matters
    {
        Adafruit_GFX_constructor (SSD1306_LCDWIDTH, SSD1306_LCDHEIGHT);
        Adafruit_SSD1306_i2c_begin (i_i2c_internal_commands, p_display_notReset);
        display_context.state = is_on;
        display_context.display_screen_name         = SCREEN_WELCOME;
        display_context.display_screen_name_last_on = SCREEN_WELCOME;
        display_context.display_screen_direction_up = true;

        for (int iof_button = 0; iof_button < BUTTONS_NUM_CLIENTS; iof_button++) {
            display_context.buttons_state[iof_button].pressed_now = false;
            display_context.buttons_state[iof_button].pressed_for_10_seconds = false;
            display_context.buttons_state[iof_button].inhibit_released_once = false;
        }

        Display_screen (display_context, RX_CONTEXT, RXTX_context, USE_THIS, i_i2c_internal_commands);

        display_context.allow_auto_switch_to_screen_rx_main_time_temp_etc = true;
    }

    // Radio matters

    i_radio.uspi_do_aux_adafruit_rfm69hcw_RST_pulse (MASKOF_SPI_AUX0_RST);
    i_radio.uspi_initialize (RXTX_context.radio_init);

    RXTX_context.device_type = i_radio.uspi_getDeviceType(); // ERROR_BITNUM_DEVICE_TYPE if not 0x24
    debug_print ("\n---> DEVICE TYPE 0x%02X <---\n\n", RXTX_context.device_type);

    #if (CLIENT_ALLOW_SESSION_TYPE_TRANS==1)
        // CALL IFF, BUT IN ANY CASE NO SPI (so no _trans1)
        {RXTX_context.some_rfm69_internals.error_bits, RXTX_context.is_new_error} = getAndClearErrorBits_iff (RXTX_context.timing_transx.timed_out_trans1to2, i_radio);
    #else
        {RXTX_context.some_rfm69_internals.error_bits, RXTX_context.is_new_error} = i_radio.getAndClearErrorBits();
    #endif

    if (RXTX_context.some_rfm69_internals.error_bits == ERROR_BITS_NONE) {

        i_radio.uspi_setHighPower (RXTX_context.radio_init.isRFM69HW);
        i_radio.uspi_encrypt16 (RXTX_context.radio_init.key, KEY_LEN);
        #if (IS_MYTARGET_SLAVE==1)
            i_radio.setListenToAll (RX_context.doListenToAll);
        #endif
        i_radio.uspi_setFrequencyRegister (MY_RFM69_FREQ_REGS); // Only needed if different from MY_RFM69_FREQ_REGS, since that done in radio_init

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

        i_radio.uspi_receiveDone(); // To have setMode(RF69_MODE_RX) done (via receiveBegin)
    } else {}

    #if (CLIENT_ALLOW_SESSION_TYPE_TRANS==1)
        // CALL IFF, BUT IN ANY CASE NO SPI (so no _trans1)
        {RXTX_context.some_rfm69_internals.error_bits, RXTX_context.is_new_error} = getAndClearErrorBits_iff (RXTX_context.timing_transx.timed_out_trans1to2, i_radio);
    #else
        {RXTX_context.some_rfm69_internals.error_bits, RXTX_context.is_new_error} = i_radio.getAndClearErrorBits();
    #endif

    if (RXTX_context.some_rfm69_internals.error_bits != ERROR_BITS_NONE) {
        debug_print ("RFM69 err1 new %u code %04X\n", RXTX_context.is_new_error, RXTX_context.some_rfm69_internals.error_bits);
    } else {}

    #if (IS_MYTARGET_SLAVE==1)
        debug_print ("\nRADIO RX IS %s\n", // "RADIO RX IS AKVA" or "RADIO RX IS KORT"
             (display_context.senderid_displayed_now == MASTER_ID_BLACK_BOARD) ? "KORT" : "AKVA");
    #endif

    i_blink_and_watchdog.init_watchdog_ok (
            XCORE_200_EXPLORER_LED_GREEN_BIT_MASK bitor XCORE_200_EXPLORER_LED_RGB_GREEN_BIT_MASK,
            AQUARIUM_RFM69_RECEIVE_TIMOUT_SEC * 1000, // 10 seconds. May lose two ok. Max 21 secs
            200);

    irq_update_e irq_update;
    bool         timeout_i_radio_usage_allowed = true;

    tmr :> divTime.time_ticks; // First sending now

    while (1) {
        select {
            case c_irq_update :> irq_update : {

                debug_print ("IRQ %u UPDATE %s\n",
                        timeout_i_radio_usage_allowed,
                        (irq_update == pin_gone_high)          ? "pin_gone_high" :
                        (irq_update == pin_still_high_timeout) ? "pin_still_high_timeout" :
                                                                 "pin_gone_low");
                time32_t then_tics, now_tics;
                tmr :> then_tics;

                if (irq_update == pin_gone_high) {

                    #if (IS_MYTARGET_SLAVE == 1)
                        #if (CLIENT_ALLOW_SESSION_TYPE_TRANS==1)
                            // ASYNCH CALL AND BACKGROUND ACTION WITH TIMEOUT
                            #if (TRANS_ASYNCH_WRAPPED==1)
                                RX_context.nowRSSI = readRSSI_dBm_iff_asynch (i_radio, RXTX_context.timing_transx, FORCETRIGGER_OFF);
                            #else
                                RXTX_context.timing_transx.start_time_trans1 = readRSSI_dBm_iff_trans1 (RXTX_context.timing_transx.timed_out_trans1to2, i_radio, FORCETRIGGER_OFF);
                                // MUST be run now:
                                do_sessions_trans2to3 (i_radio, RXTX_context.timing_transx, RXTX_context.return_trans3);
                                RX_context.nowRSSI = RXTX_context.return_trans3.u_out.rssi_dBm;
                            #endif
                            RXTX_context.radio_log_value = RXTX_context.timing_transx.radio_log_value;
                        #else
                            RX_context.nowRSSI = i_radio.uspi_readRSSI_dBm (FORCETRIGGER_OFF);
                        #endif
                    #endif

                    RFM69_handle_irq (
                       RX_CONTEXT,
                       TX_CONTEXT,
                       RXTX_context,
                       display_context,
                       i_radio,
                       i_blink_and_watchdog,
                       i_i2c_internal_commands,
                       debug_print_context);

                    timeout_i_radio_usage_allowed = false;                     // Waiting for pin_gone_low
                } else if (irq_update == pin_gone_low) {
                    timeout_i_radio_usage_allowed = true;                      // Ready
                } else if (irq_update == pin_still_high_timeout) {
                    #if (CLIENT_ALLOW_SESSION_TYPE_TRANS==1)
                        // ASYNCH CALL AND BACKGROUND ACTION WITH TIMEOUT
                        #if (TRANS_ASYNCH_WRAPPED==1)
                            ultimateIRQclear_iff_asynch (i_radio, RXTX_context.timing_transx);
                        #else
                            RXTX_context.timing_transx.start_time_trans1 = ultimateIRQclear_iff_trans1 (RXTX_context.timing_transx.timed_out_trans1to2, i_radio);
                            // MUST be run now:
                            do_sessions_trans2to3 (i_radio, RXTX_context.timing_transx, RXTX_context.return_trans3);
                        #endif
                        RXTX_context.radio_log_value = RXTX_context.radio_log_value;
                    #else
                       i_radio.uspi_ultimateIRQclear();
                    #endif

                    display_context.ultimateIRQclearCnt++;
                    timeout_i_radio_usage_allowed = false; // Still let's wait for the consequential pin_gone_low
                } else {} // Never here

                tmr :> now_tics;
                debug_print ("IRQ HANDLING %u ms and %u\n", (now_tics - then_tics) / XS1_TIMER_KHZ, display_context.ultimateIRQclearCnt);

            } break;

            case tmr when timerafter (divTime.time_ticks) :> time32_t startTime_ticks: {
                seconds_since_last_call++; // Since ONE_SECOND_TICKS used below

                #if (IS_MYTARGET_SLAVE == 1)
                    RX_context.seconds_since_last_received += seconds_since_last_call; // about, anyhow, since we don't reset divTime.time_ticks in pin_rising
                    if (timeout_i_radio_usage_allowed) {
                        if (RX_context.seconds_since_last_received > ((AQUARIUM_RFM69_REPEAT_SEND_EVERY_SEC * 5)/2)) { // 2.5 times 4 seconds = 10 seconds
                            // TODO is this really necessary after some mid January 2019:
                            #if (CLIENT_ALLOW_SESSION_TYPE_TRANS==1)
                                // ASYNCH CALL AND BACKGROUND ACTION WITH TIMEOUT
                                #if (TRANS_ASYNCH_WRAPPED==1)
                                    if (receiveDone_iff_asynch (i_radio, RXTX_context.timing_transx)) {}
                                #else
                                    RXTX_context.timing_transx.start_time_trans1 = receiveDone_iff_trans1 (RXTX_context.timing_transx.timed_out_trans1to2, i_radio);
                                    // MUST be run now:
                                    do_sessions_trans2to3 (i_radio, RXTX_context.timing_transx, RXTX_context.return_trans3);
                                #endif
                                RXTX_context.radio_log_value = RXTX_context.timing_transx.radio_log_value;

                            #else
                                if (i_radio.uspi_receiveDone()) {} // In the _Aquarium_rfm69_client this is run after every i_radio.uspi_handleSPIInterrupt
                            #endif

                            debug_print ("%s", "NO RX FOR 10 SECONDS\n");
                            RX_context.seconds_since_last_received = 0;
                        } else {}
                    } else {}
                    RX_context.is_watchdog_blinking = i_blink_and_watchdog.is_watchdog_blinking();

                    if (RX_context.is_watchdog_blinking) {
                        debug_print ("WATCHDOG BLINKING T %u\n", RX_context.seconds_since_last_received);
                    } else {
                        debug_print ("T %u\n", RX_context.seconds_since_last_received);  // "T 1", "T 2" etc.
                    }
                #endif

                if (timeout_i_radio_usage_allowed) {
                    RFM69_handle_timeout (
                            divTime,
                            startTime_ticks,
                            RX_CONTEXT,
                            TX_CONTEXT,
                            RXTX_context,
                            display_context,
                            seconds_since_last_call,
                            i_radio,
                            i_blink_and_watchdog,
                            i_i2c_internal_commands);
                    seconds_since_last_call = 0;
                } else {}

                divTime.time_ticks += ONE_SECOND_TICKS; // FUTURE TIMEOUT
                // observe AQUARIUM_RFM69_REPEAT_SEND_EVERY_SEC

                // If diffTime_ms is larger than ONE_SEC_TICS the select will be timed out immediately N times until it's AFTER again

            } break;

            // xTIMEcomposer issues an error if this is guarded
            // If any cod in here should talk over i_radio (that might side effect into IRQ) then it should be
            // protected by "not timeout_i_radio_usage_allowed" instead
            //
            case i_button_in[int iof_button].button (const button_action_t button_action) : {

                display_context.buttons_state[iof_button].pressed_now =            (button_action == BUTTON_ACTION_PRESSED);
                display_context.buttons_state[iof_button].pressed_for_10_seconds = (button_action == BUTTON_ACTION_PRESSED_FOR_10_SECONDS);

                switch (iof_button) {
                    case IOF_BUTTON_LEFT: { // Toggles display on and off, back with same screen
                        if (display_context.buttons_state[iof_button].inhibit_released_once) {
                            display_context.buttons_state[iof_button].inhibit_released_once = false;
                        } else if (button_action == BUTTON_ACTION_RELEASED) {
                            if (display_context.state == is_on) { // now switch off
                                display_context.display_screen_name_last_on = display_context.display_screen_name; // PUSH it
                                display_context.display_screen_name         = SCREEN_DARK;
                                Display_screen (display_context, RX_CONTEXT, RXTX_context, USE_PREV, i_i2c_internal_commands); // First this so that SCREEN_DARK runs..
                                display_context.state                       = is_off;                                          // ..then is_off

                                display_screen_store_RX_context_values (display_context, RX_CONTEXT);
                            } else { // is_off: now switch on
                                display_context.display_screen_name         = display_context.display_screen_name_last_on;     // PULL it
                                display_context.state                       = is_on;                                           // First is_on..
                                Display_screen (display_context, RX_CONTEXT, RXTX_context, USE_PREV, i_i2c_internal_commands); // ..then this so that screen goes on
                            }
                        } else if (button_action == BUTTON_ACTION_PRESSED) {
                            if (display_context.buttons_state[IOF_BUTTON_CENTER].pressed_now) {
                                if (display_context.state == is_on) {
                                    display_context.buttons_state[iof_button].inhibit_released_once = true;
                                    display_context.display_screen_direction_up = not display_context.display_screen_direction_up;
                                } else {}
                            } else {}
                        } else {}
                    } break;

                    case IOF_BUTTON_CENTER: { // Next screen and wraps around
                        if (button_action == BUTTON_ACTION_RELEASED) {
                            if (display_context.state == is_on) { // now switch off
                                display_context.display_screen_name_last_on = display_context.display_screen_name;

                                if (display_context.display_screen_direction_up) {
                                    display_context.display_screen_name = (display_context.display_screen_name + 1) % SCREEN_DARK;
                                } else {
                                    if (display_context.display_screen_name == 0) {
                                        display_context.display_screen_name = SCREEN_DARK - 1;
                                    } else {
                                        display_context.display_screen_name = (display_context.display_screen_name - 1);
                                    }
                                }

                                debug_print ("SCREEN NAME %u\n", display_context.display_screen_name);
                                Display_screen (display_context, RX_CONTEXT, RXTX_context, USE_PREV, i_i2c_internal_commands);

                                display_screen_store_RX_context_values (display_context, RX_CONTEXT);
                            } else {}
                        } else if (button_action == BUTTON_ACTION_PRESSED) {
                            // No code
                        } else {}
                    } break;

                    case IOF_BUTTON_RIGHT: {

                        if (button_action == BUTTON_ACTION_RELEASED) {
                            i_blink_and_watchdog.reset_watchdog_ok();
                            if (display_context.state == is_on) {
                                #if (GET_RADIO_DEBUG_REGS==1)
                                    if (display_context.display_screen_name == SCREEN_RADIO_DEBUG_REGS) {
                                        #if (IS_MYTARGET_SLAVE == 1)
                                            if (timeout_i_radio_usage_allowed) {
                                                // debug_mode_0_1 SOLVED THE PROBLEM!
                                                display_context.debug_r_button = true;
                                                RX_context.debug_state = (RX_context.debug_state + 1) % debug_void;

                                                #if (CLIENT_ALLOW_SESSION_TYPE_TRANS==1)
                                                     // ASYNCH CALL AND BACKGROUND ACTION WITH TIMEOUT
                                                     #if (TRANS_ASYNCH_WRAPPED==1)
                                                         RX_context.debug_data = getDebug_iff_asynch (i_radio, RXTX_context.timing_transx, RX_context.debug_state);
                                                     #else
                                                         RXTX_context.timing_transx.start_time_trans1 = getDebug_iff_trans1 (RXTX_context.timing_transx.timed_out_trans1to2, i_radio, RX_context.debug_state);
                                                         // MUST be run now:
                                                         do_sessions_trans2to3 (i_radio, RXTX_context.timing_transx, RXTX_context.return_trans3);
                                                         RX_context.debug_data = RXTX_context.return_trans3.u_out.debug_data;
                                                     #endif
                                                     RXTX_context.radio_log_value = RXTX_context.timing_transx.radio_log_value;
                                                 #else
                                                     RX_context.debug_data = i_radio.uspi_getDebug (RX_context.debug_state);
                                                 #endif

                                                #if (CLIENT_ALLOW_SESSION_TYPE_TRANS==1)
                                                    // CALL IFF, BUT IN ANY CASE NO SPI (so no _trans1)
                                                    {RXTX_context.some_rfm69_internals.error_bits, RXTX_context.is_new_error} = getAndClearErrorBits_iff (RXTX_context.timing_transx.timed_out_trans1to2, i_radio);
                                                #else
                                                    {RXTX_context.some_rfm69_internals.error_bits, RXTX_context.is_new_error} = i_radio.getAndClearErrorBits();
                                                #endif

                                                RXTX_context.error_bits_history or_eq RXTX_context.some_rfm69_internals.error_bits;

                                                Display_screen (display_context, RX_context, RXTX_context, USE_PREV, i_i2c_internal_commands);
                                                display_context.debug_r_button = false;
                                            } else {
                                                display_context.display_screen_name == SCREEN_HJELP;
                                                Display_screen (display_context, RX_context, RXTX_context, USE_PREV, i_i2c_internal_commands);
                                            }
                                        #endif
                                    } else
                                #endif
                                    if (display_context.display_screen_name == SCREEN_RX_DISPLAY_OVERSIKT) {
                                    #if (IS_MYTARGET_SLAVE == 1)
                                        debug_print ("\nRADIO RX WAS %s ",
                                                (display_context.senderid_displayed_now == MASTER_ID_BLACK_BOARD) ? "KORT" : "AKVA");
                                        t_swap (uint8_t, display_context.senderid_not_displayed_now, display_context.senderid_displayed_now);
                                        reset_values (display_context, RX_CONTEXT, RXTX_context);
                                        debug_print ("NOW %s = address %u\n\n",
                                                (display_context.senderid_displayed_now == MASTER_ID_BLACK_BOARD) ? "KORT" : "AKVA",
                                                 display_context.senderid_displayed_now);
                                        Display_screen (display_context, RX_context, RXTX_context, USE_PREV, i_i2c_internal_commands);
                                    #endif
                                } else if (display_context.display_screen_name == SCREEN_WELCOME) {
                                    #if (IS_MYTARGET_SLAVE == 1)
                                        RX_context.allow_10_sek_timeout = not RX_context.allow_10_sek_timeout;
                                        i_blink_and_watchdog.enable_watchdog (RX_context.allow_10_sek_timeout);
                                        Display_screen (display_context, RX_context, RXTX_context, USE_PREV, i_i2c_internal_commands);
                                    #endif
                                } else {}
                            } else {}
                        } else if (button_action == BUTTON_ACTION_PRESSED_FOR_10_SECONDS) {
                            reset_values (display_context, RX_CONTEXT, RXTX_context);
                            { // Make display blink.
                                display_screen_name_t display_screen_name_copy = display_context.display_screen_name;
                                display_context.display_screen_name = SCREEN_DARK;
                                #if (IS_MYTARGET_SLAVE == 1)
                                    Display_screen (display_context, RX_context, RXTX_context, USE_PREV, i_i2c_internal_commands);
                                #else
                                    Display_screen (display_context, null, RXTX_context, USE_PREV, i_i2c_internal_commands);
                                #endif
                                delay_milliseconds(100); // It blinks also without this
                                display_context.display_screen_name = display_screen_name_copy;
                                #if (IS_MYTARGET_SLAVE == 1)
                                    Display_screen (display_context, RX_context, RXTX_context, USE_PREV, i_i2c_internal_commands);
                                #else
                                    Display_screen (display_context, null, RXTX_context, USE_PREV, i_i2c_internal_commands);
                                #endif
                            }
                        }
                    } break;
                }
            } break;
        }
    }
}

