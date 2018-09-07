/*
 * _rfm69_commprot.h
 *
 *  Created on: 30. aug. 2018
 *      Author: teig
 */

#ifndef _RFM69_COMMPROT_H_
#define _RFM69_COMMPROT_H_

// radio
//
typedef uint8_t  month_r;
typedef uint8_t  day_r;
typedef uint8_t  hour_r;
typedef uint8_t  minute_r;
typedef uint8_t  second_r;
typedef uint8_t  heater_on_percent_r;
typedef uint8_t  heater_on_watt_r;
typedef uint8_t  light_control_scheme_r;
typedef uint16_t year_r;
typedef uint16_t error_bits_r;
typedef int16_t  onetenthDegC_r;
typedef uint16_t voltage_onetenthV_r;
typedef uint16_t application_version_num_r;
typedef uint8_t  light_intensity_thirds_r;
typedef uint8_t  light_composition_r;
typedef uint16_t num_days_since_start_r;

typedef struct { // Size must be modulo 4                                   // WORD ALIGN
    num_days_since_start_r    num_days_since_start;                         // 01,02       Saving 4 bytes for year, month and day (start date is seen in SCREEN_6_KONSTANTER)
    hour_r                    hour;                                         //       03
    minute_r                  minute;                                       //          04
    second_r                  second;                                       // 05
    heater_on_percent_r       heater_on_percent;                            //    06
    heater_on_watt_r          heater_on_watt;                               //       07
    light_control_scheme_r    light_control_scheme;                         //          08 num_minutes_left.. (two counters) not exported
    error_bits_r              error_bits_now;                               // 09-10       (Intermediate errors will disappeear)
    error_bits_r              error_bits_history;                           //       11-12 (bitor'ed from above but both cleared with right button 10 seconds)
    onetenthDegC_r            i2c_temp_heater_onetenthDegC;                 // 13-14
    onetenthDegC_r            i2c_temp_ambient_onetenthDegC;                //       15-16
    onetenthDegC_r            i2c_temp_water_onetenthDegC;                  // 17-18
    onetenthDegC_r            temp_heater_mean_last_cycle_onetenthDegC;     //       19-20
    onetenthDegC_r            internal_box_temp_onetenthDegC;               // 21-22
    voltage_onetenthV_r       rr_24V_heat_onetenthV;                        //       23-24
    voltage_onetenthV_r       rr_12V_LEDlight_onetenthV;                    // 25-26
    application_version_num_r application_version_num;                      //       27-28
    light_intensity_thirds_r  light_intensity_thirds_front;                 // 29
    light_intensity_thirds_r  light_intensity_thirds_center;                //    30
    light_intensity_thirds_r  light_intensity_thirds_back;                  //       31
    light_composition_r       light_composition;                            //          32
    //                                                                                  ##
    // _USERMAKEFILE_LIB_RFM69_XC_PAYLOAD_LEN08                                         32 -> SET IN makefile -> Must be modulo 4
    // _USERMAKEFILE_LIB_RFM69_XC_PAYLOAD_LEN08 is checked by System_Task               ##
    //                            and may cause low code size if it fails               ##
    // If PACKET_LEN08 of packet_t in /lib_rfm69_xc/rfm69_commmprot.h is 20 a           ##
    //                                MAX_SX1231H_PACKET_LEN is 61 then max here is:    41 (ie. 40 for modulo 4 requirement)
    // Also there: PACKET_LEN_FACIT (20 + _USERMAKEFILE_LIB_RFM69_XC_PAYLOAD_LEN08)
    //
} payload_u0_t;

typedef struct {  // Size must be modulo 4
    union {
        payload_u0_t payload_u0;
        uint8_t      payload_u1_uint8_arr[_USERMAKEFILE_LIB_RFM69_XC_PAYLOAD_LEN08];
    } u;
} payload_t;

#endif /* RFM69_COMMPROT_H_ */
