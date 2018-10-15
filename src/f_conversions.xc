/*
 * f_conversions.xc
 *
 *  Created on: 28. nov. 2016
 *      Author: teig
 */

#define INCLUDES
#ifdef INCLUDES
#include <platform.h>
#include <xs1.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <xccompat.h> // REFERENCE_PARAM
#include <string.h>   // memset.
#include <iso646.h>
#include <errno.h>
#include <limits.h>

#include "i2c.h"

#include "_version.h"
#include "_globals.h"
#include "param.h"
#include "_texts_and_constants.h"
#include "f_conversions.h"
#endif

#define DEBUG_PRINT_F_CONVERSIONS_MEAN 0 // Cost 1.3k
#define debug_print(fmt, ...) do { if(DEBUG_PRINT_F_CONVERSIONS_MEAN and (DEBUG_PRINT_GLOBAL_APP==1)) printf(fmt, __VA_ARGS__); } while (0)

#define INDEX_VOID (-1)

// Init_Arithmetic_Mean_Temp_OnetenthDegC
// Arithmetic_mean_value = (xn + xn-1 + xn-2 + xn-3 + ...x) / n_of_temps (or during filling, divide by how many there are)
// A direct form discrete-time FIR filter of order N. All gains are 1/n_of_temps
// https://en.wikipedia.org/wiki/Finite_impulse_response
//
void
Init_Arithmetic_Mean_Temp_OnetenthDegC (
    temp_onetenthDegC_mean_t &temps_onetenthDegC_mean_array,
    const unsigned            n_of_temps) {

    for (unsigned index_of_array = 0; index_of_array < n_of_temps; index_of_array++) {
        temps_onetenthDegC_mean_array.temps_onetenthDegC[index_of_array] = 0;
    }

    temps_onetenthDegC_mean_array.temps_index_next_to_write = 0;
    temps_onetenthDegC_mean_array.temps_num = 0;
}

// Do_Arithmetic_Mean_Temp_OnetenthDegC log from "2017 02 05 A"
// mean(W)=240 over (8-1) with input 240 changed=0 (dropped 240 240) |Observe dropped only one of them (equal)
// mean(W)=240 over (8-1) with input 240 changed=0 (dropped 240 240) |
// mean(W)=240 over (8-1) with input 240 changed=0 (dropped 240 240) |
// mean(W)=240 over (8-1) with input 240 changed=0 (dropped 240 240) |
// mean(W)=240 over (8-2) with input 239 changed=1 (dropped 240 239) <-- Filtered, early reading of low
// mean(W)=240 over (8-2) with input 240 changed=0 (dropped 240 239)
// mean(W)=240 over (8-2) with input 240 changed=0 (dropped 240 239)
//      Changing once
// mean(W)=239 over (8-2) with input 239 changed=0 (dropped 240 239)
// mean(W)=239 over (8-2) with input 240 changed=1 (dropped 240 239) <-- Filtered, bouncing at value switch
// mean(W)=239 over (8-2) with input 240 changed=1 (dropped 240 239)
// mean(W)=239 over (8-2) with input 239 changed=0 (dropped 240 239)
// mean(W)=239 over (8-2) with input 240 changed=1 (dropped 240 239) <-- Filtered, still not sure
// mean(W)=239 over (8-2) with input 240 changed=1 (dropped 240 239) <-- Filtered
// mean(W)=239 over (8-2) with input 240 changed=1 (dropped 240 239) <-- Filtered
// mean(W)=239 over (8-2) with input 239 changed=0 (dropped 240 239)
// mean(W)=239 over (8-2) with input 239 changed=0 (dropped 240 239)
// mean(W)=239 over (8-2) with input 239 changed=0 (dropped 240 239)
//         239 for some 40 readings, then the same again on 239->238
//
temp_onetenthDegC_t
Do_Arithmetic_Mean_Temp_OnetenthDegC (
    temp_onetenthDegC_mean_t   &temps_onetenthDegC_mean_array, // i/o
    const unsigned             n_of_temps,
    const temp_onetenthDegC_t  temps_onetenthDeg) { // Next value
    
    // ints needed to avoid overflow in calculations

    unsigned use_n_of_temps;        // Init ok
    unsigned remove_n_of_temps      = 0;
    bool     not_full               = (temps_onetenthDegC_mean_array.temps_num < n_of_temps);
    int      temp_return;           // Always set
    int      temps_sum              = 0;
    int      temp_largest           = INT_MIN;
    int      index_of_temp_largest  = INDEX_VOID;
    int      temp_smallest          = INT_MAX;
    int      index_of_temp_smallest = INDEX_VOID;

    // Store new data and set where to write the next
    temps_onetenthDegC_mean_array.temps_onetenthDegC          [temps_onetenthDegC_mean_array.temps_index_next_to_write] = temps_onetenthDeg;
    temps_onetenthDegC_mean_array.temps_index_next_to_write = (temps_onetenthDegC_mean_array.temps_index_next_to_write + 1) % n_of_temps;

    // Find how many values are in the set right now
    if (not_full) {
        temps_onetenthDegC_mean_array.temps_num++;
        use_n_of_temps = temps_onetenthDegC_mean_array.temps_num; // For log
    } else { // FULL, use = n_of_temps
        use_n_of_temps = n_of_temps;

        // Find largest and smallest
        for (unsigned index_of_array = 0; index_of_array < use_n_of_temps; index_of_array++) {

            temp_onetenthDegC_t value = temps_onetenthDegC_mean_array.temps_onetenthDegC[index_of_array];

            if (value > temp_largest) {
                index_of_temp_largest = index_of_array;
                temp_largest = value;
            } else {}

            if (value < temp_smallest) {
                index_of_temp_smallest = index_of_array;
                temp_smallest = value;
            } else {}
        }
    }

    if (not_full) {
        temp_return = temps_onetenthDeg; // TODO Find out why: I needed to do this for the FLASHED code!
    } else {
        // If both indices have a value and
        // index_of_temp_largest === index_of_temp_smallest then that's because
        //          temp_largest === temp_smallest          then that's because
        //                   ALL ARE EQUAL!

        // Calculate sum of data set not including largest and smallest
        for (unsigned index_of_array = 0; index_of_array < use_n_of_temps; index_of_array++) {

            if (index_of_array == index_of_temp_largest) {
                // Largest not part of mean
                remove_n_of_temps++;
            } else if (index_of_array == index_of_temp_smallest) {
                // Smallest not part of mean
                remove_n_of_temps++;
            } else { // Use
                temps_sum += temps_onetenthDegC_mean_array.temps_onetenthDegC[index_of_array];
            }
        }
        // Calculate arithmetic mean of data set
        temp_return = (temps_sum / (use_n_of_temps - remove_n_of_temps)); // arithmetic mean
    }

    // --- Debug only code, trying debug_print here too, instead of #fdef block
    // Removed 15Oct2018 in _Aquarium_1_x AQU=056 (also neededtempchip_mcp9808.h, not good for reuse)

    return (temp_onetenthDegC_t) temp_return;
}

// Convert to a signed value with one decimal, seen as an integer
//
{temp_onetenthDegC_t, bool}
Temp_OnetenthDegC_To_Str (
    const i2c_temp_onetenthDegC_t degC_dp1,
    char temp_degC_str[EXTERNAL_TEMPERATURE_DEGC_TEXT_LEN]) {

    int degC_Unary_Part   = degC_dp1/10;
    int degC_Decimal_Part = degC_dp1 - (degC_Unary_Part*10);

    temp_onetenthDegC_t return_degC_dp1 = (temp_onetenthDegC_t) degC_dp1; // int on both sides

    int sprintf_return;
    bool error = false;

    error = error bitor ((degC_Unary_Part < EXTERNAL_TEMPERATURE_MIN_ONETENTHDEGC) or (degC_Unary_Part > EXTERNAL_TEMPERATURE_MAX_ONETENTHDEGC));
    error = error bitor ((degC_Decimal_Part < 0) or (degC_Decimal_Part > 9)); // error not possible if correct math

    sprintf_return = sprintf (temp_degC_str, "%02u.%01u", degC_Unary_Part, degC_Decimal_Part);
    error = error bitor (sprintf_return != 4); // "25.0"
    error = error bitor (sprintf_return < 0);

    if (error) {
        char error_text [] = EXTERNAL_TEMPERATURE_ERROR_TEXT;
        memcpy (temp_degC_str, error_text, sizeof(error_text));
        return_degC_dp1 = EXTERNAL_TEMPERATURE_MAX_ONETENTHDEGC;
    } else {} // No code: ok

    return {return_degC_dp1, not error};
}

{temp_onetenthDegC_t, bool}
TC1047_Raw_DegC_To_String_Ok (
    const unsigned int adc_val_mean_i,
    char (&?temp_degC_str)[EXTERNAL_TEMPERATURE_DEGC_TEXT_LEN]) {

     // Internal A/D-converter
     // 0 to 65520 (0xFFF0) Actual ADC is 12 bit so bottom 4 bits always zero
     // Div 16 = 0 to 4095, ref = 3.3V
     // 3300 mV / 4096 = 0,8056640625 mV
     // 750 mV = 25 degC then 10 mV/degC = 8.05 mV/degC
     // 3300/750=4.4
     // 4.4*16 = 70.4
     // See XS1-A8A-64-FB96-Datasheet(1.3).pdf

     // Microchip TC1047A Precision Temperature-to-Voltage Converter
     // 750 mV       =  25 degC = ((65520/3300)*750) = 14890,90909090909091
     // 100 mV       = -40 degC = ((65520/3300)*100) =  1985,454545454545
     // 650 mV delta =  65 degC = ((65520/3300)*650) = 12905,4545454545425
     //                     12905,4545454545425/65   =   198,5454545454545

    // ints needed to avoid overflow in calculations
    
    int degC_dp1 = ((((adc_val_mean_i*100) - 198545) / 1985) - 400) - INNER_TEMPERATURE_OFFSET_DEGC_DP1; 
                   // 25.0 degC is 250 here since it includes one decimal point dp1
                   // Also observe that we have subtracted INNER_TEMPERATURE_OFFSET_DEGC_DP1 in the mean valued dataset

    int  degC_Unary_Part   = degC_dp1/10;
    int  degC_Decimal_Part = degC_dp1 - (degC_Unary_Part*10);

    bool error = false;

    error = error bitor ((degC_Unary_Part < INNER_TEMPERATURE_MIN_DEGC) or (degC_Unary_Part > INNER_TEMPERATURE_MAX_DEGC));
    error = error bitor ((degC_Decimal_Part < 0) or (degC_Decimal_Part > 9)); // error not possible if correct math

    if (!isnull(temp_degC_str)) {
        int sprintf_return;
        sprintf_return = sprintf (temp_degC_str, "%02u.%01u", degC_Unary_Part, degC_Decimal_Part);
        error = error bitor (sprintf_return != 4); // "25.0"
        error = error bitor (sprintf_return < 0);
        if (error) {
            char error_text [] = INNER_TEMPERATURE_ERROR_TEXT;
            memcpy (temp_degC_str, error_text, sizeof(error_text));
        } else {} // No code: ok
    } else {}

    if (error) {
        degC_dp1 = EXTERNAL_TEMPERATURE_MAX_ONETENTHDEGC;
    } else {} // No code: ok

    return {(temp_onetenthDegC_t) degC_dp1, not error};
}

{light_sensor_range_t, bool}
Ambient_Light_Sensor_ALS_PDIC243_To_String_Ok (
    const unsigned int adc_val_mean_i,
    char (&?lux_str)[INNER_LUX_TEXT_LEN]) {

    // Internal A/D-converter
    // 0 to 65520 (0xFFF0) Actual ADC is 12 bit so bottom 4 bits always zero
    // Div 16 = 0 to 4095, ref = 3.3V
    // 3300 mV / 4096 = 0.8056640625 mV
    //
    // Ambient Light Sensor ALS-PDIC243-3B from Everlight is powered from 3.3V and sources 100 to 300 uA
    // at 1000 Lux into 7.5K - gives max about 7.5K * 0.3mA = 2.25V
    // Let's take full number to 99: 65520 / 661 = 99 HOWEVER THIS TURNS OUT TO BE MAX 61 when full light
    // 65520 / 407 should give 99 as max, then

    light_sensor_range_t light_sensor_range = adc_val_mean_i/407;  // Dark to low light is zero, normal 5-10, max 99
    if (light_sensor_range > INNER_MAX_LUX) light_sensor_range = INNER_MAX_LUX; // Error will not be set

    int sprintf_return;
    bool error = false;

    error = error bitor ((light_sensor_range < INNER_MIN_LUX) or (light_sensor_range > INNER_MAX_LUX));

    if (!isnull(lux_str)) {
        sprintf_return = sprintf (lux_str, "%02u", light_sensor_range);
        error = error bitor (sprintf_return != 2); // "25.0"
        error = error bitor (sprintf_return < 0);

        if (error) {
            char error_text [] = INNER_LUX_ERROR_TEXT;
            memcpy (lux_str, error_text, sizeof(error_text));
            light_sensor_range = INNER_MAX_LUX;
        } else {} // No code: ok
    }

    if (error) {
        light_sensor_range = INNER_MAX_LUX;
    } else {} // No code: ok

    return {light_sensor_range, not error};
}

{voltage_onetenthV_t, bool}
RR_12V_24V_To_String_Ok (
    const unsigned int adc_val_mean_i,
    char (&?rr_12V_24V_str)[INNER_RR_12V_24V_TEXT_LEN]) {

    // Internal A/D-converter
    // 0 to 65520 (0xFFF0) Actual ADC is 12 bit so bottom 4 bits always zero
    // Div 16 = 0 to 4095, ref = 3.3V
    // 3300 mV / 4096 = 0.8056640625 mV
    // R-R network is 9.1k/1k which is 1k up on 10.1k which is 0.09901 (almost 0.1)
    // 990 mV is 990/0.805664062 = 1229 raw AD value

    // There are two jumpers on the aquarium Power driver board, situated to the right of thr connector
    // with 12V and 24V 9.1K resistor outputs. If we only have USB connected to power the box then the jumpers
    // may be positioned to the right, when 5V is connected to both 12V and 24V circuits via 1K on the
    // Large connection board. In that case we measure about 4.1V on both 12V and 24V

    // ints needed to avoid overflow in calculations
    
    int volt_dp1 = (adc_val_mean_i/16)*100/1229; // int needed to avoid overflow in calculations
                   // 24V is 1229*2.4 = 2949
                   // 2949*100/1229 = 239 dp1 = 23.9V

    int  volt_Unary_Part   = volt_dp1/10;
    int  volt_Decimal_Part = volt_dp1 - (volt_Unary_Part*10);

    int sprintf_return;
    bool error = false;

    error = error bitor ((volt_Unary_Part < INNER_RR_12V_24V_MIN_VOLTS) or (volt_Unary_Part > INNER_RR_12V_24V_MAX_VOLTS));
    error = error bitor ((volt_Decimal_Part < 0) or (volt_Decimal_Part > 9)); // error not possible if correct math

    if (error) {
        volt_dp1 = INNER_RR_12V_24V_MAX_VOLTS;
    } else {}

    if (!isnull(rr_12V_24V_str)) {
        sprintf_return = sprintf (rr_12V_24V_str, "%02u.%01u", volt_Unary_Part, volt_Decimal_Part);
        error = error bitor (sprintf_return != 4); // "25.0"
        error = error bitor (sprintf_return < 0);

        if (error) {
            char error_text [] = INNER_RR_12V_24V_ERROR_TEXT;
            memcpy (rr_12V_24V_str, error_text, sizeof(error_text));
        } else {} // No code: ok
    } else {}
    
    return {(voltage_onetenthV_t) volt_dp1, not error};
}

// BINARY-CODED DECIMAL - DATA WITH CHRONODOT IS BCD - ONE NIBBLE PER
// https://en.wikipedia.org/wiki/Binary-coded_decimal
uint8_t BCD_To_Bin_8 (uint8_t val)
{
    return val - (6 * (val >> 4)); //
}
uint8_t Bin_To_BCD_8 (uint8_t val)
{
    return val + (6 * (val / 10));
}
