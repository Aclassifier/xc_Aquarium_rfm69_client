/*
 * param.h
 *
 *  Created on: 3. mars 2015
 *      Author: teig
 *      I found this in a file called param.h file in gnu and FreeBSD.
 *      Just included what I needed
 */

#ifndef PARAM_H_
#define PARAM_H_

typedef enum {I2C_ERR, I2C_OK, I2C_PARAM_ERR} i2c_result_t; // Two first from usage in XMOS module_i2c_master
// Typically, in i2c_master_do_rx does return 0; for error from floatWires(i2c) else return 1; for ok

#define UINT32_HIGH_BITS 0xffffffff // As INT_MIN or -2147483648 in <limits.h>

typedef uint8_t i2c_dev_address_t;
typedef uint8_t i2c_reg_address_t;
typedef uint8_t i2c_reg_data_t;
typedef int16_t i2c_temp_onetenthDegC_t; // 25.1 is 251 (as is temp_onetenthDegC_t). TODO: make unsigned?

typedef struct tag_i2c_dev_address_reg_address_t {
    i2c_dev_address_t _dev_address;
    i2c_reg_address_t _reg_address;
} i2c_server_params_t;

typedef struct tag_i2c_master_param_t {
    i2c_dev_address_t _use_dev_address; // i2c_dev_address
    i2c_result_t      _result;
} i2c_master_params_t;

#define NUM_STARTKIT_ADC_INPUTS        4 // (I soldered it incorrectly but didn't want to resolder)
#define   IOF_ADC_STARTKIT_24V         0 // Blue cable
#define   IOF_ADC_STARTKIT_LUX         1
#define   IOF_ADC_STARTKIT_TEMPERATURE 2
#define   IOF_ADC_STARTKIT_12V         3 // Green cable

#define ADC_PERIOD_TIME_USEC_ZERO_IS_ONY_QUERY_BASED     0 // 0 is so; 1000 is 1 ms
#define NUM_STARTKIT_ADC_NEEDED_DATA_SETS             1000 // qwe Each of NUM_STARTKIT_ADC_INPUTS. MAX VALUE IS 32-16=16 bits or 0xffff = 65535

#define OFFSET_ADC_INPUTS_STARTKIT 190,175,195,187 // Measured with ADC nothing connected on startKIT ADC J2. Measured on set of NUM_STARTKIT_ADC_NEEDED_DATA_SETS

typedef struct tag_startkit_adc_vals {
    unsigned short x[NUM_STARTKIT_ADC_INPUTS];
} t_startkit_adc_vals;

// JPG ADC_PERIOD_TIME_USEC Scope   ADC_TRIG_DELAY(*)  Scope    ADC
//     ----------- us -----------   -- us 1 of 4 pulses ---    ----
// A,B 1000                  1000    100               2 us    "Ok" values read
// C,D 1000                  1000    200               4 us    None
// E,F 2000                  1000    200               4 us    None
// G,H  500                   500    200               4 us    None
// I,J  500                   500    400               8 us    None
// (*) of startkit-adc.h of lib-startkit-support
// (*) Pulse is one period of four

// System time tag of printf in xTIMEcomposer debug console
// http://www.xcore.com/viewtopic.php?p=28195#p28195
// 32-bit 100 MHz system counter. Examples:
//     1194933268
//     11.9 seconds, next is 12.1
//    -2111493615
//     21.1 seconds, next is 20.1 (since ot goes full circle through negative values)
// Needs #include <print.h>
#define timed_printf \
    { timer t; int u; t :> u; /* GETTIME instruction */ \
        printint(u); printstr(" "); \
    } printf

#define DO_ADC_NESTED_SELECT 1 // 1 compiles with [[distributable]] My_startKIT_ADC_Task, but does not run. Would be best with chanends!
                               // 0 needed

#else
    #error Nested include PARAM_H_
#endif
