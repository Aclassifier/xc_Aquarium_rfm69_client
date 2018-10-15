/*********************************************************************
This is a library for our Monochrome OLEDs based on SSD1306 drivers

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/category/63_98

These displays use SPI to communicate, 4 or 5 pins are required to
interface

Adafruit invests time and resources providing this open source code,
please support Adafruit and open-source hardware by purchasing
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.
BSD license, check license.txt for more information
All text above, and the splash screen must be included in any redistribution
*********************************************************************/

/*
 * defines_adafruit.h
 *
 *  Created on: 28. feb. 2015
 *      Author: Teig
 */

#ifndef DEFINES_ADAFRUIT_H_
#define DEFINES_ADAFRUIT_H_

// #define Wire_begin()
// #define Wire_beginTransmission(i2c_dev_address)
// #define WIRE_WRITE(control);
// #define Wire_endTransmission();
// #define delay(ms)

#define OUTPUT 1
#define LOW    1
#define HIGH   1

#define HEX 0
#define DEC 1

typedef uint8_t i2c_PortReg_t;
typedef uint8_t i2c_PortMask_t;

#else
    #error Nested include DEFINES_ADAFRUIT_H_
#endif

