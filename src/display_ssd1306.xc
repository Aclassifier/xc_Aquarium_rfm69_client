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
All text above, and the splash screen below must be included in any redistribution
*********************************************************************/
/*
 * display_ssd1306.xc
 *
 *  Created on: 9. feb. 2017
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
#include <iso646.h>
#include <string.h>   // memset. TODO Consider safestring. All over the project!
#include <timer.h>    // For delay_milliseconds (but it compiles without?)

#include "_globals.h"
#include "param.h"
#include "button_press.h"

#include "i2c.h"

#include "defines_adafruit.h"
#include "core_graphics_adafruit_GFX.h"

#include "I2C_Internal_Task.h"
#include "display_ssd1306.h"
#endif

out port outP_display_notReset =
    // on tile[0]:XS1_PORT_1G; // #RES at startKIT GPIO header (J7) port P1G0, processor pin X0D22, socket GPIO 3
    on tile[0]:XS1_PORT_1M;    // I_NRES RES at startKIT GPIO header (J7) port P1M0, processor pin X0D22, socket GPIO 3
    // on adafruit monochrome 128x32 I2C OLED graphic display PRODUCT ID: 931, containing
    // module UG-2832HSWEG02 with chip SSD1306 from Univision Technology Inc. Data sheet often says 128 x 64 bits
    // as it looks like much of the logic is the same as for 128 z 32 bits.
    // At least 3 us low to reset

// The memory buffer of the display
static uint8_t buffer[SSD1306_LCDHEIGHT*SSD1306_LCDWIDTH/8];

bool writeDisplay_i2c_command (client i2c_internal_commands_if i_i2c_internal_commands, uint8_t c) {
    bool error = false;

    unsigned char data[] = {c};
    int           nbytes = 1;

    error = error bitor not i_i2c_internal_commands.write_display_ok (I2C_ADDRESS_OF_DISPLAY, DISPLAY_REG_ADDR_COMMAND, data, nbytes);

    return not error;
}

bool writeDisplay_i2c_data (client i2c_internal_commands_if i_i2c_internal_commands, uint8_t c) {
    bool error = false;

    unsigned char data[] = {c};
    int           nbytes = 1;

    error = error bitor not i_i2c_internal_commands.write_display_ok (I2C_ADDRESS_OF_DISPLAY, DISPLAY_REG_ADDR_DATA, data, nbytes);

    return not error;
}

bool Adafruit_SSD1306_i2c_begin (client i2c_internal_commands_if i_i2c_internal_commands) {

    bool error = false;

    // By default, we'll generate the high voltage for the display in the display chip's charge pump, from the 3.3V line
    const display_vccstate_t vccstate = SSD1306_SWITCHCAPVCC; // Initially parameterised, no ned to. Just hard-code it here

    outP_display_notReset <: 1; // High. Didn't help remove qwe

    // The pin is initially not driven. Adafruit display board ID-931 contains a 10K pullup and a diode
    // So the pin is high now
    outP_display_notReset <: 0; // Low
    delay_milliseconds(10);
    outP_display_notReset <: 1; // High

    #if defined SSD1306_128_32
        // Init sequence for 128x32 OLED module
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, SSD1306_DISPLAYOFF);          // 0xAE
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, SSD1306_SETDISPLAYCLOCKDIV);  // 0xD5
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, 0x80);                        // the suggested ratio 0x80
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, SSD1306_SETMULTIPLEX);        // 0xA8
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, SSD1306_LCDHEIGHT-1);         // 0x1F
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, SSD1306_SETDISPLAYOFFSET);    // 0xD3
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, 0x0);                         // no offset
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, SSD1306_SETSTARTLINE | 0x0);  // line #0
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, SSD1306_CHARGEPUMP);          // 0x8D
        if (vccstate == SSD1306_SWITCHCAPVCC)
            { error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, 0x14); }
        else // SSD1306_EXTERNALVCC
            { error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, 0x10); }
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, SSD1306_MEMORYMODE);          // 0x20
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, 0x00);                        // 0x0 act like ks0108
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, SSD1306_SEGREMAP | 0x1);
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, SSD1306_COMSCANDEC);
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, SSD1306_SETCOMPINS);          // 0xDA
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, 0x02);
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, SSD1306_SETCONTRAST);         // 0x81
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, CONTRAST_VALUE_BRIGHT_IS_DEFAULT);
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, SSD1306_SETPRECHARGE);        // 0xd9
        if (vccstate == SSD1306_SWITCHCAPVCC)
            { error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, 0xF1); }
        else // SSD1306_EXTERNALVCC here
            { error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, 0x22); }
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, SSD1306_SETVCOMDETECT);       // 0xDB
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, 0x40);
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, SSD1306_DISPLAYALLON_RESUME); // 0xA4
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, SSD1306_NORMALDISPLAY);       // 0xA6
    #endif

    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, SSD1306_DISPLAYON);               // 0xAF turn on oled panel

    return not error;
}

// the most basic function, set a single pixel
void setPixel_in_buffer (int16_t x, int16_t y, uint16_t color) {
    if ((x < 0) || (x >= width()) || (y < 0) || (y >= height())) {
        return;
    } else {
        // check rotation, move pixel around if necessary
        switch (getRotation()) {
            case 1:
                t_swap(int16_t,x, y);
                x = display_param.WIDTH - x - 1;
                break;
            case 2:
                x = display_param.WIDTH - x - 1;
                y = display_param.HEIGHT - y - 1;
                break;
            case 3:
                t_swap(int16_t,x, y);
                y = display_param.HEIGHT - y - 1;
                break;
        }

        // x is which column
        switch (color)
        {
            case WHITE:   buffer[x + (y/8)*width()] |=  (1 << (y&7)); break;
            case BLACK:   buffer[x + (y/8)*width()] &= ~(1 << (y&7)); break;
            case INVERSE: buffer[x + (y/8)*width()] ^=  (1 << (y&7)); break;
        }
    }
}

bool tellDisplay_i2c_invert (client i2c_internal_commands_if i_i2c_internal_commands, uint8_t i) {
    bool error = false;

    if (i) {
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, SSD1306_INVERTDISPLAY);
    } else {
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, SSD1306_NORMALDISPLAY);
    }

    return not error;
}

// tellDisplay_i2c_startscrollright
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
bool tellDisplay_i2c_startscrollright (client i2c_internal_commands_if i_i2c_internal_commands, uint8_t start, uint8_t stop){
    bool error = false;

    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, SSD1306_RIGHT_HORIZONTAL_SCROLL);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, 0X00);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, start);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, 0X00);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, stop);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, 0X00);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, 0XFF);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, SSD1306_ACTIVATE_SCROLL);

    return not error;
}

// tellDisplay_i2c_startscrollleft
// Activate a right handed scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
bool tellDisplay_i2c_startscrollleft (client i2c_internal_commands_if i_i2c_internal_commands, uint8_t start, uint8_t stop){
    bool error = false;

    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,SSD1306_LEFT_HORIZONTAL_SCROLL);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,0X00);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,start);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,0X00);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,stop);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,0X00);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,0XFF);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,SSD1306_ACTIVATE_SCROLL);

    return not error;
}

// tellDisplay_i2c_startscrolldiagright
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
bool tellDisplay_i2c_startscrolldiagright (client i2c_internal_commands_if i_i2c_internal_commands, uint8_t start, uint8_t stop){
    bool error = false;

    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,SSD1306_SET_VERTICAL_SCROLL_AREA);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,0X00);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,SSD1306_LCDHEIGHT);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,0X00);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,start);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,0X00);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,stop);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,0X01);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,SSD1306_ACTIVATE_SCROLL);

    return not error;
}

// tellDisplay_i2c_startscrolldiagleft
// Activate a diagonal scroll for rows start through stop
// Hint, the display is 16 rows tall. To scroll the whole display, run:
// display.scrollright(0x00, 0x0F)
bool tellDisplay_i2c_startscrolldiagleft (client i2c_internal_commands_if i_i2c_internal_commands, uint8_t start, uint8_t stop){
    bool error = false;

    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,SSD1306_SET_VERTICAL_SCROLL_AREA);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,0X00);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,SSD1306_LCDHEIGHT);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,0X00);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,start);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,0X00);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,stop);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,0X01);
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,SSD1306_ACTIVATE_SCROLL);

    return not error;
}

bool tellDisplay_i2c_stopscroll (client i2c_internal_commands_if i_i2c_internal_commands){
    bool error = false;

    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands,SSD1306_DEACTIVATE_SCROLL);

    return not error;
}

bool writeToDisplay_i2c_all_buffer (client i2c_internal_commands_if i_i2c_internal_commands) {
    bool error = false;

    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, SSD1306_COLUMNADDR); // 0x21
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, 0);                  // 0x00 Column start address (0 = reset)
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, SSD1306_LCDWIDTH-1); // 0x7F Column end address (127 = reset)

    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, SSD1306_PAGEADDR);   // 0x22
    error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, 0);                  // 0x00 Page start address (0 = reset)
    #if SSD1306_LCDHEIGHT == 64
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, 7); // Page end address
    #endif
    #if SSD1306_LCDHEIGHT == 32
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, 3); // Page end address
    #endif
    #if SSD1306_LCDHEIGHT == 16
        error = error bitor not writeDisplay_i2c_command(i_i2c_internal_commands, 1); // Page end address
    #endif

    // I2C
    for (uint16_t i=0; i<(SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT/8); i++) {
        // send a bunch of data in one transmission
        // I2C
        #if (((SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT/8) % SSD1306_WRITE_CHUNK_SIZE) != 0)
            #error Display buffer not multiple of 16
        #endif

        int nbytes = SSD1306_WRITE_CHUNK_SIZE;
        unsigned char data[SSD1306_WRITE_CHUNK_SIZE];

        for (uint16_t x=0; x<NUM_ELEMENTS(data); x++) { // Thanks, Maxim! (was uint8_t)
            data[x] = buffer[i];
            i++;
        }
        i--; // Went one too far above

        error = error bitor not i_i2c_internal_commands.write_display_ok (I2C_ADDRESS_OF_DISPLAY, DISPLAY_REG_ADDR_DATA, data, nbytes); // Was i2c_master_write_reg (device, reg_addr, data, nbytes, i2c_internal_config);
    }
    return not error;
}

// clear everything
void Clear_All_Pixels_In_Buffer (void) {
    memset (buffer, 0, (SSD1306_LCDWIDTH*SSD1306_LCDHEIGHT/8));
}

void fillSplashScreen_in_buffer (void) {
    static uint8_t splash_buffer[SSD1306_BUFFER_SIZE] = {DISPLAY_BUFFER_ADAFRUIT_DEFAULT_0_192,DISPLAY_BUFFER_ADAFRUIT_DEFAULT_193_511};
    memcpy (buffer, splash_buffer, SSD1306_BUFFER_SIZE);
}

void drawHorisontalLine_in_buffer (int16_t x, int16_t y, int16_t w, uint16_t color) { // Virtual, instead of drawHorisontalLine
    bool bSwap = false;
    switch (display_param.rotation) {
        case 0:
            // 0 degree rotation, do nothing
            break;
        case 1:
            // 90 degree rotation, swap x & y for rotation, then invert x
            bSwap = true;
            t_swap(int16_t,x, y);
            x = display_param.WIDTH - x - 1;
            break;
        case 2:
            // 180 degree rotation, invert x and y - then shift y around for height.
            x = display_param.WIDTH - x - 1;
            y = display_param.HEIGHT - y - 1;
            x -= (w-1);
            break;
        case 3:
            // 270 degree rotation, swap x & y for rotation,
            // then invert y and adjust y for w (not to become h)
            bSwap = true;
            t_swap(int16_t,x, y);
            y = display_param.HEIGHT - y - 1;
            y -= (w-1);
            break;
    }

    if (bSwap) {
        drawVerticalLineInternal_in_buffer(x, y, w, color);
    } else {
        drawHorisontalLineInternal_in_buffer(x, y, w, color);
  }
}

void drawHorisontalLineInternal_in_buffer (int16_t x, int16_t y, int16_t w, uint16_t color) {
    // Do bounds/limit checks
    if (y < 0 || y >= display_param.HEIGHT) { return; }

    // make sure we don't try to draw below 0
    if (x < 0) {
        w += x; // increment param w before real usage
        x = 0;
    }

    // make sure we don't go off the edge of the display
    if ((x + w) > display_param.WIDTH) {
        w = (display_param.WIDTH - x);
    }

    // if our width is now negative, punt
    if (w <= 0) { return; }

    // set up the pointer for  movement through the buffer
    register uint8_t *pBuf = buffer;
    // adjust the buffer pointer for the current row
    pBuf += ((y/8) * SSD1306_LCDWIDTH); // qwe use variable instead
    // and offset x columns in
    pBuf += x;

    register uint8_t mask = 1 << (y&7);

    switch (color)
    {
        case WHITE:               while (w--) { *pBuf++ |= mask; }; break;
        case BLACK: mask = ~mask; while (w--) { *pBuf++ &= mask; }; break;
        case INVERSE:             while (w--) { *pBuf++ ^= mask; }; break;
    }
}

void drawVerticalLine_in_buffer (int16_t x, int16_t y, int16_t h, uint16_t color) { // Virtual, instead of drawVerticalLine
    bool bSwap = false;
    switch (display_param.rotation) {
        case 0:
            break;
        case 1:
            // 90 degree rotation, swap x & y for rotation, then invert x and adjust x for h (now to become w)
            bSwap = true;
            t_swap(int16_t,x, y);
            x = display_param.WIDTH - x - 1;
            x -= (h-1);
            break;
        case 2:
            // 180 degree rotation, invert x and y - then shift y around for height.
            x = display_param.WIDTH - x - 1;
            y = display_param.HEIGHT - y - 1;
            y -= (h-1);
            break;
        case 3:
            // 270 degree rotation, swap x & y for rotation, then invert y
            bSwap = true;
            t_swap(int16_t,x, y);
            y = display_param.HEIGHT - y - 1;
            break;
  }

  if (bSwap) {
      drawHorisontalLineInternal_in_buffer(x, y, h, color);
  } else {
      drawVerticalLineInternal_in_buffer(x, y, h, color);
  }
}

void drawVerticalLineInternal_in_buffer (int16_t x, int16_t __y, int16_t __h, uint16_t color) {

    // do nothing if we're off the left or right side of the screen
    if (x < 0 || x >= display_param.WIDTH) { return; }

    // make sure we don't try to draw below 0
    if (__y < 0) {
        // __y is negative, this will subtract enough from __h to account for __y being 0
        __h += __y;
        __y = 0;
    }

    // make sure we don't go past the height of the display
    if ( (__y + __h) > display_param.HEIGHT) {
        __h = (display_param.HEIGHT - __y);
    }

    // if our height is now negative, punt
    if (__h <= 0) {
      return;
    }

    // this display doesn't need ints for coordinates, use local byte registers for faster juggling
    register uint8_t y = __y;
    register uint8_t h = __h;

    // set up the pointer for fast movement through the buffer
    register uint8_t *pBuf = buffer;
    // adjust the buffer pointer for the current row
    pBuf += ((y/8) * SSD1306_LCDWIDTH);
    // and offset x columns in
    pBuf += x;

    // do the first partial byte, if necessary - this requires some masking
    register uint8_t mod = (y&7);

    if (mod) {
        // mask off the high n bits we want to set
        mod = 8-mod;

        // note - lookup table results in a nearly 10% performance improvement in fill* functions
        // register uint8_t mask = ~(0xFF >> (mod));
        static uint8_t premask[8] = {0x00, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE};
        register uint8_t mask = premask[mod];

        // adjust the mask if we're not going to reach the end of this byte
        if ( h < mod) {
            mask &= (0XFF >> (mod-h));
        }

        switch (color)
        {
            case WHITE:   *pBuf |=  mask;  break;
            case BLACK:   *pBuf &= ~mask;  break;
            case INVERSE: *pBuf ^=  mask;  break;
        }

        // fast exit if we're done here!
        if (h<mod) { return; }

        h -= mod;

        pBuf += SSD1306_LCDWIDTH; // qwe use var
    }

    // write solid bytes while we can - effectively doing 8 rows at a time
    if (h >= 8) {
        if (color == INVERSE)  {
             // separate copy of the code so we don't impact performance of the black/white write
             // version with an extra comparison per loop
             do {
                 *pBuf=~(*pBuf);

                 // adjust the buffer forward 8 rows worth of data
                 pBuf += SSD1306_LCDWIDTH; // qwe use val

                 // adjust h & y (there's got to be a faster way for me to do this, but this should
                 // still help a fair bit for now)
                 h -= 8;
              } while (h >= 8);
        }  else {
            // store a local value to work with
            register uint8_t val = (color == WHITE) ? 255 : 0;

            do {
                // write our value in
                *pBuf = val;

                // adjust the buffer forward 8 rows worth of data
                pBuf += SSD1306_LCDWIDTH; // qwe use val

                // adjust h & y (there's got to be a faster way for me to do this, but this should
                // still help a fair bit for now)
                h -= 8;
            } while (h >= 8);
        }
    }

    // now do the final partial byte, if necessary
    if (h) {
        mod = h & 7;
        // this time we want to mask the low bits of the byte, vs the high bits we did above
        // register uint8_t mask = (1 << mod) - 1;
        // note - lookup table results in a nearly 10% performance improvement in fill* functions
        static uint8_t postmask[8] = {0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F };
        register uint8_t mask = postmask[mod];
        switch (color)
        {
            case WHITE:   *pBuf |=  mask;  break;
            case BLACK:   *pBuf &= ~mask;  break;
            case INVERSE: *pBuf ^=  mask;  break;
        }
    }
}
