/*
This is the core graphics library for all our displays, providing a common
set of graphics primitives (points, lines, circles, etc.).  It needs to be
paired with a hardware-specific library for each display device we carry
(to handle the lower-level functions).

Adafruit invests time and resources providing this open source code, please
support Adafruit & open-source hardware by purchasing products from Adafruit!

Copyright (c) 2013 Adafruit Industries.  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * core_graphics_adafruit_GFX.xc
 *
 *  Created on: 28. feb. 2015
 *      Author: Teig
 */

#define INCLUDES
#ifdef INCLUDES
#include <platform.h>
#include <xs1.h>
#include <stdlib.h>
#include <stdint.h>
#include <xccompat.h> // REFERENCE_PARAM

#include "_version.h" // First this..
#include "_globals.h" // ..then this
#include "param.h"
#include "defines_adafruit.h"
//
#include "core_graphics_font5x8.h"
#include "core_graphics_adafruit_GFX.h"
#endif

display_param_t display_param;

unsigned char font[] = {ASCII_FONT5X8};

void Adafruit_GFX_constructor(int16_t w, int16_t h) {
    display_param.WIDTH = w;
    display_param.HEIGHT = h;
    display_param._width = w;
    display_param._height = h;
    display_param.rotation = 0;
    display_param.cursor_y = display_param.cursor_x = 0;
    display_param.textsize = 1;
    display_param.textcolor = display_param.textbgcolor = 0xFFFF;
    display_param.wrap = true;
}

// Needed for xmos since I don't know (yet?) how to pick this up
// writeToDisplay_i2c_all_buffer.print
void display_print(const char txt[], size_t len) {
    for (size_t i = 0; i < len; i++) {
        write(txt[i]);
    }
}

// Needed for xmos since I don't know (yet?) how to pick this up
// writeToDisplay_i2c_all_buffer.println
void display_println(const char txt[], size_t len) {
    for (size_t i = 0; i < len; i++) {
        write(txt[i]);
    }
    write('\n'); // Special case for '\n' in write, really does CR,LF
}

void write_raw (uint8_t c) { // Can thus draw directly from ASCII_FONT5X8

    drawChar(display_param.cursor_x, display_param.cursor_y, c,
             display_param.textcolor, display_param.textbgcolor,
             display_param.textsize);
    display_param.cursor_x += display_param.textsize * 6;
    if (display_param.wrap && (display_param.cursor_x > (display_param._width - display_param.textsize * 6))) {
        display_param.cursor_y += display_param.textsize * 8;
        display_param.cursor_x = 0;
    }
}

void write (uint8_t c) {

    if (c == '\n') { // Do CR, LF
        display_param.cursor_x = 0; // CR = carriage Return: start at leftmost position
        display_param.cursor_y += display_param.textsize * 8; // LF = Line Feed: go down to next line
    } else if (c == '\r') {
        // skip em
    } else {
        write_raw (c);
    }
}

void display_print_dec_8 (const uint8_t value) {
    uint8_t rest = value;              //                      231
    uint8_t hundreds = rest / 100;     // 231 / 100 =          2
    rest = rest - (hundreds * 100);    // 231 - (2 * 100) = 31
    uint8_t tens = rest / 10;          //  31 / 10 =            3
    uint8_t ones = rest - (tens * 10); //  31 - (3 * 10) =       1

    char c_hundreds[1] = { hundreds + '0' };
    char c_tens[1] = { tens + '0' };
    char c_ones[1] = { ones + '0' };

    if (hundreds == 0) {
        if (tens == 0) {               // 001
            display_print(c_ones, 1);  // ..1
        } else {                       // 021
            display_print(c_tens, 1);  // .2.
            display_print(c_ones, 1);  // ..1
        }
    } else {                           // 321
        display_print(c_hundreds, 1);  // 3..
        display_print(c_tens, 1);      // .2.
        display_print(c_ones, 1);      // ..1
    }
}

// Draw a circle outline
void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    setPixel_in_buffer(x0, y0 + r, color);
    setPixel_in_buffer(x0, y0 - r, color);
    setPixel_in_buffer(x0 + r, y0, color);
    setPixel_in_buffer(x0 - r, y0, color);

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        setPixel_in_buffer(x0 + x, y0 + y, color);
        setPixel_in_buffer(x0 - x, y0 + y, color);
        setPixel_in_buffer(x0 + x, y0 - y, color);
        setPixel_in_buffer(x0 - x, y0 - y, color);
        setPixel_in_buffer(x0 + y, y0 + x, color);
        setPixel_in_buffer(x0 - y, y0 + x, color);
        setPixel_in_buffer(x0 + y, y0 - x, color);
        setPixel_in_buffer(x0 - y, y0 - x, color);
    }
}

void drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername,
        uint16_t color) {
    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;
        if (cornername & 0x4) {
            setPixel_in_buffer(x0 + x, y0 + y, color);
            setPixel_in_buffer(x0 + y, y0 + x, color);
        }
        if (cornername & 0x2) {
            setPixel_in_buffer(x0 + x, y0 - y, color);
            setPixel_in_buffer(x0 + y, y0 - x, color);
        }
        if (cornername & 0x8) {
            setPixel_in_buffer(x0 - y, y0 + x, color);
            setPixel_in_buffer(x0 - x, y0 + y, color);
        }
        if (cornername & 0x1) {
            setPixel_in_buffer(x0 - y, y0 - x, color);
            setPixel_in_buffer(x0 - x, y0 - y, color);
        }
    }
}

void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color) {
    drawVerticalLine(x0, y0 - r, 2 * r + 1, color);
    fillCircleHelper(x0, y0, r, 3, 0, color);
}

void fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername,
        int16_t delta, uint16_t color) {

    int16_t f = 1 - r;
    int16_t ddF_x = 1;
    int16_t ddF_y = -2 * r;
    int16_t x = 0;
    int16_t y = r;

    while (x < y) {
        if (f >= 0) {
            y--;
            ddF_y += 2;
            f += ddF_y;
        }
        x++;
        ddF_x += 2;
        f += ddF_x;

        if (cornername & 0x1) {
            drawVerticalLine(x0 + x, y0 - y, 2 * y + 1 + delta, color);
            drawVerticalLine(x0 + y, y0 - x, 2 * x + 1 + delta, color);
        }
        if (cornername & 0x2) {
            drawVerticalLine(x0 - x, y0 - y, 2 * y + 1 + delta, color);
            drawVerticalLine(x0 - y, y0 - x, 2 * x + 1 + delta, color);
        }
    }
}

// Bresenham's algorithm - thx wikpedia
void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color) {
    int16_t steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep) {
        t_swap(int16_t,x0, y0);
        t_swap(int16_t,x1, y1);
    }

    if (x0 > x1) {
        t_swap(int16_t,x0, x1);
        t_swap(int16_t,y0, y1);
    }

    int16_t dx, dy;
    dx = x1 - x0;
    dy = abs(y1 - y0);

    int16_t err = dx / 2;
    int16_t ystep;

    if (y0 < y1) {
        ystep = 1;
    } else {
        ystep = -1;
    }

    for (; x0 <= x1; x0++) {
        if (steep) {
            setPixel_in_buffer(y0, x0, color);
        } else {
            setPixel_in_buffer(x0, y0, color);
        }
        err -= dy;
        if (err < 0) {
            y0 += ystep;
            err += dx;
        }
    }
}

// Draw a rectangle
void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    drawHorisontalLine(x, y, w, color);
    drawHorisontalLine(x, y + h - 1, w, color);
    drawVerticalLine(x, y, h, color);
    drawVerticalLine(x + w - 1, y, h, color);
}

void drawVerticalLine(int16_t x, int16_t y, int16_t h, uint16_t color) {
    // Update in subclasses if desired! (drawVerticalLine_in_buffer)
    drawLine(x, y, x, y + h - 1, color);
}

void drawHorisontalLine(int16_t x, int16_t y, int16_t w, uint16_t color) {
    // Update in subclasses if desired! (drawHorisontalLine_in_buffer)
    drawLine(x, y, x + w - 1, y, color);
}

void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color) {
    // Update in subclasses if desired!
    for (int16_t i = x; i < x + w; i++) {
        drawVerticalLine(i, y, h, color);
    }
}

void fillScreen(uint16_t color) {
    fillRect(0, 0, display_param._width, display_param._height, color);
}

// Draw a rounded rectangle
void drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r,
        uint16_t color) {
    // smarter version
    drawHorisontalLine(x + r, y, w - 2 * r, color); // Top
    drawHorisontalLine(x + r, y + h - 1, w - 2 * r, color); // Bottom
    drawVerticalLine(x, y + r, h - 2 * r, color); // Left
    drawVerticalLine(x + w - 1, y + r, h - 2 * r, color); // Right
    // draw four corners
    drawCircleHelper(x + r, y + r, r, 1, color);
    drawCircleHelper(x + w - r - 1, y + r, r, 2, color);
    drawCircleHelper(x + w - r - 1, y + h - r - 1, r, 4, color);
    drawCircleHelper(x + r, y + h - r - 1, r, 8, color);
}

// Fill a rounded rectangle
void fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r,
        uint16_t color) {
    // smarter version
    fillRect(x + r, y, w - 2 * r, h, color);

    // draw four corners
    fillCircleHelper(x + w - r - 1, y + r, r, 1, h - 2 * r - 1, color);
    fillCircleHelper(x + r, y + r, r, 2, h - 2 * r - 1, color);
}

// Draw a triangle
void drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2,
        int16_t y2, uint16_t color) {
    drawLine(x0, y0, x1, y1, color);
    drawLine(x1, y1, x2, y2, color);
    drawLine(x2, y2, x0, y0, color);
}

// Fill a triangle
void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2,
        int16_t y2, uint16_t color) {

    int16_t a, b, y, last;

    // Sort coordinates by Y order (y2 >= y1 >= y0)
    if (y0 > y1) {
        t_swap(int16_t,y0, y1);
        t_swap(int16_t,x0, x1);
    }
    if (y1 > y2) {
        t_swap(int16_t,y2, y1);
        t_swap(int16_t,x2, x1);
    }
    if (y0 > y1) {
        t_swap(int16_t,y0, y1);
        t_swap(int16_t,x0, x1);
    }

    if (y0 == y2) { // Handle awkward all-on-same-line case as its own thing
        a = b = x0;
        if (x1 < a)
            a = x1;
        else if (x1 > b)
            b = x1;

        if (x2 < a)
            a = x2;
        else if (x2 > b)
            b = x2;

        drawHorisontalLine(a, y0, b - a + 1, color);
        return;
    }

    int16_t dx01 = x1 - x0, dy01 = y1 - y0, dx02 = x2 - x0, dy02 = y2 - y0,
            dx12 = x2 - x1, dy12 = y2 - y1;
    int32_t sa = 0, sb = 0;

    // For upper part of triangle, find scanline crossings for segments
    // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
    // is included here (and second loop will be skipped, avoiding a /0
    // error there), otherwise scanline y1 is skipped here and handled
    // in the second loop...which also avoids a /0 error here if y0=y1
    // (flat-topped triangle).
    if (y1 == y2)
        last = y1;   // Include y1 scanline
    else
        last = y1 - 1; // Skip it

    for (y = y0; y <= last; y++) {
        a = x0 + sa / dy01;
        b = x0 + sb / dy02;
        sa += dx01;
        sb += dx02;
        /* longhand:
         a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
         b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
         */
        if (a > b)
            t_swap(int16_t,a, b);
        drawHorisontalLine(a, y, b - a + 1, color);
    }

    // For lower part of triangle, find scanline crossings for segments
    // 0-2 and 1-2.  This loop is skipped if y1=y2.
    sa = dx12 * (y - y1);
    sb = dx02 * (y - y0);
    for (; y <= y2; y++) {
        a = x1 + sa / dy12;
        b = x0 + sb / dy02;
        sa += dx12;
        sb += dx02;
        /* longhand:
         a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
         b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
         */
        if (a > b)
            t_swap(int16_t,a, b);
        drawHorisontalLine(a, y, b - a + 1, color);
    }
}

void drawBitmap(int16_t x, int16_t y, const uint8_t bitmap[], int16_t w,
        int16_t h, uint16_t color) {

    int16_t i, j, byteWidth = (w + 7) / 8;

    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++) {
            uint8_t visiblePixel = bitmap[(j * byteWidth) + (i / 8)]; // qwe correct?
            if (visiblePixel & (128 >> (i & 7))) {
                setPixel_in_buffer(x + i, y + j, color);
            }
        }
    }
}

// Draw a 1-bit color bitmap at the specified x, y position from the
// provided bitmap buffer (must be PROGMEM memory) using color as the
// foreground color and bg as the background color.

void drawBitmap_bg(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w,
        int16_t h, uint16_t color, uint16_t bg) {

    int16_t i, j, byteWidth = (w + 7) / 8;

    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++) {
            uint8_t visiblePixel = bitmap[(j * byteWidth) + (i / 8)];
            if (visiblePixel & (128 >> (i & 7))) {
                setPixel_in_buffer(x + i, y + j, color);
            } else {
                setPixel_in_buffer(x + i, y + j, bg);
            }
        }
    }
}

//Draw XBitMap Files (*.xbm), exported from GIMP,
//Usage: Export from GIMP to *.xbm, rename *.xbm to *.c and open in editor.
//C Array can be directly used with this function
void drawXBitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w,
        int16_t h, uint16_t color) {

    int16_t i, j, byteWidth = (w + 7) / 8;

    for (j = 0; j < h; j++) {
        for (i = 0; i < w; i++) {
            uint8_t visiblePixel = bitmap[(j * byteWidth) + (i / 8)];
            if (visiblePixel & (1 << (i % 8))) {
                setPixel_in_buffer(x + i, y + j, color);
            }
        }
    }
}

// Draw a character
void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color,
        uint16_t bg, uint8_t size) {

    if ((x >= display_param._width) || // Clip right
            (y >= display_param._height) || // Clip bottom
            ((x + 6 * size - 1) < 0) || // Clip left
            ((y + 8 * size - 1) < 0))        // Clip top
        return;

    for (int8_t i = 0; i < 6; i++) {
        uint8_t line;

        if (i == 5)
            line = 0x0;
        else
            line = font[(c * 5) + i];

        for (int8_t j = 0; j < 8; j++) {
            if (line & 0x1) {
                if (size == 1)
                    setPixel_in_buffer(x + i, y + j, color); // default size. One bit is one pixel
                else
                    fillRect(x + (i * size), y + (j * size), size, size, color); // big size. One bit is four pixels
            } else if (bg != color) {
                if (size == 1)
                    setPixel_in_buffer(x + i, y + j, bg); // default size
                else
                    fillRect(x + i * size, y + j * size, size, size, bg); // big size
            }
            line >>= 1;
        }
    }
}

void setCursor(int16_t x, int16_t y) {
    display_param.cursor_x = x;
    display_param.cursor_y = y;
}

void setTextSize(uint8_t s) {
    display_param.textsize = (s > 0) ? s : 1;
}

void setTextColor(uint16_t c) {
    // For 'transparent' background, we'll set the bg
    // to the same as fg instead of using a flag
    display_param.textcolor = display_param.textbgcolor = c;
}

void setTextColor_bg(uint16_t c, uint16_t bg) {
    display_param.textcolor = c;
    display_param.textbgcolor = bg;
}

void setTextWrap(bool w) {
    display_param.wrap = w;
}

uint8_t getRotation(void) {
    return display_param.rotation;
}

void setRotation(uint8_t x) {
    display_param.rotation = (x & 3);
    switch (display_param.rotation) {
    case 0:
    case 2:
        display_param._width = display_param.WIDTH;
        display_param._height = display_param.HEIGHT;
        break;
    case 1:
    case 3:
        display_param._width = display_param.HEIGHT;
        display_param._height = display_param.WIDTH;
        break;
    }
}

// Return the size of the writeToDisplay_i2c_all_buffer (per current rotation)
int16_t width(void) {
    return display_param._width;
}

int16_t heigh(void) {
    return display_param._height;
}

// Return the size of the writeToDisplay_i2c_all_buffer (per current rotation)
int16_t height(void) {
    return display_param._height;
}

