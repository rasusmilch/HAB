/*
 *    LedControl.h - A library for controling Leds with a MAX7219/MAX7221
 *    Copyright (c) 2007 Eberhard Fahle
 *
 *    Permission is hereby granted, free of charge, to any person
 *    obtaining a copy of this software and associated documentation
 *    files (the "Software"), to deal in the Software without
 *    restriction, including without limitation the rights to use,
 *    copy, modify, merge, publish, distribute, sublicense, and/or sell
 *    copies of the Software, and to permit persons to whom the
 *    Software is furnished to do so, subject to the following
 *    conditions:
 *
 *    This permission notice shall be included in all copies or
 *    substantial portions of the Software.
 *
 *    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *    EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 *    OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 *    NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *    HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 *    WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *    FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 *    OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef LedControl_h
#define LedControl_h

#include <avr/pgmspace.h>

#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

/*
 * Segments to be switched on for characters and digits on
 * 7-Segment Displays
 */

/* MSB = 7, LSB = 0
 *  6666
 * 1    5
 * 1    5
 *  0000
 * 2    4
 * 2    4
 *  3333   7
 */
const static byte charTable [] PROGMEM  = {
    // ASCII char
    // 0 - 15
    B01111110, // 0
    B00110000, // 1
    B01101101, // 2
    B01111001, // 3
    B00110011, // 4
    B01011011, // 5
    B01011111, // 6
    B01110000, // 7
    B01111111, // 8
    B01111011, // 9
    B01110111, // A
    B00011111, // b
    B00001101, // c
    B00111101, // d
    B01001111, // E
    B01000111, // F

    // 16 - 23 CONTROL
    B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,

    // 24 - 31 CONTROL
    B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,B00000000,

    // 32 - 39
    B00000000, // SPACE
    B10100000, // !
    B00100010, // "
    B00000000, // #
    B00000000, // $
    B00100101, // %
    B00000000, // &
    B00100000, // '

    // 40 - 47
    B01001110, // ( eg, [
    B01111000, // ) eg, ]
    B00000000, // *
    B00000000, // +
    B10000000, // ,
    B00000001, // -
    B10000000, // .
    B00100101, // /

    // 48 - 55
    B01111110, // 0
    B00110000, // 1
    B01101101, // 2
    B01111001, // 3
    B00110011, // 4
    B01011011, // 5
    B01011111, // 6
    B01110000, // 7

    // 56 - 63
    B01111111, // 8
    B01111011, // 9
    B10100000, // :
    B10100000, // ;
    B00110001, // <
    B00001001, // =
    B00000111, // >
    B11110001, // ?

    // 64 - 71
    B00000000, // @
    B01110111, // A
    B00011111, // B
    B00001101, // C
    B00111101, // D
    B01001111, // E
    B01000111, // F
    B01011111, // G

    // 72 - 79
    B00110111, // H
    B00010000, // I
    B00111000, // J
    B00000000, // K
    B00001110, // L
    B01010101, // M
    B00010101, // N
    B00011101, // O

    // 80 - 87
    B01100111, // P
    B00000000, // Q
    B00000101, // R
    B01011011, // S
    B00000111, // T
    B00111110, // U
    B00011100, // V
    B00101011, // W

    // 88 - 95
    B00110111, // X
    B00111011, // Y
    B01101101, // Z
    B01001110, // [
    B00010011, // BACKSLASH
    B01111000, // ]
    B00000000, // ^
    B00001000, // _

    // 96 - 103
    B00000010, // '
    B01110111, // a
    B00011111, // b
    B00001101, // c
    B00111101, // d
    B01001111, // e
    B01000111, // f
    B01111011, // g

    // 104 - 111
    B00110111, // h
    B00010000, // i
    B00111000, // j
    B00000000, // k
    B00001110, // l
    B01010101, // m
    B00010101, // n
    B00011101, // o

    // 112 - 119
    B01100111, // p
    B00000000, // q
    B00000101, // r
    B01011011, // s
    B00000111, // t
    B00111110, // u
    B00011100, // v
    B00101011, // w

    // 120 - 127
    B00110111, // x
    B00111011, // y
    B01101101, // z
    B01001110, // {
    B00110000, // |
    B01111000, // }
    B00000000, // ~
    B00000000  // DEL
};

class LedControl {
    private :
        /* The array for shifting the data to the devices */
        byte spidata[16];
        /* Send out a single command to the device */
        void spiTransfer(int addr, byte opcode, byte data);

        /* We keep track of the led-status for all 8 devices in this array */
        byte status[64];
        /* Data is shifted out of this pin*/
        int SPI_MOSI;
        /* The clock is signaled on this pin */
        int SPI_CLK;
        /* This one is driven LOW for chip selectzion */
        int SPI_CS;
        /* The maximum number of devices we use */
        int maxDevices;

    public:
        /*
         * Create a new controler
         * Params :
         * dataPin		pin on the Arduino where data gets shifted out
         * clockPin		pin for the clock
         * csPin		pin for selecting the device
         * numDevices	maximum number of devices that can be controled
         */
        LedControl(int dataPin, int clkPin, int csPin, int numDevices=1);

        /*
         * Gets the number of devices attached to this LedControl.
         * Returns :
         * int	the number of devices on this LedControl
         */
        int getDeviceCount();

        /*
         * Set the shutdown (power saving) mode for the device
         * Params :
         * addr	The address of the display to control
         * status	If true the device goes into power-down mode. Set to false
         *		for normal operation.
         */
        void shutdown(int addr, bool status);

        /*
         * Set the number of digits (or rows) to be displayed.
         * See datasheet for sideeffects of the scanlimit on the brightness
         * of the display.
         * Params :
         * addr	address of the display to control
         * limit	number of digits to be displayed (1..8)
         */
        void setScanLimit(int addr, int limit);

        /*
         * Set the brightness of the display.
         * Params:
         * addr		the address of the display to control
         * intensity	the brightness of the display. (0..15)
         */
        void setIntensity(int addr, int intensity);

        /*
         * Switch all Leds on the display off.
         * Params:
         * addr	address of the display to control
         */
        void clearDisplay(int addr);

        /*
         * Set the status of a single Led.
         * Params :
         * addr	address of the display
         * row	the row of the Led (0..7)
         * col	the column of the Led (0..7)
         * state	If true the led is switched on,
         *		if false it is switched off
         */
        void setLed(int addr, int row, int col, boolean state);

        /*
         * Set all 8 Led's in a row to a new state
         * Params:
         * addr	address of the display
         * row	row which is to be set (0..7)
         * value	each bit set to 1 will light up the
         *		corresponding Led.
         */
        void setRow(int addr, int row, byte value);

        /*
         * Set all 8 Led's in a column to a new state
         * Params:
         * addr	address of the display
         * col	column which is to be set (0..7)
         * value	each bit set to 1 will light up the
         *		corresponding Led.
         */
        void setColumn(int addr, int col, byte value);

        /*
         * Display a hexadecimal digit on a 7-Segment Display
         * Params:
         * addr	address of the display
         * digit	the position of the digit on the display (0..7)
         * value	the value to be displayed. (0x00..0x0F)
         * dp	sets the decimal point.
         */
        void setDigit(int addr, int digit, byte value, boolean dp);

        /*
         * Display a character on a 7-Segment display.
         * There are only a few characters that make sense here :
         *	'0','1','2','3','4','5','6','7','8','9','0',
         *  'A','b','c','d','E','F','H','L','P',
         *  '.','-','_',' '
         * Params:
         * addr	address of the display
         * digit	the position of the character on the display (0..7)
         * value	the character to be displayed.
         * dp	sets the decimal point.
         */
        void setChar(int addr, int digit, char value, boolean dp);
};

#endif	//LedControl.h



