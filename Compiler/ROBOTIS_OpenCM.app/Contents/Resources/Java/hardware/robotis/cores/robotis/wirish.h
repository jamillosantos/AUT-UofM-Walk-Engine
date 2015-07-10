/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Perry Hung.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @brief Main include file for the Wirish core.
 *
 * Includes various Arduino wiring macros and bit defines
 */

#ifndef _WIRISH_H_
#define _WIRISH_H_

//#include "libpandora.h"

//#include "wirish_types.h"
#include "boards.h"
//#include "io.h"
#include "bits.h"
#include "pwm.h"
#include "ext_interrupts.h"
//#include "wirish_debug.h"
//#include "wirish_math.h"
//#include "wirish_time.h"
#include "HardwareSPI.h"
#include "HardwareSerial.h"
#include "HardwareTimer.h"
#include "usb_serial.h"

//[ROBOTIS]add to support dynamixel that is super powered robot actuator
#include "Dynamixel.h"
#include "Arduino-compatibles.h" //we need to decrease object files for shortening download time.

/* Arduino wiring macros and bit defines  */
#define HIGH 0x1
#define LOW  0x0

//**********************************************
//Global Deffenition  //for MK (AUT-UofM)
#define DEG2RAD      (0.0174532925199432957)
#define RAD2DEG      (57.295779513082320876)
#define DEG2DXL      (11.377777776666666668)
#define Pi           (3.1415926535897932384)
#define TwoPi        (6.2831853071795864769)

//Global Deffenition of low and high byte of integer registers
#define _LOBYTE(w)   ((unsigned char)(((unsigned long)(w)) & 0xff))        //get low byte (8 first bits) from one 16 bit value
#define _HIBYTE(w)   ((unsigned char)((((unsigned long)(w)) >> 8) & 0xff)) //get high byte (8'th to 16'th bits) from 16 bit value

/* Serial device defines for dxl bus */
#define DXL_BUS_SERIAL1    1  //Dynamixel on Serial1(USART1)  <-OpenCM9.04
#define DXL_BUS_SERIAL2    2  //Dynamixel on Serial2(USART2)  <-LN101,BT210
#define DXL_BUS_SERIAL3    3  //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP

// Dynamixel Baudrate -> 0: 9600, 1: 57600, 2: 115200, 3: 1Mbps
#define Boudrate_9600bps      0
#define Boudrate_57600bps     1
#define Boudrate_115200bps    2
#define Boudrate_1000000bps   3

/* 485 EXP switch, LED definces for 485 EXP device */
#define RED_LED_485EXP     18
#define GREEN_LED_485EXP   19
#define BLUE_LED_485EXP    20
#define BUTTON1_485EXP     16
#define BUTTON2_485EXP     17

#define Buzzer_Pin   2

//********************************************

#define true 0x1
#define false 0x0

#define LSBFIRST 0
#define MSBFIRST 1

//#define lowByte(w)                     ((w) & 0xFF)  //can be replaced by DXL_LOBYTE()
//#define highByte(w)                    (((w) >> 8) & 0xFF) //can be replaced by DXL_LOBYTE()
#define bitRead(value, bit)            (((value) >> (bit)) & 0x01)
#define bitSet(value, bit)             ((value) |= (1UL << (bit)))
#define bitClear(value, bit)           ((value) &= ~(1UL << (bit)))
#define bitWrite(value, bit, bitvalue) (bitvalue ? bitSet(value, bit) : \
                                                   bitClear(value, bit))
#define bit(b)                         (1UL << (b))

typedef uint8 boolean;
typedef uint8 byte;

//typedef int8_t boolean;
//typedef int8_t byte;

//typedef uint8_t boolean;
//typedef uint8_t byte;

/*************************************************************************************************************
 * Removed wirish_debug.h by ROBOTIS,.LTD. 2013-04-25
 * debug codes move to wirish.h
 * @brief High level debug port configuration
 *
 *************************************************************************************************************/

/**
 * @brief Disable the JTAG and Serial Wire (SW) debug ports.
 *
 * You can call this function in order to use the JTAG and SW debug
 * pins as ordinary GPIOs.
 *
 * @see enableDebugPorts()
 */
static inline void disableDebugPorts(void) {
    afio_cfg_debug_ports(AFIO_DEBUG_NONE);
}

/**
 * @brief Enable the JTAG and Serial Wire (SW) debug ports.
 *
 * After you call this function, the JTAG and SW debug pins will no
 * longer be usable as GPIOs.
 *
 * @see disableDebugPorts()
 */
static inline void enableDebugPorts(void) {
    afio_cfg_debug_ports(AFIO_DEBUG_FULL_SWJ);
}

#endif

