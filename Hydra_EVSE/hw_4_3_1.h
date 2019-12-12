/*

  J1772 Hydra (EVSE variant) for Arduino
  Copyright 2014 Nicholas W. Sayer, Dmitriy Lyubimov

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License along
  with this program; if not, write to the Free Software Foundation, Inc.,
  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
*/

///////////////////////////////////////////////////
// Definitions of pins etc. related to a particular hardware

// HW version
#define HW_VERSION "4.3.1"

// 2.3.1 hardware display

// for adafruit shield or backpack
#define LCD_I2C_ADDR 0x20 
#define DISPLAY_DEF(name) LiquidTWI2 name(LCD_I2C_ADDR, 1)
#define DISPLAY_DECL(name) extern LiquidTWI2 name

#define GFI_PIN                 2
#define GFI_IRQ                 0

#define GFI_TEST_PIN            3

#if !defined(SWAP_CARS)
 
// ---------- DIGITAL PINS ----------
#define CAR_A_PILOT_OUT_PIN     10
#define CAR_B_PILOT_OUT_PIN     9

#define CAR_A_RELAY             8
#define CAR_B_RELAY             7

#ifdef RELAY_TEST
#define CAR_A_RELAY_TEST        A3
#define CAR_B_RELAY_TEST        A2
#endif

// ---------- ANALOG PINS ----------
#define CAR_A_PILOT_SENSE_PIN   1
#define CAR_B_PILOT_SENSE_PIN   0
#ifdef RELAY_TEST
// When the relay test was added, the current pins were moved.
#define CAR_A_CURRENT_PIN       7
#define CAR_B_CURRENT_PIN       6
#else
#define CAR_A_CURRENT_PIN       3
#define CAR_B_CURRENT_PIN       2
#endif
#else
// ---------- DIGITAL PINS ----------
#define CAR_A_PILOT_OUT_PIN     9
#define CAR_B_PILOT_OUT_PIN     10

#define CAR_A_RELAY             7
#define CAR_B_RELAY             8

#ifdef RELAY_TEST
#define CAR_A_RELAY_TEST        A2
#define CAR_B_RELAY_TEST        A3
#endif

// ---------- ANALOG PINS ----------
#define CAR_A_PILOT_SENSE_PIN   0
#define CAR_B_PILOT_SENSE_PIN   1
#ifdef RELAY_TEST
#define CAR_A_CURRENT_PIN       6
#define CAR_B_CURRENT_PIN       7
#else
#define CAR_A_CURRENT_PIN       2
#define CAR_B_CURRENT_PIN       3
#endif
#endif

