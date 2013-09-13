/*

 J1772 EV Pilot Analysis sketch for Arduino
 Copyright 2013 Nicholas W. Sayer
 
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

#include <Arduino.h>

#define VERSION "0.3"

// PB1. 0 and 2 are i2c or serial, 3 & 4 are xtal, 5 is reset.
#define PILOT_DIGITAL_SAMPLING_PIN  1

#define SAMPLE_PERIOD 1000

// Set this to 0 for i2c LCD or the SoftwareSerial baud rate to use
#define SERIAL_BAUD_RATE 0

#if SERIAL_BAUD_RATE > 0
#include <SoftwareSerial.h>

// SCK is the transmit pin, SDA is the receive pin.
SoftwareSerial serial(0, 2);

#else
#include <TinyWireM.h>
#include <LiquidTWI2.h>

#define LCD_I2C_ADDR 0x20 // for adafruit shield or backpack

LiquidTWI2 display(LCD_I2C_ADDR, 1);

#endif

// duty is tenths-of-a-percent (that is, fraction out of 1000).
inline unsigned long dutyToMA(unsigned long duty) {
  if (duty < 100) { // < 10% is an error
    return 0;
  } 
  else if (duty < 850) { // 10-85% uses the "low" function
    return (int)((duty) * 60);
  } 
  else if (duty <= 960) { // 85-96% uses the "high" function
    return (int)((duty - 640) * 250);
  } 
  else { // > 96% is an error
    return 0;
  }
}

void setup() {
  
  pinMode(PILOT_DIGITAL_SAMPLING_PIN, INPUT);
    
#if SERIAL_BAUD_RATE > 0
  serial.begin(SERIAL_BAUD_RATE);
  serial.print("EV Sim ");
  serial.print(VERSION);
  serial.print("\r\n");
#else
  display.setMCPType(LTI_TYPE_MCP23017);
  display.begin(16, 2);
  
  display.clear();
  display.setBacklight(WHITE);
  
  display.print("EV Sim ");
  display.print(VERSION);
  
  delay(2000);
  display.clear();
#endif
}

void loop() {

  unsigned int last_state = 99; // neither HIGH nor LOW
  unsigned long high_count = 0, low_count = 0, state_changes = -1; // ignore the first change from "invalid"
  
  for(unsigned long start_poll = millis(); millis() - start_poll < SAMPLE_PERIOD; ) {
    unsigned int state = digitalRead(PILOT_DIGITAL_SAMPLING_PIN);
    
    if (state == LOW)
      low_count++;
    else
      high_count++;
      
    if (state != last_state) {
      state_changes++;
      last_state = state;
    }
  }
  char buf[32];
  unsigned int amps = 0;
  if (state_changes == 0) {
    sprintf(buf, "   0 Hz     %s   ", (low_count>high_count)?"-":"+");
  } else {
    unsigned int duty = (high_count * 1000) / (high_count + low_count);
    duty %= 1000; // turn 100% into 0% just for display purposes. A 100% duty cycle isn't really possible.
  
    unsigned long frequency = (state_changes / 2) * (1000 / SAMPLE_PERIOD);
  
    amps = dutyToMA(duty);
  
    sprintf(buf, "%4ld Hz   %2d.%01d %%", frequency, duty / 10, duty % 10);
  }
#if SERIAL_BAUD_RATE > 0
  serial.print(buf);
  serial.print(" ");
#else
  display.setCursor(0, 0);
  display.print(buf);
#endif
  sprintf(buf, "%2d.%02d A", amps / 1000, (amps % 1000) / 10);
#if SERIAL_BAUD_RATE > 0
  serial.print(buf);
  serial.print("\r\n");
#else
  display.setCursor(0, 1);
  display.print(buf);
#endif
}
