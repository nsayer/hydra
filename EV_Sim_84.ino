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

#define VERSION "(84) 0.2"

// The pins connected up to the LCD.
#define LCD_D4 7
#define LCD_D5 6
#define LCD_D6 5
#define LCD_D7 4
#define LCD_RS 3
#define LCD_E 1

#define PILOT_DIGITAL_SAMPLING_PIN  0

#define SAMPLE_PERIOD 500

#include <LiquidCrystal.h>

// Turn on auto-detect
LiquidCrystal display(LCD_RS, LCD_E, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

// special return result for the 'digital communications' pilot.
#define DIGITAL_COMM_REQD 999999
// duty is tenths-of-a-percent (that is, fraction out of 1000).
static inline unsigned long dutyToMA(unsigned long duty) {
  // Cribbed from the spec - grant a +/-2% slop
  if (duty < 30) {
    // < 3% is an error
    return 0;
  }
  else if (duty <= 70) {
    // 3-7% - digital
    return DIGITAL_COMM_REQD;
  }
  else if (duty < 80) {
    // 7-8% is an error
    return 0;
  }
  else if (duty <= 100) {
    // 8-10% is 6A
    return 6000;
  }
  else if (duty <= 850) { // 10-85% uses the "low" function
    return duty * 60;
  } 
  else if (duty <= 960) { // 85-96% uses the "high" function
    return (duty - 640) * 250;
  }
  else if (duty <= 970) {
    // 96-97% is 80A
    return 80000;
  }
  else { // > 97% is an error
    return 0;
  }
}

void setup() {
  
  pinMode(PILOT_DIGITAL_SAMPLING_PIN, INPUT_PULLUP);
    
  display.begin(16, 2);
  
  display.clear();
  
  display.print("EV Sim ");
  display.print(VERSION);
  
  delay(2000);
  display.clear();
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
  unsigned long amps = 0;
  if (state_changes == 0) {
    sprintf(buf, "   0 Hz     %s   ", (low_count>high_count)?"-":"+");
  } else {
    unsigned int duty = (high_count * 1000) / (high_count + low_count);
    duty %= 1000; // turn 100% into 0% just for display purposes. A 100% duty cycle doesn't really make sense.
  
    unsigned long frequency = (state_changes / 2) * (1000 / SAMPLE_PERIOD);

    amps = dutyToMA(duty);

    sprintf(buf, "%4ld Hz   %2d.%01d %%", frequency, duty / 10, duty % 10);
  }
  display.setCursor(0, 0);
  display.print(buf);

  if (amps == DIGITAL_COMM_REQD) {
    sprintf(buf, "Digital");
  } else {
    sprintf(buf, "%2ld.%02ld A", amps / 1000, (amps % 1000) / 10);
  }
  
  display.setCursor(0, 1);
  display.print(buf);
}
