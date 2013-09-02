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
 
#include <Wire.h>
#include <LiquidTWI2.h>

#define LCD_I2C_ADDR 0x20 // for adafruit shield or backpack

LiquidTWI2 display(LCD_I2C_ADDR, 1);

#define PILOT_DIGITAL_SAMPLING_PIN  3

#define SERIAL_BAUD_RATE 9600

#define SAMPLE_PERIOD 100

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
  
  Serial.begin(SERIAL_BAUD_RATE);
  
  display.setMCPType(LTI_TYPE_MCP23017);
  display.begin(16, 2);
  
  display.clear();
  display.setBacklight(WHITE);
  
  display.print("EV Sim v0.1");
  Serial.println("EV Sim v0.1");
  
  delay(2000);
}

void loop() {

  unsigned int last_state = LOW; // meh. If it's not, we'll be off by one
  unsigned long high_count = 0, low_count = 0, state_changes = 0;
  
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
  
  unsigned int duty = (high_count * 1000) / (high_count + low_count);
  
  unsigned long frequency = (state_changes / 2) * (1000 / SAMPLE_PERIOD);
  
  unsigned int amps = dutyToMA(duty);
  
  display.setCursor(0, 0);
  char buf[32];
  sprintf(buf, "%4ld Hz   %2d.%01d %%", frequency, duty / 10, duty % 10);
  display.print(buf);
  Serial.print(buf);
  Serial.print(" ");
  display.setCursor(0, 1);
  sprintf(buf, "%2d.%02d A", amps / 1000, (amps % 1000) / 10);
  display.print(buf);
  Serial.println(buf);
}
