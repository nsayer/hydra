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

#define VERSION "(84) 1.0"

// The pins connected up to the LCD.
#define LCD_D4 7
#define LCD_D5 6
#define LCD_D6 5
#define LCD_D7 4
#define LCD_RS 3
#define LCD_E 1

// Hardware versions prior to 4.0 use DIGITAL_SAMPLING. 4.0 and beyond use
// analog.
//
// You are allowed to define both. If you do, then digital sampling is used
// to obtain the duty cycle, while analog sampling is used for the high/low
// voltage measurement. It turns out, however, that both are interleaved,
// so the speed advantage of digital sampling is lost.
//
// A workaround for that would be to replace analogRead() with manual ADC control,
// and performing the digital sampling while waiting for the ADC to complete.
//
// It turns out, however, that analog sampling is good enough for our purposes.
//#define DIGITAL_SAMPLING
#define ANALOG_SAMPLING

#if !defined(DIGITAL_SAMPLING) && !defined(ANALOG_SAMPLING)
#error Well, you have to let me do it SOMEHOW...
#endif

#ifdef DIGITAL_SAMPLING
#define PILOT_DIGITAL_SAMPLING_PIN  0
#else
// Since we're not going to use digital sampling,
// we have to decide what constitutes a high and
// what constitutes a low for the purposes of the
// square wave sampling. This happens to be the 0 volt
// level.
#define ANALOG_STATE_TRANSITION_LEVEL 556
#endif

#ifdef ANALOG_SAMPLING
#define PILOT_ANALOG_SAMPLING_PIN 2
#endif

#define BUTTON_PIN 8

#define SAMPLE_PERIOD 500

#ifdef ANALOG_SAMPLING
// The button selects from different available display modes.
// At the moment, there are two: J1772 ampacity and min/max voltage
#define MODE_COUNT 2
#else
// There's no analog display mode without analog sampling
#define MODE_COUNT 1
#endif

// The different types of button events.
#define BUTTON_NONE 0
#define BUTTON_SHORT 1
#define BUTTON_LONG 2

// When the button state changes, further state changes are ignored for this many ms.
#define DEBOUNCE_INTERVAL 50
// The boundary between a BUTTON_SHORT and a BUTTON_LONG in ms.
#define LONG_PRESS_INTERVAL 250

// The scale and offset for the linear formula that relates A/D reading to pilot voltage.
// A circuit simulation shows the readback voltage as 4.543 at 12 volts in and 891 mV at -12 volts.
// Since a single A/D unit represents 4.88 mV, we can work out a formula to convert A/D readings
// into millivolts of the actual pilot signal
#ifdef ANALOG_SAMPLING
#define PILOT_READ_SCALE 32
#define PILOT_READ_OFFSET (-556)
#endif

#include <LiquidCrystal.h>

// Keep strings in PROGMEM. Copy them into this temp buffer just-in-time.
char p_buffer[96];
#define P(str) (strcpy_P(p_buffer, PSTR(str)), p_buffer)

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

static inline char read_button() {
  static unsigned char last_state = HIGH; // HIGH is unpressed, LOW is pressed
  static unsigned long debounce_time = 0; // ignore events until this time
  static unsigned long button_press_start = 0;
  if (debounce_time != 0) {
    if (millis() > debounce_time) {
      debounce_time = 0; // debounce period is over
    } else {
      return BUTTON_NONE;
    }
  }
  unsigned char state = digitalRead(BUTTON_PIN);
  if (state == last_state) return BUTTON_NONE;
  last_state = state;
  debounce_time = millis() + DEBOUNCE_INTERVAL;
  switch(state) {
    case LOW:
      button_press_start = millis();
      return BUTTON_NONE;
      break;
    case HIGH:
      unsigned char out = (millis() - button_press_start > LONG_PRESS_INTERVAL)?BUTTON_LONG:BUTTON_SHORT;
      button_press_start = 0;
      return out;
      break;
  }
}

#ifdef ANALOG_SAMPLING
static inline int scale_mv(unsigned int value) {
  return (((int)value) + PILOT_READ_OFFSET) * PILOT_READ_SCALE;
}
#endif

void setup() {
#ifdef DIGITAL_SAMPLING
  pinMode(PILOT_DIGITAL_SAMPLING_PIN, INPUT_PULLUP);
#endif
  pinMode(BUTTON_PIN, INPUT_PULLUP);
#ifdef ANALOG_SAMPLING
  pinMode(PILOT_ANALOG_SAMPLING_PIN, INPUT);
  analogReference(DEFAULT);
#endif
    
  display.begin(16, 2);
  
  display.clear();
  
  display.print(P("EV Sim "));
  display.print(P(VERSION));
  
  delay(2000);
  display.clear();
}

void loop() {
  static unsigned char mode = 0;
  unsigned int last_state = 99; // neither HIGH nor LOW
  unsigned long high_count = 0, low_count = 0, state_changes = -1; // ignore the first change from "invalid"
#ifdef ANALOG_SAMPLING
  unsigned int high_analog=0, low_analog=0xffff;
#endif

  for(unsigned long start_poll = millis(); millis() - start_poll < SAMPLE_PERIOD; ) {
    switch(read_button()) {
      case BUTTON_NONE:
        break;
      case BUTTON_SHORT:
      case BUTTON_LONG:
        if (++mode >= MODE_COUNT) mode = 0;
        break;
    }

#ifdef ANALOG_SAMPLING    
    unsigned int analog = analogRead(PILOT_ANALOG_SAMPLING_PIN);
    if (analog > high_analog) high_analog = analog;
    if (analog < low_analog) low_analog = analog;
#endif

    unsigned int state;
#ifdef DIGITAL_SAMPLING    
    state = digitalRead(PILOT_DIGITAL_SAMPLING_PIN);
#else
    state = (analog < ANALOG_STATE_TRANSITION_LEVEL)?LOW:HIGH;
#endif

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
    sprintf(buf, P("   0 Hz     %s   "), (low_count>high_count)?"-":"+");
  } else {
    unsigned int duty = (high_count * 1000) / (high_count + low_count);
    duty %= 1000; // turn 100% into 0% just for display purposes. A 100% duty cycle doesn't really make sense.
  
    unsigned long frequency = ((state_changes / 2) * 1000) / SAMPLE_PERIOD;

    amps = dutyToMA(duty);

    sprintf(buf, P("%4ld Hz   %2d.%01d %%"), frequency, duty / 10, duty % 10);
  }
  display.setCursor(0, 0);
  display.print(buf);

  switch(mode) {
    case 0:
      if (amps == DIGITAL_COMM_REQD) {
        sprintf(buf, P("Digital         "));
      } else {
        sprintf(buf, P("%2ld.%02ld A         "), amps / 1000, (amps % 1000) / 10);
      }
      break;
#ifdef ANALOG_SAMPLING
    case 1:
      int low_mv = scale_mv(low_analog);
      int high_mv = scale_mv(high_analog);
      sprintf(buf,P(" %+03d.%01d  %+03d.%01d"), low_mv/1000, abs(low_mv % 1000) / 100, high_mv/1000, abs(high_mv % 1000) / 100);
      break;
#endif
  }
  
  display.setCursor(0, 1);
  display.print(buf);
}
