/*

 J1772 Hydra for Arduino
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
#include <PWM.h>

#define LCD_I2C_ADDR 0x20 // for adafruit shield or backpack

// ---------- DIGITAL PINS ----------
#define INCOMING_PILOT_PIN      2
#define INCOMING_PILOT_INT      0

#define INCOMING_PROXIMITY_PIN  3
#define INCOMING_PROXIMITY_INT  1

#define OUTGOING_PROXIMITY_PIN  4

#define CAR_A_PILOT_OUT_PIN     9
#define CAR_B_PILOT_OUT_PIN     10

#define CAR_A_RELAY             7
#define CAR_B_RELAY             8

// ---------- ANALOG PINS ----------
#define CAR_A_PILOT_SENSE_PIN   0
#define CAR_B_PILOT_SENSE_PIN   1
#define CAR_A_CURRENT_PIN       2
#define CAR_B_CURRENT_PIN       3

#if 0
// This is for hardware version 0.5 only, which is now obsolete.
#define CAR_A_PILOT_SENSE_PIN   0
#define CAR_A_CURRENT_PIN       1
#define CAR_B_PILOT_SENSE_PIN   2
#define CAR_B_CURRENT_PIN       3
#endif

// for things like erroring out a car
#define BOTH                    0
#define CAR_A                   1
#define CAR_B                   2

// Don't use 0 or 1 because that's the value of LOW and HIGH.
#define HALF                    3
#define FULL                    4

#define STATE_A                 1
#define STATE_B                 2
#define STATE_C                 3
#define STATE_D                 4
#define STATE_E                 5
#define DUNNO                   0

// These are the expected analogRead() ranges for pilot read-back from the cars.
// These are calculated from the expected voltages seen through the dividor network,
// then scaling those voltages for 0-1024.

// 11 volts
#define STATE_A_MIN      900
// 10 volts
#define STATE_B_MAX      868
// 8 volts
#define STATE_B_MIN      799
// 7 volts
#define STATE_C_MAX      774
// 5 volts
#define STATE_C_MIN      712
// 4 volts
#define STATE_D_MAX      681
// 2 volts
#define STATE_D_MIN      618
// This represents 0 volts. No, it's not 512. Deal.
#define PILOT_0V         556
// -10 volts. We're fairly generous.
#define PILOT_DIODE_MAX  250

// This is the amount the incoming pilot needs to change for us to react (in milliamps).
#define PILOT_FUZZ 500

// This is how long we allow a car to draw more current than it is allowed before we
// error it out (in milliseconds). The spec says that a car is supposed to have 5000
// msec to respond to a pilot reduction, but it also says that we must respond to a
// state C transition within 5000 ms, so something has to give.
#define OVERDRAW_GRACE_PERIOD 4000

// This is how much "slop" we allow a car to have in terms of keeping under its current allowance.
// That is, this value (in milliamps) plus the calculated current limit is what we enforce.
#define OVERDRAW_GRACE_AMPS 1000

// The time between withdrawing the pilot from a car and disconnecting its relay (in milliseconds).
// The spec says that this must be no shorter than 3000 ms.
#define ERROR_DELAY 3000

// When a car requests state C while the other car is already in state C, we delay them for
// this long while the other car transitions to half power. THIS INTERVAL MUST BE LONGER
// THAN THE OVERDRAW_GRACE_PERIOD! (in milliseconds) The spec says it must be shorter than
// 5000 ms.
#define TRANSITION_DELAY 4500

// This is the current limit (in milliamps) of all of the components on the inlet side of the hydra -
// the inlet itself, any fuses, and the wiring to the common sides of the relays.
#define MAXIMUM_INLET_CURRENT 75000

// This is the minimum of the ampacity (in milliamps) of all of the components from the relay to the plug -
// The relay itself, the J1772 cable and plug.
#define MAXIMUM_OUTLET_CURRENT 30000

// This can not be lower than 12, because the J1772 spec bottoms out at 6A.
// The hydra won't operate properly if it can't divide the incoming power in half. (in milliamps)
#define MINIMUM_INLET_CURRENT 12000

// This is the amount of current (in milliamps) we subtract from the inlet before apportioning it to the cars.
#define INLET_CURRENT_DERATE 0

// Amount of time, in milliseconds, we will analyse the duty cycle of the incoming pilot
// Default is 500 cycles. We're doing a digitalRead(), so this will be thousands of samples.
#define PILOT_POLL_INTERVAL 50

// Amount of time, in milliseconds, we will look for positive and negative peaks on the car
// pilot pins. It takes around .1 ms to do one read, so we should get a bit less than 1000 chances
// this way.
#define STATE_CHECK_INTERVAL 100

// How often (in milliseconds) is the state of both cars logged?
#define STATE_LOG_INTERVAL 60000

// This is the number of duty cycle or ammeter samples we keep to make a rolling average to stabilize
// the display. The balance here is between stability and responsiveness,
#define ROLLING_AVERAGE_SIZE 5

// The maximum number of milliseconds to sample an ammeter pin in order to find three zero-crossings.
// one and a half cycles at 50 Hz is 30 ms.
#define CURRENT_SAMPLE_INTERVAL 35

// Once we detect a zero-crossing, we should not look for one for another quarter cycle or so. 1/4
// cycle at 50 Hz is 5 ms.
#define CURRENT_ZERO_DEBOUNCE_INTERVAL 5

// How often (in milliseconds) is the current draw by a car logged?
#define CURRENT_LOG_INTERVAL 1000

// This multiplier is the number of milliamps per A/d converter unit.

// First, you need to select the burden resistor for the CT. You choose the largest value possible such that
// the maximum peak-to-peak voltage for the current range is 5 volts. To obtain this value, divide the maximum
// outlet current by the Te. That value is the maximum CT current RMS. You must convert that to P-P, so multiply
// by 2*sqrt(2). Divide 5 by that value and select the next lower standard resistor value. For the reference
// design, Te is 1018 and the outlet maximum is 30. 5/((30/1018)*2*sqrt(2)) = 59.995, so a 56 ohm resistor
// is called for. Call this value Rb (burden resistor).

// Next, one must use Te and Rb to determine the volts-per-amp value. Note that the readCurrent()
// method calculates the RMS value before the scaling factor, so RMS need not be taken into account.
// (1 / Te) * Rb = Rb / Te = Volts per Amp. For the reference design, that's 55.009 mV.

// Each count of the A/d converter is 4.882 mV (5/1024). V/A divided by V/unit is unit/A. For the reference
// design, that's 11.26. But we want milliamps per unit, so divide that into 1000 to get...
#define CURRENT_SCALE_FACTOR 88.7625558

#define LOG_NONE 0
#define LOG_INFO 1
#define LOG_DEBUG 2

// Hardware versions 1.0 and beyond have a 6 pin FTDI compatible port laid out on the board.
// We're going to use this sort of "log4j" style. The log level is 0 for no logging at all
// (and if it's 0, the Serial won't be initialized), 1 for info level logging (which will
// simply include state transitions only), or 2 for debugging.
#define SERIAL_LOG_LEVEL LOG_INFO
#define SERIAL_BAUD_RATE 9600

#define VERSION "0.9.4.3 beta"

LiquidTWI2 display(LCD_I2C_ADDR, 1);

unsigned long incoming_pilot_samples[ROLLING_AVERAGE_SIZE];
unsigned long car_a_current_samples[ROLLING_AVERAGE_SIZE], car_b_current_samples[ROLLING_AVERAGE_SIZE];
unsigned long incomingPilotMilliamps, lastIncomingPilot;
unsigned int last_car_a_state, last_car_b_state;
unsigned long car_a_overdraw_begin, car_b_overdraw_begin;
unsigned long car_a_request_time, car_b_request_time;
unsigned long car_a_error_time, car_b_error_time;
unsigned long last_current_log_car_a, last_current_log_car_b;
unsigned long last_state_log;
unsigned int relay_state_a, relay_state_b;
unsigned int lastProximity;

void log(unsigned int level, const char * fmt_str, ...) {
#if SERIAL_LOG_LEVEL > 0
  if (level > SERIAL_LOG_LEVEL) return;
  char buf[256]; // Danger, Will Robinson!
  va_list argptr;
  va_start(argptr, fmt_str);
  vsnprintf(buf, sizeof(buf), fmt_str, argptr);
  va_end(argptr);

  switch(level) {
  case LOG_INFO: 
    Serial.print("INFO: "); 
    break;
  case LOG_DEBUG: 
    Serial.print("DEBUG: "); 
    break;
  default: 
    Serial.print("UNKNOWN: "); 
    break;
  }
  Serial.println(buf);
#endif
}

inline const char *car_str(unsigned int car) {
  switch(car) {
    case CAR_A: return "car A";
    case CAR_B: return "car B";
    case BOTH: return "both car";
    default: return "UNKNOWN";
  }
}

inline const char *logic_str(unsigned int state) {
  switch(state) {
    case LOW: return "LOW";
    case HIGH: return "HIGH";
    case HALF: return "HALF";
    case FULL: return "FULL";
    default: return "UNKNOWN";
  }
}

inline const char* state_str(unsigned int state) {
  switch(state) {
  case STATE_A: 
    return "A";
  case STATE_B: 
    return "B";
  case STATE_C: 
    return "C";
  case STATE_D: 
    return "D";
  case STATE_E: 
    return "E";
  default: 
    return "UNKNOWN";
  }
}

// Deal in milliamps so that we don't have to use floating point.
// Convert the microsecond state timings from the incoming pilot into
// an instantaneous current allowance value. With polling, it's
// the two sample counts, but the math winds up being exactly the same.
inline unsigned long timeToMA(unsigned long msecHigh, unsigned long msecLow) {
  // First, normalize the frequency to 1000 Hz.
  // msec is the number of microseconds out of 1000 that the signal stayed high
  long msec = (msecHigh * 1000) / (msecHigh + msecLow);
  if (msec < 100) { // < 10% is an error
    return 0;
  } 
  else if (msec < 850) { // 10-85% uses the "low" function
    return (int)((msec) * 60);
  } 
  else if (msec <= 960) { // 85-96% uses the "high" function
    return (int)((msec - 640) * 250);
  } 
  else { // > 96% is an error
    return 0;
  }
}

// Convert a milliamp allowance into an outgoing pilot duty cycle.
// In lieu of floating point, this is microseconds high out of 1000.
inline unsigned long MAToMsec(unsigned long milliamps) {
  if (milliamps < 6000) {
    return 9999; // illegal - set pilot to "high"
  } 
  else if (milliamps < 51000) {
    return milliamps/60;
  } 
  else if (milliamps <= 80000) {
    return (milliamps / 250) + 640;
  } 
  else {
    return 9999; // illegal - set pilot to "high"
  }
}

// Convert a milliamp allowance into a value suitable for
// pwmWrite - a scale from 0 to 255.
inline unsigned int MAtoPwm(unsigned long milliamps) {
  unsigned long out = MAToMsec(milliamps);

  if (out >= 1000) return 255; // full on

  out =  (out * 256) / 1000;  

  return (unsigned int)out;
}

// Turn a millamp value into nn.n as amps, with the tenth rounded near.
char *formatMilliamps(unsigned long milliamps) {
  static char out[6];

  if (milliamps < 1000) {
    // truncate the units digit - there's no way we're that accurate.
    milliamps /= 10;
    milliamps *= 10;

    sprintf(out, "%3lumA", milliamps);
  } 
  else {
    int hundredths = (milliamps / 10) % 100;
    int tenths = hundredths / 10 + (((hundredths % 10) >= 5)?1:0);
    int units = milliamps / 1000;
    if (tenths >= 10) {
      tenths -= 10;
      units++;
    }

    sprintf(out, "%2d.%01dA", units, tenths);
  }

  return out;
}

void error(unsigned int car, char err) {
  unsigned long now = millis(); // so both cars get the same time.
  // Set the pilot to constant 12: indicates an EVSE error.
  // We can't use -12, because then we'd never detect a return
  // to state A. But the spec says we're allowed to return to B1
  // (that is, turn off the oscillator) whenever we want.

  if (car == BOTH || car == CAR_A) {
    setPilot(CAR_A, HIGH);
    last_car_a_state = STATE_E;
    car_a_error_time = now;
    car_a_request_time = 0;
  }
  if (car == BOTH || car == CAR_B) {
    setPilot(CAR_B, HIGH);
    last_car_b_state = STATE_E;
    car_b_error_time = now;
    car_b_request_time = 0;
  }
  
  display.setBacklight(RED);
  if (car == BOTH || car == CAR_A) {
    display.setCursor(0, 1);
    display.print("A:ERR ");
    display.print(err);
    display.print(" ");
  }
  if (car == BOTH || car == CAR_B) {
    display.setCursor(8, 1);
    display.print("B:ERR ");
    display.print(err);
    display.print(" ");
  }

  log(LOG_INFO, "Error %c on %s", err, car_str(car));
}

void setRelay(unsigned int car, unsigned int state) {
  log(LOG_DEBUG, "Setting %s relay to %s", car_str(car), logic_str(state));
  switch(car) {
  case CAR_A:
    digitalWrite(CAR_A_RELAY, state);
    relay_state_a = state;
    break;
  case CAR_B:
    digitalWrite(CAR_B_RELAY, state);
    relay_state_b = state;
    break;
  }
}

// If it's in an error state, it's not charging (the relay may still be on during error delay).
// If it's in a transition delay, then it's "charging" (the relay is off during transition delay).
// Otherwise, check the state of the relay.
inline boolean isCarCharging(unsigned int car) {
  switch(car) {
  case CAR_A:
    if (last_car_a_state == STATE_E) return LOW;
    if (car_a_request_time != 0) return HIGH;
    return relay_state_a;
    break;
  case CAR_B:
    if (last_car_b_state == STATE_E) return LOW;
    if (car_b_request_time != 0) return HIGH;
    return relay_state_b;
    break;
  default:
    return LOW; // This should not be possible
  }
}

// Set the pilot for the car as appropriate. 'which' is either HALF, FULL, LOW or HIGH.
// HIGH sets a constant +12v, which is the spec for state A, but we also use it for
// state E. HALF means that the other car is charging, so we only can have half power.

void setPilot(unsigned int car, unsigned int which) {
  log(LOG_DEBUG, "Setting %s pilot to %s", car_str(car), logic_str(which));
  // set the outgoing pilot for the given car to either HALF state, FULL state, or HIGH.
  int pin = (car == CAR_A) ? CAR_A_PILOT_OUT_PIN : CAR_B_PILOT_OUT_PIN;
  if (which == LOW || which == HIGH) {
    // This is what the pwm library does anyway.
    digitalWrite(pin, which);
  } 
  else {
    unsigned long ma = incomingPilotMilliamps;
    if (which == HALF) ma /= 2;
    if (ma > MAXIMUM_OUTLET_CURRENT) ma = MAXIMUM_OUTLET_CURRENT;
    pwmWrite(pin, MAtoPwm(ma));
  }
}

int checkState(unsigned int car) {
  // poll the pilot state pin for 10 ms (should be 10 pilot cycles), looking for the low and high.
  unsigned int low = 9999, high = 0;
  unsigned int car_pin = (car == CAR_A) ? CAR_A_PILOT_SENSE_PIN : CAR_B_PILOT_SENSE_PIN;
  for(unsigned long now = millis(); millis() - now < STATE_CHECK_INTERVAL; ) {
    unsigned int val = analogRead(car_pin);
    if (val > high) high = val;
    if (val < low) low = val;
  }

  // If the pilot low was below zero, then that means we must have
  // been oscillating. If we were, then perform the diode check.
  if (low < PILOT_0V && low > PILOT_DIODE_MAX) {
    return STATE_E; // diode check fail
  }
  if (high >= STATE_A_MIN) {
    return STATE_A;
  } 
  else if (high >= STATE_B_MIN && high <= STATE_B_MAX) {
    return STATE_B;
  } 
  else if (high >= STATE_C_MIN && high <= STATE_C_MAX) {
    return STATE_C;
  } 
  else if (high >= STATE_D_MIN && high <= STATE_D_MAX) {
    return STATE_D;
  }
  // I dunno how we got here, but we fail it.
  return STATE_E;
}

unsigned long readCurrent(unsigned int car) {
  unsigned int car_pin = (car == CAR_A) ? CAR_A_CURRENT_PIN : CAR_B_CURRENT_PIN;
  unsigned long sum = 0;
  unsigned int zero_crossings = 0;
  unsigned long last_zero_crossing_time = 0, now_ms;
  long last_sample = -1; // should be impossible - the A/d is 0 to 1023.
  unsigned int sample_count = 0;
  for(unsigned long start = millis(); (now_ms = millis()) - start < CURRENT_SAMPLE_INTERVAL; ) {
    long sample = analogRead(car_pin);
    // If this isn't the first sample, and if the sign of the value differs from the
    // sign of the previous value, then count that as a zero crossing.
    if (last_sample != -1 && ((last_sample > 512) != (sample > 512))) {
      // Once we've seen a zero crossing, don't look for one for a little bit.
      // It's possible that a little noise near zero could cause a two-sample
      // inversion.
      if (now_ms - last_zero_crossing_time > CURRENT_ZERO_DEBOUNCE_INTERVAL) {
        zero_crossings++;
        last_zero_crossing_time = now_ms;
      }
    }
    last_sample = sample;
    switch(zero_crossings) {
    case 0: 
      continue; // Still waiting to start sampling
    case 1:
    case 2:
      // Gather the sum-of-the-squares and count how many samples we've collected.
      sum += (unsigned long)((sample - 512) * (sample - 512));
      sample_count++;
      continue;
    case 3:
      // The answer is the square root of the mean of the squares.
      // But additionally, that value must be scaled to a real current value.
      return (unsigned long)(sqrt(sum / sample_count) * CURRENT_SCALE_FACTOR);
    }
  }
  // ran out of time. Assume that it's simply not oscillating any. 
  return 0;
}

unsigned long rollRollingAverage(unsigned long array[], unsigned long new_value) {
#if ROLLING_AVERAGE_SIZE == 0
  return new_value;
#else
  unsigned long sum = new_value;
  for(int i = ROLLING_AVERAGE_SIZE - 1; i >= 1; i--) {
    array[i] = array[i - 1];
    sum += array[i];
  }
  array[0] = new_value;
  return (sum / ROLLING_AVERAGE_SIZE);
#endif
}

inline void reportIncomingPilot(unsigned long milliamps) {

  milliamps = rollRollingAverage(incoming_pilot_samples, milliamps);
  // Clamp to the maximum allowable current
  if (milliamps > MAXIMUM_INLET_CURRENT) milliamps = MAXIMUM_INLET_CURRENT;

  milliamps -= INLET_CURRENT_DERATE;

  incomingPilotMilliamps = milliamps;
}

void pollIncomingPilot() {
  unsigned long now = millis();
  unsigned long high_count = 0, low_count = 0;
  while(millis() - now < PILOT_POLL_INTERVAL) {
    if (digitalRead(INCOMING_PILOT_PIN) == HIGH) high_count++;
    else
      low_count++;
  }

  if (high_count == 0 && low_count == 0) {
    // Something has gone wrong. We'll just assume a limit of 0
    // to avoid divide-by-zero later.
    reportIncomingPilot(0);
    return;
  }
  unsigned long milliamps = timeToMA(high_count, low_count);

  reportIncomingPilot(milliamps);

}

void setup() {
  InitTimersSafe();
  display.setMCPType(LTI_TYPE_MCP23017);
  display.begin(16, 2); 

#if SERIAL_LOG_LEVEL > 0
  Serial.begin(SERIAL_BAUD_RATE);
#endif

  log(LOG_DEBUG, "Starting v%s", VERSION);
  
  pinMode(INCOMING_PILOT_PIN, INPUT);
  pinMode(INCOMING_PROXIMITY_PIN, INPUT);
  pinMode(OUTGOING_PROXIMITY_PIN, OUTPUT);
  pinMode(CAR_A_PILOT_OUT_PIN, OUTPUT);
  pinMode(CAR_B_PILOT_OUT_PIN, OUTPUT);
  pinMode(CAR_A_RELAY, OUTPUT);
  pinMode(CAR_B_RELAY, OUTPUT);

  digitalWrite(OUTGOING_PROXIMITY_PIN, LOW);

  // Enter state A on both cars
  setPilot(CAR_A, HIGH);
  setPilot(CAR_B, HIGH);
  // And make sure the power is off.
  setRelay(CAR_A, LOW);
  setRelay(CAR_B, LOW);

  memset(car_a_current_samples, 0, sizeof(car_a_current_samples));
  memset(car_b_current_samples, 0, sizeof(car_b_current_samples));
  last_car_a_state = DUNNO;
  last_car_b_state = DUNNO;
  car_a_request_time = 0;
  car_b_request_time = 0;
  car_a_overdraw_begin = 0;
  car_b_overdraw_begin = 0;
  last_current_log_car_a = 0;
  last_current_log_car_b = 0;
  lastProximity = HIGH;

  display.setBacklight(WHITE);
  display.clear();
  display.setCursor(0, 0);
  display.print("J1772 Hydra");
  display.setCursor(0, 1);
  display.print(VERSION);

  boolean success = SetPinFrequency(CAR_A_PILOT_OUT_PIN, 1000);
  if (!success) {
    log(LOG_INFO, "SetPinFrequency for car A failed!");
    display.setBacklight(YELLOW);
  }
  success = SetPinFrequency(CAR_B_PILOT_OUT_PIN, 1000);
  if (!success) {
    log(LOG_INFO, "SetPinFrequency for car B failed!");
    display.setBacklight(BLUE);
  }
  // In principle, neither of the above two !success conditions should ever
  // happen.

  // meanwhile, the fill in the incoming pilot rolling average...
  for(int i = 0; i < ROLLING_AVERAGE_SIZE; i++) {
    pollIncomingPilot();
  }
  lastIncomingPilot = incomingPilotMilliamps;

  // Display the splash screen for 2 seconds total. We spent some time above sampling the
  // pilot, so don't include that.
  delay(2000 - (ROLLING_AVERAGE_SIZE * PILOT_POLL_INTERVAL)); // let the splash screen show
  display.clear();
}

void loop() {

  // cut down on how frequently we call millis()
  unsigned long now = millis();
  boolean proximityOrPilotError = false;
  
  // Update the display

  if (last_car_a_state == STATE_E || last_car_b_state == STATE_E) {
    // One or both cars in error state
    display.setBacklight(RED);
  } 
  else {
    boolean a = isCarCharging(CAR_A);
    boolean b = isCarCharging(CAR_B);

    // Neither car
    if (!a && !b) display.setBacklight(GREEN);
    // Both cars
    else if (a && b) display.setBacklight(VIOLET);
    // One car or the other
    else if (a ^ b) display.setBacklight(TEAL);
  }

  // Check proximity
  unsigned int proximity = digitalRead(INCOMING_PROXIMITY_PIN);
  if (proximity != lastProximity) {
    if (proximity != HIGH) {

      log(LOG_INFO, "Incoming proximity disconnect");
      
      // EVs are supposed to react to a proximity transition much faster than
      // an error transition.
      digitalWrite(OUTGOING_PROXIMITY_PIN, HIGH);

      display.setCursor(0, 0);
      display.print("DISCONNECTING...");
      error(BOTH, 'P');
    } 
    else {
      log(LOG_INFO, "Incoming proximity restore");
      
      // In case someone pushed the button and changed their mind
      digitalWrite(OUTGOING_PROXIMITY_PIN, LOW);
    }
  }
  lastProximity = proximity;
  if (proximity != HIGH) proximityOrPilotError = true;


  pollIncomingPilot();
  if (incomingPilotMilliamps < MINIMUM_INLET_CURRENT) {
    if (last_car_a_state != STATE_E || last_car_b_state != STATE_E) { 
      display.setCursor(0, 0);
      display.print("INPUT PILOT ERR ");
      error(BOTH, 'I');
    }
    // Forget it. Nothing else is worth doing as long as the input pilot continues to be gone.
    proximityOrPilotError = true;
  }

  if(!proximityOrPilotError) {
    display.setCursor(0, 0);
    display.print("EVSE power ");
    display.print(formatMilliamps(incomingPilotMilliamps));
  }

  // Adjust the pilot levels to follow any changes in the incoming pilot
  if (abs(incomingPilotMilliamps - lastIncomingPilot) > PILOT_FUZZ) {
    switch(last_car_a_state) {
    case STATE_B:
    case STATE_C:
    case STATE_D:
      setPilot(CAR_A, isCarCharging(CAR_B)?HALF:FULL);
      break;
    }
    switch(last_car_b_state) {
    case STATE_B:
    case STATE_C:
    case STATE_D:
      setPilot(CAR_B, isCarCharging(CAR_A)?HALF:FULL);
      break;
    }
    lastIncomingPilot = incomingPilotMilliamps;
  }

  // Check the pilot sense on each car.
  unsigned int car_a_state = checkState(CAR_A);

  if (last_car_a_state == STATE_E) {
    switch(car_a_state) {
    case STATE_A:
      // we were in error, but the car's been disconnected.
      // If we still have a pilot or proximity error, then
      // we can't clear the error.
      if (!proximityOrPilotError) {
      // If not, clear the error state. The next time through
      // will take us back to state A.
        last_car_a_state = DUNNO;
        log(LOG_INFO, "Car A disconnected, clearing error");
      }
      // fall through...
    case STATE_B:
      // If we see a transition to state B, the error is still in effect, but complete (and
      // cancel) any pending relay opening.
      if (car_a_error_time != 0) {
        car_a_error_time = 0;
        setRelay(CAR_A, LOW);
        if (isCarCharging(CAR_B) || last_car_b_state == STATE_B)
          setPilot(CAR_B, FULL);
      }
      break;
    }
  } else if (car_a_state != last_car_a_state) {
    if (last_car_a_state != DUNNO)
      log(LOG_INFO, "Car A state transition: %s->%s.", state_str(last_car_a_state), state_str(car_a_state));
    last_car_a_state = car_a_state;
    switch(car_a_state) {
    case STATE_A:
    case STATE_B:
      // We're in an "off" state of one sort or other.
      // In either case, clear any connection delay timer,
      // make sure the relay is off, and set the diplay
      // appropriately. For state A, set our pilot high,
      // and fot state B, set it to half if the other car
      // is charging, full otherwise.
      setRelay(CAR_A, LOW);
      setPilot(CAR_A, car_a_state == STATE_A ? HIGH : (isCarCharging(CAR_B)?HALF:FULL));
      display.setCursor(0, 1);
      display.print(car_a_state == STATE_A ? "A: ---  " : "A: off  ");
      car_a_request_time = 0;
      if (isCarCharging(CAR_B) || last_car_b_state == STATE_B)
        setPilot(CAR_B, FULL);
      break;
    case STATE_C:
    case STATE_D: // We don't care about vent requirements.
      // We're in an "on" state.
      if (isCarCharging(CAR_A)) {
        // we're already charging. This might be a flip from C to D. Ignore it.
        break;
      }
      if (isCarCharging(CAR_B)) {
        // if car B is charging, we must wait while we transition them.
        car_a_request_time = now;
        // Drop car B down to 50%
        setPilot(CAR_B, HALF);
        display.setCursor(0, 1);
        display.print("A: wait ");
      } 
      else {
        // Car B is not charging. We can just turn our own power on immediately.
        // The only way they have a pilot now is if they're in state B, so if they are,
        // switch them to a half pilot.
        // If they turn on, they'll be forced to wait the transition period after reducing
        // our pilot, which will coincidently give them a taste of half power
        // before their relay gets turned on.
        setPilot(CAR_A, FULL);
        if (last_car_b_state == STATE_B)
          setPilot(CAR_B, HALF);
        setRelay(CAR_A, HIGH);
        car_a_request_time = 0;
      }
      break;
    case STATE_E:
      error(CAR_A, 'E');
      break;
    }
  }

  unsigned int car_b_state = checkState(CAR_B);
  if (last_car_b_state == STATE_E) {
    switch(car_b_state) {
    case STATE_A:
      // we were in error, but the car's been disconnected.
      // If we still have a pilot or proximity error, then
      // we can't clear the error.
      if (!proximityOrPilotError) {
      // If not, clear the error state. The next time through
      // will take us back to state A.
        last_car_b_state = DUNNO;
        log(LOG_INFO, "Car B disconnected, clearing error");
      }
      // fall through...
    case STATE_B:
      // If we see a transition to state B, the error is still in effect, but complete (and
      // cancel) any pending relay opening.
      if (car_b_error_time != 0) {
        car_b_error_time = 0;
        setRelay(CAR_B, LOW);
        if (isCarCharging(CAR_A) || last_car_a_state == STATE_B)
          setPilot(CAR_A, FULL);
      }
      break;
    }
  } else if (car_b_state != last_car_b_state) {
    if (last_car_b_state != DUNNO)
      log(LOG_INFO, "Car B state transition: %s->%s.", state_str(last_car_b_state), state_str(car_b_state));
    last_car_b_state = car_b_state;    
    switch(car_b_state) {
    case STATE_A:
    case STATE_B:
      // We're in an "off" state of one sort or other.
      // In either case, clear any connection delay timer,
      // make sure the relay is off, and set the diplay
      // appropriately. For state A, set our pilot high,
      // and fot state B, set it to half if the other car
      // is charging, full otherwise.
      setRelay(CAR_B, LOW);
      setPilot(CAR_B, car_b_state == STATE_A ? HIGH : (isCarCharging(CAR_A)?HALF:FULL));
      display.setCursor(8, 1);
      display.print(car_b_state == STATE_A ? "B: ---  " : "B: off  ");
      car_b_request_time = 0;
      if (isCarCharging(CAR_A) || last_car_a_state == STATE_B)
        setPilot(CAR_A, FULL);
      break;
    case STATE_C:
    case STATE_D:
      if (isCarCharging(CAR_B)) {
        // we're already charging. This might be a flip from C to D. Ignore it.
        break;
      }
      if (isCarCharging(CAR_A)) {
        // if car A is charging, we must transition them.
        car_b_request_time = now;
        // Drop car A down to 50%
        setPilot(CAR_A, HALF);
        display.setCursor(8, 1);
        display.print("B: wait ");
      } 
      else {
        setPilot(CAR_B, FULL);
        if (last_car_a_state == STATE_B)
          setPilot(CAR_A, HALF);
        setRelay(CAR_B, HIGH);
        car_b_request_time = 0;
      }
      break;
    case STATE_E:
      error(CAR_B, 'E');
      break;
    }
  }
  
  if (now - last_state_log > STATE_LOG_INTERVAL) {
    last_state_log = now;
    log(LOG_INFO, "States: Car A, %s; Car B, %s", state_str(last_car_a_state), state_str(last_car_b_state));
    log(LOG_INFO, "Incoming pilot %s", formatMilliamps(incomingPilotMilliamps));
  }
   
  // Update the ammeter display and check for overdraw conditions.
  // We allow a 5 second grace because the J1772 spec requires allowing
  // the car 5 seconds to respond to incoming pilot changes.
  // If the overdraw condition is acute enough, we'll be blowing fuses
  // in hardware, so this isn't as dire a condition as it sounds.
  // More likely what it means is that the car hasn't reacted to an
  // attempt to reduce it to half power (and the other car has not yet
  // been turned on), so we must error them out before letting the other
  // car start.
  if (isCarCharging(CAR_A) && car_a_request_time == 0) { // Only check the ammeter if the car is charging and NOT in transition delay
    unsigned long car_a_draw = readCurrent(CAR_A);

    if (now - last_current_log_car_a > CURRENT_LOG_INTERVAL) {
      last_current_log_car_a = now;
      log(LOG_INFO, "Car A current draw %lu mA", car_a_draw);
    }
    
    // If the other car is charging, then we can only have half power
    unsigned long car_a_limit = incomingPilotMilliamps / (isCarCharging(CAR_B) ? 2 : 1);

    if (car_a_draw > car_a_limit + OVERDRAW_GRACE_AMPS) {
      // car A has begun an over-draw condition. They have 5 seconds of grace before we pull the plug.
      if (car_a_overdraw_begin == 0) {
        car_a_overdraw_begin = now;
      } 
      else {
        if (now - car_a_overdraw_begin > OVERDRAW_GRACE_PERIOD) {
          error(CAR_A, 'O');
          return;
        }
      }
    }
    else {
      // car A is under its limit. Cancel any overdraw in progress
      car_a_overdraw_begin = 0;
    }
    display.setCursor(0, 1);
    display.print("A:");
    display.print(formatMilliamps(car_a_draw));
  } 
  else {
    // Car A is not charging
    car_a_overdraw_begin = 0;
    memset(car_b_current_samples, 0, sizeof(car_b_current_samples));
  }

  if (isCarCharging(CAR_B) && car_b_request_time == 0) { // Only check the ammeter if the car is charging and NOT in transition delay
    unsigned long car_b_draw = readCurrent(CAR_B);

    if (now - last_current_log_car_b > CURRENT_LOG_INTERVAL) {
      last_current_log_car_b = now;
      log(LOG_INFO, "Car B current draw %lu mA", car_b_draw);
    }
    
    // If the other car is charging, then we can only have half power
    unsigned long car_b_limit = incomingPilotMilliamps / (isCarCharging(CAR_A) ? 2 : 1);

    if (car_b_draw > car_b_limit + OVERDRAW_GRACE_AMPS) {
      // car B has begun an over-draw condition. They have 5 seconds of grace before we pull the plug.
      if (car_b_overdraw_begin == 0) {
        car_b_overdraw_begin = now;
      } 
      else {
        if (now - car_b_overdraw_begin > OVERDRAW_GRACE_PERIOD) {
          error(CAR_B, 'O');
          return;
        }
      }
    }
    else {
      car_b_overdraw_begin = 0;
    }
    display.setCursor(8, 1);
    display.print("B:");
    display.print(formatMilliamps(car_b_draw));
  } 
  else {
    // Car B is not charging
    car_b_overdraw_begin = 0;
    memset(car_b_current_samples, 0, sizeof(car_b_current_samples));
  }

  if (car_a_request_time != 0 && now - car_a_request_time > TRANSITION_DELAY) {
    // We've waited long enough.
    car_a_request_time = 0;
    setRelay(CAR_A, HIGH);
    display.setCursor(0, 1);
    display.print("A: ON   ");
  }
  if (car_a_error_time != 0 && now - car_a_error_time > ERROR_DELAY) {
    car_a_error_time = 0;
    setRelay(CAR_A, LOW);
    if (isCarCharging(CAR_B) || last_car_b_state == STATE_B)
        setPilot(CAR_B, FULL);
  }
  if (car_b_request_time != 0 && now - car_b_request_time > TRANSITION_DELAY) {
    // We've waited long enough.
    car_b_request_time = 0;
    setRelay(CAR_B, HIGH);
    display.setCursor(8, 1);
    display.print("B: ON   ");
  }
  if (car_b_error_time != 0 && now - car_b_error_time > ERROR_DELAY) {
    car_b_error_time = 0;
    setRelay(CAR_B, LOW);
    if (isCarCharging(CAR_A) || last_car_a_state == STATE_B)
        setPilot(CAR_A, FULL);
  }
}



