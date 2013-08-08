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

#define CAR_A_PILOT_OUT_PIN     9 // output for toggling pilot on car A
#define CAR_B_PILOT_OUT_PIN     10	// output for toggling pilot on car B

#define CAR_A_RELAY             7
#define CAR_B_RELAY             8

// ---------- ANALOG PINS ----------
// Note that in hardware version 0.6
//#define CAR_A_PILOT_SENSE       0
//#define CAR_B_PILOT_SENSE       1
//#define CAR_A_CURRENT_PIN       2
//#define CAR_B_CURRENT_PIN       3
#define CAR_A_PILOT_SENSE_PIN	0	// the pilot sense for car A
#define CAR_A_CURRENT_PIN	1	// the current transformer ammeter for car A
#define CAR_B_PILOT_SENSE_PIN	2	// the pilot sense for car B
#define CAR_B_CURRENT_PIN	3	// the current transformer ammeter for car B

// for things like erroring out a car
#define BOTH                    0
#define CAR_A			1
#define CAR_B			2

// Don't use 0 or 1 because that's the value of LOW and HIGH.
#define HALF              3
#define FULL              4

#define STATE_A		1
#define STATE_B		2
#define STATE_C		3
#define STATE_D         4
#define STATE_E		5
#define DUNNO		0 // During the low portion of the pilot, we must get -12v in any state

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
#define PILOT_DIODE_MAX  244

// This is how long we allow a car to draw more current than it is allowed before we
// error it out (in milliseconds).
#define OVERDRAW_GRACE_PERIOD 2000

// This is how much "slop" we allow a car to have in terms of keeping under its current allowance.
// That is, this value (in milliamps) plus the calculated current limit is what we enforce.
#define OVERDRAW_GRACE_AMPS 2500

// The time between withdrawing the pilot from a car and disconnecting its relay (in milliseconds).
#define ERROR_DELAY 250

// When a car requests state C while the other car is already in state C, we delay them for
// this long while the other car transitions to half power. THIS INTERVAL MUST BE LONGER
// THAN THE OVERDRAW_GRACE_PERIOD! (in milliseconds)
#define TRANSITION_DELAY 3000

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

// This is the number of duty cycle or ammeter samples we keep to make a rolling average to stabilize
// the display. The balance here is between stability and responsiveness,
#define ROLLING_AVERAGE_SIZE 0

// The number of milliseconds to sample an ammeter pin in order to find the two AC peaks.
// one cycle at 50 Hz is 20 ms.
#define CURRENT_SAMPLE_INTERVAL 21

// This multiplier is the number of milliamps RMS per A/d converter unit, peak-to-peak

// To calculate this multiplier, you need to know the ratio between the RMS current and P-P volts across
// the burden resistor. You need to pick the burden resistor so that the maximum outlet RMS current is slightly
// below 5 volts p-p (2.5 volts peak amplitude, or 1.77 volts RMS).

// The CT has a specification called Te, or "Turns Equivalent." In short, this is the ratio of the current
// through the measured conductor and the current that you see output on the two leads of the CT. You place a
// burden resistor across the CT to turn the measured current of the CT into a measured voltage, which the
// controller's A/d converters can sample. Since we're measuring an AC current, we will want to get a peak-to-peak
// measurement, so the way to do that is to offset the measured voltage by half of our 5 volt measuring range.
// That means that we want to select our burden resistor value so that 5 volts is slightly higher than the
// expected voltage for your MAXIMUM_OUTLET_CURRENT. To figure this out, take your maximum current, call it A,
// and convert it to a peak-to-peak value by multiplying it by 2*sqrt(2), then divide that by the Te value of
// your coil. Divide 5 by that value and then take the next lower standard resistor value. That will be Rb.
// The standard CT in the parts list for the Hydra has a Te of 1018. For a 30A value of MAXIMUM_OUTLET_CURRENT,
// That means Rb is 56.

// Having done that, you calculate milliamps-per-volt-P-t-P with 1000 / ((sqrt(2) * 2 / Te) * Rb), where Te
// is the "turns equivalent" of the CT and Rb is the value of the burden resistor. Once you know that,
// the scale factor is mApV * (5/1024)

// Te = 1018, MAX = 30, Rb=56
#define CURRENT_SCALE_FACTOR 31

// Te = 1018, MAX = 50, Rb=33
// #define CURRENT_SCALE_FACTOR=53

// Te = 1983, MAX = 75, Rb=43
// #define CURRENT_SCALE_FACTOR=80

#define VERSION "0.8.2 beta"

LiquidTWI2 display(LCD_I2C_ADDR, 1);

unsigned long rollingIncomingAverageMA[ROLLING_AVERAGE_SIZE];
unsigned long car_a_current_samples[ROLLING_AVERAGE_SIZE], car_b_current_samples[ROLLING_AVERAGE_SIZE];
unsigned long incomingPilotMilliamps;
unsigned int last_car_a_state, last_car_b_state;
unsigned long car_a_overdraw_begin, car_b_overdraw_begin, car_a_request_time, car_b_request_time;
unsigned int relay_state_a, relay_state_b;

// Deal in milliamps so that we don't have to use floating point.
// Convert the microsecond state timings from the incoming pilot into
// an instantaneous current allowance value. With polling, it's
// the two sample counts, but the math winds up being exactly the same.
unsigned long timeToMA(unsigned long msecHigh, unsigned long msecLow) {
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
unsigned long MAToMsec(unsigned long milliamps) {
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
unsigned int MAtoPwm(unsigned long milliamps) {
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
  display.setBacklight(RED);

  // Set the pilot to constant 12: indicates an EVSE error.
  // We can't use -12, because then we'd never detect a return
  // to state A.

  if (car == BOTH || car == CAR_A) {
    setPilot(CAR_A, HIGH);
  }
  if (car == BOTH || car == CAR_B) {
    setPilot(CAR_B, HIGH);
  }
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

  // We should let the car notice the pilot change and stop drawing
  // before we turn off the relay(s).
  delay(ERROR_DELAY);

  if (car == BOTH || car == CAR_A) {
    setRelay(CAR_A, LOW);
    last_car_a_state = STATE_E;
    car_a_request_time = 0;
  }
  if (car == BOTH || car == CAR_B) {
    setRelay(CAR_B, LOW);
    last_car_b_state = STATE_E;
    car_b_request_time = 0;
  }
}

void setRelay(unsigned int car, unsigned int state) {
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

inline boolean isCarReallyCharging(unsigned int car) {
  return (car == CAR_A)?relay_state_a:relay_state_b;
}

// If the car has requested charging and is in the transition delay, or if its
// relay is actually on, then it's charging.
inline boolean isCarCharging(unsigned int car) {
  switch(car) {
    case CAR_A:
      if (car_a_request_time != 0) return HIGH;
      return relay_state_a;
    break;
    case CAR_B:
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
  } else if (high >= STATE_B_MIN && high <= STATE_B_MAX) {
    return STATE_B;
  } else if (high >= STATE_C_MIN && high <= STATE_C_MAX) {
    return STATE_C;
  } else if (high >= STATE_D_MIN && high <= STATE_D_MAX) {
    return STATE_D;
  }
  // I dunno how we got here, but we fail it.
  return STATE_E;
}

unsigned long readCurrent(unsigned int car) {
  unsigned int positive_peak = 0, negative_peak=9999;
  unsigned int car_pin = (car == CAR_A) ? CAR_A_CURRENT_PIN : CAR_B_CURRENT_PIN;
  for(unsigned long now = millis(); millis() - now < CURRENT_SAMPLE_INTERVAL; ) {
    unsigned int sample = analogRead(car_pin);
    if (sample > positive_peak) positive_peak = sample;
    if (sample < negative_peak) negative_peak = sample;
  }
  unsigned long delta = positive_peak - negative_peak;

  return rollRollingAverage((car == CAR_A) ? car_a_current_samples : car_b_current_samples, delta * CURRENT_SCALE_FACTOR);
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

  milliamps = rollRollingAverage(rollingIncomingAverageMA, milliamps);
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

  pinMode(INCOMING_PILOT_PIN, INPUT);
  pinMode(INCOMING_PROXIMITY_PIN, INPUT);
  pinMode(CAR_A_PILOT_OUT_PIN, OUTPUT);
  pinMode(CAR_B_PILOT_OUT_PIN, OUTPUT);
  pinMode(CAR_A_RELAY, OUTPUT);
  pinMode(CAR_B_RELAY, OUTPUT);

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

  display.setBacklight(WHITE);
  display.clear();
  display.setCursor(0, 0);
  display.print("J1772 Hydra");
  display.setCursor(0, 1);
  display.print(VERSION);

  boolean success = SetPinFrequency(CAR_A_PILOT_OUT_PIN, 1000);
  if (!success) display.setBacklight(YELLOW);
  success = SetPinFrequency(CAR_B_PILOT_OUT_PIN, 1000);
  if (!success) display.setBacklight(BLUE);
  // In principle, neither of the above two !success conditions should ever
  // happen. But if they do, a discolored splash screen is your clue.

  // meanwhile, the fill in the incoming pilot rolling average...
  for(int i = 0; i < ROLLING_AVERAGE_SIZE; i++) {
    pollIncomingPilot();
  }
  delay(2500); // let the splash screen show
  display.clear();
}

void loop() {

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
  if (digitalRead(INCOMING_PROXIMITY_PIN) != HIGH) {
    display.setCursor(0, 0);
    display.print("DISCONNECTING...");
    error(BOTH, 'P');
    return; // We're about to bail... don't bother with anything else.
  }
  
  pollIncomingPilot();
  display.setCursor(0, 0);
  if (incomingPilotMilliamps < MINIMUM_INLET_CURRENT) {
    display.print("INPUT PILOT ERR ");
    error(BOTH, 'I');
    // Forget it. Nothing else is worth doing as long as the input pilot continues to be gone.
    return;
  }

  
  display.print("EVSE power ");
  display.print(formatMilliamps(incomingPilotMilliamps));

  // Adjust the pilot levels to follow any changes in the incoming pilot
  switch(last_car_a_state) {
    case STATE_A:
    case STATE_E:
      setPilot(CAR_A, HIGH);
      break;
    case STATE_B:
    case STATE_C:
    case STATE_D:
      setPilot(CAR_A, isCarCharging(CAR_B)?HALF:FULL);
      break;
  }
  switch(last_car_b_state) {
    case STATE_A:
    case STATE_E:
      setPilot(CAR_B, HIGH);
      break;
    case STATE_B:
    case STATE_C:
    case STATE_D:
      setPilot(CAR_B, isCarCharging(CAR_A)?HALF:FULL);
      break;
  }

  // Check the pilot sense on each car.
  unsigned int car_a_state = checkState(CAR_A);
  if (last_car_a_state == STATE_E && car_a_state == STATE_A) {
    // we were in error, but the car's been disconnected.
    // Finesse that by clearing the error state. The following logic
    // will take us back to state A.
    last_car_a_state = DUNNO;
  }

  // If we're in error state, no transitions out are allowed (except to state A, handled above).
  // If our current state is "dunno," then it means we passed the diode check, and that we
  // have no information presently to help us transition, so skip it.
  if (last_car_a_state != STATE_E && car_a_state != last_car_a_state && car_a_state != DUNNO) {
    last_car_a_state = car_a_state;
    switch(car_a_state) {
    case STATE_A:
      // The car is unplugged. Set the pilot to +12v and if Car B isn't disconnected, give it full power
      setRelay(CAR_A, LOW);
      setPilot(CAR_A, HIGH);
      display.setCursor(0, 1);
      display.print("A: ---  ");
      car_a_request_time = 0;
      if (last_car_b_state != STATE_A)
        setPilot(CAR_B, FULL);
      break;
    case STATE_B:
      // The car is plugged in, or is returning to B from C.
      setRelay(CAR_A, LOW);
      display.setCursor(0, 1);
      display.print("A: off  ");
      car_a_request_time = 0;
      // If car B is charging, we get 50% and they get 100%. Else, everybody gets 100%
      setPilot(CAR_A, isCarCharging(CAR_B)?HALF:FULL);
      setPilot(CAR_B, FULL);
      break;
    case STATE_C:
    case STATE_D: // We don't care about vent requirements.
      if (isCarCharging(CAR_A)) {
        // we're already charging. This might be a flip from C to D. Ignore it.
        break;
      }
      if (isCarCharging(CAR_B)) {
        // if car B is charging, we must transition them.
        car_a_request_time = millis();
        // Drop car B down to 50%
        setPilot(CAR_B, HALF);
        display.setCursor(0, 1);
        display.print("A: wait ");
      } 
      else {
        // Car B is not charging. We can just turn our own power on and
        // reduce them to half power immediately. If they turn on,
        // they'll be forced to wait the transition period after reducing
        // our pilot, which will coincidently give them a taste of half power
        // before their relay gets turned on.
        setPilot(CAR_A, FULL);
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
  if (last_car_b_state == STATE_E && car_b_state == STATE_A) {
    // we were in error, but the car's been disconnected.
    // Finesse that by clearing the error state. The following logic
    // will take us back to state A.
    last_car_b_state = DUNNO;
  }
  if (last_car_b_state != STATE_E && car_b_state != last_car_b_state && car_b_state != DUNNO) {
    last_car_b_state = car_b_state;    
    switch(car_b_state) {
    case STATE_A:
      // The car is unplugged. Set the pilot to +12v and if Car B isn't disconnected, give it full power
      setRelay(CAR_B, LOW);
      setPilot(CAR_B, HIGH);
      display.setCursor(8, 1);
      display.print("B: ---  ");
      car_b_request_time = 0;
      if (last_car_a_state != STATE_A)
        setPilot(CAR_A, FULL);
      break;
    case STATE_B:
      // The car is plugged in, or is returning to B from C.
      setRelay(CAR_B, LOW);
      display.setCursor(8, 1);
      display.print("B: off  ");
      car_a_request_time = 0;
      // If car A is charging, we get 50% and they get 100%. Else, everybody gets 100%
      setPilot(CAR_B, isCarCharging(CAR_B)?HALF:FULL);
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
        car_b_request_time = millis();
        // Drop car A down to 50%
        setPilot(CAR_A, HALF);
        display.setCursor(8, 1);
        display.print("B: wait ");
      } 
      else {
        setPilot(CAR_B, FULL);
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

  // Update the ammeter display and check for overdraw conditions.
  // We allow a 5 second grace because the J1772 spec requires allowing
  // the car 5 seconds to respond to incoming pilot changes.
  // If the overdraw condition is acute enough, we'll be blowing fuses
  // in hardware, so this isn't as dire a condition as it sounds.
  // More likely what it means is that the car hasn't reacted to an
  // attempt to reduce it to half power (and the other car has not yet
  // been turned on), so we must error them out before letting the other
  // car start.
  if (isCarReallyCharging(CAR_A)) {
    int car_a_draw = readCurrent(CAR_A);

    // If the other car is charging, then we can only have half power
    int car_a_limit = incomingPilotMilliamps / (isCarCharging(CAR_B) ? 2 : 1);

    if (car_a_draw > car_a_limit + OVERDRAW_GRACE_AMPS) {
      // car A has begun an over-draw condition. They have 5 seconds of grace before we pull the plug.
      if (car_a_overdraw_begin == 0) {
        car_a_overdraw_begin = millis();
      } 
      else {
        if (millis() - car_a_overdraw_begin > OVERDRAW_GRACE_PERIOD) {
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

  if (isCarReallyCharging(CAR_B)) {
    int car_b_draw = readCurrent(CAR_B);

    // If the other car is charging, then we can only have half power
    int car_b_limit = incomingPilotMilliamps / (isCarCharging(CAR_A) ? 2 : 1);

    if (car_b_draw > car_b_limit + OVERDRAW_GRACE_AMPS) {
      // car B has begun an over-draw condition. They have 5 seconds of grace before we pull the plug.
      if (car_b_overdraw_begin == 0) {
        car_b_overdraw_begin = millis();
      } 
      else {
        if (millis() - car_b_overdraw_begin > OVERDRAW_GRACE_PERIOD) {
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

  if (car_a_request_time != 0 && millis() - car_a_request_time > TRANSITION_DELAY) {
    // We've waited long enough.
    car_a_request_time = 0;
    setRelay(CAR_A, HIGH);
    display.setCursor(0, 1);
    display.print("A: ON   ");
  }
  if (car_b_request_time != 0 && millis() - car_b_request_time > TRANSITION_DELAY) {
    // We've waited long enough.
    car_b_request_time = 0;
    setRelay(CAR_B, HIGH);
    display.setCursor(8, 1);
    display.print("B: ON   ");
  }
}



