
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
#define INCOMING_PILOT_PIN    2
#define INCOMING_PILOT_INT      0       // Watch out. Pin 2 and interrupt 0 are tied together...

#define CAR_A_PILOT_PIN		10	// output for toggling pilot on car A
#define CAR_B_PILOT_PIN		9	// output for toggling pilot on car B

#define CAR_A_RELAY            7
#define CAR_B_RELAY            8

// ---------- ANALOG PINS ----------
#define CAR_A_CURRENT_PIN		0	// the current transformer ammeter for car A
#define CAR_A_PILOT_SENSE	1	// the pilot sense for car A
#define CAR_B_CURRENT_PIN		2	// the current transformer ammeter for car B
#define CAR_B_PILOT_SENSE	3	// the pilot sense for car B

// for things like erroring out a car
#define CAR_A			1
#define CAR_B			2

// Don't use 0 or 1 because that's the value of LOW and HIGH.
#define HALF              3
#define FULL              4

#define STATE_A		1
#define STATE_B		2
#define STATE_C		3
#define STATE_E		4
#define DUNNO		0 // During the low portion of the pilot, we must get -12v in any state

// The lost pilot grace period is in microseconds
// This is how long we will tolerate losing the incoming pilot transitions
// before declaring an input pilot error
#define LOST_PILOT_GRACE_PERIOD (5000 * 1000)

// This is how long we allow a car to draw more current than it is allowed before we
// error it out (in milliseconds).
#define OVERDRAW_GRACE_PERIOD 5000

// When a car requests state C while the other car is already in state C, we delay them for
// this long while the other car transitions to half power. THIS INTERVAL MUST BE LONGER
// THAN THE OVERDRAW_GRACE_PERIOD! (in milliseconds)
#define TRANSITION_DELAY 10000

// This is the current limit of all of the components on the inlet side of the hydra -
// the inlet itself, any fuses, and the wiring to the common sides of the relays.
#define MAXIMUM_INLET_CURRENT 75

// This is the minimum of the ampacity of all of the components from the relay to the plug -
// The relay itself, the J1772 cable and plug.
#define MAXIMUM_OUTLET_CURRENT 30

// This can not be lower than 12, because the J1772 spec bottoms out at 6A.
// The hydra won't operate properly if it can't divide the incoming power in half. (in amps)
#define MINIMUM_INLET_CURRENT 12

// This is the number of duty cycle samples we keep to make a rolling average to stabilize
// our input current capacity. The balance here is between stability and responsiveness,
// but at 1 kHz, 100 samples goes by in 1/10th of a second, which is plenty responsive.
#define ROLLING_AVERAGE_SIZE 100

// The number of times to sample an ammeter pin in order to find the two AC peaks.
#define CURRENT_SAMPLE_COUNT 32
// The number of microseconds to wait between adjacent samples. analogRead is said
// to take around 100 microseconds. So if we wait 400, we should get two samples per millisecond,
// resulting in 32 samples lasting 16 milliseconds, which is one full cycle of 60 Hz.
#define CURRENT_SAMPLE_DELAY 400
// Once we have a peak-to-peak reading from the current transformer, scale that to a number of milliamps.
// We assume that we're going to set up 30A current transformers with a burden resistor and voltage divider
// that will yield a full 5 volt swing at full power, meaning a peak-to-peak reading of 1023. 1023 * 29 is
// just under 30. Close enough.
#define CURRENT_SCALE_FACTOR 29

#define VERSION "0.2 alpha"

LiquidTWI2 display(LCD_I2C_ADDR, 1);

int high_micros;
unsigned int rollingIncomingAverageMA[ROLLING_AVERAGE_SIZE];
unsigned int car_a_current_samples[ROLLING_AVERAGE_SIZE], car_b_current_samples[ROLLING_AVERAGE_SIZE];
unsigned int incomingPilotMilliamps;
int last_car_a_state=DUNNO, last_car_b_state=DUNNO;
unsigned long car_a_overdraw_begin, car_b_overdraw_begin, car_a_request_time, car_b_request_time, lastPilotChange;
int relay_state_a, relay_state_b;

// Deal in milliamps so that we don't have to use floating point.
// Convert the microsecond state timings from the incoming pilot into
// an instantaneous current allowance value.
int timeToMA(int msecHigh, int msecLow) {
  // First, normalize the frequency to 1000 Hz.
  // msec is the number of microseconds out of 1000 that the signal stayed high
  long msec = ((long)(msecHigh) * 1000) / (msecHigh + msecLow);
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
unsigned int MAToMsec(unsigned int milliamps) {
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
unsigned int MAtoPwm(unsigned int milliamps) {
  long out = MAToMsec(milliamps);

  if (out >= 1000) return 255; // full on

  out =  (out * 256) / 1000;  

  return (int)out;
}

// Turn a millamp value into nn.n as amps, with the tenth rounded near.
char *formatMilliamps(int milliamps) {
  static char out[6];

  int hundredths = (milliamps / 10) % 100;
  int tenths = hundredths / 10 + (((hundredths % 10) >= 5)?1:0);
  int units = milliamps / 1000;
  if (tenths >= 10) {
    tenths -= 10;
    units++;
  }

  sprintf(out, "%2d.%01dA", units, tenths);

  return out;

}

void error(int car) {
  display.setBacklight(RED);
  if (car==CAR_A) {
    setRelay(CAR_A, LOW); // make sure the power is off
    setPilot(CAR_A, HIGH);
    display.setCursor(0, 1);
    display.print("A: ERR  ");
    car_a_request_time = 0; // cancel any transition delay in progress
    last_car_a_state = STATE_E;
  } 
  else {
    setRelay(CAR_B, LOW);
    setPilot(CAR_B, HIGH);
    display.setCursor(9, 1);
    display.print("B: ERR  ");
    car_b_request_time = 0;
    last_car_b_state = STATE_E;
  }
}

void setRelay(int car, int state) {
  digitalWrite(car==CAR_A?CAR_A_RELAY:CAR_B_RELAY, state);
  if (car==CAR_A) relay_state_a = state;
  else
    relay_state_b = state;
}

// If the car has requested charging and is in the transition delay, or if its
// relay is actually on, then it's charging.
boolean isCarCharging(int car) {
  if (car == CAR_A) {
    if (car_a_request_time != 0) return HIGH;
    return relay_state_a;
  } 
  else {
    if (car_b_request_time != 0) return HIGH;
    return relay_state_b;
  }
}

// Set the pilot for the car as appropriate. 'which' is either HALF, FULL or HIGH.
// HIGH sets a constant +12v, which is the spec for state A, but we also use it for
// state E. HALF means that the other car is charging, so we only can have half power.

void setPilot(int car, int which) {
  // set the outgoing pilot for the given car to either HALF state, FULL state, or HIGH.
  int pin = car==CAR_A?CAR_A_PILOT_PIN:CAR_B_PILOT_PIN;
  if (which == HIGH) {
    pwmWrite(pin, 255);
  } 
  else {
    unsigned int ma = incomingPilotMilliamps / (which==HALF?2:1);
    if (ma > MAXIMUM_OUTLET_CURRENT * 1000) ma = MAXIMUM_OUTLET_CURRENT * 1000;
    pwmWrite(pin, MAtoPwm(ma));
  }
}

int checkState(int car) {
  // Check the pilot state pin for the given car. If the voltage is > 0, then return which state the car is indicating.
  // If the voltage is < 0, then return either DUNNO (diode check pass) or STATE_E if the diode check fails.
  return car == CAR_B?STATE_C:STATE_B; // XXX FIXME
}

unsigned int readCurrent(int car) {
  int positive_peak = 0, negative_peak=9999;
  for(int i = 0; i < CURRENT_SAMPLE_COUNT; i++) {
    int sample = analogRead(car==CAR_A?CAR_A_CURRENT_PIN:CAR_B_CURRENT_PIN);
    if (sample > positive_peak) positive_peak = sample;
    if (sample < negative_peak) negative_peak = sample;
    delayMicroseconds(CURRENT_SAMPLE_DELAY);
  }
  int delta = positive_peak - negative_peak;
  
  return rollRollingAverage(car==CAR_A?car_a_current_samples:car_b_current_samples, delta * CURRENT_SCALE_FACTOR);
}

unsigned int rollRollingAverage(unsigned int *array, unsigned int new_value) {
  unsigned long sum = new_value;
  for(int i = ROLLING_AVERAGE_SIZE - 1; i >= 1; i--) {
    array[i] = array[i - 1];
    sum += array[i];
  }
  array[0] = new_value;
  return (unsigned int)(sum / ROLLING_AVERAGE_SIZE);
}

void reportIncomingPilot(unsigned int milliamps) {
  incomingPilotMilliamps = rollRollingAverage(rollingIncomingAverageMA, milliamps);
  // Clamp to the maximum allowable current
  if (incomingPilotMilliamps > MAXIMUM_INLET_CURRENT * 1000) incomingPilotMilliamps = MAXIMUM_INLET_CURRENT * 1000;
}

void handlePilotChange() {
  unsigned long now = micros();
  int delta = (int)(now - lastPilotChange);
  lastPilotChange = now;

  if (delta > 1000) {
    // More than 1000 microseconds have elapsed since the last incoming
    // pilot transition. This should never happen if the incoming pilot
    // frequency is 1 kHz (one cycle is 1000 uSec long).
    // All we can do is wait for it to stabilize and hope for the best.
    // In the main loop, we will check for lastPilotChange not happening regularly and
    // throw a fit.
    return;
  }

  int pilotState = digitalRead(INCOMING_PILOT_PIN);
  if (pilotState == HIGH) {
    unsigned int milliamps = timeToMA(high_micros, delta);
    reportIncomingPilot(milliamps);
  } 
  else {
    high_micros = delta;
  }
}

void setup() {
  InitTimersSafe();
  display.setMCPType(LTI_TYPE_MCP23017);
  display.begin(16, 2); 

  pinMode(INCOMING_PILOT_PIN, INPUT);
  pinMode(CAR_A_PILOT_PIN, OUTPUT);
  pinMode(CAR_B_PILOT_PIN, OUTPUT);
  pinMode(CAR_A_RELAY, OUTPUT);
  pinMode(CAR_B_RELAY, OUTPUT);
  digitalWrite(CAR_A_PILOT_PIN, HIGH);
  digitalWrite(CAR_B_PILOT_PIN, HIGH);
  digitalWrite(CAR_A_RELAY, LOW);
  digitalWrite(CAR_B_RELAY, LOW);

  // Enter state A on both cars
  setPilot(CAR_A, HIGH);
  setPilot(CAR_B, HIGH);
  // And make sure the power is off.
  setRelay(CAR_A, LOW);
  setRelay(CAR_B, LOW);

  memset(car_a_current_samples, 0, sizeof(car_a_current_samples));
  memset(car_b_current_samples, 0, sizeof(car_b_current_samples));
    
  attachInterrupt(INCOMING_PILOT_INT, handlePilotChange, CHANGE);

  display.setBacklight(WHITE);
  display.clear();
  display.setCursor(0, 0);
  display.print("J1772 Hydra");
  display.setCursor(0, 1);
  display.print(VERSION);

  boolean success = SetPinFrequency(CAR_A_PILOT_PIN, 1000);
  if (!success) display.setBacklight(YELLOW);
  success = SetPinFrequency(CAR_B_PILOT_PIN, 1000);
  if (!success) display.setBacklight(BLUE);
  // In principle, neither of the above two !success conditions should ever
  // happen. But if they do, a discolored splash screen is your clue.

  delay(1000); // meanwhile, the hope is that the incoming pilot rolling average will fill in...
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

  display.setCursor(0, 0);
  char buf[17];
  if (incomingPilotMilliamps < MINIMUM_INLET_CURRENT * 1000 || (micros() - lastPilotChange) > LOST_PILOT_GRACE_PERIOD) {
    display.print("INPUT PILOT ERR ");
    error(CAR_A);
    error(CAR_B);
    return;
  }
  display.print("Input Pwr ");
  display.print(formatMilliamps(incomingPilotMilliamps));

  // Adjust the pilot levels to follow any changes in the incoming pilot
  if (last_car_a_state != STATE_A && last_car_a_state != STATE_E) {
    setPilot(CAR_A, isCarCharging(CAR_B)?HALF:FULL);
  }
  if (last_car_b_state != STATE_A && last_car_b_state != STATE_E) {
    setPilot(CAR_B, isCarCharging(CAR_A)?HALF:FULL);
  }

  // Check the pilot sense on each car.
  int car_a_state = checkState(CAR_A);
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
      if (isCarCharging(CAR_B)) {
        // if car B is charging, we must transition them.
        car_a_request_time = millis();
        // Drop car B down to 50%
        setPilot(CAR_B, HALF);
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
      error(CAR_A);
      break;
    }
  }

  int car_b_state = checkState(CAR_B);
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
      display.setCursor(9, 1);
      display.print("B: ---  ");
      car_b_request_time = 0;
      if (last_car_a_state != STATE_A)
        setPilot(CAR_A, FULL);
      break;
    case STATE_B:
      // The car is plugged in, or is returning to B from C.
      setRelay(CAR_B, LOW);
      display.setCursor(9, 1);
      display.print("B: off  ");
      car_a_request_time = 0;
      // If car A is charging, we get 50% and they get 100%. Else, everybody gets 100%
      setPilot(CAR_B, isCarCharging(CAR_B)?HALF:FULL);
      setPilot(CAR_A, FULL);
      break;
    case STATE_C:
      if (isCarCharging(CAR_A)) {
        // if car A is charging, we must transition them.
        car_b_request_time = millis();
        // Drop car A down to 50%
        setPilot(CAR_A, HALF);
      } 
      else {
        setPilot(CAR_B, FULL);
        setPilot(CAR_A, HALF);
        setRelay(CAR_B, HIGH);
        car_b_request_time = 0;
      }
      break;
    case STATE_E:
      error(CAR_B);
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
  if (isCarCharging(CAR_A)) {
    int car_a_draw = readCurrent(CAR_A);

    // If the other car is charging, then we can only have half power
    int car_a_limit = incomingPilotMilliamps / (isCarCharging(CAR_B)?2:1);

    if (car_a_draw > car_a_limit) {
      // car A has begun an over-draw condition. They have 5 seconds of grace before we pull the plug.
      if (car_a_overdraw_begin == 0) {
        car_a_overdraw_begin = millis();
      } 
      else {
        if (car_a_overdraw_begin < millis() - OVERDRAW_GRACE_PERIOD) {
          error(CAR_A);
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
    car_a_overdraw_begin = 0;
    memset(car_b_current_samples, 0, sizeof(car_b_current_samples));
  }

  if (isCarCharging(CAR_B)) {
    int car_b_draw = readCurrent(CAR_B);

    // If the other car is charging, then we can only have half power
    int car_b_limit = incomingPilotMilliamps / (isCarCharging(CAR_A)?2:1);

    if (car_b_draw > car_b_limit) {
      // car B has begun an over-draw condition. They have 5 seconds of grace before we pull the plug.
      if (car_b_overdraw_begin == 0) {
        car_b_overdraw_begin = millis();
      } 
      else {
        if (car_b_overdraw_begin < millis() - OVERDRAW_GRACE_PERIOD) {
          error(CAR_B);
          return;
        }
      }
    }
    else {
      car_b_overdraw_begin = 0;
    }
    display.setCursor(9, 1);
    display.print("B:");
    display.print(formatMilliamps(car_b_draw));
  } 
  else {
    car_b_overdraw_begin = 0;
    memset(car_b_current_samples, 0, sizeof(car_b_current_samples));
  }

  if (car_a_request_time != 0 && car_a_request_time < millis() - TRANSITION_DELAY) {
    // We've waited long enough.
    car_a_request_time = 0;
    setRelay(CAR_A, HIGH);
    display.setCursor(0, 1);
    display.print("A: ON   ");
  }
  if (car_b_request_time != 0 && car_b_request_time < millis() - TRANSITION_DELAY) {
    // We've waited long enough.
    car_b_request_time = 0;
    setRelay(CAR_B, HIGH);
    display.setCursor(9, 1);
    display.print("B: ON   ");
  }
}



