
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
#define CAR_A_AMPS_PIN		0	// the current transformer ammeter for car A
#define CAR_A_PILOT_SENSE	1	// the pilot sense for car A
#define CAR_B_AMPS_PIN		2	// the current transformer ammeter for car B
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

#define OVERDRAW_GRACE_PERIOD 5000
// The lost pilot grace period is in microseconds
#define LOST_PILOT_GRACE_PERIOD (1000 * 1000)
#define TRANSITION_DELAY 10000

// Set this to the lowest ampacity of any component in the HV path of the hydra.
#define MAXIMUM_CURRENT 30
// This can not be lower than 12, because the J1772 spec bottoms out at 6A.
// The hydra won't operate properly if it can't divide the incoming power in half.
#define MINIMUM_CURRENT 12

#define ROLLING_AVERAGE_SIZE 100

#define VERSION "0.1 alpha"


LiquidTWI2 m_Lcd(LCD_I2C_ADDR, 1);

// Deal in milliamps so that we don't have to use floating point.
int timeToMA(int msecHigh, int msecLow) {
  // First, normalize the frequency to 1000 Hz.
  // msec is the number of msec out of 1000 that the signal stayed high
  long msec = ((long)(msecHigh) * 1000) / (msecHigh + msecLow);
  if (msec < 100) {
    return 0;
  } 
  else if (msec < 850) {
    return (int)((msec) * 60);
  } 
  else if (msec <= 960) {
    return (int)((msec - 640) * 250);
  } 
  else {
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

int high_micros;
unsigned int rollingIncomingAverageMA[ROLLING_AVERAGE_SIZE];
unsigned int incomingPilotMilliamps;
int last_car_a_state=DUNNO, last_car_b_state=DUNNO;
unsigned long car_a_overdraw_begin, car_b_overdraw_begin, car_a_request_time, car_b_request_time, lastPilotChange;

void setup() {
  InitTimersSafe();
  m_Lcd.setMCPType(LTI_TYPE_MCP23017);
  m_Lcd.begin(16, 2); 

  pinMode(INCOMING_PILOT_PIN, INPUT);
  pinMode(CAR_A_PILOT_PIN, OUTPUT);
  pinMode(CAR_B_PILOT_PIN, OUTPUT);
  pinMode(CAR_A_RELAY, OUTPUT);
  pinMode(CAR_B_RELAY, OUTPUT);
  digitalWrite(CAR_A_PILOT_PIN, HIGH);
  digitalWrite(CAR_B_PILOT_PIN, HIGH);
  digitalWrite(CAR_A_RELAY, LOW);
  digitalWrite(CAR_B_RELAY, LOW);

  attachInterrupt(INCOMING_PILOT_INT, handlePilotChange, CHANGE);
  
  m_Lcd.setBacklight(WHITE);
  m_Lcd.clear();
  m_Lcd.setCursor(0, 0);
  m_Lcd.print("J1772 Hydra");
  m_Lcd.setCursor(0, 1);
  m_Lcd.print(VERSION);

  boolean success = SetPinFrequency(CAR_A_PILOT_PIN, 1000);
  if (!success) m_Lcd.setBacklight(YELLOW);
  success = SetPinFrequency(CAR_B_PILOT_PIN, 1000);
  if (!success) m_Lcd.setBacklight(BLUE);
  
  // Enter state A on both cars
  setPilot(CAR_A, HIGH);
  setPilot(CAR_B, HIGH);

  delay(1000); // meanwhile, the hope is that the incoming pilot rolling average will fill in...
  m_Lcd.clear();
}

void error(int car) {
  m_Lcd.setBacklight(RED);
  if (car==CAR_A) {
    setRelay(CAR_A, LOW); // make sure the power is off
    setPilot(CAR_A, HIGH);
    m_Lcd.setCursor(0, 1);
    m_Lcd.print("A: ERR  ");
    last_car_a_state = STATE_E;
  } 
  else {
    setRelay(CAR_B, LOW); // make sure the power is off
    setPilot(CAR_B, HIGH);
    m_Lcd.setCursor(9, 1);
    m_Lcd.print("B: ERR  ");
    last_car_b_state = STATE_E;
  }
}

int relay_state_a, relay_state_b;

void setRelay(int car, int state) {
  digitalWrite(car==CAR_A?CAR_A_RELAY:CAR_B_RELAY, state);
  if (car==CAR_A) relay_state_a = state;
  else
  relay_state_b = state;
}

boolean isCarCharging(int car) {
  return car==CAR_A?relay_state_a:relay_state_b;
  //return digitalRead(car==CAR_A?CAR_A_RELAY:CAR_B_RELAY)==HIGH;
}

void setPilot(int car, int which) {
  // set the outgoing pilot for the given car to either HALF state, FULL state, or HIGH.
  int pin = car==CAR_A?CAR_A_PILOT_PIN:CAR_B_PILOT_PIN;
  if (which == HIGH) {
    pwmWrite(pin, 255);
  } else {
    pwmWrite(pin, MAtoPwm(incomingPilotMilliamps / (which==HALF?2:1)));
  }
}

int checkState(int car) {
  // Check the pilot state pin for the given car. If the voltage is > 0, then return which state the car is indicating.
  // If the voltage is < 0, then return either DUNNO (diode check pass) or STATE_E if the diode check fails.
  return car == CAR_B?STATE_C:STATE_B; // XXX FIXME
}

int readCurrent(int car_pin) {
  // Return the number of milliamps being drawn by the given car.
  return 0; // XXX FIXME
}

void reportIncomingPilot(unsigned int milliamps) {
  long sum = milliamps;
  for(int i = ROLLING_AVERAGE_SIZE - 1; i >= 1; i--) {
    rollingIncomingAverageMA[i] = rollingIncomingAverageMA[i - 1];
    sum += rollingIncomingAverageMA[i];
  }
  rollingIncomingAverageMA[0] = milliamps;
  incomingPilotMilliamps = (int)(sum / ROLLING_AVERAGE_SIZE);
  // Clamp to the maximum allowable current
  if (incomingPilotMilliamps > MAXIMUM_CURRENT * 1000) incomingPilotMilliamps = MAXIMUM_CURRENT * 1000;
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

void loop() {
  
  // Update the display

  if (last_car_a_state == STATE_E || last_car_b_state == STATE_E) {
    // One or both cars in error state
    m_Lcd.setBacklight(RED);
  } 
  else {
    boolean a = isCarCharging(CAR_A);
    boolean b = isCarCharging(CAR_B);

    // Neither car
    if (!a && !b) m_Lcd.setBacklight(GREEN);
    // Both cars
    else if (a && b) m_Lcd.setBacklight(VIOLET);
    // One car or the other
    else if (a ^ b) m_Lcd.setBacklight(TEAL);
  }

  m_Lcd.setCursor(0, 0);
  char buf[17];
  if (incomingPilotMilliamps < MINIMUM_CURRENT * 1000 || (micros() - lastPilotChange) > LOST_PILOT_GRACE_PERIOD) {
    m_Lcd.print("INPUT PILOT ERR ");
    error(CAR_A);
    error(CAR_B);
    return;
  }
  m_Lcd.print("Input Pwr ");
  m_Lcd.print(formatMilliamps(incomingPilotMilliamps));

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
      m_Lcd.setCursor(0, 1);
      m_Lcd.print("A: ---  ");
      car_a_request_time = 0;
      if (last_car_b_state != STATE_A)
        setPilot(CAR_B, FULL);
      break;
    case STATE_B:
      // The car is plugged in, or is returning to B from C.
      setRelay(CAR_A, LOW);
      m_Lcd.setCursor(0, 1);
      m_Lcd.print("A: off  ");
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
      m_Lcd.setCursor(9, 1);
      m_Lcd.print("B: ---  ");
      car_b_request_time = 0;
      if (last_car_a_state != STATE_A)
        setPilot(CAR_A, FULL);
      break;
    case STATE_B:
      // The car is plugged in, or is returning to B from C.
      setRelay(CAR_B, LOW);
      m_Lcd.setCursor(9, 1);
      m_Lcd.print("B: off  ");
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
    int car_a_draw = readCurrent(CAR_A_AMPS_PIN);

    // If the other car is charging, then we can only have half power
    int car_a_limit = incomingPilotMilliamps / isCarCharging(CAR_B)?2:1;

    if (car_a_draw > car_a_limit) {
      // car A has begun an over-draw condition. They have 5 seconds of grace before we pull the plug.
      if (car_a_overdraw_begin == 0) {
        car_a_overdraw_begin = millis();
      } 
      else {
        if (car_a_overdraw_begin < millis() - OVERDRAW_GRACE_PERIOD) {
          error(CAR_A);
        }
      }
    }
    m_Lcd.setCursor(0, 1);
    m_Lcd.print("A:");
    m_Lcd.print(formatMilliamps(car_a_draw));
  } 
  else {
    car_b_overdraw_begin = 0;
  }

  if (isCarCharging(CAR_B)) {
    int car_b_draw = readCurrent(CAR_B_AMPS_PIN);

    // If the other car is charging, then we can only have half power
    int car_b_limit = incomingPilotMilliamps / isCarCharging(CAR_A)?2:1;

    if (car_b_draw > car_b_limit) {
      // car B has begun an over-draw condition. They have 5 seconds of grace before we pull the plug.
      if (car_b_overdraw_begin == 0) {
        car_b_overdraw_begin = millis();
      } 
      else {
        if (car_b_overdraw_begin < millis() - OVERDRAW_GRACE_PERIOD) {
          error(CAR_B);
        }
      }
    }
    m_Lcd.setCursor(9, 1);
    m_Lcd.print("B:");
    m_Lcd.print(formatMilliamps(car_b_draw));
  } 
  else {
    car_b_overdraw_begin = 0;
  }

  if (car_a_request_time != 0 && car_a_request_time < millis() - TRANSITION_DELAY) {
    // We've waited long enough.
    car_a_request_time = 0;
    setRelay(CAR_A, HIGH);
    m_Lcd.setCursor(0, 1);
    m_Lcd.print("A: ON   ");
  }
  if (car_b_request_time != 0 && car_b_request_time < millis() - TRANSITION_DELAY) {
    // We've waited long enough.
    car_b_request_time = 0;
    setRelay(CAR_B, HIGH);
    m_Lcd.setCursor(9, 1);
    m_Lcd.print("B: ON   ");
  }
}


