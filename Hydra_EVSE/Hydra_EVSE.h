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

#ifndef ___HYDRA_EVSE_H___
#define ___HYDRA_EVSE_H___

// Standard Arduino types like boolean
#include <Arduino.h>

#include <avr/wdt.h>
#include <Wire.h>
#include <LiquidTWI2.h>
#include <PWM.h>
#include <EEPROM.h>
#include <Time.h>
#include <DS1307RTC.h>
//#include <Timezone.h>
#include "onlineSum.h"
#include "dst.h"

// SW version
#define SW_VERSION "2.4.1"

// Define this for the basic unit tests run in a generica arduino uno board with a display shield.
// #define UNIT_TESTS

#define UINT_BITS (sizeof(unsigned int) << 3)
#define ULONG_BITS (sizeof(unsigned long) << 3)

// By historical accident, car B is actually
// the lower pin number in most cases.
//
// The reason it matters is that in the UI, car "A"
// is on the left. Therefore, car "A" is on the *right*
// side of the component side of the board, since the LCD
// is now permanently mounted. All things being equal, it's
// more user-friendly to have the physical cable layout of
// your chassis correspond to the display.
//
// If, for some reason, your wiring layout differs, then...
// #define SWAP_CARS 1

// If your Hydra lacks the ground test functionality, comment this out
//#define GROUND_TEST

// If your Hydra lacks the Relay test functionality, comment this out
//#define RELAY_TEST

// If your Hydra has a combined relay test / GCM system, then uncomment this
#define RELAY_TESTS_GROUND

#ifdef RELAY_TESTS_GROUND
// This implies relay test
#define RELAY_TEST
// but implies an alternative to the ground test
#undef GROUND_TEST
#endif

// Current production or unit test hardware (currently, version 2.3.1 in my posession).
#ifndef UNIT_TESTS
#include "hw_4_3_1.h"
#else
#include "units.h"
#endif

// Some vehicles will turn the contactors on and off repeatedly during
// some operations. If the other vehicle is charging, that will slow things
// down as the other car's pilot must be raised and lowered, with the attendant
// delay. Turning this on imposes a delay between raising the remaining car's
// pilot. If the other car changes its mind, it can get power without a delay.
//#define QUICK_CYCLING_WORKAROUND

#ifdef GROUND_TEST

// This must be high at all times while charging either car, or else it's a ground failure.
#define GROUND_TEST_PIN 6

#endif


#ifdef QUICK_CYCLING_WORKAROUND

// How many minutes do we wait after one car finishes before raising the other pilot?
#define PILOT_RELEASE_HOLDOFF_MINUTES 5

#endif

// After the relay changes state, don't bomb on relay or ground errors for this long.
#define RELAY_TEST_GRACE_TIME 500


// for things like erroring out a car
// Note: for current implementation it is essential that CAR_B == CAR_A + 1 so don't change that.
#define BOTH                    0x0
#define CAR_A                   0x1
#define CAR_B                   0x2
#define CAR_MASK                0x3
#define DEFAULT_TIEBREAK        CAR_A

// display status to be combined with CAR_A, CAR_B, or BOTH
#define STATUS_TIEBREAK         0x4

// Mutually exclusive statuses -- bits 3, 4, 5
#define STATUS_UNPLUGGED        (0x0u << 3)
#define STATUS_OFF              (0x1u << 3)
#define STATUS_ON               (0x2u << 3)
#define STATUS_WAIT             (0x3u << 3)
#define STATUS_DONE             (0x4u << 3)
#define STATUS_ERR              (0x5u << 3)
// in Arduino, x >> -bits does not work like on Intel. need actually use the size of the int
#define STATUS_MASK             ( -1u >> UINT_BITS - 3 << 3)

// Next34 bits for error codes (bits 6, 7, 8 )
#define STATUS_ERR_MASK         ( -1u >> UINT_BITS - 3 << 6)
#define STATUS_ERR_F            ( 0x0u << 6 )
#define STATUS_ERR_O            ( 0x1u << 6 )
// gfi
#define STATUS_ERR_G            ( 0x2u << 6 )
#define STATUS_ERR_T            ( 0x3u << 6 )
#define STATUS_ERR_R            ( 0x4u << 6 )
#define STATUS_ERR_E            ( 0x5u << 6 )

// Don't use 0 or 1 because that's the value of LOW and HIGH.
#define HALF                    3
#define FULL                    4

#define STATE_A                 1
#define STATE_B                 2
#define STATE_C                 3
#define STATE_D                 4
#define STATE_E                 5
#define DUNNO                   0

#define GFI_TEST_CYCLES 50 // 50 cycles
#define GFI_PULSE_DURATION_MS 8000 // of roughly 60 Hz. - 8 ms as a half-cycle
#define GFI_TEST_CLEAR_TIME 100 // Takes the GFCI this long to clear
#define GFI_TEST_DEBOUNCE_TIME 400 // Delay extra time after GFCI clears to make sure it stays.

//After each GFCI event we will retry charging up to 4 times after a 15 minute
// delay per event. (UL 2231). This MUST BE bigger than ERROR_DELAY (power on high pilot withdrawal),
#define GFI_CLEAR_MS (15 * 60 * 1000)

#define GFI_CLEAR_ATTEMPTS 4


// These are the expected analogRead() ranges for pilot read-back from the cars.
// These are calculated from the expected voltages seen through the dividor network,
// then scaling those voltages for 0-1024.

// 11 volts
#define STATE_A_MIN      870
// 10 volts
#define STATE_B_MAX      869
// 8 volts
#define STATE_B_MIN      775
// 7 volts
#define STATE_C_MAX      774
// 5 volts
#define STATE_C_MIN      682
// 4 volts
#define STATE_D_MAX      681
// 2 volts
#define STATE_D_MIN      610
// This represents 0 volts. No, it's not 512. Deal.
#define PILOT_0V         556
// -10 volts. We're fairly generous.
#define PILOT_DIODE_MAX  250

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

// Amount of time, in milliseconds, we will look for positive and negative peaks on the car
// pilot pins. It takes around .1 ms to do one read, so we should get a bit less than 200 chances
// this way.
#define STATE_CHECK_INTERVAL 20

// How often (in milliseconds) is the state of both cars logged?
#define STATE_LOG_INTERVAL 60000

// This is the number of duty cycle or ammeter samples we keep to make a rolling average to stabilize
// the display. The balance here is between stability and responsiveness,
#define ROLLING_AVERAGE_SIZE 10

// The maximum number of milliseconds to sample an ammeter pin in order to find three zero-crossings.
// one and a half cycles at 50 Hz is 30 ms.
#define CURRENT_SAMPLE_INTERVAL 35

// Once we detect a zero-crossing, we should not look for one for another quarter cycle or so. 1/4
// cycle at 50 Hz is 5 ms.
#define CURRENT_ZERO_DEBOUNCE_INTERVAL 5

// How often (in milliseconds) is the current draw by a car logged?
#define CURRENT_LOG_INTERVAL 1000

// This is the minimum of the ampacity (in milliamps) of all of the components from the distribution block to the plug -
// the wire to the relay, the relay itself, and the J1772 cable and plug. This is not part of the UI, because it's never
// going to change once you build your Hydra.
// reference design
#define MAXIMUM_OUTLET_CURRENT 30000
// Mega hydra
//#define MAXIMUM_OUTLET_CURRENT 50000

// This multiplier is the number of milliamps per A/d converter unit.

// First, you need to select the burden resistor for the CT. You choose the largest value possible such that
// the maximum peak-to-peak voltage for the current range is 5 volts. To obtain this value, divide the maximum
// outlet current by the Te. That value is the maximum CT current RMS. You must convert that to P-P, so multiply
// by 2*sqrt(2). Divide 5 by that value and select the next lower standard resistor value. For the reference
// design, Te is 1018 and the outlet maximum is 30. 5/((30/1018)*2*sqrt(2)) = 59.995, so a 56 ohm resistor
// is called for. Call this value Rb (burden resistor).
//
// Note, however, that the 56 ohm value assumes that the RMS-to-peak conversion is from a sine wave. That's
// not necessarily the case. Most battery chargers are going to be switching power supplies, which may have
// more complex waveforms. So it's best to pick the next lower standard resistor value to give a little bit
// of headroom. That's why the recommended values changed from 56 to 47 ohms and 32 to 27 ohms.
//
// Next, one must use Te and Rb to determine the volts-per-amp value. Note that the readCurrent()
// method calculates the RMS value before the scaling factor, so RMS need not be taken into account.
// (1 / Te) * Rb = Rb / Te = Volts per Amp. For the reference design, that's 55.009 mV.

// Each count of the A/d converter is 4.882 mV (5/1024). V/A divided by V/unit is unit/A. For the reference
// design, that's 11.26. But we want milliamps per unit, so divide that into 1000 to get 88.7625558. Round near...
// for RB = 56 - original design
//#define CURRENT_SCALE_FACTOR 89
// for RB = 47 - current reference design
#define CURRENT_SCALE_FACTOR 106
// for RB = 32 - Mega Hydra prototype
//#define CURRENT_SCALE_FACTOR 155
// doe RB = 27 - Mega Hydra production
//#define CURRENT_SCALE_FACTOR 184

// Irregular-sampled EWA half-period for ammeter displays (the time after which data points a weighed 1/2
// w.r.t. most recent data point). This is for Ammeter display only, it does not affect current measurements
// for overdraw etc. purposes.
#define AMM_DISPLAY_HALF_PERIOD 1500



#define LOG_NONE 0
#define LOG_INFO 1
#define LOG_DEBUG 2
#define LOG_TRACE 3

// Hardware versions 1.0 and beyond have a 6 pin FTDI compatible port laid out on the board.
// We're going to use this sort of "log4j" style. The log level is 0 for no logging at all
// (and if it's 0, the Serial won't be initialized), 1 for info level logging (which will
// simply include state transitions only), or 2 for debugging.
#ifdef UNIT_TESTS
#define SERIAL_LOG_LEVEL LOG_DEBUG
#else
#define SERIAL_LOG_LEVEL LOG_NONE
#endif

#if LOG_INFO <= SERIAL_LOG_LEVEL
#define logInfo(...) logImpl(LOG_INFO, __VA_ARGS__)
#else
#define logInfo(...)
#endif

#if LOG_DEBUG <= SERIAL_LOG_LEVEL
#define logDebug(...) logImpl(LOG_DEBUG, __VA_ARGS__)
#else
#define logDebug(...)
#endif

#if LOG_TRACE <= SERIAL_LOG_LEVEL
#define logTrace(...) logImpl(LOG_TRACE, __VA_ARGS__)
#else
#define logTrace(...)
#endif


#define SERIAL_BAUD_RATE 9600

// in shared mode, two cars connected simultaneously will get 50% of the incoming pilot
#define MODE_SHARED 0
// in sequential mode, the first car to enter state B gets the pilot until it transitions
// from C/D to B again. When it does, if the other car is in state B1, then it will be given
// a pilot.
#define MODE_SEQUENTIAL 1

// In sequential mode, if both cars are sitting in state B, flip the pilot back and forth between
// both cars every so often in case one of them changes their mind.
#define SEQ_MODE_OFFER_TIMEOUT (5 * 60 * 1000L) // 5 minutes

// If we add more modes, set this to the highest numbered one.
#define LAST_MODE MODE_SEQUENTIAL

// Set this to the desired startup mode
#define DEFAULT_MODE MODE_SHARED

// Which button do we use?
#define BUTTON BUTTON_SELECT
// If we see any state change on the button, we ignore all changes for this long
#define BUTTON_DEBOUNCE_INTERVAL 50
// How long does the button have to stay down before we call it a LONG push?
#define BUTTON_LONG_START 250

#define EVENT_NONE 0
#define EVENT_SHORT_PUSH 1
#define EVENT_LONG_PUSH 2

// timer event types
#define TE_NONE 0
#define TE_PAUSE 1
#define TE_UNPAUSE 2
#define TE_LAST TE_UNPAUSE

// Uncomment this if you want a 24 hour clock instead of AM/PM
// #define CLOCK_24HOUR 1

extern char p_buffer[];
#define P(str) (strcpy_P(p_buffer, PSTR(str)), p_buffer)

#ifdef UNIT_TESTS
#include "units.h"
#endif

struct timeouts_struct {
  unsigned long sequential_pilot_timeout;
  unsigned long button_press_time, button_debounce_time;
  // last gfi time
  unsigned long gfi_time;

  timeout_struct() {
    clear();
  }

  void clear() {
    memset(this, 0, sizeof(*this));
  }
};

struct car_struct {
  // CAR_A or CAR_B
  unsigned char car;
  car_struct& them;
  unsigned int relay_pin, pilot_out_pin, pilot_sense_pin, current_pin;
  volatile unsigned int relay_state;
  unsigned int last_state;
  unsigned long overdraw_begin;
  unsigned long request_time;
  unsigned long error_time;
  unsigned long last_current_log;
  boolean seq_done = false;
  unsigned int pilot_state;
  EWASumD ammSum;

  car_struct(unsigned int car, int themOffset, unsigned int relay_pin,
             unsigned int pilot_out_pin, unsigned int pilot_sense_pin, unsigned int current_pin) :
    car(car),
    them(*(this + themOffset)),
    relay_pin(relay_pin),
    pilot_out_pin(pilot_out_pin),
    pilot_sense_pin(pilot_sense_pin),
    current_pin(current_pin),
    last_state(DUNNO),
    relay_state(LOW),
    overdraw_begin(0),
    request_time(0),
    error_time(0),
    last_current_log(0),
    pilot_state(LOW),
    ammSum(AMM_DISPLAY_HALF_PERIOD)
  {
  };

  void setRelay(unsigned int state);
  void setPilot(unsigned int which);
  void shared_mode_transition(unsigned int car_state);
  void sequential_mode_transition(unsigned int car_state);
  boolean isCarCharging();
  int checkState();
  unsigned long readCurrent();
  // main loop symmetrical logic:
  void loopCheckPilot(unsigned int car_state);
  void loopCurrentMonitor();
  void loopCheckDelayedTransition();
  void loopSeqHandover(unsigned long nowMs);
  // Inlines
  char carLetter() {
    return 'A' + car - CAR_A;
  }
  // Returns 0 for car A and 8 for car B. Typically, to print display status or current.
  unsigned int dispCol() {
    return 8 * ( car - CAR_A );
  }

};

#define EVENT_COUNT 4

typedef struct event_struct {
  unsigned char hour;
  unsigned char minute;
  unsigned char dow_mask;
  unsigned char event_type;

  void validate() {
    // check values are out of range, if yes, set to the default values.
    if (event_type > TE_LAST) event_type = TE_NONE;
    if (hour > 23) hour = 0;
    if (minute > 59) minute = 0;
    dow_mask &= 0x7f; //there are only 7 days of the week
  }

  void reset() {
    event_type = TE_NONE;
    hour = 0;
    minute = 0;
    dow_mask = 0;
  }

} event_type;

// Calibration menu items

// this is in 0.1A units
#define CALIB_AMM_MAX 5

// this is in -% units. Can derate pilots up to 5%.
#define CALIB_PILOT_MAX 10

typedef struct calib_struct {
  char amm_a, amm_b, pilot_a, pilot_b;

  static unsigned char menuItem;

  calib_struct()  {
    reset();
  }

  void validate() {
    // Check the values are out of range, if yes, then put the default values.
    if (abs(amm_a) > CALIB_AMM_MAX) amm_a = 0;
    if (abs(amm_b) > CALIB_AMM_MAX) amm_b = 0;
    if (pilot_a > 0 || pilot_a < -CALIB_PILOT_MAX) pilot_a = 0;
    if (pilot_b > 0 || pilot_b < -CALIB_PILOT_MAX) pilot_b = 0;
  }

  void reset() {
    amm_a = 0;
    amm_b = 0;
    pilot_a = 0;
    pilot_b = 0;
  }

  void doMenu(boolean initialize);

} calib_type;

// All the data that goes to be read/saved to eprom

// eprom persistence format signature (usually minimally compatible SW_VERSION):
// 2.4.1
#define PERSIST_SIG 2411

// debug to reset eprom
//#define PERSIST_SIG -1

#define EEPROM_OFFSET 0

struct persisted_struct {

  unsigned int signature;
  unsigned char operatingMode;
  unsigned int max_amps;
  boolean enable_dst;
  event_struct events[EVENT_COUNT];
  calib_struct calib;

  RTCModel rtc;

  persisted_struct() : rtc(0.5) {
    eepromRead();
    validate();
  }

  void eepromRead() {

    EEPROMClass().get(EEPROM_OFFSET, *this);

  }

  void eepromWrite() {
    EEPROMClass().put(EEPROM_OFFSET, *this);
  }

  void validate();

  void reset();
};

////////////////////////////////////////////////////////////
// This is stuff from Hydra_EVSE.ino that unit tests use

extern boolean inMenu;
extern void doMenu(boolean);
extern void logImpl(unsigned int level, const char * fmt_str, ...);
DISPLAY_DECL(display);
extern persisted_struct persisted;
extern void Delay(unsigned int);
extern void displayStatus(unsigned int status);
extern char errLetter(unsigned int status);
extern char *formatMilliamps(unsigned long milliamps);

extern boolean &enable_dst;
extern car_struct cars[];
extern DSTRule dstRules[2];
extern timeouts_struct timeouts;


///////////////////////////////////////////////////////////
// Inline declarations
static inline time_t localTime()
{
  time_t t = now();
  return persisted.enable_dst && isSummer(dstRules, t) ? t + SECS_PER_HOUR : t;
}


#endif // ___HYDRA_EVSE_H___
