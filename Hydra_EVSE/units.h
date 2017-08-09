#ifndef ___UNITS_H___
#define ___UNITS_H___

// With unit test unit we can use regular Arduino uno3 with kuman or some other shield.
#include "KUMAN.h"

#define HW_VERSION "UNO 3 UT"

#define DISPLAY_DEF(name) KUMAN name
#define DISPLAY_DECL(name) extern KUMAN name

#define GFI_PIN                 2
#define GFI_IRQ                 0

#define GFI_TEST_PIN            3

// Unit tests hardware spec 
// ---------- DIGITAL PINS ----------
#define CAR_A_PILOT_OUT_PIN     10
#define CAR_B_PILOT_OUT_PIN     10

#define CAR_A_RELAY             11
#define CAR_B_RELAY             11

#ifdef RELAY_TEST
#define CAR_A_RELAY_TEST        A3
#define CAR_B_RELAY_TEST        A2
#endif

// ---------- ANALOG PINS ----------
#define CAR_A_PILOT_SENSE_PIN   1
#define CAR_B_PILOT_SENSE_PIN   1

#ifdef RELAY_TEST
// When the relay test was added, the current pins were moved.
#define CAR_A_CURRENT_PIN       7
#define CAR_B_CURRENT_PIN       6
#else
#define CAR_A_CURRENT_PIN       3
#define CAR_B_CURRENT_PIN       2
#endif


int unitsSetup();
int unitsLoop();

#endif // ___UNITS_H___
