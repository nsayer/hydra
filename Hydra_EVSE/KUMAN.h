/*
 * Display and button compatibility layer for Hydra for KUMAN shield display
 */
#ifndef KUMAN_H_INCLUDED
#define KUMAN_H_INCLUDED

#include <LiquidCrystal.h>

#define BUTTON_RIGHT 0x04
#define BUTTON_UP 0x08
#define BUTTON_DOWN 0x10
#define BUTTON_LEFT 0x02
#define BUTTON_SELECT 0x01
#define BUTTON_PIN 0

#define KUMAN_ADDR 8,9,4,5,6,7

// To retain compatibility with LiquidTWI2 library.
class KUMAN: public LiquidCrystal {

  public :

  KUMAN () : LiquidCrystal(KUMAN_ADDR) { }

  void setBacklight(uint8_t status) { }
  
  uint8_t readButtons() { 
    int r = analogRead(BUTTON_PIN);

    if ( r > 1000 ) return 0; //  no button

    // otherwise, work with ranges which roughly follow powers of 2.
    
    if ( (r >>= 6) == 0) return BUTTON_RIGHT;
    if ( (r >>= 1) == 0) return BUTTON_UP;
    if ( (r >>= 1) == 0) return BUTTON_DOWN;
    if ( (r >>= 1) == 0) return BUTTON_LEFT;
    return BUTTON_SELECT;
  }
    

  void setMCPType(uint8_t mcptype) {}

  
  
};


#endif // KUMAN_H_INCLUDED
