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

#include "dst.h"


boolean isSummer(DSTRule* rules , time_t t) {

  // The algorithm here is simple. in he current year,
  // if we are past the second rule, then we take that rule.
  // otherwise, if we are past the first rule, we take that rule.
  // otherwise, we take second rule.
  for (int i = 1; i >= 0; i--) {
    if ( rules[i] <= t) return rules[i].dst == summer;
  }
  // if we got here, we are before the first rule, which means we have to assume 
  // it is a continuation of the 2nd rule ( last rule in calendar order) from the
  // previous year.
  return rules[1].dst == summer;
}

time_t toDST(DSTRule *rules, time_t t) {
  return isSummer(rules, t) ? t + 3600 : t;

}

boolean DSTRule::operator<=(time_t that) {

  uint8_t thatMo = month(that);

  // try to break based on the month.
  if ( mo < thatMo) return true;
  else if ( mo > thatMo) return false;

  // same month -- we will have to figure the day of week of the rule.
  time_t ruleBound;
  if ( week != Last)
    // our week enum starts with 0, so we will just multiply that by the 
    // week duration, and then find the next closest day of week w.r.t. that:
    ruleBound = nextDow (monthBegin(that) + SECS_PER_WEEK * week, dow) + hr * SECS_PER_HOUR;
  else
    // handle last day of week in a month: take the last day of the month 
    // (which is the first day of the next month), and 
    // find the previous closest day of week.
    ruleBound = previousDow(nextMonthBegin(that) - SECS_PER_DAY, dow) + hr * SECS_PER_HOUR;

  return ruleBound <= that;
}


