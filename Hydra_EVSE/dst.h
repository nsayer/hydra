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

#ifndef ___DST_H___
#define ___DST_H___

// Standard Arduino types like boolean
#if ARDUINO >= 100
#include <Arduino.h>
#else
#include <WProgram.h>
#endif

#include <Time.h>

enum week_t {First, Second, Third, Fourth, Last};
enum dow_t {Sun = 1, Mon, Tue, Wed, Thu, Fri, Sat};
enum month_t {Jan = 1, Feb, Mar, Apr, May, Jun, Jul, Aug, Sep, Oct, Nov, Dec};
enum dst_t {summer, winter};

//structure to describe rules for when daylight/summer time begins,
//or when standard time begins.
struct DSTRule
{
  dst_t   dst;       // Whether the rule switches to summer or winter time
  uint8_t week;      // First, Second, Third, Fourth, or Last week of the month
  uint8_t dow;       // day of week, here and on per Time.h
  uint8_t mo;
  uint8_t hr;

  boolean operator<=(time_t that);
  boolean operator>(time_t that) { return ! (*this <= that); }
};
// Exactly 2 rules are expected for the rules argument, in calendar succession.
extern boolean isSummer(DSTRule* rules , time_t);
extern time_t toDST(DSTRule *rules, time_t);

#define sameWeekDow(_time_, _dow_)  ((_time_ / SECS_PER_DAY + _dow_ - dayOfWeek(_time_)) * SECS_PER_DAY)

#define monthBegin(_time_) ((_time_ / SECS_PER_DAY + 1 - day(_time_)) * SECS_PER_DAY)

#define nextMonthBegin(_time_) monthBegin((_time_ / SECS_PER_DAY + 32 - day(_time_)) * SECS_PER_DAY)

// This returns next day of week w.r.t. current time. if today is the same day of week
// as the one requested, returns today's midnight.
inline time_t nextDow (time_t _time_, uint8_t _dow_) {
  time_t sameWDow = sameWeekDow( _time_, _dow_);
  return sameWDow >= previousMidnight(_time_) ? sameWDow : sameWDow + SECS_PER_WEEK;
}

// returns previous closest day of week to the _time_. If today is the same as _dow_, returns today's midnight.
inline time_t previousDow (time_t _time_, uint8_t _dow_) {
  time_t sameWDow = sameWeekDow( _time_, _dow_);
  return sameWDow <= previousMidnight(_time_) ? sameWDow : sameWDow - SECS_PER_WEEK;
}



// We don't really need to support timezones. We just want to perform automatic DST switching.
// The following are the U.S. DST rules. If you live elsewhere, the customize these. The params
// are a descriptive string (not used, but must be 5 chars or less), a "descriptor" for which
// weekday in the active month is in the rule (first, second... last), the day of the week,
// the month, and the hour of the day. The last parameter is the offset in minutes. You should
// have the "winter" rule have a 0 offset so that turning off DST returns you to winter time.
// The summer offset should be relative to winter time (probably +60 minutes).

// US
#define US_DST_RULES(name)  DSTRule name[] = {{ summer, Second, Sun, Mar, 2 }, { winter, First, Sun, Nov, 2 }}
// Europe
#define EU_DST_RULES(name)  DSTRule name[] = {{ summer, Last, Sun, Mar, 1 }, { winter, Last, Sun, Oct, 1 }}
// Australia - note the reversal due to the Southern hemisphere
#define AU_DST_RULES(name)  DSTRule name[] = {{ winter, First, Sun, Apr, 2 }, { summer, First, Sun, Oct, 2 }}


#endif // ___DST_H___
