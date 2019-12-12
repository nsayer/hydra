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

#include "onlineSum.h"
#include <Time.h>

#define RTC_CALIB_BITS 8
#define RTC_MIN_CALIB (-1 << RTC_CALIB_BITS - 1)
#define RTC_MAX_CALIB (~RTC_MIN_CALIB)
#define RTC_HALF_RATE_PERIOD (2 * SECS_PER_WEEK)

// We normally work in terms of seconds per day (even if it is a fraction of such).
#define RTC_CALIB_PERIOD SECS_PER_DAY

// maximum normalized adjustment (secs per day) we will try to correct; otherwise, reset on bigger adjustments,
// assume they are not clock correction but rather time change
#define RTC_MAX_D 80

RTCModel::RTCModel(double initCalibRate) : ewa(RTC_HALF_RATE_PERIOD), initCalibRate(initCalibRate) {
  reset();
}

void RTCModel::reset() {
  ewa.reset();
  rate = initCalibRate;
  dn = 0;
  calibErr = calib = 0;
}

char RTCModel::update(double t, double adjustment) {
  if ( t < ewa.tn ) reset();
  if ( ewa.tn == 0) {
    ewa.update(rate, t);
  } else {
    double d = adjustment / (t - ewa.tn ) * RTC_CALIB_PERIOD;
    if ( abs(d) > RTC_MAX_D) {
      reset();
      return calib;
    } else if (calibErr != 0) {
      double newRate = (dn - d) / calibErr;
      if (newRate < 1e-3 || isnan(newRate)) {
        reset();
        return calib;
      }
      ewa.update(newRate, t);
      rate = ewa.ewa();
    } else {
      ewa.update(rate, t);
    }
    dn = d;
    int newCalibErr = round(dn / rate);
    if ( calib+ newCalibErr > RTC_MAX_CALIB) {
      calibErr = RTC_MAX_CALIB - calib;
    } else if (calib + newCalibErr < RTC_MIN_CALIB) { 
      calibErr = RTC_MIN_CALIB - calib;
    } else calibErr = (char)newCalibErr;
    calib += calibErr;
  }
  return calib;
}

