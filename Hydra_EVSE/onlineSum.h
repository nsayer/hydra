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

#ifndef ___ONLINESUM_H___
#define ___ONLINESUM_H___

#include "Arduino.h"

///////////////////////////////////////////////////////////////////
// Exponentially Weighted Average summarizer for irregularly sampled data.
// References:
// http://tdunning.blogspot.com/2011/03/exponentially-weighted-averaging-for.html
// My mods for the updates-in-the past (ensures no NaNs created regardless of updates order):
// http://weatheringthrutechdays.blogspot.com/2011/04/follow-up-for-mean-summarizer-post.html

// -log(0.5)
#define MINUS_LOG_05 0.6931471805599453

// T is double or float
template <class T>
class EWASum {

  friend class RTCModel;

    T alpha;
    T w, s, tn;

  public:

    // tHalfWeightPeriod is the amount of t (timeline point) in the past at which the observation is
    // exponentially weighed at exactly 0.5 vs. any observation right now. Since it is weighed
    // exponentially rather than linearly, observations at full period still do have a nonzero weight,
    // and in general with this summarizer most recent observations are being discounted at fastest
    // weight but never actually discounted completely (indefinite history), down to the precision of
    // the arithmetics. At full period the weights are thus still coming in at exp(-2) = 0.135 (13.5%).
    EWASum(T tHalfWeightPeriod) : alpha(tHalfWeightPeriod / (T) MINUS_LOG_05) {
      reset();
    };

    void reset() {
      w = s = tn = 0;
    }

    void update(T x, T t);

    // Evaluate EWA
    T ewa();
};

template <class T>
void EWASum<T>::update(T x, T t) {
  T pi = exp(-abs(tn - t) / alpha);
  if (t > tn) {
    s = pi * s + x;
    w = pi * w + 1;
    tn = t;
  } else {
    s += pi * x;
    w += pi;
  }
}

template <class T>
T EWASum<T>::ewa() {
  // In this sum, weights low enough should not happen, as we never discount the most recent weight,
  // which is at least one. So w>= 1 if there were at least one sample; and w = exactly 0 if there
  // are no samples at all. With no samples, we just return 0.
  return (abs(w) < 1e-6) ? 0 : s / w;
}

// On Arduino, double seems to be the same as float and take 4 bytes!
typedef EWASum<double> EWASumD;
typedef EWASum<float> EWASumF;

#define RTC_CALIB_PERIOD SECS_PER_DAY

// RTC model
class RTCModel {

  protected:

    EWASumD   ewa;
    char      calib, calibErr;
    double    initCalibRate, rate, dn;

  public:

    // for PT7C4311WEX, actual rate to add seconds per day seems to be 0.35, and to delete seconds is 0.175.
    // in simulations, 0.5 seems to be a good initial value.
    // half rate period should be in seconds, e.g., 2* SECS_PER_WEEK seems to be a good value.
    RTCModel(double initCalibRate);

    // update model after time adjustment: t is the time after adjustment (in seconds, e.g. what now() returns),
    // and adjustmentn is number of seconds added. E.g., if we moved clock forward, we wanted to add seconds (+),
    // and if we moved clock backwards, we wanted to remove extra seconds (-). Adjustment is absolute value of
    // adjustment given by a human thru the clock menu (NOT weighted per day!). Returns new calibration value
    // to use in the rtc chip.
    char update(double t, double adjustment);

    // get current calibration value.
    char getCalib() {
      return calib;
    }

    void reset();

    // For testing only
    double getRate() { return rate; }



};

#endif // ___ONLINESUM_H___
