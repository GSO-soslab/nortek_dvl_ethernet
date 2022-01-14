/**
* Copyright 2019 Woods Hole Oceanographic Institution
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice,
*    this list of conditions and the following disclaimer.
*
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* 3. Neither the name of the copyright holder nor the names of its contributors
*    may be used to endorse or promote products derived from this software
*    without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/
//
// Created by ivaughn on 3/14/19.
//

#include "ds_util/limit_change.h"
#include <iostream>
#include <cmath>

namespace ds_util {

double limit_change(double goal, double limit_delta, double limit_center) {

  if (goal > limit_center + limit_delta) {
    return limit_center + limit_delta;
  }

  if (goal < limit_center - limit_delta) {
    return limit_center - limit_delta;
  }

  return goal;

}

double limit_change_heading(double goal, double limit_delta, double limit_center) {

  // tidy up the inputs
  goal = std::fmod(goal, 2*M_PI);
  if (goal < 0) {
    goal += 2*M_PI;
  }

  limit_center = std::fmod(limit_center, 2*M_PI);
  if (limit_center < 0) {
    limit_center += 2*M_PI;
  }

  // take the difference and clamp to +/-180
  double diff = goal - limit_center;
  if (diff >  M_PI) { diff -= 2*M_PI; }
  if (diff < -M_PI) { diff += 2*M_PI; }

  // clamp to the limit change
  if (diff >  limit_delta) { diff = limit_delta; }
  if (diff < -limit_delta) { diff = -limit_delta; }

  // put the date back on
  double ret = diff + limit_center;

  // clamp to [0,360)
  if (ret < 0) { ret += 2*M_PI; }

  return std::fmod(ret, 2*M_PI);
}

}

