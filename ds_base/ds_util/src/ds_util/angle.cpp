/**
* Copyright 2018 Woods Hole Oceanographic Institution
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
#include "ds_util/angle.h"
#include <cmath>
namespace ds_util
{
double angular_separation_radians(double from, double to)
{
  // dot product produces the cosine of the included angle
  auto cDPSI = (std::sin(from) * std::sin(to) + std::cos(from) * std::cos(to));

  // cross product produces the sine of the included angle
  auto sDPSI = (std::cos(from) * std::sin(to) - std::sin(from) * std::cos(to));

  // Finally, take the arctan of the sine and cosine.  This gives us a signed
  // included angle.
  auto DPSI = std::atan2((sDPSI), (cDPSI));

  return DPSI;
}

double normalize_heading_radians(double raw) {
  double ret = std::fmod(raw, 2*M_PI);
  if (ret < 0) {
    return ret + 2*M_PI;
  }
  return ret;
}

double normalize_rollpitch_radians(double raw) {
  // first, clamp to [0, 2*pi)
  double ret = std::fmod(raw, 2*M_PI);

  // then normalize the way we'll want
  if (ret > M_PI) {
    return ret - 2*M_PI;
  }
  if (ret < -M_PI) {
    return ret + 2*M_PI;
  }
  return ret;
}

}