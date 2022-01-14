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
#ifndef DS_UTIL_ANGLE_H
#define DS_UTIL_ANGLE_H

namespace ds_util
{
/// @brief Return the separation between two angles.
///
/// This function returns the difference between two angles.  The returned
/// value will always be within the interval [-pi, pi].  The sign of the
/// returned value indicates the direction of rotation:
///
///     angular_separation_radians(0, 20*M_PI/180) == 20*M_PI/180;
///     angular_separation_radians(20*M_PI/180, 0) == -20*M_PI/180;
///
/// \param from
/// \param to
/// \return
double angular_separation_radians(double from, double to);

/// @brief Return the equivalent angle on a heading-like interval
///
/// This function takes an input, in radians, and converts to a value on
/// the traditional interval for heading, [0, 2pi)
double normalize_heading_radians(double raw);

/// @brief Return the equivalent angle on a zero-centered interval
///
/// This function takes an input, in radians, and converts to a value on
/// the traditional interval zero-centered values like roll and pitch, [-pi, pi]
double normalize_rollpitch_radians(double raw);


}

#endif  // DS_UTIL_ANGLE_H
