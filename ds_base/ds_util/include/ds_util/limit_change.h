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
// Created by ivaughn on 3/13/19.
//

#ifndef DS_UTIL_LIMIT_CHANGE_H
#define DS_UTIL_LIMIT_CHANGE_H

namespace ds_util {

/// Limit a goal value to an interval about a particular center point
///
/// The returned value is as close to the goal as possible,
/// but clamped to limit_center +/- limit_delta
/// \param goal The desired final value
/// \param limit_delta The maximum permissible distance from limit_center
/// \param limit_center The center of the permissible region
/// \return Either the goal value, or the closer of limit_center +/- limit_delta
double limit_change(double goal, double limit_delta, double limit_center);

/// Limit a goal value to an interval about a particular center point on a circle
///
/// The returned value is as close to the goal as possible,
/// but clamped to limit_center +/- limit_delta
/// \param goal The desired final value
/// \param limit_delta The maximum permissible distance from limit_center
/// \param limit_center The center of the permissible region
/// \return Either the goal value, or the closer of limit_center +/- limit_delta
double limit_change_heading(double goal, double limit_delta, double limit_center);

}

#endif //DS_UTIL_LIMIT_CHANGE_H
