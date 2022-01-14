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
//
// Created by zac on 2/9/18.
//

#ifndef DS_UTIL_THRUSTER_H
#define DS_UTIL_THRUSTER_H

namespace ds_util
{
/// @brief Calculate the required current to produce the desired force
///
/// The current is calculated using:
///
/// I = F / (alpha + beta * speed)
///
/// where
///   - I: is the required current
///   - F: is the desired force
///   - speed:  is the vehicle speed (non-negative!)
///   - alpha, beta: emperically derived fitting coeficients.
///
/// NOTE:  This transfer function has a pole at alpha = -beta*speed!
///        Divide by zero's will return infinity
///
/// NOTE:  This function assumes the current and force signs are the same!
///
/// \param alpha
/// \param beta
/// \param force
/// \param speed
/// \return  current
double linear_force_to_current(double alpha, double beta, double force, double speed);

/// @brief Transform from thruster current to approximate force
///
/// This method is the inverse of linear_force_to_current
///
/// \param alpha
/// \param beta
/// \param current
/// \param speed
/// \return force
double linear_current_to_force(double alpha, double beta, double current, double speed);
}

#endif  // DS_UTIL_THRUSTER_H
