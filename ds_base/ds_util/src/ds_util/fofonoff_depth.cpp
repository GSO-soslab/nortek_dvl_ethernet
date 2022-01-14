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
#include "ds_util/fofonoff_depth.h"

#include <cmath>

namespace ds_util
{
//  Computes depth according to the Saunders and Fofonoff equation.  Taken from
//  pg 28 of Algorithms for Computation of Fundemental Properties of Seawater,
//  UNESCO 1983.
//
//  http://unesdoc.unesco.org/images/0005/000598/059832eb.pdf
double fofonoff_depth(double pressure_dbar, double latitude_deg) noexcept
{
  const auto x = pow(sin(latitude_deg / 57.29578), 2);
  const auto gravity = 9.780318 * (1.0 + (5.2788e-3 + (2.36e-5) * x) * x) + 1.092e-6 * pressure_dbar;

  auto depth =
      (((-1.82e-15 * pressure_dbar + 2.279e-10) * pressure_dbar - 2.2512e-5) * pressure_dbar + 9.72659) * pressure_dbar;

  depth /= gravity;

  return depth;
}
}
