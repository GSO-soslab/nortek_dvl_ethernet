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
#include "ds_util/dynamics.h"
#include <cmath>

namespace ds_util
{
double calculate_wing_lift(double angle_of_attack, double speed, double density, double area, double cl0, double cl_low,
                           double cl_low_angle, double cl_high, double cl_high_angle)
{
  const auto abs_aoa = std::abs(angle_of_attack);

  auto cl = cl0 * std::sin(2.0 * abs_aoa);
  if (abs_aoa <= std::abs(cl_low_angle))
  {
    cl += cl_low * abs_aoa;
  }

  if (abs_aoa >= cl_high_angle)
  {
    cl += cl_high * (M_PI - abs_aoa);
  }

  const auto result = calculate_wing_lift(speed, density, area, cl);
  return angle_of_attack >= 0 ? result : -result;
}

double calculate_wing_drag(double angle_of_attack, double speed, double density, double area, double cd0, double cd1,
                           double cd2)
{
  const auto abs_aoa = std::abs(angle_of_attack);
  const auto da = M_PI_2 - abs_aoa;
  const auto cd = cd0 * std::exp(-(da * da) / (2.0 * cd1 * cd1)) + cd2 * (1.0 - std::cos(4.0 * abs_aoa));

  return calculate_wing_drag(speed, density, area, cd);
}

WingDynamics::WingDynamics(double area, double density)
  : area_(area)
  , density_(density)
  , cl0_(0)
  , cl_low_(0)
  , cl_high_(0)
  , cl_low_angle_(0)
  , cl_high_angle_(0)
  , cd0_(0)
  , cd1_(1)
  , cd2_(0)
{
}

WingDynamics::~WingDynamics() = default;
WingDynamics::WingDynamics(const WingDynamics& other) = default;

void WingDynamics::setDragCoefficients(double cd0, double cd1, double cd2)
{
  cd0_ = cd0;
  cd1_ = cd1;
  cd2_ = cd2;
}
std::tuple<double, double, double> WingDynamics::dragCoefficients() const noexcept
{
  return std::make_tuple(cd0_, cd1_, cd2_);
}
void WingDynamics::setLiftCoefficients(double cl0, double cl_low, double cl_high)
{
  cl0_ = cl0;
  cl_low_ = cl_low;
  cl_high_ = cl_high;
}
std::tuple<double, double, double> WingDynamics::liftCoefficients() const noexcept
{
  return std::make_tuple(cl0_, cl_low_, cl_high_);
}

void WingDynamics::setLiftAngleRegions(double cl_angle_low, double cl_angle_high)
{
  cl_low_angle_ = std::abs(cl_angle_low);
  cl_high_angle_ = std::abs(cl_angle_high);
}

std::tuple<double, double> WingDynamics::liftAngleRegions() const noexcept
{
  return std::make_tuple(cl_low_angle_, cl_high_angle_);
}

double WingDynamics::calculate_lift(double angle_of_attack, double speed)
{
  return calculate_wing_lift(angle_of_attack, speed, density_, area_, cl0_, cl_low_, cl_low_angle_, cl_high_,
                             cl_high_angle_);
}

double WingDynamics::calculate_drag(double angle_of_attack, double speed)
{
  return calculate_wing_drag(angle_of_attack, speed, density_, area_, cd0_, cd1_, cd2_);
}
void WingDynamics::setWingArea(double area)
{
  area_ = area;
}
double WingDynamics::wingArea() const noexcept
{
  return area_;
}
void WingDynamics::setFluidDensity(double density)
{
  density_ = density;
}
double WingDynamics::fluidDensity() const noexcept
{
  return density_;
}
}
