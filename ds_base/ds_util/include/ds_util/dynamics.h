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
#ifndef DS_UTIL_DYNAMICS_H
#define DS_UTIL_DYNAMICS_H

#include <tuple>

namespace ds_util
{
/// @brief Estimate the lift for a wing
///
///
/// Returns the lifting force calculated by:
///
///   L = q * A * cl
///
///   where q is the "dynamic pressure" given by:
///
///   q = 0.5 * density * speed^2
///
/// \param dynamic_pressure     0.5 * speed^2 * density
/// \param area                 Area of wing
/// \param cl                   Wing lift coefficient
///
/// \return
constexpr double calculate_wing_lift(double dynamic_pressure, double area, double cl)
{
  return dynamic_pressure * area * cl;
}

/// @brief Estimate the lift for a wing
///
///
/// Returns the lifting force calculated by:
///
///   L = 0.5 * density * speed^2 * A * cl
///
/// \param speed
/// \param density
/// \param area
/// \param cl
/// \return
constexpr double calculate_wing_lift(double speed, double density, double area, double cl)
{
  return calculate_wing_lift(0.5 * density * speed * speed, area, cl);
}

/// @brief Estimate the lift for a wing
///
///
/// Returns the lifting force calculated by:
///
///   L = 0.5 * density * speed^2 * A * cl
///
/// Where the lift coefficient 'cl' is determined by the angle of attack and two lift
/// regions:
///
///    cl = cl0 * sin(2.0 * abs(angle_of_attack)
///
///    // Low angle of attack region
///    if abs(angle_of_attack) <= cl_low_angle
///      cl += cl_low * abs(angle_of_attack)
///
///    // High angle of attack region
///    if abs(angle_of_attack) >= cl_high_angle
///      cl += cl_high * (M_PI - abs(angle_of_attack))
///
/// \param angle_of_attack         Angle of the wing (radians)
/// \param speed                   Speed through fluid
/// \param density                 Density of fluid
/// \param area                    Area of wing
/// \param cl0                     Root lift coefficient
/// \param cl_low                  Lift coefficient for low-aoa regime
/// \param cl_low_angle            Low angle of attack regime boundary
/// \param cl_high                 Lift coefficent for high-aoa regime
/// \param cl_high_angle           High angle of attack regime boundary
/// \return
double calculate_wing_lift(double angle_of_attack, double speed, double density, double area, double cl0, double cl_low,
                           double cl_low_angle, double cl_high, double cl_high_angle);

/// @brief Estimate the drag force for a wing.
///
/// Returns the drag force calculated by:
///
///   D = q * A * cd
///
///   where q is the "dynamic pressure" given by:
///
///   q = 0.5 * density * speed^2
///
/// \param dynamic_pressure
/// \param area
/// \param cd
/// \return
constexpr double calculate_wing_drag(double dynamic_pressure, double area, double cd)
{
  return dynamic_pressure * area * cd;
}

/// @brief Estimate the drag force for a wing.
///
/// Returns the drag force calculated by:
///
///   D = 0.5 * density * speed^2 * A * cd
///
/// \param speed
/// \param density
/// \param area
/// \param cd
/// \return
constexpr double calculate_wing_drag(double speed, double density, double area, double cd)
{
  return calculate_wing_drag(0.5 * density * speed * speed, area, cd);
}

/// @brief Estimate the drag force for a wing.
///
/// Returns the drag force calculated by:
///
///   D = 0.5 * density * speed^2 * A * cd
///
/// Where the final drag coefficient 'cd' is determined by the angle of attack and three drag coefficents:
///
///    da = M_PI_2 - abs_aoa;
///    cd = cd0 * std::exp(-(da * da) / (2.0 * cd1*cd1)) + cd2 * (1.0 - std::cos(4.0 * abs_aoa));
///
/// \param angle_of_attack
/// \param speed
/// \param density
/// \param area
/// \param cd0
/// \param cd1
/// \param cd2
/// \return
double calculate_wing_drag(double angle_of_attack, double speed, double density, double area, double cd0, double cd1,
                           double cd2);

/// @brief Estimate wing lift and drag forces.
///
/// This class acts as a convenience wrapper around calclate_wing_drag
/// and calculate_wing_lift so that many constants only need to be provided
/// or set once.
///
class WingDynamics
{
public:
  /// @brief
  ///
  /// \param area
  /// \param density
  WingDynamics(double area, double density);
  WingDynamics(const WingDynamics& other);

  virtual ~WingDynamics();

  /// @brief Set the wing drag coefficients.
  ///
  /// \param cd0    0th order drag coefficient
  /// \param cd1    1st order drag coefficient (default = 1)
  /// \param cd2    2nd order drag coefficient (default = 0)
  void setDragCoefficients(double cd0, double cd1 = 1, double cd2 = 0);

  /// @brief Get the wig drag coefficients
  ///
  /// \return
  std::tuple<double, double, double> dragCoefficients() const noexcept;

  /// @brief Set the wing lift coefficients
  ///
  /// \param cl0       order lift coefficient
  /// \param cl_low    Additional coefficient when the AoA is within the low region
  /// \param cl_high   Additional coefficient when the AoA is outside the high region
  void setLiftCoefficients(double cl0, double cl_low = 0, double cl_high = 0);

  /// @brief Get the wing lift coefficients
  ///
  /// \return  tuple of cl0, cl_low, cl_high
  std::tuple<double, double, double> liftCoefficients() const noexcept;

  /// @brief Set the wing angle of attack regions
  ///
  /// Angles are in radians
  ///
  /// \param cl_angle_low
  /// \param cl_angle_high
  void setLiftAngleRegions(double cl_angle_low, double cl_angle_high);

  /// @brief Get the wing angle of attack regions
  ///
  /// \return tuple of cl_low, cl_high
  std::tuple<double, double> liftAngleRegions() const noexcept;

  /// @brief Calculate the estimated wing lift
  ///
  /// \param angle_of_attack
  /// \param speed
  /// \return
  double calculate_lift(double angle_of_attack, double speed);

  /// @brief Calculate the estimated wing drag
  ///
  /// \param angle_of_attack
  /// \param speed
  /// \return
  double calculate_drag(double angle_of_attack, double speed);

  /// @brief Set the wing area
  ///
  /// \param area
  void setWingArea(double area);

  /// @brief Get the wing area
  double wingArea() const noexcept;

  /// @brief Set the fluid density
  ///
  /// \param density
  void setFluidDensity(double density);

  /// @brief Get the fluid density
  ///
  /// \return
  double fluidDensity() const noexcept;

protected:
  double cd0_;
  double cd1_;
  double cd2_;

  double cl0_;
  double cl_low_;
  double cl_high_;

  double cl_low_angle_;
  double cl_high_angle_;

  double area_;
  double density_;
};
}

#endif  // DS_UTIL_DYNAMICS_H
