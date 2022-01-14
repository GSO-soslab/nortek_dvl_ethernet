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
#ifndef DS_UTIL_REFERENCE_SMOOTHING_H
#define DS_UTIL_REFERENCE_SMOOTHING_H

#include <tuple>
#include <memory>

namespace ds_util
{
/// @brief returns +-1, depending on the sign of the input argument
double sgn(double x);

/// @brief Creates a single-DOF smoothed trajectory between the current reference
///        position and velocity to a goal position employing a trapezoidal
///        velocity profile.
/// \param[in] goal Current goal
/// \param[in] ref_pos_in Input reference position
/// \param[in] ref_vel_in Input reference velocity
/// \param[in] ref_acc_in Input reference acceleration
/// \param[in] max_vel Maximum velocity
/// \param[in] max_acc Maximum acceleration
/// \param[in] dt Timestep for trapezoidal smoothing
/// \return A std::tuple containing smoothed reference position, velocity, and acceleration
std::tuple<double, double, double> goal_trajectory_trapezoidal(double goal, double ref_pos_in, double ref_vel_in,
                                                               double ref_acc_in, double max_vel, double max_acc,
                                                               double dt);

/// @brief A convenience class around goal_trajectory_trapezoidal
///
/// This class stores the max velocity and acceleration parameters for
/// goal_trajectory_trapezoidal, cutting down the number of required
/// arguments for smoothing iterations
class TrapezoidalSmoother
{
  struct Impl;

public:
  TrapezoidalSmoother();
  ~TrapezoidalSmoother();

  void setMaxVelocity(double max_vel);
  double maxVelocity() const noexcept;

  void setMaxAcceleration(double max_acc);
  double maxAcceleration() const noexcept;

  void reset();

  /// @brief Single-DOF smoothed trajectory between the current reference
  ///        position and velocity to a goal position employing a trapezoidal
  ///        velocity profile.
  ///
  /// The returned pos, vel, and acceleration quantities are also used
  /// as the reference quantities for the next iteration.
  ///
  /// \param[in] goal Current goal
  /// \param[in] ref_pos_in Input reference position
  /// \param[in] ref_vel_in Input reference velocity
  /// \param[in] ref_acc_in Input reference acceleration
  /// \param[in] dt Timestep for trapezoidal smoothing
  /// \return A std::tuple containing smoothed reference position, velocity, and acceleration
  std::tuple<double, double, double> execute(double goal_pos, double ref_pos, double ref_vel, double ref_acc,
                                             double dt);

  /// @brief Single-DOF smoothed trajectory.
  ///
  /// Uses the current reference pos, vel, and acceleration from the
  /// last update.
  ///
  /// The returned pos, vel, and acceleration quantities are also used
  /// as the reference quantities for the next iteration.
  ///
  /// \param goal_pos
  /// \param dt
  /// \return
  std::tuple<double, double, double> execute(double goal_pos, double dt);

  /// @brief Set the reference pos, vel, and acceleration.
  ///
  /// The reference quantities are used in computing the smooth quantites
  /// returned by execute()
  ///
  /// \param pos
  /// \param vel
  /// \param acc
  /// \return
  double setReference(double pos, double vel, double acc);

  std::tuple<double, double, double> reference() const;

private:
  Impl* d_ptr();
  Impl const* d_ptr() const;

  std::unique_ptr<Impl> d_ptr_;
};
/// \brief Computes a trajectory from the current position and velocity to goal, without exceeding max_velocity.
///
/// Velocities are filtered with time constant Tau
///
/// From reference.cpp in rov.  Here's the original modification history:
///
/// ======================================================================
///
///   function goal_trajectory
///   Computes a trajectory from the current position and velocity
///   to goal, without exeeding max_velocity.  Velocities are
///   filtered with the time constant tau
///
/// Modification History:
///   DATE         AUTHOR  COMMENT
///   ???          DRY     Created and written
///   26-JUL-2000  LLW     Ported from DRY's original in J1.
///   14-OCT-2005  SCM     added acceleration term
///   22-OCT-2005  SCM     fixed bugs
///   14-FEB-2018  JISV    Ported to ROS and broke up into
///                        goal_trajectory_button_smoother and goal_trajectory_acceleration
///   ====================================================================== */
///
/// \param goal The goal to shoot for
/// \param max_velocity  The max velocity to use
/// \param position  The current position for the value we're moving
/// \param velocity The current velocity
/// \param tau The smoothing timecosntant parameter
/// \param dt The timestep
/// \param smooth_ref The smoother type.  1 for "button smoother", 2 for "goal_trajectory_acceleration", anything else
/// for "none"
void goal_trajectory(double goal, double max_velocity, double& position, double& velocity, double tau, double dt,
                     int smooth_ref);

/// \brief Smooths a reference (velocity + position) using the simple legacy max-velocity smoother
///
/// \param goal  The goal to shoot for
/// \param max_velocity  The max velocity to head that way
/// \param position  The current reference (part of reference state)
/// \param velocity  The derivative of the current reference (part of reference state)
/// \param tau The smoothing time constant
/// \param dt  The timestep to advance
void goal_trajectory_button_smoother(double goal, double max_velocity, double& position, double& velocity, double tau,
                                     double dt);

/// \brief Smooths a reference (velocity + position) using the possibly-broken constant acceleration smoother
///
/// \param goal  The goal to shoot for
/// \param max_velocity  The max velocity to head that way
/// \param position  The current reference (part of reference state)
/// \param velocity  The derivative of the current reference (part of reference state)
/// \param tau The smoothing time constant
/// \param dt  The timestep to advance
void goal_trajectory_acceleration(double goal, double max_velocity, double& position, double& velocity, double tau,
                                  double dt);
}

#endif  // DS_UTIL_REFERENCE_SMOOTHING_H
