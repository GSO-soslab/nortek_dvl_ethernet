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
#include "ds_util/reference_smoothing.h"
#include <cmath>

namespace ds_util
{
double sgn(double x)
{
  double result;
  result = 1.0;
  if (x < 0.0)
  {
    result = -1.0;
  }
  return (result);
}

std::tuple<double, double, double> goal_trajectory_trapezoidal(double goal, double ref_pos_in, double ref_vel_in,
                                                               double ref_acc_in, double max_vel, double max_acc,
                                                               double dt)
{
  double err, vel_d;
  double ref_pos = ref_pos_in;
  double ref_vel = ref_vel_in;
  double ref_acc = ref_acc_in;

  // Update current reference.
  ref_vel = ref_vel + ref_acc * dt;
  ref_pos = ref_pos + ref_vel * dt + 0.5 * ref_acc * dt * dt;

  // Compute current vector to goal.
  err = ref_pos - goal;

  // Determine acceleration to apply.
  if (fabs(ref_vel) <= max_acc * dt && fabs(err) <= 0.5 * max_acc * dt * dt)
  {
    // At goal.
    ref_acc = 0;
    ref_vel = 0;
    ref_pos = goal;
  }
  else
  {
    // Compute the nominal desired velocity as a function of err.
    if (fabs(err) < 0.5 * max_acc * pow(max_vel / max_acc, 2))  // Decceleration phase.
    {
      vel_d = -sgn(err) * sqrt(2 * fabs(err) / max_acc) * max_acc;
    }
    else
    {
      vel_d = -sgn(err) * max_vel;
    }

    // But only allow accelerations of no more than max_acc.
    ref_acc = sgn(vel_d - ref_vel) * fmin(max_acc, fabs(vel_d - ref_vel) / dt);
  }

  return std::make_tuple(ref_pos, ref_vel, ref_acc);
}

void goal_trajectory(double goal, double max_velocity, double& position, double& velocity, double tau, double dt,
                     int smooth_ref)
{
  if (smooth_ref == 1)
  {
    goal_trajectory_button_smoother(goal, max_velocity, position, velocity, tau, dt);
  }
  else if (smooth_ref == 2)
  {
    // WARNING! This might be broken!
    goal_trajectory_acceleration(goal, max_velocity, position, velocity, tau, dt);
  }
  else
  {
    velocity = 0.0;
    position = goal;
  }
}

void goal_trajectory_button_smoother(double goal, double max_velocity, double& position, double& velocity, double tau,
                                     double dt)
{
  double dx;
  double last_position;
  double last_velocity;
  double alpha;
  double beta;

  alpha = exp(-dt / tau);
  beta = 1.0 - alpha;
  last_position = position;
  last_velocity = velocity;

  dx = goal - position;
  // find teh sign of dx
  // ----------------------------------------------------------------------
  //   25-MAY-2001 LLW&DAS  Hacked button auto to do step moves
  //                        #define SMOOTH_BUTTON_TRAJECTORY 1 to have smooth buttom moves
  // ----------------------------------------------------------------------

  velocity = alpha * last_velocity + beta * max_velocity * sgn(dx);

  if (fabs(dx / tau) > max_velocity)
  {
    position = goal - dx + velocity * dt;
  }
  else
  {
    position = alpha * position + beta * goal;
    dx = position - last_position;
    velocity = dx / dt;
  }
}

void goal_trajectory_acceleration(double goal, double max_velocity, double& position, double& velocity, double tau,
                                  double dt)
{
  double dx;
  double last_velocity;
  double acceleration_const;
  double acceleration;
  double tau_2;
  acceleration_const = 0.005;

  last_velocity = velocity;
  dx = goal - position;

  // define the acceleration of the vehicle based upon
  // the position from from the goal
  if (dx > 0)
  {
    acceleration = acceleration_const;
  }
  else if (dx < 0)
  {
    acceleration = -acceleration_const;
  }

  // now find change the current velocity based upon that calc

  velocity = last_velocity + acceleration * dt;
  // is that velocity greater then the max velocity if so then...
  if (fabs(velocity) > fabs(max_velocity))
  {
    velocity = max_velocity;
    acceleration = (max_velocity - last_velocity) / dt;
  }

  // WHAT IS THE MIN AMOUNT OF TIME TO SLOW DOWN
  tau_2 = velocity / acceleration_const;

  // CHECK TO SEE IF THE THAT IS GREATER THEN THE TIME IT TAKES TO THE
  // GOAL... if so then....
  if (fabs(dx / tau_2) > last_velocity)
  {
    position = goal - dx + last_velocity * dt + 0.5 * acceleration * dt * dt;
  }
  else  // if not ...
  {
    // RECHECK sign of dx and define accel constant
    if (dx > 0)
    {
      acceleration = -acceleration_const;
    }
    else if (dx < 0)
    {
      acceleration = acceleration_const;
    }

    // slow down hopefully get zero velocity at goal.
    velocity = last_velocity - acceleration * dt;
    // define desired position
    position = goal;

    // this is wrong...
    if (fabs(last_velocity) < fabs(acceleration * dt) || (velocity <= 0.0))
    {
      velocity = 0.0;
      position = goal;
    }
  }
}

struct TrapezoidalSmoother::Impl
{
  Impl() : max_vel_(0), max_acc_(0), ref_pos_(0), ref_vel_(0), ref_acc_(0)
  {
  }

  double max_vel_;
  double max_acc_;
  double ref_pos_;
  double ref_vel_;
  double ref_acc_;
};

TrapezoidalSmoother::TrapezoidalSmoother()
  : d_ptr_(std::unique_ptr<TrapezoidalSmoother::Impl>(new TrapezoidalSmoother::Impl))
{
}

TrapezoidalSmoother::~TrapezoidalSmoother() = default;

void TrapezoidalSmoother::setMaxVelocity(double max_vel)
{
  auto d = d_ptr();
  d->max_vel_ = std::abs(max_vel);
}
double TrapezoidalSmoother::maxVelocity() const noexcept
{
  const auto d = d_ptr();
  return d->max_vel_;
}

void TrapezoidalSmoother::setMaxAcceleration(double max_acc)
{
  auto d = d_ptr();
  d->max_acc_ = max_acc;
}
double TrapezoidalSmoother::maxAcceleration() const noexcept
{
  const auto d = d_ptr();
  return d->max_acc_;
}

std::tuple<double, double, double> TrapezoidalSmoother::execute(double goal_pos, double ref_pos, double ref_vel,
                                                                double ref_acc, double dt)
{
  setReference(ref_pos, ref_vel, ref_acc);
  return execute(goal_pos, dt);
}

void TrapezoidalSmoother::reset()
{
  setReference(0, 0, 0);
}

double TrapezoidalSmoother::setReference(double pos, double vel, double acc)
{
  auto d = d_ptr();
  d->ref_pos_ = pos;
  d->ref_vel_ = vel;
  d->ref_acc_ = acc;
}
std::tuple<double, double, double> TrapezoidalSmoother::reference() const
{
  const auto d = d_ptr();
  return std::make_tuple(d->ref_pos_, d->ref_vel_, d->ref_acc_);
}
std::tuple<double, double, double> TrapezoidalSmoother::execute(double goal_pos, double dt)
{
  auto d = d_ptr();

  auto pos = 0.0;
  auto vel = 0.0;
  auto acc = 0.0;
  std::tie(pos, vel, acc) =
      goal_trajectory_trapezoidal(goal_pos, d->ref_pos_, d->ref_vel_, d->ref_acc_, d->max_vel_, d->max_vel_, dt);

  d->ref_pos_ = pos;
  d->ref_vel_ = vel;
  d->ref_acc_ = acc;

  return std::make_tuple(pos, vel, acc);
}
inline TrapezoidalSmoother::Impl* TrapezoidalSmoother::d_ptr()
{
  return d_ptr_.get();
}

inline TrapezoidalSmoother::Impl const* TrapezoidalSmoother::d_ptr() const
{
  return static_cast<TrapezoidalSmoother::Impl const*>(d_ptr_.get());
}
}
