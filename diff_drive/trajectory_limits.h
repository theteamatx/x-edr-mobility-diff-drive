/*
 * Copyright 2023 Google LLC
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MOBILITY_DIFF_DRIVE_DIFF_DRIVE_TRAJECTORY_LIMITS_H_
#define MOBILITY_DIFF_DRIVE_DIFF_DRIVE_TRAJECTORY_LIMITS_H_

#include "diff_drive/dynamic_limits.h"
#include "diff_drive/trajectory.h"
#include "diff_drive/type_aliases.h"

namespace mobility::diff_drive {

// This class represents the limits that constrain the construction of
// discretized trajectories.
// Mainly, it stores the maximum allowable jump in velocities from one
// constant-velocity section to another, given the maximum allowed
// accelerations and the cycle time of the controller.
class TrajectoryLimits {
 public:
  TrajectoryLimits()
      : max_wheel_velocity_jump_(0.0, 0.0),
        max_arc_velocity_jump_(0.0, 0.0),
        min_cycle_duration_(0.0) {}

  TrajectoryLimits(const WheelVector &max_wheel_velocity_jump,
                   const ArcVector &max_arc_velocity_jump,
                   double min_cycle_duration)
      : max_wheel_velocity_jump_(max_wheel_velocity_jump),
        max_arc_velocity_jump_(max_arc_velocity_jump),
        min_cycle_duration_(min_cycle_duration) {}

  // Creates a trajectory limits object from the given dynamic limits and
  // the cycle duration of the controller (sample-time).
  TrajectoryLimits(const DynamicLimits &dd_limits, double min_cycle_duration);

  // Computes the suitable time steps and velocity jumps so that a transition
  // from a given start velocity to a given end velocity within a given total
  // duration of travel can be done while respecting the trajectory limits.
  // The aggressivity_factor controls the desired relaxation of the motion
  // if time allows, i.e., if the total duration is longer than the time
  // required to meet the end velocity with maximum acceleration, then the
  // acceleration will be reduced, but no more than aggressivity_factor from
  // the maximum acceleration.
  void GetSuitableVelocityIncrements(
      const Kinematics &kinematics, const WheelVector &start_wheel_velocity,
      const ArcVector &start_arc_velocity,
      const WheelVector &end_wheel_velocity, const ArcVector &end_arc_velocity,
      double total_duration, double aggressivity_factor,
      double *suitable_time_step, WheelVector *suitable_wheel_vel_inc,
      ArcVector *suitable_arc_vel_inc) const;

  // Computes the linear distance required to stop from the given initial
  // translational velocity and respecting a given aggressivity factor.
  // The function also computes a suitable time-step and velocity increment
  // that can be used to create this stopping trajectory as discrete velocity
  // steps.
  double GetLinearBrakingDistance(double start_velocity,
                                  double aggressivity_factor,
                                  double *suitable_time_step,
                                  double *suitable_vel_inc) const;

  // Computes the linear displacement that would be travelled given a
  // trapezoidal ramp of equally spaced velocity increments.
  // Parameters steps_first and steps_second count the number of steps for the
  // first and second ramps, respectively.
  // Parameters velocity_step_first and velocity_step_second are the amount
  // by which to change the velocity at each time-step on the two ramps.
  // Parameter time_flat denotes the time spent in the peak velocity.
  static double GetLinearBangBangDisplacement(double start_velocity,
                                              double time_step, int steps_first,
                                              double velocity_step_first,
                                              double time_flat,
                                              int steps_second,
                                              double velocity_step_second);

  // Compute parameters for a trapezoidal/triangular ramp of equally spaced
  // velocity increments that travels a specified displacement in nominally
  // minimum time given velocity and acceleration limits.
  // Parameter velocity_step denotes how large a maximum velocity step can be.
  // Parameters steps_first and steps_second count the number of steps for the
  // first and second ramps, respectively.
  // Parameters velocity_step_first and velocity_step_second are the amount
  // by which to change the velocity at each time-step on the two ramps.
  // Parameter time_flat denotes the time spent in the peak velocity.
  static void GetLinearBangBangParameters(
      double delta_in_position, double start_velocity,
      Interval<double> velocity_range, double velocity_step, double time_step,
      int *steps_first, double *velocity_step_first, double *time_flat,
      int *steps_second, double *velocity_step_second);

  // Calculates a velocity ramp to achieve a delta in velocity that obeys a max
  // velocity step.
  void GetLinearVelocityRamp(double delta_in_velocity, double max_velocity_step,
                             int *num_velocity_steps,
                             double *last_velocity_step) const;

  // Computes the minimum time required to stop from the given initial
  // arc and wheel velocities.
  double GetMinimumStoppingTime(const ArcVector &arc_velocity,
                                const WheelVector &wheel_velocity) const;
  double GetMinimumStoppingTime(const Kinematics &kinematics,
                                const ArcVector &arc_velocity) const;
  double GetMinimumStoppingTime(const Kinematics &kinematics,
                                const WheelVector &wheel_velocity) const;

  // Append a trajectory that stops translation velocity and moves the base to a
  // desired rotation.
  //
  // If `long_rotation` is true, turns opposite the shortest way to the desired
  // orientation.  This might be convenient if the short turn is found to be in
  // collision.
  bool AppendBangBangRotationStoppingTrajectory(
      const DynamicLimits &limits, const eigenmath::SO2d &goal_so2,
      double aggressivity_factor, double realignment_aggressivity_factor,
      double time_step, Trajectory *traj, bool long_rotation = false) const;

  // Utility function to append a set of uniform velocity increments after
  // a given current state to a given trajectory until a given desired
  // velocity is reached or a total duration is reached, whichever comes first.
  // Returns true if the desired velocity was reached before the total duration
  // or the trajectory capacity was exhausted.
  bool AppendUniformVelocityTicks(const Kinematics &kinematics,
                                  const WheelVector &desired_wheel_velocity,
                                  const WheelVector &suitable_wheel_vel_inc,
                                  double suitable_time_step,
                                  double total_duration,
                                  StateAndTime *current_state,
                                  Trajectory *trajectory) const;

  // Utility function to append a stopping trajectory after a given current
  // state to a given trajectory until a total duration is reached.
  // Returns true if the trajectory capacity was not exhausted.
  bool AppendStoppingTrajectory(const Kinematics &kinematics,
                                double aggressivity_factor,
                                double total_duration,
                                StateAndTime *current_state,
                                Trajectory *trajectory) const;

  // Bounds the velocity change from current_velocity to
  // desired_velocity such that the given dynamic limits are
  // respected.
  ArcVector ComputeLimitedNextVelocity(const DynamicLimits &limits,
                                       const ArcVector &current_velocity,
                                       const ArcVector &desired_velocity) const;

  // Bounds the velocity change from current_velocity to
  // desired_velocity such that the maximum velocity jumps are respected.
  ArcVector ComputeLimitedNextVelocity(const Kinematics &kinematics,
                                       const ArcVector &current_velocity,
                                       const ArcVector &desired_velocity) const;

  // Get the maximum discrete velocity jump tolerable in wheel-space.
  const WheelVector &GetMaxWheelVelocityJump() const {
    return max_wheel_velocity_jump_;
  }
  // Get the maximum discrete velocity jump tolerable in arc-space.
  const ArcVector &GetMaxArcVelocityJump() const {
    return max_arc_velocity_jump_;
  }

  // Get the maximum discrete velocity jumps.
  BoxConstraints GetVelocityJumpLimits() const {
    return BoxConstraints(max_wheel_velocity_jump_, max_arc_velocity_jump_);
  }

  // Get the maximum acceleration tolerable in wheel-space.
  WheelVector GetMaxWheelAcceleration() const {
    return max_wheel_velocity_jump_ / min_cycle_duration_;
  }
  // Get the maximum acceleration tolerable in arc-space.
  ArcVector GetMaxArcAcceleration() const {
    return max_arc_velocity_jump_ / min_cycle_duration_;
  }

  // Get the maximum acceleration.
  BoxConstraints GetAccelerationLimits() const {
    return BoxConstraints(GetMaxWheelAcceleration(), GetMaxArcAcceleration());
  }

  double GetMinCycleDuration() const { return min_cycle_duration_; }

  void ScaleLimitsBy(double factor) {
    max_wheel_velocity_jump_ *= factor;
    max_arc_velocity_jump_ *= factor;
  }

  void ScaleArcLimitsBy(double trans_factor, double rot_factor) {
    max_arc_velocity_jump_.Translation() *= trans_factor;
    max_arc_velocity_jump_.Rotation() *= rot_factor;
  }

 private:
  WheelVector max_wheel_velocity_jump_;
  ArcVector max_arc_velocity_jump_;
  double min_cycle_duration_;
};

}  // namespace mobility::diff_drive

#endif  // MOBILITY_DIFF_DRIVE_DIFF_DRIVE_TRAJECTORY_LIMITS_H_
