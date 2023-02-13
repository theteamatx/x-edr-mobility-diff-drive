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

#ifndef MOBILITY_DIFF_DRIVE_DIFF_DRIVE_STATE_TRACKING_CONTROL_H_
#define MOBILITY_DIFF_DRIVE_DIFF_DRIVE_STATE_TRACKING_CONTROL_H_

#include <optional>

#include "diff_drive/base_feedback_control_config.pb.h"
#include "diff_drive/dynamic_limits.h"
#include "diff_drive/trajectory.h"
#include "diff_drive/trajectory_limits.h"
#include "diff_drive/type_aliases.h"

namespace mobility::diff_drive {

// This class implements the feedback law that is appropriate for
// trajectory tracking based on the nonlinear control design presented
// in section 5.3 of:
// A. De Luca, G. Oriolo, M. Vendittelli, "Control of Wheeled Mobile Robots:
// An Experimental Overview", Volume 270 of the series Lecture Notes in
// Control and Information Sciences, pp 181-226, 2001.
class StateTrackingControl {
 public:
  // The following seem to work very well in practice:
  //
  // damping_factor = 0.7
  // gain = 10
  //
  // Note that stability requires the following conditions, it is up
  // to the caller to make sure these are respected.
  // 0 < damping_factor < 1
  // gain > 0
  StateTrackingControl(double damping_factor, double gain, double kr, double kg,
                       double kv, double kw);

  StateTrackingControl() = default;

  explicit StateTrackingControl(const BaseFeedbackControlConfig& config)
      : StateTrackingControl(config.damping_factor(), config.gain(),
                             config.kr(), config.kg(), config.kv(),
                             config.kw()) {}

  // Computes the next desired velocity with bounds determined by the given
  // trajectory limits.
  // Note: This function is reentrant because this class does not keep any
  // internal state related to the control.
  void ComputeNextVelocity(double distance_tolerance,
                           const State& current_state,
                           const State& desired_state,
                           ArcVector* next_velocity) const;

  // Computes the next desired velocity with bounds determined by the given
  // trajectory limits.
  // Note: This function is reentrant because this class does not keep any
  // internal state related to the control.
  void ComputeNextVelocity(double distance_tolerance,
                           const DynamicLimits& limits,
                           const TrajectoryLimits& traj_limits,
                           double cycle_time, const State& current_state,
                           const State& desired_state,
                           ArcVector* next_velocity) const;

  // Returns the next desired velocity with bounds determined by the given
  // trajectory limits.
  // Returns std::nullopt if the final state on the trajectory was reached.
  // Note: This function is reentrant because this class does not keep any
  // internal state related to the control.
  std::optional<ArcVector> ComputeNextVelocity(
      double distance_tolerance, double angle_tolerance,
      const DynamicLimits& limits, const TrajectoryLimits& traj_limits,
      double cycle_time, const State& current_state,
      const Trajectory& trajectory, double relative_time) const;
  std::optional<ArcVector> ComputeNextVelocity(double distance_tolerance,
                                               double angle_tolerance,
                                               const DynamicLimits& limits,
                                               double cycle_time,
                                               const State& current_state,
                                               const Trajectory& trajectory,
                                               double relative_time) const {
    return ComputeNextVelocity(distance_tolerance, angle_tolerance, limits,
                               TrajectoryLimits(limits, cycle_time), cycle_time,
                               current_state, trajectory, relative_time);
  }

  // Returns the trajectory that would result from rolling out this control
  // law at the given `cycle_time` for a given `num_cycles` from a starting
  // state and time.
  // Returns std::nullopt if the final state on the trajectory was reached.
  // Note: This function is reentrant because this class does not keep any
  // internal state related to the control.
  bool ComputeResultingTrajectory(double distance_tolerance,
                                  double angle_tolerance,
                                  const DynamicLimits& limits,
                                  const TrajectoryLimits& traj_limits,
                                  double cycle_time, State current_state,
                                  double current_time, int num_cycles,
                                  const Trajectory& reference_trajectory,
                                  Trajectory* resulting_trajectory) const;

  double GetDampingFactor() const { return damping_factor_; }
  double GetGain() const { return gain_; }

 private:
  double damping_factor_ = 0.7;
  double gain_ = 10.0;
  double kr_ = 1.0;
  double kg_ = 2.0;
  double kv_ = 0.25;
  double kw_ = 0.5;
};

}  // namespace mobility::diff_drive

#endif  // MOBILITY_DIFF_DRIVE_DIFF_DRIVE_STATE_TRACKING_CONTROL_H_
