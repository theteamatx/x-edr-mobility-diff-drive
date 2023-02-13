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

#include "diff_drive/state_tracking_control.h"

#include <algorithm>
#include <cmath>
#include <optional>

#include "diff_drive/trajectory_limits.h"
#include "eigenmath/scalar_utils.h"

namespace mobility::diff_drive {
namespace {
double AddWithoutSignFlip(double a, double b) {
  if ((a + b) * a < 0.0) {
    return 0.0;
  } else {
    return a + b;
  }
}
}  // namespace

StateTrackingControl::StateTrackingControl(double damping_factor, double gain,
                                           double kr, double kg, double kv,
                                           double kw)
    : damping_factor_(damping_factor),
      gain_(gain),
      kr_(kr),
      kg_(kg),
      kv_(kv),
      kw_(kw) {
  CHECK_GT(damping_factor_, 0) << "Damping factor cannot be negative!";
  CHECK_LT(damping_factor_, 1) << "Damping factor cannot be greater than 1.0!";
  CHECK_GT(gain_, 0) << "Gain must be positive!";
  CHECK_GT(kr_, 0) << "Kr must be positive!";
  CHECK_GT(kg_, 0) << "Kg must be positive!";
  CHECK_GT(kv_, 0) << "Kv must be positive!";
  CHECK_GT(kw_, 0) << "Kw must be positive!";
}

void StateTrackingControl::ComputeNextVelocity(double distance_tolerance,
                                               const State& current_state,
                                               const State& desired_state,
                                               ArcVector* next_velocity) const {
  const eigenmath::Pose2d vehicle_pose_target =
      current_state.GetPose().inverse() * desired_state.GetPose();
  const eigenmath::Vector2d delta_pos = vehicle_pose_target.translation();
  const double rho = delta_pos.norm();

  const double e1 = delta_pos.x();
  const double e2 = delta_pos.y();
  const double e3 = vehicle_pose_target.angle();

  const double v = current_state.GetArcVelocity().Translation();
  const double w = current_state.GetArcVelocity().Rotation();

  const double vd = desired_state.GetArcVelocity().Translation();
  const double wd = desired_state.GetArcVelocity().Rotation();

  // Gamma is the angle from robot's X-axis to the goal point.
  double gamma;
  if (rho < distance_tolerance) {
    // Don't do realignment if there is no significant lateral error
    gamma = vehicle_pose_target.angle();
  } else {
    const double time_constant = 1.0 / kg_;
    double leading_x = time_constant * vd;
    if (std::abs(leading_x) < std::copysign(delta_pos.x(), vd)) {
      leading_x = delta_pos.x();
    }
    gamma = eigenmath::WrapAngle(std::atan2(delta_pos.y(), leading_x) +
                                 (leading_x < 0.0 ? M_PI : 0.0));
  }

  // as vd, wd goes to zero, kr and kg will take over to correct the pose.
  const double k1 =
      2.0 * damping_factor_ *
      std::sqrt(eigenmath::Square(wd) + gain_ * eigenmath::Square(vd));

  // Apply damping without flipping the signs.
  next_velocity->Translation() = AddWithoutSignFlip(
      vd * std::cos(e3) + k1 * e1 + std::max(0.0, kr_ - k1) * e1,
      kv_ * (vd - v));
  next_velocity->Rotation() = AddWithoutSignFlip(
      wd + gain_ * vd * eigenmath::Sinc(e3) * e2 + k1 * e3 + kg_ * gamma,
      kw_ * (wd - w));
}

void StateTrackingControl::ComputeNextVelocity(
    double distance_tolerance, const DynamicLimits& limits,
    const TrajectoryLimits& traj_limits, double cycle_time,
    const State& current_state, const State& desired_state,
    ArcVector* next_velocity) const {
  ArcVector unlimited_next_vel;
  ComputeNextVelocity(distance_tolerance, current_state, desired_state,
                      &unlimited_next_vel);
  ArcVector limited_next_velocity;
  limits.VelocityLimits().BringInBounds(
      limits.GetKinematics(), unlimited_next_vel, &limited_next_velocity);
  *next_velocity = traj_limits.ComputeLimitedNextVelocity(
      limits, current_state.GetArcVelocity(), limited_next_velocity);
}

std::optional<ArcVector> StateTrackingControl::ComputeNextVelocity(
    double distance_tolerance, double angle_tolerance,
    const DynamicLimits& limits, const TrajectoryLimits& traj_limits,
    double cycle_time, const State& current_state, const Trajectory& trajectory,
    double relative_time) const {
  // Ensure we have zero velocity if we reach the end of the trajectory:
  State desired_state;
  if (relative_time <= trajectory.GetTimeSpan().max()) {
    desired_state = trajectory.Evaluate(relative_time);
  } else {
    desired_state = trajectory.GetFinish().state;
    desired_state.SetArcVelocity(mobility::ArcVector(0.0, 0.0));
  }

  const eigenmath::Pose2d pose_diff =
      current_state.GetPose().inverse() * desired_state.GetPose();

  bool arrived_at_end =
      (relative_time > trajectory.GetTimeSpan().max()) &&
      (std::abs(pose_diff.translation().x()) < distance_tolerance) &&
      (std::abs(pose_diff.angle()) < angle_tolerance);

  if (arrived_at_end) {
    return std::nullopt;
  }
  mobility::ArcVector next_velocity;
  ComputeNextVelocity(distance_tolerance, limits, traj_limits, cycle_time,
                      current_state, desired_state, &next_velocity);
  return next_velocity;
}

bool StateTrackingControl::ComputeResultingTrajectory(
    double distance_tolerance, double angle_tolerance,
    const DynamicLimits& limits, const TrajectoryLimits& traj_limits,
    double cycle_time, State current_state, double current_time, int num_cycles,
    const Trajectory& reference_trajectory,
    Trajectory* resulting_trajectory) const {
  resulting_trajectory->Clear();
  if (!resulting_trajectory->AddState(current_time, current_state)) {
    return false;
  }
  for (int i = 0; i < num_cycles; ++i) {
    current_state = current_state.ExtrapolateConstantVelocityArc(cycle_time);
    current_time += cycle_time;
    std::optional<ArcVector> next_velocity = ComputeNextVelocity(
        distance_tolerance, angle_tolerance, limits, traj_limits, cycle_time,
        current_state, reference_trajectory, current_time);
    current_state.SetArcVelocity(
        next_velocity == std::nullopt ? ArcVector{0.0, 0.0} : *next_velocity);
    if (!resulting_trajectory->AddState(current_time, current_state)) {
      return false;
    }
  }
  return true;
}

}  // namespace mobility::diff_drive
