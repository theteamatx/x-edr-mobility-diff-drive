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

#include "diff_drive/state_feedback_control.h"

#include <algorithm>
#include <cmath>

#include "eigenmath/scalar_utils.h"

namespace mobility::diff_drive {

namespace {
// Find a more meaningful source (config / calib / status)
// for the values below, which are effectively dependent on the noise expected
// on ego-motion estimation.
constexpr double kZeroSpeedEpsilon = 0.001;
constexpr double kNoiseToToleranceFraction = 0.25;
constexpr double kPositionDriftPerRadians = 0.0015;
}  // namespace

StateFeedbackControl::StateFeedbackControl(double kr, double kd, double kg,
                                           double kv, double kw,
                                           double goal_distance_tolerance,
                                           double goal_angle_tolerance)
    : kr_(kr),
      kd_(kd),
      kg_(kg),
      kv_(kv),
      kw_(kw),
      goal_distance_tolerance_(goal_distance_tolerance),
      goal_angle_tolerance_(goal_angle_tolerance) {}

void StateFeedbackControl::ChoosePoseError(
    DirectionChoice direction_choice, const eigenmath::Pose2d& current_pose,
    const eigenmath::Pose2d& goal, eigenmath::Pose2d* vehicle_pose_target,
    double* next_velocity_sign) const {
  if (direction_choice == DirectionChoice::kForwardOnly) {
    *vehicle_pose_target = current_pose.inverse() * goal;
    *next_velocity_sign = 1.0;
  } else if (direction_choice == DirectionChoice::kBackwardOnly) {
    const eigenmath::Pose2d reverse_pose(current_pose.translation(),
                                         current_pose.angle() + M_PI);
    const eigenmath::Pose2d reverse_goal(goal.translation(),
                                         goal.angle() + M_PI);

    *vehicle_pose_target = reverse_pose.inverse() * reverse_goal;
    *next_velocity_sign = -1.0;
  } else {
    const eigenmath::Pose2d fwd_error = current_pose.inverse() * goal;

    const eigenmath::Pose2d reverse_pose(current_pose.translation(),
                                         current_pose.angle() + M_PI);
    const eigenmath::Pose2d reverse_goal(goal.translation(),
                                         goal.angle() + M_PI);
    const eigenmath::Pose2d bwd_error = reverse_pose.inverse() * reverse_goal;
    const auto heading_adjustment = [](const eigenmath::Pose2d& pose) {
      return std::abs(
          std::atan2(pose.translation().y(), pose.translation().x()));
    };

    // Choose based on least amount of turning.
    if (((std::abs(fwd_error.translation().y()) < kZeroSpeedEpsilon) &&
         (fwd_error.translation().x() >= bwd_error.translation().x())) ||
        (heading_adjustment(fwd_error) <= heading_adjustment(bwd_error))) {
      *vehicle_pose_target = fwd_error;
      *next_velocity_sign = 1.0;
    } else {
      *vehicle_pose_target = bwd_error;
      *next_velocity_sign = -1.0;
    }
  }
}

StateFeedbackControl::State StateFeedbackControl::ComputeNextVelocity(
    DirectionChoice direction_choice, double max_linear_acceleration,
    const eigenmath::Pose2d& current_pose, const ArcVector& current_velocity,
    const eigenmath::Pose2d& goal, ArcVector* next_velocity) const {
  eigenmath::Pose2d vehicle_pose_target;
  double next_velocity_sign = 1.0;
  ChoosePoseError(direction_choice, current_pose, goal, &vehicle_pose_target,
                  &next_velocity_sign);

  eigenmath::Vector2d delta_pos = vehicle_pose_target.translation();
  double rho = delta_pos.norm();

  // Gamma is the angle from robot's X-axis to the goal point.
  // Delta is the angle from the line robot-to-goal to the goal heading.
  double delta = 0.0;
  double gamma = std::atan2(delta_pos.y(), delta_pos.x());
  if (goal_angle_tolerance_ < M_PI) {
    delta = eigenmath::WrapAngle(vehicle_pose_target.angle() - gamma);
  }

  const double current_speed = std::abs(current_velocity.Translation());
  const double dp_stop =
      current_speed * current_speed * 0.5 / max_linear_acceleration;
  const bool is_stopping_in_inner_goal =
      (dp_stop + rho <
       (1.0 - kNoiseToToleranceFraction) * goal_distance_tolerance_);
  const double drift_from_angle_adjustment =
      kPositionDriftPerRadians * std::abs(vehicle_pose_target.angle());
  const bool is_stopped_in_outer_goal =
      (current_speed < kZeroSpeedEpsilon &&
       drift_from_angle_adjustment + rho < goal_distance_tolerance_);
  if (is_stopping_in_inner_goal || is_stopped_in_outer_goal ||
      !std::isfinite(gamma)) {
    // Don't worry about position error if we are within goal distance
    rho = 0.0;
    delta_pos.x() = 0.0;
    gamma = vehicle_pose_target.angle();
    delta = 0.0;
  }

  if ((rho < goal_distance_tolerance_) &&
      (vehicle_pose_target.so2().norm() < goal_angle_tolerance_)) {
    *next_velocity = ArcVector{0.0, 0.0};
    if ((std::abs(current_velocity.Translation()) < kZeroSpeedEpsilon) &&
        (std::abs(current_velocity.Rotation()) < kZeroSpeedEpsilon)) {
      return State::kCompleted;
    } else {
      return State::kInProgress;
    }
  }

  *next_velocity =
      ArcVector{kr_ * delta_pos.x(),
                kg_ * gamma + kr_ * eigenmath::Sinc(gamma) * std::cos(gamma) *
                                  (gamma + kd_ * delta)};

  next_velocity->Translation() =
      next_velocity_sign * next_velocity->Translation();
  // Do not invert rotational velocity.

  // Apply derivative gains:
  next_velocity->Translation() -= kv_ * current_velocity.Translation();
  next_velocity->Rotation() -= kw_ * current_velocity.Rotation();

  return State::kInProgress;
}

StateFeedbackControl::State StateFeedbackControl::ComputeNextVelocity(
    DirectionChoice direction_choice, const DynamicLimits& limits,
    double cycle_time, double desired_speed,
    const eigenmath::Pose2d& current_pose, const ArcVector& current_velocity,
    const eigenmath::Pose2d& goal, ArcVector* next_velocity) const {
  ArcVector unlimited_des_vel;
  const auto result = ComputeNextVelocity(
      direction_choice,
      std::min(-limits.MinArcAcceleration().Translation(),
               limits.MaxArcAcceleration().Translation()),
      current_pose, current_velocity, goal, &unlimited_des_vel);
  ApplyLimitsToVelocity(limits, cycle_time, desired_speed, current_velocity,
                        unlimited_des_vel, next_velocity);
  return result;
}

StateFeedbackControl::State StateFeedbackControl::ComputeNextVelocity(
    DirectionChoice direction_choice, const DynamicLimitsFilter& limits_filter,
    const eigenmath::Pose2d& current_pose, const ArcVector& current_velocity,
    const eigenmath::Pose2d& goal, ArcVector* next_velocity) const {
  ArcVector unlimited_des_vel;
  const auto result = ComputeNextVelocity(
      direction_choice,
      std::min(-limits_filter.GetLimits().MinArcAcceleration().Translation(),
               limits_filter.GetLimits().MaxArcAcceleration().Translation()),
      current_pose, current_velocity, goal, &unlimited_des_vel);
  if (!limits_filter.BringInBounds(current_velocity, unlimited_des_vel,
                                   next_velocity)) {
    *next_velocity = current_velocity;
  }
  return result;
}

void StateFeedbackControl::ApplyLimitsToVelocity(
    const DynamicLimits& limits, double cycle_time, double desired_speed,
    const ArcVector& current_velocity, const ArcVector& desired_velocity,
    ArcVector* next_velocity) const {
  TrajectoryLimits traj_limits(limits, cycle_time);
  ArcVector limited_desired_velocity;
  limits.VelocityLimits().BringInBounds(
      limits.GetKinematics(), desired_velocity, &limited_desired_velocity);
  limited_desired_velocity.Translation() = eigenmath::Saturate(
      limited_desired_velocity.Translation(), std::abs(desired_speed));
  *next_velocity = traj_limits.ComputeLimitedNextVelocity(
      limits, current_velocity, limited_desired_velocity);
}

}  // namespace mobility::diff_drive
