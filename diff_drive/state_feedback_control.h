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

#ifndef MOBILITY_DIFF_DRIVE_DIFF_DRIVE_STATE_FEEDBACK_CONTROL_H_
#define MOBILITY_DIFF_DRIVE_DIFF_DRIVE_STATE_FEEDBACK_CONTROL_H_

#include "diff_drive/base_feedback_control_config.pb.h"
#include "diff_drive/dynamic_limits.h"
#include "diff_drive/trajectory_limits.h"
#include "diff_drive/type_aliases.h"

namespace mobility::diff_drive {

// This class implements a state feedback control law that is inspired from
// the polar coordinate controller presented in section 6.3 of:
//
// A. De Luca, G. Oriolo, M. Vendittelli, "Control of Wheeled Mobile Robots:
// An Experimental Overview", Volume 270 of the series Lecture Notes in
// Control and Information Sciences, pp 181-226, 2001.
class StateFeedbackControl {
 public:
  enum class State {
    kInProgress,  // Far from goal.
    kCompleted    // Standing still near goal.
  };

  enum class DirectionChoice { kBestDirection, kForwardOnly, kBackwardOnly };

  // The following seem to work very well in practice:
  //
  // kr = 3.0
  // kd = -1.0
  // kg = 6.0
  // kv = 1.0
  // kw = 1.0
  //
  // Note that stability requires the following conditions, it is up
  // to the caller to make sure these are respected.
  // kr > 0
  // kd < 0
  // kg + kr + 5 * kd * kr / 3 - 2 * kr / pi > 0
  StateFeedbackControl(double kr, double kd, double kg, double kv, double kw,
                       double goal_distance_tolerance,
                       double goal_angle_tolerance);

  explicit StateFeedbackControl(const BaseFeedbackControlConfig& config,
                                double goal_distance_tolerance = 0.02,
                                double goal_angle_tolerance = 0.01)
      : StateFeedbackControl(config.kr(), config.kd(), config.kg(), config.kv(),
                             config.kw(), goal_distance_tolerance,
                             goal_angle_tolerance) {}

  // Computes the next desired velocity with bounds determined by the given
  // trajectory limits.
  // Note: This function is reentrant because this class does not keep any
  // internal state related to the control.
  State ComputeNextVelocity(DirectionChoice direction_choice,
                            double max_linear_acceleration,
                            const eigenmath::Pose2d& current_pose,
                            const ArcVector& current_velocity,
                            const eigenmath::Pose2d& goal,
                            ArcVector* next_velocity) const;

  // Computes the next desired velocity with bounds determined by the given
  // trajectory limits.
  // Note: This function is reentrant because this class does not keep any
  // internal state related to the control.
  State ComputeNextVelocity(DirectionChoice direction_choice,
                            const DynamicLimits& limits, double cycle_time,
                            double desired_speed,
                            const eigenmath::Pose2d& current_pose,
                            const ArcVector& current_velocity,
                            const eigenmath::Pose2d& goal,
                            ArcVector* next_velocity) const;

  // Computes the next desired velocity with bounds determined by the given
  // dynamic limits filter.
  // Note: This function is reentrant because this class does not keep any
  // internal state related to the control.
  State ComputeNextVelocity(DirectionChoice direction_choice,
                            const DynamicLimitsFilter& filter,
                            const eigenmath::Pose2d& current_pose,
                            const ArcVector& current_velocity,
                            const eigenmath::Pose2d& goal,
                            ArcVector* next_velocity) const;

  double GetKr() const { return kr_; }
  double GetKd() const { return kd_; }
  double GetKg() const { return kg_; }
  double GetKv() const { return kv_; }
  double GetKw() const { return kw_; }

  double GetGoalDistanceTolerance() const { return goal_distance_tolerance_; }
  void SetGoalDistanceTolerance(double tolerance) {
    goal_distance_tolerance_ = tolerance;
  }

  double GetGoalAngleTolerance() const { return goal_angle_tolerance_; }
  void SetGoalAngleTolerance(double tolerance) {
    goal_angle_tolerance_ = tolerance;
  }

 private:
  void ApplyLimitsToVelocity(const DynamicLimits& limits, double cycle_time,
                             double desired_speed,
                             const ArcVector& current_velocity,
                             const ArcVector& desired_velocity,
                             ArcVector* next_velocity) const;

  void ChoosePoseError(DirectionChoice direction_choice,
                       const eigenmath::Pose2d& current_pose,
                       const eigenmath::Pose2d& goal,
                       eigenmath::Pose2d* vehicle_pose_target,
                       double* next_velocity_sign) const;

  // Note that stability requires kr_ > 0, kd_ < 0, and kg_ > 0.
  double kr_ = 3.0;
  double kd_ = -1.5;
  double kg_ = 8.0;
  double kv_ = 3.0;
  double kw_ = 8.0;
  double goal_distance_tolerance_ = 0.02;
  double goal_angle_tolerance_ = 0.01;
};

}  // namespace mobility::diff_drive

#endif  // MOBILITY_DIFF_DRIVE_DIFF_DRIVE_STATE_FEEDBACK_CONTROL_H_
