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

#ifndef MOBILITY_DIFF_DRIVE_DIFF_DRIVE_PATH_TRACKER_H_
#define MOBILITY_DIFF_DRIVE_DIFF_DRIVE_PATH_TRACKER_H_

#include "absl/types/span.h"
#include "diff_drive/base_feedback_control_config.pb.h"
#include "diff_drive/curve.h"
#include "diff_drive/dynamic_limits.h"
#include "diff_drive/state_tracking_control.h"
#include "diff_drive/type_aliases.h"

namespace mobility::diff_drive {

// This class uses the state tracking controller to track a constant velocity
// trajectory generated from a path that it is given (which will be internally
// smoothed).
// It's main purpose is to produce motions that smoothly follow a given
// path at a given desired speed and eventually comes to a stop at the final
// path point.
//
// A few noteworthy characteristics are:
//  - The objective of the path tracker is to closely track the path (anywhere
//    along it). It does not attempt to strictly reach every intermediate
//    waypoint of the path. This means that if given a self-intersecting path,
//    the resulting motion will probably cut through or otherwise behave oddly
//    (e.g., making a beeline for the final point).
//  - The path follower is stateless in the sense that, even though it stores
//    the path and a corresponding trajectory, it does not store state between
//    consecutive invocations. It synchronizes the time frame of the trajectory
//    at the moment it receives the path and then uses current time to get the
//    current state to track.
//  - When a new path is received, the path tracker will attempt to find the
//    point on the new path that best matches the point currently being tracked
//    on the previous trajectory and synchronizes the new trajectory's time
//    values such that that point will be the new tracked point. However, if
//    the difference between the new tracked point and the old one would be too
//    great, then the new trajectory will be time-synchronized to the current
//    location of the robot.
class PathTracker {
 public:
  enum class Status {
    kNoPath,
    kFollowingPath,
    kReachedGoal,
  };

  struct Result {
    Status status;
    ArcVector velocity_command;
  };

  // The number of path points is the capacity for input points.
  // The cycle_time needs to match the rate at which you're calling
  // ComputeNextVelocity().
  // The accel_aggressivity determines how aggressive the stopping rate will be.
  // The trajectory reset threshold is a distance threshold beyond which
  // the tracked point on a trajectory will be reset to the location closest
  // to the current state of the robot.
  // The goal distance and angle tolerances will determine when the end of the
  // path has been reached.
  // The damping factor and gain are the parameters for the controller
  // (see StateTrackingControl).
  // And (kr, kg, kv, kw) parameters are analogous to the parameters for the
  // StateFeedbackControl, but they can be set weaker as their main purpose
  // is to prevent the robot from lagging behind on the trajectory and to make
  // the final adjustments to reach the goal pose.
  PathTracker(int max_num_path_points, double cycle_time,
              double accel_aggressivity, double traj_reset_threshold,
              double max_distance_to_new_path, double goal_distance_tolerance,
              double goal_angle_tolerance, double damping_factor, double gain,
              double kr, double kg, double kv, double kw);

  PathTracker(int max_num_path_points, double cycle_time,
              double accel_aggressivity, double traj_reset_threshold,
              double max_distance_to_new_path,
              const BaseFeedbackControlConfig& feedback_config,
              double goal_distance_tolerance = 0.02,
              double goal_angle_tolerance = 0.01)
      : PathTracker(max_num_path_points, cycle_time, accel_aggressivity,
                    traj_reset_threshold, max_distance_to_new_path,
                    goal_distance_tolerance, goal_angle_tolerance,
                    feedback_config.damping_factor(), feedback_config.gain(),
                    feedback_config.kr(), feedback_config.kg(),
                    feedback_config.kv(), feedback_config.kw()) {}

  // Sets a new path to follow.
  // If the point being tracked on the current trajectory is too far from the
  // new path, then the time synchronization on the new trajectory will be such
  // that the point closest to the current position of the robot will be
  // tracked. Otherwise, the point closest to the point being currently tracked
  // will be used as the time synchronization point on the new trajectory.
  bool SetPath(const DynamicLimits& limits,
               const eigenmath::Pose2d& reference_pose_input,
               absl::Span<const eigenmath::Vector2d> pts, double final_angle,
               double desired_velocity, double current_time,
               const eigenmath::Pose2d& current_pose);

  // Clears the currently stored trajectory.
  // Call this before SetPath to guarantee a hard reset for the new trajectory.
  void Clear();

  // Computes the next velocity to be commanded in order to track the path,
  // given the current time and state of the robot.
  Result ComputeNextVelocity(const DynamicLimits& limits, double current_time,
                             const State& current_state);

  bool IsAtGoal() const { return Status::kReachedGoal == status_; }

  const char* GetErrorString() const { return error_string_; }

  const diff_drive::Curve& GetCurve() const { return smooth_curve_; }

  const diff_drive::Trajectory& GetTrajectory() const {
    return ref_trajectory_;
  }

  bool GetDesiredStateForState(double current_time, const State& current_state,
                               State* tracked_state,
                               State* relative_error) const;

  bool GetDeviationForPose(double current_time, const eigenmath::Pose2d& pose,
                           double* error_linear, double* error_angular,
                           double* time) const;

  bool GetDeviationForPose(double current_time, const eigenmath::Pose3d& pose,
                           double* error_linear, double* error_angular,
                           double* time) const;

  bool GetDistanceToFinal(const eigenmath::Pose2d& pose, double* dist_linear,
                          double* dist_angular) const;

  bool GetDistanceToFinal(const eigenmath::Pose3d& pose, double* dist_linear,
                          double* dist_angular) const;

  double GetGoalDistanceTolerance() const { return goal_distance_tolerance_; }
  void SetGoalDistanceTolerance(double value) {
    goal_distance_tolerance_ = value;
  }
  double GetGoalAngleTolerance() const { return goal_angle_tolerance_; }
  void SetGoalAngleTolerance(double value) { goal_angle_tolerance_ = value; }

 private:
  Status status_ = Status::kNoPath;
  const char* error_string_;
  diff_drive::Curve input_curve_;
  diff_drive::Curve smooth_curve_;
  diff_drive::Trajectory scratch_trajectory_;
  diff_drive::Trajectory ref_trajectory_;
  double absolute_time_to_traj_time_;

  StateTrackingControl tracking_ctrl_;

  double acceleration_aggressivity_ = 0.5;
  double trajectory_reset_threshold_ = 0.5;
  double max_distance_to_new_path_ = 2.0;
  double goal_distance_tolerance_ = 0.02;
  double goal_angle_tolerance_ = 0.01;
  double cycle_time_ = 0.01;
};

}  // namespace mobility::diff_drive

#endif  // MOBILITY_DIFF_DRIVE_DIFF_DRIVE_PATH_TRACKER_H_
