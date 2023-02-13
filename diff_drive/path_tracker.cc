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

#include "diff_drive/path_tracker.h"

#include <algorithm>
#include <cmath>
#include <iterator>

#include "diff_drive/curve_trajectory_utils.h"
#include "diff_drive/trajectory_limits.h"
#include "eigenmath/pose3_utils.h"

namespace mobility::diff_drive {

namespace {
constexpr double kNearlyAtGoalEpsilon = 1e-4;
}  // namespace

PathTracker::PathTracker(int max_num_path_points, double cycle_time,
                         double accel_aggressivity, double traj_reset_threshold,
                         double max_distance_to_new_path,
                         double goal_distance_tolerance,
                         double goal_angle_tolerance, double damping_factor,
                         double gain, double kr, double kg, double kv,
                         double kw)
    : error_string_("Never got a path"),
      input_curve_(max_num_path_points),
      smooth_curve_(max_num_path_points * 2),
      scratch_trajectory_(max_num_path_points * 4),
      ref_trajectory_(max_num_path_points * 4),
      tracking_ctrl_(damping_factor, gain, kr, kg, kv, kw),
      acceleration_aggressivity_(accel_aggressivity),
      trajectory_reset_threshold_(traj_reset_threshold),
      max_distance_to_new_path_(max_distance_to_new_path),
      goal_distance_tolerance_(goal_distance_tolerance),
      goal_angle_tolerance_(goal_angle_tolerance),
      cycle_time_(cycle_time) {}

void PathTracker::Clear() {
  input_curve_.Clear();
  smooth_curve_.Clear();
  scratch_trajectory_.Clear();
  ref_trajectory_.Clear();
}

bool PathTracker::SetPath(const DynamicLimits& limits,
                          const eigenmath::Pose2d& reference_pose_input,
                          const absl::Span<const eigenmath::Vector2d> pts,
                          double final_angle, double desired_velocity,
                          double current_time,
                          const eigenmath::Pose2d& current_pose) {
  input_curve_.Clear();
  smooth_curve_.Clear();
  if (pts.size() < 2) {
    Clear();
    error_string_ = "Insufficient number of waypoints";
    return false;
  }

  // Create the input curve, as a polyline of the input points:
  auto next_it = std::next(pts.begin());
  eigenmath::Vector2d next_pt = reference_pose_input * (*pts.begin());
  double accum_s = 0.0;
  for (auto current_it = pts.begin(); current_it < pts.end();
       ++current_it, ++next_it) {
    eigenmath::Vector2d cur_pt = next_pt;
    double new_angle =
        eigenmath::WrapAngle(reference_pose_input.angle() + final_angle);
    if (next_it == pts.end()) {
      // treat end point differently
      if ((std::abs(eigenmath::WrapAngle(
               new_angle - input_curve_.GetFinish().point.GetPose().angle())) >
           goal_angle_tolerance_) &&
          (std::abs(eigenmath::WrapAngle(
               new_angle + M_PI -
               input_curve_.GetFinish().point.GetPose().angle())) >
           goal_angle_tolerance_)) {
        Clear();
        error_string_ = "Misalignment of the last two points with final angle";
        return false;
      }
      input_curve_.AddPoint(
          accum_s,
          CurvePoint(eigenmath::Pose2d(
              cur_pt, input_curve_.GetFinish().point.GetPose().angle())));
    } else {
      next_pt = reference_pose_input * (*next_it);
      const eigenmath::Vector2d delta = next_pt - cur_pt;
      const double delta_norm = delta.norm();
      if (delta_norm < Curve::kIdenticalPointsEpsilon) {
        // Skip points that are identical.
        continue;
      }
      new_angle = std::atan2(delta.y(), delta.x());
      input_curve_.AddPoint(accum_s,
                            CurvePoint(eigenmath::Pose2d(cur_pt, new_angle)));
      accum_s += delta_norm;
    }
  }

  // Round-off the polyline into a smooth curve:
  if (!RoundOffCornersOfPolyline(input_curve_, &smooth_curve_)) {
    Clear();
    error_string_ = "Smoothing of input curve failed";
    return false;
  }

  // Find the correct point on the smooth curve that matches either where we
  // were driving to on the previous trajectory, or where we are now:
  CurvePtAndCord matched_curve_pt;
  if (!ref_trajectory_.IsEmpty() && ref_trajectory_.IsSane()) {
    State old_traj_pt =
        ref_trajectory_.Evaluate(current_time - absolute_time_to_traj_time_);
    matched_curve_pt = smooth_curve_.FindClosestMatchToPoint(
        old_traj_pt.GetPose().translation(), nullptr);
    if (((matched_curve_pt.point.GetPose().translation() -
          old_traj_pt.GetPose().translation())
             .squaredNorm() >
         trajectory_reset_threshold_ * trajectory_reset_threshold_) ||
        ((matched_curve_pt.point.GetPose().translation() -
          current_pose.translation())
             .squaredNorm() >
         max_distance_to_new_path_ * max_distance_to_new_path_)) {
      matched_curve_pt = smooth_curve_.FindClosestMatchToPoint(
          current_pose.translation(), nullptr);
    }
  } else {
    matched_curve_pt = smooth_curve_.FindClosestMatchToPoint(
        current_pose.translation(), nullptr);
  }

  // Check that the new path is not too far from our current position:
  if ((matched_curve_pt.point.GetPose().translation() -
       current_pose.translation())
          .norm() > max_distance_to_new_path_) {
    Clear();
    error_string_ = "Path is too far from current pose";
    return false;
  }

  TrajectoryLimits traj_limits(limits, cycle_time_);
  traj_limits.ScaleLimitsBy(acceleration_aggressivity_);

  // Convert the smooth curve into a trajectory that stops at the end:
  double sub_desired_velocity = 0.0;
  if (!FindClothoidTrajectoryForCurve(smooth_curve_, desired_velocity, 1e-3,
                                      limits, traj_limits, &ref_trajectory_,
                                      &sub_desired_velocity)) {
    Clear();
    error_string_ = "Could not find clothoid trajectory for curve";
    return false;
  }

  scratch_trajectory_ = ref_trajectory_;
  if (!FindSlowerTrajectoryToEndWithStop(scratch_trajectory_, 1e-3,
                                         limits.GetKinematics(), traj_limits,
                                         &ref_trajectory_)) {
    Clear();
    error_string_ = "Could not amend trajectory with a stop";
    return false;
  }

  // Re-adjust the timing on the trajectory using the matched point:
  StateAndTime matched_traj_pt = ref_trajectory_.FindClosestMatchToPoint(
      matched_curve_pt.point.GetPose().translation());
  absolute_time_to_traj_time_ = current_time;
  ref_trajectory_.ApplyTimeShift(-matched_traj_pt.time);

  error_string_ = "No error";
  return true;
}

bool PathTracker::GetDesiredStateForState(double current_time,
                                          const State& current_state,
                                          State* tracked_state,
                                          State* relative_error) const {
  if (ref_trajectory_.IsEmpty() || !ref_trajectory_.IsSane()) {
    return false;
  }
  const double relative_time = current_time - absolute_time_to_traj_time_;

  *tracked_state = ref_trajectory_.Evaluate(relative_time);
  *relative_error = tracked_state->GetRelativeTo(current_state);

  return true;
}

bool PathTracker::GetDeviationForPose(double current_time,
                                      const eigenmath::Pose2d& pose,
                                      double* error_linear,
                                      double* error_angular,
                                      double* time) const {
  if (ref_trajectory_.IsEmpty() || !ref_trajectory_.IsSane()) {
    return false;
  }
  const double relative_time = current_time - absolute_time_to_traj_time_;

  const StateAndTime matched_traj_pt =
      ref_trajectory_.FindClosestMatchToPoint(pose.translation());
  const State tracked_state = ref_trajectory_.Evaluate(relative_time);

  *time = matched_traj_pt.time;

  const eigenmath::Pose2d vehicle_pose_target =
      pose.inverse() * tracked_state.GetPose();

  *error_linear = vehicle_pose_target.translation().x();
  *error_angular = vehicle_pose_target.angle();

  return true;
}

bool PathTracker::GetDeviationForPose(double current_time,
                                      const eigenmath::Pose3d& pose,
                                      double* error_linear,
                                      double* error_angular,
                                      double* time) const {
  const eigenmath::Pose2d pose_2d = eigenmath::ToPose2XY(pose);
  return GetDeviationForPose(current_time, pose_2d, error_linear, error_angular,
                             time);
}

bool PathTracker::GetDistanceToFinal(const eigenmath::Pose2d& pose,
                                     double* dist_linear,
                                     double* dist_angular) const {
  if (ref_trajectory_.IsEmpty() || !ref_trajectory_.IsSane()) {
    return false;
  }

  const StateAndTime final_pt = ref_trajectory_.GetFinish();

  const eigenmath::Pose2d vehicle_pose_target =
      pose.inverse() * final_pt.state.GetPose();

  *dist_linear = vehicle_pose_target.translation().norm();
  *dist_angular = std::abs(vehicle_pose_target.angle());

  return true;
}

bool PathTracker::GetDistanceToFinal(const eigenmath::Pose3d& pose,
                                     double* dist_linear,
                                     double* dist_angular) const {
  const eigenmath::Pose2d pose_2d = eigenmath::ToPose2XY(pose);
  return GetDistanceToFinal(pose_2d, dist_linear, dist_angular);
}

PathTracker::Result PathTracker::ComputeNextVelocity(
    const DynamicLimits& limits, double current_time,
    const State& current_state) {
  if (ref_trajectory_.IsEmpty() || !ref_trajectory_.IsSane()) {
    status_ = PathTracker::Status::kNoPath;
    return {status_, {0.0, 0.0}};
  }
  const double relative_time = current_time - absolute_time_to_traj_time_;

  const State tracked_state = ref_trajectory_.Evaluate(relative_time);
  const eigenmath::Pose2d pose_diff =
      current_state.GetPose().inverse() * tracked_state.GetPose();

  ArcVector next_velocity;
  tracking_ctrl_.ComputeNextVelocity(
      goal_distance_tolerance_, limits, TrajectoryLimits(limits, cycle_time_),
      cycle_time_, current_state, tracked_state, &next_velocity);

  status_ = PathTracker::Status::kFollowingPath;
  if (std::abs(tracked_state.GetArcVelocity().Translation() *
               (relative_time - ref_trajectory_.GetTimeSpan().max())) <
      kNearlyAtGoalEpsilon) {
    const ArcVector max_dv =
        limits.AccelerationLimits().MaxArcVector() * cycle_time_;
    if ((std::abs(current_state.GetArcVelocity().Translation()) <
         max_dv.Translation()) &&
        (std::abs(current_state.GetArcVelocity().Rotation()) <
         max_dv.Rotation()) &&
        (std::abs(pose_diff.translation().x()) < goal_distance_tolerance_) &&
        (std::abs(pose_diff.angle()) < goal_angle_tolerance_)) {
      next_velocity.Translation() = 0.0;
      next_velocity.Rotation() = 0.0;
      status_ = PathTracker::Status::kReachedGoal;
    }
  }
  return {status_, next_velocity};
}

}  // namespace mobility::diff_drive
