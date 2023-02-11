// Copyright 2023 Google LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "diff_drive/curve_trajectory_utils.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <iomanip>
#include <iterator>
#include <limits>
#include <ostream>
#include <sstream>
#include <string>

#include "eigenmath/line_search.h"
#include "eigenmath/scalar_utils.h"
#include "eigenmath/so2_interval.h"
#include "genit/adjacent_iterator.h"
#include "diff_drive/curve.h"
#include "diff_drive/trajectory.h"
#include "diff_drive/type_aliases.h"
#include "diff_drive/wheel_curve.h"
#include "absl/log/check.h"

namespace mobility::diff_drive {

namespace {
constexpr double kEpsilon = 1e-4;
constexpr double kSmallEpsilon = 1e-6;
constexpr double kTenNanoSeconds = 1.0e-8;  // 10 nanoseconds

constexpr int kPrintPrecision = 16;

bool AppendConvertedCurveToTrajectory(const Curve& curve,
                                      Interval<double> cord_length_span,
                                      double desired_speed,
                                      bool skip_last_point, double start_time,
                                      const DynamicLimits& limits,
                                      Trajectory* traj, double* end_time) {
  const bool going_backwards = (desired_speed < 0.0);
  const auto& kinematics = limits.GetKinematics();
  const auto& vel_bounds = limits.VelocityLimits();
  double last_cord = cord_length_span.min();
  CurvePoint curve_point = curve.Evaluate(last_cord);
  eigenmath::Pose2d curve_pose = curve_point.GetPose();
  if (going_backwards) {
    curve_pose.setAngle(curve_pose.angle() + M_PI);
  }
  ArcVector unlimited_desired_arc_vel(
      desired_speed, std::abs(desired_speed) * curve_point.GetCurvature());
  ArcVector desired_arc_vel;
  if (!vel_bounds.BringInBounds(kinematics, unlimited_desired_arc_vel,
                                &desired_arc_vel)) {
    return false;
  }
  if (!traj->AddState(start_time, State(curve_pose, desired_arc_vel))) {
    return false;
  }
  for (auto curve_it = curve.BeginCurvePoint();
       curve_it != curve.EndCurvePoint(); ++curve_it) {
    if (curve_it->cord_length <= last_cord) {
      continue;
    }
    if (last_cord + kSmallEpsilon >= cord_length_span.max()) {
      break;
    }
    double dt = (curve_it->cord_length - last_cord) /
                std::abs(desired_arc_vel.Translation());
    if (curve_it->cord_length > cord_length_span.max()) {
      curve_point = curve.Evaluate(cord_length_span.max());
      dt = (cord_length_span.max() - last_cord) /
           std::abs(desired_arc_vel.Translation());
    } else {
      curve_point = curve_it->point;
    }
    curve_pose = curve_point.GetPose();
    if (going_backwards) {
      curve_pose.setAngle(curve_pose.angle() + M_PI);
    }
    unlimited_desired_arc_vel = ArcVector(
        desired_speed, std::abs(desired_speed) * curve_point.GetCurvature());
    if (!vel_bounds.BringInBounds(kinematics, unlimited_desired_arc_vel,
                                  &desired_arc_vel)) {
      return false;
    }
    // check that the curve point matches.
    State arrival_state =
        traj->GetFinish().state.ExtrapolateConstantVelocityArc(dt);
    eigenmath::Pose2d pose_diff =
        arrival_state.GetPose().inverse() * curve_pose;
    // Check for fixable mismatch of traj extrapolation and curve pose:
    if ((std::abs(traj->GetFinish().state.GetArcVelocity().Translation()) >
         kSmallEpsilon) &&
        (std::abs(traj->GetFinish().state.GetArcVelocity().Rotation()) <
         kSmallEpsilon) &&
        (std::abs(pose_diff.translation().x()) > kEpsilon) &&
        (std::abs(pose_diff.translation().y()) < kEpsilon)) {
      dt += pose_diff.translation().x() /
            traj->GetFinish().state.GetArcVelocity().Translation();
      if (dt < 0.0) {
        // Cannot fix this by going back in time!
        return false;
      }
      arrival_state =
          traj->GetFinish().state.ExtrapolateConstantVelocityArc(dt);
      pose_diff = arrival_state.GetPose().inverse() * curve_pose;
    }
    // Check for still too much mismatch of traj extrapolation and curve pose:
    if ((std::abs(pose_diff.translation().x()) > kEpsilon) ||
        (std::abs(pose_diff.translation().y()) > kEpsilon)) {
      return false;
    }
    if (std::abs(pose_diff.angle()) > kEpsilon) {
      // Add a pure spin
      WheelVector desired_wheel_speed = kinematics.ComputeInverseKinematics(
          ArcVector(std::abs(desired_speed), 0.0));
      if (pose_diff.angle() > 0.0) {
        desired_wheel_speed.Left() = -desired_wheel_speed.Left();
      } else {
        desired_wheel_speed.Right() = -desired_wheel_speed.Right();
      }
      const ArcVector unlimited_desired_spin =
          kinematics.ComputeForwardKinematics(desired_wheel_speed);
      ArcVector desired_spin;
      if (!vel_bounds.BringInBounds(kinematics, unlimited_desired_spin,
                                    &desired_spin)) {
        return false;
      }
      const double spin_dt = pose_diff.angle() / desired_spin.Rotation();
      if (!traj->AddState(traj->GetFinish().time + dt,
                          State(arrival_state.GetPose(), desired_spin))) {
        return false;
      }
      dt = spin_dt;
    }
    if ((curve_it->cord_length + kSmallEpsilon < cord_length_span.max()) ||
        !skip_last_point) {
      if (dt > kSmallEpsilon) {
        if (!traj->AddState(traj->GetFinish().time + dt,
                            State(curve_pose, desired_arc_vel))) {
          return false;
        }
      } else {
        auto& mutable_finish = traj->GetMutableFinish();
        mutable_finish.time += dt;
        mutable_finish.state.SetPose(curve_pose);
        mutable_finish.state.SetArcVelocity(desired_arc_vel);
      }
    } else if (end_time != nullptr) {
      *end_time = traj->GetFinish().time + dt;
    }
    last_cord = curve_it->cord_length;
  }
  return true;
}
}  // namespace

bool ConvertTrajectoryToCurve(const Trajectory& traj, Curve* curve) {
  curve->Clear();
  double staged_cord = -1.0;
  CurvePoint staged_pt;
  double staged_speed = 0.0;
  double staged_time = 0.0;
  for (const auto& state : traj.GetStateIteratorRange()) {
    const auto& arc_vel = state.state.GetArcVelocity();
    double new_cord_inc = 0.0;
    if (staged_cord >= 0.0) {
      const double dt = state.time - staged_time;
      const double delta_cord = dt * staged_speed;
      // Only add points that do progress on cord-length, so, in-place spins
      // will just be added on the last point of the spin.
      if (delta_cord > kEpsilon) {
        if (!curve->AddPoint(staged_cord, staged_pt)) {
          return false;
        }
        new_cord_inc = delta_cord;
      }
    } else {
      staged_cord = 0.0;
    }
    staged_cord += new_cord_inc;
    eigenmath::Pose2d traj_pose = state.state.GetPose();
    if (arc_vel.Translation() < 0.0) {
      traj_pose.setAngle(traj_pose.angle() + M_PI);
    }
    staged_speed = std::abs(arc_vel.Translation());
    staged_pt = CurvePoint(
        traj_pose,
        (staged_speed < kEpsilon ? 0.0 : arc_vel.Rotation() / staged_speed));
    staged_time = state.time;
  }
  if (staged_cord < 0.0) {
    return false;
  }
  if (!curve->AddPoint(staged_cord, staged_pt)) {
    return false;
  }
  return true;
}

bool ConvertCurveToTrajectory(const Curve& curve, double desired_speed,
                              const Kinematics& kinematics, Trajectory* traj) {
  return ConvertCurveToTrajectory(curve, curve.GetCordLengthSpan(),
                                  desired_speed,
                                  DynamicLimits::Unlimited(kinematics), traj);
}

bool ConvertCurveToTrajectory(const Curve& curve, double desired_speed,
                              const DynamicLimits& limits, Trajectory* traj) {
  return ConvertCurveToTrajectory(curve, curve.GetCordLengthSpan(),
                                  desired_speed, limits, traj);
}

bool ConvertCurveToTrajectory(const Curve& curve,
                              Interval<double> cord_length_span,
                              double desired_speed,
                              const Kinematics& kinematics, Trajectory* traj) {
  return ConvertCurveToTrajectory(curve, cord_length_span, desired_speed,
                                  DynamicLimits::Unlimited(kinematics), traj);
}

bool ConvertCurveToTrajectory(const Curve& curve,
                              Interval<double> cord_length_span,
                              double desired_speed, const DynamicLimits& limits,
                              Trajectory* traj) {
  if (std::abs(desired_speed) < kEpsilon) {
    return false;
  }
  traj->Clear();
  return AppendConvertedCurveToTrajectory(curve, cord_length_span,
                                          desired_speed, false, 0.0, limits,
                                          traj, nullptr);
}

bool ResampleCurve(const Curve& curve_in, double cord_step, Curve* curve_out) {
  if (curve_in.GetSize() < 1) {
    return false;
  }
  auto prev_it = curve_in.BeginCurvePoint();
  auto next_it = std::next(prev_it);
  if (!curve_out->AddPoint(prev_it->cord_length, prev_it->point)) {
    return false;
  }
  while (next_it != curve_in.EndCurvePoint()) {
    const double total_ds = next_it->cord_length - prev_it->cord_length;
    const double adjusted_step = total_ds / std::ceil(total_ds / cord_step);
    for (int i = 1; adjusted_step * (i + 0.5) < total_ds; ++i) {
      auto last_pt = curve_out->GetFinish();
      if (!curve_out->AddPoint(
              last_pt.cord_length + adjusted_step,
              last_pt.point.ExtrapolateConstantArc(adjusted_step))) {
        return false;
      }
    }
    if (!curve_out->AddPoint(next_it->cord_length, next_it->point)) {
      return false;
    }
    ++prev_it;
    ++next_it;
  }
  return true;
}

bool ResampleCurveToPolyline(const Curve& curve_in, double cord_step,
                             Curve* polyline_out) {
  if (curve_in.GetSize() < 1) {
    return false;
  }
  if (curve_in.GetSize() == 1) {
    auto start = curve_in.GetStart();
    start.point.SetCurvature(0.0);
    if (!polyline_out->AddPoint(start.cord_length, start.point)) {
      return false;
    }
    return true;
  }
  const double final_cord = curve_in.GetCordLengthSpan().max();
  auto prev_it = curve_in.BeginInCordLength();
  CurvePtAndCord staged_point;
  staged_point.cord_length = 0.0;
  staged_point.point.SetPose(prev_it.GetPoint().GetPose());
  staged_point.point.SetCurvature(0.0);
  while (prev_it.GetCordLength() < final_cord - kEpsilon) {
    const auto next_it =
        prev_it + std::min(cord_step, final_cord - prev_it.GetCordLength());
    const auto next_point = next_it.GetPoint();
    const eigenmath::Vector2d delta =
        next_point.GetPose().translation() -
        staged_point.point.GetPose().translation();
    const double delta_norm = delta.norm();
    staged_point.point.SetPose(
        eigenmath::Pose2d(staged_point.point.GetPose().translation(),
                          std::atan2(delta.y(), delta.x())));
    if (!polyline_out->AddPoint(staged_point.cord_length, staged_point.point)) {
      return false;
    }
    staged_point.cord_length += delta_norm;
    staged_point.point.SetPose(
        eigenmath::Pose2d(next_point.GetPose().translation(),
                          staged_point.point.GetPose().so2()));
    prev_it = next_it;
  }
  if (!polyline_out->AddPoint(staged_point.cord_length, staged_point.point)) {
    return false;
  }
  return true;
}

bool CheckPolylineProperties(const Curve& polyline) {
  if (polyline.GetSize() < 1) {
    return true;
  }
  auto prev_it = polyline.BeginCurvePoint();
  auto next_it = std::next(prev_it);
  while (next_it != polyline.EndCurvePoint()) {
    if (std::abs(prev_it->point.GetCurvature()) > kSmallEpsilon) {
      return false;
    }
    const double total_ds = next_it->cord_length - prev_it->cord_length;
    const auto from_prev = prev_it->point.ExtrapolateConstantArc(total_ds);
    const eigenmath::Vector2d delta = next_it->point.GetPose().translation() -
                                      from_prev.GetPose().translation();
    if (std::abs(delta.x()) > kSmallEpsilon ||
        std::abs(delta.y()) > kSmallEpsilon) {
      return false;
    }
    ++prev_it;
    ++next_it;
  }
  if (std::abs(prev_it->point.GetCurvature()) > kSmallEpsilon) {
    return false;
  }
  return true;
}

bool ResampleTrajectory(const Trajectory& traj_in, double time_step,
                        Trajectory* traj_out) {
  if (traj_in.GetSize() < 1) {
    return false;
  }
  auto prev_it = traj_in.BeginState();
  auto next_it = std::next(prev_it);
  if (!traj_out->AddState(prev_it->time, prev_it->state)) {
    return false;
  }
  while (next_it != traj_in.EndState()) {
    const double total_dt = next_it->time - prev_it->time;
    const double adjusted_step = total_dt / std::ceil(total_dt / time_step);
    for (int i = 1; adjusted_step * (i + 0.5) < total_dt; ++i) {
      auto last_pt = traj_out->GetFinish();
      if (!traj_out->AddState(
              last_pt.time + adjusted_step,
              last_pt.state.ExtrapolateConstantVelocityArc(adjusted_step))) {
        return false;
      }
    }
    if (!traj_out->AddState(next_it->time, next_it->state)) {
      return false;
    }
    ++prev_it;
    ++next_it;
  }
  return true;
}

void TimeWarpTrajectoryIntoLimits(const DynamicLimits& limits,
                                  Trajectory* traj) {
  for (auto state_it = traj->BeginState(); state_it != traj->EndState();
       ++state_it) {
    const ArcVector original_vel = state_it->state.GetArcVelocity();
    ArcVector limited_vel = ArcVector::Zero();
    if (original_vel.lpNorm<Eigen::Infinity>() > kEpsilon &&
        limits.VelocityLimits().BringInBounds(limits.GetKinematics(),
                                              original_vel, &limited_vel)) {
      if (limited_vel.lpNorm<Eigen::Infinity>() > kEpsilon) {
        traj->TimeWarpSegment(
            limited_vel.lpNorm<1>() / original_vel.lpNorm<1>(), state_it);
      } else {
        // We have a zero limit but need to move. All we can do is wait here.
        const double t = state_it->time;
        const double t_end = traj->GetTimeSpan().max();
        traj->TruncateTo({traj->GetTimeSpan().min(), t});
        traj->AddStationarySegment(t, std::max(kEpsilon, t_end - t));
        return;
      }
    }
  }
}

void DecaySpeedOfTrajectory(const Kinematics& kinematics,
                            const TrajectoryLimits& traj_limits,
                            double decay_rate_per_second, Trajectory* traj) {
  for (auto state_it = traj->BeginState(); state_it != traj->EndState();
       ++state_it) {
    const auto next_state_it = std::next(state_it);
    if (next_state_it == traj->EndState()) {
      break;
    }
    const ArcVector original_vel = next_state_it->state.GetArcVelocity();
    if (original_vel.lpNorm<Eigen::Infinity>() < kEpsilon) {
      continue;
    }
    const double accumulated_decay =
        std::exp(-decay_rate_per_second *
                 (next_state_it->time - traj->GetTimeSpan().min()));
    const ArcVector desired_vel = original_vel * accumulated_decay;
    const ArcVector limited_vel = traj_limits.ComputeLimitedNextVelocity(
        kinematics, state_it->state.GetArcVelocity(), desired_vel);
    if (limited_vel.lpNorm<Eigen::Infinity>() >
        kEpsilon * original_vel.lpNorm<Eigen::Infinity>()) {
      traj->TimeWarpSegment(limited_vel.lpNorm<Eigen::Infinity>() /
                                original_vel.lpNorm<Eigen::Infinity>(),
                            next_state_it);
    } else {
      // We have a zero limit but need to move. All we can do is wait here.
      const double t = next_state_it->time;
      const double t_end = traj->GetTimeSpan().max();
      traj->TruncateTo({traj->GetTimeSpan().min(), t});
      traj->AddStationarySegment(t, std::max(kEpsilon, t_end - t));
      return;
    }
  }
}

bool BringTrajectoryIntoLimits(const ArcVector& start_arc_vel,
                               const Trajectory& traj_in,
                               const DynamicLimits& limits,
                               const TrajectoryLimits& traj_limits,
                               Trajectory* traj_out) {
  ArcVector last_segment_arc_vel = start_arc_vel;
  traj_out->Clear();
  if (!traj_out->AddState(traj_in.GetStart())) {
    return false;
  }
  for (auto state_it = traj_in.BeginState(); state_it != traj_in.EndState();
       ++state_it) {
    const ArcVector desired_vel = state_it->state.GetArcVelocity();
    double next_time = (std::next(state_it) != traj_in.EndState()
                            ? std::next(state_it)->time
                            : std::numeric_limits<double>::infinity());
    while (traj_out->GetFinish().time + kTenNanoSeconds < next_time) {
      ArcVector limited_vel = traj_limits.ComputeLimitedNextVelocity(
          limits, last_segment_arc_vel, desired_vel);
      double delta_time = traj_limits.GetMinCycleDuration();
      if ((limited_vel - desired_vel).lpNorm<1>() <
          10.0 * std::numeric_limits<double>::epsilon()) {
        if (std::isfinite(next_time)) {
          // Reached desired velocity, just continue.
          delta_time = next_time - traj_out->GetFinish().time;
          if (delta_time < traj_limits.GetMinCycleDuration()) {
            limited_vel = last_segment_arc_vel;
          }
        } else {
          next_time = traj_out->GetFinish().time + delta_time;
        }
      }
      traj_out->GetMutableFinish().state.SetArcVelocity(limited_vel);
      last_segment_arc_vel = limited_vel;
      if (!traj_out->AddState(
              traj_out->GetFinish().time + delta_time,
              traj_out->GetFinish().state.ExtrapolateConstantVelocityArc(
                  delta_time))) {
        return false;
      }
    }
  }
  return true;
}

namespace {
bool AmendTrajectoryToEndWithStopRepairFuture(
    const Kinematics& kinematics, const TrajectoryLimits& traj_limits,
    Trajectory::StateIterator prev_it, Trajectory* traj) {
  const ArcVector& prev_velocity = prev_it->state.GetArcVelocity();
  const double& min_cycle_duration = traj_limits.GetMinCycleDuration();
  for (auto future_it = prev_it + 1;
       future_it != traj->EndState() &&
       future_it->time < prev_it->time + min_cycle_duration;
       ++future_it) {
    const double time_since_inter = future_it->time - prev_it->time;
    BoxConstraints future_vel_limits = traj_limits.GetVelocityJumpLimits();
    future_vel_limits.TranslateLimits(kinematics, prev_velocity);
    const ArcVector future_velocity = future_it->state.GetArcVelocity();
    const double fut_inf_norm = future_velocity.lpNorm<Eigen::Infinity>();
    if (fut_inf_norm < kSmallEpsilon) {
      continue;
    }
    ArcVector limited_future_velocity;
    if (!future_vel_limits.BringInBounds(
            kinematics, future_velocity, &limited_future_velocity,
            BoxConstraints::kPreserveDirectionOrFail)) {
      // Something is wrong, we cannot preserve curvature of the segment.
      return false;
    }
    const double lim_fut_inf_norm =
        limited_future_velocity.lpNorm<Eigen::Infinity>();
    if (lim_fut_inf_norm < kSmallEpsilon) {
      future_it = traj->AddStationarySegment(
          future_it->time, min_cycle_duration - time_since_inter);
      if (future_it == traj->EndState()) {
        return false;
      }
      continue;
    }
    traj->TimeWarpSegment(lim_fut_inf_norm / fut_inf_norm, future_it);
  }
  return true;
}
}  // namespace

bool AmendTrajectoryToEndWithStop(const Kinematics& kinematics,
                                  const TrajectoryLimits& traj_limits,
                                  Trajectory* traj) {
  return AmendTrajectoryToEndWithSlowdown(kinematics, 0.0, traj_limits, traj);
}

bool AmendTrajectoryToEndWithSlowdown(const Kinematics& kinematics,
                                      double desired_speed,
                                      const TrajectoryLimits& traj_limits,
                                      Trajectory* traj) {
  if (traj->IsEmpty() || !traj->IsSane()) {
    return false;
  }
  // General idea of the algorithm:
  // Walk backwards from the end of the trajectory and try to segment and
  // dilate the tail-end of the trajectory in order to create a ramp from zero
  // velocity to meeting up with the velocity in the trajectory.
  const double& min_cycle_duration = traj_limits.GetMinCycleDuration();

  auto next_it = std::prev(traj->EndState());
  if (std::abs(desired_speed) > kSmallEpsilon) {
    const double orig_final_speed =
        std::abs(next_it->state.GetArcVelocity().Translation());
    if (std::abs(desired_speed - orig_final_speed) < kSmallEpsilon) {
      return true;
    }
    if (desired_speed > orig_final_speed) {
      return false;
    }
    traj->TimeWarpSegment(desired_speed / orig_final_speed, next_it);
  } else {
    // Handle full-stop differently to catch pure-spin ending in original
    // trajectory.
    if (next_it->state.GetArcVelocity().lpNorm<1>() < kSmallEpsilon) {
      return true;  // Already stopped.
    }
    traj->TimeWarpSegment(0.0, next_it);
  }
  ArcVector next_velocity = next_it->state.GetArcVelocity();
  double cycle_duration_left = min_cycle_duration;
  while (next_it != traj->BeginState()) {
    const auto prev_it = std::prev(next_it);
    const ArcVector& prev_velocity = prev_it->state.GetArcVelocity();

    // Have we arrived at a velocity consistent with the original trajectory?
    if ((next_velocity - prev_velocity).lpNorm<Eigen::Infinity>() <
        kSmallEpsilon) {
      return true;
    }

    // Have we encountered a stationary segment? Must remove it.
    const double prev_inf_norm = prev_velocity.lpNorm<Eigen::Infinity>();
    if (prev_inf_norm < kSmallEpsilon) {
      traj->RemoveStationarySegment(prev_it);
      next_it = prev_it;
      continue;
    }

    // Get the next reachable speed.
    BoxConstraints prev_vel_limits = traj_limits.GetVelocityJumpLimits();
    prev_vel_limits.TranslateLimits(kinematics, next_velocity);
    ArcVector limited_prev_velocity;
    if (!prev_vel_limits.BringInBounds(
            kinematics, prev_velocity, &limited_prev_velocity,
            BoxConstraints::kPreserveDirectionOrFail)) {
      // Something is wrong, we cannot preserve curvature of the segment.
      return false;
    }

    // Check if we reached a stationary point:
    const double lim_inf_norm = limited_prev_velocity.lpNorm<Eigen::Infinity>();
    if (lim_inf_norm < kSmallEpsilon) {
      next_it = traj->AddStationarySegment(next_it->time, cycle_duration_left);
      if (next_it == traj->EndState()) {
        return false;
      }
      if (cycle_duration_left != min_cycle_duration) {
        // There were other intermediate states added, we must back-track to
        // make sure future states are reachable.
        if (!AmendTrajectoryToEndWithStopRepairFuture(kinematics, traj_limits,
                                                      next_it, traj)) {
          return false;
        }
      }
      next_velocity = limited_prev_velocity;
      cycle_duration_left = min_cycle_duration;
      continue;
    }

    // We want to insert a segment at time (t - cycle_duration_left) with
    // limited_prev_velocity.
    const double warp_factor = lim_inf_norm / prev_inf_norm;
    const double original_duration = cycle_duration_left * warp_factor;
    if (prev_it->time + original_duration + kTenNanoSeconds < next_it->time) {
      auto inter_it =
          traj->AddIntermediateState(next_it->time - original_duration);
      if (inter_it == traj->EndState()) {
        return false;
      }
      traj->TimeWarpSegment(warp_factor, inter_it);
      next_it = inter_it;
      if (cycle_duration_left != min_cycle_duration) {
        // There were other intermediate states added, we must back-track to
        // make sure future states are reachable.
        if (!AmendTrajectoryToEndWithStopRepairFuture(kinematics, traj_limits,
                                                      inter_it, traj)) {
          return false;
        }
      }
      next_velocity = limited_prev_velocity;
      cycle_duration_left = min_cycle_duration;
    } else {
      traj->TimeWarpSegment(warp_factor, prev_it);
      cycle_duration_left -= next_it->time - prev_it->time;
      next_it = prev_it;
    }
  }
  // We have amended the entire trajectory, that's fine too.
  return true;
}

bool FindSlowerTrajectoryToMeetEndSpeed(
    const Trajectory& original_traj, double original_speed,
    double desired_end_speed, double tolerance, const Kinematics& kinematics,
    const TrajectoryLimits& traj_limits, Trajectory* result_traj) {
  *result_traj = original_traj;
  if (!AmendTrajectoryToEndWithSlowdown(kinematics, desired_end_speed,
                                        traj_limits, result_traj)) {
    // Warping the entire trajectory is needed.
    double cumulative_warp_factor = 1.0;
    int bisect_result = eigenmath::BisectionSearchZeroCross(
        desired_end_speed / original_speed + tolerance, 1.0,
        [&](double candidate_warp_factor) {
          *result_traj = original_traj;
          result_traj->TimeWarp(candidate_warp_factor);
          const bool success = AmendTrajectoryToEndWithSlowdown(
              kinematics, desired_end_speed, traj_limits, result_traj);
          return (success ? 1.0 : -1.0);
        },
        tolerance, &cumulative_warp_factor, nullptr);
    if (bisect_result == -1) {
      return false;
    }
    *result_traj = original_traj;
    if (bisect_result == 0) {
      result_traj->TimeWarp(cumulative_warp_factor);
    }
    AmendTrajectoryToEndWithSlowdown(kinematics, desired_end_speed, traj_limits,
                                     result_traj);
  }
  return true;
}

bool FindSlowerTrajectoryToEndWithStop(const Trajectory& original_traj,
                                       double tolerance,
                                       const Kinematics& kinematics,
                                       const TrajectoryLimits& traj_limits,
                                       Trajectory* result_traj) {
  // The original_speed parameter is meaningless when desired_end_speed is 0.0.
  return FindSlowerTrajectoryToMeetEndSpeed(
      original_traj, /*original_speed=*/1.0, /*desired_end_speed=*/0.0,
      tolerance, kinematics, traj_limits, result_traj);
}

bool RoundOffCornersOfPolyline(const Curve& polyline, Curve* curve) {
  if (polyline.GetSize() < 1) {
    return false;
  }
  auto prev_it = polyline.BeginCurvePoint();
  auto next_it = std::next(prev_it);
  CurvePtAndCord staged_pt = *prev_it;
  double staged_orig_cord = prev_it->cord_length;
  while (next_it != polyline.EndCurvePoint()) {
    // Compute arc between staged_pt, prev_it, and mid-way to next_it.
    const double prev_mid_ds = prev_it->cord_length - staged_orig_cord;
    double next_mid_ds = 0.5 * (next_it->cord_length - prev_it->cord_length);
    if (std::next(next_it) == polyline.EndCurvePoint()) {
      next_mid_ds *= 2.0;  // go all the way to the end.
    }
    const CurvePoint next_mid_pt =
        prev_it->point.ExtrapolateConstantArc(next_mid_ds);
    const eigenmath::SO2d arc_so2 =
        staged_pt.point.GetPose().so2().inverse() * next_mid_pt.GetPose().so2();
    const double arc_angle = arc_so2.angle();

    if (prev_mid_ds < kSmallEpsilon) {
      staged_pt.point = prev_it->point;
      if (std::next(next_it) == polyline.EndCurvePoint()) {
        // If we are at the end, we must move the staged point to the end:
        if (!curve->AddPoint(staged_pt.cord_length, staged_pt.point)) {
          return false;
        }
        // Finally, stage the last point, with zero curvature:
        staged_pt.cord_length += next_mid_ds;
        staged_pt.point = next_mid_pt;
        staged_orig_cord = prev_it->cord_length + next_mid_ds;
      }
    } else if (next_mid_ds < kSmallEpsilon) {
      // skip
    } else if (std::abs(arc_so2.sin_angle()) < kSmallEpsilon) {
      if (!curve->AddPoint(staged_pt.cord_length, staged_pt.point)) {
        return false;
      }
      staged_pt.cord_length += prev_mid_ds;
      if (!curve->AddPoint(staged_pt.cord_length, prev_it->point)) {
        return false;
      }
      staged_pt.cord_length += next_mid_ds;
      staged_pt.point = next_mid_pt;
      staged_orig_cord = prev_it->cord_length + next_mid_ds;
    } else if (prev_mid_ds <= next_mid_ds) {
      // First, compute new curvature for staged point, and add it:
      const double kappa =
          (1.0 - arc_so2.cos_angle()) / arc_so2.sin_angle() / prev_mid_ds;
      staged_pt.point.SetCurvature(kappa);
      if (!curve->AddPoint(staged_pt.cord_length, staged_pt.point)) {
        return false;
      }
      // Second, compute the opposite end of the arc, and add it:
      const double arc_length = arc_angle / kappa;
      staged_pt.cord_length += arc_length;
      staged_pt.point = staged_pt.point.ExtrapolateConstantArc(arc_length);
      staged_pt.point.SetCurvature(0.0);
      // Avoid repeating nearly identical points:
      if (next_mid_ds > prev_mid_ds + kSmallEpsilon) {
        if (!curve->AddPoint(staged_pt.cord_length, staged_pt.point)) {
          return false;
        }
        // Finally, stage the last point, with zero curvature:
        staged_pt.cord_length += next_mid_ds - prev_mid_ds;
        staged_pt.point = next_mid_pt;
      }
      staged_orig_cord = prev_it->cord_length + next_mid_ds;
    } else if (next_mid_ds < prev_mid_ds) {
      // First, add the staged point, with zero curvature:
      // Avoid repeating nearly identical points:
      if (prev_mid_ds > next_mid_ds + kSmallEpsilon) {
        if (!curve->AddPoint(staged_pt.cord_length, staged_pt.point)) {
          return false;
        }
      }
      // Second, compute the start of the arc by going straight, then update
      // its curvature, and add it:
      const double kappa =
          (1.0 - arc_so2.cos_angle()) / arc_so2.sin_angle() / next_mid_ds;
      const double gap_length = prev_mid_ds - next_mid_ds;
      staged_pt.point = staged_pt.point.ExtrapolateConstantArc(gap_length);
      staged_pt.point.SetCurvature(kappa);
      staged_pt.cord_length += gap_length;
      if (!curve->AddPoint(staged_pt.cord_length, staged_pt.point)) {
        return false;
      }
      // Finally, compute the opposite end of the arc, and stage it:
      const double arc_length = arc_angle / kappa;
      staged_pt.point = staged_pt.point.ExtrapolateConstantArc(arc_length);
      staged_pt.cord_length += arc_length;
      staged_pt.point.SetCurvature(0.0);
      staged_orig_cord = prev_it->cord_length + next_mid_ds;
    }
    ++prev_it;
    ++next_it;
  }
  staged_pt.point = polyline.GetFinish().point;
  if (!curve->AddPoint(staged_pt.cord_length, staged_pt.point)) {
    return false;
  }
  return true;
}

bool ConvertCurveToClothoidTrajectory(const Curve& curve, double desired_speed,
                                      const DynamicLimits& limits,
                                      const TrajectoryLimits& traj_limits,
                                      Trajectory* traj) {
  if (curve.GetSize() <= 1 || std::abs(desired_speed) < kSmallEpsilon) {
    return false;
  }
  // This lambda flips the pose by 180 degrees if desired_speed is negative.
  auto maybe_flip_pose = [&](eigenmath::Pose2d pose) {
    if (desired_speed < 0.0) {
      pose.so2() =
          eigenmath::SO2d(-pose.so2().cos_angle(), -pose.so2().sin_angle(),
                          /* do_normalize = */ false);
    }
    return pose;
  };
  ArcVector desired_velocity{desired_speed, 0.0};
  limits.VelocityLimits().BringInBounds(
      limits.GetKinematics(), ArcVector{desired_speed, 0.0}, &desired_velocity);

  // For every segment of the curve:
  //  - If the curvature is reachable from the current velocity, then simply
  //    convert the curved segment to a constant velocity arc.
  //  - If the curvature is not reachable, then make a segment of the minimum
  //    cycle duration on both ends that achieves as much curvature as possible
  //    and try again to meet both ends of the reduced segment with one arc.
  traj->Clear();
  double current_time = 0.0;
  const double delta_time = traj_limits.GetMinCycleDuration();
  for (const auto segment :
       genit::AdjacentElementsRange<2>(curve.GetCurvePointIteratorRange())) {
    const int segment_start = traj->GetSize();

    // Get the end poses of the current complete curve segment.
    const eigenmath::Pose2d src_pose =
        maybe_flip_pose(segment[0].point.GetPose());
    const eigenmath::Pose2d dst_pose =
        maybe_flip_pose(segment[1].point.GetPose());

    // Compute the upper-bound total angle, to avoid winding loops:
    const double segment_curvature = segment[0].point.GetCurvature();
    eigenmath::SO2dInterval arc_segment = eigenmath::SO2dInterval(
        src_pose.so2(), dst_pose.so2(), segment_curvature);

    // Update the end states of the segment to account of straight segments
    // on both ends.
    ArcVector current_velocity = desired_velocity;
    State src_state(src_pose, current_velocity);
    State dst_state(dst_pose, current_velocity);

    if (std::abs(segment_curvature) > std::numeric_limits<double>::epsilon()) {
      // Add the starting state of this segment, with zero curvature.
      if (!traj->AddState(current_time, State(src_pose, current_velocity))) {
        return false;
      }
      current_time += delta_time;
      src_state = src_state.ExtrapolateConstantVelocityArc(delta_time);
      dst_state = dst_state.ExtrapolateConstantVelocityArc(-delta_time);
    }

    // Compute the ramp up to sufficient curvature.
    while (true) {
      // Calculate the parameters of the ideal arc to meet the pose difference:
      const eigenmath::Pose2d pose_diff =
          src_state.GetPose().inverse() * dst_state.GetPose();
      double angular_velocity = 0.0;
      if (1.0 - pose_diff.so2().cos_angle() <
          std::numeric_limits<double>::epsilon()) {
        if (std::abs(pose_diff.translation().y()) > kSmallEpsilon ||
            pose_diff.translation().x() * desired_speed < -kSmallEpsilon) {
          // This would require a pure spin or reverse driving.
          return false;
        }
        // Ensure that we measure the remaining orientation interval in the
        // right direction by adjusting the sign of the curvature here.
        angular_velocity = std::copysign(
            0.0, segment_curvature * current_velocity.Translation());
      } else {
        // If the curve is well-formed, this calculation should be too.
        if (std::abs(pose_diff.translation().x()) >
            std::abs(pose_diff.translation().y())) {
          angular_velocity =
              pose_diff.so2().sin_angle() *
              (current_velocity.Translation() / pose_diff.translation().x());
        } else {
          angular_velocity =
              (1.0 - pose_diff.so2().cos_angle()) *
              (current_velocity.Translation() / pose_diff.translation().y());
        }
      }

      // Check if the remaining part of the arc is still contained in the
      // original arc segment, using the latest calculated orientation.
      const eigenmath::SO2dInterval remaining_arc_segment =
          eigenmath::SO2dInterval(src_state.GetPose().so2(),
                                  dst_state.GetPose().so2(), angular_velocity);
      if (!arc_segment.Contains(remaining_arc_segment,
                                4.0 * std::numeric_limits<double>::epsilon())) {
        return false;
      }
      arc_segment = remaining_arc_segment;

      // Compute a reachable velocity for the given desired curvature.
      const ArcVector unlimited_next_velocity{current_velocity.Translation(),
                                              angular_velocity};
      const ArcVector next_velocity = traj_limits.ComputeLimitedNextVelocity(
          limits, current_velocity, unlimited_next_velocity);
      if (std::abs(next_velocity.Rotation() -
                   unlimited_next_velocity.Rotation()) < kSmallEpsilon) {
        // We can reach the desired curvature right now.
        current_velocity = next_velocity;
        if (!traj->AddState(current_time,
                            State(src_state.GetPose(), current_velocity))) {
          return false;
        }
        if (std::abs(angular_velocity) >
            std::numeric_limits<double>::epsilon()) {
          const double time_along_arc =
              arc_segment.Length() / std::abs(angular_velocity);
          current_time += time_along_arc;
        } else {
          current_time += std::max(0.0, pose_diff.translation().x() /
                                            current_velocity.Translation());
        }
        break;
      } else {
        // We cannot reach the desired curvature yet, just add a segment to
        // get closer to that and loop again.
        current_velocity = next_velocity;
        if (!traj->AddState(current_time,
                            State(src_state.GetPose(), current_velocity))) {
          return false;
        }
        current_time += delta_time;
        src_state = State(src_state.GetPose(), current_velocity)
                        .ExtrapolateConstantVelocityArc(delta_time);
        dst_state = State(dst_state.GetPose(), current_velocity)
                        .ExtrapolateConstantVelocityArc(-delta_time);
      }
    }

    // Reflect the ramp up in the ramp down to the end of the segment.
    for (auto state_it = traj->EndState() - 2,
              state_it_end = traj->BeginState() + segment_start - 1;
         state_it != state_it_end; --state_it) {
      const StateAndTime& ramp_up_state = *state_it;
      if (!traj->AddState(current_time,
                          State(dst_state.GetPose(),
                                ramp_up_state.state.GetArcVelocity()))) {
        return false;
      }
      current_time += delta_time;
      dst_state =
          traj->GetFinish().state.ExtrapolateConstantVelocityArc(delta_time);
    }
  }
  // Add the final state:
  const eigenmath::Pose2d final_pose =
      maybe_flip_pose(curve.GetFinish().point.GetPose());
  if (!traj->AddState(current_time, State(final_pose, desired_velocity))) {
    return false;
  }
  return true;
}

bool FindClothoidTrajectoryForCurve(const Curve& curve, double desired_speed,
                                    double tolerance,
                                    const DynamicLimits& limits,
                                    const TrajectoryLimits& traj_limits,
                                    Trajectory* traj,
                                    double* sub_desired_speed) {
  *sub_desired_speed = desired_speed;
  if (eigenmath::BisectionSearchZeroCross(
          tolerance * desired_speed, desired_speed,
          [&](double candidate_speed) {
            const bool success = ConvertCurveToClothoidTrajectory(
                curve, candidate_speed, limits, traj_limits, traj);
            return (success ? 1.0 : -1.0);
          },
          tolerance * std::abs(desired_speed), sub_desired_speed,
          nullptr) == -1) {
    return false;
  }
  ConvertCurveToClothoidTrajectory(curve, *sub_desired_speed, limits,
                                   traj_limits, traj);
  return true;
}

bool AppendClothoidFromCurveToTrajectory(
    double desired_speed, const DynamicLimits& limits,
    const TrajectoryLimits& traj_limits, const Curve& curve_segment,
    bool going_backwards, Trajectory* traj_segment,
    Trajectory* traj_scratch_space, Trajectory* result_traj) {
  // Must convert current curve segment to trajectory.
  double sub_desired_speed = 0.0;
  traj_segment->Clear();
  if (curve_segment.GetSize() == 1) {
    if (result_traj->IsEmpty()) {
      // Just use the pose from the curve as the start pose.
      return result_traj->AddState(
          0.0,
          State(curve_segment.GetFinish().point.GetPose(), ArcVector::Zero()));
    }
    // We have a pure-spin trajectory instead.
    if (!traj_segment->AddState(0.0,
                                State(result_traj->GetFinish().state.GetPose(),
                                      ArcVector::Zero()))) {
      return false;
    }
    if (!traj_limits.AppendBangBangRotationStoppingTrajectory(
            limits, curve_segment.GetFinish().point.GetPose().so2(), 1.0, 1.0,
            traj_limits.GetMinCycleDuration(), traj_segment)) {
      return false;
    }
    going_backwards = false;
    sub_desired_speed = 0.0;
  } else {
    if (!FindClothoidTrajectoryForCurve(curve_segment, desired_speed, 0.01,
                                        limits, traj_limits, traj_segment,
                                        &sub_desired_speed)) {
      return false;
    }
  }
  double interface_speed = 0.0;
  double last_added_time = 0.0;
  if (!result_traj->IsEmpty()) {
    const double current_velocity =
        result_traj->GetFinish().state.GetArcVelocity().Translation();
    if ((current_velocity > 0.0) != going_backwards) {
      interface_speed = std::min(std::abs(current_velocity), sub_desired_speed);
    }
    if (std::abs(current_velocity) > interface_speed) {
      // Amend slowdown to current trajectory.
      if (traj_scratch_space->GetCapacity() < result_traj->GetSize()) {
        return false;
      }
      *traj_scratch_space = *result_traj;
      if (!FindSlowerTrajectoryToMeetEndSpeed(
              *traj_scratch_space, std::abs(current_velocity), interface_speed,
              /*tolerance=*/0.01, limits.GetKinematics(), traj_limits,
              result_traj)) {
        return false;
      }
    }
    last_added_time = result_traj->GetTimeSpan().max();
  }
  if (sub_desired_speed > interface_speed) {
    if (!going_backwards) {
      // Reverse trajectory to amend it with slowdown at the start:
      traj_segment->Reverse();
    }
    // Amend reversed trajectory with a slowdown at the end (will be start
    // after reversal):
    if (traj_scratch_space->GetCapacity() < traj_segment->GetSize()) {
      return false;
    }
    *traj_scratch_space = *traj_segment;
    if (!FindSlowerTrajectoryToMeetEndSpeed(
            *traj_scratch_space, sub_desired_speed, interface_speed,
            /*tolerance=*/0.01, limits.GetKinematics(), traj_limits,
            traj_segment)) {
      return false;
    }
    // Restore trajectory (or reverse it, if going backwards):
    traj_segment->Reverse();
  } else {
    if (going_backwards) {
      // Reverse trajectory:
      traj_segment->Reverse();
    }
  }
  traj_segment->ApplyTimeShift(last_added_time -
                               traj_segment->GetTimeSpan().min());
  if (!result_traj->AppendTrajectory(*traj_segment)) {
    return false;
  }
  return true;
}

bool ComputeTwoArcsToFinalPose(const eigenmath::Vector2d& final_pt,
                               const eigenmath::SO2d& final_so2, double* kappa1,
                               double* arc_length1, double* kappa2,
                               double* arc_length2,
                               eigenmath::Pose2d* median_pose) {
  // The method used here is difficult to explain without a diagram, but here
  // goes an attempt to do so.
  //
  // We are given a start position (p1), start tangent (t1), end position (p2)
  // and end tangent (t2). Here, without loss of generality, we use p1 = (0,0)
  // and t1 = (1,0), and transform p2 and t2 to be relative to the start, and
  // given as final_pt and final_so2 parameters.
  //
  // We are looking for a distance (d). Let a1 == p1 + d * t1, meaning that a1
  // is a point at distance "d" forward from the start. Let a2 == p2 - d * t2,
  // meaning that a2 is "d" backward from the end.
  // We want to find "d" such that |a2 - a1| == 2*d. Working this out, we get
  // the quadratic equation seen in the code below. If a root exists (for "d"),
  // then we can form two isosceles triangles with summits at a1 and a2, i.e.,
  // {p1, a1, m} and {p2, a2, m}, where "m" is the midpoint between a1 and a2.
  // We can always create an arc that goes from one base corner to the other
  // in an isosceles triangle that falls tangent to both congruent sides. This
  // is used to form each of the two arcs that make the "S" shape that joins
  // the start and the end conditions.
  //
  // Obviously, this method is just one particular geometric method that
  // seems to produce reasonable results in reasonable cases. But it can get
  // a bit absurd in some extreme cases (e.g., end point behind the start).
  //
  // There are various corner cases being handled in this function too:
  // 1) If an isosceles triangle "squashed down" to a "flat triangle", then the
  //    arc is simply a straight line segment.
  // 2) If an isosceles triangle "squeezed from the sides" to make its base
  //    disappear, then the arc would have infinite curvature 180 degrees.
  //    This occurs when the end is directly behind the start and is aligned
  //    with it. It also occurs when end condition is 45 degrees behind the
  //    start and facing 90 degrees outwards. In these cases, "false" is
  //    returned because those motions are impossible without a straight
  //    motion between the two arcs.
  // 3) If the quadratic is degenerate (no roots), then it means that the
  //    desired motion is a perfect lateral motion. This is handled by an
  //    ad hoc solution: a symmetric "S" motion.
  // 4) If the roots of the quadratic are both negative, which seems to occur
  //    only when we have an end condition somewhere behind the start, aligned
  //    with the start. This is handled by reversing the direction of the arcs
  //    such that they span the outside of the isosceles triangles' bases
  //    instead of the inside.
  constexpr double kDblEpsilon = std::numeric_limits<double>::epsilon();
  if (final_pt.squaredNorm() < kDblEpsilon) {
    return false;
  }
  const eigenmath::Vector2d dogleg =
      eigenmath::Vector2d{1.0, 0.0} + final_so2.xAxis();
  // Clamp quad_a to expected max of zero in case of floating point error.
  const double quad_a = std::min(dogleg.dot(dogleg) - 4.0, 0.0);
  const double quad_b = -2.0 * final_pt.dot(dogleg);
  const double quad_c = final_pt.dot(final_pt);
  double d1, d2;
  int num_real_roots =
      eigenmath::ComputeQuadraticRoots(quad_a, quad_b, quad_c, &d1, &d2);
  if (num_real_roots == -1) {
    // CASE 3
    // Must be a lateral translation case (quad_a == 0 means that
    // final_so2.xAxis() == (1,0), and quad_b == 0 means that motion is
    // perpendicular to start). In this case, do an "S" shape through the
    // mid-point.
    *kappa1 = 4.0 / final_pt.y();
    *kappa2 = -(*kappa1);
    *arc_length1 = M_PI / std::abs(*kappa1);
    *arc_length2 = *arc_length1;
    median_pose->translation() = eigenmath::Vector2d{0.0, 0.5 * final_pt.y()};
    median_pose->so2() = eigenmath::SO2d{-1.0, 0.0, false};
    return true;
  }
  if (num_real_roots == 0) {
    // UNKNOWN CASE, this does not seem to occur in practice.
    return false;
  }
  const double dist = std::max(d1, d2);
  const eigenmath::Vector2d a1(dist, 0.0);
  const eigenmath::Vector2d a2 = final_pt - dist * final_so2.xAxis();
  // CASE 4: If dist < 0, reverse direction of median vector.
  const eigenmath::Vector2d median_vector = (dist >= 0.0 ? a2 - a1 : a1 - a2);
  const eigenmath::SO2d median_so2(median_vector.x(), median_vector.y());
  const eigenmath::SO2d final_so2_from_median =
      median_so2.inverse() * final_so2;

  if (std::abs(1.0 - median_so2.cos_angle()) < kDblEpsilon) {
    if (dist < 0.0) {
      // CASE 2
      // Would have to go backwards, which is not representable by a curve.
      return false;
    }
    // CASE 1
    // Straight segment of 2.0 * dist.
    *kappa1 = 0.0;
    *arc_length1 = 2.0 * dist;
  } else {
    if (std::abs(dist) < kDblEpsilon) {
      // CASE 2
      // Would have to make a pure 180 degree spin.
      return false;
    }
    *kappa1 = std::tan(median_so2.angle() / 2.0) / dist;
    *arc_length1 = median_so2.angle() / (*kappa1);
    if (*arc_length1 < 0.0) {
      // CASE 4
      *arc_length1 = 2.0 * M_PI / std::abs(*kappa1) + (*arc_length1);
    }
  }

  if (std::abs(1.0 - final_so2_from_median.cos_angle()) < kDblEpsilon) {
    if (dist < 0.0) {
      // CASE 2
      // Would have to go backwards, which is not representable by a curve.
      return false;
    }
    // CASE 1
    // Straight segment of 2.0 * dist.
    *kappa2 = 0.0;
    *arc_length2 = 2.0 * dist;
  } else {
    if (std::abs(dist) < kDblEpsilon) {
      // CASE 2
      // Would have to make a pure spin.
      return false;
    }
    *kappa2 = std::tan(final_so2_from_median.angle() / 2.0) / dist;
    *arc_length2 = final_so2_from_median.angle() / (*kappa2);
    if (*arc_length2 < 0.0) {
      // CASE 4
      *arc_length2 = 2.0 * M_PI / std::abs(*kappa2) + (*arc_length2);
    }
  }
  *median_pose = eigenmath::Pose2d{
      eigenmath::Vector2d{a1 + dist * median_so2.xAxis()}, median_so2};
  return true;
}

bool ComputeTwoArcsToFinalPose(const eigenmath::Pose2d& start_pose,
                               const eigenmath::Pose2d& final_pose,
                               Curve* output_curve) {
  if (output_curve == nullptr || output_curve->GetCapacity() < 3) {
    return false;
  }
  const eigenmath::Pose2d relative_final = start_pose.inverse() * final_pose;
  double kappa1, kappa2, arc_length1, arc_length2;
  eigenmath::Pose2d relative_median;
  if (!ComputeTwoArcsToFinalPose(relative_final.translation(),
                                 relative_final.so2(), &kappa1, &arc_length1,
                                 &kappa2, &arc_length2, &relative_median)) {
    return false;
  }

  output_curve->Clear();
  if (!output_curve->AddPoint(0.0, CurvePoint(start_pose, kappa1)) ||
      !output_curve->AddPoint(
          arc_length1, CurvePoint(start_pose * relative_median, kappa2)) ||
      !output_curve->AddPoint(arc_length1 + arc_length2,
                              CurvePoint(final_pose, 0.0))) {
    return false;
  }

  return true;
}

namespace {
bool TryTwoCirclesToFinalPose(
    const eigenmath::Vector2d& final_spline_pt,
    const eigenmath::SO2d& final_so2,
    const eigenmath::Vector2d& final_spline_pt_relative,
    const eigenmath::SO2d& final_so2_relative, double* prev_cord,
    eigenmath::Pose2d* prev_pose, Curve* curve) {
  double kappa1, arc_length1, kappa2, arc_length2;
  eigenmath::Pose2d median_pose_relative;
  if (!ComputeTwoArcsToFinalPose(final_spline_pt_relative, final_so2_relative,
                                 &kappa1, &arc_length1, &kappa2, &arc_length2,
                                 &median_pose_relative)) {
    return false;
  }

  if (!curve->AddPoint(*prev_cord, CurvePoint{*prev_pose, kappa1})) {
    return false;
  }
  *prev_cord += arc_length1;
  *prev_pose = (*prev_pose) * median_pose_relative;

  if (!curve->AddPoint(*prev_cord, CurvePoint{*prev_pose, kappa2})) {
    return false;
  }
  *prev_cord += arc_length2;
  prev_pose->translation() = final_spline_pt;
  prev_pose->so2() = final_so2;
  return true;
}

bool TryToMeetFinalAngleOfSpline(
    const eigenmath::QuinticSpline& spline,
    const eigenmath::Vector2d& final_spline_pt,
    const eigenmath::Vector2d& final_spline_pt_relative, double* prev_cord,
    eigenmath::Pose2d* prev_pose, Curve* curve) {
  // Treat the last point differently to try and get correct alignment of
  // the final angle without needing a pure rotation.
  const eigenmath::Vector2d final_spline_tangent =
      spline.Sample<1>(spline.MaxSplineIndex());
  if (final_spline_tangent.squaredNorm() < kSmallEpsilon * kSmallEpsilon) {
    // Unknown angle at the final point.
    return false;
  }
  const eigenmath::SO2d final_so2{final_spline_tangent.x(),
                                  final_spline_tangent.y()};
  const eigenmath::SO2d final_so2_relative =
      prev_pose->so2().inverse() * final_so2;
  if (final_so2_relative.sin_angle() * final_spline_pt_relative.y() <
      kSmallEpsilon * kSmallEpsilon) {
    // Final angle is not directed towards the final point coordinates.
    return TryTwoCirclesToFinalPose(
        final_spline_pt, final_so2, final_spline_pt_relative,
        final_so2_relative, prev_cord, prev_pose, curve);
  }
  const double distance_to_end =
      final_spline_pt_relative.y() / final_so2_relative.sin_angle();
  const double distance_from_start =
      final_spline_pt_relative.x() -
      final_so2_relative.cos_angle() * distance_to_end;
  if (distance_from_start < kSmallEpsilon) {
    // Start angle points away from the median line.
    return TryTwoCirclesToFinalPose(
        final_spline_pt, final_so2, final_spline_pt_relative,
        final_so2_relative, prev_cord, prev_pose, curve);
  }
  const double arc_angle = final_so2_relative.angle();
  if (distance_from_start > distance_to_end) {
    const double kappa = (1.0 - final_so2_relative.cos_angle()) /
                         final_so2_relative.sin_angle() / distance_to_end;
    const double arc_length = arc_angle / kappa;
    const double straight_length = distance_from_start - distance_to_end;
    if (!curve->AddPoint(*prev_cord, CurvePoint{*prev_pose, 0.0})) {
      return false;
    }
    *prev_cord += straight_length;
    prev_pose->translation() += straight_length * prev_pose->xAxis();
    if (!curve->AddPoint(*prev_cord, CurvePoint{*prev_pose, kappa})) {
      return false;
    }
    *prev_cord += arc_length;
    prev_pose->translation() = final_spline_pt;
    prev_pose->so2() = final_so2;
  } else {
    const double kappa = (1.0 - final_so2_relative.cos_angle()) /
                         final_so2_relative.sin_angle() / distance_from_start;
    const double arc_length = arc_angle / kappa;
    const double straight_length = distance_to_end - distance_from_start;
    if (!curve->AddPoint(*prev_cord, CurvePoint{*prev_pose, kappa})) {
      return false;
    }
    const eigenmath::Pose2d arc_pose{
        eigenmath::Vector2d{final_spline_pt -
                            straight_length * final_so2.xAxis()},
        final_so2};
    *prev_cord += arc_length;
    *prev_pose = arc_pose;
    if (!curve->AddPoint(*prev_cord, CurvePoint{*prev_pose, 0.0})) {
      return false;
    }
    *prev_cord += straight_length;
    prev_pose->translation() = final_spline_pt;
    prev_pose->so2() = final_so2;
  }
  return true;
}
}  // namespace

bool ConvertQuinticSplineToCurve(const eigenmath::QuinticSpline& spline,
                                 int num_samples_per_segment, Curve* curve) {
  // Empty spline, nothing to do, successfully return an empty curve.
  curve->Clear();
  if (spline.NumSegments() == 0) {
    return true;
  }
  CHECK_EQ(spline.NumDof(), 2) <<
             "Spline points should have 2 degrees of freedom.";
  CHECK_GE(num_samples_per_segment, 3) <<
             "Number of sample per spline segment should be at least 3.";
  const int num_samples =
      (num_samples_per_segment - 1) * spline.NumSegments() + 1;
  CHECK_GE(curve->GetCapacity(), num_samples + 1) << absl::StrFormat(
             "Capacity of curve (%d) should be at least num_samples + 1 (%d).",
             curve->GetCapacity(), num_samples + 1);
  const double spline_step = 1.0 / (num_samples_per_segment - 1);

  double current_spline_s = 0.0;
  double prev_cord = 0.0;
  const eigenmath::Vector2d prev_spline_pt = spline.Sample<0>(current_spline_s);
  eigenmath::Vector2d prev_spline_tangent = spline.Sample<1>(current_spline_s);
  while (prev_spline_tangent.squaredNorm() < kSmallEpsilon * kSmallEpsilon) {
    current_spline_s += spline_step;
    CHECK_LE(current_spline_s, spline.MaxSplineIndex()) <<
               "The spline appears to be completely stationary.";
    // Use a finite difference to the next point:
    prev_spline_tangent = spline.Sample<0>(current_spline_s) - prev_spline_pt;
  }
  eigenmath::Pose2d prev_pose{
      eigenmath::Vector2d{prev_spline_pt.x(), prev_spline_pt.y()},
      std::atan2(prev_spline_tangent.y(), prev_spline_tangent.x())};

  for (int spline_id = 1; spline_id < num_samples; ++spline_id) {
    current_spline_s = spline_id * spline_step;
    const eigenmath::Vector2d next_spline_pt =
        spline.Sample<0>(current_spline_s);
    const eigenmath::Vector2d next_spline_pt_relative =
        prev_pose.inverse() * next_spline_pt;
    const double ds_sqr = next_spline_pt_relative.squaredNorm();
    if (ds_sqr < kSmallEpsilon * kSmallEpsilon) {
      // No significant motion, just skip this point.
      continue;
    }
    if (std::abs(next_spline_pt_relative.y()) < kSmallEpsilon) {
      // Straight segment:
      CHECK_GT(next_spline_pt_relative.x(), 0.0) <<
                 "Straight motion reversal detected in spline!";
      if (!curve->AddPoint(prev_cord, CurvePoint{prev_pose, 0.0})) {
        return false;
      }
      prev_cord += next_spline_pt_relative.x();
      prev_pose.translation() = next_spline_pt;
    } else {
      if (spline_id == num_samples - 1) {
        // Treat the last point differently to try and get correct alignment of
        // the final angle without needing a pure rotation.
        if (TryToMeetFinalAngleOfSpline(spline, next_spline_pt,
                                        next_spline_pt_relative, &prev_cord,
                                        &prev_pose, curve)) {
          break;
        }
      }
      // Curved segment, find the arc dimensions:
      double arc_angle_for_xy = 2.0 * std::atan2(next_spline_pt_relative.y(),
                                                 next_spline_pt_relative.x());
      const double kappa_for_xy = std::copysign(
          std::sqrt(2.0 * (1.0 - std::cos(arc_angle_for_xy)) / ds_sqr),
          next_spline_pt_relative.y());
      const eigenmath::Vector2d next_spline_tangent =
          spline.Sample<1>(current_spline_s);
      if (spline_id != num_samples - 1 &&
          next_spline_tangent.squaredNorm() > kSmallEpsilon * kSmallEpsilon) {
        const eigenmath::SO2d next_so2(next_spline_tangent.x(),
                                       next_spline_tangent.y());
        const eigenmath::SO2d next_so2_relative =
            prev_pose.so2().inverse() * next_so2;
        if (next_so2_relative.sin_angle() * next_spline_pt_relative.y() >
            kSmallEpsilon * kSmallEpsilon) {
          if (0.5 * next_spline_pt_relative.x() <
              next_so2_relative.cos_angle() * next_spline_pt_relative.y() /
                  next_so2_relative.sin_angle()) {
            arc_angle_for_xy = next_so2_relative.angle();
          }
        }
      }
      if (!curve->AddPoint(prev_cord, CurvePoint{prev_pose, kappa_for_xy})) {
        return false;
      }
      const double arc_length = arc_angle_for_xy / kappa_for_xy;
      const eigenmath::Pose2d arc_pose = CurvePoint{prev_pose, kappa_for_xy}
                                             .ExtrapolateConstantArc(arc_length)
                                             .GetPose();
      prev_cord += arc_length;
      prev_pose = arc_pose;
    }
  }
  // We are at the final point, must check if we need to match the final
  // tangent of the spline.
  const eigenmath::Vector2d final_spline_tangent =
      spline.Sample<1>(spline.MaxSplineIndex());
  if (final_spline_tangent.squaredNorm() > kSmallEpsilon * kSmallEpsilon) {
    prev_pose.so2() =
        eigenmath::SO2d{final_spline_tangent.x(), final_spline_tangent.y()};
  }
  if (!curve->AddPoint(prev_cord, CurvePoint{prev_pose, 0.0})) {
    return false;
  }
  return true;
}

bool AppendRampToSpeedWithConstantAccel(const ArcVector& desired_arc_velocity,
                                        const Kinematics& kinematics,
                                        const TrajectoryLimits& traj_limits,
                                        double desired_duration,
                                        double aggressivity_factor,
                                        Trajectory* result_traj) {
  // Compute the wheel sampled velocity:
  WheelVector desired_wheel_velocity =
      kinematics.ComputeInverseKinematics(desired_arc_velocity);

  diff_drive::StateAndTime current_state = result_traj->GetFinish();
  const double final_time = current_state.time + desired_duration;

  // Compute good time-step and velocity increments to get from the
  // initial_state to sampled velocity:
  double suitable_time_step = 0.0;
  WheelVector suitable_wheel_vel_inc = {0.0, 0.0};
  ArcVector suitable_arc_vel_inc = {0.0, 0.0};
  traj_limits.GetSuitableVelocityIncrements(
      kinematics,
      kinematics.ComputeInverseKinematics(current_state.state.GetArcVelocity()),
      current_state.state.GetArcVelocity(), desired_wheel_velocity,
      desired_arc_velocity, desired_duration, aggressivity_factor,
      &suitable_time_step, &suitable_wheel_vel_inc, &suitable_arc_vel_inc);

  // Add points until we reach target velocity or final time:
  const bool accel_phase_complete = traj_limits.AppendUniformVelocityTicks(
      kinematics, desired_wheel_velocity, suitable_wheel_vel_inc,
      suitable_time_step, desired_duration, &current_state, result_traj);

  if (accel_phase_complete && final_time > current_state.time) {
    current_state.state = current_state.state.ExtrapolateConstantVelocityArc(
        final_time - current_state.time);
    current_state.time = final_time;
    if (!result_traj->AddState(current_state.time, current_state.state)) {
      return false;  // Exhausted capacity.
    }
  } else if (final_time > current_state.time) {
    return false;  // Exhausted capacity in AppendUniformVelocityTicks.
  }

  return true;
}

namespace {
double ComputeTotalWheelMotionImpl(const Trajectory& traj,
                                   const Kinematics& kinematics,
                                   const Interval<double>& time_interval,
                                   bool accumulate_absolute_value) {
  if (!time_interval.Intersects(traj.GetTimeSpan())) {
    return 0.0;
  }
  double left_wheel_motion = 0.0;
  double right_wheel_motion = 0.0;
  for (auto it = traj.BeginState(); it + 1 < traj.EndState(); ++it) {
    auto it_next = std::next(it);
    if (it_next->time < time_interval.min()) {
      continue;
    }
    auto dt = it_next->time - it->time;
    if (it->time < time_interval.min()) {
      dt = it_next->time - time_interval.min();
    }
    if (it_next->time > time_interval.max()) {
      dt -= it_next->time - time_interval.max();
    }
    auto wheel_velocity =
        kinematics.ComputeInverseKinematics(it->state.GetArcVelocity());
    if (accumulate_absolute_value) {
      left_wheel_motion += dt * std::abs(wheel_velocity.Left());
      right_wheel_motion += dt * std::abs(wheel_velocity.Right());
    } else {
      left_wheel_motion += dt * wheel_velocity.Left();
      right_wheel_motion += dt * wheel_velocity.Right();
    }
    if (it_next->time > time_interval.max()) {
      break;
    }
  }
  return std::abs(left_wheel_motion) + std::abs(right_wheel_motion);
}
}  // namespace

double ComputeTotalWheelMotion(const Trajectory& traj,
                               const Kinematics& kinematics) {
  return ComputeTotalWheelMotionImpl(traj, kinematics, traj.GetTimeSpan(),
                                     false);
}

double ComputeTotalWheelMotion(const Trajectory& traj,
                               const Kinematics& kinematics,
                               const Interval<double>& time_interval) {
  return ComputeTotalWheelMotionImpl(traj, kinematics, time_interval, false);
}

double ComputeTotalAbsoluteWheelMotion(const Trajectory& traj,
                                       const Kinematics& kinematics) {
  return ComputeTotalWheelMotionImpl(traj, kinematics, traj.GetTimeSpan(),
                                     true);
}

double ComputeTotalAbsoluteWheelMotion(const Trajectory& traj,
                                       const Kinematics& kinematics,
                                       const Interval<double>& time_interval) {
  return ComputeTotalWheelMotionImpl(traj, kinematics, time_interval, true);
}

bool ConvertCurveToWheelCurve(const Curve& curve, const Kinematics& kinematics,
                              const WheelVector& initial_wheel_position,
                              double initial_total_motion,
                              WheelCurve* wheel_curve) {
  wheel_curve->Clear();
  double w = initial_total_motion;
  WheelVector wheel_pos = initial_wheel_position;
  for (auto curve_segment :
       genit::AdjacentElementsRange<2>(curve.GetCurvePointIteratorRange())) {
    const CurvePtAndCord& pt0 = curve_segment[0];
    const CurvePtAndCord& pt1 = curve_segment[1];
    double ds = pt1.cord_length - pt0.cord_length;
    WheelVector wheel_arc_support = kinematics.ComputeInverseKinematics(
        ArcVector{1.0, pt0.point.GetCurvature()});
    double wheel_arc_support_norm1 = wheel_arc_support.lpNorm<1>();
    if (!wheel_curve->AddPoint(
            w, WheelState(wheel_pos,
                          wheel_arc_support / wheel_arc_support_norm1))) {
      return false;
    }
    // Check for pure rotation:
    CurvePoint arrival_point = pt0.point.ExtrapolateConstantArc(ds);
    eigenmath::SO2d rotation_diff =
        arrival_point.GetPose().so2().inverse() * pt1.point.GetPose().so2();
    if (std::abs(rotation_diff.angle()) > kEpsilon) {
      // First update the wheel position and total motion.
      w += ds * wheel_arc_support_norm1;
      wheel_pos += wheel_arc_support * ds;
      // Find unitary wheel arc supports.
      wheel_arc_support = kinematics.ComputeInverseKinematics(
          ArcVector{0.0, rotation_diff.angle()});
      wheel_arc_support_norm1 = wheel_arc_support.lpNorm<1>();
      ds = 1.0;
      if (!wheel_curve->AddPoint(
              w, WheelState(wheel_pos,
                            wheel_arc_support / wheel_arc_support_norm1))) {
        return false;
      }
    }
    w += ds * wheel_arc_support_norm1;
    wheel_pos += wheel_arc_support * ds;
  }
  const WheelVector wheel_arc_support = kinematics.ComputeInverseKinematics(
      ArcVector{1.0, curve.GetFinish().point.GetCurvature()});
  const double wheel_arc_support_norm1 = wheel_arc_support.lpNorm<1>();
  return wheel_curve->AddPoint(
      w, WheelState(wheel_pos, wheel_arc_support / wheel_arc_support_norm1));
}

bool ConvertTrajectoryToWheelCurve(const Trajectory& traj,
                                   const Kinematics& kinematics,
                                   const WheelVector& initial_wheel_position,
                                   double initial_total_motion,
                                   WheelCurve* wheel_curve) {
  wheel_curve->Clear();
  double w = initial_total_motion;
  WheelVector wheel_pos = initial_wheel_position;
  for (auto traj_segment :
       genit::AdjacentElementsRange<2>(traj.GetStateIteratorRange())) {
    const StateAndTime& traj_s0 = traj_segment[0];
    const StateAndTime& traj_s1 = traj_segment[1];
    const double dt = traj_s1.time - traj_s0.time;
    const WheelVector wheel_velocity =
        kinematics.ComputeInverseKinematics(traj_s0.state.GetArcVelocity());
    const double wheel_velocity_norm1 = wheel_velocity.lpNorm<1>();
    if (!wheel_curve->AddPoint(
            w, WheelState(wheel_pos, wheel_velocity / wheel_velocity_norm1))) {
      return false;
    }
    w += dt * wheel_velocity_norm1;
    wheel_pos += wheel_velocity * dt;
  }
  const WheelVector wheel_velocity = kinematics.ComputeInverseKinematics(
      traj.GetFinish().state.GetArcVelocity());
  const double wheel_velocity_norm1 = wheel_velocity.lpNorm<1>();
  return wheel_curve->AddPoint(
      w, WheelState(wheel_pos, wheel_velocity / wheel_velocity_norm1));
}

State TrajectoryWheelMotionIterator::ComputeState() const {
  auto dt = current_time_ - current_it_->time;
  return current_it_->state.ExtrapolateConstantVelocityArc(dt);
}

TrajectoryWheelMotionIterator& TrajectoryWheelMotionIterator::operator+=(
    double dw) {
  constexpr double kDblEpsilon = std::numeric_limits<double>::epsilon();
  // Do not allow backward wheel-motion-iteration:
  CHECK_GE(dw, 0.0) << "Cannot iterate backward along trajectory";
  while (true) {
    WheelVector wheel_velocity = kinematics_->ComputeInverseKinematics(
        current_it_->state.GetArcVelocity());
    const double total_wheel_vel = wheel_velocity.lpNorm<1>();
    if (dw < kDblEpsilon && total_wheel_vel < kDblEpsilon) {
      // We must have had a 0 / 0, which is fine, no motion:
      break;
    }
    if (total_wheel_vel < kDblEpsilon) {
      // we must have zero wheel motion, just move on to next state:
      if (current_time_ < max_time_) {
        ++current_it_;
        current_time_ = current_it_->time;
      }
      if (current_time_ >= max_time_) {
        // Reached the end of a stopping trajectory.
        // Set such that it will not be less than the sentinel iterator:
        current_time_ =
            std::nextafter(current_time_, std::numeric_limits<double>::max());
        break;
      }
      continue;
    }
    const double dt = dw / total_wheel_vel;
    if (current_it_->time < max_time_ &&
        dt + current_time_ > std::next(current_it_)->time) {
      dw -= (std::next(current_it_)->time - current_time_) * total_wheel_vel;
      ++current_it_;
      current_time_ = current_it_->time;
    } else {
      current_time_ += dt;
      break;
    }
  }
  return *this;
}

namespace {
struct OneLineCurvePtAndCord {
  double s;
  CurvePoint pt;
  OneLineCurvePtAndCord(double s_, CurvePoint pt_) : s(s_), pt(pt_) {}
  template <typename Sink>
  friend void AbslStringify(Sink& sink, const OneLineCurvePtAndCord& value) {
    absl::Format(&sink, "%v %v %v %v %v", FullPrecisionDouble(value.s),
                 FullPrecisionDouble(value.pt.GetPose().translation().x()),
                 FullPrecisionDouble(value.pt.GetPose().translation().y()),
                 FullPrecisionDouble(value.pt.GetPose().angle()),
                 FullPrecisionDouble(value.pt.GetCurvature()));
  }
};
struct OneLineWheelStateAndMotion {
  double w;
  WheelState state;
  OneLineWheelStateAndMotion(double w_, WheelState state_)
      : w(w_), state(state_) {}
  template <typename Sink>
  friend void AbslStringify(Sink& sink,
                            const OneLineWheelStateAndMotion& value) {
    absl::Format(&sink, "%v %v %v %v %v", FullPrecisionDouble(value.w),
                 FullPrecisionDouble(value.state.GetPositions().Left()),
                 FullPrecisionDouble(value.state.GetPositions().Right()),
                 FullPrecisionDouble(value.state.GetMotionRates().Left()),
                 FullPrecisionDouble(value.state.GetMotionRates().Right()));
  }
};
struct OneLineStateAndTime {
  double t;
  State state;
  OneLineStateAndTime(double t_, State state_) : t(t_), state(state_) {}
  template <typename Sink>
  friend void AbslStringify(Sink& sink, const OneLineStateAndTime& value) {
    absl::Format(
        &sink, "%v %v %v %v %v %v", FullPrecisionDouble(value.t),
        FullPrecisionDouble(value.state.GetPose().translation().x()),
        FullPrecisionDouble(value.state.GetPose().translation().y()),
        FullPrecisionDouble(value.state.GetPose().angle()),
        FullPrecisionDouble(value.state.GetArcVelocity().Translation()),
        FullPrecisionDouble(value.state.GetArcVelocity().Rotation()));
  }
};
}  // namespace

void PrintCurveTrace(const ContinuousCurve& curve, double ds,
                     std::ostream* out) {
  const Interval<double> cord_length_span = curve.GetCordLengthSpan();
  double s = cord_length_span.min();
  while (s < cord_length_span.max()) {
    const CurvePoint pt = curve.Evaluate(s);
    (*out) << absl::StrCat(OneLineCurvePtAndCord(s, pt)) << std::endl;
    s += ds;
  }
  const CurvePtAndCord pt = curve.GetFinish();
  (*out) << absl::StrCat(OneLineCurvePtAndCord(pt.cord_length, pt.point))
         << std::endl;
}

std::string CurveTraceToString(const ContinuousCurve& curve, double ds) {
  std::ostringstream result;
  PrintCurveTrace(curve, ds, &result);
  return result.str();
}

void PrintWheelCurveTrace(const WheelCurve& curve, double dw,
                          std::ostream* out) {
  const Interval<double> total_motion_span = curve.GetTotalMotionSpan();
  double w = total_motion_span.min();
  while (w < total_motion_span.max()) {
    const WheelState state = curve.Evaluate(w);
    (*out) << absl::StrCat(OneLineWheelStateAndMotion(w, state)) << std::endl;
    w += dw;
  }
  const WheelStateAndTotalMotion state = curve.GetFinish();
  (*out) << absl::StrCat(
                OneLineWheelStateAndMotion(state.total_motion, state.state))
         << std::endl;
}

std::string WheelCurveTraceToString(const WheelCurve& curve, double dw) {
  std::ostringstream result;
  PrintWheelCurveTrace(curve, dw, &result);
  return result.str();
}

void PrintTrajectoryTrace(const Trajectory& traj, double dt,
                          std::ostream* out) {
  for (auto it = traj.BeginInTime(), it_end = traj.EndInTime(); it < it_end;
       it += dt) {
    const State state = it.GetState();
    (*out) << absl::StrCat(OneLineStateAndTime(it.GetTime(), state))
           << std::endl;
  }
  const StateAndTime state = traj.GetFinish();
  (*out) << absl::StrCat(OneLineStateAndTime(state.time, state.state))
         << std::endl;
}

std::string TrajectoryTraceToString(const Trajectory& traj, double dt) {
  std::ostringstream result;
  PrintTrajectoryTrace(traj, dt, &result);
  return result.str();
}

}  // namespace mobility::diff_drive
