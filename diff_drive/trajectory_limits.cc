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


#include "diff_drive/trajectory_limits.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "diff_drive/interval.h"
#include "genit/iterator_range.h"
#include "diff_drive/kinematics.h"

namespace mobility::diff_drive {

namespace {
// Like std::partition_point, but inspects boundary elements before searching
// interior.
template <typename Iterator, typename Predicate>
Iterator PartitionPointBoundaryFirst(Iterator &&begin, Iterator &&end,
                                     Predicate &&pred) {
  using std::next;
  using std::prev;
  if (begin == end) return end;
  if (pred(*prev(end))) return end;
  if (begin == prev(end)) return begin;
  if (!pred(*begin)) return begin;

  return std::partition_point(next(begin), prev(end),
                              std::forward<Predicate>(pred));
}
}  // namespace

TrajectoryLimits::TrajectoryLimits(const DynamicLimits &dd_limits,
                                   double min_cycle_duration) {
  max_wheel_velocity_jump_ =
      dd_limits.MaxWheelAcceleration() * min_cycle_duration;
  max_arc_velocity_jump_ = dd_limits.MaxArcAcceleration() * min_cycle_duration;
  min_cycle_duration_ = min_cycle_duration;
}

void TrajectoryLimits::GetSuitableVelocityIncrements(
    const Kinematics & /*kinematics*/, const WheelVector &start_wheel_velocity,
    const ArcVector &start_arc_velocity, const WheelVector &end_wheel_velocity,
    const ArcVector &end_arc_velocity, double total_duration,
    double aggressivity_factor, double *suitable_time_step,
    WheelVector *suitable_wheel_vel_inc,
    ArcVector *suitable_arc_vel_inc) const {
  // Compute the time-step that will yield the given reduction factor on the
  // velocity jumps.
  const WheelVector total_wheel_vel_change =
      end_wheel_velocity - start_wheel_velocity;

  const ArcVector total_arc_vel_change = end_arc_velocity - start_arc_velocity;

  if (std::abs(total_wheel_vel_change.Left()) <
          max_wheel_velocity_jump_.Left() &&
      std::abs(total_wheel_vel_change.Right()) <
          max_wheel_velocity_jump_.Right() &&
      std::abs(total_arc_vel_change.Translation()) <
          max_arc_velocity_jump_.Translation() &&
      std::abs(total_arc_vel_change.Rotation()) <
          max_arc_velocity_jump_.Rotation()) {
    // there is no significant change in velocity:
    *suitable_time_step = min_cycle_duration_;
    *suitable_wheel_vel_inc = total_wheel_vel_change * aggressivity_factor;
    *suitable_arc_vel_inc = total_arc_vel_change * aggressivity_factor;
    return;
  }

  // Find the minimum number of jumps (in velocity) required to get to the
  // end velocity, which depends on which dimension is hardest to meet:
  const double all_min_num_jumps[4] = {
      std::abs(total_wheel_vel_change.Left()) / max_wheel_velocity_jump_.Left(),
      std::abs(total_wheel_vel_change.Right()) /
          max_wheel_velocity_jump_.Right(),
      std::abs(total_arc_vel_change.Translation()) /
          max_arc_velocity_jump_.Translation(),
      std::abs(total_arc_vel_change.Rotation()) /
          max_arc_velocity_jump_.Rotation()};
  const double min_num_jumps =
      std::ceil(*std::max_element(all_min_num_jumps, all_min_num_jumps + 4));
  // Check if we have enough time to meet the end condition:
  const double min_total_duration = min_cycle_duration_ * min_num_jumps;
  double total_num_jumps = 0.0;
  if (total_duration < min_total_duration) {
    // If we don't have enough time, just do as much as possible:
    total_num_jumps = min_num_jumps;
    *suitable_time_step = min_cycle_duration_;
  } else {
    // If we have enough time, use min-duration / aggressivity, unless it is
    // longer then the total duration available:
    const double suitable_total_duration =
        std::min(total_duration, min_total_duration / aggressivity_factor);
    total_num_jumps = std::floor(suitable_total_duration / min_cycle_duration_);
    *suitable_time_step = suitable_total_duration / total_num_jumps;
  }

  const double frac_per_step = 1.0 / total_num_jumps;
  *suitable_wheel_vel_inc = frac_per_step * total_wheel_vel_change;
  *suitable_arc_vel_inc = frac_per_step * total_arc_vel_change;
}

double TrajectoryLimits::GetLinearBrakingDistance(
    double start_velocity, double aggressivity_factor,
    double *suitable_time_step, double *suitable_vel_inc) const {
  if (start_velocity >= 0.0) {
    *suitable_vel_inc = -max_arc_velocity_jump_.Translation();
  } else {
    *suitable_vel_inc = max_arc_velocity_jump_.Translation();
  }
  const double num_of_steps = -start_velocity / (*suitable_vel_inc);
  *suitable_time_step = min_cycle_duration_ / aggressivity_factor;
  const double floor_num_of_steps = std::floor(num_of_steps);
  const double brake_dist =
      floor_num_of_steps * (*suitable_time_step) *
      (start_velocity + 0.5 * (floor_num_of_steps + 1.0) * (*suitable_vel_inc));
  return std::abs(brake_dist);
}

double TrajectoryLimits::GetLinearBangBangDisplacement(
    double start_velocity, double time_step, int steps_first,
    double velocity_step_first, double time_flat, int steps_second,
    double velocity_step_second) {
  const double dv_first = velocity_step_first * steps_first;
  const double dp_first =
      0.5 * (std::abs(steps_first) + 1) * dv_first * time_step +
      start_velocity * std::abs(steps_first) * time_step;
  const double peak_velocity = start_velocity + dv_first;
  const double dp_cruise = peak_velocity * time_flat;
  const double dp_second =
      (steps_second == 0
           ? 0.0
           : (steps_second - 1) * time_step *
                 (peak_velocity + 0.5 * steps_second * velocity_step_second));
  return dp_first + dp_cruise + dp_second;
}

void TrajectoryLimits::GetLinearBangBangParameters(
    double delta_in_position, double start_velocity,
    Interval<double> velocity_range, double velocity_step, double time_step,
    int *steps_first, double *velocity_step_first, double *time_flat,
    int *steps_second, double *velocity_step_second) {
  constexpr double kDblEpsilon = std::numeric_limits<double>::epsilon();
  if (std::abs(delta_in_position) < kDblEpsilon &&
      std::abs(start_velocity) < kDblEpsilon) {
    *steps_first = 0;
    *velocity_step_first = 0.0;
    *time_flat = 0.0;
    *steps_second = 0;
    *velocity_step_second = 0.0;
    return;
  }

  // Displacement of a two-ramp bang bang profile without a velocity hold.  This
  // is monotonic in steps_first for a fixed velocity_step_first.
  auto displacement_without_hold = [start_velocity, velocity_step, time_step](
                                       int steps_first,
                                       double velocity_step_first) {
    constexpr double kTimeFlat = 0.0;
    // Exactly meet 0 velocity at the end.  This guarantees that this
    // function is monotonic in `steps_first`.
    const double peak_velocity =
        start_velocity + velocity_step_first * steps_first;
    const int steps_second = std::lrint(
        std::max(1.0, std::ceil(std::abs(peak_velocity) / velocity_step)));
    const double velocity_step_second = -peak_velocity / steps_second;
    return GetLinearBangBangDisplacement(start_velocity, time_step, steps_first,
                                         velocity_step_first, kTimeFlat,
                                         steps_second, velocity_step_second);
  };
  // Bound the number of steps via the maximum velocity.
  const int min_steps_first = std::lrint(
      std::ceil((velocity_range.min() - start_velocity) / velocity_step));
  const int max_steps_first = std::lrint(
      std::floor((velocity_range.max() - start_velocity) / velocity_step));
  const auto steps_range = genit::IndexRange(min_steps_first, max_steps_first + 1);
  // Find the ramps which get closest to the desired distance. Obtain
  //  displacement(n_steps - 1) < delta_in_position <= displacement(n_steps).
  // Get almost 50% runtime reduction on average for benchmark input
  // distribution over std::partition_point.
  int n_steps = *PartitionPointBoundaryFirst(
      steps_range.begin(), steps_range.end(), [&](int steps_first) {
        return displacement_without_hold(steps_first, velocity_step) <
               delta_in_position;
      });

  // Choose between extrapolation at the peak velocity and interpolation of the
  // velocity step.
  const Interval<double> small_velocities(-velocity_step / 2,
                                          velocity_step / 2);
  if (start_velocity + (n_steps - 1) * velocity_step >=
      small_velocities.max()) {
    // Extrapolate the underestimate (n_steps - 1) with the positive peak
    // velocity.
    *steps_first = std::abs(n_steps - 1);
    *velocity_step_first = std::copysign(velocity_step, n_steps - 1);
  } else if (start_velocity + n_steps * velocity_step <=
             small_velocities.min()) {
    // Extrapolate the overestimate (n_steps) with the negative peak velocity.
    *steps_first = std::abs(n_steps);
    *velocity_step_first = std::copysign(velocity_step, n_steps);
  } else {
    // The peak velocity is near 0. Interpolate velocity_step to find a
    // single ramp matching the desired distance, or to get a large enough
    // peak velocity. With this number of steps, any peak velocity in
    // small_velocities can be reached.
    if (n_steps - 1 < 0) {
      n_steps = 1 - n_steps;
    }
    // otherwise keep n_steps >= 1.

    // Match velocity_step to get to the desired displacement if the peak
    // velocity stays in small_velocities.  Otherwise, use a velocity hold at
    // the boundary of small_velocities.
    *steps_first = n_steps;
    *velocity_step_first =
        (delta_in_position - start_velocity * std::abs(n_steps) * time_step) /
        (0.5 * (std::abs(n_steps) + 1) * n_steps * time_step);
    double peak_velocity = start_velocity + n_steps * *velocity_step_first;
    peak_velocity = std::clamp(peak_velocity, small_velocities.min(),
                               small_velocities.max());
    *velocity_step_first = (peak_velocity - start_velocity) / n_steps;
    *velocity_step_first =
        std::clamp(*velocity_step_first, -velocity_step, velocity_step);
    // By construction, the peak velocity should be contained in
    // small_velocities.
    peak_velocity = start_velocity + n_steps * *velocity_step_first;
  }

  const double peak_velocity =
      start_velocity + *steps_first * *velocity_step_first;
  *steps_second = std::lrint(
      std::max(1.0, std::ceil(std::abs(peak_velocity) / velocity_step)));
  *velocity_step_second = -peak_velocity / *steps_second;
  const double remaining_distance =
      delta_in_position -
      displacement_without_hold(*steps_first, *velocity_step_first);
  // Interpolation can lead to negligible negative hold times.
  *time_flat = std::abs(remaining_distance) /
               std::max(kDblEpsilon, std::abs(peak_velocity));
}

void TrajectoryLimits::GetLinearVelocityRamp(double delta_in_velocity,
                                             double max_velocity_step,
                                             int *num_velocity_steps,
                                             double *last_velocity_step) const {
  *num_velocity_steps = static_cast<int>(
      std::floor(std::abs(delta_in_velocity / max_velocity_step)));
  *last_velocity_step = std::abs(delta_in_velocity) -
                        *num_velocity_steps * std::abs(max_velocity_step);
}

double TrajectoryLimits::GetMinimumStoppingTime(
    const ArcVector &arc_velocity, const WheelVector &wheel_velocity) const {
  const double stopping_steps[4] = {
      std::abs(wheel_velocity.Left()) / max_wheel_velocity_jump_.Left(),
      std::abs(wheel_velocity.Right()) / max_wheel_velocity_jump_.Right(),
      std::abs(arc_velocity.Translation()) /
          max_arc_velocity_jump_.Translation(),
      std::abs(arc_velocity.Rotation()) / max_arc_velocity_jump_.Rotation()};

  return std::ceil(*std::max_element(stopping_steps, stopping_steps + 4)) *
         min_cycle_duration_;
}

double TrajectoryLimits::GetMinimumStoppingTime(
    const Kinematics &kinematics, const ArcVector &arc_velocity) const {
  return GetMinimumStoppingTime(
      arc_velocity, kinematics.ComputeInverseKinematics(arc_velocity));
}

double TrajectoryLimits::GetMinimumStoppingTime(
    const Kinematics &kinematics, const WheelVector &wheel_velocity) const {
  return GetMinimumStoppingTime(
      kinematics.ComputeForwardKinematics(wheel_velocity), wheel_velocity);
}

bool TrajectoryLimits::AppendBangBangRotationStoppingTrajectory(
    const DynamicLimits &limits, const eigenmath::SO2d &goal_so2,
    double aggressivity_factor, double realignment_aggressivity_factor,
    double time_step, Trajectory *traj, bool long_rotation) const {
  // Get the number and size of the velocity steps to stop translation.
  StateAndTime next_finish = traj->GetFinish();
  ArcVector cur_velocity = next_finish.state.GetArcVelocity();
  ArcVector velocity_step = limits.MaxArcAcceleration() * time_step;
  velocity_step.Translation() *= aggressivity_factor;
  velocity_step.Rotation() *= realignment_aggressivity_factor;
  int translation_steps;
  double last_translation_step;
  GetLinearVelocityRamp(cur_velocity.Translation(), velocity_step.Translation(),
                        &translation_steps, &last_translation_step);
  double translation_sign = 1.0;
  if (cur_velocity.Translation() < 0.0) {
    translation_sign = -1.0;
  }
  // Get the parameters for the rotation trapezoidal velocity ramp.
  eigenmath::Pose2d cur_pose = next_finish.state.GetPose();
  double delta_in_rotation = eigenmath::DeltaAngle(cur_pose.so2(), goal_so2);
  if (long_rotation) {
    // Use the complementing angle.
    delta_in_rotation += (delta_in_rotation > 0) ? -2 * M_PI : 2 * M_PI;
  }
  int steps_first = 0;
  double omega_step_first = 0.0;
  double time_flat = 0.0;
  int steps_second = 0;
  double omega_step_second = 0.0;
  GetLinearBangBangParameters(
      delta_in_rotation, cur_velocity.Rotation(),
      {limits.MinArcVelocity().Rotation(), limits.MaxArcVelocity().Rotation()},
      velocity_step.Rotation(), time_step, &steps_first, &omega_step_first,
      &time_flat, &steps_second, &omega_step_second);

  int total_steps = 0;
  // Do the ramp up.
  double rotation_velocity = 0.0;
  double translation_velocity = 0.0;
  for (int c = 0; c < steps_first; ++c) {
    rotation_velocity =
        next_finish.state.GetArcVelocity().Rotation() + omega_step_first;
    if (total_steps < translation_steps) {
      translation_velocity = next_finish.state.GetArcVelocity().Translation() -
                             translation_sign * velocity_step.Translation();
    } else if (total_steps == translation_steps) {
      translation_velocity = last_translation_step;
    } else {
      translation_velocity = 0.0;
    }
    next_finish.state.SetArcVelocity(
        ArcVector(translation_velocity, rotation_velocity));
    if (!traj->AddState(next_finish.time, next_finish.state)) {
      return false;
    }
    next_finish.state =
        next_finish.state.ExtrapolateConstantVelocityArc(time_step);
    next_finish.time += time_step;
    ++total_steps;
  }
  // Do the plateau.
  if (total_steps <= translation_steps) {
    // If the translation ramp has not completed, continue integer steps in time
    // for each state.
    while ((time_flat - time_step) > 0.0) {
      rotation_velocity = next_finish.state.GetArcVelocity().Rotation();
      if (total_steps < translation_steps) {
        translation_velocity =
            next_finish.state.GetArcVelocity().Translation() -
            translation_sign * velocity_step.Translation();
      } else if (total_steps == translation_steps) {
        translation_velocity = last_translation_step;
      } else {
        translation_velocity = 0.0;
      }
      next_finish.state.SetArcVelocity(
          ArcVector(translation_velocity, rotation_velocity));
      if (!traj->AddState(next_finish.time, next_finish.state)) {
        return false;
      }
      next_finish.state =
          next_finish.state.ExtrapolateConstantVelocityArc(time_step);
      next_finish.time += time_step;
      ++total_steps;
      time_flat -= time_step;
    }
  }
  // Create a non-integer step to accommodate plateau time correction for total
  // rotation traveled.
  if (time_flat > 0.0) {
    // Since this should only be at most time_step, or translational velocity
    // already zero, we allow the possibility of a "long hold" on one
    // translational velocity step.
    if (!traj->AddState(next_finish.time, next_finish.state)) {
      return false;
    }
    next_finish.state =
        next_finish.state.ExtrapolateConstantVelocityArc(time_flat);
    next_finish.time += time_flat;
    ++total_steps;
  }
  // Do the ramp-down.
  for (int c = 0; c < steps_second; ++c) {
    rotation_velocity =
        next_finish.state.GetArcVelocity().Rotation() + omega_step_second;
    if (c == steps_second - 1) {
      rotation_velocity = 0.0;
    }
    if (total_steps < translation_steps) {
      translation_velocity = next_finish.state.GetArcVelocity().Translation() -
                             translation_sign * velocity_step.Translation();
    } else if (total_steps == translation_steps) {
      translation_velocity = last_translation_step;
    } else {
      translation_velocity = 0.0;
    }
    next_finish.state.SetArcVelocity(
        ArcVector(translation_velocity, rotation_velocity));
    if (!traj->AddState(next_finish.time, next_finish.state)) {
      return false;
    }
    next_finish.state =
        next_finish.state.ExtrapolateConstantVelocityArc(time_step);
    next_finish.time += time_step;
    ++total_steps;
  }
  // Finish any remaining translational ramp down.
  while (total_steps <= translation_steps) {
    rotation_velocity = 0.0;
    translation_velocity = next_finish.state.GetArcVelocity().Translation() -
                           translation_sign * velocity_step.Translation();
    if (total_steps == translation_steps) {
      translation_velocity = last_translation_step;
    }
    next_finish.state.SetArcVelocity(
        ArcVector(translation_velocity, rotation_velocity));
    if (!traj->AddState(next_finish.time, next_finish.state)) {
      return false;
    }
    next_finish.state =
        next_finish.state.ExtrapolateConstantVelocityArc(time_step);
    next_finish.time += time_step;
    total_steps++;
  }
  //
  // Check to see if one final finishing step is necessary to zero out velocity
  // state.
  if (std::abs(rotation_velocity) > 0.0 ||
      std::abs(translation_velocity) > 0.0) {
    rotation_velocity = 0.0;
    translation_velocity = 0.0;
    next_finish.state.SetArcVelocity(
        ArcVector(translation_velocity, rotation_velocity));
    if (!traj->AddState(next_finish.time, next_finish.state)) {
      return false;
    }
  }
  return true;
}

bool TrajectoryLimits::AppendUniformVelocityTicks(
    const Kinematics &kinematics, const WheelVector &desired_wheel_velocity,
    const WheelVector &suitable_wheel_vel_inc, double suitable_time_step,
    double total_duration, StateAndTime *current_state,
    Trajectory *trajectory) const {
  // Add points until we reach target velocity or final time:
  double final_time = current_state->time + total_duration;
  while (current_state->time < final_time) {
    current_state->time += suitable_time_step;
    current_state->state =
        current_state->state.ExtrapolateConstantVelocityArc(suitable_time_step);
    const WheelVector current_wheel_vel = kinematics.ComputeInverseKinematics(
        current_state->state.GetArcVelocity());
    if ((current_wheel_vel - desired_wheel_velocity).squaredNorm() <
        suitable_wheel_vel_inc.squaredNorm()) {
      current_state->state.SetArcVelocity(
          kinematics.ComputeForwardKinematics(desired_wheel_velocity));
      // Precipitate the termination:
      final_time = current_state->time;
    } else {
      current_state->state.SetArcVelocity(kinematics.ComputeForwardKinematics(
          WheelVector(current_wheel_vel + suitable_wheel_vel_inc)));
    }
    if (!trajectory->AddState(current_state->time, current_state->state)) {
      // Ran out of points to add to the trajectory.
      return false;
    }
  }
  return true;
}

bool TrajectoryLimits::AppendStoppingTrajectory(const Kinematics &kinematics,
                                                double aggressivity_factor,
                                                double total_duration,
                                                StateAndTime *current_state,
                                                Trajectory *trajectory) const {
  const ArcVector &initial_arc_velocity = current_state->state.GetArcVelocity();
  const WheelVector initial_wheel_velocity =
      kinematics.ComputeInverseKinematics(initial_arc_velocity);

  const double min_stopping_time =
      GetMinimumStoppingTime(initial_arc_velocity, initial_wheel_velocity);

  // Come to a stop:
  const ArcVector zero_arc_velocity = {0.0, 0.0};
  const WheelVector zero_wheel_velocity = {0.0, 0.0};

  // Compute good time-step and velocity increments to get from the
  // initial_state to sampled velocity:
  double suitable_time_step = 0.0;
  WheelVector suitable_wheel_vel_inc = {0.0, 0.0};
  ArcVector suitable_arc_vel_inc = {0.0, 0.0};
  GetSuitableVelocityIncrements(
      kinematics, initial_wheel_velocity, initial_arc_velocity,
      zero_wheel_velocity, zero_arc_velocity,
      min_stopping_time / aggressivity_factor, aggressivity_factor,
      &suitable_time_step, &suitable_wheel_vel_inc, &suitable_arc_vel_inc);

  if (!AppendUniformVelocityTicks(kinematics, zero_wheel_velocity,
                                  suitable_wheel_vel_inc, suitable_time_step,
                                  min_stopping_time / aggressivity_factor,
                                  current_state, trajectory)) {
    return false;
  }

  const double start_time = trajectory->GetStart().time;
  if (current_state->time < start_time + total_duration) {
    if (!trajectory->AddState(start_time + total_duration,
                              current_state->state)) {
      return false;
    }
    current_state->time = start_time + total_duration;
  }

  return true;
}

ArcVector TrajectoryLimits::ComputeLimitedNextVelocity(
    const DynamicLimits &limits, const ArcVector &current_velocity,
    const ArcVector &desired_velocity) const {
  BoxConstraints max_jumps(GetMaxWheelVelocityJump(), GetMaxArcVelocityJump());

  ArcVector limited_desired_velocity = desired_velocity;
  if (!limits.VelocityLimits().IsInBounds(limits.GetKinematics(),
                                          desired_velocity)) {
    limits.VelocityLimits().BringInBounds(
        limits.GetKinematics(), desired_velocity, &limited_desired_velocity);
  }
  return ComputeLimitedNextVelocity(limits.GetKinematics(), current_velocity,
                                    limited_desired_velocity);
}

ArcVector TrajectoryLimits::ComputeLimitedNextVelocity(
    const Kinematics &kinematics, const ArcVector &current_velocity,
    const ArcVector &desired_velocity) const {
  BoxConstraints max_jumps(GetMaxWheelVelocityJump(), GetMaxArcVelocityJump());

  const ArcVector des_rel_velocity{
      desired_velocity.Translation() - current_velocity.Translation(),
      desired_velocity.Rotation() - current_velocity.Rotation()};
  if (!max_jumps.IsInBounds(kinematics, des_rel_velocity)) {
    ArcVector limited_des_rel_velocity = des_rel_velocity;
    max_jumps.BringInBounds(kinematics, des_rel_velocity,
                            &limited_des_rel_velocity);
    return ArcVector(current_velocity + limited_des_rel_velocity);
  }
  return desired_velocity;
}

}  // namespace mobility::diff_drive
