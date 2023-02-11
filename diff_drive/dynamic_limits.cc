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

#include "diff_drive/dynamic_limits.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "absl/log/check.h"

namespace mobility::diff_drive {

namespace {
constexpr double kSmallEpsilon = 1e-10;
inline bool IsInBound1D(double x, double x_min, double x_max) {
  return (x < x_max + kSmallEpsilon) && (x > x_min - kSmallEpsilon);
}
}  // namespace

bool BoxConstraints::BringInBounds(const eigenmath::Vector2d &min_vector,
                                   const eigenmath::Vector2d &max_vector,
                                   const eigenmath::Vector2d &vector_in,
                                   eigenmath::Vector2d *vector_out) {
  // We can assume that the vector can simply be scaled
  // towards the origin if it is outside the bounds.
  // Therefore, we need the box constraints to contain the origin.
  // Supporting non-origin-centric limits would require some assumption about
  // the direction of the clamping (minimize change of angle or magnitude).
  if (!ContainsOrigin(min_vector, max_vector) || (vector_out == nullptr)) {
    return false;
  }

  // Map [x,inf] -> [0,max], [inf,y] -> [max,0], [inf,inf] -> [0,0].
  if (std::isinf(vector_in[0]) || std::isinf(vector_in[1])) {
    (*vector_out)[0] =
        (std::isinf(vector_in[1])
             ? 0.0
             : (vector_in[0] < 0.0 ? min_vector[0] : max_vector[0]));
    (*vector_out)[1] =
        (std::isinf(vector_in[0])
             ? 0.0
             : (vector_in[1] < 0.0 ? min_vector[1] : max_vector[1]));
    return true;
  }

  // This is a safe division function that is specific to this particular
  // case, where we want to preserve a value of 1.0 instead of
  // divide by zero
  auto capped_division = [](double num, double denom) -> double {
    if (std::abs(denom) < kSmallEpsilon) {
      return 1.0;
    }
    return std::min(num / denom, 1.0);
  };

  double scale_factors[4] = {capped_division(min_vector(0), vector_in(0)),
                             capped_division(max_vector(0), vector_in(0)),
                             capped_division(min_vector(1), vector_in(1)),
                             capped_division(max_vector(1), vector_in(1))};
  double best_scale_factor = *std::min_element(
      scale_factors, scale_factors + 4, [](double lhs, double rhs) -> bool {
        // Search for the minimum positive scale factor:
        return (rhs < 0.0) || ((lhs >= 0.0) && (lhs < rhs));
      });
  *vector_out = best_scale_factor * vector_in;
  return true;
}

bool BoxConstraints::BringInBoundsPreserveDirection(
    const eigenmath::Vector2d &min_vector,
    const eigenmath::Vector2d &max_vector, const eigenmath::Vector2d &vector_in,
    eigenmath::Vector2d *vector_out) {
  if (IsInBound1D(vector_in[0], min_vector[0], max_vector[0]) &&
      IsInBound1D(vector_in[1], min_vector[1], max_vector[1])) {
    // Trivial solution, already in bounds.
    *vector_out = vector_in;
    return true;
  }
  // Compute per-dimension scale factors:
  eigenmath::Vector2d per_dim_scale_factors{1.0, 1.0};
  for (int i = 0; i < 2; ++i) {
    if (std::abs(vector_in[i]) > kSmallEpsilon) {
      if (vector_in[i] <= max_vector[i] && vector_in[i] >= min_vector[i]) {
        // Preserve it.
        per_dim_scale_factors[i] = 1.0;
      } else if (vector_in[i] <= max_vector[i]) {
        // Raise it to the minimum.
        per_dim_scale_factors[i] = min_vector[i] / vector_in[i];
      } else {
        // Lower it to the maximum.
        per_dim_scale_factors[i] = max_vector[i] / vector_in[i];
      }
    }
  }
  // Find a viable solution that scales the vector in the least:
  double best_scale_factor = -std::numeric_limits<double>::infinity();
  for (int i = 0; i < 2; ++i) {
    const eigenmath::Vector2d new_v = per_dim_scale_factors[i] * vector_in;
    if (IsInBound1D(new_v[0], min_vector[0], max_vector[0]) &&
        IsInBound1D(new_v[1], min_vector[1], max_vector[1]) &&
        std::abs(1.0 - best_scale_factor) >
            std::abs(1.0 - per_dim_scale_factors[i])) {
      // We found a solution:
      best_scale_factor = per_dim_scale_factors[i];
    }
  }
  if (std::isfinite(best_scale_factor)) {
    // At least one viable solution was found:
    *vector_out = best_scale_factor * vector_in;
    return true;
  }
  return false;
}

bool BoxConstraints::BringInBounds(const WheelVector &wheel,
                                   WheelVector *wheel_out,
                                   BringInBoundsPolicy policy) const {
  switch (policy) {
    case kMustContainOrigin:
      return BringInBounds(min_wheel_, max_wheel_, wheel, wheel_out);
    case kPreserveDirectionOrFail:
      return BringInBoundsPreserveDirection(min_wheel_, max_wheel_, wheel,
                                            wheel_out);
  }
}

bool BoxConstraints::BringInBounds(const ArcVector &arc, ArcVector *arc_out,
                                   BringInBoundsPolicy policy) const {
  switch (policy) {
    case kMustContainOrigin:
      return BringInBounds(min_arc_, max_arc_, arc, arc_out);
    case kPreserveDirectionOrFail:
      return BringInBoundsPreserveDirection(min_arc_, max_arc_, arc, arc_out);
  }
}

bool BoxConstraints::BringInBounds(const Kinematics &kinematics,
                                   const WheelVector &wheel,
                                   WheelVector *wheel_out,
                                   BringInBoundsPolicy policy) const {
  WheelVector limited_wheel;
  if (!BringInBounds(wheel, &limited_wheel, policy)) {
    return false;
  }
  const auto limited_arc = kinematics.ComputeForwardKinematics(limited_wheel);
  ArcVector final_arc;
  if (!BringInBounds(limited_arc, &final_arc, policy)) {
    return false;
  }
  limited_wheel = kinematics.ComputeInverseKinematics(final_arc);
  if (IsInBounds(limited_wheel)) {
    *wheel_out = limited_wheel;
    return true;
  }
  return false;
}

bool BoxConstraints::BringInBounds(const Kinematics &kinematics,
                                   const ArcVector &arc, ArcVector *arc_out,
                                   BringInBoundsPolicy policy) const {
  ArcVector limited_arc;
  if (!BringInBounds(arc, &limited_arc, policy)) {
    return false;
  }
  const auto limited_wheel = kinematics.ComputeInverseKinematics(limited_arc);
  WheelVector final_wheel;
  if (!BringInBounds(limited_wheel, &final_wheel, policy)) {
    return false;
  }
  limited_arc = kinematics.ComputeForwardKinematics(final_wheel);
  if (IsInBounds(limited_arc)) {
    *arc_out = limited_arc;
    return true;
  }
  return false;
}

BoxConstraints BoxConstraints::Intersect(const BoxConstraints &first,
                                         const BoxConstraints &second) {
  return BoxConstraints(
      WheelVector(
          std::max(first.min_wheel_.Left(), second.min_wheel_.Left()),
          std::max(first.min_wheel_.Right(), second.min_wheel_.Right())),
      WheelVector(
          std::min(first.max_wheel_.Left(), second.max_wheel_.Left()),
          std::min(first.max_wheel_.Right(), second.max_wheel_.Right())),
      ArcVector(
          std::max(first.min_arc_.Translation(), second.min_arc_.Translation()),
          std::max(first.min_arc_.Rotation(), second.min_arc_.Rotation())),
      ArcVector(
          std::min(first.max_arc_.Translation(), second.max_arc_.Translation()),
          std::min(first.max_arc_.Rotation(), second.max_arc_.Rotation())));
}

BoxConstraints BoxConstraints::GrowToIntersect(
    const BoxConstraints &first, const BoxConstraints &second_growable) {
  BoxConstraints result = Intersect(first, second_growable);
  if (!result.IsEmpty()) {
    return result;  // NRVO
  }
  // If the result is empty, it means that some min-max are inverted.
  // The code below basically figures out if the min has the brought to
  // the max or vice versa based on which represents a growth of the
  // second_growable limits to make the intersection happen.
  // This is simply repeated for each left-right-trans-rot directions.

  if (result.min_wheel_.Left() > result.max_wheel_.Left()) {
    if (second_growable.max_wheel_.Left() < result.min_wheel_.Left()) {
      result.max_wheel_.Left() = result.min_wheel_.Left();
    } else {
      result.min_wheel_.Left() = result.max_wheel_.Left();
    }
  }

  if (result.min_wheel_.Right() > result.max_wheel_.Right()) {
    if (second_growable.max_wheel_.Right() < result.min_wheel_.Right()) {
      result.max_wheel_.Right() = result.min_wheel_.Right();
    } else {
      result.min_wheel_.Right() = result.max_wheel_.Right();
    }
  }

  if (result.min_arc_.Translation() > result.max_arc_.Translation()) {
    if (second_growable.max_arc_.Translation() <
        result.min_arc_.Translation()) {
      result.max_arc_.Translation() = result.min_arc_.Translation();
    } else {
      result.min_arc_.Translation() = result.max_arc_.Translation();
    }
  }

  if (result.min_arc_.Rotation() > result.max_arc_.Rotation()) {
    if (second_growable.max_arc_.Rotation() < result.min_arc_.Rotation()) {
      result.max_arc_.Rotation() = result.min_arc_.Rotation();
    } else {
      result.min_arc_.Rotation() = result.max_arc_.Rotation();
    }
  }
  return result;  // NRVO
}

bool BoxConstraints::IsEmpty() const {
  // A set of constraints is empty if at least one min-max pair is inverted.
  return (min_wheel_.Left() > max_wheel_.Left()) ||
         (min_wheel_.Right() > max_wheel_.Right()) ||
         (min_arc_.Translation() > max_arc_.Translation()) ||
         (min_arc_.Rotation() > max_arc_.Rotation());
}

BoxConstraints BoxConstraints::ReverseConstraints(
    const BoxConstraints &constraints) {
  BoxConstraints reflected(-constraints.max_wheel_, -constraints.min_wheel_,
                           -constraints.max_arc_, -constraints.min_arc_);
  return reflected;
}

DynamicLimits::DynamicLimits(const Kinematics &kinematics,
                             const WheelVector &max_wheel_velocity,
                             const WheelVector &max_wheel_acceleration,
                             const ArcVector &max_arc_velocity,
                             const ArcVector &max_arc_acceleration)
    : kinematics_(kinematics),
      velocity_limits_(max_wheel_velocity, max_arc_velocity),
      acceleration_limits_(max_wheel_acceleration, max_arc_acceleration) {}

DynamicLimits::DynamicLimits(const Kinematics &kinematics,
                             const WheelVector &min_wheel_velocity,
                             const WheelVector &max_wheel_velocity,
                             const WheelVector &min_wheel_acceleration,
                             const WheelVector &max_wheel_acceleration,
                             const ArcVector &min_arc_velocity,
                             const ArcVector &max_arc_velocity,
                             const ArcVector &min_arc_acceleration,
                             const ArcVector &max_arc_acceleration)
    : kinematics_(kinematics),
      velocity_limits_(min_wheel_velocity, max_wheel_velocity, min_arc_velocity,
                       max_arc_velocity),
      acceleration_limits_(min_wheel_acceleration, max_wheel_acceleration,
                           min_arc_acceleration, max_arc_acceleration) {}

void DynamicLimits::ComputeSafetyDistances(double reaction_time,
                                           double *dist_forward,
                                           double *dist_backward,
                                           double *dist_sideways) const {
  QCHECK_NE(dist_forward, nullptr);
  QCHECK_NE(dist_backward, nullptr);
  QCHECK_NE(dist_sideways, nullptr);
  const double max_obstacle_speed =
      std::max(-velocity_limits_.MinArcVector().Translation(),
               velocity_limits_.MaxArcVector().Translation());
  *dist_sideways = max_obstacle_speed * reaction_time;
  const double linear_decel =
      std::abs(acceleration_limits_.MinArcVector().Translation());
  const double forward_speed = velocity_limits_.MaxArcVector().Translation();
  const double forward_stop_distance =
      0.5 * forward_speed * forward_speed / linear_decel;
  *dist_forward =
      forward_stop_distance + *dist_sideways + forward_speed * reaction_time;
  const double backward_speed = -velocity_limits_.MinArcVector().Translation();
  const double backward_stop_distance =
      0.5 * backward_speed * backward_speed / linear_decel;
  *dist_backward =
      backward_stop_distance + *dist_sideways + backward_speed * reaction_time;
}

DynamicLimits DynamicLimits::ReverseLimits(const DynamicLimits &limits) {
  DynamicLimits reversed(
      limits.kinematics_,
      BoxConstraints::ReverseConstraints(limits.velocity_limits_),
      BoxConstraints::ReverseConstraints(limits.acceleration_limits_));
  return reversed;
}
}  // namespace mobility::diff_drive
