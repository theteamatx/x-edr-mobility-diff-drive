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

#ifndef MOBILITY_DIFF_DRIVE_DIFF_DRIVE_DYNAMIC_LIMITS_H_
#define MOBILITY_DIFF_DRIVE_DIFF_DRIVE_DYNAMIC_LIMITS_H_

#include <limits>

#include "diff_drive/kinematics.h"

namespace mobility::diff_drive {

//
// This class stores wheel / arc velocity limits as two bounding boxes
// (min,max) corners aligned in the arc-space and wheel-space, respectively.
// The class provides a number of utility functions to verify or enforce
// the limits on wheel or arc velocity vectors.
//
class BoxConstraints {
 public:
  // Default constructor; leaves the min and max to be zero.
  BoxConstraints() = default;

  // Constructor for asymmetric limits. Takes a min and max vector
  // for both the wheel-space and the arc-space.
  BoxConstraints(const WheelVector &min_wheel, const WheelVector &max_wheel,
                 const ArcVector &min_arc, const ArcVector &max_arc)
      : min_wheel_(min_wheel),
        max_wheel_(max_wheel),
        min_arc_(min_arc),
        max_arc_(max_arc) {}

  // Constructor for symmetric limits. Takes a max vector
  // for both the wheel-space and the arc-space, and assumes that the
  // limits are symmetric around the origin.
  BoxConstraints(const WheelVector &max_wheel, const ArcVector &max_arc)
      : min_wheel_(-max_wheel),
        max_wheel_(max_wheel),
        min_arc_(-max_arc),
        max_arc_(max_arc) {}

  // Remove an ambiguous overload.
  BoxConstraints(const eigenmath::Vector2d &unknown1,
                 const eigenmath::Vector2d &unknown2) = delete;

  // Remove an ambiguous overload.
  BoxConstraints(const eigenmath::Vector2d &unknown1,
                 const eigenmath::Vector2d &unknown2,
                 const eigenmath::Vector2d &unknown3,
                 const eigenmath::Vector2d &unknown4) = delete;

  static BoxConstraints Unlimited() {
    return BoxConstraints(WheelVector{std::numeric_limits<double>::infinity(),
                                      std::numeric_limits<double>::infinity()},
                          ArcVector{std::numeric_limits<double>::infinity(),
                                    std::numeric_limits<double>::infinity()});
  }

  bool IsEmpty() const;

  // Checks if a wheel vector is within the wheel-space box constraints.
  bool IsInBounds(const WheelVector &wheel) const {
    return wheel.Left() >= min_wheel_.Left() - kEpsilon &&
           wheel.Right() >= min_wheel_.Right() - kEpsilon &&
           wheel.Left() <= max_wheel_.Left() + kEpsilon &&
           wheel.Right() <= max_wheel_.Right() + kEpsilon;
  }

  // Checks if a arc velocity is within the arc-space box constraints.
  bool IsInBounds(const ArcVector &arc) const {
    return arc.Translation() >= min_arc_.Translation() - kEpsilon &&
           arc.Rotation() >= min_arc_.Rotation() - kEpsilon &&
           arc.Translation() <= max_arc_.Translation() + kEpsilon &&
           arc.Rotation() <= max_arc_.Rotation() + kEpsilon;
  }

  // Remove an ambiguous overload.
  bool IsInBounds(const eigenmath::Vector2d &unknown) const = delete;

  // Checks if a wheel vector is within the wheel-space and arc-space
  // box constraints, using the given kinematics object to map between them.
  bool IsInBounds(const Kinematics &kinematics,
                  const WheelVector &wheel) const {
    ArcVector arc = kinematics.ComputeForwardKinematics(wheel);
    return IsInBounds(wheel) && IsInBounds(arc);
  }

  // Checks if an arc vector is within the wheel-space and arc-space
  // box constraints, using the given kinematics object to map between them.
  bool IsInBounds(const Kinematics &kinematics, const ArcVector &arc) const {
    WheelVector wheel = kinematics.ComputeInverseKinematics(arc);
    return IsInBounds(wheel) && IsInBounds(arc);
  }

  // Remove an ambiguous overload.
  bool IsInBounds(const Kinematics &kinematics,
                  const eigenmath::Vector2d &unknown_velocity) const = delete;

  // Checks if the box constraints contain the origin (e.g., full-stop).
  bool ContainsOrigin() const {
    return ContainsOrigin(min_wheel_, max_wheel_) &&
           ContainsOrigin(min_arc_, max_arc_);
  }

  // BringInBounds functions:
  // Brings a given vector into the box constraints by scaling it down
  // towards the origin while preserving the direction.
  // This will return false if the bounds don't contain the origin, because
  // in that case, it is not possible to always scale down the vector to
  // get to within the bounds, and additional assumptions must be made on a
  // case-by-case basis. Users must write their own functions for that case.

  enum BringInBoundsPolicy {
    kMustContainOrigin = 0,
    kPreserveDirectionOrFail,
  };

  // Brings a given wheel vector into the wheel-space box constraints
  // by scaling it down towards the origin.
  bool BringInBounds(const WheelVector &wheel, WheelVector *wheel_out,
                     BringInBoundsPolicy policy = kMustContainOrigin) const;

  // Brings a given arc vector into the arc-space box constraints
  // by scaling it down towards the origin.
  bool BringInBounds(const ArcVector &arc, ArcVector *arc_out,
                     BringInBoundsPolicy policy = kMustContainOrigin) const;

  // Remove an ambiguous overload.
  bool BringInBounds(
      const eigenmath::Vector2d &unknown, eigenmath::Vector2d *unknown_out,
      BringInBoundsPolicy policy = kMustContainOrigin) const = delete;

  // Brings a given wheel vector into the wheel-space and arc-space
  // box constraints by scaling it down towards the origin. The given
  // kinematics object is used to map between the two spaces.
  bool BringInBounds(const Kinematics &kinematics, const WheelVector &wheel,
                     WheelVector *wheel_out,
                     BringInBoundsPolicy policy = kMustContainOrigin) const;

  // Brings a given arc vector into the wheel-space and arc-space
  // box constraints by scaling it down towards the origin. The given
  // kinematics object is used to map between the two spaces.
  bool BringInBounds(const Kinematics &kinematics, const ArcVector &arc,
                     ArcVector *arc_out,
                     BringInBoundsPolicy policy = kMustContainOrigin) const;

  // Remove an ambiguous overload.
  bool BringInBounds(
      const Kinematics &kinematics, const eigenmath::Vector2d &unknown,
      eigenmath::Vector2d *unknown_out,
      BringInBoundsPolicy policy = kMustContainOrigin) const = delete;

  // Get the minimum vector in wheel-space.
  const WheelVector &MinWheelVector() const { return min_wheel_; }

  // Get the maximum vector in wheel-space.
  const WheelVector &MaxWheelVector() const { return max_wheel_; }

  // Get the minimum vector in arc-space.
  const ArcVector &MinArcVector() const { return min_arc_; }

  // Get the maximum vector in arc-space.
  const ArcVector &MaxArcVector() const { return max_arc_; }

  // Translate the limits by a given amount in wheel-space and arc-space.
  void TranslateLimits(const WheelVector &wheel_center,
                       const ArcVector &arc_center) {
    min_wheel_ += wheel_center;
    max_wheel_ += wheel_center;
    min_arc_ += arc_center;
    max_arc_ += arc_center;
  }

  // Remove an ambiguous overload.
  void TranslateLimits(const eigenmath::Vector2d &unknown1,
                       const eigenmath::Vector2d &unknown2) = delete;

  // Translate the limits by a given amount in wheel-space, using the
  // given kinematics object to obtain the corresponding arc-space vector.
  void TranslateLimits(const Kinematics &kinematics,
                       const WheelVector &wheel_center) {
    const ArcVector arc_center =
        kinematics.ComputeForwardKinematics(wheel_center);
    TranslateLimits(wheel_center, arc_center);
  }

  // Translate the limits by a given amount in arc-space, using the
  // given kinematics object to obtain the corresponding wheel-space vector.
  void TranslateLimits(const Kinematics &kinematics,
                       const ArcVector &arc_center) {
    const WheelVector wheel_center =
        kinematics.ComputeInverseKinematics(arc_center);
    TranslateLimits(wheel_center, arc_center);
  }

  // Remove an ambiguous overload.
  void TranslateLimits(const Kinematics &kinematics,
                       const eigenmath::Vector2d &unknown) = delete;

  // Scale the limits by a given scale factor.
  void ScaleLimits(double scale_factor) {
    min_wheel_ *= scale_factor;
    max_wheel_ *= scale_factor;
    min_arc_ *= scale_factor;
    max_arc_ *= scale_factor;
  }

  // Scales the Cartesian limits by a given scale factor in translation and
  // rotation. The wheel limits are kept the same.
  void ScaleCartesianLimits(double translation_factor, double rotation_factor,
                            bool forbid_backwards_driving = false) {
    if (forbid_backwards_driving) {
      // If backwards driving is forbidden, given an epsilon backward limit
      // to keep containing the origin within limits.
      min_arc_.Translation() = -kEpsilon;
    } else {
      min_arc_.Translation() *= translation_factor;
    }
    min_arc_.Rotation() *= rotation_factor;
    max_arc_.Translation() *= translation_factor;
    max_arc_.Rotation() *= rotation_factor;
  }

  // Compute the intersection between two velocity limits objects
  static BoxConstraints Intersect(const BoxConstraints &first,
                                  const BoxConstraints &second);

  // Compute the intersection between two velocity limits objects
  // and if there is no intersection, grow the second bounds until there is.
  static BoxConstraints GrowToIntersect(const BoxConstraints &first,
                                        const BoxConstraints &second_growable);

  // Reverses the constraints for reverse driving.
  static BoxConstraints ReverseConstraints(const BoxConstraints &constraint);

 private:
  // This epsilon allows for a few conversions that each accumulate 1 bit of
  // round-off error.
  static constexpr double kEpsilon =
      4.0 * std::numeric_limits<double>::epsilon();

  static bool ContainsOrigin(const eigenmath::Vector2d &min_vector,
                             const eigenmath::Vector2d &max_vector) {
    return (min_vector(0) <= kEpsilon) && (min_vector(1) <= kEpsilon) &&
           (max_vector(0) >= -kEpsilon) && (max_vector(1) >= -kEpsilon);
  }

  // These functions scale the input vector to bring the
  // vector into the given bounds. This function will scale to the boundary
  // whether the boundaries are violated to not, so, it should only be called
  // after checking that the bounds are being violated.
  static bool BringInBounds(const eigenmath::Vector2d &min_vector,
                            const eigenmath::Vector2d &max_vector,
                            const eigenmath::Vector2d &vector_in,
                            eigenmath::Vector2d *vector_out);

  static bool BringInBoundsPreserveDirection(
      const eigenmath::Vector2d &min_vector,
      const eigenmath::Vector2d &max_vector,
      const eigenmath::Vector2d &vector_in, eigenmath::Vector2d *vector_out);

  WheelVector min_wheel_;
  WheelVector max_wheel_;
  ArcVector min_arc_;
  ArcVector max_arc_;
};

class DynamicLimits {
 public:
  DynamicLimits() = default;

  DynamicLimits(const Kinematics &kinematics,
                const WheelVector &max_wheel_velocity,
                const WheelVector &max_wheel_acceleration,
                const ArcVector &max_arc_velocity,
                const ArcVector &max_arc_acceleration);

  DynamicLimits(const Kinematics &kinematics,
                const BoxConstraints &velocity_limits,
                const BoxConstraints &acceleration_limits)
      : DynamicLimits(kinematics, velocity_limits.MinWheelVector(),
                      velocity_limits.MaxWheelVector(),
                      acceleration_limits.MinWheelVector(),
                      acceleration_limits.MaxWheelVector(),
                      velocity_limits.MinArcVector(),
                      velocity_limits.MaxArcVector(),
                      acceleration_limits.MinArcVector(),
                      acceleration_limits.MaxArcVector()) {}

  DynamicLimits(const Kinematics &kinematics,
                const WheelVector &min_wheel_velocity,
                const WheelVector &max_wheel_velocity,
                const WheelVector &min_wheel_acceleration,
                const WheelVector &max_wheel_acceleration,
                const ArcVector &min_arc_velocity,
                const ArcVector &max_arc_velocity,
                const ArcVector &min_arc_acceleration,
                const ArcVector &max_arc_acceleration);

  static DynamicLimits Unlimited(const Kinematics &kinematics) {
    return DynamicLimits(kinematics, BoxConstraints::Unlimited(),
                         BoxConstraints::Unlimited());
  }

  const Kinematics &GetKinematics() const { return kinematics_; }
  const BoxConstraints &VelocityLimits() const { return velocity_limits_; }
  BoxConstraints *MutableVelocityLimits() { return &velocity_limits_; }
  const BoxConstraints &AccelerationLimits() const {
    return acceleration_limits_;
  }
  BoxConstraints *MutableAccelerationLimits() { return &acceleration_limits_; }
  const WheelVector &MaxWheelVelocity() const {
    return velocity_limits_.MaxWheelVector();
  }
  const ArcVector &MinArcVelocity() const {
    return velocity_limits_.MinArcVector();
  }
  const ArcVector &MaxArcVelocity() const {
    return velocity_limits_.MaxArcVector();
  }
  const WheelVector &MaxWheelAcceleration() const {
    return acceleration_limits_.MaxWheelVector();
  }
  const ArcVector &MinArcAcceleration() const {
    return acceleration_limits_.MinArcVector();
  }
  const ArcVector &MaxArcAcceleration() const {
    return acceleration_limits_.MaxArcVector();
  }

  // This computes the forward, backward and side safety distances based on
  // the assumption that no obstacle around the robot is going faster than
  // the robot itself.
  // The reaction time should reflect the expected maximum latency between
  // physical sensor hits on obstacles and when this information reaches the
  // local planner (or other obstacle avoidance module).
  // The calculations of the safety distances boil down to adding up the
  // relative distance covered by the obstacle and robot during the reaction
  // time as well as the stopping distance of the robot at maximum speed.
  void ComputeSafetyDistances(double reaction_time, double *dist_forward,
                              double *dist_backward,
                              double *dist_sideways) const;

  // Reverse limits for reverse driving to allow reusing trajectory generation
  // primitives.
  static DynamicLimits ReverseLimits(const DynamicLimits &limits);

 private:
  Kinematics kinematics_;
  BoxConstraints velocity_limits_;
  BoxConstraints acceleration_limits_;
};

// Helper class for applying velocity and acceleration constraints to
// a desired velocity.
class DynamicLimitsFilter {
 public:
  // Simple constructor, turns acceleration limits into
  // velocity-increment limits based on the given cycle_duration. For
  // more fine-grained control, see the other ctor.
  DynamicLimitsFilter(const DynamicLimits &limits, double cycle_duration)
      : limits_(limits),
        absolute_constraints_(limits.VelocityLimits()),
        relative_constraints_(limits.AccelerationLimits()) {
    relative_constraints_.ScaleLimits(cycle_duration);
  }

  // The velocity_scale is applied to the VelocityLimits() of the
  // given DynamicLimits object. Use velocity_scale=1.0 to maintain
  // the same limits.
  //
  // The AccelerationLimits() of the DynamicLimits object are
  // multiplied by the aggressivity, which allows to soften
  // them.
  //
  // Cycle_duration has the same meaning as for the simple
  // ctor. Passing velocity_scale==aggressivity==1.0 is equivalent to
  // using the simple ctor.
  DynamicLimitsFilter(const DynamicLimits &limits, double velocity_scale,
                      double aggressivity, double cycle_duration)
      : limits_(limits),
        absolute_constraints_(limits.VelocityLimits()),
        relative_constraints_(limits.AccelerationLimits()) {
    absolute_constraints_.ScaleLimits(velocity_scale);
    relative_constraints_.ScaleLimits(aggressivity * cycle_duration);
  }

  const DynamicLimits &GetLimits() const { return limits_; }

  // Filter a wheel or arc velocity through the dynamic limits, such
  // that the computed clamped value never exceeds the
  // VelocityLimits() or AccelerationLimits() of the underlying
  // DynamicLimits instance.
  //
  // Returns false when an error occurs.
  template <typename VectorType>
  bool BringInBounds(const VectorType &current, const VectorType &desired,
                     VectorType *clamped) const {
    VectorType clamped_desired;
    if (!absolute_constraints_.BringInBounds(desired, &clamped_desired)) {
      return false;
    }
    VectorType clamped_increment;
    if (!relative_constraints_.BringInBounds(limits_.GetKinematics(),
                                             clamped_desired - current,
                                             &clamped_increment)) {
      return false;
    }
    *clamped = current + clamped_increment;
    return true;
  }

 private:
  const DynamicLimits &limits_;
  BoxConstraints absolute_constraints_;
  BoxConstraints relative_constraints_;
};

}  // namespace mobility::diff_drive

#endif  // MOBILITY_DIFF_DRIVE_DIFF_DRIVE_DYNAMIC_LIMITS_H_
