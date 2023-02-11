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

#ifndef MOBILITY_DIFF_DRIVE_DIFF_DRIVE_KINEMATICS_H_
#define MOBILITY_DIFF_DRIVE_DIFF_DRIVE_KINEMATICS_H_

#include <limits>
#include <memory>

#include "diff_drive/type_aliases.h"

namespace mobility::diff_drive {

// Kinematics for a differentially driven robot.
class Kinematics {
 public:
  Kinematics() = default;

  // Create a Kinematics from given parameters.
  // Asserts that the wheel radius or base are positive.
  Kinematics(const WheelVector &wheel_radius, double wheel_base);

  // Change the underlying parameters.
  // Asserts that the wheel radius or base are positive.
  void Configure(const WheelVector &wheel_radius, double wheel_base);

  // Compute the pose change induced by given wheel position
  // changes. The changes can be deltas or velocities (or
  // accelerations etc for that matter).
  //
  // NOTE: This method's signature is most useful for velocities. For
  // position changes, the ComputeIncrement() method is more
  // convenient.
  //
  // Returns the resulting motion (i.e. velocity or positional
  // increment). ArcVector::translation contains the curvilinear
  // displacement, and ArcVector::rotation the heading angle change.
  ArcVector ComputeForwardKinematics(
      // velocity (or position increment) of drive wheels
      const WheelVector &delta) const;

  // Remove ambiguous overload.
  ArcVector ComputeForwardKinematics(
      const eigenmath::Vector2d &unknown_delta) const = delete;

  // Convenience method. Forwards to the other ComputeForwardKinematics().
  ArcVector ComputeForwardKinematics(
      // velocity (or position increment) of left wheel
      double delta_l,
      // velocity (or position increment) of right wheel
      double delta_r) const {
    return ComputeForwardKinematics(WheelVector{delta_l, delta_r});
  }

  // Compute the wheel position changes (or velocity) that correspond
  // to a given desired pose change of the robot.
  //
  // Returns the wheel motion (i.e. velocity or positional
  // increment) that achieves the given planar motion.
  WheelVector ComputeInverseKinematics(
      // translational and rotation velocity (or increment) of the robot
      ArcVector delta) const;

  // Remove ambiguous overload.
  WheelVector ComputeInverseKinematics(
      const eigenmath::Vector2d &unknown_delta) const = delete;

  // Convenience method. Forwards to the other ComputeInverseKinematics().
  WheelVector ComputeInverseKinematics(
      // translational velocity (or curvilinear position increment) of the robot
      double delta_trans,
      // rotational velocity (or heading angle increment) of the robot
      double delta_rot) const {
    return ComputeInverseKinematics(ArcVector{delta_trans, delta_rot});
  }

  // Compute the ("local") pose increment due to a given wheel
  // position increment.
  //
  // NOTE This method is most convenient for position increments. If
  // you are interested in velocities, ComputeForwardKinematics() may
  // suit you better.
  //
  // Returns the pose where the robot ends up, expressed relative to
  // the old pose.
  eigenmath::Pose2d ComputeIncrement(
      // position increments of drive wheels
      const WheelVector &delta,
      // (out-parameter) curvilinear distance travelled due to delta_l and
      // delta_r; pass a nullptr if you do not need this info.
      double *delta_trans) const;

  // Compute the ("local") pose increment due to a given arc motion.
  //
  // Returns the pose where the robot ends up, expressed relative to
  // the old pose.
  static eigenmath::Pose2d ComputeIncrement(
      // Total arc motion (interpreted as movement along an arc of circle).
      const ArcVector &arc);

  const WheelVector &GetWheelRadius() const { return wheel_radius_; }
  double GetWheelBase() const { return wheel_base_; }

 protected:
  WheelVector wheel_radius_ = {1.0, 1.0};
  double wheel_base_ = 1.0;
};

// The following functions are inlined because they are rather simple and
// often used in hot loops:

inline ArcVector Kinematics::ComputeForwardKinematics(
    const WheelVector &delta) const {
  const WheelVector delta_trans = {wheel_radius_.Left() * delta.Left(),
                                   wheel_radius_.Right() * delta.Right()};
  ArcVector motion;
  motion.Translation() = (delta_trans.Left() + delta_trans.Right()) / 2.0;
  motion.Rotation() = (delta_trans.Right() - delta_trans.Left()) / wheel_base_;
  return motion;
}

inline WheelVector Kinematics::ComputeInverseKinematics(ArcVector delta) const {
  const double delta_diff = delta.Rotation() * wheel_base_ / 2.0;
  WheelVector motion;
  motion.Left() = (delta.Translation() - delta_diff) / wheel_radius_.Left();
  motion.Right() = (delta.Translation() + delta_diff) / wheel_radius_.Right();
  return motion;
}

inline eigenmath::Pose2d Kinematics::ComputeIncrement(
    const WheelVector &delta, double *delta_trans) const {
  auto const motion = ComputeForwardKinematics(delta);
  if (nullptr != delta_trans) {
    *delta_trans = motion.Translation();
  }
  return ComputeIncrement(motion);
}

inline eigenmath::Pose2d Kinematics::ComputeIncrement(const ArcVector &arc) {
  eigenmath::Pose2d delta_pose;
  delta_pose.setAngle(arc.Rotation());
  if (std::abs(arc.Rotation()) < std::numeric_limits<double>::epsilon()) {
    // straight-line
    delta_pose.translation().x() = arc.Translation();
    delta_pose.translation().y() = 0.0;
  } else {  // circular arc
    const double arc_radius = arc.Translation() / arc.Rotation();
    delta_pose.translation().x() = arc_radius * delta_pose.so2().sin_angle();
    delta_pose.translation().y() =
        arc_radius * (1.0 - delta_pose.so2().cos_angle());
  }

  return delta_pose;
}

}  // namespace mobility::diff_drive

#endif  // MOBILITY_DIFF_DRIVE_DIFF_DRIVE_KINEMATICS_H_
