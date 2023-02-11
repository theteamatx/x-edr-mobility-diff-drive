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

#ifndef MOBILITY_DIFF_DRIVE_DIFF_DRIVE_STATE_H_
#define MOBILITY_DIFF_DRIVE_DIFF_DRIVE_STATE_H_

#include "diff_drive/kinematics.h"
#include "diff_drive/type_aliases.h"

namespace mobility::diff_drive {

// Represents the state of a differential-drive robot, including its 2d pose
// and its wheel / arc velocity vectors.
class State {
 public:
  // Default constructor. Leaves all data members default-initialized.
  State() = default;

  // Create a diff-drive state as a static pose.
  explicit State(const eigenmath::Pose2d &pose)
      : pose_(pose), arc_velocity_(eigenmath::Vector2d::Zero()) {}

  // Create a diff-drive state from a given pose and a arc velocity vector.
  State(const eigenmath::Pose2d &pose, const ArcVector &arc_velocity)
      : pose_(pose), arc_velocity_(arc_velocity) {}

  // Get a const-reference to the pose of the robot.
  const eigenmath::Pose2d &GetPose() const { return pose_; }
  // Sets the pose of the robot.
  void SetPose(const eigenmath::Pose2d &pose) { pose_ = pose; }

  // Re-normalize the rotation component (cos, sin) of the pose of this state.
  // This is to get rid of roundoff errors after many pose multiplications.
  void NormalizeRotation() { pose_.so2().normalize(); }

  // Get a const-reference to the arc velocity vector.
  const ArcVector &GetArcVelocity() const { return arc_velocity_; }

  // Sets the velocity vectors of the robot through a given arc velocity
  // vector and a kinematics object to compute the wheel velocity.
  void SetArcVelocity(const ArcVector &arc_velocity) {
    arc_velocity_ = arc_velocity;
  }

  // Extrapolates from the robot's stored state by assuming a constant
  // velocity arc-motion over a given delta-time.
  State ExtrapolateConstantVelocityArc(double dt) const {
    const eigenmath::Pose2d rel_motion =
        Kinematics::ComputeIncrement(ArcVector(dt * arc_velocity_));
    return State(pose_ * rel_motion, arc_velocity_);
  }

  // Get this state expressed in coordinates relative to a given parent state
  // and with relative velocities.
  State GetRelativeTo(const State &parent) const {
    State result;
    result.SetPose(parent.GetPose().inverse() * pose_);
    const double dir_cos = result.GetPose().so2().cos_angle();
    const auto &parent_vel = parent.GetArcVelocity();
    result.SetArcVelocity(ArcVector(
        arc_velocity_.Translation() - parent_vel.Translation() * dir_cos,
        arc_velocity_.Rotation() - parent_vel.Rotation()));
    result.NormalizeRotation();
    return result;  // NRVO
  }

 private:
  eigenmath::Pose2d pose_;
  ArcVector arc_velocity_;
};

// Represents the state of a differential-drive robot and the time at which
// the robot is at that state.
class StateAndTime {
 public:
  double time;
  State state;

  // This comparator class is used when searching or sorting state-time points
  // according to time. This is a less-than comparison (per C++ convention).
  class TimeComparator {
   public:
    bool operator()(const StateAndTime &lhs, const StateAndTime &rhs) const {
      return lhs.time < rhs.time;
    }
    bool operator()(const StateAndTime &pt, double t) const {
      return pt.time < t;
    }
    bool operator()(double t, const StateAndTime &pt) const {
      return t < pt.time;
    }
  };
};

}  // namespace mobility::diff_drive

#endif  // MOBILITY_DIFF_DRIVE_DIFF_DRIVE_STATE_H_
