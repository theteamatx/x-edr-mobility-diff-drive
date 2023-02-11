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

#ifndef MOBILITY_DIFF_DRIVE_DIFF_DRIVE_CURVE_POINT_H_
#define MOBILITY_DIFF_DRIVE_DIFF_DRIVE_CURVE_POINT_H_

#include "diff_drive/kinematics.h"
#include "diff_drive/type_aliases.h"

namespace mobility::diff_drive {

// Represents a curve-point for a differential-drive robot, including
// its 2d pose and local curvature of the curve (piece-wise constant curvature).
class CurvePoint {
 public:
  // Default constructor. Leaves data members in undefined values.
  CurvePoint() = default;

  // Create a diff-drive curve-point as a straight-forward pose.
  explicit CurvePoint(const eigenmath::Pose2d &pose)
      : pose_(pose), curvature_(0.0) {}

  // Create a diff-drive curve-point from a pose and curvature.
  explicit CurvePoint(const eigenmath::Pose2d &pose, double curvature)
      : pose_(pose), curvature_(curvature) {}

  // Get a const-reference to the robot pose at this curve point.
  const eigenmath::Pose2d &GetPose() const { return pose_; }
  // Sets the robot pose at this curve point.
  void SetPose(const eigenmath::Pose2d &pose) { pose_ = pose; }

  // Get the curvature of the path of the robot at this curve point.
  double GetCurvature() const { return curvature_; }

  // Sets the curvature of the path of the robot at this curve point.
  void SetCurvature(double curvature) { curvature_ = curvature; }

  // Extrapolates from the robot's stored curve point by assuming a constant
  // curvature arc-motion over a given travel distance.
  CurvePoint ExtrapolateConstantArc(double ds) const {
    const eigenmath::Pose2d rel_motion =
        Kinematics::ComputeIncrement(ArcVector(ds, ds * curvature_));
    return CurvePoint(pose_ * rel_motion, curvature_);
  }

 private:
  eigenmath::Pose2d pose_;
  double curvature_;
};

// Represents a curve point for a differential-drive robot along with a
// cord-length value marking its position along a curve, parametrized by
// the cord-length of the curve.
class CurvePtAndCord {
 public:
  double cord_length;
  CurvePoint point;

  // This comparator class is used when searching or sorting curve-points
  // according to their cord-length parameter. This is a less-than
  // comparison (per C++ convention).
  class CordLengthComparator {
   public:
    bool operator()(const CurvePtAndCord &lhs,
                    const CurvePtAndCord &rhs) const {
      return lhs.cord_length < rhs.cord_length;
    }
    bool operator()(const CurvePtAndCord &pt, double s) const {
      return pt.cord_length < s;
    }
    bool operator()(double s, const CurvePtAndCord &pt) const {
      return s < pt.cord_length;
    }
  };
};

}  // namespace mobility::diff_drive

#endif  // MOBILITY_DIFF_DRIVE_DIFF_DRIVE_CURVE_POINT_H_
