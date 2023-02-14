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

#ifndef MOBILITY_DIFF_DRIVE_DIFF_DRIVE_CONTINUOUS_CURVE_H_
#define MOBILITY_DIFF_DRIVE_DIFF_DRIVE_CONTINUOUS_CURVE_H_

#include "diff_drive/curve_point.h"
#include "diff_drive/interval.h"
#include "diff_drive/type_aliases.h"

namespace mobility::diff_drive {

// Represents a differential-drive continuous curve that is parametrized
// by the cord-length.
class ContinuousCurve {
 public:
  // Holds the distance tolerance to consider two points to be identical.
  static constexpr double kIdenticalPointsEpsilon = 1e-4;

  virtual ~ContinuousCurve() {}

  // Checks if the curve is empty.
  virtual bool IsEmpty() const { return GetCordLengthSpan().Empty(); }

  // Checks if the curve has enough points to do anything with it.
  virtual bool IsSane() const { return true; }

  // Checks that the curve has no discontinuities in position.
  virtual bool HasContinuousPosition() const = 0;

  // Evaluates the curve point at a given cord-length.
  // Trajectory must have IsSane() == true
  virtual CurvePoint Evaluate(double s) const = 0;

  // Get the cord-length interval over which the curve exists.
  virtual Interval<double> GetCordLengthSpan() const = 0;

  // Gets the first valid curve-point in this curve.
  virtual CurvePtAndCord GetStart() const {
    const double start_cord = GetCordLengthSpan().min();
    return CurvePtAndCord{start_cord, Evaluate(start_cord)};
  }
  // Gets the last valid curve-point in this curve.
  virtual CurvePtAndCord GetFinish() const {
    const double finish_cord = GetCordLengthSpan().max();
    return CurvePtAndCord{finish_cord, Evaluate(finish_cord)};
  }

  // Apply a cord-length shift to all the points of the curve.
  virtual void ApplyCordLengthShift(double ds) = 0;

  // Apply a transform to all the points of the curve.
  virtual void ApplyTransform(const eigenmath::Pose2d& new_pose_old) = 0;

  // Finds the curve point and its cord-length that is closest to a given point.
  virtual CurvePtAndCord FindClosestMatchToPoint(
      const eigenmath::Vector2d& pt, double* min_distance) const = 0;
};

}  // namespace mobility::diff_drive

#endif  // MOBILITY_DIFF_DRIVE_DIFF_DRIVE_CONTINUOUS_CURVE_H_
