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

#ifndef MOBILITY_DIFF_DRIVE_DIFF_DRIVE_TRANSFORMED_CURVE_VIEW_H_
#define MOBILITY_DIFF_DRIVE_DIFF_DRIVE_TRANSFORMED_CURVE_VIEW_H_

#include "diff_drive/continuous_curve.h"

namespace mobility::diff_drive {

// Creates a view of a differential-drive continuous curve to which a given
// transform and cord-length shift can be applied without modifying the memory
// representation of the base curve.
// This class performs the transformations on-demand, which means that there
// will be an additional overhead to each evaluation of a curve point, and so,
// if evaluations are frequent and memory is not a constraint, it might be
// preferable to copy-transform the underlying curve instead.
class TransformedCurveView : public ContinuousCurve {
 public:
  explicit TransformedCurveView(
      const ContinuousCurve* underlying_curve,
      const eigenmath::Pose2d& transform = eigenmath::Pose2d::Identity(),
      double cord_length_shift = 0.0)
      : underlying_curve_(underlying_curve),
        transform_(transform),
        cord_length_shift_(cord_length_shift) {}

  TransformedCurveView(const ContinuousCurve* underlying_curve,
                       double cord_length_shift)
      : TransformedCurveView(underlying_curve, eigenmath::Pose2d::Identity(),
                             cord_length_shift) {}

  // Checks if the curve is empty.
  bool IsEmpty() const override { return underlying_curve_->IsEmpty(); }

  // Checks if the curve has enough points to do anything with it.
  bool IsSane() const override { return underlying_curve_->IsSane(); }

  // Checks that the curve has no discontinuities in position.
  bool HasContinuousPosition() const override {
    return underlying_curve_->HasContinuousPosition();
  }

  // Evaluates the curve point at a given cord-length.
  // Trajectory must have IsSane() == true
  CurvePoint Evaluate(double s) const override {
    CurvePoint result = underlying_curve_->Evaluate(s - cord_length_shift_);
    result.SetPose(transform_ * result.GetPose());
    return result;  // NRVO
  }

  // Get the cord-length interval over which the curve exists.
  Interval<double> GetCordLengthSpan() const override {
    Interval<double> result = underlying_curve_->GetCordLengthSpan();
    result.SetMin(result.min() + cord_length_shift_);
    result.SetMax(result.max() + cord_length_shift_);
    return result;  // NRVO
  }

  // Gets the first valid curve-point in this curve.
  CurvePtAndCord GetStart() const override {
    CurvePtAndCord result = underlying_curve_->GetStart();
    result.cord_length += cord_length_shift_;
    result.point.SetPose(transform_ * result.point.GetPose());
    return result;  // NRVO
  }
  // Gets the last valid curve-point in this curve.
  CurvePtAndCord GetFinish() const override {
    CurvePtAndCord result = underlying_curve_->GetFinish();
    result.cord_length += cord_length_shift_;
    result.point.SetPose(transform_ * result.point.GetPose());
    return result;  // NRVO
  }

  // Apply a cord-length shift to all the points of the curve.
  void ApplyCordLengthShift(double ds) override { cord_length_shift_ += ds; }

  // Apply a transform to all the points of the curve.
  void ApplyTransform(const eigenmath::Pose2d& new_pose_old) override {
    transform_ = new_pose_old * transform_;
  }

  // Finds the curve point and its cord-length that is closest to a given point.
  CurvePtAndCord FindClosestMatchToPoint(const eigenmath::Vector2d& pt,
                                         double* min_distance) const override {
    const eigenmath::Vector2d underlying_pt = transform_.inverse() * pt;
    CurvePtAndCord result =
        underlying_curve_->FindClosestMatchToPoint(underlying_pt, min_distance);
    result.cord_length += cord_length_shift_;
    result.point.SetPose(transform_ * result.point.GetPose());
    return result;  // NRVO
  }

 private:
  const ContinuousCurve* underlying_curve_;
  eigenmath::Pose2d transform_;
  double cord_length_shift_;
};

}  // namespace mobility::diff_drive

#endif  // MOBILITY_DIFF_DRIVE_DIFF_DRIVE_TRANSFORMED_CURVE_VIEW_H_
