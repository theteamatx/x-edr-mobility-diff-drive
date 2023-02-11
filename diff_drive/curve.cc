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

#include "diff_drive/curve.h"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iterator>
#include <limits>

#include "genit/adjacent_iterator.h"
#include "absl/log/check.h"

namespace mobility::diff_drive {

namespace {
constexpr double kEpsilon = 1.0e-3;
constexpr double kTenNanoMeters = 1.0e-8;  // 10 nanometers
}  // namespace

Curve::Curve() : points_(kDefaultCapacity), points_count_(0) {}

Curve::Curve(int capacity) : points_(capacity), points_count_(0) {}

bool Curve::operator==(const Curve &rhs) const {
  if (GetSize() != rhs.GetSize()) {
    return false;
  }
  auto it_pair = std::mismatch(
      points_.begin(), points_.begin() + points_count_, rhs.points_.begin(),
      [](const CurvePtAndCord &lhs, const CurvePtAndCord &rhs) -> bool {
        if (std::abs(lhs.cord_length - rhs.cord_length) > kTenNanoMeters) {
          return false;
        }
        const eigenmath::Pose2d pose_difference =
            lhs.point.GetPose().inverse() * rhs.point.GetPose();
        if (pose_difference.translation().squaredNorm() > kEpsilon * kEpsilon ||
            std::abs(pose_difference.so2().sin_angle()) > kEpsilon) {
          return false;
        }
        return true;
      });
  return (it_pair.first == points_.begin() + points_count_);
}

bool Curve::HasContinuousPosition() const {
  for (const auto segment :
       genit::AdjacentElementsRange<2>(GetCurvePointIteratorRange())) {
    const CurvePoint from_previous_pt = segment[0].point.ExtrapolateConstantArc(
        segment[1].cord_length - segment[0].cord_length);
    const eigenmath::Vector2d position_difference =
        from_previous_pt.GetPose().translation() -
        segment[1].point.GetPose().translation();
    if (position_difference.squaredNorm() > kEpsilon * kEpsilon) {
      return false;
    }
  }
  return true;
}

bool Curve::HasMonotonicCordLengths() const {
  for (const auto segment :
       genit::AdjacentElementsRange<2>(GetCurvePointIteratorRange())) {
    if (segment[1].cord_length < segment[0].cord_length + kTenNanoMeters) {
      return false;
    }
  }
  return true;
}

double Curve::ComputeRelativeCordLength() const {
  const double cord_length = std::max(kEpsilon, GetCordLengthSpan().Length());
  const double line_length =
      std::max(kEpsilon, (GetFinish().point.GetPose().translation() -
                          GetStart().point.GetPose().translation())
                             .norm());
  return cord_length / line_length;
}

double Curve::GetMaxCurvature() const {
  double result = 0.0;
  for (auto &pt : GetCurvePointIteratorRange()) {
    const double pt_kappa = std::abs(pt.point.GetCurvature());
    if (result < pt_kappa) {
      result = pt_kappa;
    }
  }
  return result;
}

CurvePoint Curve::Evaluate(double s) const {
  // Assume that the curve is non-empty.
  s = std::clamp(s, points_[0].cord_length,
                 points_[points_count_ - 1].cord_length);
  // Find the pair of points bracketing s in [s0, s1]
  auto it_s1 =
      std::upper_bound(points_.begin(), points_.begin() + points_count_, s,
                       CurvePtAndCord::CordLengthComparator());
  if (it_s1 == points_.begin()) {
    return it_s1->point.ExtrapolateConstantArc(s - it_s1->cord_length);
  }
  auto it_s0 = std::prev(it_s1);
  // Evaluate a constant arc extrapolation from s0 to s
  auto ds = s - it_s0->cord_length;
  return it_s0->point.ExtrapolateConstantArc(ds);
}

bool Curve::AddPoint(double s, const CurvePoint &new_point) {
  if (points_count_ == points_.size() ||
      ((points_count_ > 0) && (points_[points_count_ - 1].cord_length >= s))) {
    return false;
  }
  points_[points_count_++] = CurvePtAndCord{s, new_point};
  return true;
}

bool Curve::AppendCurve(const Curve &tail) {
  for (auto &pt : tail.GetCurvePointIteratorRange()) {
    if (points_count_ > 0 &&
        (pt.cord_length <
         (points_[points_count_ - 1].cord_length - kTenNanoMeters))) {
      continue;
    }
    if (points_count_ == points_.size()) {
      return false;
    }
    if (points_count_ > 0 &&
        pt.cord_length <
            points_[points_count_ - 1].cord_length + kTenNanoMeters) {
      points_[points_count_ - 1] = pt;
    } else {
      points_[points_count_++] = pt;
    }
  }
  return true;
}

bool Curve::PrependCurve(const Curve &head) {
  if (head.IsEmpty()) {
    return true;
  }
  if (head.GetSize() + GetSize() > GetCapacity() + 1) {
    return false;
  }
  if (IsEmpty()) {
    return AppendCurve(head);
  }
  // Neither curves are empty, check that they match up.
  const eigenmath::Pose2d pose_difference =
      head.GetFinish().point.GetPose().inverse() * GetStart().point.GetPose();
  if (pose_difference.translation().squaredNorm() > kEpsilon * kEpsilon ||
      std::abs(pose_difference.so2().sin_angle()) > kEpsilon) {
    return false;
  }
  if (head.GetSize() == 1) {
    return true;
  }
  ApplyCordLengthShift(head.GetCordLengthSpan().max() - points_[0].cord_length);
  std::copy_backward(points_.begin(), points_.begin() + points_count_,
                     points_.begin() + points_count_ + head.GetSize() - 1);
  std::copy(head.BeginCurvePoint(), std::prev(head.EndCurvePoint()),
            points_.begin());
  points_count_ += head.GetSize() - 1;
  return true;
}

void Curve::TruncateTo(double final_cord_length) {
  TruncateTo(Interval<double>(points_[0].cord_length, final_cord_length));
}

void Curve::TruncateTo(const Interval<double> &cord_length_interval) {
  // Find the pair of points bracketing s_max in [s0, s1]
  auto it_s1 =
      std::upper_bound(points_.begin(), points_.begin() + points_count_,
                       cord_length_interval.max() - kTenNanoMeters,
                       CurvePtAndCord::CordLengthComparator());
  if (it_s1 == points_.begin()) {
    Clear();
    return;
  }
  auto it_s0 = std::prev(it_s1);
  // Evaluate a constant arc extrapolation from s0 to s
  const double last_ds = cord_length_interval.max() - it_s0->cord_length;
  CurvePoint last_pt = it_s0->point.ExtrapolateConstantArc(last_ds);
  // Remove tail, and add new finish point.
  points_count_ = it_s1 - points_.begin();
  AddPoint(cord_length_interval.max(), last_pt);

  // Find the pair of points bracketing s_min in [s0, s1]
  it_s1 = std::upper_bound(points_.begin(), points_.begin() + points_count_,
                           cord_length_interval.min() + kTenNanoMeters,
                           CurvePtAndCord::CordLengthComparator());
  if (it_s1 == points_.begin() + points_count_) {
    Clear();
    return;
  }
  it_s0 = it_s1;
  if (it_s1 != points_.begin()) {
    it_s0 = std::prev(it_s1);
  }
  // Evaluate a constant arc extrapolation from s0 to s
  const double first_ds = cord_length_interval.min() - it_s0->cord_length;
  CurvePoint first_pt = it_s0->point.ExtrapolateConstantArc(first_ds);
  // Remove the head of the curve, by copying points [it_s1, end) to start.
  if (it_s1 != points_.begin()) {
    std::copy(it_s1, points_.begin() + points_count_, points_.begin() + 1);
    points_count_ = ((points_.begin() + points_count_) - it_s1) + 1;
  }
  // Update the start point:
  points_.begin()->cord_length = cord_length_interval.min();
  points_.begin()->point = first_pt;
}

void Curve::CutCorners() {
  CutCornersUnlessPredicateFails(
      std::numeric_limits<double>::max(),
      [](const CurvePoint & /*unused*/) { return true; });
}

void Curve::ApplyCordLengthShift(double ds) {
  for (auto it = points_.begin(), it_end = points_.begin() + points_count_;
       it != it_end; ++it) {
    it->cord_length += ds;
  }
}

void Curve::ApplyTransform(const eigenmath::Pose2d &new_pose_old) {
  for (auto it = points_.begin(), it_end = points_.begin() + points_count_;
       it != it_end; ++it) {
    it->point.SetPose(new_pose_old * it->point.GetPose());
  }
}

Curve::CordLengthIterator &Curve::CordLengthIterator::operator+=(double ds) {
  // Do not allow backward cord-length-iteration:
  CHECK_GE(ds, 0.0) << "Cannot iterate backward along curve";
  current_cord_ = std::min(current_cord_ + ds, max_cord_);

  while (current_it_->cord_length < max_cord_ &&
         current_cord_ >= std::next(current_it_)->cord_length) {
    ++current_it_;
  }
  return *this;
}

Curve::CurvePointIterator Curve::FindClosestKnotPointMatch(
    const eigenmath::Vector2d &pt, double *min_distance) const {
  double min_dist_sqr = std::numeric_limits<double>::max();
  auto curve_pts = GetCurvePointIteratorRange();
  auto best_it = curve_pts.begin();
  for (auto curve_it = curve_pts.begin(); curve_it < curve_pts.end();
       ++curve_it) {
    const double dist_value_sqr =
        (pt - curve_it->point.GetPose().translation()).squaredNorm();
    if (dist_value_sqr < min_dist_sqr) {
      min_dist_sqr = dist_value_sqr;
      best_it = curve_it;
    }
  }
  if (min_distance != nullptr) {
    *min_distance = std::sqrt(min_dist_sqr);
  }
  return best_it;
}

CurvePtAndCord Curve::FindClosestMatchToPoint(const eigenmath::Vector2d &pt,
                                              double *min_distance) const {
  double min_dist;
  auto best_it = FindClosestKnotPointMatch(pt, &min_dist);
  double cord_length_at_match = best_it->cord_length;
  auto curve_pts = GetCurvePointIteratorRange();
  if (best_it == curve_pts.begin()) {
    ++best_it;
  }
  int last_direction = 0;
  while ((best_it != curve_pts.end()) && (best_it != curve_pts.begin())) {
    auto before_best_it = std::prev(best_it);
    double distance, normalized_arc_param;
    eigenmath::DistanceFromArc<double, Eigen::AutoAlign>(
        before_best_it->point.GetPose().translation(),
        before_best_it->point.GetPose().xAxis(),
        before_best_it->point.GetCurvature(),
        best_it->cord_length - before_best_it->cord_length, pt, &distance,
        &normalized_arc_param);
    cord_length_at_match =
        normalized_arc_param *
            (best_it->cord_length - before_best_it->cord_length) +
        before_best_it->cord_length;

    if (std::abs(cord_length_at_match - best_it->cord_length) < kEpsilon) {
      // Must move forward by one section
      if (last_direction < 0) {
        break;
      }
      ++best_it;
      last_direction = 1;
    } else if (std::abs(cord_length_at_match - before_best_it->cord_length) <
               kEpsilon) {
      // Must move backward by one section
      if (last_direction > 0) {
        break;
      }
      --best_it;
      last_direction = -1;
    } else {
      break;
    }
  }
  if (min_distance != nullptr) {
    *min_distance = min_dist;
  }
  return {cord_length_at_match, Evaluate(cord_length_at_match)};
}

}  // namespace mobility::diff_drive

