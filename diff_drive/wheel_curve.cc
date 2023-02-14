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

#include "diff_drive/wheel_curve.h"

#include <algorithm>
#include <iterator>

#include "absl/log/check.h"
#include "genit/adjacent_iterator.h"

namespace mobility::diff_drive {

namespace {
constexpr double kEpsilon = 1.0e-3;
constexpr double kTenNanoRadians = 1.0e-8;  // 10 nanoradians
}  // namespace

WheelCurve::WheelCurve(int capacity) : points_(capacity), points_count_(0) {}

bool WheelCurve::operator==(const WheelCurve& rhs) const {
  if (GetSize() != rhs.GetSize()) {
    return false;
  }
  auto it_pair = std::mismatch(
      points_.begin(), points_.begin() + points_count_, rhs.points_.begin(),
      [](const WheelStateAndTotalMotion& lhs,
         const WheelStateAndTotalMotion& rhs) -> bool {
        if (std::abs(lhs.total_motion - rhs.total_motion) > kTenNanoRadians) {
          return false;
        }
        const WheelVector wheel_diff =
            rhs.state.GetPositions() - lhs.state.GetPositions();
        if (wheel_diff.lpNorm<Eigen::Infinity>() > kEpsilon) {
          return false;
        }
        return true;
      });
  return (it_pair.first == points_.begin() + points_count_);
}

bool WheelCurve::HasContinuousPosition() const {
  for (const auto segment :
       genit::AdjacentElementsRange<2>(GetWheelStateIteratorRange())) {
    const WheelState from_previous_pt = segment[0].state.Extrapolate(
        segment[1].total_motion - segment[0].total_motion);
    const WheelVector wheel_diff =
        from_previous_pt.GetPositions() - segment[1].state.GetPositions();
    if (wheel_diff.lpNorm<Eigen::Infinity>() > kEpsilon) {
      return false;
    }
  }
  return true;
}

bool WheelCurve::HasMonotonicTotalMotion() const {
  for (const auto segment :
       genit::AdjacentElementsRange<2>(GetWheelStateIteratorRange())) {
    if (segment[1].total_motion < segment[0].total_motion + kTenNanoRadians) {
      return false;
    }
  }
  return true;
}

WheelState WheelCurve::Evaluate(double w) const {
  // Find the pair of points bracketing w in [w0, w1]
  auto it_w1 =
      std::upper_bound(points_.begin(), points_.begin() + points_count_, w,
                       WheelStateAndTotalMotion::TotalMotionComparator());
  if (it_w1 == points_.begin()) {
    return it_w1->state.Extrapolate(w - it_w1->total_motion);
  }
  auto it_w0 = std::prev(it_w1);
  // Evaluate a constant arc extrapolation from w0 to w
  auto dw = w - it_w0->total_motion;
  return it_w0->state.Extrapolate(dw);
}

double WheelCurve::GetMaxCurvature(double wheel_base) const {
  double result = 0.0;
  for (auto& pt : GetWheelStateIteratorRange()) {
    const double pt_kappa = std::abs(pt.state.GetCurvature(wheel_base));
    if (result < pt_kappa) {
      result = pt_kappa;
    }
  }
  return result;
}

bool WheelCurve::AddPoint(double w, const WheelState& new_point) {
  if (points_count_ == points_.size() ||
      ((points_count_ > 0) && (points_[points_count_ - 1].total_motion >= w))) {
    return false;
  }
  points_[points_count_++] = WheelStateAndTotalMotion{w, new_point};
  return true;
}

bool WheelCurve::AppendCurve(const WheelCurve& tail) {
  for (auto& pt : tail.GetWheelStateIteratorRange()) {
    if (points_count_ > 0 &&
        (pt.total_motion <
         (points_[points_count_ - 1].total_motion - kTenNanoRadians))) {
      continue;
    }
    if (points_count_ == points_.size()) {
      return false;
    }
    if (points_count_ > 0 &&
        pt.total_motion <
            points_[points_count_ - 1].total_motion + kTenNanoRadians) {
      points_[points_count_ - 1] = pt;
    } else {
      points_[points_count_++] = pt;
    }
  }
  return true;
}

bool WheelCurve::PrependCurve(const WheelCurve& head) {
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
  const WheelVector wheel_diff =
      GetStart().state.GetPositions() - head.GetFinish().state.GetPositions();
  if (wheel_diff.lpNorm<Eigen::Infinity>() > kEpsilon) {
    return false;
  }
  if (head.GetSize() == 1) {
    return true;
  }
  ApplyTotalMotionShift(head.GetTotalMotionSpan().max() -
                        points_[0].total_motion);
  std::copy_backward(points_.begin(), points_.begin() + points_count_,
                     points_.begin() + points_count_ + head.GetSize() - 1);
  std::copy(head.BeginWheelState(), std::prev(head.EndWheelState()),
            points_.begin());
  points_count_ += head.GetSize() - 1;
  return true;
}

void WheelCurve::TruncateTo(double final_total_motion) {
  TruncateTo(Interval<double>(points_[0].total_motion, final_total_motion));
}

void WheelCurve::TruncateTo(const Interval<double>& total_motion_interval) {
  // Find the pair of points bracketing w_max in [w0, w1]
  auto it_w1 =
      std::upper_bound(points_.begin(), points_.begin() + points_count_,
                       total_motion_interval.max() - kTenNanoRadians,
                       WheelStateAndTotalMotion::TotalMotionComparator());
  if (it_w1 == points_.begin()) {
    Clear();
    return;
  }
  auto it_w0 = std::prev(it_w1);
  // Evaluate a constant arc extrapolation from w0 to w
  const double last_dw = total_motion_interval.max() - it_w0->total_motion;
  WheelState last_pt = it_w0->state.Extrapolate(last_dw);
  // Remove tail, and add new finish point.
  points_count_ = it_w1 - points_.begin();
  AddPoint(total_motion_interval.max(), last_pt);

  // Find the pair of points bracketing w_min in [w0, w1]
  it_w1 = std::upper_bound(points_.begin(), points_.begin() + points_count_,
                           total_motion_interval.min() + kTenNanoRadians,
                           WheelStateAndTotalMotion::TotalMotionComparator());
  if (it_w1 == points_.begin() + points_count_) {
    Clear();
    return;
  }
  it_w0 = it_w1;
  if (it_w1 != points_.begin()) {
    it_w0 = std::prev(it_w1);
  }
  // Evaluate a constant arc extrapolation from w0 to w
  const double first_dw = total_motion_interval.min() - it_w0->total_motion;
  WheelState first_pt = it_w0->state.Extrapolate(first_dw);
  // Remove the head of the curve, by copying points [it_w1, end) to start.
  if (it_w1 != points_.begin()) {
    std::copy(it_w1, points_.begin() + points_count_, points_.begin() + 1);
    points_count_ = ((points_.begin() + points_count_) - it_w1) + 1;
  }
  // Update the start point:
  points_.begin()->total_motion = total_motion_interval.min();
  points_.begin()->state = first_pt;
}

void WheelCurve::ApplyWheelPositionsShift(const WheelVector& delta_positions) {
  for (auto it = points_.begin(), it_end = points_.begin() + points_count_;
       it != it_end; ++it) {
    it->state.SetPositions(it->state.GetPositions() + delta_positions);
  }
}

void WheelCurve::ApplyTotalMotionShift(double dw) {
  for (auto it = points_.begin(), it_end = points_.begin() + points_count_;
       it != it_end; ++it) {
    it->total_motion += dw;
  }
}

WheelCurve::TotalMotionIterator& WheelCurve::TotalMotionIterator::operator+=(
    double dw) {
  // Do not allow backward cord-length-iteration:
  CHECK_GE(dw, 0.0) << "Cannot iterate backward along curve";
  current_motion_ += dw;

  while (current_it_->total_motion < max_motion_ &&
         current_motion_ > std::next(current_it_)->total_motion) {
    ++current_it_;
  }
  return *this;
}

}  // namespace mobility::diff_drive
