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

#ifndef MOBILITY_DIFF_DRIVE_DIFF_DRIVE_WHEEL_CURVE_H_
#define MOBILITY_DIFF_DRIVE_DIFF_DRIVE_WHEEL_CURVE_H_

#include <vector>

#include "diff_drive/interval.h"
#include "diff_drive/type_aliases.h"
#include "diff_drive/wheel_state.h"
#include "genit/iterator_range.h"

namespace mobility::diff_drive {

// Represents a differential-drive curve as a set of piece-wise constant
// wheel motion sections.
class WheelCurve {
 public:
  using MutableWheelStateIterator =
      std::vector<WheelStateAndTotalMotion>::iterator;
  using WheelStateIterator =
      std::vector<WheelStateAndTotalMotion>::const_iterator;

  // Holds the maximum number of curve points that can be stored.
  static constexpr int kDefaultCapacity = 64;

  // Creates an empty curve of the given capacity.
  explicit WheelCurve(int capacity = kDefaultCapacity);

  // Range constructor, creates a curve with given curve points.
  template <typename FwdIter>
  WheelCurve(FwdIter first, FwdIter last) : WheelCurve() {
    for (; first != last; ++first) {
      AddPoint(first->total_motion, first->point);
    }
  }

  // Copy constructor
  WheelCurve(const WheelCurve& rhs) : WheelCurve(rhs.points_.size()) {
    AppendCurve(rhs);
  }

  // Copy-assignment operator
  WheelCurve& operator=(const WheelCurve& rhs) {
    Clear();
    AppendCurve(rhs);
    return *this;
  }

  // Move constructor / assignment
  WheelCurve(WheelCurve&&) = default;
  WheelCurve& operator=(WheelCurve&&) = default;

  // Allocates a given amount of capacity.
  void SetMinCapacity(int capacity) {
    if (points_.size() > capacity) {
      return;
    }
    points_.resize(capacity);
  }

  // Equal operator
  bool operator==(const WheelCurve& rhs) const;

  // Get the current capacity of this curve.
  int GetCapacity() const { return points_.size(); }

  // Get the number of points of this curve.
  int GetSize() const { return points_count_; }

  // Clears the curve.
  void Clear() { points_count_ = 0; }

  // Checks if the curve is empty.
  bool IsEmpty() const { return (points_count_ == 0); }

  // Checks if the curve has enough points to do anything with it.
  bool IsSane() const { return (points_count_ >= 2); }

  // Checks that the curve has no discontinuities in position.
  bool HasContinuousPosition() const;

  // Checks that the curve has monotonically increasing total motion values.
  bool HasMonotonicTotalMotion() const;

  // Evaluates the wheel state at a given total motion value.
  // Trajectory must have IsSane() == true
  WheelState Evaluate(double w) const;

  // Get the total-motion interval over which the curve exists.
  Interval<double> GetTotalMotionSpan() const {
    if (points_count_ < 2) {
      return Interval<double>();
    }
    return Interval<double>(points_[0].total_motion,
                            points_[points_count_ - 1].total_motion);
  }

  // Compute the maximum curvature encountered along the trajectory, using
  // the given wheel base value.
  double GetMaxCurvature(double wheel_base) const;

  // Gets the first valid wheel-state in this curve.
  WheelStateAndTotalMotion GetStart() const {
    if (points_count_ < 1) {
      return WheelStateAndTotalMotion{};
    }
    return points_[0];
  }

  // Gets the last valid wheel-state in this curve.
  WheelStateAndTotalMotion GetFinish() const {
    if (points_count_ < 1) {
      return WheelStateAndTotalMotion{};
    }
    return points_[points_count_ - 1];
  }

  // Get an iterator to the first valid wheel state stored in this curve.
  WheelStateIterator BeginWheelState() const { return points_.begin(); }
  // Get an iterator to the one-past-last valid wheel state stored in this
  // curve.
  WheelStateIterator EndWheelState() const {
    return points_.begin() + points_count_;
  }

  // Get an iterator to the first valid state stored in this trajectory.
  genit::IteratorRange<WheelStateIterator> GetWheelStateIteratorRange() const {
    return genit::IteratorRange(BeginWheelState(), EndWheelState());
  }

  // Add a point to the curve at a given total motion value.
  // Returns false if capacity is exhausted.
  bool AddPoint(double w, const WheelState& new_point);

  // Append a given curve (tail) to the end of this curve.
  // Returns false if capacity is exhausted, will append as much of the tail
  // curve as possible before exhausting capacity.
  // If parts of the curves overlap, it will append only the new parts of the
  // other 'tail' curve.
  bool AppendCurve(const WheelCurve& tail);

  // Prepend a given curve (head) to the start of this curve.
  // Returns false if capacity is exhausted.
  // If parts of the curves overlap, it will prepend only the new parts of the
  // other 'head' curve.
  bool PrependCurve(const WheelCurve& head);

  // Truncate the curve at a given final total motion.
  void TruncateTo(double final_total_motion);

  // Truncate the curve to a given final total motion interval.
  void TruncateTo(const Interval<double>& total_motion_interval);

  // Apply a wheel positions shift to all the points of the curve.
  void ApplyWheelPositionsShift(const WheelVector& delta_position);

  // Apply a total-motion shift to all the points of the curve.
  void ApplyTotalMotionShift(double dw);

  // This class can be used to iterate over the total motion of the curve.
  class TotalMotionIterator {
   public:
    // Create a motion-iterator pointing exactly at a given point iterator.
    TotalMotionIterator(WheelStateIterator it, double max_motion)
        : current_it_(it),
          current_motion_(it->total_motion),
          max_motion_(max_motion) {}

    // Create a motion-iterator pointing exactly at a given point iterator.
    TotalMotionIterator(WheelStateIterator it, double current_motion,
                        double max_motion)
        : current_it_(it),
          current_motion_(current_motion),
          max_motion_(max_motion) {}

    // Get the wheel state to which this motion-iterator points.
    WheelState GetState() const {
      auto dw = current_motion_ - current_it_->total_motion;
      return current_it_->state.Extrapolate(dw);
    }

    // Get the point to which this motion-iterator points.
    double GetTotalMotion() const { return current_motion_; }

    // Advance the motion-iterator by a given delta in total motion.
    TotalMotionIterator& operator+=(double dw);

    // Advance the motion-iterator by a given delta in total motion.
    friend TotalMotionIterator operator+(const TotalMotionIterator& it,
                                         double dw) {
      TotalMotionIterator result = it;
      result += dw;
      return result;  // NRVO
    }

    // Advance the motion-iterator by a given delta in total motion.
    friend TotalMotionIterator operator+(double dw,
                                         const TotalMotionIterator& it) {
      TotalMotionIterator result = it;
      result += dw;
      return result;  // NRVO
    }

    // Compare two motion-iterators to see if LHS comes before RHS.
    // Note, it is important to use less-than to terminate iterations.
    bool operator<(const TotalMotionIterator& rhs) const {
      return current_motion_ < rhs.current_motion_;
    }

    // Returns the underlying curve iterator for the wheel state just
    // before the current total motion value of this iterator.
    WheelStateIterator BaseIterator() const { return current_it_; }

   private:
    WheelStateIterator current_it_;
    double current_motion_;
    double max_motion_;
  };

  // Get a motion-iterator to the start of the trajectory.
  TotalMotionIterator BeginInTotalMotion() const {
    if (points_count_ < 1) {
      return TotalMotionIterator(points_.begin(), 0.0, 0.0);
    }
    return TotalMotionIterator(points_.begin(),
                               points_[points_count_ - 1].total_motion);
  }

  // Get a motion-iterator to the end of the trajectory.
  TotalMotionIterator EndInTotalMotion() const {
    if (points_count_ < 1) {
      return TotalMotionIterator(points_.begin(), 0.0, 0.0);
    }
    return TotalMotionIterator(points_.begin() + points_count_ - 1,
                               points_[points_count_ - 1].total_motion);
  }

 private:
  std::vector<WheelStateAndTotalMotion> points_;
  int points_count_ = 0;
};

}  // namespace mobility::diff_drive

#endif  // MOBILITY_DIFF_DRIVE_DIFF_DRIVE_WHEEL_CURVE_H_
