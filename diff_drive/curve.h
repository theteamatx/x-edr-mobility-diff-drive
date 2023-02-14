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

#ifndef MOBILITY_DIFF_DRIVE_DIFF_DRIVE_CURVE_H_
#define MOBILITY_DIFF_DRIVE_DIFF_DRIVE_CURVE_H_

#include <iterator>
#include <vector>

#include "diff_drive/continuous_curve.h"
#include "diff_drive/curve_point.h"
#include "diff_drive/interval.h"
#include "genit/iterator_range.h"

namespace mobility::diff_drive {

// Represents a differential-drive curve as a set of piece-wise constant
// curvature sections (constant arc motions).
// Note: The creator of such a curve is responsible for ensuring that
// each sequential pose is consistent with a constant arc motion from the
// previous curve point.
class Curve : public ContinuousCurve {
 public:
  using MutableCurvePointIterator = std::vector<CurvePtAndCord>::iterator;
  using CurvePointIterator = std::vector<CurvePtAndCord>::const_iterator;

  // Holds the maximum number of curve points that can be stored.
  static constexpr int kDefaultCapacity = 64;

  // Default constructor, creates an empty curve.
  Curve();

  // Creates an empty curve of the given capacity.
  explicit Curve(int capacity);

  // Range constructor, creates a curve with given curve points.
  template <typename FwdIter>
  Curve(FwdIter first, FwdIter last) : Curve() {
    for (; first != last; ++first) {
      AddPoint(first->cord_length, first->point);
    }
  }

  // Copy constructor
  Curve(const Curve &rhs) : Curve(rhs.points_.size()) { AppendCurve(rhs); }

  // Copy-assignment operator
  Curve &operator=(const Curve &rhs) {
    Clear();
    AppendCurve(rhs);
    return *this;
  }

  // Move constructor
  Curve(Curve &&) = default;

  // Move-assignment operator
  Curve &operator=(Curve &&) = default;

  // Allocates a given amount of capacity.
  void SetMinCapacity(int capacity) {
    if (points_.size() > capacity) {
      return;
    }
    points_.resize(capacity);
  }

  // Equal operator
  bool operator==(const Curve &rhs) const;

  // Get the current capacity of this curve.
  int GetCapacity() const { return points_.size(); }

  // Get the number of points of this curve.
  int GetSize() const { return points_count_; }

  // Clears the curve.
  void Clear() { points_count_ = 0; }

  bool IsEmpty() const override { return (points_count_ == 0); }

  bool IsSane() const override { return (points_count_ >= 2); }

  bool HasContinuousPosition() const override;

  // Checks that the curve has monotonically increasing cord lengths.
  bool HasMonotonicCordLengths() const;

  CurvePoint Evaluate(double s) const override;

  Interval<double> GetCordLengthSpan() const override {
    if (points_count_ < 2) {
      return Interval<double>();
    }
    return Interval<double>(points_[0].cord_length,
                            points_[points_count_ - 1].cord_length);
  }

  // Computes the ratio between the total cord-length of the curve and
  // the straight-line distance from start to end.
  // Curve must have IsSane() == true
  double ComputeRelativeCordLength() const;

  double GetMaxCurvature() const;

  CurvePtAndCord GetStart() const override {
    if (points_count_ < 1) {
      return CurvePtAndCord{};
    }
    return points_[0];
  }

  CurvePtAndCord GetFinish() const override {
    if (points_count_ < 1) {
      return CurvePtAndCord{};
    }
    return points_[points_count_ - 1];
  }

  // Get an iterator to the first valid curve-point stored in this curve.
  CurvePointIterator BeginCurvePoint() const { return points_.begin(); }
  // Get an iterator to the one-past-last valid curve-point stored in this
  // curve.
  CurvePointIterator EndCurvePoint() const {
    return points_.begin() + points_count_;
  }

  // Get an iterator to the first valid state stored in this trajectory.
  genit::IteratorRange<CurvePointIterator> GetCurvePointIteratorRange() const {
    return genit::IteratorRange(BeginCurvePoint(), EndCurvePoint());
  }

  // Get an iterator to the first valid curve-point stored in this curve.
  MutableCurvePointIterator MutableBeginCurvePoint() { return points_.begin(); }
  // Get an iterator to the one-past-last valid curve-point stored in this
  // curve.
  MutableCurvePointIterator MutableEndCurvePoint() {
    return points_.begin() + points_count_;
  }

  // Get an iterator to the first valid state stored in this trajectory.
  genit::IteratorRange<MutableCurvePointIterator> GetCurvePointIteratorRange() {
    return genit::IteratorRange(MutableBeginCurvePoint(),
                                MutableEndCurvePoint());
  }

  // Add a point to the curve at a given cord-length.
  // Returns false if capacity is exhausted.
  bool AddPoint(double s, const CurvePoint &new_point);

  // Append a given curve (tail) to the end of this curve.
  // Returns false if capacity is exhausted, will append as much of the tail
  // curve as possible before exhausting capacity.
  bool AppendCurve(const Curve &tail);

  // Prepend a given curve (head) to the start of this curve.
  // Returns false if capacity is exhausted.
  bool PrependCurve(const Curve &head);

  // Fill curve with a given set of polyline points.
  // The final orientation will be set according to the orientation of the
  // last segment of the polyline.
  // Returns false if capacity is exhausted, will fill in as much of the
  // polyline as possible before exhausting capacity.
  template <typename PointIter>
  bool FillWithPolylinePoints(
      const eigenmath::Pose2d &curve_reference_pose_input, PointIter first,
      PointIter last);

  template <typename PointRange>
  bool FillWithPolylinePoints(
      const eigenmath::Pose2d &curve_reference_pose_input,
      PointRange pt_range) {
    using std::begin, std::end;
    return FillWithPolylinePoints(curve_reference_pose_input, begin(pt_range),
                                  end(pt_range));
  }

  // Truncate the curve at a given final cord length.
  void TruncateTo(double final_cord_length);

  // Truncate the curve to a given final cord length interval.
  void TruncateTo(const Interval<double> &cord_length_interval);

  // Truncate the curve at the point where a predicate fails, when checked at
  // cord_length_step intervals.
  // The predicate takes a CurvePoint and returns a bool.
  template <typename Predicate>
  void TruncateToPredicateFailure(double cord_length_step, Predicate pred);

  // Cut corners of the curve (polyline) if the new shortcuts pass the given
  // predicate, when checked at cord_length_step intervals.
  // The predicate takes a CurvePoint and returns a bool.
  template <typename Predicate>
  void CutCornersUnlessPredicateFails(double cord_length_step, Predicate pred);

  // Cut corners of the curve (polyline) as a simple smoothing pass.
  void CutCorners();

  void ApplyCordLengthShift(double ds) override;

  void ApplyTransform(const eigenmath::Pose2d &new_pose_old) override;

  CurvePointIterator FindClosestKnotPointMatch(
      const eigenmath::Vector2d &pt, double *min_distance = nullptr) const;

  CurvePtAndCord FindClosestMatchToPoint(const eigenmath::Vector2d &pt,
                                         double *min_distance) const override;

  // This class can be used to iterate over the length of the curve.
  class CordLengthIterator {
   public:
    // Create a length-iterator pointing exactly at a given point iterator.
    CordLengthIterator(CurvePointIterator it, double max_cord)
        : current_it_(it),
          current_cord_(it->cord_length),
          max_cord_(max_cord) {}

    // Create a length-iterator pointing exactly at a given point iterator.
    CordLengthIterator(CurvePointIterator it, double current_cord,
                       double max_cord)
        : current_it_(it), current_cord_(current_cord), max_cord_(max_cord) {}

    // Get the point to which this length-iterator points.
    CurvePoint GetPoint() const {
      auto ds = current_cord_ - current_it_->cord_length;
      return current_it_->point.ExtrapolateConstantArc(ds);
    }

    // Get the point to which this length-iterator points.
    double GetCordLength() const { return current_cord_; }

    // Advance the length-iterator by a given delta in cord-length.
    CordLengthIterator &operator+=(double ds);

    // Advance the length-iterator by a given delta in cord-length.
    friend CordLengthIterator operator+(const CordLengthIterator &it,
                                        double ds) {
      CordLengthIterator result = it;
      result += ds;
      return result;  // NRVO
    }

    // Advance the length-iterator by a given delta in cord-length.
    friend CordLengthIterator operator+(double ds,
                                        const CordLengthIterator &it) {
      CordLengthIterator result = it;
      result += ds;
      return result;  // NRVO
    }

    // Compare two length-iterators to see if LHS comes before RHS.
    // Note, it is important to use less-than to terminate iterations.
    bool operator<(const CordLengthIterator &rhs) const {
      return current_cord_ < rhs.current_cord_;
    }

    // Returns the underlying curve point iterator for the curve point just
    // before the current cord-length value of this iterator.
    CurvePointIterator BaseIterator() const { return current_it_; }

   private:
    CurvePointIterator current_it_;
    double current_cord_;
    double max_cord_;
  };

  // Get a length-iterator to the start of the trajectory.
  CordLengthIterator BeginInCordLength() const {
    if (points_count_ < 1) {
      return CordLengthIterator(points_.begin(), 0.0, 0.0);
    }
    return CordLengthIterator(points_.begin(),
                              points_[points_count_ - 1].cord_length);
  }

  // Get a length-iterator to the end of the trajectory.
  CordLengthIterator EndInCordLength() const {
    if (points_count_ < 1) {
      return CordLengthIterator(points_.begin(), 0.0, 0.0);
    }
    return CordLengthIterator(points_.begin() + points_count_ - 1,
                              points_[points_count_ - 1].cord_length);
  }

 private:
  std::vector<CurvePtAndCord> points_;
  int points_count_;
};

template <typename PointIter>
bool Curve::FillWithPolylinePoints(
    const eigenmath::Pose2d &curve_reference_pose_input, PointIter first,
    PointIter last) {
  // Create the input curve, as a polyline of the input points:
  Clear();
  auto next = std::next(first);
  // Dereference only once in case this is a special iterator that does
  // interpolation within the dereference operator.
  const auto &first_input = *first;
  eigenmath::Vector2d next_point =
      curve_reference_pose_input *
      eigenmath::Vector2d{first_input.x(), first_input.y()};
  double accum_s = 0.0;
  for (; first != last; ++first, ++next) {
    const eigenmath::Vector2d current_point = next_point;
    if (next == last) {
      eigenmath::Pose2d final_pose{current_point,
                                   GetFinish().point.GetPose().angle()};
      if (!AddPoint(accum_s, CurvePoint(final_pose))) {
        return false;
      }
    } else {
      // Dereference only once in case this is a special iterator that does
      // interpolation within the dereference operator.
      const auto &next_input = *next;
      next_point = curve_reference_pose_input *
                   eigenmath::Vector2d{next_input.x(), next_input.y()};
      const eigenmath::Vector2d delta = next_point - current_point;
      const double delta_norm = delta.norm();
      if (delta_norm < kIdenticalPointsEpsilon) {
        // Skip points that are identical.
        continue;
      }
      const eigenmath::Pose2d current_pose{
          current_point, eigenmath::SO2d{delta.x(), delta.y()}};
      if (!AddPoint(accum_s, CurvePoint{current_pose})) {
        return false;
      }
      accum_s += delta_norm;
    }
  }
  return true;
}

template <typename Predicate>
void Curve::TruncateToPredicateFailure(double cord_length_step,
                                       Predicate pred) {
  auto last_it = BeginInCordLength();
  for (auto curve_it = BeginInCordLength(), curve_it_end = EndInCordLength();
       curve_it < curve_it_end; curve_it += cord_length_step) {
    auto curve_pt = curve_it.GetPoint();
    if (!pred(curve_pt)) {
      points_count_ = (last_it.BaseIterator() - points_.begin()) + 1;
      AddPoint(last_it.GetCordLength(), last_it.GetPoint());
      return;
    }
    last_it = curve_it;
  }
}

template <typename Predicate>
void Curve::CutCornersUnlessPredicateFails(double cord_length_step,
                                           Predicate pred) {
  // Nothing to do if the curve is too small:
  if (GetSize() < 4) {
    return;
  }
  auto it_prev = points_.begin();
  auto it_mid = std::next(it_prev);
  auto it_next = std::next(it_mid);
  for (; std::next(it_next) < points_.begin() + points_count_;
       ++it_prev, ++it_mid, ++it_next) {
    const auto &prev_pt = it_prev->point.GetPose().translation();
    const auto &next_pt = it_next->point.GetPose().translation();
    // Skip segments that are already aligned:
    if (std::abs((it_prev->point.GetPose().so2().inverse() *
                  it_mid->point.GetPose().so2())
                     .angle()) > 0.01) {
      const eigenmath::Vector2d prev_to_next_delta = next_pt - prev_pt;
      const double ds = prev_to_next_delta.norm();
      const eigenmath::Vector2d prev_to_next_dir = prev_to_next_delta / ds;
      const eigenmath::SO2d prev_to_next_angle(prev_to_next_dir.x(),
                                               prev_to_next_dir.y(), false);
      double rel_s = cord_length_step;
      const mobility::diff_drive::CurvePoint start_pt(
          eigenmath::Pose2d(prev_pt, prev_to_next_angle));
      bool predicate_failed = false;
      while (rel_s < ds) {
        const auto curve_pt = start_pt.ExtrapolateConstantArc(rel_s);
        if (!pred(curve_pt)) {
          predicate_failed = true;
          break;
        }
        rel_s += cord_length_step;
      }
      if (!predicate_failed) {
        // We can short-cut this corner:
        it_prev->point = start_pt;
        rel_s = ds / 2;
        it_mid->point = start_pt.ExtrapolateConstantArc(rel_s);
        it_mid->cord_length = it_prev->cord_length + rel_s;
      }
    }
    // We need to update the cord-length at it_next:
    it_next->cord_length =
        it_mid->cord_length +
        (next_pt - it_mid->point.GetPose().translation()).norm();
  }
  // it_next should point to the last point now, we must update its cord-length:
  it_next->cord_length =
      it_mid->cord_length + (it_next->point.GetPose().translation() -
                             it_mid->point.GetPose().translation())
                                .norm();
}

}  // namespace mobility::diff_drive

#endif  // MOBILITY_DIFF_DRIVE_DIFF_DRIVE_CURVE_H_
