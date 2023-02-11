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

#ifndef MOBILITY_DIFF_DRIVE_DIFF_DRIVE_TRAJECTORY_H_
#define MOBILITY_DIFF_DRIVE_DIFF_DRIVE_TRAJECTORY_H_

#include <vector>

#include "diff_drive/interval.h"
#include "genit/iterator_range.h"
#include "diff_drive/state.h"
#include "diff_drive/type_aliases.h"

namespace mobility::diff_drive {

// Represents a differential-drive trajectory as a set of piece-wise constant
// velocity sections (constant arc motions).
// Note: The creator of such a trajectory is responsible for ensuring that
// each sequential pose is consistent with a constant arc motion from the
// previous state.
class Trajectory {
 public:
  using StateIterator = std::vector<StateAndTime>::const_iterator;

  // Holds the maximum number of state points that can be stored.
  static constexpr int kDefaultCapacity = 64;

  // Holds the smallest time step that can separate two states.
  static const double kSmallestTimeStepSeconds;

  // Default constructor, creates an empty trajectory.
  Trajectory();

  // Creates an empty trajectory of the given capacity.
  explicit Trajectory(int capacity);

  // Range constructor, creates a trajectory with given state points.
  template <typename FwdIter>
  Trajectory(FwdIter first, FwdIter last) : Trajectory() {
    for (; first != last; ++first) {
      AddState(first->time, first->state);
    }
  }

  // Copy constructor
  Trajectory(const Trajectory &rhs) : Trajectory(rhs.states_.size()) {
    AppendTrajectory(rhs);
  }

  // Copy-assignment operator
  Trajectory &operator=(const Trajectory &rhs) {
    Clear();
    AppendTrajectory(rhs);
    return *this;
  }

  // Move constructor
  Trajectory(Trajectory &&) = default;

  // Move-assignment operator
  Trajectory &operator=(Trajectory &&) = default;

  // Allocates a given amount of capacity.
  void SetMinCapacity(int capacity) {
    if (states_.size() > capacity) {
      return;
    }
    states_.resize(capacity);
  }

  // Equal operator
  bool operator==(const Trajectory &rhs) const;
  bool operator!=(const Trajectory &rhs) const { return !(*this == rhs); }

  // Get the current capacity of this trajectory.
  int GetCapacity() const { return states_.size(); }

  // Get the number of points of this curve.
  int GetSize() const { return states_count_; }

  // Clears the trajectory.
  void Clear() { states_count_ = 0; }

  // Checks if the trajectory is empty.
  bool IsEmpty() const { return (states_count_ == 0); }

  // Checks if the trajectory has enough points to do anything with it.
  bool IsSane() const { return (states_count_ >= 2); }

  // Checks that the trajectory has no discontinuities in position.
  bool HasContinuousPosition() const;

  // Checks that the trajectory has no discontinuities in position within a
  // given tolerance.
  bool HasContinuousPosition(double tolerance) const;

  // Checks that two states have approximately the same position.
  static bool StatesHaveSamePosition(const State &s1, const State &s2);

  // Checks that two states have approximately the same position within a
  // given tolerance.
  static bool StatesHaveSamePosition(const State &s1, const State &s2,
                                     double tolerance);

  // Checks that the trajectory has monotonically increasing time values.
  bool HasMonotonicTimeValues() const;

  // Evaluates the state at a given time.
  // Trajectory must have IsSane() == true
  State Evaluate(double t) const;

  // Computes the total cord-length of the trajectory.
  // Trajectory must have IsSane() == true
  double ComputeTotalCordLength() const;

  // Computes the ratio between the total cord-length of the trajectory and
  // the straight-line distance from start to end.
  // Trajectory must have IsSane() == true
  double ComputeRelativeCordLength() const;

  // Get the time interval over which the trajectory exists.
  Interval<double> GetTimeSpan() const {
    if (states_count_ < 1) {
      return Interval<double>();
    }
    return Interval<double>(states_[0].time, states_[states_count_ - 1].time);
  }

  // Gets the first valid state-time point in this trajectory.
  StateAndTime GetStart() const {
    if (states_count_ < 1) {
      return StateAndTime{};
    }
    return states_[0];
  }
  // Gets the last valid state-time point in this trajectory.
  StateAndTime GetFinish() const {
    if (states_count_ < 1) {
      return StateAndTime{};
    }
    return states_[states_count_ - 1];
  }

  // Get an iterator to the first valid state stored in this trajectory.
  StateIterator BeginState() const { return states_.begin(); }
  // Get an iterator to the one-past-last valid state stored in this trajectory.
  StateIterator EndState() const { return states_.begin() + states_count_; }

  // Get an iterator to the first valid state stored in this trajectory.
  genit::IteratorRange<StateIterator> GetStateIteratorRange() const {
    return genit::IteratorRange(BeginState(), EndState());
  }

  // Append the given range of state-and-time iterators.
  // This function assumes that the state range comes from another trajectory
  // and is therefore continuous and sane.
  // Checks that the first state is at the same time or later than the current
  // finish state of the trajectory (if any). Returns false if not the case.
  // Returns false if capacity is exhausted, filling as many states as capacity
  // will allow.
  // Does not check continuity of the poses.
  bool AppendStates(StateIterator first, StateIterator last);

  // Add a state to the trajectory at a given time.
  // Returns false if capacity is exhausted.
  bool AddState(double t, const State &new_state);
  bool AddState(const StateAndTime &new_state) {
    return AddState(new_state.time, new_state.state);
  }

  // Add an intermediate state to the trajectory at a given time.
  // Returns EndState() if capacity is exhausted.
  // If successful, returns an iterator to the added state.
  StateIterator AddIntermediateState(double t);

  // Add a segment with a static pose to the trajectory over the given time
  // interval.
  // Returns EndState() if capacity is exhausted.
  // If successful, returns an iterator to the added state at the start of the
  // stationary segment.
  StateIterator AddStationarySegment(double time, double duration);

  // Remove a stationary segment or part of a stationary segment from the
  // trajectory. The start iterator marks the start of the stationary segment.
  // The duration, if positive, is the amount of time to remove from the
  // segment. If duration is negative or greater than the duration of the
  // segment, the whole segment will be removed.
  void RemoveStationarySegment(StateIterator start, double duration = -1.0);

  // Warps a segment of the trajectory to change the speed in that segment
  // by a given factor and adjusting the time duration of the segment so that
  // the pose reached at the end is the same. This also shifts the time values
  // in the remainder of the trajectory to adjust to this warped segment.
  void TimeWarpSegment(double speed_factor, StateIterator start);

  // Warps the trajectory to change the speed by a given factor and adjusting
  // the time durations so that the poses reached are the same.
  void TimeWarp(double speed_factor);

  // Appends a given trajectory (tail) to the end of this trajectory.
  // Returns false if capacity is exhausted, will append as much of the tail
  // trajectory as possible before exhausting capacity.
  bool AppendTrajectory(const Trajectory &tail);

  // Prepends a given trajectory (head) to the start of this trajectory.
  // Returns false if capacity is exhausted.
  bool PrependTrajectory(const Trajectory &head);

  // Truncate or extend the trajectory at a given time interval.
  // When a bound of the time interval falls between two state points of the
  // trajectory, the end point will be an interpolation between those states.
  // If either or both bounds of the desired time interval falls outside of
  // the current time interval of the trajectory, extrapolation will be used to
  // ensure that the trajectory (after this call) spans the desired interval.
  void TruncateOrExtendTo(const Interval<double> &time_interval);

  // Truncate the trajectory at a given time interval.
  // When a bound of the time interval falls between two state points of the
  // trajectory, the end point will be an interpolation between those states.
  // If either or both bounds of the desired time interval falls outside of
  // the current time interval of the trajectory, the resulting trajectory will
  // span the intersection of the two intervals.
  void TruncateTo(const Interval<double> &time_interval) {
    Interval<double> intersection = GetTimeSpan();
    intersection.IntersectWith(time_interval);
    TruncateOrExtendTo(intersection);
  }

  // Apply a time shift to all the points of the trajectory.
  void ApplyTimeShift(double dt);

  // Apply a transform to all the points of the trajectory.
  void ApplyTransform(const eigenmath::Pose2d &new_pose_old);

  // Reverse the entire trajectory.
  void Reverse();

  // This class can be used to iterate over the time-span of the trajectory.
  class TimeIterator {
   public:
    // Create a time-iterator pointing exactly at a given state iterator.
    TimeIterator(StateIterator it, double max_time)
        : current_it_(it), current_time_(it->time), max_time_(max_time) {}

    // Create a time-iterator pointing exactly at a given state iterator.
    TimeIterator(StateIterator it, double current_time, double max_time)
        : current_it_(it), current_time_(current_time), max_time_(max_time) {}

    // Get the state to which this time-iterator points.
    State GetState() const {
      const auto dt = current_time_ - current_it_->time;
      return current_it_->state.ExtrapolateConstantVelocityArc(dt);
    }

    // Get the time value of this time-iterator.
    double GetTime() const { return current_time_; }

    // Advance the time-iterator by a given delta.
    TimeIterator &operator+=(double dt);

    // Advance the time-iterator by a given delta.
    friend TimeIterator operator+(const TimeIterator &it, double ds) {
      TimeIterator result = it;
      result += ds;
      return result;  // NRVO
    }

    // Advance the time-iterator by a given delta.
    friend TimeIterator operator+(double ds, const TimeIterator &it) {
      TimeIterator result = it;
      result += ds;
      return result;  // NRVO
    }

    // Compare two time-iterators to see if LHS is comes before RHS.
    // Note, it is important to use less-than to terminate iterations.
    bool operator<(const TimeIterator &rhs) const {
      return current_time_ < rhs.current_time_;
    }

   private:
    StateIterator current_it_;
    double current_time_;
    double max_time_;
  };

  // Get a time-iterator to the start of the trajectory.
  TimeIterator BeginInTime() const {
    if (states_count_ < 1) {
      return TimeIterator(states_.begin(), 0.0, 0.0);
    }
    return TimeIterator(states_.begin(), states_[states_count_ - 1].time);
  }

  // Get a time-iterator to the end of the trajectory.
  TimeIterator EndInTime() const {
    if (states_count_ < 1) {
      return TimeIterator(states_.begin(), 0.0, 0.0);
    }
    return TimeIterator(states_.begin() + states_count_ - 1,
                        states_[states_count_ - 1].time);
  }

  // This class can be used to iterate over the length of the trajectory.
  class CordLengthIterator {
   public:
    // Create a length-iterator pointing exactly at a given state iterator.
    CordLengthIterator(StateIterator it, double max_time)
        : current_it_(it), current_time_(it->time), max_time_(max_time) {}

    // Create a length-iterator pointing exactly at a given state iterator.
    CordLengthIterator(StateIterator it, double current_time, double max_time)
        : current_it_(it), current_time_(current_time), max_time_(max_time) {}

    // Get the state to which this length-iterator points.
    State GetState() const {
      const auto dt = current_time_ - current_it_->time;
      return current_it_->state.ExtrapolateConstantVelocityArc(dt);
    }

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
      return current_time_ < rhs.current_time_;
    }

   private:
    StateIterator current_it_;
    double current_time_;
    double max_time_;
  };

  // Get a length-iterator to the start of the trajectory.
  CordLengthIterator BeginInCordLength() const {
    if (states_count_ < 1) {
      return CordLengthIterator(states_.begin(), 0.0, 0.0);
    }
    return CordLengthIterator(states_.begin(), states_[states_count_ - 1].time);
  }

  // Get a length-iterator to the end of the trajectory.
  CordLengthIterator EndInCordLength() const {
    if (states_count_ < 1) {
      return CordLengthIterator(states_.begin(), 0.0, 0.0);
    }
    return CordLengthIterator(states_.begin() + states_count_ - 1,
                              states_[states_count_ - 1].time);
  }

  StateAndTime FindClosestMatchToPoint(const eigenmath::Vector2d &pt) const;

  StateAndTime FindClosestMatchInTimeInterval(
      const eigenmath::Vector2d &pt,
      const Interval<double> &time_interval) const;

  static double RmsPositionDeviation(const Trajectory &lhs,
                                     const Trajectory &rhs);

  // ---- Functions to be used only for very special purposes:

  StateAndTime &GetMutableStart() { return states_[0]; }
  StateAndTime &GetMutableFinish() { return states_[states_count_ - 1]; }

 private:
  std::vector<StateAndTime> states_;
  int states_count_;
};

}  // namespace mobility::diff_drive

#endif  // MOBILITY_DIFF_DRIVE_DIFF_DRIVE_TRAJECTORY_H_
