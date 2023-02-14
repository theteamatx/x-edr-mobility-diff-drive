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

#include "diff_drive/trajectory.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <tuple>

#include "absl/log/check.h"
#include "absl/strings/str_format.h"
#include "diff_drive/curve_trajectory_utils.h"  // IWYU pragma: keep
#include "eigenmath/line_search.h"
#include "eigenmath/quadrature.h"
#include "genit/adjacent_iterator.h"

namespace mobility::diff_drive {

namespace {
constexpr double kEpsilon = 1.0e-3;
}  // namespace

const double Trajectory::kSmallestTimeStepSeconds = 1.0e-8;  // 10 nanoseconds

Trajectory::Trajectory() : states_(kDefaultCapacity), states_count_(0) {}

Trajectory::Trajectory(int capacity) : states_(capacity), states_count_(0) {}

bool Trajectory::operator==(const Trajectory &rhs) const {
  if (GetSize() != rhs.GetSize()) {
    return false;
  }
  auto it_pair = std::mismatch(
      states_.begin(), states_.begin() + states_count_, rhs.states_.begin(),
      [](const StateAndTime &lhs, const StateAndTime &rhs) -> bool {
        if (std::abs(lhs.time - rhs.time) > kSmallestTimeStepSeconds) {
          return false;
        }
        const eigenmath::Pose2d pose_difference =
            lhs.state.GetPose().inverse() * rhs.state.GetPose();
        if (pose_difference.translation().squaredNorm() > kEpsilon * kEpsilon ||
            std::abs(pose_difference.so2().sin_angle()) > kEpsilon) {
          return false;
        }
        return true;
      });
  return (it_pair.first == states_.begin() + states_count_);
}

bool Trajectory::HasContinuousPosition() const {
  return HasContinuousPosition(kEpsilon);
}

bool Trajectory::HasContinuousPosition(double tolerance) const {
  for (const auto segment :
       genit::AdjacentElementsRange<2>(GetStateIteratorRange())) {
    const State from_previous_pt =
        segment[0].state.ExtrapolateConstantVelocityArc(segment[1].time -
                                                        segment[0].time);
    if (!StatesHaveSamePosition(from_previous_pt, segment[1].state,
                                tolerance)) {
      return false;
    }
  }
  return true;
}

bool Trajectory::StatesHaveSamePosition(const State &s1, const State &s2) {
  return StatesHaveSamePosition(s1, s2, kEpsilon);
}

bool Trajectory::StatesHaveSamePosition(const State &s1, const State &s2,
                                        double tolerance) {
  const eigenmath::Vector2d position_difference =
      s1.GetPose().translation() - s2.GetPose().translation();
  return (position_difference.squaredNorm() < tolerance * tolerance);
}

bool Trajectory::HasMonotonicTimeValues() const {
  for (const auto segment :
       genit::AdjacentElementsRange<2>(GetStateIteratorRange())) {
    if (segment[1].time < segment[0].time + kSmallestTimeStepSeconds) {
      return false;
    }
  }
  return true;
}

State Trajectory::Evaluate(double t) const {
  // Find the pair of points bracketing t in [t0, t1]
  auto it_t1 =
      std::upper_bound(states_.begin(), states_.begin() + states_count_, t,
                       StateAndTime::TimeComparator());
  if (it_t1 == states_.begin()) {
    return it_t1->state.ExtrapolateConstantVelocityArc(t - it_t1->time);
  }
  auto it_t0 = std::prev(it_t1);
  // Evaluate a constant arc extrapolation from t0 to t
  auto dt = t - it_t0->time;
  return it_t0->state.ExtrapolateConstantVelocityArc(dt);
}

double Trajectory::ComputeTotalCordLength() const {
  double cord_length = 0.0;
  for (const auto segment :
       genit::AdjacentElementsRange<2>(GetStateIteratorRange())) {
    auto dt = segment[1].time - segment[0].time;
    cord_length +=
        dt * std::abs(segment[0].state.GetArcVelocity().Translation());
  }
  return cord_length;
}

double Trajectory::ComputeRelativeCordLength() const {
  const double cord_length = std::max(kEpsilon, ComputeTotalCordLength());
  const double line_length =
      std::max(kEpsilon, (GetFinish().state.GetPose().translation() -
                          GetStart().state.GetPose().translation())
                             .norm());
  return cord_length / line_length;
}

bool Trajectory::AppendStates(StateIterator first, StateIterator last) {
  if (first == last) {
    return true;
  }
  if (states_count_ == states_.size() ||
      ((states_count_ > 0) &&
       (states_[states_count_ - 1].time > first->time))) {
    return false;
  }
  if ((states_count_ > 0) &&
      (states_[states_count_ - 1].time + kSmallestTimeStepSeconds >
       first->time)) {
    states_[states_count_ - 1] = *first;
    ++first;
  }
  bool result = true;
  if (last - first > states_.size() - states_count_) {
    result = false;
    last = first + states_.size() - states_count_;
  }
  std::copy(first, last, states_.begin() + states_count_);
  states_count_ += (last - first);
  return result;
}

bool Trajectory::AddState(double t, const State &new_state) {
  if (states_count_ == states_.size() ||
      ((states_count_ > 0) && (states_[states_count_ - 1].time > t))) {
    return false;
  }
  if ((states_count_ > 0) &&
      (states_[states_count_ - 1].time + kSmallestTimeStepSeconds > t)) {
    states_[states_count_ - 1].time = t;
    states_[states_count_ - 1].state = new_state;
  } else {
    states_[states_count_++] = StateAndTime{t, new_state};
  }
  states_[states_count_ - 1].state.NormalizeRotation();
  return true;
}

Trajectory::StateIterator Trajectory::AddIntermediateState(double t) {
  if (states_count_ == states_.size()) {
    return EndState();
  }
  // Find the pair of states bracketing t in [t0, t1]
  auto it_t1 =
      std::upper_bound(states_.begin(), states_.begin() + states_count_, t,
                       StateAndTime::TimeComparator());
  // Evaluate a constant arc extrapolation from t0 to t
  State intermediate_state;
  if (it_t1 == states_.begin()) {
    if (it_t1->time < t + kSmallestTimeStepSeconds) {
      return it_t1;
    }
    intermediate_state =
        it_t1->state.ExtrapolateConstantVelocityArc(t - it_t1->time);
  } else {
    if (it_t1 != states_.end() && it_t1->time < t + kSmallestTimeStepSeconds) {
      return it_t1;
    }
    auto it_t0 = std::prev(it_t1);
    if (it_t0->time + kSmallestTimeStepSeconds > t) {
      return it_t0;
    }
    intermediate_state =
        it_t0->state.ExtrapolateConstantVelocityArc(t - it_t0->time);
  }
  std::copy_backward(it_t1, states_.begin() + states_count_,
                     states_.begin() + states_count_ + 1);
  ++states_count_;
  it_t1->time = t;
  it_t1->state = intermediate_state;
  return it_t1;
}

Trajectory::StateIterator Trajectory::AddStationarySegment(double time,
                                                           double duration) {
  // Need room for 2 states.
  if (states_count_ >= states_.size() - 1) {
    return EndState();
  }
  CHECK_GE(duration, kSmallestTimeStepSeconds) << absl::StrFormat(
      "Cannot add a stationary segment of duration (%f) less "
      "than minimum duration (%f)",
      duration, kSmallestTimeStepSeconds);
  // Find the pair of states bracketing time in [t0, t1]
  auto it_t1 =
      std::upper_bound(states_.begin(), states_.begin() + states_count_, time,
                       StateAndTime::TimeComparator());
  auto it_t0 = std::prev(it_t1);
  // Evaluate the state at time and check how many points need to inserted.
  State intermediate_state;
  int num_inserted_states = 0;
  if (it_t1 == states_.begin()) {
    intermediate_state =
        it_t1->state.ExtrapolateConstantVelocityArc(time - it_t1->time);
    num_inserted_states =
        (it_t1->time < time + kSmallestTimeStepSeconds ? 1 : 2);
  } else {
    intermediate_state =
        it_t0->state.ExtrapolateConstantVelocityArc(time - it_t0->time);
    num_inserted_states =
        (it_t0->time + kSmallestTimeStepSeconds > time ? 1 : 2);
  }
  // Shift the trailing states by the number of inserted states.
  std::copy_backward(it_t1, states_.begin() + states_count_,
                     states_.begin() + states_count_ + num_inserted_states);
  if (num_inserted_states == 2 || it_t1 == states_.begin()) {
    it_t0 = it_t1;
    ++it_t1;
  }
  states_count_ += num_inserted_states;
  // Store the stationary segment and shift trailing states by duration.
  it_t0->time = time;
  it_t0->state = intermediate_state;
  it_t0->state.SetArcVelocity(ArcVector{0.0, 0.0});
  it_t1->time = time;
  it_t1->state = intermediate_state;
  for (; it_t1 != states_.begin() + states_count_; ++it_t1) {
    it_t1->time += duration;
  }
  return it_t0;
}

void Trajectory::RemoveStationarySegment(StateIterator start, double duration) {
  if (start >= EndState() || IsEmpty()) {
    return;  // Nothing to do.
  }
  CHECK_LE(start->state.GetArcVelocity().lpNorm<Eigen::Infinity>(), kEpsilon)
      << absl::StrFormat(
             "State to be removed is not stationary (arc velocity: %f, %f)",
             start->state.GetArcVelocity().Translation(),
             start->state.GetArcVelocity().Rotation());
  const int start_idx = start - BeginState();

  // If start is the last state, simply remove it.
  if (start_idx == states_count_ - 1) {
    --states_count_;
    return;
  }

  // Otherwise, either eliminate the segment altogether or shorten it.
  const double segment_duration =
      states_[start_idx + 1].time - states_[start_idx].time;
  if (duration < 0.0 || duration > segment_duration) {
    std::copy(states_.begin() + start_idx + 1, states_.begin() + states_count_,
              states_.begin() + start_idx);
    --states_count_;
    for (auto it = states_.begin() + start_idx,
              it_end = states_.begin() + states_count_;
         it != it_end; ++it) {
      it->time -= segment_duration;
    }
  } else {
    for (auto it = states_.begin() + start_idx + 1,
              it_end = states_.begin() + states_count_;
         it != it_end; ++it) {
      it->time -= duration;
    }
  }
}

void Trajectory::TimeWarpSegment(double speed_factor, StateIterator start) {
  if (start >= EndState()) {
    return;  // Nothing to do.
  }
  const auto mutable_start = states_.begin() + (start - states_.begin());
  mutable_start->state.SetArcVelocity(speed_factor *
                                      start->state.GetArcVelocity());
  if (mutable_start == states_.begin() + states_count_ - 1) {
    return;  // Only need to adjust velocity if start is the last state.
  }
  auto mutable_end = mutable_start + 1;
  CHECK_GE(std::abs(speed_factor), kEpsilon * kEpsilon)
      << absl::StrFormat("Cannot apply a zero (%f) speed factor", speed_factor);
  const double original_dt = mutable_end->time - mutable_start->time;
  const double time_shift = original_dt / speed_factor - original_dt;
  for (; mutable_end != states_.begin() + states_count_; ++mutable_end) {
    mutable_end->time += time_shift;
  }
}

void Trajectory::TimeWarp(double speed_factor) {
  if (states_count_ == 0) {
    return;  // Nothing to do.
  }
  if (states_count_ == 1) {
    // Only need to adjust velocity if start is the last state.
    states_[0].state.SetArcVelocity(speed_factor *
                                    states_[0].state.GetArcVelocity());
    return;
  }
  CHECK_GE(std::abs(speed_factor), kEpsilon)
      << absl::StrFormat("Cannot apply a zero (%f) speed factor", speed_factor);
  double accumulated_time_shift = 0.0;
  for (const auto mutable_segment : genit::AdjacentElementsRange<2>(
           states_.begin(), states_.begin() + states_count_)) {
    const double original_dt =
        mutable_segment[1].time - mutable_segment[0].time;
    mutable_segment[0].time += accumulated_time_shift;
    mutable_segment[0].state.SetArcVelocity(
        speed_factor * mutable_segment[0].state.GetArcVelocity());
    const double time_shift = original_dt / speed_factor - original_dt;
    accumulated_time_shift += time_shift;
  }
  states_[states_count_ - 1].state.SetArcVelocity(
      speed_factor * states_[states_count_ - 1].state.GetArcVelocity());
  states_[states_count_ - 1].time += accumulated_time_shift;
}

bool Trajectory::AppendTrajectory(const Trajectory &tail) {
  bool first_state_point = true;
  for (auto &state_time : tail.GetStateIteratorRange()) {
    if (first_state_point && states_count_ > 0) {
      if (state_time.time < states_[states_count_ - 1].time) {
        continue;
      }
      if (states_[states_count_ - 1].time + kSmallestTimeStepSeconds >
          state_time.time) {
        states_[states_count_ - 1] = state_time;
        continue;
      }
    }
    if (states_count_ == states_.size()) {
      return false;
    }
    states_[states_count_++] = state_time;
    first_state_point = false;
  }
  return true;
}

bool Trajectory::PrependTrajectory(const Trajectory &head) {
  if (head.IsEmpty()) {
    return true;
  }
  if (head.GetSize() + GetSize() > GetCapacity() + 1) {
    return false;
  }
  if (IsEmpty()) {
    return AppendTrajectory(head);
  }
  // Neither trajectories are empty, check that they match up.
  const eigenmath::Pose2d pose_difference =
      head.GetFinish().state.GetPose().inverse() * GetStart().state.GetPose();
  if (pose_difference.translation().squaredNorm() > kEpsilon * kEpsilon ||
      std::abs(pose_difference.so2().sin_angle()) > kEpsilon) {
    return false;
  }
  if (head.GetSize() == 1) {
    return true;
  }
  ApplyTimeShift(head.GetTimeSpan().max() - states_[0].time);
  std::copy_backward(states_.begin(), states_.begin() + states_count_,
                     states_.begin() + states_count_ + head.GetSize() - 1);
  std::copy(head.BeginState(), std::prev(head.EndState()), states_.begin());
  states_count_ += head.GetSize() - 1;
  return true;
}

void Trajectory::TruncateOrExtendTo(const Interval<double> &time_interval) {
  if (states_count_ < 1) {
    return;
  }

  if (time_interval.Empty()) {
    Clear();
    return;
  }

  Interval<double> intersection = GetTimeSpan();
  intersection.IntersectWith(time_interval);

  if (intersection.Empty()) {
    if (time_interval.min() > states_[states_count_ - 1].time) {
      const StateAndTime old_finish = states_[states_count_ - 1];
      states_[0].time = time_interval.min();
      states_[0].state = old_finish.state.ExtrapolateConstantVelocityArc(
          time_interval.min() - old_finish.time);
      states_[1].time = time_interval.max();
      states_[1].state = old_finish.state.ExtrapolateConstantVelocityArc(
          time_interval.max() - old_finish.time);
      states_count_ = 2;
    } else {
      const StateAndTime old_start = states_[0];
      states_[0].time = time_interval.min();
      states_[0].state = old_start.state.ExtrapolateConstantVelocityArc(
          time_interval.min() - old_start.time);
      states_[1].time = time_interval.max();
      states_[1].state = old_start.state.ExtrapolateConstantVelocityArc(
          time_interval.max() - old_start.time);
      states_count_ = 2;
    }
    return;
  }

  auto it_first = states_.begin();
  if (intersection.min() > it_first->time) {
    it_first =
        std::upper_bound(states_.begin(), states_.begin() + states_count_,
                         intersection.min(), StateAndTime::TimeComparator());
    if (it_first->time > intersection.min() + kSmallestTimeStepSeconds) {
      --it_first;
      it_first->state = it_first->state.ExtrapolateConstantVelocityArc(
          intersection.min() - it_first->time);
    }
    it_first->time = intersection.min();
  }

  auto it_last = states_.begin() + states_count_ - 1;
  if (intersection.max() < it_last->time) {
    it_last =
        std::upper_bound(states_.begin(), states_.begin() + states_count_,
                         intersection.max(), StateAndTime::TimeComparator());
    auto it_prevlast = std::prev(it_last);
    if (it_prevlast->time + kSmallestTimeStepSeconds < intersection.max()) {
      it_last->state = it_prevlast->state.ExtrapolateConstantVelocityArc(
          intersection.max() - it_prevlast->time);
    } else {
      it_last = it_prevlast;
    }
    it_last->time = intersection.max();
  }

  // Update the start of trajectory (with extrapolation)
  if (time_interval.min() >= intersection.min() - kSmallestTimeStepSeconds) {
    states_count_ = it_last - it_first + 1;
    if (it_first != states_.begin()) {
      std::move(it_first, std::next(it_last), states_.begin());
      it_first = states_.begin();
      it_last = it_first + states_count_ - 1;
    }
    it_first->time = time_interval.min();
  } else {
    states_count_ = it_last - it_first + 1;
    if (states_count_ < states_.size()) {
      std::move_backward(it_first, std::next(it_last),
                         std::next(std::next(it_last)));
      it_first = states_.begin() + 1;
      states_[0].time = time_interval.min();
      states_[0].state = it_first->state.ExtrapolateConstantVelocityArc(
          states_[0].time - it_first->time);
      it_first = states_.begin();
      ++it_last;
      ++states_count_;
    }
  }

  // Update the end of trajectory (with extrapolation)
  if (intersection.max() != time_interval.max()) {
    if (it_last->time + kSmallestTimeStepSeconds > time_interval.max()) {
      it_last->time = time_interval.max();
    } else if (states_count_ < states_.size()) {
      states_[states_count_].time = time_interval.max();
      states_[states_count_].state =
          it_last->state.ExtrapolateConstantVelocityArc(
              states_[states_count_].time - it_last->time);
      ++it_last;
      ++states_count_;
    }
  }

  if (!HasMonotonicTimeValues()) {
    CHECK(false) << absl::StrFormat(
        "Time in trajectory is not monotonic! Trajectory =\n%v\n", *this);
  }
}

void Trajectory::ApplyTimeShift(double dt) {
  for (auto &state : states_) {
    state.time += dt;
  }
}

void Trajectory::ApplyTransform(const eigenmath::Pose2d &new_pose_old) {
  for (auto &state : states_) {
    state.state.SetPose(new_pose_old * state.state.GetPose());
  }
}

void Trajectory::Reverse() {
  const Interval<double> old_time_span = GetTimeSpan();
  std::reverse(states_.begin(), states_.begin() + states_count_);
  double current_time = old_time_span.min();
  auto it_cur = states_.begin();
  auto it_next = std::next(it_cur);
  auto it_end = states_.begin() + states_count_;
  while (it_next != it_end) {
    double dt = it_cur->time - it_next->time;
    it_cur->time = current_time;
    it_cur->state.SetArcVelocity(-it_next->state.GetArcVelocity());
    current_time += dt;
    ++it_cur;
    ++it_next;
  }
  it_cur->time = old_time_span.max();
  it_cur->state.SetArcVelocity(-it_cur->state.GetArcVelocity());
}

Trajectory::TimeIterator &Trajectory::TimeIterator::operator+=(double dt) {
  // Do not allow backward time-iteration:
  CHECK_GE(dt, 0.0) << "Cannot iterate backward in time along trajectory";
  current_time_ += dt;

  while (current_it_->time < max_time_ &&
         current_time_ > std::next(current_it_)->time) {
    ++current_it_;
  }
  return *this;
}

Trajectory::CordLengthIterator &Trajectory::CordLengthIterator::operator+=(
    double ds) {
  constexpr double kDblEpsilon = std::numeric_limits<double>::epsilon();
  // Do not allow backward cord-length-iteration:
  CHECK_GE(ds, 0.0) << "Cannot iterate backward along trajectory";
  while (true) {
    const double trans_speed =
        std::abs(current_it_->state.GetArcVelocity().Translation());
    if (ds < kDblEpsilon && trans_speed < kDblEpsilon) {
      // we must have had a 0 / 0, which is fine, no motion:
      return *this;
    }
    if (trans_speed < kDblEpsilon) {
      // we must have zero translation velocity, just move on to next state:
      if (current_time_ < max_time_) {
        ++current_it_;
        current_time_ = current_it_->time;
      }
      if (current_time_ >= max_time_) {
        // Reached the end of a stopping trajectory.
        // Set such that it will not be less than the sentinel iterator:
        current_time_ =
            std::nextafter(current_time_, std::numeric_limits<double>::max());
        return *this;
      }
      continue;
    }
    double dt = ds / trans_speed;
    if (current_it_->time < max_time_ &&
        dt + current_time_ > std::next(current_it_)->time) {
      ds -= (std::next(current_it_)->time - current_time_) *
            std::abs(current_it_->state.GetArcVelocity().Translation());
      ++current_it_;
      current_time_ = current_it_->time;
    } else {
      current_time_ += dt;
      return *this;
    }
  }
  CHECK(false) << "Should never reach this point!";
  return *this;  // <-- keep compiler quiet
}

StateAndTime Trajectory::FindClosestMatchToPoint(
    const eigenmath::Vector2d &pt) const {
  double min_dist = std::numeric_limits<double>::max();
  double time_at_match = 0.0;
  auto traj_pts = GetStateIteratorRange();
  auto best_it = traj_pts.begin();
  for (auto traj_it = traj_pts.begin(); traj_it < traj_pts.end(); ++traj_it) {
    const double dist_value =
        (pt - traj_it->state.GetPose().translation()).norm();
    if (dist_value < min_dist) {
      min_dist = dist_value;
      best_it = traj_it;
      time_at_match = traj_it->time;
    }
  }
  if (best_it == traj_pts.begin()) {
    ++best_it;
  }
  int last_direction = 0;
  while ((best_it != traj_pts.end()) && (best_it != traj_pts.begin())) {
    auto before_best_it = std::prev(best_it);
    std::tie(time_at_match, min_dist) = eigenmath::GoldenSectionSearchMinimize(
        before_best_it->time, best_it->time,
        [&](double t) -> double {
          auto inter_pt = before_best_it->state.ExtrapolateConstantVelocityArc(
              t - before_best_it->time);
          return (pt - inter_pt.GetPose().translation()).norm();
        },
        kEpsilon * 0.01);
    if (std::abs(time_at_match - best_it->time) < kEpsilon) {
      // Must move forward by one section
      if (last_direction < 0) {
        break;
      }
      ++best_it;
      last_direction = 1;
    } else if (std::abs(time_at_match - before_best_it->time) < kEpsilon) {
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
  return {time_at_match, Evaluate(time_at_match)};
}

StateAndTime Trajectory::FindClosestMatchInTimeInterval(
    const eigenmath::Vector2d &pt,
    const Interval<double> &time_interval) const {
  double min_dist = std::numeric_limits<double>::max();
  double time_at_match = 0.0;
  std::tie(time_at_match, min_dist) = eigenmath::GoldenSectionSearchMinimize(
      time_interval.min(), time_interval.max(),
      [this, &pt](double t) -> double {
        const auto inter_pt = Evaluate(t);
        return (pt - inter_pt.GetPose().translation()).norm();
      },
      kEpsilon * 0.01);
  return {time_at_match, Evaluate(time_at_match)};
}

double Trajectory::RmsPositionDeviation(const Trajectory &lhs,
                                        const Trajectory &rhs) {
  Interval<double> time_interval = lhs.GetTimeSpan();
  time_interval.SpanningUnion(rhs.GetTimeSpan());
  if (time_interval.Length() < kSmallestTimeStepSeconds) {
    return 0.0;
  }
  return std::sqrt(eigenmath::ComputeGaussLegendreIntegral<12>(
                       time_interval.min(), time_interval.max(),
                       [&](double t) {
                         return (lhs.Evaluate(t).GetPose().translation() -
                                 rhs.Evaluate(t).GetPose().translation())
                             .squaredNorm();
                       }) /
                   time_interval.Length());
}

}  // namespace mobility::diff_drive
