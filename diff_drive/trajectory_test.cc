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
#include <vector>

#include "eigenmath/matchers.h"
#include "diff_drive/curve_trajectory_utils.h"
#include "diff_drive/test_trajectories.h"
#include "diff_drive/test_utils.h"
#include "diff_drive/type_aliases.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace mobility::diff_drive {
namespace {

const testing::RawStateTime kExpectedTrajPts[] = {
    {0.0, 0.0, 0.0, 0.0, 1.0, 2.0 * M_PI},
    {0.25, 0.5 / M_PI, 0.5 / M_PI, 0.5 * M_PI, 1.0, 0.0},
    {1.25, 0.5 / M_PI, 1.0 + 0.5 / M_PI, 0.5 * M_PI, 1.0, -2.0 * M_PI},
    {1.75, 1.5 / M_PI, 1.0 + 0.5 / M_PI, -0.5 * M_PI, 1.0, 0.0},
    {2.75, 1.5 / M_PI, 0.5 / M_PI, -0.5 * M_PI, 1.0, 2.0 * M_PI},
    {3.0, 2.0 / M_PI, 0.0, 0.0, 1.0, 0.0}};

using eigenmath::testing::IsApprox;
using testing::IsStateAndTimeApprox;
using testing::IsStateApprox;
using testing::IsTrajectoryApprox;
using testing::kBigEpsilon;
using testing::kEpsilon;
using testing::kTestTraj;
using testing::kTestTrajEval;
using testing::ToState;
using testing::ToStateAndTime;
using testing::ToTrajectory;

TEST(Trajectory, BasicMemberFunctions) {
  EXPECT_FALSE(kTestTraj.IsEmpty());
  EXPECT_TRUE(kTestTraj.IsSane());
  Interval<double> time_span = kTestTraj.GetTimeSpan();
  EXPECT_NEAR(time_span.min(), 0.0, kEpsilon);
  EXPECT_NEAR(time_span.max(), 3.0, kEpsilon);
  EXPECT_EQ(kTestTraj, kTestTraj);
}

TEST(Trajectory, ComputeTotalCordLength) {
  const double total_cord_length = kTestTraj.ComputeTotalCordLength();
  EXPECT_NEAR(total_cord_length, 3.0, kEpsilon);
}

TEST(Trajectory, ComputeRelativeCordLength) {
  const double rel_cord_length = kTestTraj.ComputeRelativeCordLength();
  EXPECT_NEAR(rel_cord_length, 3.0 / 2.0 * M_PI, kEpsilon);
}

TEST(Trajectory, EvaluateAtT) {
  for (auto& expected_pose : kTestTrajEval) {
    const State cur_state = kTestTraj.Evaluate(expected_pose.t);
    EXPECT_THAT(cur_state, IsStateApprox(ToState(expected_pose), kBigEpsilon));
  }
}

TEST(Trajectory, EvaluateBeforeStart) {
  const State cur_state = kTestTraj.Evaluate(-0.25);
  EXPECT_THAT(cur_state, IsStateApprox(ToState({-0.25, -0.5 / M_PI, 0.5 / M_PI,
                                                -0.5 * M_PI, 1.0, 2 * M_PI}),
                                       kBigEpsilon));
}

TEST(Trajectory, EvaluateAfterEnd) {
  const State cur_state = kTestTraj.Evaluate(3.25);
  EXPECT_THAT(
      cur_state,
      IsStateApprox(ToState({3.25, 2.0 / M_PI + 0.25, 0.0, 0.0, 1.0, 0.0}),
                    kBigEpsilon));
}

TEST(Trajectory, AddStateAlmostAtEnd) {
  Trajectory added_traj = kTestTraj;
  EXPECT_TRUE(added_traj.AddState(
      3.0 + 1e-9,
      State(eigenmath::Pose2d(eigenmath::Vector2d(2.0 / M_PI, 0.0), 0.5 * M_PI),
            ArcVector(0.5, 0.25))));
  EXPECT_EQ(added_traj.GetSize(), kTestTraj.GetSize());
  const State finish_state = added_traj.GetFinish().state;
  EXPECT_NEAR(finish_state.GetPose().translation().x(), 2.0 / M_PI, kEpsilon);
  EXPECT_NEAR(finish_state.GetPose().translation().y(), 0.0, kEpsilon);
  EXPECT_THAT(finish_state.GetPose().so2(),
              IsApprox(eigenmath::SO2d(0.5 * M_PI), kEpsilon));
  EXPECT_NEAR(finish_state.GetArcVelocity().Translation(), 0.5, kEpsilon);
  EXPECT_NEAR(finish_state.GetArcVelocity().Rotation(), 0.25, kEpsilon);
}

TEST(Trajectory, HasContinuousPosition) {
  EXPECT_TRUE(kTestTraj.HasContinuousPosition());
  Trajectory discontinuous_traj = kTestTraj;
  EXPECT_TRUE(discontinuous_traj.AddState(kTestTraj.GetTimeSpan().max() + 1.0,
                                          kTestTraj.GetFinish().state));
  EXPECT_FALSE(discontinuous_traj.HasContinuousPosition());
}

TEST(Trajectory, ApplyTimeShift) {
  constexpr double kTimeShift = 0.5;
  Trajectory shifted_traj = kTestTraj;
  shifted_traj.ApplyTimeShift(kTimeShift);
  for (auto& expected_pose : kTestTrajEval) {
    State cur_state = shifted_traj.Evaluate(expected_pose.t + kTimeShift);
    EXPECT_THAT(cur_state, IsStateApprox(ToState(expected_pose), kBigEpsilon));
  }
}

TEST(Trajectory, ApplyTransform) {
  Trajectory moved_traj = kTestTraj;
  const eigenmath::Pose2d new_pose_old(eigenmath::Vector2d(1.0, 3.0),
                                       0.5 * M_PI);
  moved_traj.ApplyTransform(new_pose_old);
  for (auto& expected_pose : kTestTrajEval) {
    const State cur_state = moved_traj.Evaluate(expected_pose.t);
    State cur_retransformed_state = cur_state;
    cur_retransformed_state.SetPose(new_pose_old.inverse() *
                                    cur_retransformed_state.GetPose());
    EXPECT_THAT(cur_retransformed_state,
                IsStateApprox(ToState(expected_pose), kBigEpsilon));
  }
}

TEST(Trajectory, GetStartAndFinish) {
  EXPECT_THAT(
      kTestTraj.GetStart(),
      IsStateAndTimeApprox(ToStateAndTime(kTestTrajEval.front()), kBigEpsilon));
  EXPECT_THAT(
      kTestTraj.GetFinish(),
      IsStateAndTimeApprox(ToStateAndTime(kTestTrajEval.back()), kBigEpsilon));
}

TEST(Trajectory, TimeIterator) {
  double prev_time = 0.0;
  Trajectory::TimeIterator it = kTestTraj.BeginInTime();
  Trajectory::TimeIterator it_end = kTestTraj.EndInTime();
  for (int i = 0; i < kTestTrajEval.size(); ++i) {
    auto it_pre = (kTestTrajEval[i].t - prev_time) + it;
    auto it_post = it + (kTestTrajEval[i].t - prev_time);
    it += kTestTrajEval[i].t - prev_time;
    EXPECT_NEAR(it.GetTime(), kTestTrajEval[i].t, kEpsilon);
    EXPECT_NEAR(it_pre.GetTime(), kTestTrajEval[i].t, kEpsilon);
    EXPECT_NEAR(it_post.GetTime(), kTestTrajEval[i].t, kEpsilon);
    if (i == kTestTrajEval.size() - 1) {
      EXPECT_FALSE(it < it_end);
      EXPECT_FALSE(it_pre < it_end);
      EXPECT_FALSE(it_post < it_end);
    } else {
      EXPECT_TRUE(it < it_end);
      EXPECT_TRUE(it_pre < it_end);
      EXPECT_TRUE(it_post < it_end);
    }

    EXPECT_THAT(it.GetState(),
                IsStateApprox(ToState(kTestTrajEval[i]), kBigEpsilon));

    EXPECT_THAT(it_pre.GetState(),
                IsStateApprox(ToState(kTestTrajEval[i]), kBigEpsilon));

    EXPECT_THAT(it_post.GetState(),
                IsStateApprox(ToState(kTestTrajEval[i]), kBigEpsilon));

    prev_time = kTestTrajEval[i].t;
  }
  // Check if termination works past the end too:
  it += kBigEpsilon;
  EXPECT_FALSE(it < it_end);
}

TEST(Trajectory, TimeIteratorWithEmpty) {
  Trajectory traj(kTestTrajEval.size());

  Trajectory::TimeIterator it = traj.BeginInTime();
  Trajectory::TimeIterator it_end = traj.EndInTime();

  EXPECT_FALSE(it < it_end);
}

TEST(Trajectory, CordLengthIterator) {
  // Cord-length happens to equal time, because velocity always equals 1.0
  double prev_time = 0.0;
  Trajectory::CordLengthIterator it = kTestTraj.BeginInCordLength();
  Trajectory::CordLengthIterator it_end = kTestTraj.EndInCordLength();
  for (int i = 0; i < kTestTrajEval.size(); ++i) {
    auto it_pre = (kTestTrajEval[i].t - prev_time) + it;
    auto it_post = it + (kTestTrajEval[i].t - prev_time);
    it += kTestTrajEval[i].t - prev_time;
    if (i == kTestTrajEval.size() - 1) {
      EXPECT_FALSE(it < it_end);
      EXPECT_FALSE(it_pre < it_end);
      EXPECT_FALSE(it_post < it_end);
    } else {
      EXPECT_TRUE(it < it_end);
      EXPECT_TRUE(it_pre < it_end);
      EXPECT_TRUE(it_post < it_end);
    }

    EXPECT_THAT(it.GetState(),
                IsStateApprox(ToState(kTestTrajEval[i]), kBigEpsilon));

    EXPECT_THAT(it_pre.GetState(),
                IsStateApprox(ToState(kTestTrajEval[i]), kBigEpsilon));

    EXPECT_THAT(it_post.GetState(),
                IsStateApprox(ToState(kTestTrajEval[i]), kBigEpsilon));

    prev_time = kTestTrajEval[i].t;
  }
  // Check if termination works past the end too:
  it += kBigEpsilon;
  EXPECT_FALSE(it < it_end);
}

TEST(Trajectory, CordLengthIteratorWithStop) {
  Trajectory traj(16);
  EXPECT_TRUE(traj.AddState(StateAndTime{
      0.0, State(eigenmath::Pose2d(eigenmath::Vector2d(0.0, 0.0), 0.0),
                 ArcVector{1.0, 0.0})}));
  EXPECT_TRUE(traj.AddState(StateAndTime{
      1.0, State(eigenmath::Pose2d(eigenmath::Vector2d(1.0, 0.0), 0.0),
                 ArcVector{1.0, 0.0})}));
  EXPECT_TRUE(traj.AddState(StateAndTime{
      2.0, State(eigenmath::Pose2d(eigenmath::Vector2d(2.0, 0.0), 0.0),
                 ArcVector{0.0, 0.0})}));
  EXPECT_TRUE(traj.AddState(StateAndTime{
      3.0, State(eigenmath::Pose2d(eigenmath::Vector2d(2.0, 0.0), 0.0),
                 ArcVector{0.0, 0.0})}));
  EXPECT_TRUE(traj.AddState(StateAndTime{
      4.0, State(eigenmath::Pose2d(eigenmath::Vector2d(2.0, 0.0), 0.0),
                 ArcVector{1.0, 0.0})}));
  EXPECT_TRUE(traj.AddState(StateAndTime{
      5.0, State(eigenmath::Pose2d(eigenmath::Vector2d(3.0, 0.0), 0.0),
                 ArcVector{1.0, 0.0})}));
  EXPECT_TRUE(traj.AddState(StateAndTime{
      6.0, State(eigenmath::Pose2d(eigenmath::Vector2d(4.0, 0.0), 0.0),
                 ArcVector{0.0, 0.0})}));
  EXPECT_TRUE(traj.AddState(StateAndTime{
      7.0, State(eigenmath::Pose2d(eigenmath::Vector2d(4.0, 0.0), 0.0),
                 ArcVector{0.0, 0.0})}));

  Trajectory::CordLengthIterator it = traj.BeginInCordLength();
  Trajectory::CordLengthIterator it_end = traj.EndInCordLength();

  EXPECT_THAT(
      it.GetState(),
      IsStateApprox(ToState({0.0, 0.0, 0.0, 0.0, 1.0, 0.0}), kBigEpsilon));

  it += 1.0;
  EXPECT_THAT(
      it.GetState(),
      IsStateApprox(ToState({1.0, 1.0, 0.0, 0.0, 1.0, 0.0}), kBigEpsilon));

  it += kEpsilon;
  EXPECT_THAT(
      it.GetState(),
      IsStateApprox(ToState({2.0, 1.0, 0.0, 0.0, 1.0, 0.0}), kBigEpsilon));

  it += 1.0;
  EXPECT_THAT(
      it.GetState(),
      IsStateApprox(ToState({4.0, 2.0, 0.0, 0.0, 1.0, 0.0}), kBigEpsilon));

  it += 1.0;
  EXPECT_THAT(
      it.GetState(),
      IsStateApprox(ToState({5.0, 3.0, 0.0, 0.0, 1.0, 0.0}), kBigEpsilon));

  it += 1.0;
  EXPECT_THAT(
      it.GetState(),
      IsStateApprox(ToState({6.0, 4.0, 0.0, 0.0, 0.0, 0.0}), kBigEpsilon));

  // Check if termination works past the end too:
  EXPECT_FALSE(it < it_end);
}

TEST(Trajectory, CordLengthIteratorWithEmpty) {
  Trajectory traj(16);

  Trajectory::CordLengthIterator it = traj.BeginInCordLength();
  Trajectory::CordLengthIterator it_end = traj.EndInCordLength();

  EXPECT_FALSE(it < it_end);
}

TEST(Trajectory, StateIterator) {
  int i = 0;
  for (auto& state : kTestTraj.GetStateIteratorRange()) {
    EXPECT_THAT(state, IsStateAndTimeApprox(ToStateAndTime(kExpectedTrajPts[i]),
                                            kBigEpsilon))
        << "at index " << i;
    ++i;
  }
}

TEST(Trajectory, AppendStates) {
  Trajectory first_traj = kTestTraj;
  first_traj.TruncateTo(Interval<double>(0.0, 1.25));
  Trajectory second_traj = kTestTraj;
  second_traj.TruncateTo(Interval<double>(1.25, 3.0));
  EXPECT_NE(first_traj, second_traj);
  Trajectory combined_traj = kTestTraj;
  combined_traj = first_traj;
  EXPECT_TRUE(combined_traj.AppendStates(second_traj.BeginState(),
                                         second_traj.EndState()));
  EXPECT_THAT(combined_traj, IsTrajectoryApprox(kTestTraj, kBigEpsilon));
}

TEST(Trajectory, AppendStatesEmpty) {
  Trajectory second_traj;
  Trajectory combined_traj = kTestTraj;
  EXPECT_TRUE(combined_traj.AppendStates(second_traj.BeginState(),
                                         second_traj.EndState()));
  EXPECT_THAT(combined_traj, IsTrajectoryApprox(kTestTraj, kBigEpsilon));
}

TEST(Trajectory, AppendStatesOverCapacity) {
  Trajectory first_traj = kTestTraj;
  first_traj.TruncateTo(Interval<double>(0.0, 1.25));
  EXPECT_EQ(first_traj.GetSize(), 3);
  Trajectory second_traj = kTestTraj;
  second_traj.TruncateTo(Interval<double>(1.25, 3.0));
  EXPECT_EQ(second_traj.GetSize(), 4);
  Trajectory combined_traj(4);
  combined_traj = first_traj;
  EXPECT_EQ(combined_traj.GetSize(), 3);
  EXPECT_FALSE(combined_traj.AppendStates(second_traj.BeginState(),
                                          second_traj.EndState()));
  EXPECT_EQ(combined_traj.GetCapacity(), 4);
  EXPECT_EQ(combined_traj.GetSize(), 4);
  Trajectory third_traj = kTestTraj;
  third_traj.TruncateTo(Interval<double>(0.0, 1.75));
  EXPECT_THAT(combined_traj, IsTrajectoryApprox(third_traj, kBigEpsilon));
}

TEST(Trajectory, AddIntermediateState) {
  std::vector<testing::RawStateTime> new_expected_traj_pts;
  new_expected_traj_pts.insert(new_expected_traj_pts.end(),
                               std::begin(kExpectedTrajPts),
                               std::begin(kExpectedTrajPts) + 2);
  new_expected_traj_pts.push_back(
      {0.5 * (kExpectedTrajPts[1].t + kExpectedTrajPts[2].t),
       0.5 * (kExpectedTrajPts[1].x + kExpectedTrajPts[2].x),
       0.5 * (kExpectedTrajPts[1].y + kExpectedTrajPts[2].y),
       0.5 * (kExpectedTrajPts[1].theta + kExpectedTrajPts[2].theta),
       kExpectedTrajPts[1].v, kExpectedTrajPts[1].w});
  new_expected_traj_pts.insert(new_expected_traj_pts.end(),
                               std::begin(kExpectedTrajPts) + 2,
                               std::end(kExpectedTrajPts));

  Trajectory inserted_traj = kTestTraj;
  auto inter_it = inserted_traj.AddIntermediateState(
      0.5 * (kExpectedTrajPts[1].t + kExpectedTrajPts[2].t));
  EXPECT_NE(inter_it, inserted_traj.EndState());
  EXPECT_THAT(
      inserted_traj,
      IsTrajectoryApprox(ToTrajectory(new_expected_traj_pts), kBigEpsilon));
}

TEST(Trajectory, AddIntermediateStateBeforeStart) {
  std::vector<testing::RawStateTime> new_expected_traj_pts;
  new_expected_traj_pts.push_back({-0.5, 0.0, 1.0 / M_PI, M_PI,
                                   kExpectedTrajPts[0].v,
                                   kExpectedTrajPts[0].w});
  new_expected_traj_pts.insert(new_expected_traj_pts.end(),
                               std::begin(kExpectedTrajPts),
                               std::end(kExpectedTrajPts));

  Trajectory inserted_traj = kTestTraj;
  auto inter_it = inserted_traj.AddIntermediateState(-0.5);
  EXPECT_NE(inter_it, inserted_traj.EndState());
  EXPECT_THAT(
      inserted_traj,
      IsTrajectoryApprox(ToTrajectory(new_expected_traj_pts), kBigEpsilon));
}

TEST(Trajectory, AddIntermediateStateAtStart) {
  Trajectory inserted_traj = kTestTraj;
  auto inter_it = inserted_traj.AddIntermediateState(-1e-9);
  EXPECT_NE(inter_it, inserted_traj.EndState());
  EXPECT_THAT(inserted_traj, IsTrajectoryApprox(kTestTraj, kBigEpsilon));
}

TEST(Trajectory, AddIntermediateStateAtState) {
  Trajectory inserted_traj = kTestTraj;
  auto inter_it = inserted_traj.AddIntermediateState(kExpectedTrajPts[1].t);
  EXPECT_NE(inter_it, inserted_traj.EndState());
  EXPECT_THAT(inserted_traj, IsTrajectoryApprox(kTestTraj, kBigEpsilon));
}

TEST(Trajectory, AddStationarySegment) {
  constexpr double kTimeOffset = 0.5;
  std::vector<testing::RawStateTime> new_expected_traj_pts;
  new_expected_traj_pts.insert(new_expected_traj_pts.end(),
                               std::begin(kExpectedTrajPts),
                               std::begin(kExpectedTrajPts) + 2);
  new_expected_traj_pts.push_back(
      {0.5 * (kExpectedTrajPts[1].t + kExpectedTrajPts[2].t),
       0.5 * (kExpectedTrajPts[1].x + kExpectedTrajPts[2].x),
       0.5 * (kExpectedTrajPts[1].y + kExpectedTrajPts[2].y),
       0.5 * (kExpectedTrajPts[1].theta + kExpectedTrajPts[2].theta), 0.0,
       0.0});
  new_expected_traj_pts.push_back(
      {0.5 * (kExpectedTrajPts[1].t + kExpectedTrajPts[2].t) + kTimeOffset,
       0.5 * (kExpectedTrajPts[1].x + kExpectedTrajPts[2].x),
       0.5 * (kExpectedTrajPts[1].y + kExpectedTrajPts[2].y),
       0.5 * (kExpectedTrajPts[1].theta + kExpectedTrajPts[2].theta),
       kExpectedTrajPts[1].v, kExpectedTrajPts[1].w});
  new_expected_traj_pts.insert(new_expected_traj_pts.end(),
                               std::begin(kExpectedTrajPts) + 2,
                               std::end(kExpectedTrajPts));
  for (auto it = new_expected_traj_pts.begin() + 4;
       it != new_expected_traj_pts.end(); ++it) {
    it->t += kTimeOffset;
  }

  Trajectory inserted_traj = kTestTraj;
  auto inter_it = inserted_traj.AddStationarySegment(
      0.5 * (kExpectedTrajPts[1].t + kExpectedTrajPts[2].t), kTimeOffset);
  ASSERT_NE(inter_it, inserted_traj.EndState());
  EXPECT_NEAR(inter_it->state.GetArcVelocity().Translation(), 0.0, kEpsilon);
  EXPECT_NEAR(inter_it->state.GetArcVelocity().Rotation(), 0.0, kEpsilon);
  EXPECT_THAT(
      inserted_traj,
      IsTrajectoryApprox(ToTrajectory(new_expected_traj_pts), kBigEpsilon));
}

TEST(Trajectory, AddStationarySegmentBeforeStart) {
  constexpr double kTimeOffset = 0.5;
  std::vector<testing::RawStateTime> new_expected_traj_pts;
  new_expected_traj_pts.push_back({-0.5, 0.0, 1.0 / M_PI, M_PI, 0.0, 0.0});
  new_expected_traj_pts.push_back({0.0, 0.0, 1.0 / M_PI, M_PI,
                                   kExpectedTrajPts[0].v,
                                   kExpectedTrajPts[0].w});
  new_expected_traj_pts.insert(new_expected_traj_pts.end(),
                               std::begin(kExpectedTrajPts),
                               std::end(kExpectedTrajPts));
  for (auto it = new_expected_traj_pts.begin() + 2;
       it != new_expected_traj_pts.end(); ++it) {
    it->t += kTimeOffset;
  }

  Trajectory inserted_traj = kTestTraj;
  auto inter_it = inserted_traj.AddStationarySegment(-0.5, kTimeOffset);
  ASSERT_NE(inter_it, inserted_traj.EndState());
  EXPECT_NEAR(inter_it->state.GetArcVelocity().Translation(), 0.0, kEpsilon);
  EXPECT_NEAR(inter_it->state.GetArcVelocity().Rotation(), 0.0, kEpsilon);
  EXPECT_THAT(
      inserted_traj,
      IsTrajectoryApprox(ToTrajectory(new_expected_traj_pts), kBigEpsilon));
}

TEST(Trajectory, AddStationarySegmentAtStart) {
  constexpr double kTimeOffset = 0.5;
  std::vector<testing::RawStateTime> new_expected_traj_pts;
  new_expected_traj_pts.push_back({-1e-9, 0.0, 0.0, 0.0, 0.0, 0.0});
  new_expected_traj_pts.insert(new_expected_traj_pts.end(),
                               std::begin(kExpectedTrajPts),
                               std::end(kExpectedTrajPts));
  for (auto it = new_expected_traj_pts.begin() + 1;
       it != new_expected_traj_pts.end(); ++it) {
    it->t += kTimeOffset;
  }

  Trajectory inserted_traj = kTestTraj;
  auto inter_it = inserted_traj.AddStationarySegment(-1e-9, kTimeOffset);
  ASSERT_NE(inter_it, inserted_traj.EndState());
  EXPECT_NEAR(inter_it->state.GetArcVelocity().Translation(), 0.0, kEpsilon);
  EXPECT_NEAR(inter_it->state.GetArcVelocity().Rotation(), 0.0, kEpsilon);
  EXPECT_THAT(
      inserted_traj,
      IsTrajectoryApprox(ToTrajectory(new_expected_traj_pts), kBigEpsilon));
}

TEST(Trajectory, AddStationarySegmentAtState) {
  constexpr double kTimeOffset = 0.5;
  std::vector<testing::RawStateTime> new_expected_traj_pts;
  new_expected_traj_pts.insert(new_expected_traj_pts.end(),
                               std::begin(kExpectedTrajPts),
                               std::begin(kExpectedTrajPts) + 1);
  new_expected_traj_pts.push_back({kExpectedTrajPts[1].t, kExpectedTrajPts[1].x,
                                   kExpectedTrajPts[1].y,
                                   kExpectedTrajPts[1].theta, 0.0, 0.0});
  new_expected_traj_pts.insert(new_expected_traj_pts.end(),
                               std::begin(kExpectedTrajPts) + 1,
                               std::end(kExpectedTrajPts));
  for (auto it = new_expected_traj_pts.begin() + 2;
       it != new_expected_traj_pts.end(); ++it) {
    it->t += kTimeOffset;
  }

  Trajectory inserted_traj = kTestTraj;
  auto inter_it =
      inserted_traj.AddStationarySegment(kExpectedTrajPts[1].t, kTimeOffset);
  ASSERT_NE(inter_it, inserted_traj.EndState());
  EXPECT_NEAR(inter_it->state.GetArcVelocity().Translation(), 0.0, kEpsilon);
  EXPECT_NEAR(inter_it->state.GetArcVelocity().Rotation(), 0.0, kEpsilon);
  EXPECT_THAT(
      inserted_traj,
      IsTrajectoryApprox(ToTrajectory(new_expected_traj_pts), kBigEpsilon));
}

TEST(Trajectory, RemoveStationarySegment) {
  Trajectory inserted_traj = kTestTraj;
  constexpr double kTimeOffset = 0.5;
  auto inter_it =
      inserted_traj.AddStationarySegment(kExpectedTrajPts[1].t, kTimeOffset);
  ASSERT_NE(inter_it, inserted_traj.EndState());
  inserted_traj.RemoveStationarySegment(inter_it);
  EXPECT_THAT(inserted_traj, IsTrajectoryApprox(kTestTraj, kBigEpsilon));
}

TEST(Trajectory, RemoveStationarySegmentPartial) {
  constexpr double kTimeOffset = 0.5;
  std::vector<testing::RawStateTime> new_expected_traj_pts;
  new_expected_traj_pts.insert(new_expected_traj_pts.end(),
                               std::begin(kExpectedTrajPts),
                               std::begin(kExpectedTrajPts) + 1);
  new_expected_traj_pts.push_back({kExpectedTrajPts[1].t, kExpectedTrajPts[1].x,
                                   kExpectedTrajPts[1].y,
                                   kExpectedTrajPts[1].theta, 0.0, 0.0});
  new_expected_traj_pts.insert(new_expected_traj_pts.end(),
                               std::begin(kExpectedTrajPts) + 1,
                               std::end(kExpectedTrajPts));
  for (auto it = new_expected_traj_pts.begin() + 2;
       it != new_expected_traj_pts.end(); ++it) {
    it->t += kTimeOffset * 0.5;
  }

  Trajectory inserted_traj = kTestTraj;
  auto inter_it =
      inserted_traj.AddStationarySegment(kExpectedTrajPts[1].t, kTimeOffset);
  ASSERT_NE(inter_it, inserted_traj.EndState());
  inserted_traj.RemoveStationarySegment(inter_it, kTimeOffset * 0.5);
  EXPECT_THAT(
      inserted_traj,
      IsTrajectoryApprox(ToTrajectory(new_expected_traj_pts), kBigEpsilon));
}

TEST(Trajectory, TimeWarp) {
  constexpr double kSpeedFactor = 0.4;
  std::vector<testing::RawStateTime> new_expected_traj_pts;
  new_expected_traj_pts.insert(new_expected_traj_pts.end(),
                               std::begin(kExpectedTrajPts),
                               std::end(kExpectedTrajPts));
  for (auto& pt : new_expected_traj_pts) {
    pt.t /= kSpeedFactor;
    pt.v *= kSpeedFactor;
    pt.w *= kSpeedFactor;
  }

  Trajectory warped_traj = kTestTraj;
  warped_traj.TimeWarp(kSpeedFactor);
  EXPECT_THAT(
      warped_traj,
      IsTrajectoryApprox(ToTrajectory(new_expected_traj_pts), kBigEpsilon));
}

TEST(Trajectory, TimeWarpSegment) {
  constexpr double kSpeedFactor = 0.4;
  constexpr double kTimeOffset = 1.0 / kSpeedFactor - 1.0;
  std::vector<testing::RawStateTime> new_expected_traj_pts;
  new_expected_traj_pts.insert(new_expected_traj_pts.end(),
                               std::begin(kExpectedTrajPts),
                               std::end(kExpectedTrajPts));
  new_expected_traj_pts[1].v *= kSpeedFactor;
  new_expected_traj_pts[1].w *= kSpeedFactor;
  for (int i = 2; i < new_expected_traj_pts.size(); ++i) {
    new_expected_traj_pts[i].t += kTimeOffset;
  }

  Trajectory warped_traj = kTestTraj;
  warped_traj.TimeWarpSegment(kSpeedFactor, warped_traj.BeginState() + 1);
  EXPECT_THAT(
      warped_traj,
      IsTrajectoryApprox(ToTrajectory(new_expected_traj_pts), kBigEpsilon));

  // Warp end state:
  warped_traj.TimeWarpSegment(kSpeedFactor, std::prev(warped_traj.EndState()));
  new_expected_traj_pts.back().v *= kSpeedFactor;
  new_expected_traj_pts.back().w *= kSpeedFactor;
  EXPECT_THAT(
      warped_traj,
      IsTrajectoryApprox(ToTrajectory(new_expected_traj_pts), kBigEpsilon));

  // Warp past the end:
  warped_traj.TimeWarpSegment(kSpeedFactor, warped_traj.EndState());
  EXPECT_THAT(
      warped_traj,
      IsTrajectoryApprox(ToTrajectory(new_expected_traj_pts), kBigEpsilon));
}

TEST(Trajectory, ReverseTrajectory) {
  const double t_offset = std::rbegin(kExpectedTrajPts)->t;
  std::vector<testing::RawStateTime> new_expected_traj_pts;
  new_expected_traj_pts.insert(new_expected_traj_pts.end(),
                               std::begin(kExpectedTrajPts),
                               std::end(kExpectedTrajPts));
  for (int i = new_expected_traj_pts.size() - 1; i > 0; --i) {
    new_expected_traj_pts[i].t = t_offset - new_expected_traj_pts[i].t;
    new_expected_traj_pts[i].v = -new_expected_traj_pts[i - 1].v;
    new_expected_traj_pts[i].w = -new_expected_traj_pts[i - 1].w;
  }
  new_expected_traj_pts[0].t = t_offset - new_expected_traj_pts[0].t;
  new_expected_traj_pts[0].v = -new_expected_traj_pts[0].v;
  new_expected_traj_pts[0].w = -new_expected_traj_pts[0].w;
  std::reverse(new_expected_traj_pts.begin(), new_expected_traj_pts.end());

  Trajectory rev_traj = kTestTraj;
  rev_traj.Reverse();
  EXPECT_NE(kTestTraj, rev_traj);
  EXPECT_THAT(rev_traj, IsTrajectoryApprox(ToTrajectory(new_expected_traj_pts),
                                           kBigEpsilon));
}

TEST(Trajectory, ClearedTrajectory) {
  Trajectory cleared_traj = kTestTraj;
  EXPECT_FALSE(cleared_traj.IsEmpty());
  EXPECT_TRUE(cleared_traj.IsSane());
  cleared_traj.Clear();
  EXPECT_TRUE(cleared_traj.IsEmpty());
  EXPECT_FALSE(cleared_traj.IsSane());

  Interval<double> time_span = cleared_traj.GetTimeSpan();
  EXPECT_TRUE(time_span.Empty());

  // Check if termination works for empty time-iteration:
  EXPECT_FALSE(cleared_traj.BeginInTime() < cleared_traj.EndInTime());

  // Check if termination works for empty cord-iteration:
  EXPECT_FALSE(cleared_traj.BeginInCordLength() <
               cleared_traj.EndInCordLength());

  const StateAndTime start = cleared_traj.GetStart();
  EXPECT_NEAR(start.time, 0.0, kBigEpsilon);
  EXPECT_NEAR(start.state.GetPose().translation().x(), 0.0, kBigEpsilon);
  EXPECT_NEAR(start.state.GetPose().translation().y(), 0.0, kBigEpsilon);
  EXPECT_THAT(start.state.GetPose().so2(),
              IsApprox(eigenmath::SO2d(0.0), kBigEpsilon));

  const StateAndTime finish = cleared_traj.GetFinish();
  EXPECT_NEAR(finish.time, 0.0, kBigEpsilon);
  EXPECT_NEAR(finish.state.GetPose().translation().x(), 0.0, kBigEpsilon);
  EXPECT_NEAR(finish.state.GetPose().translation().y(), 0.0, kBigEpsilon);
  EXPECT_THAT(finish.state.GetPose().so2(),
              IsApprox(eigenmath::SO2d(0.0), kBigEpsilon));
}

TEST(Trajectory, SetCapacity) {
  Trajectory new_traj(10);
  EXPECT_GE(new_traj.GetCapacity(), 10);
  new_traj.SetMinCapacity(5);
  EXPECT_GE(new_traj.GetCapacity(), 10);
  new_traj.SetMinCapacity(50);
  EXPECT_GE(new_traj.GetCapacity(), 50);
}

TEST(Trajectory, AppendLimitedCapacity) {
  Trajectory traj1 = ToTrajectory({{0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
                                   {1.0, 1.0, 0.0, 0.0, 1.0, 0.0},
                                   {2.0, 2.0, 0.0, 0.0, 1.0, 0.0}});

  Trajectory traj2 = ToTrajectory(
      {{2.0, 2.0, 0.0, 0.0, 1.0, 0.0}, {3.0, 3.0, 0.0, 0.0, 1.0, 0.0}});

  EXPECT_FALSE(traj1.AppendTrajectory(traj2));

  Trajectory traj3(4);
  EXPECT_TRUE(traj3.AppendTrajectory(traj1));
  EXPECT_TRUE(traj3.AppendTrajectory(traj2));
  EXPECT_EQ(traj3.GetSize(), 4);

  const Interval<double> time_span = traj3.GetTimeSpan();
  EXPECT_NEAR(time_span.min(), 0.0, kEpsilon);
  EXPECT_NEAR(time_span.max(), 3.0, kEpsilon);
}

TEST(Trajectory, PrependLimitedCapacity) {
  Trajectory traj1 = ToTrajectory({{0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
                                   {1.0, 1.0, 0.0, 0.0, 1.0, 0.0},
                                   {2.0, 2.0, 0.0, 0.0, 1.0, 0.0}});

  Trajectory traj2 = ToTrajectory(
      {{2.0, 2.0, 0.0, 0.0, 1.0, 0.0}, {3.0, 3.0, 0.0, 0.0, 1.0, 0.0}});

  EXPECT_FALSE(traj2.PrependTrajectory(traj1));

  Trajectory traj3(4);
  EXPECT_TRUE(traj3.PrependTrajectory(traj2));
  EXPECT_TRUE(traj3.PrependTrajectory(traj1));
  EXPECT_EQ(traj3.GetSize(), 4);

  const Interval<double> time_span = traj3.GetTimeSpan();
  EXPECT_NEAR(time_span.min(), 0.0, kEpsilon);
  EXPECT_NEAR(time_span.max(), 3.0, kEpsilon);
}

TEST(Trajectory, SplitAndCombineWithAppendAndPrepend) {
  Trajectory test_traj_part1 = kTestTraj;
  test_traj_part1.TruncateTo(Interval<double>(0.0, 1.25));

  Trajectory test_traj_part2 = kTestTraj;
  test_traj_part2.TruncateTo(Interval<double>(1.25, 3.0));

  Trajectory appended_traj(kTestTraj.GetCapacity());
  EXPECT_TRUE(appended_traj.AppendTrajectory(test_traj_part1));
  EXPECT_TRUE(appended_traj.AppendTrajectory(test_traj_part2));
  EXPECT_THAT(appended_traj, IsTrajectoryApprox(kTestTraj, kEpsilon));

  Trajectory prepended_traj(kTestTraj.GetCapacity());
  EXPECT_TRUE(prepended_traj.PrependTrajectory(test_traj_part2));
  EXPECT_TRUE(prepended_traj.PrependTrajectory(test_traj_part1));
  EXPECT_THAT(prepended_traj, IsTrajectoryApprox(kTestTraj, kEpsilon));
}

TEST(Trajectory, MatchPointToTrajectory) {
  const auto match1 = kTestTraj.FindClosestMatchToPoint({-1.0, 0.0});
  EXPECT_THAT(match1, IsStateAndTimeApprox(
                          ToStateAndTime({0.0, 0.0, 0.0, 0.0, 1.0, 2 * M_PI}),
                          kBigEpsilon));

  const auto match2 = kTestTraj.FindClosestMatchToPoint({0.0, 0.1});
  EXPECT_THAT(match2, IsStateAndTimeApprox(
                          ToStateAndTime({0.0, 0.0, 0.0, 0.0, 1.0, 2 * M_PI}),
                          kBigEpsilon));

  const auto match3 =
      kTestTraj.FindClosestMatchToPoint({2.0 / M_PI + 1.0, 0.0});
  EXPECT_THAT(match3, IsStateAndTimeApprox(ToStateAndTime({3.0, 2.0 / M_PI, 0.0,
                                                           0.0, 1.0, 2 * M_PI}),
                                           kBigEpsilon));

  const auto match4 = kTestTraj.FindClosestMatchToPoint({2.0 / M_PI, 0.1});
  EXPECT_THAT(match4, IsStateAndTimeApprox(ToStateAndTime({3.0, 2.0 / M_PI, 0.0,
                                                           0.0, 1.0, 2 * M_PI}),
                                           kBigEpsilon));

  const auto match5 = kTestTraj.FindClosestMatchToPoint({kEpsilon, 0.5 / M_PI});
  EXPECT_THAT(match5,
              IsStateAndTimeApprox(ToStateAndTime({0.25, 0.5 / M_PI, 0.5 / M_PI,
                                                   M_PI_2, 1.0, 0.0}),
                                   kBigEpsilon));

  const auto match6 =
      kTestTraj.FindClosestMatchToPoint({0.0, 0.5 / M_PI + 1.0});
  EXPECT_THAT(match6, IsStateAndTimeApprox(
                          ToStateAndTime({1.25, 0.5 / M_PI, 1.0 + 0.5 / M_PI,
                                          M_PI_2, 1.0, -2 * M_PI}),
                          kBigEpsilon));
}

TEST(Trajectory, FindClosestMatchInTimeInterval) {
  const auto match1 =
      kTestTraj.FindClosestMatchInTimeInterval({-1.0, 0.0}, {0.0, 1.0});
  EXPECT_THAT(match1, IsStateAndTimeApprox(
                          ToStateAndTime({0.0, 0.0, 0.0, 0.0, 1.0, 2 * M_PI}),
                          kBigEpsilon));

  const auto match2 =
      kTestTraj.FindClosestMatchInTimeInterval({0.0, 0.1}, {0.0, 1.0});
  EXPECT_THAT(match2, IsStateAndTimeApprox(
                          ToStateAndTime({0.0, 0.0, 0.0, 0.0, 1.0, 2 * M_PI}),
                          kBigEpsilon));

  const auto match3 =
      kTestTraj.FindClosestMatchInTimeInterval({0.0, 0.1}, {0.25, 1.0});
  EXPECT_THAT(match3,
              IsStateAndTimeApprox(ToStateAndTime({0.25, 0.5 / M_PI, 0.5 / M_PI,
                                                   M_PI_2, 1.0, 0.0}),
                                   kBigEpsilon));
}

TEST(Trajectory, TruncateToTimeSpan) {
  Trajectory traj(10);
  traj = ToTrajectory({{0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
                       {1.0, 1.0, 0.0, 0.0, 1.0, 0.0},
                       {2.0, 2.0, 0.0, 0.0, 1.0, 0.0},
                       {3.0, 3.0, 0.0, 0.0, 1.0, 0.0},
                       {4.0, 4.0, 0.0, 0.0, 1.0, 0.0}});

  traj.TruncateOrExtendTo(traj.GetTimeSpan());
  EXPECT_THAT(traj.GetStart(),
              IsStateAndTimeApprox(
                  ToStateAndTime({0.0, 0.0, 0.0, 0.0, 1.0, 0.0}), kEpsilon));
  EXPECT_THAT(traj.GetFinish(),
              IsStateAndTimeApprox(
                  ToStateAndTime({4.0, 4.0, 0.0, 0.0, 1.0, 0.0}), kEpsilon));
  EXPECT_EQ(traj.GetSize(), 5);

  traj.TruncateOrExtendTo(Interval<double>(0.0, 3.5));
  EXPECT_THAT(traj.GetStart(),
              IsStateAndTimeApprox(
                  ToStateAndTime({0.0, 0.0, 0.0, 0.0, 1.0, 0.0}), kEpsilon));
  EXPECT_THAT(traj.GetFinish(),
              IsStateAndTimeApprox(
                  ToStateAndTime({3.5, 3.5, 0.0, 0.0, 1.0, 0.0}), kEpsilon));
  EXPECT_EQ(traj.GetSize(), 5);

  traj.TruncateOrExtendTo(Interval<double>(0.5, 3.5));
  EXPECT_THAT(traj.GetStart(),
              IsStateAndTimeApprox(
                  ToStateAndTime({0.5, 0.5, 0.0, 0.0, 1.0, 0.0}), kEpsilon));
  EXPECT_THAT(traj.GetFinish(),
              IsStateAndTimeApprox(
                  ToStateAndTime({3.5, 3.5, 0.0, 0.0, 1.0, 0.0}), kEpsilon));
  EXPECT_EQ(traj.GetSize(), 5);

  traj.TruncateOrExtendTo(Interval<double>(1.0, 3.5));
  EXPECT_THAT(traj.GetStart(),
              IsStateAndTimeApprox(
                  ToStateAndTime({1.0, 1.0, 0.0, 0.0, 1.0, 0.0}), kEpsilon));
  EXPECT_THAT(traj.GetFinish(),
              IsStateAndTimeApprox(
                  ToStateAndTime({3.5, 3.5, 0.0, 0.0, 1.0, 0.0}), kEpsilon));
  EXPECT_EQ(traj.GetSize(), 4);

  traj.TruncateOrExtendTo(Interval<double>(1.0, 3.0));
  EXPECT_THAT(traj.GetStart(),
              IsStateAndTimeApprox(
                  ToStateAndTime({1.0, 1.0, 0.0, 0.0, 1.0, 0.0}), kEpsilon));
  EXPECT_THAT(traj.GetFinish(),
              IsStateAndTimeApprox(
                  ToStateAndTime({3.0, 3.0, 0.0, 0.0, 1.0, 0.0}), kEpsilon));
  EXPECT_EQ(traj.GetSize(), 3);

  traj.TruncateOrExtendTo(Interval<double>(3.5, 4.0));
  EXPECT_THAT(traj.GetStart(),
              IsStateAndTimeApprox(
                  ToStateAndTime({3.5, 3.5, 0.0, 0.0, 1.0, 0.0}), kEpsilon));
  EXPECT_THAT(traj.GetFinish(),
              IsStateAndTimeApprox(
                  ToStateAndTime({4.0, 4.0, 0.0, 0.0, 1.0, 0.0}), kEpsilon));
  EXPECT_EQ(traj.GetSize(), 2);

  traj.TruncateOrExtendTo(Interval<double>(0.0, 0.5));
  EXPECT_THAT(traj.GetStart(),
              IsStateAndTimeApprox(
                  ToStateAndTime({0.0, 0.0, 0.0, 0.0, 1.0, 0.0}), kEpsilon));
  EXPECT_THAT(traj.GetFinish(),
              IsStateAndTimeApprox(
                  ToStateAndTime({0.5, 0.5, 0.0, 0.0, 1.0, 0.0}), kEpsilon));
  EXPECT_EQ(traj.GetSize(), 2);

  traj.TruncateOrExtendTo(Interval<double>(-1.0, 0.5));
  EXPECT_THAT(traj.GetStart(),
              IsStateAndTimeApprox(
                  ToStateAndTime({-1.0, -1.0, 0.0, 0.0, 1.0, 0.0}), kEpsilon));
  EXPECT_THAT(traj.GetFinish(),
              IsStateAndTimeApprox(
                  ToStateAndTime({0.5, 0.5, 0.0, 0.0, 1.0, 0.0}), kEpsilon));
  EXPECT_EQ(traj.GetSize(), 3);

  traj.TruncateOrExtendTo(Interval<double>(-1.0, 1.5));
  EXPECT_THAT(traj.GetStart(),
              IsStateAndTimeApprox(
                  ToStateAndTime({-1.0, -1.0, 0.0, 0.0, 1.0, 0.0}), kEpsilon));
  EXPECT_THAT(traj.GetFinish(),
              IsStateAndTimeApprox(
                  ToStateAndTime({1.5, 1.5, 0.0, 0.0, 1.0, 0.0}), kEpsilon));
  EXPECT_EQ(traj.GetSize(), 4);

  traj.TruncateOrExtendTo(Interval<double>());
  EXPECT_EQ(traj.GetSize(), 0);
}

}  // namespace
}  // namespace mobility::diff_drive
