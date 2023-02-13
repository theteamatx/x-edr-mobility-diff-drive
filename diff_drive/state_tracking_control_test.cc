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

#include "diff_drive/state_tracking_control.h"

#include <cmath>
#include <ostream>

#include "diff_drive/trajectory_limits.h"
#include "eigenmath/matchers.h"
#include "gtest/gtest.h"

namespace mobility::diff_drive {
namespace {

using eigenmath::testing::IsApprox;

constexpr double kCycleTime = 0.01;

struct TestLimitsFactory {
 public:
  Kinematics kinematics;
  DynamicLimits dd_limits;
  TrajectoryLimits traj_limits;

  TestLimitsFactory()
      : kinematics({0.075, 0.075}, 0.35),
        dd_limits(kinematics, WheelVector(27.0, 27.0),
                  WheelVector(500.0, 500.0), ArcVector(2.0, 1.8),
                  ArcVector(4.0, 7.2)),
        traj_limits(dd_limits, kCycleTime) {}
};

const TestLimitsFactory kTestLimits;

constexpr double kStateTrackingKr = 1.0;
constexpr double kStateTrackingKg = 2.0;
constexpr double kStateTrackingKv = 0.25;
constexpr double kStateTrackingKw = 0.5;
constexpr double kStateTrackingDamping = 0.7;
constexpr double kStateTrackingGain = 10.0;

constexpr double kGoalAngleTolerance = 0.05;

struct TestParameters {
  bool go_forward;
  double start_px;
  double start_py;
  double start_angle;
  double start_vtrans;
  double start_vrot;
  double tolerable_x_error;
  double tolerable_y_error;
};

const TestParameters kDriveForward = {true, 0.0, 0.0,  0.0,
                                      0.0,  0.0, 0.05, 0.05};
const TestParameters kDriveBackwards = {false, 0.0, 0.0, 0.0,
                                        0.0,   0.0, 0.1, 0.05};
const TestParameters kRealignForward = {true, 0.0, 0.0, M_PI * 0.95,
                                        0.0,  0.0, 0.1, 0.05};
const TestParameters kDriveForwardFromOffset = {true, 0.25, 1.0,  0.0,
                                                0.0,  0.0,  0.05, 0.06};
const TestParameters kDriveForwardFromOffsetMisaligned = {
    true, 0.25, 1.0, M_PI * 0.95, 0.0, 0.0, 0.1, 0.1};
const TestParameters kDriveForwardSlow = {true, 0.0, 0.0,  0.0,
                                          0.5,  0.0, 0.05, 0.05};
const TestParameters kDriveForwardFast = {true, 0.0, 0.0,  0.0,
                                          1.8,  0.0, 0.05, 0.05};
const TestParameters kDriveForwardTurn = {true, 0.0, 0.0,  0.0,
                                          1.0,  1.0, 0.05, 0.05};

class StateTrackingControlTestWithParam
    : public ::testing::TestWithParam<TestParameters> {
 public:
  StateTrackingControlTestWithParam()
      : state_controller_(kStateTrackingDamping, kStateTrackingGain,
                          kStateTrackingKr, kStateTrackingKg, kStateTrackingKv,
                          kStateTrackingKw) {}

 protected:
  StateTrackingControl state_controller_;
};

INSTANTIATE_TEST_SUITE_P(
    StateTrackingControl, StateTrackingControlTestWithParam,
    ::testing::Values(kDriveForward, kDriveBackwards, kRealignForward,
                      kDriveForwardFromOffset,
                      kDriveForwardFromOffsetMisaligned, kDriveForwardSlow,
                      kDriveForwardFast, kDriveForwardTurn));

TEST_P(StateTrackingControlTestWithParam, Drive) {
  TestParameters params = GetParam();

  const double desired_angle = (params.go_forward ? 0.0 : M_PI);
  const double desired_speed = (params.go_forward ? 1.0 : -1.0);
  Trajectory target_traj(16);
  target_traj.AddState(0.0,
                       State(eigenmath::Pose2d(eigenmath::Vector2d(0.0, 0.0),
                                               eigenmath::SO2d(desired_angle)),
                             ArcVector(desired_speed, 0.0)));
  target_traj.AddState(0.5,
                       State(eigenmath::Pose2d(eigenmath::Vector2d(0.5, 0.0),
                                               eigenmath::SO2d(desired_angle)),
                             ArcVector(desired_speed, 0.0)));
  target_traj.AddState(1.0,
                       State(eigenmath::Pose2d(eigenmath::Vector2d(1.0, 0.0),
                                               eigenmath::SO2d(desired_angle)),
                             ArcVector(desired_speed, 0.0)));
  target_traj.AddState(1.5,
                       State(eigenmath::Pose2d(eigenmath::Vector2d(1.5, 0.0),
                                               eigenmath::SO2d(desired_angle)),
                             ArcVector(desired_speed, 0.0)));
  target_traj.AddState(2.0,
                       State(eigenmath::Pose2d(eigenmath::Vector2d(2.0, 0.0),
                                               eigenmath::SO2d(desired_angle)),
                             ArcVector(desired_speed, 0.0)));
  target_traj.AddState(2.5,
                       State(eigenmath::Pose2d(eigenmath::Vector2d(2.5, 0.0),
                                               eigenmath::SO2d(desired_angle)),
                             ArcVector(desired_speed, 0.0)));
  target_traj.AddState(3.0,
                       State(eigenmath::Pose2d(eigenmath::Vector2d(3.0, 0.0),
                                               eigenmath::SO2d(desired_angle)),
                             ArcVector(desired_speed, 0.0)));
  target_traj.AddState(3.5,
                       State(eigenmath::Pose2d(eigenmath::Vector2d(3.5, 0.0),
                                               eigenmath::SO2d(desired_angle)),
                             ArcVector(desired_speed, 0.0)));
  target_traj.AddState(4.0,
                       State(eigenmath::Pose2d(eigenmath::Vector2d(4.0, 0.0),
                                               eigenmath::SO2d(desired_angle)),
                             ArcVector(desired_speed, 0.0)));

  const ArcVector start_arc_vel = {params.start_vtrans, params.start_vrot};
  StateAndTime current_state;
  current_state.time = 0.0;
  current_state.state = State(
      eigenmath::Pose2d(eigenmath::Vector2d(params.start_px, params.start_py),
                        params.start_angle),
      start_arc_vel);

  bool in_transient_phase = true;
  for (auto time_it = target_traj.BeginInTime();
       time_it < target_traj.EndInTime(); time_it += kCycleTime) {
    // Compute next control velocity from state feedback controller:
    auto target_state = time_it.GetState();
    eigenmath::Pose2d pose_diff =
        current_state.state.GetPose().inverse() * target_state.GetPose();
    if (in_transient_phase) {
      if ((time_it.GetTime() > 0.5) &&
          (std::abs(pose_diff.translation().x()) < params.tolerable_x_error) &&
          (std::abs(pose_diff.translation().y()) < params.tolerable_y_error)) {
        in_transient_phase = false;
      }
    } else {
      // After tracking tolerance has been reached, expect no more than 10%
      // deviation outside the tolerance.
      EXPECT_NEAR(pose_diff.translation().x(), 0.0,
                  1.1 * params.tolerable_x_error)
          << "At time: " << time_it.GetTime()
          << " tracking pose: " << target_state.GetPose()
          << ", currently at: " << current_state.state.GetPose() << std::endl;
      EXPECT_NEAR(pose_diff.translation().y(), 0.0,
                  1.1 * params.tolerable_y_error)
          << "At time: " << time_it.GetTime()
          << " tracking pose: " << target_state.GetPose()
          << ", currently at: " << current_state.state.GetPose() << std::endl;
    }

    ArcVector next_velocity;
    state_controller_.ComputeNextVelocity(
        params.tolerable_x_error, kTestLimits.dd_limits,
        kTestLimits.traj_limits, kCycleTime, current_state.state, target_state,
        &next_velocity);

    // Integrate in time, assuming perfect velocity control:
    current_state.state.SetArcVelocity(next_velocity);
    current_state.state =
        current_state.state.ExtrapolateConstantVelocityArc(kCycleTime);
    current_state.time += kCycleTime;
  }
  EXPECT_FALSE(in_transient_phase);
  auto target_state = target_traj.GetFinish().state;
  EXPECT_THAT(current_state.state.GetPose(),
              IsApprox(target_state.GetPose(), params.tolerable_x_error,
                       kGoalAngleTolerance));
}

TEST_F(StateTrackingControlTestWithParam, CurvyTrajectory) {
  Trajectory target_traj(16);

  target_traj.AddState(
      0.0, State(eigenmath::Pose2d(eigenmath::Vector2d(0.0, 0.0), 0.0),
                 ArcVector{1.0, 0.4 * M_PI}));

  target_traj.AddState(
      1.25, State(eigenmath::Pose2d(eigenmath::Vector2d(2.5 / M_PI, 2.5 / M_PI),
                                    0.5 * M_PI),
                  ArcVector{1.0, 0.0}));

  target_traj.AddState(
      2.25,
      State(eigenmath::Pose2d(eigenmath::Vector2d(2.5 / M_PI, 1.0 + 2.5 / M_PI),
                              0.5 * M_PI),
            ArcVector{1.0, -0.4 * M_PI}));

  target_traj.AddState(
      4.75,
      State(eigenmath::Pose2d(eigenmath::Vector2d(7.5 / M_PI, 1.0 + 2.5 / M_PI),
                              -0.5 * M_PI),
            ArcVector{1.0, 0.0}));

  target_traj.AddState(
      5.75, State(eigenmath::Pose2d(eigenmath::Vector2d(7.5 / M_PI, 2.5 / M_PI),
                                    -0.5 * M_PI),
                  ArcVector{1.0, 0.4 * M_PI}));

  target_traj.AddState(
      7.0, State(eigenmath::Pose2d(eigenmath::Vector2d(10.0 / M_PI, 0.0), 0.0),
                 ArcVector{1.0, 0.0}));

  constexpr double kTolerancePosError = 0.035;

  const ArcVector start_arc_vel = {0.0, 0.0};
  StateAndTime current_state;
  current_state.time = 0.0;
  current_state.state = State(
      eigenmath::Pose2d(eigenmath::Vector2d(0.0, 0.0), 0.0), start_arc_vel);

  bool in_transient_phase = true;
  for (auto time_it = target_traj.BeginInTime();
       time_it < target_traj.EndInTime(); time_it += kCycleTime) {
    // Compute next control velocity from state feedback controller:
    auto target_state = time_it.GetState();
    eigenmath::Pose2d pose_diff =
        current_state.state.GetPose().inverse() * target_state.GetPose();
    if (in_transient_phase) {
      if ((time_it.GetTime() > 0.5) &&
          (std::abs(pose_diff.translation().x()) < kTolerancePosError) &&
          (std::abs(pose_diff.translation().y()) < kTolerancePosError)) {
        in_transient_phase = false;
      }
    } else {
      // After tracking tolerance has been reached, expect no more than 10%
      // deviation outside the tolerance.
      EXPECT_NEAR(pose_diff.translation().x(), 0.0, 1.1 * kTolerancePosError)
          << "At time: " << time_it.GetTime()
          << " tracking pose: " << target_state.GetPose()
          << ", currently at: " << current_state.state.GetPose() << std::endl;
      EXPECT_NEAR(pose_diff.translation().y(), 0.0, 1.1 * kTolerancePosError)
          << "At time: " << time_it.GetTime()
          << " tracking pose: " << target_state.GetPose()
          << ", currently at: " << current_state.state.GetPose() << std::endl;
    }

    ArcVector next_velocity;
    state_controller_.ComputeNextVelocity(
        kTolerancePosError, kTestLimits.dd_limits, kTestLimits.traj_limits,
        kCycleTime, current_state.state, target_state, &next_velocity);

    // Integrate in time, assuming perfect velocity control:
    current_state.state.SetArcVelocity(next_velocity);
    current_state.state =
        current_state.state.ExtrapolateConstantVelocityArc(kCycleTime);
    current_state.time += kCycleTime;
  }
  EXPECT_FALSE(in_transient_phase);
  auto target_state = target_traj.GetFinish().state;
  EXPECT_THAT(current_state.state.GetPose(),
              IsApprox(target_state.GetPose(), kTolerancePosError,
                       kGoalAngleTolerance));
}

}  // namespace
}  // namespace mobility::diff_drive
