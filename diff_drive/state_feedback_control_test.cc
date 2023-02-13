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

#include "diff_drive/state_feedback_control.h"

#include <cmath>

#include "diff_drive/trajectory_limits.h"
#include "eigenmath/matchers.h"
#include "gtest/gtest.h"

namespace mobility::diff_drive {
namespace {

using eigenmath::testing::IsApprox;

constexpr double kCycleTime = 0.01;

class TestLimitsFactory {
 public:
  Kinematics kinematics;
  DynamicLimits dd_limits;

  TestLimitsFactory()
      : kinematics({2.0, 2.0}, 2.0),
        dd_limits(kinematics, WheelVector(1.0, 1.0), WheelVector(3.0, 3.0),
                  ArcVector(2.0, 2.0), ArcVector(6.0, 6.0)) {}
};

const TestLimitsFactory kTestLimits;

constexpr double kStateFeedbackKr = 3.0;
constexpr double kStateFeedbackKd = -1.5;
constexpr double kStateFeedbackKg = 8.0;
constexpr double kStateFeedbackKv = 3.0;
constexpr double kStateFeedbackKw = 8.0;

constexpr double kGoalDistanceTolerance = 0.05;
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

const TestParameters kDriveForward = {true, 0.0, 0.0, 0.0, 0.0, 0.0, 0.2, 0.1};
const TestParameters kDriveBackwards = {false, 0.0, 0.0, 0.0,
                                        0.0,   0.0, 0.2, 0.7};
const TestParameters kRealignForward = {true, 0.0, 0.0, M_PI * 0.95,
                                        0.0,  0.0, 0.2, 0.7};
const TestParameters kDriveForwardFromOffset = {true, 0.25, 1.0, 0.0,
                                                0.0,  0.0,  0.2, 1.1};
const TestParameters kDriveForwardFromOffsetMisaligned = {
    true, 0.25, 1.0, M_PI * 0.95, 0.0, 0.0, 0.3, 1.2};

class StateFeedbackControlTestWithParam
    : public ::testing::TestWithParam<TestParameters> {
 public:
  StateFeedbackControlTestWithParam()
      : state_controller_(kStateFeedbackKr, kStateFeedbackKd, kStateFeedbackKg,
                          kStateFeedbackKv, kStateFeedbackKw,
                          kGoalDistanceTolerance, kGoalAngleTolerance) {}

 protected:
  StateFeedbackControl state_controller_;
};

INSTANTIATE_TEST_SUITE_P(StateFeedbackControl,
                         StateFeedbackControlTestWithParam,
                         ::testing::Values(kDriveForward, kDriveBackwards,
                                           kRealignForward,
                                           kDriveForwardFromOffset,
                                           kDriveForwardFromOffsetMisaligned));

TEST_P(StateFeedbackControlTestWithParam, Drive) {
  TestParameters params = GetParam();

  const eigenmath::Vector2d goal_point(3.0, 0.0);
  const StateFeedbackControl::DirectionChoice desired_direction =
      (params.go_forward
           ? StateFeedbackControl::DirectionChoice::kForwardOnly
           : StateFeedbackControl::DirectionChoice::kBackwardOnly);
  const double final_angle = (params.go_forward ? 0.0 : M_PI);
  const double desired_speed = (params.go_forward ? 1.0 : -1.0);

  const eigenmath::Pose2d goal_pose(goal_point, eigenmath::SO2d(final_angle));

  const ArcVector start_arc_vel = {params.start_vtrans, params.start_vrot};
  StateAndTime current_state;
  current_state.time = 0.0;
  current_state.state = State(
      eigenmath::Pose2d(eigenmath::Vector2d(params.start_px, params.start_py),
                        params.start_angle),
      start_arc_vel);

  const double total_duration = 15.0;

  for (double time = 0.0; time < total_duration + kCycleTime * 0.5;
       time += kCycleTime) {
    // Compute next control velocity from state feedback controller:
    ArcVector next_velocity;
    StateFeedbackControl::State ctrl_result =
        state_controller_.ComputeNextVelocity(
            desired_direction, kTestLimits.dd_limits, kCycleTime, desired_speed,
            current_state.state.GetPose(), current_state.state.GetArcVelocity(),
            goal_pose, &next_velocity);

    if (ctrl_result == StateFeedbackControl::State::kCompleted) {
      break;
    }
    ASSERT_EQ(ctrl_result, StateFeedbackControl::State::kInProgress);

    // Integrate in time, assuming perfect velocity control:
    current_state.state.SetArcVelocity(next_velocity);
    current_state.state =
        current_state.state.ExtrapolateConstantVelocityArc(kCycleTime);
    current_state.time += kCycleTime;
    EXPECT_GE(current_state.state.GetPose().translation().x(),
              0.0 - params.tolerable_x_error);
    EXPECT_LE(current_state.state.GetPose().translation().x(),
              3.0 + params.tolerable_x_error);
    EXPECT_LE(std::abs(current_state.state.GetPose().translation().y()),
              params.tolerable_y_error);
  }

  EXPECT_LT(current_state.time, total_duration - kCycleTime * 0.5)
      << "State feedback controller did not reach goal in " << total_duration
      << " seconds!";
  EXPECT_THAT(current_state.state.GetPose(),
              IsApprox(eigenmath::Pose2d(goal_point, final_angle),
                       kGoalDistanceTolerance, kGoalAngleTolerance));
}

}  // namespace
}  // namespace mobility::diff_drive
