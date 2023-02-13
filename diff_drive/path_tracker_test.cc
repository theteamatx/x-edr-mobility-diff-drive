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

#include "diff_drive/path_tracker.h"

#include <cmath>

#include "eigenmath/matchers.h"
#include "gtest/gtest.h"

namespace mobility::diff_drive {
namespace {

using eigenmath::testing::IsApprox;

constexpr double kEpsilon = 1.0e-6;
constexpr double kCycleTime = 0.01;

constexpr double kTrackerKr = 1.0;
constexpr double kTrackerKg = 2.0;
constexpr double kTrackerKv = 0.5;
constexpr double kTrackerKw = 0.5;
constexpr double kTrackerDampingFactor = 0.7;
constexpr double kTrackerGain = 10.0;
constexpr int kMaxMumPathPoints = 1024;
constexpr double kAccelAggressivity = 0.5;
constexpr double kTrajResetThreshold = 0.5;
constexpr double kMaxDistanceToPath = 2.0;
constexpr double kGoalDistanceTolerance = 0.1;
constexpr double kGoalAngleTolerance = 0.05;

const Kinematics kTestKinematics({2.0, 2.0}, 2.0);
const DynamicLimits kTestDynamicLimits({{2.0, 2.0}, 2.0},
                                       /*max_wheel_velocity=*/{1.0, 1.0},
                                       /*max_wheel_acceleration=*/{3.0, 3.0},
                                       /*max_arc_velocity=*/{2.0, 2.0},
                                       /*max_arc_acceleration=*/{6.0, 6.0});

struct TestParameters {
  bool go_forward;
  double start_px;
  double start_py;
  double start_angle;
  double tolerable_error_linear;
  double tolerable_error_angular;
};

const TestParameters kDriveForward = {true, 0.0, 0.0, 0.0, 0.02, 0.0001};
const TestParameters kDriveBackwards = {false, 0.0, 0.0, 0.0, 0.5, 0.25};
const TestParameters kRealignForward = {true, 0.0, 0.0, M_PI * 0.95, 0.5, 0.25};
const TestParameters kDriveForwardFromOffset = {true, 0.25, 1.0,
                                                0.0,  0.05, 0.2};
const TestParameters kDriveForwardFromOffsetMisaligned = {
    true, 0.25, 1.0, M_PI * 0.95, 0.6, 0.5};

class PathTrackerTest : public ::testing::Test {
 public:
  PathTrackerTest()
      : path_tracker_(kMaxMumPathPoints, kCycleTime, kAccelAggressivity,
                      kTrajResetThreshold, kMaxDistanceToPath,
                      kGoalDistanceTolerance, kGoalAngleTolerance,
                      kTrackerDampingFactor, kTrackerGain, kTrackerKr,
                      kTrackerKg, kTrackerKv, kTrackerKw) {}

 protected:
  PathTracker path_tracker_;
};

TEST_F(PathTrackerTest, SetPath) {
  const eigenmath::Vector2d path_pts[] = {{0.0, 0.0}, {0.5, 0.0}, {1.0, 0.0},
                                          {1.5, 0.0}, {2.0, 0.0}, {2.5, 0.0},
                                          {3.0, 0.0}};
  const double final_angle = 0.0;
  const double desired_velocity = 0.5;

  const eigenmath::Pose2d identity;
  path_tracker_.Clear();
  bool set_path_result =
      path_tracker_.SetPath(kTestDynamicLimits, identity, path_pts, final_angle,
                            desired_velocity, 0.0, identity);

  EXPECT_TRUE(set_path_result);

  auto &resulting_path = path_tracker_.GetCurve();

  EXPECT_GE(resulting_path.GetSize(), 7);

  for (auto &pt : path_pts) {
    auto curve_pt = resulting_path.Evaluate(pt.x());
    EXPECT_THAT(curve_pt.GetPose(),
                IsApprox(eigenmath::Pose2d(pt, 0.0), kEpsilon, kEpsilon));
    EXPECT_NEAR(0.0, curve_pt.GetCurvature(), kEpsilon);
  }
}

TEST_F(PathTrackerTest, EmptyPath) {
  const eigenmath::Vector2d path_pts[] = {{0.0, 0.0}};
  const double final_angle = 0.0;
  const double desired_velocity = 0.5;

  const eigenmath::Pose2d identity;
  path_tracker_.Clear();
  bool set_path_result =
      path_tracker_.SetPath(kTestDynamicLimits, identity, {path_pts, 0},
                            final_angle, desired_velocity, 0.0, identity);

  EXPECT_FALSE(set_path_result);

  const WheelVector start_wheel_vel = {0.0, 0.0};
  const ArcVector start_arc_vel =
      kTestKinematics.ComputeForwardKinematics(start_wheel_vel);
  StateAndTime current_state;
  current_state.time = 0.0;
  current_state.state = State(
      eigenmath::Pose2d(eigenmath::Vector2d(0.0, 0.0), 0.0), start_arc_vel);

  // Compute next control velocity from path-follower:
  PathTracker::Result ctrl_result = path_tracker_.ComputeNextVelocity(
      kTestDynamicLimits, current_state.time, current_state.state);
  ASSERT_NE(ctrl_result.status, PathTracker::Status::kFollowingPath);
  ASSERT_NE(ctrl_result.status, PathTracker::Status::kReachedGoal);
  EXPECT_EQ(ctrl_result.status, PathTracker::Status::kNoPath);

  EXPECT_NEAR(ctrl_result.velocity_command.Translation(), 0.0, kEpsilon);
  EXPECT_NEAR(ctrl_result.velocity_command.Rotation(), 0.0, kEpsilon);
}

TEST_F(PathTrackerTest, SetPathRepeatedPoints) {
  const eigenmath::Vector2d path_pts[] = {{0.0, 0.0}, {0.5, 0.0}, {0.5, 0.0},
                                          {1.0, 0.0}, {1.5, 0.0}, {1.5, 0.0},
                                          {2.0, 0.0}, {2.5, 0.0}, {3.0, 0.0}};
  const double final_angle = 0.0;
  const double desired_velocity = 0.5;

  const eigenmath::Pose2d identity;
  path_tracker_.Clear();
  bool set_path_result =
      path_tracker_.SetPath(kTestDynamicLimits, identity, path_pts, final_angle,
                            desired_velocity, 0.0, identity);

  EXPECT_TRUE(set_path_result);

  auto &resulting_path = path_tracker_.GetCurve();

  EXPECT_GE(resulting_path.GetSize(), 7);

  for (auto &pt : path_pts) {
    auto curve_pt = resulting_path.Evaluate(pt.x());
    EXPECT_THAT(curve_pt.GetPose(),
                IsApprox(eigenmath::Pose2d(pt, 0.0), kEpsilon, kEpsilon));
    EXPECT_NEAR(0.0, curve_pt.GetCurvature(), kEpsilon);
  }
}

TEST_F(PathTrackerTest, SetPathTooFarAway) {
  const eigenmath::Vector2d path_pts[] = {{3.0, 0.0}, {3.5, 0.0}, {4.0, 0.0},
                                          {4.5, 0.0}, {5.0, 0.0}, {5.5, 0.0},
                                          {6.0, 0.0}};
  const double final_angle = 0.0;
  const double desired_velocity = 0.5;

  const eigenmath::Pose2d identity;
  path_tracker_.Clear();
  bool set_path_result =
      path_tracker_.SetPath(kTestDynamicLimits, identity, path_pts, final_angle,
                            desired_velocity, 0.0, identity);

  EXPECT_FALSE(set_path_result);
}

TEST_F(PathTrackerTest, TransformPath) {
  const eigenmath::Vector2d path_pts[] = {{0.0, 0.0}, {0.5, 0.0}, {1.0, 0.0},
                                          {1.5, 0.0}, {2.0, 0.0}, {2.5, 0.0},
                                          {3.0, 0.0}};
  const eigenmath::Vector2d transformed_path_pts[] = {
      {-3.0, 1.0},  {-3.0, 0.5},  {-3.0, 0.0}, {-3.0, -0.5},
      {-3.0, -1.0}, {-3.0, -1.5}, {-3.0, -2.0}};
  const double transformed_final_angle = -M_PI * 0.5;
  const double desired_velocity = 0.5;

  const eigenmath::Pose2d ref_pose_input(eigenmath::Vector2d(1.0, 3.0),
                                         M_PI * 0.5);
  path_tracker_.Clear();
  bool set_path_result = path_tracker_.SetPath(
      kTestDynamicLimits, ref_pose_input, transformed_path_pts,
      transformed_final_angle, desired_velocity, 0.0, eigenmath::Pose2d());

  EXPECT_TRUE(set_path_result);

  auto &resulting_path = path_tracker_.GetCurve();

  EXPECT_GE(resulting_path.GetSize(), 7);

  for (auto &pt : path_pts) {
    auto curve_pt = resulting_path.Evaluate(pt.x());
    EXPECT_THAT(curve_pt.GetPose(),
                IsApprox(eigenmath::Pose2d(pt, 0.0), kEpsilon, kEpsilon));
    EXPECT_NEAR(0.0, curve_pt.GetCurvature(), kEpsilon);
  }
}

TEST_F(PathTrackerTest, AlreadyAtGoal) {
  const eigenmath::Vector2d path_pts[] = {{0.0, 0.0}, {0.5, 0.0}, {1.0, 0.0},
                                          {1.5, 0.0}, {2.0, 0.0}, {2.5, 0.0},
                                          {3.0, 0.0}};
  const double final_angle = 0.0;
  const double desired_velocity = 0.5;
  const eigenmath::Pose2d goal_pose(eigenmath::Vector2d(3.0, 0.0), 0.0);

  const ArcVector start_arc_vel = {0.0, 0.0};
  StateAndTime current_state;
  current_state.time = 0.0;
  current_state.state = State(
      eigenmath::Pose2d(eigenmath::Vector2d(3.0, 0.0), 0.0), start_arc_vel);

  const eigenmath::Pose2d identity;
  path_tracker_.Clear();
  bool set_path_result = path_tracker_.SetPath(
      kTestDynamicLimits, identity, path_pts, final_angle, desired_velocity,
      current_state.time, current_state.state.GetPose());
  ASSERT_TRUE(set_path_result);

  // Compute next control velocity from path-follower:
  PathTracker::Result ctrl_result = path_tracker_.ComputeNextVelocity(
      kTestDynamicLimits, current_state.time, current_state.state);
  ASSERT_NE(ctrl_result.status, PathTracker::Status::kNoPath);
  ASSERT_NE(ctrl_result.status, PathTracker::Status::kFollowingPath);
  EXPECT_EQ(ctrl_result.status, PathTracker::Status::kReachedGoal);

  EXPECT_NEAR(ctrl_result.velocity_command.Translation(), 0.0, kEpsilon);
  EXPECT_NEAR(ctrl_result.velocity_command.Rotation(), 0.0, kEpsilon);
}

class PathTrackerTestWithParam
    : public ::testing::TestWithParam<TestParameters> {
 public:
  PathTrackerTestWithParam()
      : path_tracker_(kMaxMumPathPoints, kCycleTime, kAccelAggressivity,
                      kTrajResetThreshold, kMaxDistanceToPath,
                      kGoalDistanceTolerance, kGoalAngleTolerance,
                      kTrackerDampingFactor, kTrackerGain, kTrackerKr,
                      kTrackerKg, kTrackerKv, kTrackerKw) {}

 protected:
  PathTracker path_tracker_;
};

INSTANTIATE_TEST_SUITE_P(PathTracker, PathTrackerTestWithParam,
                         ::testing::Values(kDriveForward, kDriveBackwards,
                                           kRealignForward,
                                           kDriveForwardFromOffset,
                                           kDriveForwardFromOffsetMisaligned));

TEST_P(PathTrackerTestWithParam, Drive) {
  const TestParameters params = GetParam();

  const eigenmath::Vector2d path_pts[] = {{0.0, 0.0}, {0.5, 0.0}, {1.0, 0.0},
                                          {1.5, 0.0}, {2.0, 0.0}, {2.5, 0.0},
                                          {3.0, 0.0}};
  const double final_angle = 0.0;
  const double desired_velocity = (params.go_forward ? 0.5 : -0.5);
  const eigenmath::Pose2d goal_pose(eigenmath::Vector2d(3.0, 0.0), final_angle);

  const ArcVector start_arc_vel = {0.0, 0.0};
  StateAndTime current_state;
  current_state.time = 0.0;
  current_state.state = State(
      eigenmath::Pose2d(eigenmath::Vector2d(params.start_px, params.start_py),
                        params.start_angle),
      start_arc_vel);

  const eigenmath::Pose2d identity;
  path_tracker_.Clear();
  bool set_path_result = path_tracker_.SetPath(
      kTestDynamicLimits, identity, path_pts, final_angle, desired_velocity,
      current_state.time, current_state.state.GetPose());
  ASSERT_TRUE(set_path_result);

  const double total_duration = 20.0;
  const double transient_duration = 5.0;

  bool passed_linear_error = false;
  bool passed_angular_error = false;

  for (double time = 0.0; time < total_duration + kCycleTime * 0.5;
       time += kCycleTime) {
    // Compute next control velocity from path-follower:
    PathTracker::Result ctrl_result = path_tracker_.ComputeNextVelocity(
        kTestDynamicLimits, current_state.time, current_state.state);
    ASSERT_NE(ctrl_result.status, PathTracker::Status::kNoPath);

    if (ctrl_result.status == PathTracker::Status::kReachedGoal) {
      break;
    }

    // Integrate in time, assuming perfect velocity control:
    EXPECT_EQ(ctrl_result.status, PathTracker::Status::kFollowingPath);
    current_state.state.SetArcVelocity(ctrl_result.velocity_command);
    current_state.state =
        current_state.state.ExtrapolateConstantVelocityArc(kCycleTime);
    current_state.time += kCycleTime;

    double error_linear, error_angular, cord_length;
    ASSERT_TRUE(path_tracker_.GetDeviationForPose(
        current_state.time, current_state.state.GetPose(), &error_linear,
        &error_angular, &cord_length));
    if (passed_linear_error ||
        (std::abs(error_linear) < params.tolerable_error_linear)) {
      EXPECT_LE(std::abs(error_linear), params.tolerable_error_linear);
      passed_linear_error = (current_state.time > transient_duration);
    }
    if (passed_angular_error ||
        (std::abs(error_angular) < params.tolerable_error_angular)) {
      EXPECT_LE(std::abs(error_angular), params.tolerable_error_angular);
      passed_angular_error = (current_state.time > transient_duration);
    }
  }

  EXPECT_TRUE(passed_linear_error);
  EXPECT_TRUE(passed_angular_error);

  EXPECT_LT(current_state.time, total_duration - kCycleTime * 0.5)
      << "Path follower did not reach goal in " << total_duration
      << " seconds!";
  EXPECT_THAT(current_state.state.GetPose(),
              IsApprox(eigenmath::Pose2d(
                           goal_pose.translation(),
                           (params.go_forward ? goal_pose.angle()
                                              : goal_pose.angle() + M_PI)),
                       kGoalDistanceTolerance, kGoalAngleTolerance));
}

// This is a regression test for b/38030254
TEST(PathTrackerRegression, Bug38030254) {
  const DynamicLimits dv_limits(
      Kinematics(WheelVector(0.0762, 0.0762), 0.31), WheelVector(8.0, 8.0),
      WheelVector(120.0, 120.0), ArcVector(0.3, 0.3), ArcVector(2.0, 2.0));
  constexpr double kDvMediumDesiredSpeed = 0.3;

  constexpr int kDvMaxNumPathPoints = 512;
  constexpr double kDvCycleTime = 0.004;
  constexpr double kDvAccelAggressivity = 0.5;
  constexpr double kDvTrajResetThreshold = 0.5;
  constexpr double kDvMaxDistanceToPath = 2.0;
  constexpr double kDvGoalDistanceTolerance = 0.02;
  constexpr double kDvGoalAngleTolerance = 0.02;
  constexpr double kDvDampingFactor = 0.75;
  constexpr double kDvGain = 4.0;
  constexpr double kDvKr = 0.30;
  constexpr double kDvKg = 1.25;
  constexpr double kDvKv = 0.25;
  constexpr double kDvKw = 1.0;

  PathTracker dv_path_tracker(kDvMaxNumPathPoints, kDvCycleTime,
                              kDvAccelAggressivity, kDvTrajResetThreshold,
                              kDvMaxDistanceToPath, kDvGoalDistanceTolerance,
                              kDvGoalAngleTolerance, kDvDampingFactor, kDvGain,
                              kDvKr, kDvKg, kDvKv, kDvKw);

  {
    const eigenmath::Vector2d polyline_pts[] = {
        {-53.951763779458048, -54.002134314324557},
        {-54.2, -54.2},
        {-54.400000000000006, -54.400000000000006},
        {-54.6, -54.6},
        {-54.6, -54.800000000000004},
        {-54.6, -55},
        {-54.6, -55.2},
        {-54.400000000000006, -55.400000000000006},
        {-54.400000000000006, -55.6},
        {-54.2, -55.800000000000004},
        {-54.2, -56},
        {-54, -56.2},
        {-53.800000000000004, -56.400000000000006},
        {-53.6, -56.400000000000006},
        {-53.400000000000006, -56.6},
        {-53.2, -56.6},
        {-53, -56.6},
        {-52.800000000000004, -56.800000000000004},
        {-52.6, -56.800000000000004},
        {-52.400000000000006, -56.800000000000004},
        {-52.2, -56.800000000000004},
        {-52, -56.800000000000004},
        {-51.800000000000004, -56.800000000000004},
        {-51.6, -56.6},
        {-51.400000000000006, -56.400000000000006},
        {-51.2, -56.2},
        {-51, -56},
        {-50.800000000000004, -55.800000000000004},
        {-50.6, -55.6},
        {-50.400000000000006, -55.400000000000006},
        {-50.400000000000006, -55.2},
        {-50.400000000000006, -55},
        {-50.400000000000006, -54.800000000000004},
        {-50.400000000000006, -54.6},
        {-50.400000000000006, -54.4},
        {-50.400000000000006, -54.2},
        {-50.400000000000006, -54},
        {-50.400000000000006, -53.800000000000004},
        {-50.400000000000006, -53.6},
        {-50.400000000000006, -53.400000000000006},
        {-50.400000000000006, -53.2},
        {-50.2, -53},
        {-50, -52.800000000000004},
        {-49.800000000000004, -52.6},
        {-49.580238898013221, -52.680000095120448},
        {-49.58, -52.98}};
    double final_angle = -1.5699999999999998;

    dv_path_tracker.Clear();
    EXPECT_TRUE(dv_path_tracker.SetPath(
        dv_limits, eigenmath::Pose2d(), polyline_pts, final_angle,
        kDvMediumDesiredSpeed, 0.0, eigenmath::Pose2d(polyline_pts[0], 0.0)))
        << " Got error string: " << dv_path_tracker.GetErrorString();
  }

  {
    const eigenmath::Vector2d polyline_pts[] = {
        {-58.124807300589175, -54.3094663088909},
        {-58.2, -54.2},
        {-58.2, -54},
        {-58, -53.800000000000004},
        {-57.800000000000004, -53.6},
        {-57.6, -53.400000000000006},
        {-57.6, -53.2},
        {-57.6, -53},
        {-57.400000000000006, -52.800000000000004},
        {-57.2, -52.6},
        {-57, -52.400000000000006},
        {-56.800000000000004, -52.2},
        {-56.6, -52.2},
        {-56.400000000000006, -52},
        {-56.2, -51.800000000000004},
        {-56, -51.6},
        {-55.800000000000004, -51.400000000000006},
        {-55.6, -51.400000000000006},
        {-55.400000000000006, -51.400000000000006},
        {-55.2, -51.400000000000006},
        {-55, -51.400000000000006},
        {-54.860238898013222, -51.39999990487955},
        {-54.86, -51.1}};
    double final_angle = 1.5699999999999998;

    dv_path_tracker.Clear();
    EXPECT_TRUE(dv_path_tracker.SetPath(
        dv_limits, eigenmath::Pose2d(), polyline_pts, final_angle,
        kDvMediumDesiredSpeed, 0.0, eigenmath::Pose2d(polyline_pts[0], 0.0)))
        << " Got error string: " << dv_path_tracker.GetErrorString();
  }
}

// This is a regression test for a bug found during testing of local
// planner based on trajectory-rollout
TEST(PathTrackerRegression, BugFromTrajRollout) {
  const DynamicLimits dv_limits(
      Kinematics(WheelVector(0.0762, 0.0762), 0.31), WheelVector(8.0, 8.0),
      WheelVector(120.0, 120.0), ArcVector(0.3, 0.3), ArcVector(2.0, 2.0));
  constexpr double kDvMediumDesiredSpeed = 0.3;

  constexpr int kDvMaxNumPathPoints = 128;
  constexpr double kDvCycleTime = 0.004;
  constexpr double kDvAccelAggressivity = 0.5;
  constexpr double kDvTrajResetThreshold = 0.5;
  constexpr double kDvMaxDistanceToPath = 2.0;
  constexpr double kDvGoalDistanceTolerance = 0.05;
  constexpr double kDvGoalAngleTolerance = 0.05;
  constexpr double kDvDampingFactor = 0.75;
  constexpr double kDvGain = 4.0;
  constexpr double kDvKr = 0.30;
  constexpr double kDvKg = 1.25;
  constexpr double kDvKv = 0.25;
  constexpr double kDvKw = 1.0;

  PathTracker dv_path_tracker(kDvMaxNumPathPoints, kDvCycleTime,
                              kDvAccelAggressivity, kDvTrajResetThreshold,
                              kDvMaxDistanceToPath, kDvGoalDistanceTolerance,
                              kDvGoalAngleTolerance, kDvDampingFactor, kDvGain,
                              kDvKr, kDvKg, kDvKv, kDvKw);

  {
    const eigenmath::Vector2d polyline_pts[] = {
        {-0.4813234809041807, -13.00054775501466},
        {-0.50381754637495868, -13.045202169798003},
        {-0.52623469659587285, -13.089895221841104},
        {-0.54865292779030983, -13.134587755060087},
        {-0.57102389690973743, -13.179303950997244},
        {-0.59332369531124407, -13.224055671917792},
        {-0.61566417771333237, -13.268787119144173},
        {-0.63788871913975165, -13.313576353984201},
        {-0.66007555326675682, -13.358384187289541},
        {-0.68230312240850222, -13.403171849597072},
        {-0.70441460516506493, -13.448017006854865},
        {-0.72648607676197918, -13.492881774292449},
        {-0.74855779190272909, -13.537746438078468},
        {-0.77057437495911418, -13.582638169563831},
        {-0.79253153090668038, -13.627558969866364},
        {-0.81448228087955976, -13.672482960641414},
        {-0.83632010469543283, -13.71746196779856},
        {-0.85788932455707034, -13.762570418501221},
        {-0.8793682043802038, -13.807721902473027},
        {-0.900794109059303, -13.852898552670661},
        {-0.9220987023343673, -13.898132579943741},
        {-0.9433445780062153, -13.943394150913235},
        {-0.96459549565714175, -13.988653375508436},
        {-0.98572149266033049, -14.033971114050615},
        {-1.0067942774735763, -14.079313531056165},
        {-1.0278721131941435, -14.12465362096898},
        {-1.0488248056082439, -14.170051747103836},
        {-1.0697241919291403, -14.215474348208355},
        {-1.0906297126690967, -14.260894146243226},
        {-1.1114143559641994, -14.306369451044482},
        {-1.1321469624825053, -14.351868459440885},
        {-1.1527742246179427, -14.397415325606797},
        {-1.1733470666224985, -14.442986757354166},
        {-1.193805974650431, -14.488609452711412},
        {-1.2142428791601216, -14.534242037517185},
        {-1.2347051190263032, -14.579863260144766},
        {-1.2551338017768878, -14.625499523826171},
        {-1.2755338875129065, -14.671148557782837},
        {-1.2959855224875168, -14.716774534666341},
        {-1.31634498278139, -14.762441771220207},
        {-1.3366879997273855, -14.808116265861621},
        {-1.3570825946887926, -14.853767767874901},
        {-1.3773849634744884, -14.899460414325006},
        {-1.3976708798538811, -14.945160298305844},
        {-1.4180084029449385, -14.990837254139493},
        {-1.4382565577099551, -15.036553946019135},
        {-1.4585009458541782, -15.082272260465363},
        {-1.4787342280452078, -15.127995477803481},
        {-1.4989148107258874, -15.17374199105365},
        {-1.5190816580382729, -15.219494541397092},
        {-1.5392493993922938, -15.265246722422022},
        {-1.5594521857298473, -15.310983440749119},
        {-1.5796782251230608, -15.356709879261473},
        {-1.5999053180147575, -15.40243584522846},
        {-1.6201751408235887, -15.448142890865805},
        {-1.6404212095048438, -15.493860447607952},
        {-1.660608949654661, -15.539603787055386},
        {-1.6808289829042664, -15.585332893122143},
        {-1.701020677643807, -15.631074490766549},
        {-1.7211540132366847, -15.676841801743539},
        {-1.7413196588304656, -15.722594917747433},
        {-1.7614586382630146, -15.768359747731703},
        {-1.7815583144189537, -15.814141871903278},
        {-1.8016460812029864, -15.859929210295645},
        {-1.8217213190755177, -15.905722033295321},
        {-1.8417289944224962, -15.951544435901779},
        {-1.8617532736708959, -15.997359588046923},
        {-1.8664131446498502, -16.008052805966578}};
    double final_angle = -1.9817606502583738;

    dv_path_tracker.Clear();
    EXPECT_TRUE(dv_path_tracker.SetPath(
        dv_limits, eigenmath::Pose2d(), polyline_pts, final_angle,
        kDvMediumDesiredSpeed, 0.0, eigenmath::Pose2d(polyline_pts[0], 0.0)))
        << " Got error string: " << dv_path_tracker.GetErrorString();
  }

  {
    const eigenmath::Vector2d polyline_pts[] = {
        {-5.377647276410598, -16.676673087663413},
        {-5.3277705586591368, -16.680182010941234},
        {-5.2778885337791985, -16.683614665775131},
        {-5.2280010310855021, -16.68696674858457},
        {-5.178108331283422, -16.690240596239992},
        {-5.1282097826658992, -16.693424026048156},
        {-5.0783071877615633, -16.696543071345307},
        {-5.0284061081825788, -16.699686503858675},
        {-4.978497734196659, -16.702712825003825},
        {-4.92858711319763, -16.705700689331955},
        {-4.8786779434163234, -16.708712944775122},
        {-4.8287618671146095, -16.711609425965356},
        {-4.7788423228923156, -16.714443930107329},
        {-4.7289229355256994, -16.71728159465253},
        {-4.6789939095947979, -16.719941608249357},
        {-4.6290544922830508, -16.722400600405958},
        {-4.5790988200412857, -16.724501865158114},
        {-4.5291232862191029, -16.726062589848961},
        {-4.4791486647213947, -16.727654985856191},
        {-4.4291787073615465, -16.729387485157041},
        {-4.3792152079927433, -16.731296655371317},
        {-4.3292582520600238, -16.733369706449352},
        {-4.2793078055873242, -16.735594658208718},
        {-4.2293672649120779, -16.73803124137066},
        {-4.1794322022152821, -16.740577400193033},
        {-4.1295083173472911, -16.74333456629352},
        {-4.0796034508620194, -16.746415598663958},
        {-4.0297148658520463, -16.749748648779462},
        {-3.9798521884010993, -16.753450306644535},
        {-3.9300332674690597, -16.757701672976129},
        {-3.8802665354947945, -16.76252422290543},
        {-3.8305979791791187, -16.768263739100824},
        {-3.78106862697338, -16.775097360768815},
        {-3.7317446892900845, -16.78327994036075},
        {-3.6826283656590348, -16.792634930642997},
        {-3.6336708705729821, -16.802789500393921},
        {-3.5849055340375608, -16.813831123092939},
        {-3.5363336766055755, -16.825692987459558},
        {-3.4879545441304676, -16.838318558401635},
        {-3.4398152186281479, -16.851830805417663},
        {-3.3919109184935592, -16.866152369203107},
        {-3.3442380096615802, -16.881226826545085},
        {-3.29684977922273, -16.897174587990332},
        {-3.2497375640517716, -16.913918787988734},
        {-3.2028973498274294, -16.931409880088612},
        {-3.1564078160170976, -16.949813851170248},
        {-3.1102824277071353, -16.969111494325372},
        {-3.0645869931668042, -16.989405376023019},
        {-3.0193784138780053, -17.0107607898616},
        {-2.9747380067839044, -17.033279375305121},
        {-2.9305718810685346, -17.056716921691468},
        {-2.88634493760271, -17.080039709856678},
        {-2.8419494903986844, -17.103039971016667},
        {-2.797347905229532, -17.12563803851917},
        {-2.7525320201710843, -17.147807998586003},
        {-2.707577966216812, -17.169697033580125},
        {-2.66254463969231, -17.191422583106217},
        {-2.6174614651218064, -17.213044408001611},
        {-2.57226894350546, -17.234436694843989},
        {-2.5430993082637681, -17.248074226438387}};
    double final_angle = -0.43733170400527605;

    dv_path_tracker.Clear();
    EXPECT_TRUE(dv_path_tracker.SetPath(
        dv_limits, eigenmath::Pose2d(), polyline_pts, final_angle,
        kDvMediumDesiredSpeed, 0.0, eigenmath::Pose2d(polyline_pts[0], 0.0)))
        << " Got error string: " << dv_path_tracker.GetErrorString();
  }

  {
    const eigenmath::Vector2d polyline_pts[] = {
        {-1.2371673797079656, -10.531003063169937},
        {-1.2585108543307288, -10.576218693887393},
        {-1.279670571037822, -10.621520577924947},
        {-1.3007290956779656, -10.666869597421895},
        {-1.3216295694905007, -10.71229170363161},
        {-1.3423489651873948, -10.757796658861039},
        {-1.3630008869190915, -10.803332318385584},
        {-1.3834257155776819, -10.848970423676256},
        {-1.4037028416380617, -10.894674170428663},
        {-1.4239121991438162, -10.940407965687225},
        {-1.4438934807627102, -10.98624199960441},
        {-1.4637196561151982, -11.032143151507691},
        {-1.4834287413293221, -11.078094669841496},
        {-1.5029527066359543, -11.124125143070366},
        {-1.5222721134272066, -11.170241768706477},
        {-1.5414208790052304, -11.216429576431155},
        {-1.5602833817291648, -11.26273501020248},
        {-1.5789591192121506, -11.309116210071753},
        {-1.5976543012406739, -11.355489585101969},
        {-1.6164542984346957, -11.401820518979507},
        {-1.6352331654986376, -11.448160082960664},
        {-1.654053338320111, -11.494482857625171},
        {-1.6729910053524562, -11.540757727876146},
        {-1.691890582992714, -11.58704819129129},
        {-1.71083142249238, -11.633321757792011},
        {-1.7298896309510539, -11.679547113817879},
        {-1.7489097907873195, -11.725788162226973},
        {-1.7679704220375909, -11.772012513436314},
        {-1.7871142409865441, -11.8182025132433},
        {-1.8063511432233896, -11.864353705819227},
        {-1.8256894278207514, -11.910462453791693},
        {-1.8452473710010273, -11.956478263725653},
        {-1.86551326072011, -12.00218649533187},
        {-1.8866484118486242, -12.047498786761849},
        {-1.9082738833351025, -12.092580249331634},
        {-1.9300307576148663, -12.137598450488452},
        {-1.9519056363096576, -12.182559389715058},
        {-1.9737955150840065, -12.227513055274489},
        {-1.9958340335865599, -12.272394119535859},
        {-2.0179487184579035, -12.317237593601368},
        {-2.0400791037747554, -12.362073348147407},
        {-2.0623577377555495, -12.406835706161369},
        {-2.0847123356382942, -12.451560066489083},
        {-2.1070825923871275, -12.496276623299467},
        {-2.1296000387765583, -12.540919324662223},
        {-2.1522140052379894, -12.58551307283888},
        {-2.1748797890394767, -12.630080481412906},
        {-2.1976195531316529, -12.674610266645427},
        {-2.2204974616051549, -12.719069221868201},
        {-2.2434939579238682, -12.763466951169752},
        {-2.2668106057673461, -12.807697343148934},
        {-2.2902010011374356, -12.851888850100936},
        {-2.3135514754508986, -12.89610146265386},
        {-2.3368113777155131, -12.940361760796479},
        {-2.3600098044609403, -12.984654296235767},
        {-2.3830836172324656, -13.02901190107278},
        {-2.4059989488013458, -13.073451556454817},
        {-2.4288443217437066, -13.11792722516072},
        {-2.4515521712660768, -13.16247328222255},
        {-2.47402105916403, -13.207140272843422},
        {-2.4962959519681438, -13.251904229831169},
        {-2.5182445258052151, -13.296829179830844},
        {-2.5397315852192275, -13.341976700177701},
        {-2.5606618471818723, -13.387384931576792},
        {-2.5809053325927578, -13.433103117424912},
        {-2.6002212400776519, -13.479219539976217},
        {-2.6177007480964702, -13.526059864303013},
        {-2.6191104538480827, -13.530193076828448}};
    double final_angle = -1.8994916855375521;

    dv_path_tracker.Clear();
    EXPECT_TRUE(dv_path_tracker.SetPath(
        dv_limits, eigenmath::Pose2d(), polyline_pts, final_angle,
        kDvMediumDesiredSpeed, 0.0, eigenmath::Pose2d(polyline_pts[0], 0.0)))
        << " Got error string: " << dv_path_tracker.GetErrorString();
  }
}

// This is a regression test for a bug found during testing of local
// planner based on trajectory-rollout
TEST(PathTrackerRegression, BugFromTrajRolloutRelaxed) {
  const DynamicLimits dv_limits(
      Kinematics(WheelVector(0.0762, 0.0762), 0.31), WheelVector(8.0, 8.0),
      WheelVector(120.0, 120.0), ArcVector(0.3, 0.3), ArcVector(2.0, 2.0));
  constexpr double kDvMediumDesiredSpeed = 0.3;

  constexpr int kDvMaxNumPathPoints = 128;
  constexpr double kDvCycleTime = 0.004;
  constexpr double kDvAccelAggressivity = 0.2;  // << relaxed acceleration
  constexpr double kDvTrajResetThreshold = 0.5;
  constexpr double kDvMaxDistanceToPath = 2.0;
  constexpr double kDvGoalDistanceTolerance = 0.05;
  constexpr double kDvGoalAngleTolerance = 0.05;
  constexpr double kDvDampingFactor = 0.75;
  constexpr double kDvGain = 4.0;
  constexpr double kDvKr = 0.30;
  constexpr double kDvKg = 1.25;
  constexpr double kDvKv = 0.25;
  constexpr double kDvKw = 1.0;

  PathTracker dv_path_tracker(kDvMaxNumPathPoints, kDvCycleTime,
                              kDvAccelAggressivity, kDvTrajResetThreshold,
                              kDvMaxDistanceToPath, kDvGoalDistanceTolerance,
                              kDvGoalAngleTolerance, kDvDampingFactor, kDvGain,
                              kDvKr, kDvKg, kDvKv, kDvKw);

  {
    const eigenmath::Vector2d polyline_pts[] = {
        {-2.39796011861796, -17.642948172376556},
        {-2.3540814069339677, -17.666920211525387}};
    double final_angle = -0.500017370952205;

    dv_path_tracker.Clear();
    EXPECT_TRUE(dv_path_tracker.SetPath(
        dv_limits, eigenmath::Pose2d(), polyline_pts, final_angle,
        kDvMediumDesiredSpeed, 0.0, eigenmath::Pose2d(polyline_pts[0], 0.0)))
        << " Got error string: " << dv_path_tracker.GetErrorString();
  }
}

// This is a regression test for a bug found during testing of local
// planner based on trajectory-rollout
TEST(PathTrackerRegression, Bug62917511) {
  const DynamicLimits dv_limits(
      Kinematics(WheelVector(0.0762, 0.0762), 0.31), WheelVector(8.0, 8.0),
      WheelVector(120.0, 120.0), ArcVector(0.3, 0.3), ArcVector(2.0, 2.0));
  constexpr double kDvMediumDesiredSpeed = 0.3;

  constexpr int kDvMaxNumPathPoints = 128;
  constexpr double kDvCycleTime = 0.004;
  constexpr double kDvAccelAggressivity = 0.1;
  constexpr double kDvTrajResetThreshold = 0.5;
  constexpr double kDvMaxDistanceToPath = 2.0;
  constexpr double kDvGoalDistanceTolerance = 0.1;
  constexpr double kDvGoalAngleTolerance = 6.2831853071795862;
  constexpr double kDvDampingFactor = 0.75;
  constexpr double kDvGain = 4.0;
  constexpr double kDvKr = 0.30;
  constexpr double kDvKg = 1.25;
  constexpr double kDvKv = 0.25;
  constexpr double kDvKw = 1.0;

  PathTracker dv_path_tracker(kDvMaxNumPathPoints, kDvCycleTime,
                              kDvAccelAggressivity, kDvTrajResetThreshold,
                              kDvMaxDistanceToPath, kDvGoalDistanceTolerance,
                              kDvGoalAngleTolerance, kDvDampingFactor, kDvGain,
                              kDvKr, kDvKg, kDvKv, kDvKw);

  {
    const eigenmath::Vector2d polyline_pts[] = {
        {-3.8103589320074938, 6.9802682867449022},
        {-3.8363809488263487, 7.0229631163287518},
        {-3.862683913263985, 7.065485370801925},
        {-3.8891925561135134, 7.1078796953346606},
        {-3.9159591093065962, 7.1501117384142541},
        {-3.9430035413081264, 7.192166307267156},
        {-3.9702199656370283, 7.2341098430367685},
        {-3.9977542386569249, 7.2758456341553188},
        {-4.02553182468112, 7.317419589684679},
        {-4.0534794299385419, 7.3588795137859471},
        {-4.0817414827670149, 7.4001259544406048},
        {-4.110267998016516, 7.4411895736127036},
        {-4.1390105525517678, 7.482102126755847},
        {-4.168031349842658, 7.5228178304055753},
        {-4.1973557660084344, 7.5633153549798413},
        {-4.2270991176184047, 7.6035060156805345},
        {-4.257486472064735, 7.6432119068184932},
        {-4.2884302389446844, 7.682486246750944},
        {-4.3193760941908614, 7.7217590577509485},
        {-4.35028504237811, 7.761060924344604},
        {-4.3811358927982864, 7.800408408610414},
        {-4.4119815874430941, 7.8397599186524234},
        {-4.442890960705741, 7.8790614317472718},
        {-4.4737635041134549, 7.9183919232518765},
        {-4.5046549442979522, 7.95770753216845},
        {-4.5355966320363841, 7.99698361169358},
        {-4.5664813774927167, 8.0363045370409534},
        {-4.5973429218609487, 8.0756436033017067},
        {-4.6281831865274468, 8.1149993578213238},
        {-4.6589154633284364, 8.1544395019483211},
        {-4.689488490770013, 8.1940033131696151},
        {-4.7198703273598461, 8.2337139839648525},
        {-4.7497588129829111, 8.2737966668217826},
        {-4.7789184059509555, 8.3144123615119678},
        {-4.8077646853665721, 8.355252168330324},
        {-4.8364858223466012, 8.3961801756277765},
        {-4.8651242415629072, 8.43716606115912},
        {-4.8936198808669058, 8.4782513323370257},
        {-4.92182578788273, 8.5195349744061026},
        {-4.9492492294599089, 8.5613424260267035},
        {-4.9750615856317344, 8.60415759196051},
        {-4.9980474751643236, 8.6485463694254339},
        {-5.0046565809706411, 8.6634681680586052},
        {-5.0451537083690159, 8.7549011089086068}};
    double final_angle = 1.9877437396171762;

    dv_path_tracker.Clear();
    EXPECT_TRUE(dv_path_tracker.SetPath(
        dv_limits, eigenmath::Pose2d(), polyline_pts, final_angle,
        kDvMediumDesiredSpeed, 0.0, eigenmath::Pose2d(polyline_pts[0], 0.0)))
        << " Got error string: " << dv_path_tracker.GetErrorString();
  }

  {
    const eigenmath::Vector2d polyline_pts[] = {
        {-13.628508758999189, -9.3060233716606078},
        {-13.678051434182651, -9.2992764159819536},
        {-13.72756142622287, -9.292294101918797},
        {-13.777047979611089, -9.28514756183638},
        {-13.82650142885748, -9.2777751332301452},
        {-13.875916852354383, -9.270152273950595},
        {-13.92531259900964, -9.2624026307937015},
        {-13.974660573358927, -9.25435347224621},
        {-14.023973155729257, -9.2460914335786946},
        {-14.073264421341856, -9.2377028764323281},
        {-14.122504040603664, -9.229015447546125},
        {-14.171702532309569, -9.2200993248468261},
        {-14.220867906743806, -9.21100267431357},
        {-14.269977528264649, -9.2016103223945116},
        {-14.318978521287152, -9.1916673801271127},
        {-14.367835269943575, -9.18103884659759},
        {-14.41651359969382, -9.1696226894928756},
        {-14.464999731711236, -9.1574142141696431},
        {-14.51331167976681, -9.1445335099352913},
        {-14.561486522468, -9.1311492855447742},
        {-14.609503647687164, -9.1172089722428176},
        {-14.657340012545237, -9.1026608203340835},
        {-14.705027513425557, -9.0876323496988238},
        {-14.752538092620727, -9.0720528274401318},
        {-14.799847163583523, -9.0558720261178411},
        {-14.846990978989142, -9.0392162962075044},
        {-14.893939070878776, -9.0220159090841836},
        {-14.940664105263574, -9.0042188204931133},
        {-14.987198346467308, -8.985929307008627},
        {-15.033491945278559, -8.9670387363313},
        {-15.079516011880067, -8.9475009025220213},
        {-15.125238645887327, -8.9272678136708485},
        {-15.170577792999945, -8.90618990560406},
        {-15.215596001705629, -8.88443355687247},
        {-15.260534967703657, -8.8625135335892029},
        {-15.305341220616283, -8.8403250554661259},
        {-15.349533463087495, -8.8169418815156657},
        {-15.392504408621159, -8.7914059061355552},
        {-15.428956571087429, -8.7640955414182837},
        {-15.449479567858551, -8.7454290144527924},
        {-15.523456964187533, -8.67814346747289}};
    double final_angle = 2.4035306865707384;

    dv_path_tracker.Clear();
    EXPECT_TRUE(dv_path_tracker.SetPath(
        dv_limits, eigenmath::Pose2d(), polyline_pts, final_angle,
        kDvMediumDesiredSpeed, 0.0, eigenmath::Pose2d(polyline_pts[0], 0.0)))
        << " Got error string: " << dv_path_tracker.GetErrorString();
  }
}

}  // namespace
}  // namespace mobility::diff_drive
