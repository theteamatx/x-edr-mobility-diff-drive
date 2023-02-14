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

#include "diff_drive/curve_trajectory_utils.h"

#include <cmath>
#include <iostream>
#include <iterator>
#include <ostream>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "absl/flags/flag.h"
#include "diff_drive/dynamic_limits.h"
#include "diff_drive/matchers.h"
#include "diff_drive/test_curves.h"
#include "diff_drive/test_trajectories.h"
#include "diff_drive/test_utils.h"
#include "diff_drive/test_wheel_curves.h"
#include "eigenmath/matchers.h"
#include "eigenmath/sampling.h"
#include "genit/zip_iterator.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

ABSL_FLAG(bool, print_results, false,
          "Print the results of curve / trajectory computations.");

namespace mobility::diff_drive {
namespace {

using eigenmath::testing::IsApprox;
using testing::CurveEvaluatesTo;
using testing::IsCurveApprox;
using testing::IsCurvePointApprox;
using testing::IsStateApprox;
using testing::IsTrajectoryApprox;
using testing::IsWheelCurveApprox;
using testing::kBigEpsilon;
using testing::kEpsilon;
using testing::kTestClothoidTraj;
using testing::kTestCurve;
using testing::kTestCurveEval;
using testing::kTestDynamicLimits;
using testing::kTestKinematics;
using testing::kTestTraj;
using testing::kTestTrajEval;
using testing::kTestWheelCurve;
using testing::kTestWheelCurveEval;
using testing::ToCurve;
using testing::ToState;
using testing::ToTrajectory;
using testing::ToWheelCurve;
using testing::TrajectoryEvaluatesTo;
using testing::WheelCurveEvaluatesTo;

constexpr double kMidEpsilon = 5e-5;

TEST(CurveTrajectoryUtils, CurveToTrajectory) {
  Trajectory converted_traj(64);
  ASSERT_TRUE(ConvertCurveToTrajectory(kTestCurve, 1.0, kTestKinematics,
                                       &converted_traj));
  EXPECT_THAT(converted_traj, IsTrajectoryApprox(kTestTraj, kEpsilon));

  const DynamicLimits fake_limits(kTestKinematics, WheelVector(5.0, 5.0),
                                  WheelVector(30.0, 30.0), ArcVector(6.0, 10.0),
                                  ArcVector(60.0, 60.0));
  Trajectory small_converted_traj = converted_traj;
  small_converted_traj.TruncateTo({0.0, 1.0});

  // Check overloads.
  {
    Trajectory alternate_traj(64);
    ASSERT_TRUE(ConvertCurveToTrajectory(kTestCurve, 1.0, fake_limits,
                                         &alternate_traj));
    EXPECT_THAT(alternate_traj, IsTrajectoryApprox(converted_traj, kEpsilon));
  }
  {
    Trajectory alternate_traj(64);
    ASSERT_TRUE(ConvertCurveToTrajectory(kTestCurve, {0.0, 1.0}, 1.0,
                                         kTestKinematics, &alternate_traj));
    EXPECT_THAT(alternate_traj,
                IsTrajectoryApprox(small_converted_traj, kEpsilon));
  }
  {
    Trajectory alternate_traj(64);
    ASSERT_TRUE(ConvertCurveToTrajectory(kTestCurve, {0.0, 1.0}, 1.0,
                                         fake_limits, &alternate_traj));
    EXPECT_THAT(alternate_traj,
                IsTrajectoryApprox(small_converted_traj, kEpsilon));
  }
}

TEST(CurveTrajectoryUtils, CurveToTrajectoryBackwards) {
  Trajectory converted_traj(64);
  ASSERT_TRUE(ConvertCurveToTrajectory(kTestCurve, -1.0, kTestKinematics,
                                       &converted_traj));
  const std::vector<testing::RawStateTime> reverse_traj_pts = {
      {0.0, 0.0, 0.0, M_PI, -1.0, 2.0 * M_PI},
      {0.25, 0.5 / M_PI, 0.5 / M_PI, -0.5 * M_PI, -1.0, 0.0},
      {1.25, 0.5 / M_PI, 1.0 + 0.5 / M_PI, -0.5 * M_PI, -1.0, -2.0 * M_PI},
      {1.75, 1.5 / M_PI, 1.0 + 0.5 / M_PI, 0.5 * M_PI, -1.0, 0.0},
      {2.75, 1.5 / M_PI, 0.5 / M_PI, 0.5 * M_PI, -1.0, 2.0 * M_PI},
      {3.0, 2.0 / M_PI, 0.0, M_PI, -1.0, 0.0}};
  EXPECT_THAT(converted_traj,
              IsTrajectoryApprox(ToTrajectory(reverse_traj_pts), kEpsilon));
}

TEST(CurveTrajectoryUtils, TrajectoryToCurve) {
  Curve converted_curve(64);
  ASSERT_TRUE(ConvertTrajectoryToCurve(kTestTraj, &converted_curve));
  EXPECT_TRUE(converted_curve.HasContinuousPosition());
  EXPECT_TRUE(converted_curve.HasMonotonicCordLengths());
  EXPECT_THAT(converted_curve, CurveEvaluatesTo(kTestCurveEval, kBigEpsilon));
  EXPECT_THAT(converted_curve, IsCurveApprox(kTestCurve, kBigEpsilon));
}

TEST(CurveTrajectoryUtils, TrajectoryToCurveBackwards) {
  Trajectory converted_traj(64);
  ASSERT_TRUE(ConvertCurveToTrajectory(kTestCurve, -1.0, kTestKinematics,
                                       &converted_traj));
  Curve converted_curve(64);
  ASSERT_TRUE(ConvertTrajectoryToCurve(converted_traj, &converted_curve));
  EXPECT_THAT(converted_curve, CurveEvaluatesTo(kTestCurveEval, kBigEpsilon));
  EXPECT_THAT(converted_curve, IsCurveApprox(kTestCurve, kBigEpsilon));
}

TEST(CurveTrajectoryUtils, PolylineToTrajectory) {
  const Curve polyline = ToCurve({{0.0, 0.0, 0.0, 0.0, 0.0},
                                  {1.0, 1.0, 0.0, 0.5 * M_PI, 0.0},
                                  {2.0, 1.0, 1.0, 0.0, 0.0},
                                  {3.0, 2.0, 1.0, -0.5 * M_PI, 0.0},
                                  {4.0, 2.0, 0.0, 0.0, 0.0}});

  Trajectory converted_traj(64);
  ASSERT_TRUE(ConvertCurveToTrajectory(polyline, 1.0, kTestKinematics,
                                       &converted_traj));

  const Trajectory polyline_traj =
      ToTrajectory({{0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
                    {1.0, 1.0, 0.0, 0.0, 0.0, 1.0},
                    {1.0 + 0.5 * M_PI, 1.0, 0.0, 0.5 * M_PI, 1.0, 0.0},
                    {2.0 + 0.5 * M_PI, 1.0, 1.0, 0.5 * M_PI, 0.0, -1.0},
                    {2.0 + 1.0 * M_PI, 1.0, 1.0, 0.0, 1.0, 0.0},
                    {3.0 + 1.0 * M_PI, 2.0, 1.0, 0.0, 0.0, -1.0},
                    {3.0 + 1.5 * M_PI, 2.0, 1.0, -0.5 * M_PI, 1.0, 0.0},
                    {4.0 + 1.5 * M_PI, 2.0, 0.0, -0.5 * M_PI, 0.0, 1.0},
                    {4.0 + 2.0 * M_PI, 2.0, 0.0, 0.0, 1.0, 0.0}});

  EXPECT_THAT(converted_traj, IsTrajectoryApprox(polyline_traj, kBigEpsilon));
}

TEST(CurveTrajectoryUtils, PolylineToTrajectoryBackwards) {
  const Curve polyline = ToCurve({{0.0, 0.0, 0.0, 0.0, 0.0},
                                  {1.0, 1.0, 0.0, 0.5 * M_PI, 0.0},
                                  {2.0, 1.0, 1.0, 0.0, 0.0},
                                  {3.0, 2.0, 1.0, -0.5 * M_PI, 0.0},
                                  {4.0, 2.0, 0.0, 0.0, 0.0}});

  Trajectory converted_traj(64);
  EXPECT_TRUE(ConvertCurveToTrajectory(polyline, -1.0, kTestKinematics,
                                       &converted_traj));

  const Trajectory polyline_traj =
      ToTrajectory({{0.0, 0.0, 0.0, M_PI, -1.0, 0.0},
                    {1.0, 1.0, 0.0, M_PI, 0.0, 1.0},
                    {1.0 + 0.5 * M_PI, 1.0, 0.0, -0.5 * M_PI, -1.0, 0.0},
                    {2.0 + 0.5 * M_PI, 1.0, 1.0, -0.5 * M_PI, 0.0, -1.0},
                    {2.0 + 1.0 * M_PI, 1.0, 1.0, M_PI, -1.0, 0.0},
                    {3.0 + 1.0 * M_PI, 2.0, 1.0, M_PI, 0.0, -1.0},
                    {3.0 + 1.5 * M_PI, 2.0, 1.0, 0.5 * M_PI, -1.0, 0.0},
                    {4.0 + 1.5 * M_PI, 2.0, 0.0, 0.5 * M_PI, 0.0, 1.0},
                    {4.0 + 2.0 * M_PI, 2.0, 0.0, M_PI, -1.0, 0.0}});

  EXPECT_THAT(converted_traj, IsTrajectoryApprox(polyline_traj, kBigEpsilon));
}

TEST(CurveTrajectoryUtils, ResampleCurve) {
  Curve converted_curve(64);
  EXPECT_TRUE(ResampleCurve(kTestCurve, 0.1, &converted_curve));
  EXPECT_TRUE(converted_curve.HasContinuousPosition());
  EXPECT_TRUE(converted_curve.HasMonotonicCordLengths());
  EXPECT_THAT(converted_curve, CurveEvaluatesTo(kTestCurveEval, kBigEpsilon));
}

TEST(CurveTrajectoryUtils, ResampleCurveToPolyline) {
  Curve converted_curve(128);
  EXPECT_TRUE(ResampleCurveToPolyline(kTestCurve, 0.1, &converted_curve));
  EXPECT_TRUE(converted_curve.HasContinuousPosition());
  EXPECT_TRUE(converted_curve.HasMonotonicCordLengths());
  EXPECT_TRUE(CheckPolylineProperties(converted_curve));
  double expected_matched_cord = 0.0;
  int i = 0;
  for (auto& cur_pt : converted_curve.GetCurvePointIteratorRange()) {
    CurvePtAndCord matched_pt = kTestCurve.FindClosestMatchToPoint(
        cur_pt.point.GetPose().translation(), nullptr);
    EXPECT_NEAR(expected_matched_cord, matched_pt.cord_length, kBigEpsilon)
        << "at index " << i;
    EXPECT_THAT(cur_pt.point.GetPose().translation(),
                IsApprox(matched_pt.point.GetPose().translation(), kBigEpsilon))
        << "at index " << i;
    EXPECT_NEAR(cur_pt.point.GetCurvature(), 0.0, kEpsilon) << "at index " << i;
    expected_matched_cord += 0.1;
    ++i;
  }

  Curve single_pt_curve(2);
  single_pt_curve.AddPoint(
      1.0, CurvePoint(eigenmath::Pose2d({0.5, -1.0}, 0.0), 0.5));
  Curve single_pt_polyline(2);
  EXPECT_TRUE(
      ResampleCurveToPolyline(single_pt_curve, 0.1, &single_pt_polyline));
  EXPECT_EQ(single_pt_polyline.GetSize(), 1);
  EXPECT_NEAR(single_pt_polyline.GetStart().cord_length,
              single_pt_curve.GetStart().cord_length, kEpsilon);
  EXPECT_NEAR(single_pt_polyline.GetStart().point.GetPose().translation().x(),
              single_pt_curve.GetStart().point.GetPose().translation().x(),
              kEpsilon);
  EXPECT_NEAR(single_pt_polyline.GetStart().point.GetPose().translation().y(),
              single_pt_curve.GetStart().point.GetPose().translation().y(),
              kEpsilon);
  EXPECT_NEAR(single_pt_polyline.GetStart().point.GetCurvature(), 0.0,
              kEpsilon);
}

TEST(CurveTrajectoryUtils, ResampleTrajectory) {
  Trajectory converted_traj(64);
  ASSERT_TRUE(ResampleTrajectory(kTestTraj, 0.1, &converted_traj));
  EXPECT_GE(converted_traj.GetSize(), 30);
  EXPECT_THAT(converted_traj,
              TrajectoryEvaluatesTo(kTestTrajEval, kBigEpsilon));
}

TEST(CurveTrajectoryUtils, TimeWarpTrajectoryIntoLimits) {
  DynamicLimits lowered_limits = kTestDynamicLimits;
  lowered_limits.MutableVelocityLimits()->ScaleLimits(0.5);
  Trajectory warped_traj = kTestTraj;
  TimeWarpTrajectoryIntoLimits(lowered_limits, &warped_traj);
  EXPECT_EQ(warped_traj.GetSize(), kTestTraj.GetSize());
  EXPECT_EQ(warped_traj.GetTimeSpan().min(), kTestTraj.GetTimeSpan().min());
  EXPECT_GE(warped_traj.GetTimeSpan().Length(),
            kTestTraj.GetTimeSpan().Length());
  for (const auto& state : warped_traj.GetStateIteratorRange()) {
    EXPECT_TRUE(lowered_limits.VelocityLimits().IsInBounds(
        lowered_limits.GetKinematics(), state.state.GetArcVelocity()))
        << "At time " << state.time << ", arc velocity is ("
        << state.state.GetArcVelocity().transpose() << ").";
  }
  EXPECT_NEAR(warped_traj.ComputeTotalCordLength(),
              kTestTraj.ComputeTotalCordLength(), kBigEpsilon);
  Curve warped_traj_as_curve(warped_traj.GetCapacity());
  EXPECT_TRUE(ConvertTrajectoryToCurve(warped_traj, &warped_traj_as_curve));
  Curve test_traj_as_curve(kTestTraj.GetCapacity());
  EXPECT_TRUE(ConvertTrajectoryToCurve(kTestTraj, &test_traj_as_curve));
  EXPECT_THAT(warped_traj_as_curve,
              IsCurveApprox(test_traj_as_curve, kBigEpsilon))
      << "Test traj:\n"
      << kTestTraj << "Warped traj:\n"
      << warped_traj;
}

TEST(CurveTrajectoryUtils, DecaySpeedOfTrajectory) {
  Trajectory warped_traj = kTestClothoidTraj;
  const TrajectoryLimits test_traj_limits(kTestDynamicLimits, 0.1);
  const double decay_rate = 0.5;
  DecaySpeedOfTrajectory(kTestKinematics, test_traj_limits, decay_rate,
                         &warped_traj);
  EXPECT_EQ(warped_traj.GetSize(), kTestClothoidTraj.GetSize());
  EXPECT_EQ(warped_traj.GetTimeSpan().min(),
            kTestClothoidTraj.GetTimeSpan().min());
  EXPECT_GE(warped_traj.GetTimeSpan().Length(),
            kTestClothoidTraj.GetTimeSpan().Length());
  for (const auto& [orig_state, new_state] :
       genit::ZipRange(kTestClothoidTraj.GetStateIteratorRange(),
                       warped_traj.GetStateIteratorRange())) {
    EXPECT_GE(new_state.time, orig_state.time);
    EXPECT_THAT(new_state.state.GetPose(),
                IsApprox(orig_state.state.GetPose(), kEpsilon));
    EXPECT_LE(new_state.state.GetArcVelocity().lpNorm<Eigen::Infinity>(),
              orig_state.state.GetArcVelocity().lpNorm<Eigen::Infinity>());
    const double decay_factor = std::exp(
        -decay_rate * (new_state.time - kTestClothoidTraj.GetTimeSpan().min()));
    EXPECT_NEAR(std::abs(new_state.state.GetArcVelocity().Translation()),
                std::abs(orig_state.state.GetArcVelocity().Translation()) *
                    decay_factor,
                kBigEpsilon);
    EXPECT_NEAR(
        std::abs(new_state.state.GetArcVelocity().Rotation()),
        std::abs(orig_state.state.GetArcVelocity().Rotation()) * decay_factor,
        kBigEpsilon);
  }
  EXPECT_NEAR(warped_traj.ComputeTotalCordLength(),
              kTestClothoidTraj.ComputeTotalCordLength(), kBigEpsilon);
  Curve warped_traj_as_curve(warped_traj.GetCapacity());
  EXPECT_TRUE(ConvertTrajectoryToCurve(warped_traj, &warped_traj_as_curve));
  Curve test_traj_as_curve(kTestClothoidTraj.GetCapacity());
  EXPECT_TRUE(ConvertTrajectoryToCurve(kTestClothoidTraj, &test_traj_as_curve));
  EXPECT_THAT(warped_traj_as_curve,
              IsCurveApprox(test_traj_as_curve, kBigEpsilon))
      << "Test traj:\n"
      << kTestClothoidTraj << "Warped traj:\n"
      << warped_traj;
}

TEST(CurveTrajectoryUtils, BringTrajectoryIntoLimits) {
  Trajectory warped_traj = kTestTraj;
  TimeWarpTrajectoryIntoLimits(kTestDynamicLimits, &warped_traj);
  Trajectory limited_traj(128);
  const TrajectoryLimits test_traj_limits(kTestDynamicLimits, 0.1);
  EXPECT_TRUE(BringTrajectoryIntoLimits(ArcVector::Zero(), warped_traj,
                                        kTestDynamicLimits, test_traj_limits,
                                        &limited_traj))
      << "Warped traj:\n"
      << warped_traj << "Dynamic limit:\n"
      << kTestDynamicLimits << "Trajectory limits:\n"
      << test_traj_limits << "Partial traj:\n"
      << limited_traj;
  EXPECT_EQ(limited_traj.GetTimeSpan().min(), kTestTraj.GetTimeSpan().min());
  ArcVector prev_arc_vel = ArcVector::Zero();
  for (const auto& state : limited_traj.GetStateIteratorRange()) {
    EXPECT_TRUE(test_traj_limits.GetVelocityJumpLimits().IsInBounds(
        kTestDynamicLimits.GetKinematics(),
        ArcVector(state.state.GetArcVelocity() - prev_arc_vel)))
        << "At time " << state.time << " excessive jump from ("
        << prev_arc_vel.transpose() << ") to ("
        << state.state.GetArcVelocity().transpose() << ").";
    prev_arc_vel = state.state.GetArcVelocity();
  }
}

TEST(CurveTrajectoryUtils, BringTrajectoryIntoLimitsRegressions) {
  const Kinematics meta_kinematics(WheelVector(0.0625, 0.0625), 0.365);
  const diff_drive::DynamicLimits meta_dd_limits_fast_fw(
      meta_kinematics, WheelVector(-12.6, -12.6), WheelVector(12.6, 12.6),
      WheelVector(-120.0, -120.0), WheelVector(120.0, 120.0),
      ArcVector(-0.24, -1.2), ArcVector(0.8, 1.2), ArcVector(-4.0, -8.0),
      ArcVector(4.0, 8.0));

  const TrajectoryLimits meta_traj_limits_relaxed(WheelVector(6.0, 6.0),
                                                  ArcVector(0.05, 0.2), 0.05);

  {
    ArcVector start_arc_vel{0.722579, -0.146216};
    Trajectory traj_in = ToTrajectory(
        {{0.124311707, 29.30622380748629, -30.46844802667221,
          -1.174724550931439, 0.7279592443719516, 0.3262507157701287},
         {0.2425855425640781, 29.34096448802277, -30.5472206270754,
          -1.13613762742178, 0.7176275502926925, 0.3828627381222323},
         {0.3610585791848087, 29.37850260456701, -30.62349623599633,
          -1.090778716227512, 0.7062386451228827, 0.4452676979568079},
         {0.4797657263832953, 29.41916418559483, -30.69679973694181,
          -1.037922258063421, 0.6970319005837293, 0.495715613239839},
         {0.5983428812331889, 29.46321992904599, -30.76671745789449,
          -0.979141711030771, 0.6900475509742996, 0.5339860220586328},
         {0.7164053695664725, 29.51075863172337, -30.83286145136942,
          -0.9160979925313372, 0.6847726721291104, 0.5628894677856965},
         {0.8336821598958443, 29.56172610298267, -30.8949045050694,
          -0.8500841224392223, 0.6810025228290474, 0.583547820114807},
         {0.9499791847313793, 29.61596907323036, -30.95259077933439,
          -0.7822192471106082, 0.6786507970927598, 0.5964339885328236},
         {1.065169749125972, 29.67327047614712, -31.00574476664995,
          -0.7135156793473945, 0.6776453468686501, 0.6019433048293144},
         {1.179201901743146, 29.73338341146652, -31.05427581552971,
          -0.6448747885442119, 0.6779031567620278, 0.6005306478792994},
         {1.292099082063338, 29.79605901153137, -31.09817205976709,
          -0.5770765717027807, 0.6807798139251404, 0.5847681428759437},
         {1.404299934439127, 29.86139425533699, -31.13771621382921,
          -0.5114650876298926, 0.6855679859894036, 0.5585315836197069},
         {1.515939831362893, 29.92926078116746, -31.17307309990365,
          -0.4491106792059212, 0.6909071671701127, 0.5292757963281498},
         {1.626782335808781, 29.99918376584125, -31.20428013088769,
          -0.3904444243983174, 0.6966545152065597, 0.4977834783202202},
         {1.736716321953304, 30.07076792329871, -31.231476847714,
          -0.3357211023896897, 0.7026322863813701, 0.4650285677733152},
         {1.845656138921827, 30.14364701608874, -31.25485345240403,
          -0.2850609753313302, 0.7087076055914916, 0.4317391474438812},
         {1.953535211788185, 30.21749044806586, -31.27463766504372,
          -0.2384853563849726, 0.7148772128377682, 0.3979330803409969},
         {2.060294410155643, 30.29201068350617, -31.29108630956835,
          -0.1960023397238749, 0.7211817711672298, 0.3633875552480559},
         {2.165882408210438, 30.3669669567664, -31.30447977393275,
          -0.1576329752472063, 0.7232255732423677, 0.3521886397678475},
         {2.270968868525014, 30.44222950138749, -31.31501897850719,
          -0.1206227177309978, 0.7271257369811278, 0.3308178795554634},
         {2.376439991104168, 30.5185085572736, -31.32291725816136,
          -0.08573098460502798, 0.7455557898947138, 0.229831288248143},
         {2.47774207768954, 30.59382601479555, -31.3285077458058,
          -0.06244859554288696, 0.7226372494661973, 0.3554123316920703},
         {2.582041565509409, 30.66911973662276, -31.33181631635591,
          -0.02537927138253886, 0.7497278315101358, -0.1904833617787202},
         {2.682041565509409, 30.74404572384476, -31.33443255351243,
          -0.04442760756041089, 0.7449562577593823, -0.2331163958390018},
         {2.782153870934336, 30.81850600942531, -31.3386139031276,
          -0.06776542738020305, 0.7371759933788816, -0.2757479814855797},
         {2.882945176684224, 30.8925569503892, -31.34467452374494,
          -0.09555842649203065, 0.729318329594362, -0.3188036734555501},
         {2.984093036221392, 30.96586318814867, -31.35289568898611,
          -0.1278047356746455, 0.7226511372392954, -0.3553362343052299},
         {3.08557386278951, 31.03841603551229, -31.36355206057184,
          -0.1638645504415429, 0.7163022885143843, -0.3901244464965233},
         {3.187304355523169, 31.11005494072377, -31.37686280224642,
          -0.2035521026110798, 0.7101603208987498, -0.4237790635684938},
         {3.2892076131419, 31.18059068725915, -31.39301741187351,
          -0.2467365696993248, 0.7040602745773508, -0.4572039749186272}});

    Trajectory limited_traj(2048);
    EXPECT_TRUE(BringTrajectoryIntoLimits(
        start_arc_vel, traj_in, meta_dd_limits_fast_fw,
        meta_traj_limits_relaxed, &limited_traj))
        << "Traj:\n"
        << traj_in << "Dynamic limit:\n"
        << meta_dd_limits_fast_fw << "Trajectory limits:\n"
        << meta_traj_limits_relaxed << "Partial traj:\n"
        << limited_traj;
  }
}

TEST(CurveTrajectoryUtils, RoundOffCurve) {
  const Curve polyline = ToCurve(
      {{0.0, 0.0, 0.0, 0.0, 0.0},
       {0.5 / M_PI, 0.5 / M_PI, 0.0, 0.5 * M_PI, 0.0},
       {1.0 + 1.5 / M_PI, 0.5 / M_PI, 1.0 + 1.0 / M_PI, 0.0, 0.0},
       {1.0 + 2.5 / M_PI, 1.5 / M_PI, 1.0 + 1.0 / M_PI, -0.5 * M_PI, 0.0},
       {2.0 + 3.5 / M_PI, 1.5 / M_PI, 0.0, 0.0, 0.0},
       {2.0 + 4.0 / M_PI, 2.0 / M_PI, 0.0, 0.0, 0.0}});
  EXPECT_TRUE(CheckPolylineProperties(polyline));

  Curve rounded_curve(64);
  EXPECT_TRUE(RoundOffCornersOfPolyline(polyline, &rounded_curve));
  EXPECT_TRUE(rounded_curve.HasContinuousPosition());
  EXPECT_TRUE(rounded_curve.HasMonotonicCordLengths());
  EXPECT_FALSE(CheckPolylineProperties(rounded_curve));
  EXPECT_THAT(rounded_curve, CurveEvaluatesTo(kTestCurveEval, kBigEpsilon));
}

TEST(CurveTrajectoryUtils, RoundOffCurveAcuteAndObtuse) {
  const Curve polyline =
      ToCurve({{0.0, 0.0, 0.0, 0.25 * M_PI, 0.0},
               {0.5 * std::sqrt(2.0), 0.5, 0.5, 0.25 * M_PI, 0.0},
               {std::sqrt(2.0), 1.0, 1.0, -0.25 * M_PI, 0.0},
               {1.5 * std::sqrt(2.0), 1.5, 0.5, -0.25 * M_PI, 0.0},
               {2.0 * std::sqrt(2.0), 2.0, 0.0, -0.25 * M_PI, 0.0},
               {2.5 * std::sqrt(2.0), 2.5, -0.5, -0.25 * M_PI, 0.0},
               {3.0 * std::sqrt(2.0), 3.0, -1.0, M_PI, 0.0},
               {3.0 * std::sqrt(2.0) + 1.5, 1.5, -1.0, -0.75 * M_PI, 0.0},
               {3.5 * std::sqrt(2.0) + 1.5, 1.0, -1.5, -0.75 * M_PI, 0.0},
               {4.0 * std::sqrt(2.0) + 1.5, 0.5, -2.0, -0.75 * M_PI, 0.0}});

  Curve rounded_curve(64);
  EXPECT_TRUE(RoundOffCornersOfPolyline(polyline, &rounded_curve));
  EXPECT_TRUE(rounded_curve.HasContinuousPosition());
  EXPECT_TRUE(rounded_curve.HasMonotonicCordLengths());

  const std::vector<testing::RawCurvePointAndCord> expected_rounded_pts = {
      {0.0, 0.0, 0.0, 0.785398, 0.0},
      {0.707107, 0.5, 0.5, 0.785398, 0.0},
      {1.06067, 0.75, 0.75, 0.785398, -2.82843},
      {1.61603, 1.25, 0.75, -0.785398, 0.0},
      {1.96957, 1.5, 0.5, -0.785398, 0.0},
      {2.32313, 1.75, 0.25, -0.785398, 0.0},
      {2.67668, 2.0, 0.0, -0.785398, 0.0},
      {3.03023, 2.25, -0.25, -0.785398, 0.0},
      {3.38379, 2.5, -0.5, -0.785398, 0.0},
      {3.73735, 2.75, -0.75, -0.785398, -6.82843},
      {4.08241, 2.64645, -1.0, 3.14159, 0.0},
      {4.47884, 2.25, -1.0, 3.14159, 0.0},
      {4.87530, 1.85355, -1.0, -3.14159, 1.17157},
      {5.54568, 1.25, -1.25, -2.35619, 0.0},
      {5.89922, 1.0, -1.5, -2.35619, 0.0},
      {6.60633, 0.5, -2.0, -2.35619, 0.0}};
  EXPECT_THAT(rounded_curve,
              CurveEvaluatesTo(expected_rounded_pts, kBigEpsilon));
}

TEST(CurveTrajectoryUtils, ComputeTwoArcsToFinalPose) {
  static constexpr int kXYSamples = 20;
  static constexpr int kThetaSamples = 16;
  for (int i = -kXYSamples; i <= kXYSamples; ++i) {
    for (int j = -kXYSamples; j <= kXYSamples; ++j) {
      const eigenmath::Vector2d final_pt(i, j);
      for (int k = -kThetaSamples; k < kThetaSamples; ++k) {
        const eigenmath::SO2d final_so2(k * M_PI / kThetaSamples);
        double kappa1, arc_length1, kappa2, arc_length2;
        eigenmath::Pose2d median_pose;
        const bool result = ComputeTwoArcsToFinalPose(
            final_pt, final_so2, &kappa1, &arc_length1, &kappa2, &arc_length2,
            &median_pose);
        if (!result) {
          // Known failure cases to ignore:
          if (i == 0 && j == 0) {
            continue;
          }
          if (k == -kThetaSamples / 2 && i == j) {
            continue;
          }
          if (k == kThetaSamples / 2 && i == -j) {
            continue;
          }
          if (k == 0 && i < 0 && j == 0) {
            continue;
          }
          if (k == -kThetaSamples && j == 0) {
            continue;
          }
        }
        EXPECT_TRUE(result) << "Failed with point: " << final_pt.transpose()
                            << " and angle: " << final_so2.angle();

        if (result) {
          Curve curve(3);
          curve.AddPoint(0.0, CurvePoint(eigenmath::Pose2d(), kappa1));
          eigenmath::Pose2d new_median =
              curve.GetFinish()
                  .point.ExtrapolateConstantArc(arc_length1)
                  .GetPose();
          ASSERT_TRUE(std::isfinite(new_median.so2().angle()))
              << "Failed with point: " << final_pt.transpose()
              << " and angle: " << final_so2.angle();
          EXPECT_THAT(median_pose, IsApprox(new_median, kBigEpsilon))
              << "Failed with point: " << final_pt.transpose()
              << " and angle: " << final_so2.angle()
              << " expected: t = " << median_pose.translation().transpose()
              << " r = " << median_pose.so2().xAxis().transpose()
              << " got: t = " << new_median.translation().transpose()
              << " r = " << new_median.so2().xAxis().transpose();
          curve.AddPoint(arc_length1, CurvePoint(median_pose, kappa2));
          eigenmath::Pose2d new_final =
              curve.GetFinish()
                  .point.ExtrapolateConstantArc(arc_length2)
                  .GetPose();
          ASSERT_TRUE(std::isfinite(new_final.so2().angle()))
              << "Failed with point: " << final_pt.transpose()
              << " and angle: " << final_so2.angle();
          EXPECT_THAT(eigenmath::Pose2d(final_pt, final_so2),
                      IsApprox(new_final, kBigEpsilon))
              << "Failed with point: " << final_pt.transpose()
              << " and angle: " << final_so2.angle()
              << " expected: t = " << final_pt.transpose()
              << " r = " << final_so2.xAxis().transpose()
              << " got: t = " << new_final.translation().transpose()
              << " r = " << new_final.so2().xAxis().transpose();
          curve.AddPoint(arc_length1 + arc_length2, CurvePoint(new_final, 0.0));
        }
      }
    }
  }
}

// Most of the cases tested below are covered by the test above, but it is
// useful to have a separate test for specific known corner cases to debug.
TEST(CurveTrajectoryUtils, ComputeTwoArcsToFinalPoseSpecials) {
  {
    const eigenmath::Vector2d final_pt(-3, -3);
    const eigenmath::SO2d final_so2(-0.5 * M_PI);
    Curve output_curve(3);
    EXPECT_TRUE(ComputeTwoArcsToFinalPose(
        eigenmath::Pose2d(), eigenmath::Pose2d(final_pt, final_so2),
        &output_curve))
        << "Failed with point: " << final_pt.transpose()
        << " and angle: " << final_so2.angle();
    EXPECT_TRUE(output_curve.HasContinuousPosition()) << "Curve:\n"
                                                      << output_curve;
  }

  {
    const eigenmath::Vector2d final_pt(-3, 0);
    const eigenmath::SO2d final_so2(0.0);
    Curve output_curve(3);
    EXPECT_FALSE(ComputeTwoArcsToFinalPose(
        eigenmath::Pose2d(), eigenmath::Pose2d(final_pt, final_so2),
        &output_curve))
        << "Unexpectedly succeeded with point: " << final_pt.transpose()
        << " and angle: " << final_so2.angle();
  }

  {
    const eigenmath::Vector2d final_pt(0, 1);
    const eigenmath::SO2d final_so2(0.0);
    Curve output_curve(3);
    EXPECT_TRUE(ComputeTwoArcsToFinalPose(
        eigenmath::Pose2d(), eigenmath::Pose2d(final_pt, final_so2),
        &output_curve))
        << "Failed with point: " << final_pt.transpose()
        << " and angle: " << final_so2.angle();
    EXPECT_TRUE(output_curve.HasContinuousPosition()) << "Curve:\n"
                                                      << output_curve;
  }

  {
    const eigenmath::Vector2d final_pt(1, 0);
    const eigenmath::SO2d final_so2(0.0);
    Curve output_curve(3);
    EXPECT_TRUE(ComputeTwoArcsToFinalPose(
        eigenmath::Pose2d(), eigenmath::Pose2d(final_pt, final_so2),
        &output_curve))
        << "Failed with point: " << final_pt.transpose()
        << " and angle: " << final_so2.angle();
    EXPECT_TRUE(output_curve.HasContinuousPosition()) << "Curve:\n"
                                                      << output_curve;
  }

  {
    const eigenmath::Vector2d final_pt(3, 1);
    const eigenmath::SO2d final_so2(0.5 * M_PI);
    Curve output_curve(3);
    EXPECT_TRUE(ComputeTwoArcsToFinalPose(
        eigenmath::Pose2d(), eigenmath::Pose2d(final_pt, final_so2),
        &output_curve))
        << "Failed with point: " << final_pt.transpose()
        << " and angle: " << final_so2.angle();
    EXPECT_TRUE(output_curve.HasContinuousPosition()) << "Curve:\n"
                                                      << output_curve;
  }

  {
    const eigenmath::Vector2d final_pt(-3, -3);
    const eigenmath::SO2d final_so2(0.0);
    Curve output_curve(3);
    EXPECT_TRUE(ComputeTwoArcsToFinalPose(
        eigenmath::Pose2d(), eigenmath::Pose2d(final_pt, final_so2),
        &output_curve))
        << "Failed with point: " << final_pt.transpose()
        << " and angle: " << final_so2.angle();
    EXPECT_TRUE(output_curve.HasContinuousPosition()) << "Curve:\n"
                                                      << output_curve;
  }

  {
    const eigenmath::Vector2d final_pt(-3, -2);
    const eigenmath::SO2d final_so2(0.0);
    Curve output_curve(3);
    EXPECT_TRUE(ComputeTwoArcsToFinalPose(
        eigenmath::Pose2d(), eigenmath::Pose2d(final_pt, final_so2),
        &output_curve))
        << "Failed with point: " << final_pt.transpose()
        << " and angle: " << final_so2.angle();
    EXPECT_TRUE(output_curve.HasContinuousPosition()) << "Curve:\n"
                                                      << output_curve;
  }
}

TEST(CurveTrajectoryUtils, ComputeTwoArcsToFinalPoseRegressions) {
  {
    const eigenmath::Pose2d start_pose(eigenmath::Vector2d(-1.75, 3.15),
                                       1.57915541223);
    const eigenmath::Pose2d final_pose(eigenmath::Vector2d(-1.75, 3.35),
                                       1.56760344024);
    Curve output_curve(3);
    EXPECT_TRUE(
        ComputeTwoArcsToFinalPose(start_pose, final_pose, &output_curve))
        << "Unexpectedly failed from pose " << start_pose << " to pose "
        << final_pose;

    EXPECT_THAT(
        output_curve,
        IsCurveApprox(
            ToCurve({{0, -1.75, 3.15, 1.57915541223, -0.109421231953},
                     {0.100000169905, -1.7502888001, 3.249999254, 1.56821327044,
                      -0.00609823101373},
                     {0.200001334524, -1.75, 3.35, 1.56760344024, 0.0}}),
            kEpsilon));
  }

  {
    const eigenmath::Pose2d start_pose(eigenmath::Vector2d(-5.7, 11.6),
                                       0.657284831334);
    const eigenmath::Pose2d final_pose(eigenmath::Vector2d(-5.5, 11.75),
                                       0.603183693993);
    Curve output_curve(3);
    EXPECT_TRUE(
        ComputeTwoArcsToFinalPose(start_pose, final_pose, &output_curve))
        << "Unexpectedly failed from pose " << start_pose << " to pose "
        << final_pose;

    EXPECT_THAT(
        output_curve,
        IsCurveApprox(
            ToCurve({{0, -5.7, 11.6, 0.657284831334, -0.0041727118658},
                     {0.125033868822, -5.60099651222, 11.6763660703,
                      0.656763101038, -0.428621},
                     {0.250037824436, -5.5, 11.75, 0.603183693993, 0.0}}),
            kEpsilon));
  }

  {
    const eigenmath::Pose2d start_pose(
        eigenmath::Vector2d(-0.635279067271, -0.00022314018254),
        -0.000695474978576);
    const eigenmath::Pose2d final_pose(
        eigenmath::Vector2d(0.317321775201, -0.000886025680988),
        -0.000695594151467);
    Curve output_curve(3);
    EXPECT_TRUE(
        ComputeTwoArcsToFinalPose(start_pose, final_pose, &output_curve))
        << "Unexpectedly failed from pose " << start_pose << " to pose "
        << final_pose;

    EXPECT_TRUE(output_curve.HasContinuousPosition()) << "Curve:\n"
                                                      << output_curve;

    EXPECT_THAT(
        output_curve,
        IsCurveApprox(
            ToCurve({{0.0, -0.635279067271, -0.00022314018254,
                      -0.000695474978576, -1.52938644843e-06},
                     {0.476300536556, -0.158978646025, -0.000554568741239,
                      -0.000696203426162, 1.27918120646e-06},
                     {0.952601073113, 0.317321775201, -0.000886025680988,
                      -0.000695594151467, 0}}),
            kEpsilon));
  }

  {
    const eigenmath::Pose2d start_pose(
        eigenmath::Vector2d(-0.982960815674, -0.000175516398989),
        -0.0067591112826);
    const eigenmath::Pose2d final_pose(
        eigenmath::Vector2d(-0.682927538875, -0.00220580375282),
        -0.00678953010866);
    Curve output_curve(3);
    EXPECT_TRUE(
        ComputeTwoArcsToFinalPose(start_pose, final_pose, &output_curve))
        << "Unexpectedly failed from pose " << start_pose << " to pose "
        << final_pose;

    EXPECT_TRUE(output_curve.HasContinuousPosition()) << "Curve:\n"
                                                      << output_curve;

    EXPECT_THAT(
        output_curve,
        IsCurveApprox(
            ToCurve({{0.0, -0.982960815674, -0.000175516398989,
                      -0.0067591112826, -7.28441853031e-07},
                     {0.150020073048, -0.832944169546, -0.00118951924346,
                      -0.0067592205635, -0.000202036597814},
                     {0.300040146084, -0.682927538875, -0.00220580375282,
                      -0.00678953010866, 0}}),
            kEpsilon));
  }
  {
    // Test case for nearly straight path that presents some challenging
    // floating point math edge cases.
    const eigenmath::Pose2d start_pose(
        eigenmath::Vector2d(-0.151496, -1.05394e-06), -1.0383e-08);
    const eigenmath::Pose2d final_pose(
        eigenmath::Vector2d(3.65116e-06, -1.0539e-06), 1.11933e-08);
    Curve output_curve(3);
    EXPECT_TRUE(
        ComputeTwoArcsToFinalPose(start_pose, final_pose, &output_curve))
        << "Unexpectedly failed from pose " << start_pose << " to pose "
        << final_pose;
    EXPECT_TRUE(output_curve.HasContinuousPosition()) << "Curve:\n"
                                                      << output_curve;
    EXPECT_TRUE(output_curve.GetCordLengthSpan().Length() < 1.0)
        << "Curve:\n"
        << output_curve;
  }
}

TEST(CurveTrajectoryUtils, RoundOffCurveTwoPoints) {
  Curve polyline(64);
  polyline.AddPoint(
      0.0, CurvePoint(eigenmath::Pose2d(eigenmath::Vector2d(-4.52132, 0.792893),
                                        -1.24024),
                      0.0));
  polyline.AddPoint(
      2.55239,
      CurvePoint(
          eigenmath::Pose2d(eigenmath::Vector2d(-3.69289, -1.62132), 0.785398),
          0.0));

  Curve rounded_curve(64);
  EXPECT_TRUE(RoundOffCornersOfPolyline(polyline, &rounded_curve));
  EXPECT_GE(rounded_curve.GetSize(), 2);
  EXPECT_TRUE(rounded_curve.HasContinuousPosition());
  EXPECT_TRUE(rounded_curve.HasMonotonicCordLengths());

  EXPECT_THAT(rounded_curve.GetStart().point,
              IsCurvePointApprox(polyline.GetStart().point, kEpsilon));
  EXPECT_THAT(rounded_curve.GetFinish().point,
              IsCurvePointApprox(polyline.GetFinish().point, kEpsilon));
}

TEST(CurveTrajectoryUtils, ConvertCurveToClothoidTrajectory) {
  const DynamicLimits fake_limits(Kinematics(WheelVector(2.0, 2.0), 2.0),
                                  WheelVector(1.0, 1.0), WheelVector(3.0, 3.0),
                                  ArcVector(2.0, 2.0), ArcVector(6.0, 6.0));
  const diff_drive::TrajectoryLimits fake_traj_limits(fake_limits, 0.05);

  // Convert the smooth curve into a clothoid trajectory:
  Trajectory clothoid_traj(128);
  EXPECT_TRUE(ConvertCurveToClothoidTrajectory(
      kTestCurve, 0.5, fake_limits, fake_traj_limits, &clothoid_traj));
  EXPECT_TRUE(clothoid_traj.HasContinuousPosition());
  EXPECT_TRUE(clothoid_traj.HasMonotonicTimeValues());

  if (absl::GetFlag(FLAGS_print_results)) {
    std::cout << "clothoid_traj (data structure) = \n";
    std::cout << clothoid_traj << std::endl;
    std::cout << "clothoid_traj (trajectory trace) = \n";
    PrintTrajectoryTrace(clothoid_traj, 0.01, &std::cout);
  }

  const std::vector<testing::RawStateTime> expected_pts = {
      {0.0, 0.0, 0.0, 0.0, 0.5, 0},
      {1.2453212, 0.1591549, 0.1591549, 1.570796, 0.5, 0.0},
      {3.2453212, 0.1591549, 1.1591549, 1.570796, 0.5, 0.0},
      {5.4383882, 0.4774648, 1.1591549, -1.570796, 0.5, 0.0},
      {7.4383882, 0.4774648, 0.1591549, -1.570796, 0.5, 0.0},
      {8.6837094, 0.6366198, 0.0, 0.0, 0.5, 0.0}};
  EXPECT_THAT(clothoid_traj, TrajectoryEvaluatesTo(expected_pts, kMidEpsilon));
}

TEST(CurveTrajectoryUtils, ConvertCurveToClothoidTrajectoryBackwards) {
  const DynamicLimits fake_limits(Kinematics(WheelVector(2.0, 2.0), 2.0),
                                  WheelVector(1.0, 1.0), WheelVector(3.0, 3.0),
                                  ArcVector(2.0, 2.0), ArcVector(6.0, 6.0));
  const diff_drive::TrajectoryLimits fake_traj_limits(fake_limits, 0.05);

  // Convert the smooth curve into a clothoid trajectory:
  Trajectory clothoid_traj(128);
  EXPECT_TRUE(ConvertCurveToClothoidTrajectory(
      kTestCurve, -0.5, fake_limits, fake_traj_limits, &clothoid_traj));
  EXPECT_TRUE(clothoid_traj.HasContinuousPosition());
  EXPECT_TRUE(clothoid_traj.HasMonotonicTimeValues());

  if (absl::GetFlag(FLAGS_print_results)) {
    std::cout << "clothoid_traj (data structure) = \n";
    std::cout << clothoid_traj << std::endl;
    std::cout << "clothoid_traj (trajectory trace) = \n";
    PrintTrajectoryTrace(clothoid_traj, 0.01, &std::cout);
  }

  const std::vector<testing::RawStateTime> expected_pts = {
      {0.0, 0.0, 0.0, 3.14159, -0.5, 0},
      {1.2453212, 0.1591549, 0.1591549, -1.570796, -0.5, 0.0},
      {3.2453212, 0.1591549, 1.1591549, -1.570796, -0.5, 0.0},
      {5.4383882, 0.4774648, 1.1591549, 1.570796, -0.5, 0.0},
      {7.4383882, 0.4774648, 0.1591549, 1.570796, -0.5, 0.0},
      {8.6837094, 0.6366198, 0.0, 3.14159, -0.5, 0.0}};
  EXPECT_THAT(clothoid_traj, TrajectoryEvaluatesTo(expected_pts, kMidEpsilon));
}

TEST(CurveTrajectoryUtils, ConvertCurveToClothoidTrajectoryRegressions) {
  const diff_drive::DynamicLimits gv_limits(
      diff_drive::Kinematics(WheelVector(0.0754, 0.0754), 0.312),
      WheelVector(12.6, 12.6), WheelVector(120.0, 120.0), ArcVector(0.8, 1.2),
      ArcVector(8.0, 6.0));
  const diff_drive::TrajectoryLimits gv_traj_limits(gv_limits, 0.05);

  {
    const Curve curve_segment =
        ToCurve({{0.0, -1.7, 14.45, 0.00835908543864, -0.109421231988},
                 {0.100000169906, -1.600000746, 14.4502888001,
                  -0.00258305635153, -0.00609823104675},
                 {0.200001334516, -1.5, 14.45, -0.00319288655826, 0.0}});

    Trajectory clothoid_traj(128);
    EXPECT_TRUE(ConvertCurveToClothoidTrajectory(
        curve_segment, 0.6, gv_limits, gv_traj_limits, &clothoid_traj))
        << "Trajectory =\n"
        << clothoid_traj;
    EXPECT_TRUE(clothoid_traj.HasContinuousPosition());
    EXPECT_TRUE(clothoid_traj.HasMonotonicTimeValues());
  }

  {
    const Curve curve_segment =
        ToCurve({{0.0, -2.25, -0.7, 3.13323356815, 0.437684927954},
                 {0.0250000424765, -2.2749998135, -0.699927799974,
                  -3.13900959724, 0.0243929241868},
                 {0.0500003336294, -2.3, -0.7, -3.13839976703, 0.0}});

    Trajectory clothoid_traj(128);
    EXPECT_TRUE(ConvertCurveToClothoidTrajectory(
        curve_segment, 0.15, gv_limits, gv_traj_limits, &clothoid_traj))
        << "Trajectory =\n"
        << clothoid_traj;
    EXPECT_TRUE(clothoid_traj.HasContinuousPosition());
    EXPECT_TRUE(clothoid_traj.HasMonotonicTimeValues());
  }

  {
    const Curve curve_segment =
        ToCurve({{0.0, -3.75, 10.35, 2.34266920583, 4.66419345839e-05},
                 {0.106095344497, -3.82399926352, 10.4260281798, 2.34267415433,
                  0.510008221185},
                 {0.212164593213, -3.9, 10.5, 2.39677034319, 0.0}});

    Trajectory clothoid_traj(128);
    EXPECT_TRUE(ConvertCurveToClothoidTrajectory(
        curve_segment, 0.05, gv_limits, gv_traj_limits, &clothoid_traj))
        << "Trajectory =\n"
        << clothoid_traj;
    EXPECT_TRUE(clothoid_traj.HasContinuousPosition());
    EXPECT_TRUE(clothoid_traj.HasMonotonicTimeValues());
  }

  {
    const Curve curve_segment =
        ToCurve({{0.0, -2.55, 11.5, -1.601039782, 0.00220998080686},
                 {0.0250343067741, -2.55075631635, 11.4749771194,
                  -1.60098445666, 4.8360052126},
                 {0.0500381043552, -2.55, 11.45, -1.48006596123, 0.0}});

    Trajectory clothoid_traj(128);
    EXPECT_TRUE(ConvertCurveToClothoidTrajectory(
        curve_segment, 0.05, gv_limits, gv_traj_limits, &clothoid_traj))
        << "Trajectory =\n"
        << clothoid_traj;
    EXPECT_TRUE(clothoid_traj.HasContinuousPosition());
    EXPECT_TRUE(clothoid_traj.HasMonotonicTimeValues());
  }

  {  // Low velocities should succeed if high velocities succeed.
    const Curve curve_segment =
        ToCurve({{0.0, 10.0, 10.0, 0.0, -1.995012468828},
                 {1.624806609323, 9.95, 9.0, 3.041675862146, 1.995012468828},
                 {3.249613218646, 9.9, 8.0, 0.0, 0.0}});

    // Test for low velocity.
    Trajectory clothoid_traj(128);
    EXPECT_TRUE(ConvertCurveToClothoidTrajectory(
        curve_segment, 0.05, gv_limits, gv_traj_limits, &clothoid_traj))
        << "Trajectory =\n"
        << clothoid_traj;

    EXPECT_TRUE(clothoid_traj.HasContinuousPosition());
    EXPECT_TRUE(clothoid_traj.HasMonotonicTimeValues());

    // Test for high velocity.
    EXPECT_TRUE(ConvertCurveToClothoidTrajectory(
        curve_segment, 0.5, gv_limits, gv_traj_limits, &clothoid_traj))
        << "Trajectory =\n"
        << clothoid_traj;

    EXPECT_TRUE(clothoid_traj.HasContinuousPosition());
    EXPECT_TRUE(clothoid_traj.HasMonotonicTimeValues());
  }

  {  // Curve segment with half a turn.
    const Curve curve_segment =
        ToCurve({{0.0, 0.0, -1.0, 0.0, 1}, {M_PI, 0.0, 1.0, M_PI, 0}});

    Trajectory clothoid_traj(128);
    EXPECT_TRUE(ConvertCurveToClothoidTrajectory(
        curve_segment, 0.3, gv_limits, gv_traj_limits, &clothoid_traj))
        << "Trajectory =\n"
        << clothoid_traj;
    EXPECT_TRUE(clothoid_traj.HasContinuousPosition());
    EXPECT_TRUE(clothoid_traj.HasMonotonicTimeValues());
  }

  {  // Curve segment with three quarter turn.
    const Curve curve_segment = ToCurve(
        {{0.0, 0.0, -1.0, 0.0, 1}, {1.5 * M_PI, -1.0, 0.0, -M_PI / 2, 0}});

    Trajectory clothoid_traj(128);
    EXPECT_TRUE(ConvertCurveToClothoidTrajectory(
        curve_segment, 0.3, gv_limits, gv_traj_limits, &clothoid_traj))
        << "Trajectory =\n"
        << clothoid_traj;
    EXPECT_TRUE(clothoid_traj.HasContinuousPosition());
    EXPECT_TRUE(clothoid_traj.HasMonotonicTimeValues());
  }

  {  // Curve segment with half a turn with reverse motion.
    const Curve curve_segment =
        ToCurve({{0.0, 0.0, -1.0, 0.0, 1}, {M_PI, 0.0, 1.0, M_PI, 0}});

    Trajectory clothoid_traj(128);
    EXPECT_TRUE(ConvertCurveToClothoidTrajectory(
        curve_segment, -0.3, gv_limits, gv_traj_limits, &clothoid_traj))
        << "Trajectory =\n"
        << clothoid_traj;
    EXPECT_TRUE(clothoid_traj.HasContinuousPosition());
    EXPECT_TRUE(clothoid_traj.HasMonotonicTimeValues());
  }

  {  // Curve segment with three quarter turn with reverse motion.
    const Curve curve_segment = ToCurve(
        {{0.0, 0.0, -1.0, 0.0, 1}, {1.5 * M_PI, -1.0, 0.0, -M_PI / 2, 0}});

    Trajectory clothoid_traj(128);
    EXPECT_TRUE(ConvertCurveToClothoidTrajectory(
        curve_segment, -0.3, gv_limits, gv_traj_limits, &clothoid_traj))
        << "Trajectory =\n"
        << clothoid_traj;
    EXPECT_TRUE(clothoid_traj.HasContinuousPosition()) << clothoid_traj;
    EXPECT_TRUE(clothoid_traj.HasMonotonicTimeValues());
  }

  {  // Curve segment is straight but too short.
    const Curve curve_segment =
        ToCurve({{0.0, 0.00, 0.0, 0.0, 0.0}, {0.04, 0.04, 0.0, 0.0, 0.0}});

    Trajectory clothoid_traj(128);
    EXPECT_TRUE(ConvertCurveToClothoidTrajectory(
        curve_segment, 0.8, gv_limits, gv_traj_limits, &clothoid_traj))
        << "Trajectory =\n"
        << clothoid_traj;
    EXPECT_TRUE(clothoid_traj.HasContinuousPosition());
    EXPECT_TRUE(clothoid_traj.HasMonotonicTimeValues());
  }
}

TEST(CurveTrajectoryUtils, FindClothoidTrajectoryForCurveRegressions) {
  const diff_drive::DynamicLimits dd_limits(
      diff_drive::Kinematics(WheelVector(0.0754, 0.0754), 0.312),
      WheelVector(12.6, 12.6), WheelVector(120.0, 120.0), ArcVector(0.48, 0.96),
      ArcVector(8.0, 6.0));
  const diff_drive::TrajectoryLimits dd_traj_limits(dd_limits, 0.05);

  const diff_drive::DynamicLimits dd_medium_fw_limits(
      diff_drive::Kinematics(WheelVector(0.0754, 0.0754), 0.312),
      WheelVector(-4.0, -4.0), WheelVector(4.0, 4.0),
      WheelVector(-120.0, -120.0), WheelVector(120.0, 120.0),
      ArcVector(-0.06, -0.9), ArcVector(0.3, 0.9), ArcVector(-4.0, -8.0),
      ArcVector(4.0, 8.0));
  const diff_drive::TrajectoryLimits dd_medium_traj_limits(dd_medium_fw_limits,
                                                           0.05);

  {
    const Curve curve_segment =
        ToCurve({{0.0, -0.635279067271, -0.00022314018254, -0.000695474978576,
                  -1.52889266131e-06},
                 {0.476454367543, -0.158978646025, -0.000554568741239,
                  -0.000696203426162, 1.27856397544e-06},
                 {0.952984839769, 0.317321775201, -0.000886025680988,
                  -0.000695594151467, 0}});

    const double desired_speed = 0.48;
    const double tolerance = 0.01;

    double sub_desired_speed;
    Trajectory clothoid_traj(128);
    EXPECT_TRUE(FindClothoidTrajectoryForCurve(
        curve_segment, desired_speed, tolerance, dd_limits, dd_traj_limits,
        &clothoid_traj, &sub_desired_speed))
        << "Trajectory =\n"
        << clothoid_traj;

    EXPECT_TRUE(clothoid_traj.HasContinuousPosition());
    EXPECT_TRUE(clothoid_traj.HasMonotonicTimeValues());
    EXPECT_THAT(clothoid_traj, SatisfiesLimits(dd_limits));
    EXPECT_LT(clothoid_traj.GetTimeSpan().Length(), 2.5) << "Trajectory =\n"
                                                         << clothoid_traj;
  }

  {  // Regression for b/134529166.
    const Curve curve_segment =
        ToCurve({{0.0, -0.985545656383, -3.15486276897e-05, -0.00134079470748,
                  -4.44936756248e-06},
                 {0.150000171205, -0.835545620075, -0.000232718058557,
                  -0.00134146211337, -8.82999819031e-07},
                 {0.300000342411, -0.685545583847, -0.000433947478641,
                  -0.0013415945635, 0}});

    const double desired_speed = 0.48;
    const double tolerance = 0.01;

    double sub_desired_speed;
    Trajectory clothoid_traj(128);
    EXPECT_TRUE(FindClothoidTrajectoryForCurve(
        curve_segment, desired_speed, tolerance, dd_limits, dd_traj_limits,
        &clothoid_traj, &sub_desired_speed))
        << "Trajectory =\n"
        << clothoid_traj;

    EXPECT_TRUE(clothoid_traj.HasContinuousPosition());
    EXPECT_TRUE(clothoid_traj.HasMonotonicTimeValues());
    EXPECT_THAT(clothoid_traj, SatisfiesLimits(dd_limits));
    EXPECT_LT(clothoid_traj.GetTimeSpan().Length(), 1.0) << "Trajectory =\n"
                                                         << clothoid_traj;
  }

  {  // A tight curve for which the slowest test speed failed.
    const Curve curve_segment =
        ToCurve({{0.0, 10.0, 10.0, 0.0, -1.995012468828},
                 {1.624806609323, 9.95, 9.0, 3.041675862146, 1.995012468828},
                 {3.249613218646, 9.9, 8.0, 0.0, 0.0}});

    const double desired_speed = 0.48;
    const double tolerance = 0.01;

    double sub_desired_speed;
    Trajectory clothoid_traj(128);
    EXPECT_TRUE(FindClothoidTrajectoryForCurve(
        curve_segment, desired_speed, tolerance, dd_limits, dd_traj_limits,
        &clothoid_traj, &sub_desired_speed))
        << "Trajectory =\n"
        << clothoid_traj;

    EXPECT_TRUE(clothoid_traj.HasContinuousPosition());
    EXPECT_TRUE(clothoid_traj.HasMonotonicTimeValues());
    EXPECT_THAT(clothoid_traj, SatisfiesLimits(dd_limits));
    EXPECT_LT(clothoid_traj.GetTimeSpan().Length(), 8.0) << "Trajectory =\n"
                                                         << clothoid_traj;
  }

  {  // A small straight segment with a bit of angle round-off error.
    const Curve curve_segment =
        ToCurve({{0.0, -55.1, -51.40000000000001, 0.0, 0.0},
                 {0.100000000000001, -55, -51.40000000000001,
                  6.805931991089454e-07, 0.0}});

    const double desired_speed = 0.3;
    const double tolerance = 0.001;

    double sub_desired_speed;
    Trajectory clothoid_traj(256);
    EXPECT_TRUE(FindClothoidTrajectoryForCurve(
        curve_segment, desired_speed, tolerance, dd_limits, dd_traj_limits,
        &clothoid_traj, &sub_desired_speed))
        << "Trajectory =\n"
        << clothoid_traj;

    EXPECT_TRUE(clothoid_traj.HasContinuousPosition());
    EXPECT_TRUE(clothoid_traj.HasMonotonicTimeValues());
    EXPECT_THAT(clothoid_traj, SatisfiesLimits(dd_limits));
    EXPECT_LT(clothoid_traj.GetTimeSpan().Length(), 80.0) << "Trajectory =\n"
                                                          << clothoid_traj;
  }

  {  // A curve whose conversion regressed in cl/288953117 causing b/147482628.
    const eigenmath::Pose2d first_pose(eigenmath::Vector2d(10.85, -3.9),
                                       -1.5708);
    const eigenmath::Pose2d second_pose(eigenmath::Vector2d(10.85, -4.05),
                                        -1.44268);

    Curve curve(5);
    EXPECT_TRUE(
        diff_drive::ComputeTwoArcsToFinalPose(first_pose, second_pose, &curve))
        << "It should be possible to generate a two-arc curve!";

    const double desired_speed = 0.5;
    const double tolerance = 0.01;

    double sub_desired_speed;
    Trajectory clothoid_traj(128);
    EXPECT_TRUE(FindClothoidTrajectoryForCurve(
        curve, desired_speed, tolerance, dd_limits, dd_traj_limits,
        &clothoid_traj, &sub_desired_speed))
        << "Trajectory =\n"
        << clothoid_traj;

    EXPECT_TRUE(clothoid_traj.HasContinuousPosition());
    EXPECT_TRUE(clothoid_traj.HasMonotonicTimeValues());
    EXPECT_THAT(clothoid_traj, SatisfiesLimits(dd_limits));
    EXPECT_LT(clothoid_traj.GetTimeSpan().Length(), 1.0) << "Trajectory =\n"
                                                         << clothoid_traj;
  }

  {  // Examples taken from b/145025677.
    const Curve curves[] = {
        ToCurve({{0.0, 7.15564872711e-05, -0.000330802515018, 0.000942534654414,
                  172.257050325},
                 {0.012345156686, 0.00499482676426, 0.00854184128627,
                  2.12748281119, -251.878473553},
                 {0.0219320690855, 0.00949036937678, 0.0144470754467,
                  -0.287254050085, 0.0}}),
        ToCurve({{0.0, 7.15564872711e-05, -0.000330802515018, 0.000942534654414,
                  161.184230204},
                 {0.0112498292978, 0.00608685765652, 0.00736873121479,
                  1.81423760995, -202.241424191},
                 {0.0212677112223, 0.0119250656129, 0.0133946945894,
                  -0.21179309783, 0.0}}),
        ToCurve({{0.0, 7.15564872711e-05, -0.000330802515018, 0.000942534654414,
                  83.0859357576},
                 {0.0223593347334, 0.0116006060221, 0.015122253355,
                  1.8586887839, -113.935263514},
                 {0.0411717302823, 0.0224815230067, 0.0260378922475,
                  -0.284706460307, 0.0}}),
        ToCurve({{0.0, 0.33705532353, -0.0383991091803, -0.072650143995,
                  65.2711558899},
                 {0.035390201, 0.350209198381, -0.0136468118686, 2.23730918245,
                  -45.9236454331},
                 {0.0793907910316, 0.36264356261, 0.0210820022472,
                  0.216641686991, 0.0}}),
        ToCurve({{0.0, 0.000464468720521, 0.0114748229469, -0.251013612986,
                  -30.8232120798},
                 {0.0654986900305, 0.0172387580249, -0.0408295296869,
                  -2.26989362674, 35.3438262706},
                 {0.126000069427, 0.0351841569605, -0.087086197734,
                  -0.131543384214, 0.0}}),
        ToCurve({{0.0, 0.0351841569605, -0.087086197734, -0.131543384214,
                  34.1179513632},
                 {0.0601267908069, 0.0665710904809, -0.0480047684691,
                  1.91985954016, -39.6745820202},
                 {0.115080671677, 0.0967458984859, -0.0150289589242,
                  -0.260412713769, 0.0}}),
        ToCurve({{0.0, 0.0351841569605, -0.087086197734, -0.131543384214,
                  28.2948210529},
                 {0.0707112516829, 0.0735998737241, -0.041658328333,
                  1.86921882858, -30.7503608003},
                 {0.138191568881, 0.11132875023, -0.000263514819831,
                  -0.205825272195, 0.0}}),
        ToCurve({{0.0, 0.0351841569605, -0.087086197734, -0.131543384214,
                  41.9601778164},
                 {0.0499447343565, 0.0603221463337, -0.0543254794308,
                  1.96414655038, -47.3484775654},
                 {0.0963445620952, 0.0847019651597, -0.025680296752,
                  -0.232814652344, 0.0}}),
        ToCurve({{0.0, 0.0351841569605, -0.087086197734, -0.131543384214,
                  112.604977929},
                 {0.0187886564244, 0.0444816328122, -0.0747151100219,
                  1.98415285778, -131.076135653},
                 {0.0358800949331, 0.0534009786019, -0.0642703112653,
                  -0.256126854688, 0.0}}),
        ToCurve({{0.0, 0.0351841569605, -0.087086197734, -0.131543384214,
                  33.2451248696},
                 {0.0569143042232, 0.0686690502907, -0.0515920652967,
                  1.76057976655, -38.16206893},
                 {0.109842124287, 0.101120277531, -0.0213204525457,
                  -0.259255351022, 0.0}}),
        ToCurve({{0.0, 0.0351841569605, -0.087086197734, -0.131543384214,
                  61.1730469159},
                 {0.0336620644904, 0.0526454211991, -0.0651696002208,
                  1.92766766615, -73.0192074382},
                 {0.0639094878849, 0.0692751161145, -0.0472273363849,
                  -0.280975217165, 0.0}}),
        ToCurve({{0.0, 0.0364266692271, -0.0465225667022, -0.265368106289,
                  -80.9916833712},
                 {0.0336612582021, 0.0350329070009, -0.0706457309655,
                  -2.99165007247, 120.443070279},
                 {0.0574078023714, 0.0351841569605, -0.087086197734,
                  -0.131543384214, 0.0}}),
        ToCurve({{0.0, -0.0218186932843, 0.0344191371265, -0.204543351537,
                  -23.5396817081},
                 {0.0803212463457, 0.00632368390599, -0.0284500189806,
                  -2.09527992492, 25.4472026011},
                 {0.157490299914, 0.0351841569605, -0.087086197734,
                  -0.131543384214, 0.0}}),
        ToCurve({{0.0, 0.0351841569605, -0.087086197734, -0.131543384214,
                  44.4817458008},
                 {0.0435852240322, 0.059988715853, -0.0595339549258,
                  1.80720347186, -52.4469592648},
                 {0.0833858339495, 0.0837984928015, -0.0367450926237,
                  -0.280217495185, 0.0}}),
        ToCurve({{0.0, 0.0157230637188, -7.93299569706e-05, -0.216653734399,
                  -39.0289591359},
                 {0.0591501068471, 0.0250268129043, -0.0460094156093,
                  -2.52522083742, 43.9991095125},
                 {0.113552968172, 0.0351841569605, -0.087086197734,
                  -0.131543384214, 0.0}}),
        ToCurve({{0.0, 0.0180255195588, 0.00290069186211, -0.217050352483,
                  -38.3038165653},
                 {0.0619001025127, 0.0261277164738, -0.0448023851801,
                  -2.5880605245, 43.56356571},
                 {0.118289360377, 0.0351841569605, -0.087086197734,
                  -0.131543384214, 0.0}}),
        ToCurve({{0.0, 0.0351841569605, -0.087086197734, -0.131543384214,
                  28.0952325821},
                 {0.0675680534561, 0.0747644546895, -0.0448687654099,
                  1.76679679276, -32.2753291642},
                 {0.130353674849, 0.11310879183, -0.00888979773758,
                  -0.259629804464, 0.0}}),
        ToCurve({{0.0, 0.035184156960465174, -0.08708619773395287,
                  -0.131543384213569758, 28.2810765505},
                 {0.0710276230474, 0.0735345562965, -0.0413670423374,
                  1.87719426039, -32.5382303058},
                 {0.136541615972, 0.11057413734352578, -0.002354109784605783,
                  -0.254515129651337713, 0.0}}),
    };

    const double desired_speed = 0.15;
    const double tolerance = 0.01;

    eigenmath::TestGenerator rng_gen(eigenmath::kGeneratorTestSeed);
    std::uniform_real_distribution<double> noise_dist(-1e-11, 1e-11);

    for (const Curve& curve : curves) {
      for (int i = 0; i < 10000; ++i) {
        Curve perturbed_curve = curve;
        for (auto& pt : perturbed_curve.GetCurvePointIteratorRange()) {
          pt.cord_length += noise_dist(rng_gen);
          eigenmath::Pose2d nominal_pose = pt.point.GetPose();
          nominal_pose.translation().x() += noise_dist(rng_gen);
          nominal_pose.translation().y() += noise_dist(rng_gen);
          nominal_pose.so2() =
              nominal_pose.so2() * eigenmath::SO2d(noise_dist(rng_gen));
          pt.point.SetPose(nominal_pose);
          pt.point.SetCurvature(pt.point.GetCurvature() +
                                10.0 * noise_dist(rng_gen));
        }
        double sub_desired_speed;
        Trajectory clothoid_traj(128);
        EXPECT_TRUE(FindClothoidTrajectoryForCurve(
            perturbed_curve, desired_speed, tolerance, dd_medium_fw_limits,
            dd_medium_traj_limits, &clothoid_traj, &sub_desired_speed))
            << "Curve =\n"
            << perturbed_curve << "Trajectory =\n"
            << clothoid_traj;

        EXPECT_TRUE(clothoid_traj.HasContinuousPosition());
        EXPECT_TRUE(clothoid_traj.HasMonotonicTimeValues());
        EXPECT_THAT(clothoid_traj, SatisfiesLimits(dd_medium_fw_limits));
        EXPECT_LT(clothoid_traj.GetTimeSpan().Length(), 20.0)
            << "Trajectory =\n"
            << clothoid_traj;
      }
    }
  }
}

TEST(CurveTrajectoryUtils, AppendClothoidFromCurveToTrajectory) {
  const DynamicLimits fake_limits(Kinematics(WheelVector(2.0, 2.0), 2.0),
                                  WheelVector(1.0, 1.0), WheelVector(3.0, 3.0),
                                  ArcVector(2.0, 2.0), ArcVector(6.0, 6.0));
  diff_drive::TrajectoryLimits fake_traj_limits(fake_limits, 0.05);
  fake_traj_limits.ScaleArcLimitsBy(0.2, 0.3);

  // Convert the smooth curve into a clothoid trajectory:
  Trajectory traj_segment(128);
  Trajectory traj_scratch(512);
  Trajectory combined_traj(512);
  EXPECT_TRUE(AppendClothoidFromCurveToTrajectory(
      0.5, fake_limits, fake_traj_limits, kTestCurve, /*going_backwards=*/false,
      &traj_segment, &traj_scratch, &combined_traj));
  EXPECT_TRUE(combined_traj.HasContinuousPosition());
  EXPECT_TRUE(combined_traj.HasMonotonicTimeValues());

  // The kTestCurve ends around (0.64, 0) and 0 radians.
  // Append a pure-spin to turn around by pi radians.
  Curve turn_back_around(3);
  EXPECT_TRUE(turn_back_around.AddPoint(
      0.0, CurvePoint(
               eigenmath::Pose2d(
                   kTestCurve.GetFinish().point.GetPose().translation(), M_PI),
               0.0)));
  EXPECT_TRUE(AppendClothoidFromCurveToTrajectory(
      0.5, fake_limits, fake_traj_limits, turn_back_around,
      /*going_backwards=*/true, &traj_segment, &traj_scratch, &combined_traj));
  EXPECT_TRUE(combined_traj.HasContinuousPosition());
  EXPECT_TRUE(combined_traj.HasMonotonicTimeValues());

  // Now, we are around (0.64, 0) and pi radians.
  // Append a quarter-circle that goes to around (0.32, -0.32) at -pi/2 rad.
  Curve arc_down(3);
  EXPECT_TRUE(arc_down.AddPoint(
      0.0,
      CurvePoint(
          eigenmath::Pose2d(
              kTestCurve.GetFinish().point.GetPose().translation(), M_PI),
          2.0 / kTestCurve.GetFinish().point.GetPose().translation().x())));
  EXPECT_TRUE(arc_down.AddPoint(
      kTestCurve.GetFinish().point.GetPose().translation().x() * M_PI / 4.0,
      CurvePoint(
          eigenmath::Pose2d(
              eigenmath::Vector2d(
                  kTestCurve.GetFinish().point.GetPose().translation().x() /
                      2.0,
                  -kTestCurve.GetFinish().point.GetPose().translation().x() /
                      2.0),
              -M_PI / 2.0),
          0.0)));
  EXPECT_TRUE(AppendClothoidFromCurveToTrajectory(
      0.5, fake_limits, fake_traj_limits, arc_down, /*going_backwards=*/false,
      &traj_segment, &traj_scratch, &combined_traj));
  EXPECT_TRUE(combined_traj.HasContinuousPosition());
  EXPECT_TRUE(combined_traj.HasMonotonicTimeValues());

  // Now, we are around (0.32, -0.32) at -pi/2 rad.
  // Append a backwards quarter-circle that reverses to the start (origin).
  Curve arc_back_to_start(3);
  EXPECT_TRUE(arc_back_to_start.AddPoint(
      0.0,
      CurvePoint(
          eigenmath::Pose2d::Identity(),
          -2.0 / kTestCurve.GetFinish().point.GetPose().translation().x())));
  EXPECT_TRUE(arc_back_to_start.AddPoint(
      kTestCurve.GetFinish().point.GetPose().translation().x() * M_PI / 4.0,
      arc_down.GetFinish().point));
  EXPECT_TRUE(AppendClothoidFromCurveToTrajectory(
      0.5, fake_limits, fake_traj_limits, arc_back_to_start,
      /*going_backwards=*/true, &traj_segment, &traj_scratch, &combined_traj));
  EXPECT_TRUE(combined_traj.HasContinuousPosition());
  EXPECT_TRUE(combined_traj.HasMonotonicTimeValues());

  if (absl::GetFlag(FLAGS_print_results)) {
    std::cout << "combined_traj (data structure) = \n";
    std::cout << combined_traj << std::endl;
    std::cout << "combined_traj (trajectory trace) = \n";
    PrintTrajectoryTrace(combined_traj, 0.01, &std::cout);
  }

  const std::vector<testing::RawStateTime> expected_pts = {
      {0.0, 0.0, 0.0, 0.0, 0.06, 0},
      {1.9977071, 0.1591549, 0.1591549, 1.570796, 0.1364844, 0.0},
      {9.3245531, 0.1591549, 1.1591549, 1.570796, 0.1364844, 0.0},
      {13.4929605, 0.4774648, 1.1591549, -1.570796, 0.1364844, 0.0},
      {20.8198065, 0.4774648, 0.1591549, -1.570796, 0.1364844, 0.0},
      {22.8175136, 0.6366198, 0.0, 0.0, 0.0, 0.3},
      {24.8128429, 0.6366198, 0.0, 3.1415926, 0.06, 0.0},
      {26.9265666, 0.3183099, -0.3183099, -1.570796, -0.06, 0.0},
      {28.9436749, 0.0, 0.0, 0.0, -0.2795703, 0.0}};
  EXPECT_THAT(combined_traj, TrajectoryEvaluatesTo(expected_pts, kMidEpsilon));
}

TEST(CurveTrajectoryUtils, AmendTrajectoryToEndWithStop) {
  Kinematics kinematics({2.0, 2.0}, 2.0);

  const WheelVector max_wheel_velocity = {1.0, 1.0};
  const WheelVector max_wheel_acceleration = {3.0, 3.0};
  const ArcVector max_arc_velocity = {1.5, 1.5};
  const ArcVector max_arc_acceleration = {6.0, 6.0};

  DynamicLimits limits(kinematics, max_wheel_velocity, max_wheel_acceleration,
                       max_arc_velocity, max_arc_acceleration);

  TrajectoryLimits traj_limits(limits, 0.05);

  Trajectory clothoid_traj(128);
  EXPECT_TRUE(ConvertCurveToClothoidTrajectory(kTestCurve, 0.5, limits,
                                               traj_limits, &clothoid_traj));
  EXPECT_TRUE(clothoid_traj.HasContinuousPosition());
  EXPECT_TRUE(clothoid_traj.HasMonotonicTimeValues());

  Trajectory amended_traj = clothoid_traj;
  EXPECT_TRUE(
      AmendTrajectoryToEndWithStop(kinematics, traj_limits, &amended_traj));
  EXPECT_TRUE(amended_traj.HasContinuousPosition());
  EXPECT_TRUE(amended_traj.HasMonotonicTimeValues());

  if (absl::GetFlag(FLAGS_print_results)) {
    std::cout << "clothoid_traj (data structure) = \n";
    std::cout << clothoid_traj << std::endl;
    std::cout << "amended_traj (data structure) = \n";
    std::cout << amended_traj << std::endl;
  }

  const std::vector<testing::RawStateTime> expected_pts = {
      {0.0, 0.0, 0.0, 0.0, 0.5, 0},
      {1.4095684, 0.1591549, 0.1591549, 1.570796, 0.5, 0.0},
      {3.4095684, 0.1591549, 1.1591549, 1.570796, 0.5, 0.0},
      {5.8486442, 0.4774648, 1.1591549, -1.570796, 0.5, 0.0},
      {7.8486442, 0.4774648, 0.1591549, -1.570796, 0.5, 0.0},
      {9.1825110, 0.60005, 0.00004, -0.00643, 0.385672, 0.214328},
      {9.2125110, 0.6116199, 0.0, 0.0, 0.4713447, 0.0},
      {9.2837200, 0.6366180, 0.0, 0.0, 0.3, 0.0},
      {9.2837300, 0.6366198, 0.0, 0.0, 0.0, 0.0}};
  EXPECT_THAT(amended_traj, TrajectoryEvaluatesTo(expected_pts, kMidEpsilon));
}

TEST(CurveTrajectoryUtils, AmendTrajectoryToEndWithStopPureSpin) {
  Kinematics kinematics({2.0, 2.0}, 2.0);

  const WheelVector max_wheel_velocity = {1.0, 1.0};
  const WheelVector max_wheel_acceleration = {3.0, 3.0};
  const ArcVector max_arc_velocity = {1.5, 1.5};
  const ArcVector max_arc_acceleration = {6.0, 6.0};

  DynamicLimits limits(kinematics, max_wheel_velocity, max_wheel_acceleration,
                       max_arc_velocity, max_arc_acceleration);

  TrajectoryLimits traj_limits(limits, 0.05);

  Trajectory pure_spin_traj(128);
  State spin_state{eigenmath::Pose2d::Identity(), ArcVector{0.0, 1.0}};
  for (int i = 0; i < 8; ++i) {
    pure_spin_traj.AddState(i * 0.1,
                            spin_state.ExtrapolateConstantVelocityArc(i * 0.1));
  }

  Trajectory amended_traj(128);
  amended_traj = pure_spin_traj;
  EXPECT_TRUE(
      AmendTrajectoryToEndWithStop(kinematics, traj_limits, &amended_traj));
  EXPECT_TRUE(amended_traj.HasContinuousPosition());
  EXPECT_TRUE(amended_traj.HasMonotonicTimeValues());

  if (absl::GetFlag(FLAGS_print_results)) {
    std::cout << "pure_spin_traj (data structure) = \n";
    std::cout << pure_spin_traj << std::endl;
    std::cout << "amended_traj (data structure) = \n";
    std::cout << amended_traj << std::endl;
  }

  const std::vector<testing::RawStateTime> expected_pts = {
      {0.0, 0.0, 0.0, 0.0, 0.0, 1.0},
      {0.1, 0.0, 0.0, 0.1, 0.0, 1.0},
      {0.2, 0.0, 0.0, 0.2, 0.0, 1.0},
      {0.3, 0.0, 0.0, 0.3, 0.0, 1.0},
      {0.4, 0.0, 0.0, 0.4, 0.0, 1.0},
      {0.5, 0.0, 0.0, 0.5, 0.0, 1.0},
      {0.56, 0.0, 0.0, 0.56, 0.0, 1.0},
      {0.6, 0.0, 0.0, 0.6, 0.0, 1.0},
      {0.6100001, 0.0, 0.0, 0.61000009, 0.0, 0.9},
      {0.6600001, 0.0, 0.0, 0.65500006, 0.0, 0.6},
      {0.7100001, 0.0, 0.0, 0.68500003, 0.0, 0.3},
      {0.7600001, 0.0, 0.0, 0.7, 0.0, 0.0}};
  EXPECT_THAT(amended_traj, TrajectoryEvaluatesTo(expected_pts, kMidEpsilon));
}

TEST(CurveTrajectoryUtils, AmendTrajectoryToEndWithSlowdown) {
  Kinematics kinematics({2.0, 2.0}, 2.0);

  const WheelVector max_wheel_velocity = {1.0, 1.0};
  const WheelVector max_wheel_acceleration = {3.0, 3.0};
  const ArcVector max_arc_velocity = {1.5, 1.5};
  const ArcVector max_arc_acceleration = {6.0, 6.0};

  DynamicLimits limits(kinematics, max_wheel_velocity, max_wheel_acceleration,
                       max_arc_velocity, max_arc_acceleration);

  TrajectoryLimits traj_limits(limits, 0.05);

  Trajectory clothoid_traj(128);
  EXPECT_TRUE(ConvertCurveToClothoidTrajectory(kTestCurve, 0.5, limits,
                                               traj_limits, &clothoid_traj));
  EXPECT_TRUE(clothoid_traj.HasContinuousPosition());
  EXPECT_TRUE(clothoid_traj.HasMonotonicTimeValues());

  Trajectory amended_traj = clothoid_traj;
  EXPECT_TRUE(AmendTrajectoryToEndWithSlowdown(kinematics, 0.2, traj_limits,
                                               &amended_traj));
  EXPECT_TRUE(amended_traj.HasContinuousPosition());
  EXPECT_TRUE(amended_traj.HasMonotonicTimeValues());

  if (absl::GetFlag(FLAGS_print_results)) {
    std::cout << "clothoid_traj (data structure) = \n";
    std::cout << clothoid_traj << std::endl;
    std::cout << "amended_traj (data structure) = \n";
    std::cout << amended_traj << std::endl;
  }

  const std::vector<testing::RawStateTime> expected_pts = {
      {0.0, 0.0, 0.0, 0.0, 0.5, 0},
      {1.4095684, 0.1591549, 0.1591549, 1.570796, 0.5, 0.0},
      {3.4095684, 0.1591549, 1.1591549, 1.570796, 0.5, 0.0},
      {5.8486442, 0.4774648, 1.1591549, -1.570796, 0.5, 0.0},
      {7.8486442, 0.4774648, 0.1591549, -1.570796, 0.5, 0.0},
      {9.1611534, 0.5891118, 0.0001408, -0.0125086, 0.4045226, 0.2248032},
      {9.2167957, 0.6116199, 0.0, 0.0, 0.3569682, 0.0},
      {9.2868200, 0.6366180, 0.0, 0.0, 0.3569682, 0.0},
      {9.2868300, 0.6366198, 0.0, 0.0, 0.2, 0.0}};
  EXPECT_THAT(amended_traj, TrajectoryEvaluatesTo(expected_pts, kMidEpsilon));
}

TEST(CurveTrajectoryUtils, AmendTrajectoryToEndWithStopWithStationaryPoints) {
  Kinematics kinematics({2.0, 2.0}, 2.0);

  const WheelVector max_wheel_velocity = {1.0, 1.0};
  const WheelVector max_wheel_acceleration = {3.0, 3.0};
  const ArcVector max_arc_velocity = {1.5, 1.5};
  const ArcVector max_arc_acceleration = {6.0, 6.0};

  DynamicLimits limits(kinematics, max_wheel_velocity, max_wheel_acceleration,
                       max_arc_velocity, max_arc_acceleration);

  TrajectoryLimits traj_limits(limits, 0.05);

  Trajectory clothoid_traj(128);
  EXPECT_TRUE(ConvertCurveToClothoidTrajectory(kTestCurve, 0.5, limits,
                                               traj_limits, &clothoid_traj));
  EXPECT_TRUE(clothoid_traj.HasContinuousPosition());
  EXPECT_TRUE(clothoid_traj.HasMonotonicTimeValues());

  auto clothoid_finish = clothoid_traj.GetFinish();
  clothoid_finish.state =
      clothoid_finish.state.ExtrapolateConstantVelocityArc(0.01);
  clothoid_finish.state.SetArcVelocity(ArcVector(0.0, 0.0));
  clothoid_finish.time += 0.01;
  clothoid_traj.AddState(clothoid_finish.time, clothoid_finish.state);
  clothoid_finish.time += 0.01;
  clothoid_finish.state.SetArcVelocity(ArcVector(-0.5, 0.0));
  clothoid_traj.AddState(clothoid_finish.time, clothoid_finish.state);
  clothoid_finish.state =
      clothoid_finish.state.ExtrapolateConstantVelocityArc(0.05);
  clothoid_finish.time += 0.05;
  clothoid_traj.AddState(clothoid_finish.time, clothoid_finish.state);

  Trajectory amended_traj = clothoid_traj;
  EXPECT_TRUE(
      AmendTrajectoryToEndWithStop(kinematics, traj_limits, &amended_traj));
  EXPECT_TRUE(amended_traj.HasContinuousPosition());
  EXPECT_TRUE(amended_traj.HasMonotonicTimeValues());

  if (absl::GetFlag(FLAGS_print_results)) {
    std::cout << "clothoid_traj (data structure) = \n";
    std::cout << clothoid_traj << std::endl;
    std::cout << "amended_traj (data structure) = \n";
    std::cout << amended_traj << std::endl;
  }

  const std::vector<testing::RawStateTime> expected_pts = {
      {0.0, 0.0, 0.0, 0.0, 0.5, 0},
      {1.4095684, 0.1591549, 0.1591549, 1.570796, 0.5, 0.0},
      {3.4095684, 0.1591549, 1.1591549, 1.570796, 0.5, 0.0},
      {5.8486442, 0.4774648, 1.1591549, -1.570796, 0.5, 0.0},
      {7.8486442, 0.4774648, 0.1591549, -1.570796, 0.5, 0.0},
      {9.2429030, 0.62661977, 0.0, 0.0, 0.3, 0.0},
      {9.2762360, 0.63661977, 0.0, 0.0, 0.3, 0.0},
      {9.2929030, 0.64161977, 0.0, 0.0, 0.0, 0.0},
      {9.3229030, 0.64161977, 0.0, 0.0, -0.3, 0.0},
      {9.356236, 0.63161977, 0.0, 0.0, -0.3, 0.0},
      {9.406236, 0.61661977, 0.0, 0.0, 0.0, 0.0}};
  EXPECT_THAT(amended_traj, TrajectoryEvaluatesTo(expected_pts, kMidEpsilon));
}

TEST(CurveTrajectoryUtils, AmendTrajectoryToEndWithStopRegression1) {
  const diff_drive::DynamicLimits gv_limits(
      diff_drive::Kinematics(WheelVector(0.0754, 0.0754), 0.312),
      WheelVector(10.6, 10.6), WheelVector(120.0, 120.0), ArcVector(0.8, 1.2),
      ArcVector(8.0, 6.0));
  const diff_drive::TrajectoryLimits gv_traj_limits(gv_limits, 0.05);

  Trajectory clothoid_traj = ToTrajectory(
      {{0.0, -1.7, -0.8, 0.392699081699, -0.253125, 0.0},
       {0.05, -1.71169285033, -0.804843337191, 0.392699081699, -0.22829482511,
        -0.3},
       {0.1, -1.72227106181, -0.809132313387, 0.377699081699, -0.183791951338,
        -0.6},
       {0.15, -1.73086248586, -0.812392651231, 0.347699081699, -0.114059520317,
        -0.9},
       {0.2, -1.73626609663, -0.81421458373, 0.302699081699, -0.00261262570278,
        -1.2},
       {0.25, -1.73639188188, -0.814249761607, 0.242699081699,
        -0.00261262570277, -1.2},
       {0.342303421426, -1.73662869666, -0.814294647257, 0.131934975988,
        -0.00261262570278, -1.2},
       {0.392303421426, -1.73675863036, -0.814307938111, 0.071934975988,
        -0.114059520317, -0.9},
       {0.442303421426, -1.74245415869, -0.814589726001, 0.026934975988,
        -0.183791951338, -0.6},
       {0.492303421426, -1.75164275719, -0.814699396911, -0.00306502401198,
        -0.22829482511, -0.3},
       {0.542303421426, -1.76305675439, -0.814578803269, -0.018065024012,
        -0.253125, -0},
       {0.592303421426, -1.77571093929, -0.814350180245, -0.018065024012,
        -0.253125, -0},
       {0.642303421426, -1.78836512419, -0.81412155722, -0.018065024012,
        -0.1950919029, 0.3},
       {0.692303421426, -1.7981190835, -0.814018502572, -0.00306502401198,
        -0.134517482166, 0.6},
       {0.742303421426, -1.80484422638, -0.814098770802, 0.026934975988,
        -0.0712340140049, 0.9},
       {0.792303421426, -1.80840127575, -0.814274756834, 0.071934975988,
        -0.00493691304554, 1.2},
       {0.842303421426, -1.80864680323, -0.814299871719, 0.131934975988,
        -0.00493691304554, 1.2},
       {2.24360378185, -1.81209910552, -0.819366924576, 1.81349540849,
        -0.00493691304554, 1.2},
       {2.29360378185, -1.81203263212, -0.819604612997, 1.87349540849,
        -0.0712340140049, 0.9},
       {2.34360378185, -1.81089477388, -0.822979350014, 1.91849540849,
        -0.134517482166, 0.6},
       {2.39360378185, -1.8085085299, -0.829267420423, 1.94849540849,
        -0.1950919029, 0.3},
       {2.44360378185, -1.80484333719, -0.838307149667, 1.96349540849,
        -0.253125, -0},
       {2.49360378185, -1.8, -0.85, 1.96349540849, -0.253125, -0}});

  Trajectory amended_traj(128);
  amended_traj = clothoid_traj;
  EXPECT_TRUE(AmendTrajectoryToEndWithStop(gv_limits.GetKinematics(),
                                           gv_traj_limits, &amended_traj));
  EXPECT_TRUE(amended_traj.HasContinuousPosition());
  EXPECT_TRUE(amended_traj.HasMonotonicTimeValues());

  if (absl::GetFlag(FLAGS_print_results)) {
    std::cout << "clothoid_traj (data structure) = \n";
    std::cout << clothoid_traj << std::endl;
    std::cout << "amended_traj (data structure) = \n";
    std::cout << amended_traj << std::endl;
  }

  auto last_pt = std::prev(amended_traj.EndState());
  auto second_last_pt = std::prev(last_pt);
  auto third_last_pt = std::prev(second_last_pt);
  auto fourth_last_pt = std::prev(third_last_pt);

  EXPECT_THAT(fourth_last_pt->state,
              IsStateApprox(ToState({0.0, -1.81089477388, -0.822979350014,
                                     1.91849540849, -0.134517482166, 0.6}),
                            kMidEpsilon));

  EXPECT_THAT(third_last_pt->state,
              IsStateApprox(ToState({0.0, -1.8085085299, -0.829267420423,
                                     1.94849540849, -0.1950919029, 0.3}),
                            kMidEpsilon));

  EXPECT_THAT(second_last_pt->state,
              IsStateApprox(ToState({0.0, -1.80484333719, -0.838307149667,
                                     1.96349540849, -0.253125, 0.0}),
                            kMidEpsilon));

  EXPECT_THAT(
      last_pt->state,
      IsStateApprox(ToState({0.0, -1.8, -0.85, 1.96349540849, 0.0, 0.0}),
                    kMidEpsilon));
}

TEST(CurveTrajectoryUtils, AmendTrajectoryToEndWithStopRegression2) {
  const diff_drive::DynamicLimits gv_limits(
      diff_drive::Kinematics(WheelVector(0.0754, 0.0754), 0.312),
      WheelVector(10.6, 10.6), WheelVector(120.0, 120.0), ArcVector(0.8, 1.2),
      ArcVector(8.0, 6.0));
  const diff_drive::TrajectoryLimits gv_traj_limits(gv_limits, 0.05);

  Trajectory clothoid_traj = ToTrajectory(
      {{18.9969893197, 0.0549329212344, -0.147956717123, -2.74889357189,
        -0.0856623417967, -0.3},
       {19.0469893197, 0.0589021499142, -0.146347378213, -2.76389357189,
        -0.0613922785071, -0.6},
       {19.0969893197, 0.0617719551146, -0.145258323075, -2.79389357189,
        -0.0336400926226, -0.9},
       {19.1469893197, 0.0633656665936, -0.144720972244, -2.83889357189,
        -0.0017168224974, -1.2},
       {19.1969893197, 0.0634483232708, -0.144697855971, -2.89889357189,
        -0.0017168224974, -1.2},
       {19.7544585661, 0.0643836989506, -0.144783948943, 2.71532863958,
        -0.0017168224974, -1.2},
       {19.8044585661, 0.064460747384, -0.144821764644, 2.65532863958,
        -0.0336400926226, -0.9},
       {19.8544585661, 0.0659295975129, -0.145640996903, 2.61032863958,
        -0.0613922785071, -0.6},
       {19.9044585661, 0.0685523962184, -0.147235597145, 2.58032863958,
        -0.0856623417967, -0.3},
       {19.9544585661, 0.0721611768288, -0.149542424088, 2.56532863958,
        -0.106787109375, -0},
       {20.0044585661, 0.0766382489076, -0.152451812756, 2.56532863958,
        -0.106787109375, -0},
       {20.0544585661, 0.0811153209864, -0.155361201424, 2.56532863958,
        -0.0837107382063, 0.3},
       {20.1044585661, 0.0846418844983, -0.157615473047, 2.58032863958,
        -0.0586072139972, 0.6},
       {20.1544585661, 0.0871456997783, -0.159137734151, 2.61032863958,
        -0.0313170822878, 0.9},
       {20.2044585661, 0.088513118735, -0.159900394478, 2.65532863958,
        -0.00153482222223, 1.2},
       {20.2544585661, 0.0885819992675, -0.159934201339, 2.71532863958,
        -0.00153482222223, 1.2},
       {21.1391770473, 0.0898699325901, -0.159799032903, -2.50619449019,
        -0.00153482222223, 1.2},
       {21.1891770473, 0.0899302935441, -0.159751662165, -2.44619449019,
        -0.0313170822878, 0.9},
       {21.2391770473, 0.091109579675, -0.158721725468, -2.40119449019,
        -0.0586072139972, 0.6},
       {21.2891770473, 0.093242798248, -0.156712808261, -2.37119449019,
        -0.0837107382063, 0.3},
       {21.3391770473, 0.0962245055409, -0.153775494459, -2.35619449019,
        -0.106787109375, -0},
       {21.3891770473, 0.1, -0.15, -2.35619449019, -0.106787109375, -0}});

  Trajectory amended_traj(128);
  amended_traj = clothoid_traj;
  EXPECT_TRUE(AmendTrajectoryToEndWithStop(gv_limits.GetKinematics(),
                                           gv_traj_limits, &amended_traj));
  EXPECT_TRUE(amended_traj.HasContinuousPosition());
  EXPECT_TRUE(amended_traj.HasMonotonicTimeValues());

  if (absl::GetFlag(FLAGS_print_results)) {
    std::cout << "clothoid_traj (data structure) = \n";
    std::cout << clothoid_traj << std::endl;
    std::cout << "amended_traj (data structure) = \n";
    std::cout << amended_traj << std::endl;
  }

  auto last_pt = std::prev(amended_traj.EndState());
  auto second_last_pt = std::prev(last_pt);
  auto third_last_pt = std::prev(second_last_pt);

  EXPECT_THAT(third_last_pt->state,
              IsStateApprox(ToState({0.0, 0.093242798248, -0.156712808261,
                                     -2.37119449019, -0.0837107382063, 0.3}),
                            kMidEpsilon));

  EXPECT_THAT(second_last_pt->state,
              IsStateApprox(ToState({0.0, 0.0962245055409, -0.153775494459,
                                     -2.35619449019, -0.106787109375, 0.0}),
                            kMidEpsilon));

  EXPECT_THAT(
      last_pt->state,
      IsStateApprox(ToState({0.0, 0.1, -0.15, -2.35619449019, 0.0, 0.0}),
                    kMidEpsilon));
}

TEST(CurveTrajectoryUtils, AmendTrajectoryToEndWithStopRegression3) {
  const diff_drive::Kinematics meta_kinematics(WheelVector(0.0625, 0.0625),
                                               0.365);
  const diff_drive::TrajectoryLimits meta_traj_limits(
      WheelVector(1.5, 1.5), ArcVector(0.1, 0.075), 0.05);

  Trajectory orig_traj = ToTrajectory(
      {{0.0, -0.00458786343628, 0, 0, 0.085910652921, 0.0429553264605},
       {0.05, -0.000292334092704, 4.61289840512e-06, 0.00214776632302,
        0.171821305842, 0.085910652921},
       {0.1, 0.00829864533517, 4.1515957973e-05, 0.00644329896907,
        0.257731958763, 0.128865979381},
       {0.15, 0.0211846191109, 0.000166062108317, 0.0128865979381,
        0.343642611684, 0.171821305842},
       {0.2, 0.0383641606232, 0.000461272285731, 0.0214776632302, 0.4, 0.2},
       {0.25, 0.0583570670622, 0.000990761470938, 0.0314776632302, 0.4, 0.2},
       {2.4352233677, 0.898548844055, 0.215526944003, 0.46852233677, 0.4, 0.2},
       {2.4852233677, 0.916348120746, 0.224647383465, 0.47852233677,
        0.343642611684, 0.171821305842},
       {2.5352233677, 0.931566118096, 0.232624623103, 0.487113402062,
        0.257731958763, 0.128865979381},
       {2.5852233677, 0.94293433458, 0.238693188492, 0.493556701031,
        0.171821305842, 0.085910652921},
       {2.6352233677, 0.950491320585, 0.242779537949, 0.497852233677,
        0.085910652921, 0.0429553264605},
       {2.6852233677, 0.954263213772, 0.244834876219, 0.5, 0.085910652921,
        0.0429553264605}});

  Trajectory amended_traj(128);
  amended_traj = orig_traj;
  EXPECT_TRUE(AmendTrajectoryToEndWithStop(meta_kinematics, meta_traj_limits,
                                           &amended_traj));
  EXPECT_TRUE(amended_traj.HasContinuousPosition());
  EXPECT_TRUE(amended_traj.HasMonotonicTimeValues());

  if (absl::GetFlag(FLAGS_print_results)) {
    std::cout << "orig_traj (data structure) = \n";
    std::cout << orig_traj << std::endl;
    std::cout << "amended_traj (data structure) = \n";
    std::cout << amended_traj << std::endl;
  }

  auto last_pt = std::prev(amended_traj.EndState());

  EXPECT_THAT(last_pt->state,
              IsStateApprox(ToState({2.6852233677, 0.954263213772,
                                     0.244834876219, 0.5, 0.0, 0.0}),
                            kMidEpsilon));
}

TEST(CurveTrajectoryUtils, FindSlowerTrajectoryToMeetEndSpeed) {
  const DynamicLimits fake_limits(kTestKinematics, WheelVector(1.0, 1.0),
                                  WheelVector(0.1, 0.1), ArcVector(1.0, 1.0),
                                  ArcVector(0.1, 0.1));
  const diff_drive::TrajectoryLimits fake_traj_limits(fake_limits, 0.1);

  EXPECT_TRUE(kTestClothoidTraj.HasContinuousPosition());

  Trajectory amended_traj = kTestClothoidTraj;
  EXPECT_TRUE(FindSlowerTrajectoryToMeetEndSpeed(
      kTestClothoidTraj, 1.0, 0.1, 1e-3, fake_limits.GetKinematics(),
      fake_traj_limits, &amended_traj));
  EXPECT_TRUE(amended_traj.HasContinuousPosition());
  EXPECT_TRUE(amended_traj.HasMonotonicTimeValues());
  EXPECT_THAT(amended_traj, SatisfiesLimits(fake_limits));

  if (absl::GetFlag(FLAGS_print_results)) {
    std::cout << "clothoid_traj (data structure) = \n";
    std::cout << kTestClothoidTraj << std::endl;
    std::cout << "amended_traj (data structure) = \n";
    std::cout << amended_traj << std::endl;
  }

  EXPECT_THAT(amended_traj.GetStart().state.GetPose(),
              IsApprox(eigenmath::Pose2d({0.0, 0.0}, 0.0), kMidEpsilon));
  EXPECT_NEAR(amended_traj.GetStart().state.GetArcVelocity().Translation(),
              0.55, 0.05);
  EXPECT_NEAR(amended_traj.GetStart().state.GetArcVelocity().Translation(),
              amended_traj.GetStart().state.GetArcVelocity().Rotation(),
              kMidEpsilon);

  EXPECT_THAT(
      amended_traj.GetFinish().state.GetPose(),
      IsApprox(eigenmath::Pose2d({0.9504156, 0.8481688}, 1.491), kMidEpsilon));
  EXPECT_NEAR(amended_traj.GetFinish().state.GetArcVelocity().Translation(),
              0.1, kMidEpsilon);
  EXPECT_NEAR(amended_traj.GetFinish().state.GetArcVelocity().Rotation(), 0.114,
              kMidEpsilon);
}

TEST(CurveTrajectoryUtils, FindSlowerTrajectoryToEndWithStop) {
  const DynamicLimits fake_limits(kTestKinematics, WheelVector(1.0, 1.0),
                                  WheelVector(0.1, 0.1), ArcVector(1.0, 1.0),
                                  ArcVector(0.1, 0.1));
  const diff_drive::TrajectoryLimits fake_traj_limits(fake_limits, 0.1);

  EXPECT_TRUE(kTestClothoidTraj.HasContinuousPosition());

  Trajectory amended_traj = kTestClothoidTraj;
  EXPECT_TRUE(FindSlowerTrajectoryToEndWithStop(
      kTestClothoidTraj, 1e-3, fake_limits.GetKinematics(), fake_traj_limits,
      &amended_traj));
  EXPECT_TRUE(amended_traj.HasContinuousPosition());
  EXPECT_TRUE(amended_traj.HasMonotonicTimeValues());
  EXPECT_THAT(amended_traj, SatisfiesLimits(fake_limits));

  if (absl::GetFlag(FLAGS_print_results)) {
    std::cout << "clothoid_traj (data structure) = \n";
    std::cout << kTestClothoidTraj << std::endl;
    std::cout << "amended_traj (data structure) = \n";
    std::cout << amended_traj << std::endl;
  }

  EXPECT_THAT(amended_traj.GetStart().state.GetPose(),
              IsApprox(eigenmath::Pose2d({0.0, 0.0}, 0.0), kMidEpsilon));
  EXPECT_NEAR(amended_traj.GetStart().state.GetArcVelocity().Translation(),
              0.45, 0.05);
  EXPECT_NEAR(amended_traj.GetStart().state.GetArcVelocity().Translation(),
              amended_traj.GetStart().state.GetArcVelocity().Rotation(),
              kMidEpsilon);

  EXPECT_THAT(
      amended_traj.GetFinish().state.GetPose(),
      IsApprox(eigenmath::Pose2d({0.9504156, 0.8481688}, 1.491), kMidEpsilon));
  EXPECT_NEAR(amended_traj.GetFinish().state.GetArcVelocity().Translation(),
              0.0, kMidEpsilon);
  EXPECT_NEAR(amended_traj.GetFinish().state.GetArcVelocity().Rotation(), 0.0,
              kMidEpsilon);
}

TEST(CurveTrajectoryUtils, QuinticSplineToCurve) {
  Eigen::MatrixXd control_points{6, 2};
  control_points.row(0) << 0.0, 0.0;
  control_points.row(1) << 0.973404761, 0.0;
  control_points.row(2) << -6.361045073, 0.0;
  control_points.row(3) << 0.5, 1.0;
  control_points.row(4) << 0.580120311, -0.903483853;
  control_points.row(5) << -3.700131697, 5.762613687;
  const eigenmath::QuinticSpline spline{control_points};

  {
    Curve simple_curve{6};
    EXPECT_TRUE(ConvertQuinticSplineToCurve(spline, 5, &simple_curve));
    EXPECT_GE(simple_curve.GetSize(), 5);
    EXPECT_TRUE(simple_curve.HasContinuousPosition());
    EXPECT_TRUE(simple_curve.HasMonotonicCordLengths());
    if (absl::GetFlag(FLAGS_print_results)) {
      std::cout << "simple_curve (data structure) = \n";
      std::cout << simple_curve << std::endl;
      std::cout << "simple_curve (curve trace) = \n";
      PrintCurveTrace(simple_curve, 0.01, &std::cout);
    }

    const CurvePoint start_pt = simple_curve.GetStart().point;
    EXPECT_NEAR(start_pt.GetPose().translation().x(), control_points(0, 0),
                kEpsilon);
    EXPECT_NEAR(start_pt.GetPose().translation().y(), control_points(0, 1),
                kEpsilon);

    const CurvePoint end_pt = simple_curve.GetFinish().point;
    EXPECT_NEAR(end_pt.GetPose().translation().x(), control_points(3, 0),
                kEpsilon);
    EXPECT_NEAR(end_pt.GetPose().translation().y(), control_points(3, 1),
                kEpsilon);

    EXPECT_LT(simple_curve.GetCordLengthSpan().Length(), 1.6);
  }

  {
    Curve mid_curve(16);
    EXPECT_TRUE(ConvertQuinticSplineToCurve(spline, 15, &mid_curve));
    EXPECT_GE(mid_curve.GetSize(), 15);
    EXPECT_TRUE(mid_curve.HasContinuousPosition());
    EXPECT_TRUE(mid_curve.HasMonotonicCordLengths());
    if (absl::GetFlag(FLAGS_print_results)) {
      std::cout << "mid_curve (data structure) = \n";
      std::cout << mid_curve << std::endl;
      std::cout << "mid_curve (curve trace) = \n";
      PrintCurveTrace(mid_curve, 0.01, &std::cout);
    }

    const CurvePoint start_pt = mid_curve.GetStart().point;
    EXPECT_NEAR(start_pt.GetPose().translation().x(), control_points(0, 0),
                kEpsilon);
    EXPECT_NEAR(start_pt.GetPose().translation().y(), control_points(0, 1),
                kEpsilon);

    const CurvePoint end_pt = mid_curve.GetFinish().point;
    EXPECT_NEAR(end_pt.GetPose().translation().x(), control_points(3, 0),
                kEpsilon);
    EXPECT_NEAR(end_pt.GetPose().translation().y(), control_points(3, 1),
                kEpsilon);

    EXPECT_LT(mid_curve.GetCordLengthSpan().Length(), 1.55);
  }

  {
    Curve precise_curve(101);
    EXPECT_TRUE(ConvertQuinticSplineToCurve(spline, 100, &precise_curve));
    EXPECT_GE(precise_curve.GetSize(), 100);
    EXPECT_TRUE(precise_curve.HasContinuousPosition());
    EXPECT_TRUE(precise_curve.HasMonotonicCordLengths());
    if (absl::GetFlag(FLAGS_print_results)) {
      std::cout << "precise_curve (data structure) = \n";
      std::cout << precise_curve << std::endl;
      std::cout << "precise_curve (curve trace) = \n";
      PrintCurveTrace(precise_curve, 0.01, &std::cout);
    }

    const CurvePoint start_pt = precise_curve.GetStart().point;
    EXPECT_NEAR(start_pt.GetPose().translation().x(), control_points(0, 0),
                kEpsilon);
    EXPECT_NEAR(start_pt.GetPose().translation().y(), control_points(0, 1),
                kEpsilon);

    const CurvePoint end_pt = precise_curve.GetFinish().point;
    EXPECT_NEAR(end_pt.GetPose().translation().x(), control_points(3, 0),
                kEpsilon);
    EXPECT_NEAR(end_pt.GetPose().translation().y(), control_points(3, 1),
                kEpsilon);

    EXPECT_LT(precise_curve.GetCordLengthSpan().Length(), 1.52);
  }
}

TEST(CurveTrajectoryUtils, QuinticSpline3ptToCurve) {
  Eigen::MatrixXd control_points{9, 2};
  control_points.row(0) << 0.0, 0.0;
  control_points.row(1) << 0.552076129, 0.0;
  control_points.row(2) << -0.341905086, 0.0;
  control_points.row(3) << 0.5, 1.0;
  control_points.row(4) << 0.202267382, -0.315012783;
  control_points.row(5) << -2.196184244, 3.420354306;
  control_points.row(6) << 1.0, 0.5;
  control_points.row(7) << -0.053974919, -0.117937350;
  control_points.row(8) << 0.912336173, 1.993490905;
  const eigenmath::QuinticSpline spline{control_points};

  {
    Curve simple_curve{11};
    EXPECT_TRUE(ConvertQuinticSplineToCurve(spline, 5, &simple_curve));
    EXPECT_GE(simple_curve.GetSize(), 10);
    EXPECT_TRUE(simple_curve.HasContinuousPosition());
    EXPECT_TRUE(simple_curve.HasMonotonicCordLengths());
    if (absl::GetFlag(FLAGS_print_results)) {
      std::cout << "simple_curve (data structure) = \n";
      std::cout << simple_curve << std::endl;
      std::cout << "simple_curve (curve trace) = \n";
      PrintCurveTrace(simple_curve, 0.01, &std::cout);
    }

    const CurvePoint start_pt = simple_curve.GetStart().point;
    EXPECT_NEAR(start_pt.GetPose().translation().x(), control_points(0, 0),
                kEpsilon);
    EXPECT_NEAR(start_pt.GetPose().translation().y(), control_points(0, 1),
                kEpsilon);

    const CurvePoint end_pt = simple_curve.GetFinish().point;
    EXPECT_NEAR(end_pt.GetPose().translation().x(), control_points(6, 0),
                kEpsilon);
    EXPECT_NEAR(end_pt.GetPose().translation().y(), control_points(6, 1),
                kEpsilon);

    EXPECT_LT(simple_curve.GetCordLengthSpan().Length(), 1.98);
  }

  {
    Curve mid_curve{31};
    EXPECT_TRUE(ConvertQuinticSplineToCurve(spline, 15, &mid_curve));
    EXPECT_GE(mid_curve.GetSize(), 30);
    EXPECT_TRUE(mid_curve.HasContinuousPosition());
    EXPECT_TRUE(mid_curve.HasMonotonicCordLengths());
    if (absl::GetFlag(FLAGS_print_results)) {
      std::cout << "mid_curve (data structure) = \n";
      std::cout << mid_curve << std::endl;
      std::cout << "mid_curve (curve trace) = \n";
      PrintCurveTrace(mid_curve, 0.01, &std::cout);
    }

    const CurvePoint start_pt = mid_curve.GetStart().point;
    EXPECT_NEAR(start_pt.GetPose().translation().x(), control_points(0, 0),
                kEpsilon);
    EXPECT_NEAR(start_pt.GetPose().translation().y(), control_points(0, 1),
                kEpsilon);

    const CurvePoint end_pt = mid_curve.GetFinish().point;
    EXPECT_NEAR(end_pt.GetPose().translation().x(), control_points(6, 0),
                kEpsilon);
    EXPECT_NEAR(end_pt.GetPose().translation().y(), control_points(6, 1),
                kEpsilon);

    EXPECT_LT(mid_curve.GetCordLengthSpan().Length(), 1.98);
  }
}

TEST(CurveTrajectoryUtils, AppendRampToSpeedWithConstantAccel) {
  const ArcVector desired_arc_velocity{0.5, 0.3};
  Trajectory result_traj(128);
  result_traj.AddState(1.0, State({{1.0, 2.0}, 0.1}, {0.1, -0.3}));

  EXPECT_TRUE(AppendRampToSpeedWithConstantAccel(
      desired_arc_velocity, kTestKinematics,
      TrajectoryLimits(kTestDynamicLimits, 0.1), 5.0, 0.5, &result_traj));
  EXPECT_GE(result_traj.GetSize(), 10);

  const std::vector<testing::RawStateTime> expected_pts = {
      {1.0, 1.0, 2.0, 0.1, 0.1, -0.3},
      {1.1, 1.0099635, 2.0008489, 0.07, 0.15, -0.225},
      {1.2, 1.0249373, 2.0017297, 0.0475, 0.2, -0.15},
      {1.3, 1.0449211, 2.0025294, 0.0325, 0.25, -0.075},
      {1.4, 1.0699108, 2.0032481, 0.025, 0.3, 0.0},
      {1.5, 1.0999014, 2.003998, 0.025, 0.35, 0.075},
      {1.6, 1.1348868, 2.0050041, 0.032500, 0.4, 0.15},
      {1.7, 1.1748545, 2.0066037, 0.047500, 0.45, 0.225},
      {1.8, 1.2197759, 2.0092459, 0.070000, 0.5, 0.3},
      {6.0, 2.7217851, 3.2743708, 1.33, 0.5, 0.3}};
  EXPECT_THAT(result_traj,
              IsTrajectoryApprox(ToTrajectory(expected_pts), kMidEpsilon));
}

TEST(CurveTrajectoryUtils, AppendRampToSpeedWithConstantAccelTooShort) {
  const ArcVector desired_arc_velocity{0.5, 0.3};
  Trajectory result_traj(128);
  result_traj.AddState(1.0, State({{1.0, 2.0}, 0.1}, {0.1, -0.3}));

  EXPECT_TRUE(AppendRampToSpeedWithConstantAccel(
      desired_arc_velocity, kTestKinematics,
      TrajectoryLimits(kTestDynamicLimits, 0.1), 0.5, 0.5, &result_traj));
  EXPECT_GE(result_traj.GetSize(), 6);

  const std::vector<testing::RawStateTime> expected_pts = {
      {1.0, 1.0, 2.0, 0.1, 0.1, -0.3},
      {1.1, 1.0099635, 2.0008489, 0.07, 0.18, -0.18},
      {1.2, 1.0279298, 2.0019462, 0.052, 0.26, -0.06},
      {1.3, 1.0538986, 2.0032197, 0.046, 0.34, 0.06},
      {1.4, 1.0878577, 2.0048851, 0.052, 0.42, 0.18},
      {1.5, 1.129779, 2.0074454, 0.07, 0.5, 0.3}};
  EXPECT_THAT(result_traj,
              IsTrajectoryApprox(ToTrajectory(expected_pts), kMidEpsilon));
}

TEST(CurveTrajectoryUtils, AppendRampToSpeedWithConstantAccelMuchTooShort) {
  const ArcVector desired_arc_velocity{0.5, 0.3};
  Trajectory result_traj(128);
  result_traj.AddState(1.0, State({{1.0, 2.0}, 0.1}, {0.1, -0.3}));

  EXPECT_TRUE(AppendRampToSpeedWithConstantAccel(
      desired_arc_velocity, kTestKinematics,
      TrajectoryLimits(kTestDynamicLimits, 0.1), 0.3, 0.5, &result_traj));
  EXPECT_GE(result_traj.GetSize(), 4);

  const std::vector<testing::RawStateTime> expected_pts = {
      {1.0, 1.0, 2.0, 0.1, 0.1, -0.3},
      {1.1, 1.0099635, 2.0008489, 0.07, 0.2, -0.15},
      {1.2, 1.0299243, 2.0020981, 0.055, 0.3, 0.0},
      {1.3, 1.0598789, 2.0037473, 0.055, 0.4, 0.15}};
  EXPECT_THAT(result_traj,
              IsTrajectoryApprox(ToTrajectory(expected_pts), kMidEpsilon));
}

TEST(CurveTrajectoryUtils, ComputeTotalWheelMotion) {
  // Segment 1:
  EXPECT_NEAR(ComputeTotalWheelMotion(kTestTraj, kTestKinematics,
                                      Interval<double>(0.0, 0.25)),
              M_PI_2, kEpsilon);
  // Segment 2:
  EXPECT_NEAR(ComputeTotalWheelMotion(kTestTraj, kTestKinematics,
                                      Interval<double>(0.25, 1.25)),
              1.0, kEpsilon);

  // Total interval:
  EXPECT_NEAR(ComputeTotalWheelMotion(kTestTraj, kTestKinematics), 3.0,
              kEpsilon);
  // Before:
  EXPECT_NEAR(ComputeTotalWheelMotion(kTestTraj, kTestKinematics,
                                      Interval<double>(-1.0, -0.5)),
              0.0, kEpsilon);
  // After:
  EXPECT_NEAR(ComputeTotalWheelMotion(kTestTraj, kTestKinematics,
                                      Interval<double>(3.5, 4.0)),
              0.0, kEpsilon);
  // Straddle start:
  EXPECT_NEAR(ComputeTotalWheelMotion(kTestTraj, kTestKinematics,
                                      Interval<double>(-0.5, 1.0)),
              M_PI_2, kEpsilon);
  // Straddle finish:
  EXPECT_NEAR(ComputeTotalWheelMotion(kTestTraj, kTestKinematics,
                                      Interval<double>(2.0, 3.5)),
              M_PI_2, kEpsilon);
  // Inside:
  EXPECT_NEAR(ComputeTotalWheelMotion(kTestTraj, kTestKinematics,
                                      Interval<double>(1.0, 2.0)),
              M_PI, kEpsilon);
}

TEST(CurveTrajectoryUtils, ComputeTotalAbsoluteWheelMotion) {
  // Segment 1:
  EXPECT_NEAR(ComputeTotalAbsoluteWheelMotion(kTestTraj, kTestKinematics,
                                              Interval<double>(0.0, 0.25)),
              M_PI_2, kEpsilon);
  // Segment 2:
  EXPECT_NEAR(ComputeTotalAbsoluteWheelMotion(kTestTraj, kTestKinematics,
                                              Interval<double>(0.25, 1.25)),
              1.0, kEpsilon);

  // Total interval:
  EXPECT_NEAR(ComputeTotalAbsoluteWheelMotion(kTestTraj, kTestKinematics),
              2.0 * M_PI + 2, kEpsilon);
  // Before:
  EXPECT_NEAR(ComputeTotalAbsoluteWheelMotion(kTestTraj, kTestKinematics,
                                              Interval<double>(-1.0, -0.5)),
              0.0, kEpsilon);
  // After:
  EXPECT_NEAR(ComputeTotalAbsoluteWheelMotion(kTestTraj, kTestKinematics,
                                              Interval<double>(3.5, 4.0)),
              0.0, kEpsilon);
  // Straddle start:
  EXPECT_NEAR(ComputeTotalAbsoluteWheelMotion(kTestTraj, kTestKinematics,
                                              Interval<double>(-0.5, 1.0)),
              2.3207963, kEpsilon);
  // Straddle finish:
  EXPECT_NEAR(ComputeTotalAbsoluteWheelMotion(kTestTraj, kTestKinematics,
                                              Interval<double>(2.0, 3.5)),
              2.3207963, kEpsilon);
  // Inside:
  EXPECT_NEAR(ComputeTotalAbsoluteWheelMotion(kTestTraj, kTestKinematics,
                                              Interval<double>(1.0, 2.0)),
              3.6415927, kEpsilon);
}

TEST(CurveTrajectoryUtils, ConvertCurveToWheelCurve) {
  WheelCurve curve_out;
  EXPECT_TRUE(
      ConvertCurveToWheelCurve(kTestCurve, kTestKinematics, &curve_out));
  EXPECT_THAT(curve_out, IsWheelCurveApprox(kTestWheelCurve, kEpsilon));
  EXPECT_THAT(curve_out, WheelCurveEvaluatesTo(kTestWheelCurveEval, kEpsilon));
}

TEST(CurveTrajectoryUtils, ConvertCurveToWheelCurvePureRotation) {
  const Curve test_curve = ToCurve({{0.0, 0.0, 0.0, 0.0, 0.0},
                                    {1.0, 1.0, 0.0, 0.5 * M_PI, 0.0},
                                    {2.0, 1.0, 1.0, 0.5 * M_PI, 0.0}});
  WheelCurve curve_out;
  EXPECT_TRUE(
      ConvertCurveToWheelCurve(test_curve, kTestKinematics, &curve_out));

  const WheelCurve expected_curve = ToWheelCurve(
      {{0.0, 0.0, 0.0, 0.5, 0.5},
       {1.0, 0.5, 0.5, -0.5, 0.5},
       {0.5 * M_PI + 1.0, -0.285398163397, 1.2853981634, 0.5, 0.5},
       {0.5 * M_PI + 2.0, 0.214601836603, 1.7853981634, 0.5, 0.5}});
  EXPECT_THAT(curve_out, IsWheelCurveApprox(expected_curve, kEpsilon));
}

TEST(CurveTrajectoryUtils, ConvertTrajectoryToWheelCurve) {
  WheelCurve curve_out;
  EXPECT_TRUE(
      ConvertTrajectoryToWheelCurve(kTestTraj, kTestKinematics, &curve_out));
  EXPECT_THAT(curve_out, IsWheelCurveApprox(kTestWheelCurve, kEpsilon));
  EXPECT_THAT(curve_out, WheelCurveEvaluatesTo(kTestWheelCurveEval, kEpsilon));
}

TEST(CurveTrajectoryUtils, ConvertTrajectoryToWheelCurvePureRotation) {
  const Trajectory test_traj =
      ToTrajectory({{0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
                    {1.0, 1.0, 0.0, 0.0, 0.0, 0.5 * M_PI},
                    {2.0, 1.0, 0.0, 0.5 * M_PI, 1.0, 0.0},
                    {3.0, 1.0, 1.0, 0.5 * M_PI, 1.0, 0.0}});
  WheelCurve curve_out;
  EXPECT_TRUE(
      ConvertTrajectoryToWheelCurve(test_traj, kTestKinematics, &curve_out));

  const WheelCurve expected_curve = ToWheelCurve(
      {{0.0, 0.0, 0.0, 0.5, 0.5},
       {1.0, 0.5, 0.5, -0.5, 0.5},
       {0.5 * M_PI + 1.0, -0.285398163397, 1.2853981634, 0.5, 0.5},
       {0.5 * M_PI + 2.0, 0.214601836603, 1.7853981634, 0.5, 0.5}});
  EXPECT_THAT(curve_out, IsWheelCurveApprox(expected_curve, kEpsilon));
}

TEST(CurveTrajectoryUtils, WheelMotionIterator) {
  TrajectoryWheelMotionIterator it =
      BeginByWheelMotion(&kTestKinematics, kTestTraj);
  TrajectoryWheelMotionIterator it_end =
      EndByWheelMotion(&kTestKinematics, kTestTraj);

  constexpr double kBigWheelEpsilon = kBigEpsilon * 2.0 * M_PI;

  const double wheel_increments[12] = {0.0,
                                       kBigWheelEpsilon,
                                       M_PI_2 - 2.0 * kBigWheelEpsilon,
                                       kBigWheelEpsilon + kBigEpsilon,
                                       1.0 - 2.0 * kBigEpsilon,
                                       kBigWheelEpsilon + kBigEpsilon,
                                       M_PI - 2.0 * kBigWheelEpsilon,
                                       kBigWheelEpsilon + kBigEpsilon,
                                       1.0 - 2.0 * kBigEpsilon,
                                       kBigWheelEpsilon + kBigEpsilon,
                                       M_PI_2 - 2.0 * kBigWheelEpsilon,
                                       kBigWheelEpsilon + kEpsilon};
  constexpr int kNumWheelIncrements =
      sizeof(wheel_increments) / sizeof(wheel_increments[0]);

  for (int i = 0; i < kNumWheelIncrements; ++i) {
    it += wheel_increments[i];
    if (i == kNumWheelIncrements - 1) {
      EXPECT_FALSE(it < it_end);
    } else {
      EXPECT_TRUE(it < it_end);
    }

    State cur_state = it.ComputeState();
    EXPECT_THAT(
        cur_state.GetPose(),
        IsApprox(eigenmath::Pose2d({kTestCurveEval[i].x, kTestCurveEval[i].y},
                                   kTestCurveEval[i].theta),
                 kBigEpsilon));
  }
  // Check if termination works past the end too:
  it += kBigEpsilon;
  EXPECT_FALSE(it < it_end);
}

TEST(CurveTrajectoryUtils, WheelMotionIteratorWithStop) {
  Trajectory traj(16);
  traj = ToTrajectory({{0.0, 0.0, 0.0, 0.0, 1.0, 0.0},
                       {1.0, 1.0, 0.0, 0.0, 1.0, 0.0},
                       {2.0, 2.0, 0.0, 0.0, 0.0, 0.0},
                       {3.0, 2.0, 0.0, 0.0, 0.0, 0.0},
                       {4.0, 2.0, 0.0, 0.0, 1.0, 0.0},
                       {5.0, 3.0, 0.0, 0.0, 1.0, 0.0},
                       {6.0, 4.0, 0.0, 0.0, 0.0, 0.0},
                       {7.0, 4.0, 0.0, 0.0, 0.0, 0.0}});

  TrajectoryWheelMotionIterator it = BeginByWheelMotion(&kTestKinematics, traj);
  TrajectoryWheelMotionIterator it_end =
      EndByWheelMotion(&kTestKinematics, traj);

  State cur_state = it.ComputeState();
  EXPECT_NEAR(it.GetTime(), 0.0, kBigEpsilon);
  EXPECT_THAT(cur_state, IsStateApprox(ToState({0.0, 0.0, 0.0, 0.0, 1.0, 0.0}),
                                       kBigEpsilon));

  it += 1.0;
  cur_state = it.ComputeState();
  EXPECT_NEAR(it.GetTime(), 1.0, kBigEpsilon);
  EXPECT_THAT(cur_state, IsStateApprox(ToState({1.0, 1.0, 0.0, 0.0, 1.0, 0.0}),
                                       kBigEpsilon));

  it += kEpsilon;
  cur_state = it.ComputeState();
  EXPECT_NEAR(it.GetTime(), 1.0, kBigEpsilon);
  EXPECT_THAT(cur_state, IsStateApprox(ToState({1.0, 1.0, 0.0, 0.0, 1.0, 0.0}),
                                       kBigEpsilon));

  it += 1.0;
  cur_state = it.ComputeState();
  EXPECT_NEAR(it.GetTime(), 4.0, kBigEpsilon);
  EXPECT_THAT(cur_state, IsStateApprox(ToState({4.0, 2.0, 0.0, 0.0, 1.0, 0.0}),
                                       kBigEpsilon));

  it += 1.0;
  cur_state = it.ComputeState();
  EXPECT_NEAR(it.GetTime(), 5.0, kBigEpsilon);
  EXPECT_THAT(cur_state, IsStateApprox(ToState({5.0, 3.0, 0.0, 0.0, 1.0, 0.0}),
                                       kBigEpsilon));

  it += 1.0;
  cur_state = it.ComputeState();
  EXPECT_NEAR(it.GetTime(), 7.0, kBigEpsilon);
  EXPECT_THAT(cur_state, IsStateApprox(ToState({7.0, 4.0, 0.0, 0.0, 0.0, 0.0}),
                                       kBigEpsilon));

  // Check if termination works past the end too:
  EXPECT_FALSE(it < it_end);
}

TEST(CurveTrajectoryUtils, WheelMotionIteratorWithEmpty) {
  Trajectory traj(16);

  TrajectoryWheelMotionIterator it = BeginByWheelMotion(&kTestKinematics, traj);
  TrajectoryWheelMotionIterator it_end =
      EndByWheelMotion(&kTestKinematics, traj);

  EXPECT_FALSE(it < it_end);
}

TEST(CurveTrajectoryUtils, PrintLimits) {
  const diff_drive::DynamicLimits dv_limits(
      diff_drive::Kinematics(WheelVector(0.0762, 0.0762), 0.31),
      WheelVector(6.0, 6.0), WheelVector(120.0, 120.0), ArcVector(0.4, 0.4),
      ArcVector(2.0, 2.0));
  std::stringstream ss_dd_limits;
  ss_dd_limits << dv_limits;
  EXPECT_EQ(ss_dd_limits.str(),
            "kinematics: wheel radius left: 0.0762 wheel radius right: "
            "0.0762 wheel base: 0.31 velocity limits: min wheel: (l: -6, r: "
            "-6) max wheel: (l: 6, r: 6) min arc: (t: -0.4, r: -0.4) max arc: "
            "(t: 0.4, r: 0.4) acceleration limits: min wheel: (l: -120, r: "
            "-120) max wheel: (l: 120, r: 120) min arc: (t: -2, r: -2) max "
            "arc: (t: 2, r: 2)");

  const diff_drive::TrajectoryLimits dv_traj_limits(dv_limits, 0.004);
  std::stringstream ss_traj_limits;
  ss_traj_limits << dv_traj_limits;
  EXPECT_EQ(ss_traj_limits.str(),
            "max wheel velocity jump: (l: 0.48, r: 0.48) max arc velocity "
            "jump: (t: 0.008, r: 0.008) min cycle duration: 0.004");
}

TEST(CurveTrajectoryUtils, PrintCurve) {
  std::stringstream ss;
  ss << kTestCurve;

  const std::string expected_str =
      R"""(s: 0.0000000000000000 x: 0.0000000000000000 y: 0.0000000000000000 theta: 0.0000000000000000 kappa: 6.2831853071795862
s: 0.2500000000000000 x: 0.1591549430918953 y: 0.1591549430918953 theta: 1.5707963267948966 kappa: 0.0000000000000000
s: 1.2500000000000000 x: 0.1591549430918953 y: 1.1591549430918953 theta: 1.5707963267948966 kappa: -6.2831853071795862
s: 1.7500000000000000 x: 0.4774648292756860 y: 1.1591549430918953 theta: -1.5707963267948966 kappa: 0.0000000000000000
s: 2.7500000000000000 x: 0.4774648292756860 y: 0.1591549430918953 theta: -1.5707963267948966 kappa: 6.2831853071795862
s: 3.0000000000000000 x: 0.6366197723675814 y: 0.0000000000000000 theta: 0.0000000000000000 kappa: 0.0000000000000000
)""";

  EXPECT_EQ(ss.str(), expected_str);
}

TEST(CurveTrajectoryUtils, PrintCurveTrace) {
  std::stringstream ss;
  PrintCurveTrace(kTestCurve, 0.25, &ss);

  const std::string expected_str =
      R"""(0.0000000000000000 0.0000000000000000 0.0000000000000000 0.0000000000000000 6.2831853071795862
0.2500000000000000 0.1591549430918953 0.1591549430918953 1.5707963267948966 0.0000000000000000
0.5000000000000000 0.1591549430918954 0.4091549430918954 1.5707963267948966 0.0000000000000000
0.7500000000000000 0.1591549430918954 0.6591549430918954 1.5707963267948966 0.0000000000000000
1.0000000000000000 0.1591549430918954 0.9091549430918954 1.5707963267948966 0.0000000000000000
1.2500000000000000 0.1591549430918953 1.1591549430918953 1.5707963267948966 -6.2831853071795862
1.5000000000000000 0.3183098861837906 1.3183098861837905 0.0000000000000000 -6.2831853071795862
1.7500000000000000 0.4774648292756860 1.1591549430918953 -1.5707963267948966 0.0000000000000000
2.0000000000000000 0.4774648292756860 0.9091549430918953 -1.5707963267948966 0.0000000000000000
2.2500000000000000 0.4774648292756861 0.6591549430918953 -1.5707963267948966 0.0000000000000000
2.5000000000000000 0.4774648292756861 0.4091549430918953 -1.5707963267948966 0.0000000000000000
2.7500000000000000 0.4774648292756860 0.1591549430918953 -1.5707963267948966 6.2831853071795862
3.0000000000000000 0.6366197723675814 0.0000000000000000 0.0000000000000000 0.0000000000000000
)""";

  EXPECT_EQ(ss.str(), expected_str);
}

TEST(CurveTrajectoryUtils, PrintWheelCurve) {
  std::stringstream ss;
  ss << kTestWheelCurve;

  const std::string expected_str =
      R"""(w: 0.0000000000000000 l: 0.0000000000000000 r: 0.0000000000000000 fl: -0.4204225284540000 fr: 0.5795774715460000
w: 1.5707963267948966 l: -0.6603981633970000 r: 0.9103981633970000 fl: 0.5000000000000000 fr: 0.5000000000000000
w: 2.5707963267948966 l: -0.1603981633970000 r: 1.4103981634000000 fl: 0.5795774715460000 fr: -0.4204225284540000
w: 5.7123889803846897 l: 1.6603981634000000 r: 0.0896018366026000 fl: 0.5000000000000000 fr: 0.5000000000000000
w: 6.7123889803846897 l: 2.1603981634000000 r: 0.5896018366030000 fl: -0.4204225284540000 fr: 0.5795774715460000
w: 8.2831853071795862 l: 1.5000000000000000 r: 1.5000000000000000 fl: 0.5000000000000000 fr: 0.5000000000000000
)""";

  EXPECT_EQ(ss.str(), expected_str);
}

TEST(CurveTrajectoryUtils, PrintWheelCurveTrace) {
  std::stringstream ss;
  PrintWheelCurveTrace(kTestWheelCurve, 0.5, &ss);

  const std::string expected_str =
      R"""(0.0000000000000000 0.0000000000000000 0.0000000000000000 -0.4204225284540000 0.5795774715460000
0.5000000000000000 -0.2102112642270000 0.2897887357730000 -0.4204225284540000 0.5795774715460000
1.0000000000000000 -0.4204225284540000 0.5795774715460000 -0.4204225284540000 0.5795774715460000
1.5000000000000000 -0.6306337926810001 0.8693662073190001 -0.4204225284540000 0.5795774715460000
2.0000000000000000 -0.4457963267944482 1.1249999999995517 0.5000000000000000 0.5000000000000000
2.5000000000000000 -0.1957963267944482 1.3749999999995517 0.5000000000000000 0.5000000000000000
3.0000000000000000 0.0883586162974695 1.2299512698893662 0.5795774715460000 -0.4204225284540000
3.5000000000000000 0.3781473520704696 1.0197400056623660 0.5795774715460000 -0.4204225284540000
4.0000000000000000 0.6679360878434696 0.8095287414353660 0.5795774715460000 -0.4204225284540000
4.5000000000000000 0.9577248236164697 0.5993174772083660 0.5795774715460000 -0.4204225284540000
5.0000000000000000 1.2475135593894697 0.3891062129813661 0.5795774715460000 -0.4204225284540000
5.5000000000000000 1.5373022951624697 0.1788949487543661 0.5795774715460000 -0.4204225284540000
6.0000000000000000 1.8042036732076552 0.2334073464102552 0.5000000000000000 0.5000000000000000
6.5000000000000000 2.0542036732076552 0.4834073464102552 0.5000000000000000 0.5000000000000000
7.0000000000000000 2.0394800113220981 0.7562947041404087 -0.4204225284540000 0.5795774715460000
7.5000000000000000 1.8292687470950981 1.0460834399134087 -0.4204225284540000 0.5795774715460000
8.0000000000000000 1.6190574828680981 1.3358721756864087 -0.4204225284540000 0.5795774715460000
8.2831853071795862 1.5000000000000000 1.5000000000000000 0.5000000000000000 0.5000000000000000
)""";

  EXPECT_EQ(ss.str(), expected_str);
}

TEST(CurveTrajectoryUtils, PrintTrajectory) {
  std::stringstream ss;
  ss << kTestTraj;

  const std::string expected_str =
      R"""(t: 0.0000000000000000 x: 0.0000000000000000 y: 0.0000000000000000 theta: 0.0000000000000000 v: 1.0000000000000000 w: 6.2831853071795862
t: 0.2500000000000000 x: 0.1591549430918953 y: 0.1591549430918953 theta: 1.5707963267948966 v: 1.0000000000000000 w: 0.0000000000000000
t: 1.2500000000000000 x: 0.1591549430918953 y: 1.1591549430918953 theta: 1.5707963267948966 v: 1.0000000000000000 w: -6.2831853071795862
t: 1.7500000000000000 x: 0.4774648292756860 y: 1.1591549430918953 theta: -1.5707963267948966 v: 1.0000000000000000 w: 0.0000000000000000
t: 2.7500000000000000 x: 0.4774648292756860 y: 0.1591549430918953 theta: -1.5707963267948966 v: 1.0000000000000000 w: 6.2831853071795862
t: 3.0000000000000000 x: 0.6366197723675814 y: 0.0000000000000000 theta: 0.0000000000000000 v: 1.0000000000000000 w: 0.0000000000000000
)""";

  EXPECT_EQ(ss.str(), expected_str);
}

TEST(CurveTrajectoryUtils, PrintTrajectoryTrace) {
  std::stringstream ss;
  PrintTrajectoryTrace(kTestTraj, 0.25, &ss);

  const std::string expected_str =
      R"""(0.0000000000000000 0.0000000000000000 0.0000000000000000 0.0000000000000000 1.0000000000000000 6.2831853071795862
0.2500000000000000 0.1591549430918953 0.1591549430918953 1.5707963267948966 1.0000000000000000 6.2831853071795862
0.5000000000000000 0.1591549430918954 0.4091549430918954 1.5707963267948966 1.0000000000000000 0.0000000000000000
0.7500000000000000 0.1591549430918954 0.6591549430918954 1.5707963267948966 1.0000000000000000 0.0000000000000000
1.0000000000000000 0.1591549430918954 0.9091549430918954 1.5707963267948966 1.0000000000000000 0.0000000000000000
1.2500000000000000 0.1591549430918954 1.1591549430918953 1.5707963267948966 1.0000000000000000 0.0000000000000000
1.5000000000000000 0.3183098861837906 1.3183098861837905 0.0000000000000000 1.0000000000000000 -6.2831853071795862
1.7500000000000000 0.4774648292756860 1.1591549430918953 -1.5707963267948966 1.0000000000000000 -6.2831853071795862
2.0000000000000000 0.4774648292756860 0.9091549430918953 -1.5707963267948966 1.0000000000000000 0.0000000000000000
2.2500000000000000 0.4774648292756861 0.6591549430918953 -1.5707963267948966 1.0000000000000000 0.0000000000000000
2.5000000000000000 0.4774648292756861 0.4091549430918953 -1.5707963267948966 1.0000000000000000 0.0000000000000000
2.7500000000000000 0.4774648292756861 0.1591549430918953 -1.5707963267948966 1.0000000000000000 0.0000000000000000
3.0000000000000000 0.6366197723675814 0.0000000000000000 0.0000000000000000 1.0000000000000000 0.0000000000000000
)""";

  EXPECT_EQ(ss.str(), expected_str);
}

TEST(CurveTrajectoryUtils, FindClothoidNegativeCurvatureAlmostStraightLine) {
  {
    // A curve that failed in b/239945371.
    const Curve curve =
        ToCurve({{0.0, -0.2999965018875757, -1.06391622356483e-06,
                  1.372758240414e-08, -1.182799456134644e-07},
                 {0.1500000915356911, -0.1499964103518845,
                  -1.063187735959823e-06, -4.014420264716225e-09, 0},
                 {0.3000001830713822, 3.68118380655869e-06,
                  -1.0626572524014e-06, 1.108753006074e-08, 0}});

    const double desired_speed = 0.03;
    const double tolerance = 0.01;

    const diff_drive::DynamicLimits sorty_meta_limits(
        diff_drive::Kinematics(WheelVector(0.0625, 0.0625), 0.365),
        WheelVector(-4.0, -4.0), WheelVector(4.0, 4.0),
        WheelVector(-120.0, -120.0), WheelVector(120.0, 120.0),
        ArcVector(-0.3, -0.3), ArcVector(0.06, 0.06), ArcVector(-4.0, -8.0),
        ArcVector(4.0, 8.0));
    const diff_drive::TrajectoryLimits sorty_meta_traj_limits(sorty_meta_limits,
                                                              0.05);
    double sub_desired_speed;

    Trajectory clothoid_traj(128);
    EXPECT_TRUE(FindClothoidTrajectoryForCurve(
        curve, desired_speed, tolerance, sorty_meta_limits,
        sorty_meta_traj_limits, &clothoid_traj, &sub_desired_speed))
        << "Trajectory =\n"
        << clothoid_traj;

    EXPECT_TRUE(clothoid_traj.HasContinuousPosition());
    EXPECT_TRUE(clothoid_traj.HasMonotonicTimeValues());
    EXPECT_THAT(clothoid_traj, SatisfiesLimits(sorty_meta_limits));
    EXPECT_LT(clothoid_traj.GetTimeSpan().Length(), 20.0) << "Trajectory =\n"
                                                          << clothoid_traj;
  }
  {
    const Curve curve =
        ToCurve({{0.0, -0.299996501498872, -1.070765326792455e-06,
                  1.373279334693223e-08, 0},
                 {0.1500000920295069, -0.1499964094693652,
                  -1.069246446493957e-06, 6.518931541332057e-09, 0},
                 {0.3000001840590137, 3.682560141743396e-06,
                  -1.067930752433888e-06, 1.102364516336607e-08, 0}});

    const double desired_speed = 0.03;
    const double tolerance = 0.01;

    const diff_drive::DynamicLimits sorty_meta_limits(
        diff_drive::Kinematics(WheelVector(0.0625, 0.0625), 0.365),
        WheelVector(-4.0, -4.0), WheelVector(4.0, 4.0),
        WheelVector(-120.0, -120.0), WheelVector(120.0, 120.0),
        ArcVector(-0.3, -0.3), ArcVector(0.06, 0.06), ArcVector(-4.0, -8.0),
        ArcVector(4.0, 8.0));
    const diff_drive::TrajectoryLimits sorty_meta_traj_limits(sorty_meta_limits,
                                                              0.05);
    double sub_desired_speed;

    Trajectory clothoid_traj(128);
    EXPECT_TRUE(FindClothoidTrajectoryForCurve(
        curve, desired_speed, tolerance, sorty_meta_limits,
        sorty_meta_traj_limits, &clothoid_traj, &sub_desired_speed))
        << "Trajectory =\n"
        << clothoid_traj;

    EXPECT_TRUE(clothoid_traj.HasContinuousPosition());
    EXPECT_TRUE(clothoid_traj.HasMonotonicTimeValues());
    EXPECT_THAT(clothoid_traj, SatisfiesLimits(sorty_meta_limits));
    EXPECT_LT(clothoid_traj.GetTimeSpan().Length(), 20.0) << "Trajectory =\n"
                                                          << clothoid_traj;
  }
}

TEST(CurveTrajectoryUtils, FindClothoidTrajectoryForCurveBackwards) {
  const Curve curve = ToCurve(
      {{0.0, 0.99109364784454667685, 0.34304851522696194754,
        -1.6071826053644664434, 0.0},
       {0.24999999999999997224, 0.98199908532026214392, 0.093213992127187228487,
        -1.6071826053644664434, -4.0000000000000017764},
       {0.64269908169872402848, 0.72306999969620300295, -0.14752596844830292988,
        3.1052063750202232306, 0.0},
       {0.89269908169872402848, 0.47323547659642828389, -0.13843140592401845246,
        3.1052063750202232306, 0.0},
       {1.3926990816987241395, -0.026433569603121223612,
        -0.12024228087544940047, 3.1052063750202232306, 0.0}});
  const double desired_speed = -0.8;
  const double tolerance = 0.001;
  const DynamicLimits limits{Kinematics{WheelVector{0.0625, 0.0625}, 0.365},
                             WheelVector{12.6, 12.6}, WheelVector{120, 120},
                             ArcVector{0.8, 1.2}, ArcVector{4.0, 8.0}};
  const TrajectoryLimits traj_limits{limits, 0.008};

  double sub_desired_speed;
  Trajectory clothoid_traj(128);
  EXPECT_TRUE(FindClothoidTrajectoryForCurve(
      curve, desired_speed, tolerance, limits, traj_limits, &clothoid_traj,
      &sub_desired_speed))
      << "Trajectory =\n"
      << clothoid_traj;

  EXPECT_TRUE(clothoid_traj.HasContinuousPosition());
  EXPECT_TRUE(clothoid_traj.HasMonotonicTimeValues());
  EXPECT_THAT(clothoid_traj, SatisfiesLimits(limits));
  EXPECT_LT(clothoid_traj.GetTimeSpan().Length(), 20.0) << "Trajectory =\n"
                                                        << clothoid_traj;
}

}  // namespace
}  // namespace mobility::diff_drive
