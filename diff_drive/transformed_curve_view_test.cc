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

#include "diff_drive/transformed_curve_view.h"

#include <cmath>

#include "diff_drive/curve.h"
#include "diff_drive/test_curves.h"
#include "gtest/gtest.h"

namespace mobility::diff_drive {
namespace {

using eigenmath::testing::IsApprox;
using testing::kBigEpsilon;
using testing::kEpsilon;
using testing::kTestCurve;
using testing::kTestCurveEval;

constexpr double kCordLengthShift = 0.5;
const eigenmath::Pose2d kPoseShift{{1.0, 2.0}, 2.0};

TEST(TransformedCurveView, BasicMemberFunctions) {
  const TransformedCurveView transformed_curve{&kTestCurve, kPoseShift,
                                               kCordLengthShift};
  EXPECT_FALSE(transformed_curve.IsEmpty());
  EXPECT_TRUE(transformed_curve.IsSane());
  Interval<double> cord_span = transformed_curve.GetCordLengthSpan();
  EXPECT_NEAR(cord_span.min(), 0.5, kEpsilon);
  EXPECT_NEAR(cord_span.max(), 3.5, kEpsilon);
  EXPECT_TRUE(transformed_curve.HasContinuousPosition());
}

TEST(TransformedCurveView, EvaluateAtS) {
  const TransformedCurveView transformed_curve{&kTestCurve, kPoseShift,
                                               kCordLengthShift};
  for (int i = 0; i < kTestCurveEval.size(); ++i) {
    const eigenmath::Pose2d expected_pose =
        kPoseShift *
        eigenmath::Pose2d{{kTestCurveEval[i].x, kTestCurveEval[i].y},
                          kTestCurveEval[i].theta};
    CurvePoint cur_pt =
        transformed_curve.Evaluate(kTestCurveEval[i].s + kCordLengthShift);
    EXPECT_THAT(cur_pt.GetPose(), IsApprox(expected_pose, kBigEpsilon));
  }
}

TEST(TransformedCurveView, ApplyCordLengthShift) {
  const TransformedCurveView transformed_curve{&kTestCurve, kPoseShift,
                                               kCordLengthShift};
  constexpr double kAddedCordLengthShift = 0.25;
  TransformedCurveView shifted_curve = transformed_curve;
  shifted_curve.ApplyCordLengthShift(kAddedCordLengthShift);
  for (int i = 0; i < kTestCurveEval.size(); ++i) {
    const eigenmath::Pose2d expected_pose =
        kPoseShift *
        eigenmath::Pose2d{{kTestCurveEval[i].x, kTestCurveEval[i].y},
                          kTestCurveEval[i].theta};
    CurvePoint cur_pt = shifted_curve.Evaluate(
        kTestCurveEval[i].s + kCordLengthShift + kAddedCordLengthShift);
    EXPECT_THAT(cur_pt.GetPose(), IsApprox(expected_pose, kBigEpsilon));
  }
}

TEST(TransformedCurveView, ApplyTransform) {
  const TransformedCurveView transformed_curve{&kTestCurve, kPoseShift,
                                               kCordLengthShift};
  const eigenmath::Pose2d kAddedPoseShift{{0.0, -1.0}, 0.5};
  TransformedCurveView shifted_curve = transformed_curve;
  shifted_curve.ApplyTransform(kAddedPoseShift);
  for (int i = 0; i < kTestCurveEval.size(); ++i) {
    const eigenmath::Pose2d expected_pose =
        kAddedPoseShift * kPoseShift *
        eigenmath::Pose2d{{kTestCurveEval[i].x, kTestCurveEval[i].y},
                          kTestCurveEval[i].theta};
    CurvePoint cur_pt =
        shifted_curve.Evaluate(kTestCurveEval[i].s + kCordLengthShift);
    EXPECT_THAT(cur_pt.GetPose(), IsApprox(expected_pose, kBigEpsilon));
  }
}

TEST(TransformedCurveView, GetStartAndFinish) {
  const TransformedCurveView transformed_curve{&kTestCurve, kPoseShift,
                                               kCordLengthShift};
  const eigenmath::Pose2d expected_start =
      kPoseShift *
      eigenmath::Pose2d{{kTestCurveEval.front().x, kTestCurveEval.front().y},
                        kTestCurveEval.front().theta};
  CurvePtAndCord start = transformed_curve.GetStart();
  EXPECT_NEAR(start.cord_length, kTestCurveEval.front().s + kCordLengthShift,
              kBigEpsilon);
  EXPECT_THAT(start.point.GetPose(), IsApprox(expected_start, kBigEpsilon));

  const eigenmath::Pose2d expected_finish =
      kPoseShift *
      eigenmath::Pose2d{{kTestCurveEval.back().x, kTestCurveEval.back().y},
                        kTestCurveEval.back().theta};
  CurvePtAndCord finish = transformed_curve.GetFinish();
  EXPECT_NEAR(finish.cord_length, kTestCurveEval.back().s + kCordLengthShift,
              kBigEpsilon);
  EXPECT_THAT(finish.point.GetPose(), IsApprox(expected_finish, kBigEpsilon));
}

TEST(TransformedCurveView, MatchPointToCurve) {
  const TransformedCurveView transformed_curve{&kTestCurve, kPoseShift,
                                               kCordLengthShift};
  Interval<double> cord_span = transformed_curve.GetCordLengthSpan();

  auto match1 = transformed_curve.FindClosestMatchToPoint(
      kPoseShift * eigenmath::Vector2d(-1.0, 0.0), nullptr);
  EXPECT_NEAR(match1.cord_length, cord_span.min(), kBigEpsilon);
  eigenmath::Vector2d expected_point =
      kPoseShift * eigenmath::Vector2d{0.0, 0.0};
  EXPECT_THAT(match1.point.GetPose().translation(),
              IsApprox(expected_point, kBigEpsilon));

  auto match2 = transformed_curve.FindClosestMatchToPoint(
      kPoseShift * eigenmath::Vector2d(0.0, 0.1), nullptr);
  EXPECT_NEAR(match2.cord_length, cord_span.min(), kBigEpsilon);
  expected_point = kPoseShift * eigenmath::Vector2d{0.0, 0.0};
  EXPECT_THAT(match2.point.GetPose().translation(),
              IsApprox(expected_point, kBigEpsilon));

  auto match3 = transformed_curve.FindClosestMatchToPoint(
      kPoseShift * eigenmath::Vector2d(2.0 / M_PI + 1.0, 0.0), nullptr);
  EXPECT_NEAR(match3.cord_length, cord_span.max(), kBigEpsilon);
  expected_point = kPoseShift * eigenmath::Vector2d{2.0 / M_PI, 0.0};
  EXPECT_THAT(match3.point.GetPose().translation(),
              IsApprox(expected_point, kBigEpsilon));

  auto match4 = transformed_curve.FindClosestMatchToPoint(
      kPoseShift * eigenmath::Vector2d(2.0 / M_PI, 0.1), nullptr);
  EXPECT_NEAR(match4.cord_length, cord_span.max(), kBigEpsilon);
  expected_point = kPoseShift * eigenmath::Vector2d{2.0 / M_PI, 0.0};
  EXPECT_THAT(match4.point.GetPose().translation(),
              IsApprox(expected_point, kBigEpsilon));

  auto match5 = transformed_curve.FindClosestMatchToPoint(
      kPoseShift * eigenmath::Vector2d(kEpsilon, 0.5 / M_PI), nullptr);
  EXPECT_NEAR(match5.cord_length, cord_span.min() + 0.25, kBigEpsilon);
  expected_point = kPoseShift * eigenmath::Vector2d{0.5 / M_PI, 0.5 / M_PI};
  EXPECT_THAT(match5.point.GetPose().translation(),
              IsApprox(expected_point, kBigEpsilon));

  auto match6 = transformed_curve.FindClosestMatchToPoint(
      kPoseShift * eigenmath::Vector2d(0.0, 0.5 / M_PI + 1.0), nullptr);
  EXPECT_NEAR(match6.cord_length, cord_span.min() + 1.25, kBigEpsilon);
  expected_point =
      kPoseShift * eigenmath::Vector2d{0.5 / M_PI, 1.0 + 0.5 / M_PI};
  EXPECT_THAT(match6.point.GetPose().translation(),
              IsApprox(expected_point, kBigEpsilon));
}

}  // namespace
}  // namespace mobility::diff_drive
