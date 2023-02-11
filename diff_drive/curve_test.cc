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

#include "diff_drive/curve.h"

#include <cmath>
#include <vector>

#include "diff_drive/test_curves.h"
#include "diff_drive/test_utils.h"
#include "gtest/gtest.h"

namespace mobility::diff_drive {
namespace {

using eigenmath::testing::IsApprox;
using testing::CurveEvaluatesTo;
using testing::IsCurvePointApprox;
using testing::IsCurvePtAndCordApprox;
using testing::kBigEpsilon;
using testing::kEpsilon;
using testing::kTestCurve;
using testing::kTestCurveEval;
using testing::ToCurve;
using testing::ToCurvePoint;
using testing::ToCurvePtAndCord;

TEST(Curve, BasicMemberFunctions) {
  EXPECT_FALSE(kTestCurve.IsEmpty());
  EXPECT_TRUE(kTestCurve.IsSane());
  Interval<double> cord_span = kTestCurve.GetCordLengthSpan();
  EXPECT_NEAR(cord_span.min(), 0.0, kEpsilon);
  EXPECT_NEAR(cord_span.max(), 3.0, kEpsilon);
}

TEST(Curve, EvaluateAtS) {
  for (int i = 0; i < kTestCurveEval.size(); ++i) {
    CurvePoint cur_pt = kTestCurve.Evaluate(kTestCurveEval[i].s);
    EXPECT_THAT(cur_pt, IsCurvePointApprox(ToCurvePoint(kTestCurveEval[i]),
                                           kBigEpsilon))
        << "at index " << i;
  }
  // Also test matcher:
  EXPECT_THAT(kTestCurve, CurveEvaluatesTo(kTestCurveEval, kBigEpsilon));
}

TEST(Curve, HasContinuousPosition) {
  EXPECT_TRUE(kTestCurve.HasContinuousPosition());
  Curve discontinuous_curve = kTestCurve;
  EXPECT_TRUE(
      discontinuous_curve.AddPoint(kTestCurve.GetCordLengthSpan().max() + 1.0,
                                   kTestCurve.GetFinish().point));
  EXPECT_FALSE(discontinuous_curve.HasContinuousPosition());
}

TEST(Curve, ApplyCordLengthShift) {
  constexpr double kCordLengthShift = 0.5;
  Curve shifted_curve = kTestCurve;
  shifted_curve.ApplyCordLengthShift(kCordLengthShift);
  for (int i = 0; i < kTestCurveEval.size(); ++i) {
    CurvePoint cur_pt =
        shifted_curve.Evaluate(kTestCurveEval[i].s + kCordLengthShift);
    EXPECT_THAT(cur_pt, IsCurvePointApprox(ToCurvePoint(kTestCurveEval[i]),
                                           kBigEpsilon))
        << "at index " << i;
  }
}

TEST(Curve, ApplyTransform) {
  const eigenmath::Pose2d kPoseShift({1.0, 2.0}, 2.0);
  Curve shifted_curve = kTestCurve;
  shifted_curve.ApplyTransform(kPoseShift);
  for (int i = 0; i < kTestCurveEval.size(); ++i) {
    const eigenmath::Pose2d expected_pose =
        kPoseShift *
        eigenmath::Pose2d{{kTestCurveEval[i].x, kTestCurveEval[i].y},
                          kTestCurveEval[i].theta};
    CurvePoint cur_pt = shifted_curve.Evaluate(kTestCurveEval[i].s);
    EXPECT_THAT(cur_pt, IsCurvePointApprox(
                            CurvePoint(expected_pose, kTestCurveEval[i].kappa),
                            kBigEpsilon))
        << "at index " << i;
  }
}

TEST(Curve, GetStartAndFinish) {
  EXPECT_THAT(kTestCurve.GetStart(),
              IsCurvePtAndCordApprox(ToCurvePtAndCord(kTestCurveEval.front()),
                                     kBigEpsilon));
  EXPECT_THAT(kTestCurve.GetFinish(),
              IsCurvePtAndCordApprox(ToCurvePtAndCord(kTestCurveEval.back()),
                                     kBigEpsilon));
}

TEST(Curve, CordLengthIterator) {
  double prev_s = 0.0;
  Curve::CordLengthIterator it = kTestCurve.BeginInCordLength();
  Curve::CordLengthIterator it_end = kTestCurve.EndInCordLength();
  for (int i = 0; i < kTestCurveEval.size(); ++i) {
    auto it_pre = (kTestCurveEval[i].s - prev_s) + it;
    auto it_post = it + (kTestCurveEval[i].s - prev_s);
    it += kTestCurveEval[i].s - prev_s;
    if (i == kTestCurveEval.size() - 1) {
      EXPECT_FALSE(it < it_end);
      EXPECT_FALSE(it_pre < it_end);
      EXPECT_FALSE(it_post < it_end);
    } else {
      EXPECT_TRUE(it < it_end);
      EXPECT_TRUE(it_pre < it_end);
      EXPECT_TRUE(it_post < it_end);
    }

    CurvePoint cur_pt = it.GetPoint();
    EXPECT_THAT(cur_pt, IsCurvePointApprox(ToCurvePoint(kTestCurveEval[i]),
                                           kBigEpsilon))
        << "at index " << i;

    CurvePoint cur_pt_pre = it_pre.GetPoint();
    EXPECT_THAT(cur_pt_pre, IsCurvePointApprox(ToCurvePoint(kTestCurveEval[i]),
                                               kBigEpsilon))
        << "at index " << i;

    CurvePoint cur_pt_post = it_post.GetPoint();
    EXPECT_THAT(cur_pt_post, IsCurvePointApprox(ToCurvePoint(kTestCurveEval[i]),
                                                kBigEpsilon))
        << "at index " << i;

    prev_s = kTestCurveEval[i].s;
  }
  // Check if termination works past the end too:
  it += kBigEpsilon;
  EXPECT_FALSE(it < it_end);
}

TEST(Curve, CurvePointIterator) {
  const testing::RawCurvePointAndCord expected_curve_pts[6] = {
      {0.0, 0.0, 0.0, 0.0, 2 * M_PI},
      {0.25, 0.5 / M_PI, 0.5 / M_PI, M_PI_2, 0.0},
      {1.25, 0.5 / M_PI, 1.0 + 0.5 / M_PI, M_PI_2, -2 * M_PI},
      {1.75, 1.5 / M_PI, 1.0 + 0.5 / M_PI, -M_PI_2, 0.0},
      {2.75, 1.5 / M_PI, 0.5 / M_PI, -M_PI_2, 2 * M_PI},
      {3.0, 2.0 / M_PI, 0.0, 0.0, 0.0}};

  int i = 0;
  for (auto& pt : kTestCurve.GetCurvePointIteratorRange()) {
    EXPECT_THAT(pt, IsCurvePtAndCordApprox(
                        ToCurvePtAndCord(expected_curve_pts[i]), kBigEpsilon));
    ++i;
  }
}

TEST(Curve, ClearedCurve) {
  Curve cleared_curve = kTestCurve;
  EXPECT_FALSE(cleared_curve.IsEmpty());
  EXPECT_TRUE(cleared_curve.IsSane());
  cleared_curve.Clear();
  EXPECT_TRUE(cleared_curve.IsEmpty());
  EXPECT_FALSE(cleared_curve.IsSane());

  Interval<double> cord_span = cleared_curve.GetCordLengthSpan();
  EXPECT_TRUE(cord_span.Empty());

  // Check if termination works for empty cord-iteration:
  EXPECT_FALSE(cleared_curve.BeginInCordLength() <
               cleared_curve.EndInCordLength());

  CurvePtAndCord start = cleared_curve.GetStart();
  EXPECT_NEAR(start.cord_length, 0.0, kBigEpsilon);
  EXPECT_NEAR(start.point.GetPose().translation().x(), 0.0, kBigEpsilon);
  EXPECT_NEAR(start.point.GetPose().translation().y(), 0.0, kBigEpsilon);
  EXPECT_NEAR(start.point.GetPose().angle(), 0.0, kBigEpsilon);

  CurvePtAndCord finish = cleared_curve.GetFinish();
  EXPECT_NEAR(finish.cord_length, 0.0, kBigEpsilon);
  EXPECT_NEAR(finish.point.GetPose().translation().x(), 0.0, kBigEpsilon);
  EXPECT_NEAR(finish.point.GetPose().translation().y(), 0.0, kBigEpsilon);
  EXPECT_NEAR(finish.point.GetPose().angle(), 0.0, kBigEpsilon);
}

TEST(Curve, SetCapacity) {
  Curve new_curve(10);
  EXPECT_GE(new_curve.GetCapacity(), 10);
  new_curve.SetMinCapacity(5);
  EXPECT_GE(new_curve.GetCapacity(), 10);
  new_curve.SetMinCapacity(50);
  EXPECT_GE(new_curve.GetCapacity(), 50);
}

TEST(Curve, AppendLimitedCapacity) {
  Curve curve1 = ToCurve({{0.0, 0.0, 0.0, 0.0, 0.0},
                          {1.0, 1.0, 0.0, 0.0, 0.0},
                          {2.0, 2.0, 0.0, 0.0, 0.0}});

  Curve curve2 =
      ToCurve({{2.0, 2.0, 0.0, 0.0, 0.0}, {3.0, 3.0, 0.0, 0.0, 0.0}});

  EXPECT_FALSE(curve1.AppendCurve(curve2));

  Curve curve3(4);
  EXPECT_TRUE(curve3.AppendCurve(curve1));
  EXPECT_TRUE(curve3.AppendCurve(curve2));
  EXPECT_EQ(curve3.GetSize(), 4);

  Interval<double> cord_span = curve3.GetCordLengthSpan();
  EXPECT_NEAR(cord_span.min(), 0.0, kEpsilon);
  EXPECT_NEAR(cord_span.max(), 3.0, kEpsilon);
}

TEST(Curve, PrependLimitedCapacity) {
  Curve curve1 = ToCurve({{0.0, 0.0, 0.0, 0.0, 0.0},
                          {1.0, 1.0, 0.0, 0.0, 0.0},
                          {2.0, 2.0, 0.0, 0.0, 0.0}});

  Curve curve2 =
      ToCurve({{2.0, 2.0, 0.0, 0.0, 0.0}, {3.0, 3.0, 0.0, 0.0, 0.0}});

  EXPECT_FALSE(curve2.PrependCurve(curve1));

  Curve curve3(4);
  EXPECT_TRUE(curve3.PrependCurve(curve2));
  EXPECT_TRUE(curve3.PrependCurve(curve1));
  EXPECT_EQ(curve3.GetSize(), 4);

  Interval<double> cord_span = curve3.GetCordLengthSpan();
  EXPECT_NEAR(cord_span.min(), 0.0, kEpsilon);
  EXPECT_NEAR(cord_span.max(), 3.0, kEpsilon);
}

TEST(Curve, SplitAndCombineWithAppendAndPrepend) {
  Curve test_curve_part1 = kTestCurve;
  test_curve_part1.TruncateTo(Interval<double>(0.0, 1.25));

  Curve test_curve_part2 = kTestCurve;
  test_curve_part2.TruncateTo(Interval<double>(1.25, 3.0));

  Curve appended_curve(kTestCurve.GetCapacity());
  EXPECT_TRUE(appended_curve.AppendCurve(test_curve_part1));
  EXPECT_TRUE(appended_curve.AppendCurve(test_curve_part2));
  EXPECT_TRUE(kTestCurve == appended_curve);

  Curve prepended_curve(kTestCurve.GetCapacity());
  EXPECT_TRUE(prepended_curve.PrependCurve(test_curve_part2));
  EXPECT_TRUE(prepended_curve.PrependCurve(test_curve_part1));
  EXPECT_TRUE(kTestCurve == prepended_curve);
}

TEST(Curve, MatchPointToCurve) {
  Interval<double> cord_span = kTestCurve.GetCordLengthSpan();
  double min_distance = 0.0;

  auto match1 = kTestCurve.FindClosestMatchToPoint({-1.0, 0.0}, &min_distance);
  EXPECT_NEAR(match1.cord_length, cord_span.min(), kBigEpsilon);
  EXPECT_THAT(match1.point.GetPose().translation(),
              IsApprox(eigenmath::Vector2d::Zero(), kBigEpsilon));
  EXPECT_NEAR(min_distance, 1.0, kBigEpsilon);

  auto match2 = kTestCurve.FindClosestMatchToPoint({0.0, 0.1}, &min_distance);
  EXPECT_NEAR(match2.cord_length, cord_span.min(), kBigEpsilon);
  EXPECT_THAT(match2.point.GetPose().translation(),
              IsApprox(eigenmath::Vector2d::Zero(), kBigEpsilon));
  EXPECT_NEAR(min_distance, 0.1, kBigEpsilon);

  auto match3 = kTestCurve.FindClosestMatchToPoint({2.0 / M_PI + 1.0, 0.0},
                                                   &min_distance);
  EXPECT_NEAR(match3.cord_length, cord_span.max(), kBigEpsilon);
  EXPECT_THAT(match3.point.GetPose().translation(),
              IsApprox(eigenmath::Vector2d(2.0 / M_PI, 0.0), kBigEpsilon));
  EXPECT_NEAR(min_distance, 1.0, kBigEpsilon);

  auto match4 =
      kTestCurve.FindClosestMatchToPoint({2.0 / M_PI, 0.1}, &min_distance);
  EXPECT_NEAR(match4.cord_length, cord_span.max(), kBigEpsilon);
  EXPECT_THAT(match4.point.GetPose().translation(),
              IsApprox(eigenmath::Vector2d(2.0 / M_PI, 0.0), kBigEpsilon));
  EXPECT_NEAR(min_distance, 0.1, kBigEpsilon);

  auto match5 =
      kTestCurve.FindClosestMatchToPoint({kEpsilon, 0.5 / M_PI}, &min_distance);
  EXPECT_NEAR(match5.cord_length, cord_span.min() + 0.25, kBigEpsilon);
  EXPECT_THAT(
      match5.point.GetPose().translation(),
      IsApprox(eigenmath::Vector2d(0.5 / M_PI, 0.5 / M_PI), kBigEpsilon));
  EXPECT_NEAR(min_distance, 0.5 / M_PI, kBigEpsilon);

  auto match6 = kTestCurve.FindClosestMatchToPoint({0.0, 0.5 / M_PI + 1.0},
                                                   &min_distance);
  EXPECT_NEAR(match6.cord_length, cord_span.min() + 1.25, kBigEpsilon);
  EXPECT_THAT(
      match6.point.GetPose().translation(),
      IsApprox(eigenmath::Vector2d(0.5 / M_PI, 1.0 + 0.5 / M_PI), kBigEpsilon));
  EXPECT_NEAR(min_distance, 0.5 / M_PI, kBigEpsilon);
}

TEST(Curve, MatchPointStraightLine) {
  Curve curve = ToCurve({{0.0, 0.0, 0.0, 0.0, 0.0},
                         {1.0, 1.0, 0.0, 0.0, 0.0},
                         {2.0, 2.0, 0.0, 0.0, 0.0}});

  auto match1 = curve.FindClosestMatchToPoint({-1.0, 0.0}, nullptr);
  EXPECT_NEAR(match1.cord_length, 0.0, kBigEpsilon);
  EXPECT_THAT(match1.point.GetPose().translation(),
              IsApprox(eigenmath::Vector2d::Zero(), kBigEpsilon));

  auto match2 = curve.FindClosestMatchToPoint({0.0, 0.1}, nullptr);
  EXPECT_NEAR(match2.cord_length, 0.0, kBigEpsilon);
  EXPECT_THAT(match2.point.GetPose().translation(),
              IsApprox(eigenmath::Vector2d::Zero(), kBigEpsilon));

  auto match3 = curve.FindClosestMatchToPoint({1.0, 0.0}, nullptr);
  EXPECT_NEAR(match3.cord_length, 1.0, kBigEpsilon);
  EXPECT_THAT(match3.point.GetPose().translation(),
              IsApprox(eigenmath::Vector2d(1.0, 0.0), kBigEpsilon));

  auto match4 = curve.FindClosestMatchToPoint({2.0, 0.0}, nullptr);
  EXPECT_NEAR(match4.cord_length, 2.0, kBigEpsilon);
  EXPECT_THAT(match4.point.GetPose().translation(),
              IsApprox(eigenmath::Vector2d(2.0, 0.0), kBigEpsilon));

  auto match5 = curve.FindClosestMatchToPoint({3.0, 0.0}, nullptr);
  EXPECT_NEAR(match5.cord_length, 2.0, kBigEpsilon);
  EXPECT_THAT(match5.point.GetPose().translation(),
              IsApprox(eigenmath::Vector2d(2.0, 0.0), kBigEpsilon));
}

TEST(Curve, TruncateTo) {
  Curve curve = ToCurve({{0.0, 0.0, 0.0, 0.0, 0.0},
                         {1.0, 1.0, 0.0, 0.0, 0.0},
                         {2.0, 2.0, 0.0, 0.0, 0.0}});

  curve.TruncateTo(1.5);
  EXPECT_EQ(curve.GetSize(), 3);

  auto curve_it = curve.BeginCurvePoint();
  auto first_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, 0.0, kEpsilon);
  EXPECT_THAT(first_pt, IsCurvePointApprox(
                            ToCurvePoint({0.0, 0.0, 0.0, 0.0, 0.0}), kEpsilon));

  ++curve_it;
  auto second_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, 1.0, kEpsilon);
  EXPECT_THAT(
      second_pt,
      IsCurvePointApprox(ToCurvePoint({1.0, 1.0, 0.0, 0.0, 0.0}), kEpsilon));

  ++curve_it;
  auto third_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, 1.5, kEpsilon);
  EXPECT_THAT(third_pt, IsCurvePointApprox(
                            ToCurvePoint({1.5, 1.5, 0.0, 0.0, 0.0}), kEpsilon));

  curve.TruncateTo(-0.1);
  EXPECT_EQ(curve.GetSize(), 0);
}

TEST(Curve, TruncateToPredicateFailure) {
  Curve curve = ToCurve({{0.0, 0.0, 0.0, 0.0, 0.0},
                         {1.0, 1.0, 0.0, 0.0, 0.0},
                         {2.0, 2.0, 0.0, 0.0, 0.0}});

  curve.TruncateToPredicateFailure(0.01, [](const CurvePoint& pt) {
    return pt.GetPose().translation().x() < 1.5;
  });
  EXPECT_EQ(curve.GetSize(), 3);

  auto curve_it = curve.BeginCurvePoint();
  auto first_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, 0.0, kEpsilon);
  EXPECT_THAT(first_pt, IsCurvePointApprox(
                            ToCurvePoint({0.0, 0.0, 0.0, 0.0, 0.0}), kEpsilon));

  ++curve_it;
  auto second_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, 1.0, kEpsilon);
  EXPECT_THAT(
      second_pt,
      IsCurvePointApprox(ToCurvePoint({1.0, 1.0, 0.0, 0.0, 0.0}), kEpsilon));

  ++curve_it;
  auto third_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, 1.5, 0.01);
  EXPECT_THAT(third_pt, IsCurvePointApprox(
                            ToCurvePoint({1.5, 1.5, 0.0, 0.0, 0.0}), 0.01));
}

TEST(Curve, CutCornersUnlessPredicateFails) {
  Curve curve(8);
  curve = ToCurve({{0.0, 0.0, 0.0, 0.0, 0.0},
                   {1.0, 1.0, 0.0, M_PI_2, 0.0},
                   {2.0, 1.0, 1.0, 0.0, 0.0},
                   {3.0, 2.0, 1.0, 0.0, 0.0},
                   {4.0, 3.0, 1.0, 0.0, 0.0}});

  curve.CutCornersUnlessPredicateFails(0.01, [](const CurvePoint& pt) {
    // prevent second corner from being cut:
    return !((pt.GetPose().translation().x() > 1.05) &&
             (pt.GetPose().translation().y() < 0.95));
  });
  EXPECT_EQ(curve.GetSize(), 5);

  auto curve_it = curve.BeginCurvePoint();
  auto first_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, 0.0, kEpsilon);
  EXPECT_THAT(first_pt,
              IsCurvePointApprox(
                  ToCurvePoint({0.0, 0.0, 0.0, M_PI * 0.25, 0.0}), kEpsilon));

  ++curve_it;
  auto second_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, M_SQRT1_2, kEpsilon);
  EXPECT_THAT(
      second_pt,
      IsCurvePointApprox(ToCurvePoint({M_SQRT1_2, 0.5, 0.5, M_PI * 0.25, 0.0}),
                         kEpsilon));

  ++curve_it;
  auto third_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, M_SQRT2, 0.01);
  EXPECT_THAT(third_pt, IsCurvePointApprox(
                            ToCurvePoint({M_SQRT2, 1.0, 1.0, 0.0, 0.0}), 0.01));

  ++curve_it;
  auto fourth_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, 1.0 + M_SQRT2, 0.01);
  EXPECT_THAT(fourth_pt,
              IsCurvePointApprox(
                  ToCurvePoint({1.0 + M_SQRT2, 2.0, 1.0, 0.0, 0.0}), 0.01));

  ++curve_it;
  auto fifth_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, 2.0 + M_SQRT2, 0.01);
  EXPECT_THAT(fifth_pt,
              IsCurvePointApprox(
                  ToCurvePoint({2.0 + M_SQRT2, 3.0, 1.0, 0.0, 0.0}), 0.01));
}

TEST(Curve, CutCorners) {
  Curve curve(8);
  curve = ToCurve({{0.0, 0.0, 0.0, 0.0, 0.0},
                   {1.0, 1.0, 0.0, M_PI_2, 0.0},
                   {2.0, 1.0, 1.0, 0.0, 0.0},
                   {3.0, 2.0, 1.0, M_PI_2, 0.0},
                   {4.0, 2.0, 2.0, M_PI_2, 0.0}});

  curve.CutCorners();
  EXPECT_EQ(curve.GetSize(), 5);

  auto curve_it = curve.BeginCurvePoint();
  auto first_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, 0.0, kEpsilon);
  EXPECT_THAT(first_pt,
              IsCurvePointApprox(
                  ToCurvePoint({0.0, 0.0, 0.0, M_PI * 0.25, 0.0}), kEpsilon));

  ++curve_it;
  auto second_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, M_SQRT1_2, kEpsilon);
  EXPECT_THAT(
      second_pt,
      IsCurvePointApprox(ToCurvePoint({M_SQRT1_2, 0.5, 0.5, 0.32175055, 0.0}),
                         kEpsilon));

  ++curve_it;
  auto third_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, M_SQRT1_2 + std::sqrt(0.625), 0.01);
  EXPECT_THAT(third_pt,
              IsCurvePointApprox(ToCurvePoint({M_SQRT1_2 + std::sqrt(0.625),
                                               1.25, 0.75, 0.32175055, 0.0}),
                                 0.01));

  ++curve_it;
  auto fourth_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, M_SQRT1_2 + 2.0 * std::sqrt(0.625), 0.01);
  EXPECT_THAT(fourth_pt, IsCurvePointApprox(
                             ToCurvePoint({M_SQRT1_2 + 2.0 * std::sqrt(0.625),
                                           2.0, 1.0, M_PI_2, 0.0}),
                             0.01));

  ++curve_it;
  auto fifth_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, 1.0 + M_SQRT1_2 + 2.0 * std::sqrt(0.625),
              0.01);
  EXPECT_THAT(
      fifth_pt,
      IsCurvePointApprox(ToCurvePoint({1.0 + M_SQRT1_2 + 2.0 * std::sqrt(0.625),
                                       2.0, 2.0, M_PI_2, 0.0}),
                         0.01));
}

TEST(Curve, FillWithPolylinePts) {
  Curve curve(8);
  std::vector<eigenmath::Vector2d> pts = {
      {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {2.0, 1.0}, {2.0, 2.0}};
  EXPECT_TRUE(curve.FillWithPolylinePoints(eigenmath::Pose2d(), pts));

  EXPECT_EQ(curve.GetSize(), 5);

  auto curve_it = curve.BeginCurvePoint();
  auto first_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, 0.0, kEpsilon);
  EXPECT_THAT(first_pt, IsCurvePointApprox(
                            ToCurvePoint({0.0, 0.0, 0.0, 0.0, 0.0}), kEpsilon));

  ++curve_it;
  auto second_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, 1.0, kEpsilon);
  EXPECT_THAT(
      second_pt,
      IsCurvePointApprox(ToCurvePoint({1.0, 1.0, 0.0, M_PI_2, 0.0}), kEpsilon));

  ++curve_it;
  auto third_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, 2.0, 0.01);
  EXPECT_THAT(third_pt, IsCurvePointApprox(
                            ToCurvePoint({2.0, 1.0, 1.0, 0.0, 0.0}), 0.01));

  ++curve_it;
  auto fourth_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, 3.0, 0.01);
  EXPECT_THAT(fourth_pt, IsCurvePointApprox(
                             ToCurvePoint({3.0, 2.0, 1.0, M_PI_2, 0.0}), 0.01));

  ++curve_it;
  auto fifth_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, 4.0, 0.01);
  EXPECT_THAT(fifth_pt, IsCurvePointApprox(
                            ToCurvePoint({4.0, 2.0, 2.0, M_PI_2, 0.0}), 0.01));
}

TEST(Curve, FillWithTransformedPolylinePts) {
  Curve curve(8);
  std::vector<eigenmath::Vector2d> pts = {
      {0.0, 0.0}, {1.0, 0.0}, {1.0, 1.0}, {2.0, 1.0}, {2.0, 2.0}};
  eigenmath::Pose2d transform(eigenmath::Vector2d(1.0, 2.0), M_PI_2);
  EXPECT_TRUE(curve.FillWithPolylinePoints(transform, pts));

  EXPECT_EQ(curve.GetSize(), 5);

  auto curve_it = curve.BeginCurvePoint();
  auto first_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, 0.0, kEpsilon);
  EXPECT_THAT(
      first_pt,
      IsCurvePointApprox(ToCurvePoint({0.0, 1.0, 2.0, M_PI_2, 0.0}), kEpsilon));

  ++curve_it;
  auto second_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, 1.0, kEpsilon);
  EXPECT_THAT(
      second_pt,
      IsCurvePointApprox(ToCurvePoint({1.0, 1.0, 3.0, M_PI, 0.0}), kEpsilon));

  ++curve_it;
  auto third_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, 2.0, 0.01);
  EXPECT_THAT(third_pt, IsCurvePointApprox(
                            ToCurvePoint({2.0, 0.0, 3.0, M_PI_2, 0.0}), 0.01));

  ++curve_it;
  auto fourth_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, 3.0, 0.01);
  EXPECT_THAT(fourth_pt, IsCurvePointApprox(
                             ToCurvePoint({3.0, 0.0, 4.0, M_PI, 0.0}), 0.01));

  ++curve_it;
  auto fifth_pt = curve_it->point;
  EXPECT_NEAR(curve_it->cord_length, 4.0, 0.01);
  EXPECT_THAT(fifth_pt, IsCurvePointApprox(
                            ToCurvePoint({4.0, -1.0, 4.0, M_PI, 0.0}), 0.01));
}

}  // namespace
}  // namespace mobility::diff_drive
