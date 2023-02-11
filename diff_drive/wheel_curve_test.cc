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

#include "diff_drive/wheel_curve.h"

#include <cmath>
#include <vector>

#include "diff_drive/test_trajectories.h"
#include "diff_drive/test_utils.h"
#include "diff_drive/test_wheel_curves.h"
#include "gtest/gtest.h"

namespace mobility::diff_drive {
namespace {

using eigenmath::testing::IsApprox;
using testing::IsWheelCurveApprox;
using testing::IsWheelStateAndTotalMotionApprox;
using testing::IsWheelStateApprox;
using testing::kBigEpsilon;
using testing::kEpsilon;
using testing::kTestKinematics;
using testing::kTestWheelCurve;
using testing::kTestWheelCurveEval;
using testing::kTestWheelCurveKnots;
using testing::ToWheelCurve;
using testing::ToWheelState;
using testing::ToWheelStateAndTotalMotion;
using testing::WheelCurveEvaluatesTo;

TEST(WheelCurve, BasicMemberFunctions) {
  EXPECT_FALSE(kTestWheelCurve.IsEmpty());
  EXPECT_TRUE(kTestWheelCurve.IsSane());
  Interval<double> total_motion_span = kTestWheelCurve.GetTotalMotionSpan();
  EXPECT_NEAR(total_motion_span.min(), 0.0, kEpsilon);
  EXPECT_NEAR(total_motion_span.max(), 2.0 * M_PI + 2.0, kEpsilon);
}

TEST(WheelCurve, EvaluateAtW) {
  for (int i = 0; i < kTestWheelCurveEval.size(); ++i) {
    WheelState cur_pt = kTestWheelCurve.Evaluate(kTestWheelCurveEval[i].w);
    EXPECT_THAT(cur_pt, IsWheelStateApprox(ToWheelState(kTestWheelCurveEval[i]),
                                           kBigEpsilon))
        << "at index " << i;
  }
  // Also test matcher:
  EXPECT_THAT(kTestWheelCurve,
              WheelCurveEvaluatesTo(kTestWheelCurveEval, kBigEpsilon));
}

TEST(WheelCurve, HasContinuousPosition) {
  EXPECT_TRUE(kTestWheelCurve.HasContinuousPosition());
  WheelCurve discontinuous_curve = kTestWheelCurve;
  EXPECT_TRUE(discontinuous_curve.AddPoint(
      kTestWheelCurve.GetTotalMotionSpan().max() + 1.0,
      kTestWheelCurve.GetFinish().state));
  EXPECT_FALSE(discontinuous_curve.HasContinuousPosition());
}

TEST(WheelCurve, HasMonotonicTotalMotion) {
  EXPECT_TRUE(kTestWheelCurve.HasContinuousPosition());
  WheelCurve test_curve = kTestWheelCurve;
  EXPECT_FALSE(
      test_curve.AddPoint(kTestWheelCurve.GetTotalMotionSpan().max() - 1.0,
                          kTestWheelCurve.GetFinish().state));
  EXPECT_TRUE(test_curve.HasMonotonicTotalMotion());
}

TEST(WheelCurve, GetMaxCurvature) {
  EXPECT_NEAR(kTestWheelCurve.GetMaxCurvature(kTestKinematics.GetWheelBase()),
              2.0 * M_PI, kEpsilon);
}

TEST(WheelCurve, ApplyTotalMotionShift) {
  constexpr double kTotalMotionShift = 0.5;
  WheelCurve shifted_curve = kTestWheelCurve;
  shifted_curve.ApplyTotalMotionShift(kTotalMotionShift);
  for (int i = 0; i < kTestWheelCurveEval.size(); ++i) {
    WheelState cur_pt =
        shifted_curve.Evaluate(kTestWheelCurveEval[i].w + kTotalMotionShift);
    EXPECT_THAT(cur_pt, IsWheelStateApprox(ToWheelState(kTestWheelCurveEval[i]),
                                           kBigEpsilon))
        << "at index " << i;
  }
}

TEST(WheelCurve, ApplyWheelPositionsShift) {
  const WheelVector kWheelPositionsShift = {0.5, 0.75};
  WheelCurve shifted_curve = kTestWheelCurve;
  shifted_curve.ApplyWheelPositionsShift(kWheelPositionsShift);
  for (int i = 0; i < kTestWheelCurveEval.size(); ++i) {
    WheelState cur_pt = shifted_curve.Evaluate(kTestWheelCurveEval[i].w);
    WheelState expected_pt = ToWheelState(kTestWheelCurveEval[i]);
    expected_pt.SetPositions(expected_pt.GetPositions() + kWheelPositionsShift);
    EXPECT_THAT(cur_pt, IsWheelStateApprox(expected_pt, kBigEpsilon))
        << "at index " << i;
  }
}

TEST(WheelCurve, GetStartAndFinish) {
  EXPECT_THAT(kTestWheelCurve.GetStart(),
              IsWheelStateAndTotalMotionApprox(
                  ToWheelStateAndTotalMotion(kTestWheelCurveEval.front()),
                  kBigEpsilon));
  EXPECT_THAT(
      kTestWheelCurve.GetFinish(),
      IsWheelStateAndTotalMotionApprox(
          ToWheelStateAndTotalMotion(kTestWheelCurveEval.back()), kBigEpsilon));
}

TEST(WheelCurve, TotalMotionIterator) {
  double prev_w = 0.0;
  WheelCurve::TotalMotionIterator it = kTestWheelCurve.BeginInTotalMotion();
  WheelCurve::TotalMotionIterator it_end = kTestWheelCurve.EndInTotalMotion();
  for (int i = 0; i < kTestWheelCurveEval.size(); ++i) {
    auto it_pre = (kTestWheelCurveEval[i].w - prev_w) + it;
    auto it_post = it + (kTestWheelCurveEval[i].w - prev_w);
    it += kTestWheelCurveEval[i].w - prev_w;
    if (i == kTestWheelCurveEval.size() - 1) {
      EXPECT_FALSE(it < it_end);
      EXPECT_FALSE(it_pre < it_end);
      EXPECT_FALSE(it_post < it_end);
    } else {
      EXPECT_TRUE(it < it_end);
      EXPECT_TRUE(it_pre < it_end);
      EXPECT_TRUE(it_post < it_end);
    }

    WheelState cur_pt = it.GetState();
    EXPECT_THAT(cur_pt, IsWheelStateApprox(ToWheelState(kTestWheelCurveEval[i]),
                                           kBigEpsilon))
        << "at index " << i;

    WheelState cur_pt_pre = it_pre.GetState();
    EXPECT_THAT(
        cur_pt_pre,
        IsWheelStateApprox(ToWheelState(kTestWheelCurveEval[i]), kBigEpsilon))
        << "at index " << i;

    WheelState cur_pt_post = it_post.GetState();
    EXPECT_THAT(
        cur_pt_post,
        IsWheelStateApprox(ToWheelState(kTestWheelCurveEval[i]), kBigEpsilon))
        << "at index " << i;

    prev_w = kTestWheelCurveEval[i].w;
  }
  // Check if termination works past the end too:
  it += kBigEpsilon;
  EXPECT_FALSE(it < it_end);
}

TEST(WheelCurve, WheelStateIterator) {
  int i = 0;
  for (auto& pt : kTestWheelCurve.GetWheelStateIteratorRange()) {
    EXPECT_THAT(pt, IsWheelStateAndTotalMotionApprox(
                        ToWheelStateAndTotalMotion(kTestWheelCurveKnots[i]),
                        kBigEpsilon));
    ++i;
  }
}

TEST(WheelCurve, ClearedCurve) {
  WheelCurve cleared_curve = kTestWheelCurve;
  EXPECT_FALSE(cleared_curve.IsEmpty());
  EXPECT_TRUE(cleared_curve.IsSane());
  cleared_curve.Clear();
  EXPECT_TRUE(cleared_curve.IsEmpty());
  EXPECT_FALSE(cleared_curve.IsSane());

  Interval<double> total_motion_span = cleared_curve.GetTotalMotionSpan();
  EXPECT_TRUE(total_motion_span.Empty());

  // Check if termination works for empty motion-iteration:
  EXPECT_FALSE(cleared_curve.BeginInTotalMotion() <
               cleared_curve.EndInTotalMotion());

  WheelStateAndTotalMotion start = cleared_curve.GetStart();
  EXPECT_NEAR(start.total_motion, 0.0, kBigEpsilon);
  EXPECT_THAT(start.state.GetPositions(),
              IsApprox(WheelVector{0.0, 0.0}, kBigEpsilon));
  EXPECT_THAT(start.state.GetMotionRates(),
              IsApprox(WheelVector{0.0, 0.0}, kBigEpsilon));

  WheelStateAndTotalMotion finish = cleared_curve.GetFinish();
  EXPECT_NEAR(finish.total_motion, 0.0, kBigEpsilon);
  EXPECT_THAT(finish.state.GetPositions(),
              IsApprox(WheelVector{0.0, 0.0}, kBigEpsilon));
  EXPECT_THAT(finish.state.GetMotionRates(),
              IsApprox(WheelVector{0.0, 0.0}, kBigEpsilon));
}

TEST(WheelCurve, SetCapacity) {
  WheelCurve new_curve(10);
  EXPECT_GE(new_curve.GetCapacity(), 10);
  new_curve.SetMinCapacity(5);
  EXPECT_GE(new_curve.GetCapacity(), 10);
  new_curve.SetMinCapacity(50);
  EXPECT_GE(new_curve.GetCapacity(), 50);
}

TEST(WheelCurve, AppendLimitedCapacity) {
  WheelCurve curve1 = ToWheelCurve({{0.0, 0.0, 0.0, 0.5, 0.5},
                                    {1.0, 0.5, 0.5, 0.5, 0.5},
                                    {2.0, 1.0, 1.0, 0.5, 0.5}});

  WheelCurve curve2 = ToWheelCurve(
      {{2.0, 1.0, 1.0, 0.25, 0.75}, {3.0, 1.25, 1.75, 0.25, 0.75}});

  EXPECT_FALSE(curve1.AppendCurve(curve2));

  const WheelCurve expected_curve =
      ToWheelCurve({{0.0, 0.0, 0.0, 0.5, 0.5},
                    {1.0, 0.5, 0.5, 0.5, 0.5},
                    {2.0, 1.0, 1.0, 0.25, 0.75},
                    {3.0, 1.25, 1.75, 0.25, 0.75}});

  WheelCurve curve3(4);
  EXPECT_TRUE(curve3.AppendCurve(curve1));
  EXPECT_TRUE(curve3.AppendCurve(curve2));
  EXPECT_EQ(curve3.GetSize(), 4);
  EXPECT_THAT(curve3, IsWheelCurveApprox(expected_curve, kEpsilon));

  Interval<double> total_motion_span = curve3.GetTotalMotionSpan();
  EXPECT_NEAR(total_motion_span.min(), 0.0, kEpsilon);
  EXPECT_NEAR(total_motion_span.max(), 3.0, kEpsilon);
}

TEST(WheelCurve, PrependLimitedCapacity) {
  WheelCurve curve1 = ToWheelCurve({{0.0, 0.0, 0.0, 0.5, 0.5},
                                    {1.0, 0.5, 0.5, 0.5, 0.5},
                                    {2.0, 1.0, 1.0, 0.5, 0.5}});

  WheelCurve curve2 =
      ToWheelCurve({{2.0, 1.0, 1.0, 0.5, 0.5}, {3.0, 1.5, 1.5, 0.5, 0.5}});

  EXPECT_FALSE(curve2.PrependCurve(curve1));

  WheelCurve curve3(4);
  EXPECT_TRUE(curve3.PrependCurve(curve2));
  EXPECT_TRUE(curve3.PrependCurve(curve1));
  EXPECT_EQ(curve3.GetSize(), 4);

  Interval<double> total_motion_span = curve3.GetTotalMotionSpan();
  EXPECT_NEAR(total_motion_span.min(), 0.0, kEpsilon);
  EXPECT_NEAR(total_motion_span.max(), 3.0, kEpsilon);
}

TEST(WheelCurve, SplitAndCombineWithAppendAndPrepend) {
  WheelCurve test_curve_part1 = kTestWheelCurve;
  test_curve_part1.TruncateTo(Interval<double>(0.0, 1.25));

  WheelCurve test_curve_part2 = kTestWheelCurve;
  test_curve_part2.TruncateTo(Interval<double>(1.25, 3.0));

  WheelCurve appended_curve(kTestWheelCurve.GetCapacity());
  EXPECT_TRUE(appended_curve.AppendCurve(test_curve_part1));
  EXPECT_TRUE(appended_curve.AppendCurve(test_curve_part2));

  WheelCurve prepended_curve(kTestWheelCurve.GetCapacity());
  EXPECT_TRUE(prepended_curve.PrependCurve(test_curve_part2));
  EXPECT_TRUE(prepended_curve.PrependCurve(test_curve_part1));

  EXPECT_TRUE(appended_curve == prepended_curve);
}

TEST(WheelCurve, TruncateTo) {
  WheelCurve curve = ToWheelCurve({{0.0, 0.0, 0.0, 0.5, 0.5},
                                   {1.0, 0.5, 0.5, 0.5, 0.5},
                                   {2.0, 1.0, 1.0, 0.5, 0.5}});

  curve.TruncateTo(1.5);
  EXPECT_EQ(curve.GetSize(), 3);

  auto curve_it = curve.BeginWheelState();
  auto first_pt = curve_it->state;
  EXPECT_NEAR(curve_it->total_motion, 0.0, kEpsilon);
  EXPECT_THAT(first_pt, IsWheelStateApprox(
                            ToWheelState({0.0, 0.0, 0.0, 0.5, 0.5}), kEpsilon));

  ++curve_it;
  auto second_pt = curve_it->state;
  EXPECT_NEAR(curve_it->total_motion, 1.0, kEpsilon);
  EXPECT_THAT(
      second_pt,
      IsWheelStateApprox(ToWheelState({1.0, 0.5, 0.5, 0.5, 0.5}), kEpsilon));

  ++curve_it;
  auto third_pt = curve_it->state;
  EXPECT_NEAR(curve_it->total_motion, 1.5, kEpsilon);
  EXPECT_THAT(
      third_pt,
      IsWheelStateApprox(ToWheelState({1.5, 0.75, 0.75, 0.5, 0.5}), kEpsilon));

  curve.TruncateTo(-0.1);
  EXPECT_EQ(curve.GetSize(), 0);
}

}  // namespace
}  // namespace mobility::diff_drive
