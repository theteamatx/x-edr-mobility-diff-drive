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

#ifndef MOBILITY_DIFF_DRIVE_DIFF_DRIVE_TEST_UTILS_H_
#define MOBILITY_DIFF_DRIVE_DIFF_DRIVE_TEST_UTILS_H_

#include <algorithm>
#include <vector>

#include "diff_drive/curve.h"
#include "diff_drive/curve_trajectory_utils.h"
#include "diff_drive/trajectory.h"
#include "diff_drive/wheel_curve.h"
#include "eigenmath/matchers.h"
#include "gmock/gmock.h"

namespace mobility::diff_drive::testing {

constexpr double kEpsilon = 1.0e-6;                     // NOLINT
constexpr double kBigEpsilon = 1.0e-3;                  // NOLINT
constexpr double k2PiEpsilon = 2.0 * M_PI * 1.0e-6;     // NOLINT
constexpr double k2PiBigEpsilon = 2.0 * M_PI * 1.0e-3;  // NOLINT

struct RawCurvePointAndCord {
  double s, x, y, theta, kappa;
};

inline CurvePoint ToCurvePoint(const RawCurvePointAndCord& raw_pt) {
  return CurvePoint(eigenmath::Pose2d({raw_pt.x, raw_pt.y}, raw_pt.theta),
                    raw_pt.kappa);
}

inline CurvePtAndCord ToCurvePtAndCord(const RawCurvePointAndCord& raw_pt) {
  return {raw_pt.s,
          CurvePoint(eigenmath::Pose2d({raw_pt.x, raw_pt.y}, raw_pt.theta),
                     raw_pt.kappa)};
}

inline Curve ToCurve(const std::vector<RawCurvePointAndCord>& raw_pts) {
  Curve result(raw_pts.size());
  for (auto& raw_pt : raw_pts) {
    result.AddPoint(raw_pt.s, CurvePoint(eigenmath::Pose2d({raw_pt.x, raw_pt.y},
                                                           raw_pt.theta),
                                         raw_pt.kappa));
  }
  return result;
}

struct RawStateTime {
  double t, x, y, theta, v, w;
};

inline State ToState(const RawStateTime& raw_pt) {
  return State(eigenmath::Pose2d({raw_pt.x, raw_pt.y}, raw_pt.theta),
               {raw_pt.v, raw_pt.w});
}

inline StateAndTime ToStateAndTime(const RawStateTime& raw_pt) {
  return {raw_pt.t, State(eigenmath::Pose2d({raw_pt.x, raw_pt.y}, raw_pt.theta),
                          {raw_pt.v, raw_pt.w})};
}

inline Trajectory ToTrajectory(const std::vector<RawStateTime>& raw_pts,
                               int capacity = 0) {
  Trajectory result(std::max(capacity, static_cast<int>(raw_pts.size())));
  for (auto& raw_pt : raw_pts) {
    result.AddState(raw_pt.t,
                    State(eigenmath::Pose2d({raw_pt.x, raw_pt.y}, raw_pt.theta),
                          {raw_pt.v, raw_pt.w}));
  }
  return result;
}

struct RawWheelStateAndMotion {
  double w, l, r, fl, fr;
};

inline WheelState ToWheelState(const RawWheelStateAndMotion& raw_pt) {
  return WheelState(WheelVector{raw_pt.l, raw_pt.r},
                    WheelVector{raw_pt.fl, raw_pt.fr});
}

inline WheelStateAndTotalMotion ToWheelStateAndTotalMotion(
    const RawWheelStateAndMotion& raw_pt) {
  return {raw_pt.w, ToWheelState(raw_pt)};
}

inline WheelCurve ToWheelCurve(
    const std::vector<RawWheelStateAndMotion>& raw_pts) {
  WheelCurve result(raw_pts.size());
  for (auto& raw_pt : raw_pts) {
    result.AddPoint(raw_pt.w, ToWheelState(raw_pt));
  }
  return result;
}

MATCHER_P2(IsCurvePointApprox, expected, tolerance, "") {
  bool result = true;
  if (!::testing::Value(arg.GetPose(), eigenmath::testing::IsApprox(
                                           expected.GetPose(), tolerance))) {
    *result_listener << "\nPose mismatch,";
    result = false;
  }
  if (!::testing::Value(
          arg.GetCurvature(),
          ::testing::DoubleNear(expected.GetCurvature(), tolerance))) {
    *result_listener << "\nCurvature mismatch,";
    result = false;
  }
  if (!result) {
    *result_listener << "\nat curve-point:"
                     << "\n  expected: " << expected << "\n  actual:   " << arg;
  }
  return result;
}

MATCHER_P2(IsWheelStateApprox, expected, tolerance, "") {
  bool result = true;
  if (!::testing::Value(
          arg.GetPositions(),
          eigenmath::testing::IsApprox(expected.GetPositions(), tolerance))) {
    *result_listener << "\nWheel positions mismatch,";
    result = false;
  }
  if (!::testing::Value(
          arg.GetMotionRates(),
          eigenmath::testing::IsApprox(expected.GetMotionRates(), tolerance))) {
    *result_listener << "\nWheel motion rates mismatch,";
    result = false;
  }
  if (!result) {
    *result_listener << "\nat wheel-state:"
                     << "\n  expected: " << expected << "\n  actual:   " << arg;
  }
  return result;
}

MATCHER_P2(IsStateApprox, expected, tolerance, "") {
  bool result = true;
  if (!::testing::Value(arg.GetPose(), eigenmath::testing::IsApprox(
                                           expected.GetPose(), tolerance))) {
    *result_listener << "\nPose mismatch,";
    result = false;
  }
  if (!::testing::Value(
          arg.GetArcVelocity(),
          eigenmath::testing::IsApprox(expected.GetArcVelocity(), tolerance))) {
    *result_listener << "\nArc velocity mismatch,";
    result = false;
  }
  if (!result) {
    *result_listener << "\nat state:"
                     << "\n  expected: " << expected << "\n  actual:   " << arg;
  }
  return result;
}

MATCHER_P2(IsCurvePtAndCordApprox, expected, tolerance, "") {
  bool result = true;
  if (!::testing::Value(
          arg.cord_length,
          ::testing::DoubleNear(expected.cord_length, tolerance))) {
    *result_listener << "\nCord-length mismatch, at cord-length:"
                     << "\n  expected: " << expected.cord_length
                     << "\n  actual:   " << arg.cord_length;
    result = false;
  }
  if (!::testing::ExplainMatchResult(
          IsCurvePointApprox(expected.point, tolerance), arg.point,
          result_listener)) {
    result = false;
  }
  return result;
}

MATCHER_P2(IsWheelStateAndTotalMotionApprox, expected, tolerance, "") {
  bool result = true;
  if (!::testing::Value(
          arg.total_motion,
          ::testing::DoubleNear(expected.total_motion, tolerance))) {
    *result_listener << "\nTotal motion mismatch, at total_motion:"
                     << "\n  expected: " << expected.total_motion
                     << "\n  actual:   " << arg.total_motion;
    result = false;
  }
  if (!::testing::ExplainMatchResult(
          IsWheelStateApprox(expected.state, tolerance), arg.state,
          result_listener)) {
    result = false;
  }
  return result;
}

MATCHER_P2(IsStateAndTimeApprox, expected, tolerance, "") {
  bool result = true;
  if (!::testing::Value(arg.time,
                        ::testing::DoubleNear(expected.time, tolerance))) {
    *result_listener << "\nTime mismatch, at time:"
                     << "\n  expected: " << expected.time
                     << "\n  actual:   " << arg.time;
    result = false;
  }
  if (!::testing::ExplainMatchResult(IsStateApprox(expected.state, tolerance),
                                     arg.state, result_listener)) {
    result = false;
  }
  return result;
}

MATCHER_P2(IsCurveApprox, expected, tolerance, "") {
  bool result = true;
  if (arg.GetSize() != expected.GetSize()) {
    *result_listener << "\nMismatching curve sizes:"
                     << "\n  expected: " << expected.GetSize()
                     << " curve points"
                     << "\n  actual: " << arg.GetSize() << " curve points";
    result = false;
  }
  auto arg_it = arg.BeginCurvePoint();
  auto arg_it_end = arg.EndCurvePoint();
  auto expected_it = expected.BeginCurvePoint();
  auto expected_it_end = expected.EndCurvePoint();
  int i = 0;
  for (; arg_it != arg_it_end && expected_it != expected_it_end;
       ++arg_it, ++expected_it, ++i) {
    if (!::testing::ExplainMatchResult(
            IsCurvePtAndCordApprox(*expected_it, tolerance), *arg_it,
            result_listener)) {
      result = false;
      *result_listener << "\nat curve-point with index " << i;
    }
  }
  return result;
}

MATCHER_P2(IsWheelCurveApprox, expected, tolerance, "") {
  bool result = true;
  if (arg.GetSize() != expected.GetSize()) {
    *result_listener << "\nMismatching wheel-curve sizes:"
                     << "\n  expected: " << expected.GetSize()
                     << " curve states"
                     << "\n  actual: " << arg.GetSize() << " curve states";
    result = false;
  }
  auto arg_it = arg.BeginWheelState();
  auto arg_it_end = arg.EndWheelState();
  auto expected_it = expected.BeginWheelState();
  auto expected_it_end = expected.EndWheelState();
  int i = 0;
  for (; arg_it != arg_it_end && expected_it != expected_it_end;
       ++arg_it, ++expected_it, ++i) {
    if (!::testing::ExplainMatchResult(
            IsWheelStateAndTotalMotionApprox(*expected_it, tolerance), *arg_it,
            result_listener)) {
      result = false;
      *result_listener << "\nat wheel state with index " << i;
    }
  }
  return result;
}

MATCHER_P2(IsTrajectoryApprox, expected, tolerance, "") {
  bool result = true;
  if (arg.GetSize() != expected.GetSize()) {
    *result_listener << "\nMismatching trajectory sizes:"
                     << "\n  expected: " << expected.GetSize()
                     << " trajectory points"
                     << "\n  actual: " << arg.GetSize() << " trajectory points";
    result = false;
  }
  auto arg_it = arg.BeginState();
  auto arg_it_end = arg.EndState();
  auto expected_it = expected.BeginState();
  auto expected_it_end = expected.EndState();
  int i = 0;
  for (; arg_it != arg_it_end && expected_it != expected_it_end;
       ++arg_it, ++expected_it, ++i) {
    if (!::testing::ExplainMatchResult(
            IsStateAndTimeApprox(*expected_it, tolerance), *arg_it,
            result_listener)) {
      result = false;
      *result_listener << "\nat state with index " << i;
    }
  }
  return result;
}

MATCHER_P2(CurveEvaluatesTo, expected, tolerance, "") {
  bool result = true;
  int i = 0;
  for (auto& expected_point : expected) {
    const CurvePoint arg_pt = arg.Evaluate(expected_point.s);
    const CurvePoint exp_pt(
        eigenmath::Pose2d({expected_point.x, expected_point.y},
                          expected_point.theta),
        expected_point.kappa);
    if (!::testing::ExplainMatchResult(IsCurvePointApprox(exp_pt, tolerance),
                                       arg_pt, result_listener)) {
      result = false;
      *result_listener << "\nat curve-point with index " << i;
    }
    ++i;
  }
  return result;
}

MATCHER_P2(WheelCurveEvaluatesTo, expected, tolerance, "") {
  bool result = true;
  int i = 0;
  for (auto& expected_point : expected) {
    const WheelState arg_pt = arg.Evaluate(expected_point.w);
    const WheelState exp_pt = ToWheelState(expected_point);
    if (!::testing::ExplainMatchResult(IsWheelStateApprox(exp_pt, tolerance),
                                       arg_pt, result_listener)) {
      result = false;
      *result_listener << "\nat wheel state with index " << i;
    }
    ++i;
  }
  return result;
}

MATCHER_P2(TrajectoryEvaluatesTo, expected, tolerance, "") {
  bool result = true;
  int i = 0;
  for (auto& expected_point : expected) {
    const State arg_pt = arg.Evaluate(expected_point.t);
    const State exp_pt(eigenmath::Pose2d({expected_point.x, expected_point.y},
                                         expected_point.theta),
                       {expected_point.v, expected_point.w});
    if (!::testing::ExplainMatchResult(IsStateApprox(exp_pt, tolerance), arg_pt,
                                       result_listener)) {
      result = false;
      *result_listener << "\nat state with index " << i;
    }
    ++i;
  }
  return result;
}

}  // namespace mobility::diff_drive::testing

#endif  // MOBILITY_DIFF_DRIVE_DIFF_DRIVE_TEST_UTILS_H_
