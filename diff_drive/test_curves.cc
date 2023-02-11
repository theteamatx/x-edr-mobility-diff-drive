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

#include "diff_drive/test_curves.h"

#include <vector>

namespace mobility::diff_drive::testing {

class TestCurveFactory {
 public:
  Curve curve;
  std::vector<RawCurvePointAndCord> curve_eval;

  TestCurveFactory() {
    curve.AddPoint(
        0.0, CurvePoint(eigenmath::Pose2d(eigenmath::Vector2d(0.0, 0.0), 0.0),
                        2.0 * M_PI));

    curve.AddPoint(
        0.25,
        CurvePoint(eigenmath::Pose2d(
                       eigenmath::Vector2d(0.5 / M_PI, 0.5 / M_PI), 0.5 * M_PI),
                   0.0));

    curve.AddPoint(
        1.25, CurvePoint(eigenmath::Pose2d(
                             eigenmath::Vector2d(0.5 / M_PI, 1.0 + 0.5 / M_PI),
                             0.5 * M_PI),
                         -2.0 * M_PI));

    curve.AddPoint(
        1.75, CurvePoint(eigenmath::Pose2d(
                             eigenmath::Vector2d(1.5 / M_PI, 1.0 + 0.5 / M_PI),
                             -0.5 * M_PI),
                         0.0));

    curve.AddPoint(
        2.75, CurvePoint(
                  eigenmath::Pose2d(eigenmath::Vector2d(1.5 / M_PI, 0.5 / M_PI),
                                    -0.5 * M_PI),
                  2.0 * M_PI));

    curve.AddPoint(
        3.0,
        CurvePoint(eigenmath::Pose2d(eigenmath::Vector2d(2.0 / M_PI, 0.0), 0.0),
                   0.0));

    curve_eval = {
        {0.0, 0.0, 0.0, 0.0, 2 * M_PI},
        {kBigEpsilon, kBigEpsilon, 0.0, k2PiBigEpsilon, 2 * M_PI},
        {0.25 - kBigEpsilon, 0.5 / M_PI, 0.5 / M_PI - kBigEpsilon,
         M_PI_2 - k2PiBigEpsilon, 2 * M_PI},
        {0.25 + kBigEpsilon, 0.5 / M_PI, 0.5 / M_PI + kBigEpsilon, M_PI_2, 0.0},
        {1.25 - kBigEpsilon, 0.5 / M_PI, 1.0 + 0.5 / M_PI - kBigEpsilon, M_PI_2,
         0.0},
        {1.25 + kBigEpsilon, 0.5 / M_PI, 1.0 + 0.5 / M_PI + kBigEpsilon,
         M_PI_2 - k2PiBigEpsilon, -2 * M_PI},
        {1.75 - kBigEpsilon, 1.5 / M_PI, 1.0 + 0.5 / M_PI + kBigEpsilon,
         -M_PI_2 + k2PiBigEpsilon, -2 * M_PI},
        {1.75 + kBigEpsilon, 1.5 / M_PI, 1.0 + 0.5 / M_PI - kBigEpsilon,
         -M_PI_2, 0.0},
        {2.75 - kBigEpsilon, 1.5 / M_PI, 0.5 / M_PI + kBigEpsilon, -M_PI_2,
         0.0},
        {2.75 + kBigEpsilon, 1.5 / M_PI, 0.5 / M_PI - kBigEpsilon,
         -M_PI_2 + k2PiBigEpsilon, 2 * M_PI},
        {3.0 - kBigEpsilon, 2.0 / M_PI - kBigEpsilon, 0.0, -k2PiBigEpsilon,
         2 * M_PI},
        {3.0 + kBigEpsilon, 2.0 / M_PI, 0.0, 0.0, 0.0}};
  }
};
extern const TestCurveFactory kTestCurveFactory;

const TestCurveFactory kTestCurveFactory{};
const Curve& kTestCurve = kTestCurveFactory.curve;
const std::vector<RawCurvePointAndCord>& kTestCurveEval =
    kTestCurveFactory.curve_eval;

}  // namespace mobility::diff_drive::testing
