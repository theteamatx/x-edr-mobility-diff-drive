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

#include "diff_drive/test_wheel_curves.h"

#include <vector>

namespace mobility::diff_drive::testing {

class TestWheelCurveFactory {
 public:
  WheelCurve curve;
  std::vector<RawWheelStateAndMotion> curve_eval;
  std::vector<RawWheelStateAndMotion> curve_knots;

  TestWheelCurveFactory() {
    // The following values were generated from the kTestCurve in test_curves.cc
    // to correspond to the same curve as kTestCurve and kTestTrajectory.
    curve_knots = {{0.0, 0.0, 0.0, -0.420422528454, 0.579577471546},
                   {0.5 * M_PI, -0.660398163397, 0.910398163397, 0.5, 0.5},
                   {0.5 * M_PI + 1.0, -0.160398163397, 1.4103981634,
                    0.579577471546, -0.420422528454},
                   {1.5 * M_PI + 1.0, 1.6603981634, 0.0896018366026, 0.5, 0.5},
                   {1.5 * M_PI + 2.0, 2.1603981634, 0.589601836603,
                    -0.420422528454, 0.579577471546},
                   {2.0 * M_PI + 2.0, 1.5, 1.5, 0.5, 0.5}};

    for (auto& raw_pt : curve_knots) {
      curve.AddPoint(raw_pt.w, ToWheelState(raw_pt));
    }

    curve_eval.push_back(curve_knots[0]);
    for (int i = 1; i < curve_knots.size(); ++i) {
      curve_eval.push_back(
          {curve_knots[i - 1].w + kBigEpsilon,
           curve_knots[i - 1].l + kBigEpsilon * curve_knots[i - 1].fl,
           curve_knots[i - 1].r + kBigEpsilon * curve_knots[i - 1].fr,
           curve_knots[i - 1].fl, curve_knots[i - 1].fr});
      curve_eval.push_back(
          {curve_knots[i].w - kBigEpsilon,
           curve_knots[i].l - kBigEpsilon * curve_knots[i - 1].fl,
           curve_knots[i].r - kBigEpsilon * curve_knots[i - 1].fr,
           curve_knots[i - 1].fl, curve_knots[i - 1].fr});
    }
    curve_eval.push_back({curve_knots[5].w + kBigEpsilon,
                          curve_knots[5].l + kBigEpsilon * curve_knots[5].fl,
                          curve_knots[5].r + kBigEpsilon * curve_knots[5].fr,
                          curve_knots[5].fl, curve_knots[5].fr});
  }
};
extern const TestWheelCurveFactory kTestWheelCurveFactory;

const TestWheelCurveFactory kTestWheelCurveFactory{};
const WheelCurve& kTestWheelCurve = kTestWheelCurveFactory.curve;
const std::vector<RawWheelStateAndMotion>& kTestWheelCurveEval =
    kTestWheelCurveFactory.curve_eval;
const std::vector<RawWheelStateAndMotion>& kTestWheelCurveKnots =
    kTestWheelCurveFactory.curve_knots;

}  // namespace mobility::diff_drive::testing
