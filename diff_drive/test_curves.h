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

#ifndef MOBILITY_DIFF_DRIVE_DIFF_DRIVE_TEST_CURVES_H_
#define MOBILITY_DIFF_DRIVE_DIFF_DRIVE_TEST_CURVES_H_

#include <cmath>
#include <vector>

#include "diff_drive/curve.h"
#include "diff_drive/test_utils.h"

namespace mobility::diff_drive::testing {

extern const Curve& kTestCurve;
extern const std::vector<RawCurvePointAndCord>& kTestCurveEval;

}  // namespace mobility::diff_drive::testing

#endif  // MOBILITY_DIFF_DRIVE_DIFF_DRIVE_TEST_CURVES_H_
