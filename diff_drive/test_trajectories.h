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

#ifndef MOBILITY_DIFF_DRIVE_DIFF_DRIVE_TEST_TRAJECTORIES_H_
#define MOBILITY_DIFF_DRIVE_DIFF_DRIVE_TEST_TRAJECTORIES_H_

#include <cmath>
#include <memory>
#include <vector>

#include "diff_drive/dynamic_limits.h"
#include "diff_drive/kinematics.h"
#include "diff_drive/test_utils.h"
#include "diff_drive/trajectory.h"

namespace mobility::diff_drive::testing {

extern const Kinematics kTestKinematics;
extern const DynamicLimits kTestDynamicLimits;

std::shared_ptr<DynamicLimits> GetSharedDynamicLimits();

extern const Trajectory& kTestTraj;
extern const std::vector<RawStateTime>& kTestTrajEval;
extern const Trajectory& kTestClothoidTraj;
extern const Trajectory& kTestRampTraj;

}  // namespace mobility::diff_drive::testing

#endif  // MOBILITY_DIFF_DRIVE_DIFF_DRIVE_TEST_TRAJECTORIES_H_
