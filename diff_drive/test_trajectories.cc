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

#include "diff_drive/test_trajectories.h"

#include <memory>
#include <vector>

#include "diff_drive/trajectory.h"

namespace mobility::diff_drive::testing {

const Kinematics kTestKinematics({2.0, 2.0}, 2.0);

// Use the same values as for kTestKinematics, but do not rely on initialization.
const DynamicLimits kTestDynamicLimits({{2.0, 2.0}, 2.0},
                                       /*max_wheel_velocity=*/{1.0, 1.0},
                                       /*max_wheel_acceleration=*/{2.0, 2.0},
                                       /*max_arc_velocity=*/{1.0, 0.8},
                                       /*max_arc_acceleration=*/{1.0, 1.5});

std::shared_ptr<DynamicLimits> GetSharedDynamicLimits() {
  static auto* kSharedDynamicLimits =
      new std::shared_ptr<DynamicLimits>(new DynamicLimits(kTestDynamicLimits));
  return *kSharedDynamicLimits;
}

class TestTrajectoryFactory {
 public:
  Trajectory trajectory;
  std::vector<RawStateTime> traj_eval;

  TestTrajectoryFactory() {
    trajectory = ToTrajectory(
        {{0.0, 0.0, 0.0, 0.0, 1.0, 2.0 * M_PI},
         {0.25, 0.5 / M_PI, 0.5 / M_PI, 0.5 * M_PI, 1.0, 0.0},
         {1.25, 0.5 / M_PI, 1.0 + 0.5 / M_PI, 0.5 * M_PI, 1.0, -2.0 * M_PI},
         {1.75, 1.5 / M_PI, 1.0 + 0.5 / M_PI, -0.5 * M_PI, 1.0, 0.0},
         {2.75, 1.5 / M_PI, 0.5 / M_PI, -0.5 * M_PI, 1.0, 2.0 * M_PI},
         {3.0, 2.0 / M_PI, 0.0, 0.0, 1.0, 0.0}},
        64);

    constexpr double kAlmostBigEpsilon = kBigEpsilon - kEpsilon;
    constexpr double kAlmost2PiBigEpsilon = k2PiBigEpsilon - k2PiEpsilon;
    traj_eval = {
        {0.0, 0.0, 0.0, 0.0, 1.0, 2 * M_PI},
        {kAlmostBigEpsilon, kAlmostBigEpsilon, 0.0, kAlmost2PiBigEpsilon, 1.0,
         2 * M_PI},
        {0.25 - kAlmostBigEpsilon, 0.5 / M_PI, 0.5 / M_PI - kAlmostBigEpsilon,
         M_PI_2 - kAlmost2PiBigEpsilon, 1.0, 2 * M_PI},
        {0.25 + kAlmostBigEpsilon, 0.5 / M_PI, 0.5 / M_PI + kAlmostBigEpsilon,
         M_PI_2, 1.0, 0.0},
        {1.25 - kAlmostBigEpsilon, 0.5 / M_PI,
         1.0 + 0.5 / M_PI - kAlmostBigEpsilon, M_PI_2, 1.0, 0.0},
        {1.25 + kAlmostBigEpsilon, 0.5 / M_PI,
         1.0 + 0.5 / M_PI + kAlmostBigEpsilon, M_PI_2 - kAlmost2PiBigEpsilon,
         1.0, -2 * M_PI},
        {1.75 - kAlmostBigEpsilon, 1.5 / M_PI,
         1.0 + 0.5 / M_PI + kAlmostBigEpsilon, -M_PI_2 + kAlmost2PiBigEpsilon,
         1.0, -2 * M_PI},
        {1.75 + kAlmostBigEpsilon, 1.5 / M_PI,
         1.0 + 0.5 / M_PI - kAlmostBigEpsilon, -M_PI_2, 1.0, 0.0},
        {2.75 - kAlmostBigEpsilon, 1.5 / M_PI, 0.5 / M_PI + kAlmostBigEpsilon,
         -M_PI_2, 1.0, 0.0},
        {2.75 + kAlmostBigEpsilon, 1.5 / M_PI, 0.5 / M_PI - kAlmostBigEpsilon,
         -M_PI_2 + kAlmost2PiBigEpsilon, 1.0, 2 * M_PI},
        {3.0 - kAlmostBigEpsilon, 2.0 / M_PI - kAlmostBigEpsilon, 0.0,
         -kAlmost2PiBigEpsilon, 1.0, 2 * M_PI},
        {3.0 + kAlmostBigEpsilon, 2.0 / M_PI, 0.0, 0.0, 1.0, 0.0}};
  }
};
extern const TestTrajectoryFactory kTestTrajFactory;

const TestTrajectoryFactory kTestTrajFactory{};
const Trajectory& kTestTraj = kTestTrajFactory.trajectory;
const std::vector<RawStateTime>& kTestTrajEval = kTestTrajFactory.traj_eval;

// TestClothoidTrajectoryFactory creates a clothoid trajectory with
// a rate of change of curvature of 0.1 and an arc length of 1.4 m.
class TestClothoidTrajectoryFactory {
 public:
  Trajectory trajectory;

  TestClothoidTrajectoryFactory() {
    trajectory = ToTrajectory({{0.0, 0.0, 0.0, 0.0, 1.0, 1.0},
                               {0.1, 0.0998334, 0.0049958, 0.1, 1.0, 1.01},
                               {0.2, 0.1986610, 0.0199827, 0.201, 1.0, 1.02},
                               {0.3, 0.2954606, 0.0449060, 0.303, 1.0, 1.03},
                               {0.4, 0.3892012, 0.0796028, 0.406, 1.0, 1.04},
                               {0.5, 0.4788546, 0.1237984, 0.51, 1.0, 1.05},
                               {0.6, 0.5634082, 0.1771042, 0.615, 1.0, 1.06},
                               {0.7, 0.6418777, 0.2390169, 0.721, 1.0, 1.07},
                               {0.8, 0.7133207, 0.3089194, 0.828, 1.0, 1.08},
                               {0.9, 0.7768507, 0.3860830, 0.936, 1.0, 1.09},
                               {1.0, 0.8316507, 0.4696716, 1.045, 1.0, 1.10},
                               {1.1, 0.8769874, 0.5587474, 1.155, 1.0, 1.11},
                               {1.2, 0.9122245, 0.6522786, 1.266, 1.0, 1.12},
                               {1.3, 0.9368354, 0.7491489, 1.378, 1.0, 1.13},
                               {1.4, 0.9504156, 0.8481688, 1.491, 1.0, 1.14}},
                              64);
  }
};
extern const TestClothoidTrajectoryFactory kTestClothoidTrajFactory;

const TestClothoidTrajectoryFactory kTestClothoidTrajFactory{};
const Trajectory& kTestClothoidTraj = kTestClothoidTrajFactory.trajectory;

// TestRampTrajectoryFactory creates a clothoid trajectory with
// a rate of change of curvature of 0.1 and an arc length of 1.4 m.
class TestRampTrajectoryFactory {
 public:
  Trajectory trajectory;

  TestRampTrajectoryFactory() {
    trajectory = ToTrajectory({{0.0, 0.0, 0.0, 0.0, 1.0, 1.0},
                               {0.1, 0.0998334, 0.00499583, 0.1, 1.01, 1.0},
                               {0.2, 0.199658, 0.0200828, 0.2, 1.02, 1.0},
                               {0.3, 0.298446, 0.0453075, 0.3, 1.03, 1.0},
                               {0.4, 0.395161, 0.0806112, 0.4, 1.04, 1.0},
                               {0.5, 0.488768, 0.125829, 0.5, 1.05, 1.0},
                               {0.6, 0.578246, 0.180688, 0.6, 1.06, 1.0},
                               {0.7, 0.662596, 0.244811, 0.7, 1.07, 1.0},
                               {0.8, 0.740854, 0.317716, 0.8, 1.08, 1.0},
                               {0.9, 0.812102, 0.398821, 0.9, 1.09, 1.0},
                               {1.0, 0.875479, 0.487446, 1.0, 1.1, 1.0},
                               {1.1, 0.930189, 0.582823, 1.1, 1.11, 1.0},
                               {1.2, 0.975513, 0.684097, 1.2, 1.12, 1.0},
                               {1.3, 1.01081, 0.790339, 1.3, 1.13, 1.0},
                               {1.4, 1.03555, 0.90055, 1.4, 1.14, 1.0}},
                              64);
  }
};
extern const TestRampTrajectoryFactory kTestRampTrajFactory;

const TestRampTrajectoryFactory kTestRampTrajFactory{};
const Trajectory& kTestRampTraj = kTestRampTrajFactory.trajectory;

}  // namespace mobility::diff_drive::testing

