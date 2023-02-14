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

#include "diff_drive/trajectory_limits.h"

#include <cmath>
#include <iterator>
#include <string>
#include <vector>

#include "absl/random/distributions.h"
#include "absl/strings/str_format.h"
#include "benchmark/benchmark.h"
#include "diff_drive/state.h"
#include "diff_drive/trajectory.h"
#include "eigenmath/matchers.h"
#include "eigenmath/sampling.h"
#include "eigenmath/so2_interval.h"
#include "genit/iterator_range.h"
#include "genit/zip_iterator.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace mobility::diff_drive {

namespace {
using eigenmath::testing::IsApprox;

constexpr double kEpsilon = 1.0e-6;
constexpr double kCycleTime = 0.01;
constexpr double kMaxWheelVelocity = 1.0;
constexpr double kMaxWheelAcceleration = 3.0;
constexpr double kMaxArcVelocity = 1.5;
constexpr double kMaxArcAcceleration = 6.0;

void CreateTrajectoryLimits(Kinematics *kinematics, DynamicLimits *dd_limits,
                            TrajectoryLimits *traj_limits) {
  *kinematics = Kinematics({2.0, 2.0}, 2.0);

  const WheelVector max_wheel_velocity = {kMaxWheelVelocity, kMaxWheelVelocity};
  const WheelVector max_wheel_acceleration =
      WheelVector{kMaxWheelAcceleration, kMaxWheelAcceleration};
  const ArcVector max_arc_velocity = {kMaxArcVelocity, kMaxArcVelocity};
  const ArcVector max_arc_accel_trans =
      kinematics->ComputeForwardKinematics(max_wheel_acceleration);
  const ArcVector max_arc_accel_rot =
      kinematics->ComputeForwardKinematics(WheelVector(
          -max_wheel_acceleration.Left(), max_wheel_acceleration.Right()));
  const ArcVector max_arc_acceleration = {max_arc_accel_trans.Translation(),
                                          max_arc_accel_rot.Rotation()};

  *dd_limits =
      DynamicLimits(*kinematics, max_wheel_velocity, max_wheel_acceleration,
                    max_arc_velocity, max_arc_acceleration);

  *traj_limits = TrajectoryLimits(*dd_limits, kCycleTime);
}

TEST(TrajectoryLimits, BasicMembers) {
  Kinematics kinematics;
  DynamicLimits dd_limits;
  TrajectoryLimits traj_limits;
  CreateTrajectoryLimits(&kinematics, &dd_limits, &traj_limits);

  const WheelVector &max_wheel_vel_jump = traj_limits.GetMaxWheelVelocityJump();
  const ArcVector &max_arc_vel_jump = traj_limits.GetMaxArcVelocityJump();

  EXPECT_NEAR(max_wheel_vel_jump.Left(), kMaxWheelAcceleration * kCycleTime,
              kEpsilon);
  EXPECT_NEAR(max_wheel_vel_jump.Right(), kMaxWheelAcceleration * kCycleTime,
              kEpsilon);

  EXPECT_NEAR(max_arc_vel_jump.Translation(), kMaxArcAcceleration * kCycleTime,
              kEpsilon);
  EXPECT_NEAR(max_arc_vel_jump.Rotation(), kMaxArcAcceleration * kCycleTime,
              kEpsilon);
}

TEST(TrajectoryLimits, StraightAggressive) {
  Kinematics kinematics;
  DynamicLimits dd_limits;
  TrajectoryLimits traj_limits;
  CreateTrajectoryLimits(&kinematics, &dd_limits, &traj_limits);

  const WheelVector start_wheel_vel = {0.2, 0.2};
  const ArcVector start_arc_vel =
      kinematics.ComputeForwardKinematics(start_wheel_vel);

  double total_duration = 5.0;
  double aggressivity = 1.0;

  // Straight-line with maximum acceleration:

  WheelVector end_wheel_vel = {0.8, 0.8};
  ArcVector end_arc_vel = kinematics.ComputeForwardKinematics(end_wheel_vel);

  double suitable_time_step = 0.0;
  WheelVector suitable_wheel_vel_inc = {0.0, 0.0};
  ArcVector suitable_arc_vel_inc = {0.0, 0.0};
  traj_limits.GetSuitableVelocityIncrements(
      kinematics, start_wheel_vel, start_arc_vel, end_wheel_vel, end_arc_vel,
      total_duration, aggressivity, &suitable_time_step,
      &suitable_wheel_vel_inc, &suitable_arc_vel_inc);

  double fraction = suitable_time_step / kCycleTime;
  EXPECT_NEAR(fraction, 1.0, 0.05);
  EXPECT_LE(std::abs(suitable_wheel_vel_inc.Left()),
            kMaxWheelAcceleration * kCycleTime * aggressivity *
                (fraction + kEpsilon));
  EXPECT_LE(std::abs(suitable_wheel_vel_inc.Right()),
            kMaxWheelAcceleration * kCycleTime * aggressivity *
                (fraction + kEpsilon));
  EXPECT_LE(
      std::abs(suitable_arc_vel_inc.Translation()),
      kMaxArcAcceleration * kCycleTime * aggressivity * (fraction + kEpsilon));
  EXPECT_LE(
      std::abs(suitable_arc_vel_inc.Rotation()),
      kMaxArcAcceleration * kCycleTime * aggressivity * (fraction + kEpsilon));
  double integral_num_of_steps_computed =
      (end_wheel_vel.Left() - start_wheel_vel.Left()) /
      suitable_wheel_vel_inc.Left();
  double minimum_num_of_steps =
      std::abs(end_wheel_vel.Left() - start_wheel_vel.Left()) /
      (kCycleTime * traj_limits.GetMaxWheelAcceleration().Left() *
       aggressivity);
  EXPECT_GT(integral_num_of_steps_computed, minimum_num_of_steps - 1.0);
  EXPECT_LE(integral_num_of_steps_computed, minimum_num_of_steps + 1.0);
  WheelVector resulting_end_wheel_vel =
      integral_num_of_steps_computed * suitable_wheel_vel_inc + start_wheel_vel;
  ArcVector resulting_end_arc_vel =
      integral_num_of_steps_computed * suitable_arc_vel_inc + start_arc_vel;
  EXPECT_NEAR(resulting_end_wheel_vel.Left(), end_wheel_vel.Left(), kEpsilon);
  EXPECT_NEAR(resulting_end_wheel_vel.Right(), end_wheel_vel.Right(), kEpsilon);
  EXPECT_NEAR(resulting_end_arc_vel.Translation(), end_arc_vel.Translation(),
              kEpsilon);
  EXPECT_NEAR(resulting_end_arc_vel.Rotation(), end_arc_vel.Rotation(),
              kEpsilon);
}

TEST(TrajectoryLimits, ToPureSpinAggressive) {
  Kinematics kinematics;
  DynamicLimits dd_limits;
  TrajectoryLimits traj_limits;
  CreateTrajectoryLimits(&kinematics, &dd_limits, &traj_limits);

  const WheelVector start_wheel_vel = {0.2, 0.2};
  const ArcVector start_arc_vel =
      kinematics.ComputeForwardKinematics(start_wheel_vel);

  double total_duration = 5.0;
  double aggressivity = 1.0;

  // Towards a pure spin with maximum acceleration:

  WheelVector end_wheel_vel = {-0.8, 0.8};
  ArcVector end_arc_vel = kinematics.ComputeForwardKinematics(end_wheel_vel);

  double suitable_time_step = 0.0;
  WheelVector suitable_wheel_vel_inc = {0.0, 0.0};
  ArcVector suitable_arc_vel_inc = {0.0, 0.0};
  traj_limits.GetSuitableVelocityIncrements(
      kinematics, start_wheel_vel, start_arc_vel, end_wheel_vel, end_arc_vel,
      total_duration, aggressivity, &suitable_time_step,
      &suitable_wheel_vel_inc, &suitable_arc_vel_inc);

  double fraction = suitable_time_step / kCycleTime;
  EXPECT_NEAR(fraction, 1.0, 0.05);
  EXPECT_LE(std::abs(suitable_wheel_vel_inc.Left()),
            kMaxWheelAcceleration * kCycleTime * aggressivity *
                (fraction + kEpsilon));
  EXPECT_LE(std::abs(suitable_wheel_vel_inc.Right()),
            kMaxWheelAcceleration * kCycleTime * aggressivity *
                (fraction + kEpsilon));
  EXPECT_LE(
      std::abs(suitable_arc_vel_inc.Translation()),
      kMaxArcAcceleration * kCycleTime * aggressivity * (fraction + kEpsilon));
  EXPECT_LE(
      std::abs(suitable_arc_vel_inc.Rotation()),
      kMaxArcAcceleration * kCycleTime * aggressivity * (fraction + kEpsilon));
  double integral_num_of_steps_computed =
      (end_wheel_vel.Left() - start_wheel_vel.Left()) /
      suitable_wheel_vel_inc.Left();
  double minimum_num_of_steps =
      std::abs(end_wheel_vel.Left() - start_wheel_vel.Left()) /
      (kCycleTime * traj_limits.GetMaxWheelAcceleration().Left() *
       aggressivity);
  EXPECT_GT(integral_num_of_steps_computed, minimum_num_of_steps - 1.0);
  EXPECT_LE(integral_num_of_steps_computed, minimum_num_of_steps + 1.0);
  WheelVector resulting_end_wheel_vel =
      integral_num_of_steps_computed * suitable_wheel_vel_inc + start_wheel_vel;
  ArcVector resulting_end_arc_vel =
      integral_num_of_steps_computed * suitable_arc_vel_inc + start_arc_vel;
  EXPECT_NEAR(resulting_end_wheel_vel.Left(), end_wheel_vel.Left(), kEpsilon);
  EXPECT_NEAR(resulting_end_wheel_vel.Right(), end_wheel_vel.Right(), kEpsilon);
  EXPECT_NEAR(resulting_end_arc_vel.Translation(), end_arc_vel.Translation(),
              kEpsilon);
  EXPECT_NEAR(resulting_end_arc_vel.Rotation(), end_arc_vel.Rotation(),
              kEpsilon);
}

TEST(TrajectoryLimits, StraightRelaxed) {
  Kinematics kinematics;
  DynamicLimits dd_limits;
  TrajectoryLimits traj_limits;
  CreateTrajectoryLimits(&kinematics, &dd_limits, &traj_limits);

  const WheelVector start_wheel_vel = {0.2, 0.2};
  const ArcVector start_arc_vel =
      kinematics.ComputeForwardKinematics(start_wheel_vel);

  double total_duration = 5.0;
  double aggressivity = 0.5;

  // Straight-line with half acceleration:

  WheelVector end_wheel_vel = {0.8, 0.8};
  ArcVector end_arc_vel = kinematics.ComputeForwardKinematics(end_wheel_vel);

  double suitable_time_step = 0.0;
  WheelVector suitable_wheel_vel_inc = {0.0, 0.0};
  ArcVector suitable_arc_vel_inc = {0.0, 0.0};
  traj_limits.GetSuitableVelocityIncrements(
      kinematics, start_wheel_vel, start_arc_vel, end_wheel_vel, end_arc_vel,
      total_duration, aggressivity, &suitable_time_step,
      &suitable_wheel_vel_inc, &suitable_arc_vel_inc);

  double fraction = suitable_time_step / kCycleTime;
  EXPECT_NEAR(fraction, 1.0, 0.05);
  EXPECT_LE(std::abs(suitable_wheel_vel_inc.Left()),
            kMaxWheelAcceleration * kCycleTime * aggressivity *
                (fraction + kEpsilon));
  EXPECT_LE(std::abs(suitable_wheel_vel_inc.Right()),
            kMaxWheelAcceleration * kCycleTime * aggressivity *
                (fraction + kEpsilon));
  EXPECT_LE(
      std::abs(suitable_arc_vel_inc.Translation()),
      kMaxArcAcceleration * kCycleTime * aggressivity * (fraction + kEpsilon));
  EXPECT_LE(
      std::abs(suitable_arc_vel_inc.Rotation()),
      kMaxArcAcceleration * kCycleTime * aggressivity * (fraction + kEpsilon));
  double integral_num_of_steps_computed =
      (end_wheel_vel.Left() - start_wheel_vel.Left()) /
      suitable_wheel_vel_inc.Left();
  double minimum_num_of_steps =
      std::abs(end_wheel_vel.Left() - start_wheel_vel.Left()) /
      (kCycleTime * traj_limits.GetMaxWheelAcceleration().Left() *
       aggressivity);
  EXPECT_GT(integral_num_of_steps_computed, minimum_num_of_steps);
  EXPECT_LE(integral_num_of_steps_computed, minimum_num_of_steps + 2.0);
  WheelVector resulting_end_wheel_vel =
      integral_num_of_steps_computed * suitable_wheel_vel_inc + start_wheel_vel;
  ArcVector resulting_end_arc_vel =
      integral_num_of_steps_computed * suitable_arc_vel_inc + start_arc_vel;
  EXPECT_NEAR(resulting_end_wheel_vel.Left(), end_wheel_vel.Left(), kEpsilon);
  EXPECT_NEAR(resulting_end_wheel_vel.Right(), end_wheel_vel.Right(), kEpsilon);
  EXPECT_NEAR(resulting_end_arc_vel.Translation(), end_arc_vel.Translation(),
              kEpsilon);
  EXPECT_NEAR(resulting_end_arc_vel.Rotation(), end_arc_vel.Rotation(),
              kEpsilon);
}

TEST(TrajectoryLimits, ToPureSpinRelaxed) {
  Kinematics kinematics;
  DynamicLimits dd_limits;
  TrajectoryLimits traj_limits;
  CreateTrajectoryLimits(&kinematics, &dd_limits, &traj_limits);

  const WheelVector start_wheel_vel = {0.2, 0.2};
  const ArcVector start_arc_vel =
      kinematics.ComputeForwardKinematics(start_wheel_vel);

  double total_duration = 5.0;
  double aggressivity = 0.5;

  // Towards a pure spin with half acceleration:

  WheelVector end_wheel_vel = {-0.8, 0.8};
  ArcVector end_arc_vel = kinematics.ComputeForwardKinematics(end_wheel_vel);

  double suitable_time_step = 0.0;
  WheelVector suitable_wheel_vel_inc = {0.0, 0.0};
  ArcVector suitable_arc_vel_inc = {0.0, 0.0};
  traj_limits.GetSuitableVelocityIncrements(
      kinematics, start_wheel_vel, start_arc_vel, end_wheel_vel, end_arc_vel,
      total_duration, aggressivity, &suitable_time_step,
      &suitable_wheel_vel_inc, &suitable_arc_vel_inc);

  double fraction = suitable_time_step / kCycleTime;
  EXPECT_NEAR(fraction, 1.0, 0.05);
  EXPECT_LE(std::abs(suitable_wheel_vel_inc.Left()),
            kMaxWheelAcceleration * kCycleTime * aggressivity *
                (fraction + kEpsilon));
  EXPECT_LE(std::abs(suitable_wheel_vel_inc.Right()),
            kMaxWheelAcceleration * kCycleTime * aggressivity *
                (fraction + kEpsilon));
  EXPECT_LE(
      std::abs(suitable_arc_vel_inc.Translation()),
      kMaxArcAcceleration * kCycleTime * aggressivity * (fraction + kEpsilon));
  EXPECT_LE(
      std::abs(suitable_arc_vel_inc.Rotation()),
      kMaxArcAcceleration * kCycleTime * aggressivity * (fraction + kEpsilon));
  double integral_num_of_steps_computed =
      (end_wheel_vel.Left() - start_wheel_vel.Left()) /
      suitable_wheel_vel_inc.Left();
  double minimum_num_of_steps =
      std::abs(end_wheel_vel.Left() - start_wheel_vel.Left()) /
      (kCycleTime * traj_limits.GetMaxWheelAcceleration().Left() *
       aggressivity);
  EXPECT_GT(integral_num_of_steps_computed, minimum_num_of_steps);
  EXPECT_LE(integral_num_of_steps_computed, minimum_num_of_steps + 2.0);
  WheelVector resulting_end_wheel_vel =
      integral_num_of_steps_computed * suitable_wheel_vel_inc + start_wheel_vel;
  ArcVector resulting_end_arc_vel =
      integral_num_of_steps_computed * suitable_arc_vel_inc + start_arc_vel;
  EXPECT_NEAR(resulting_end_wheel_vel.Left(), end_wheel_vel.Left(), kEpsilon);
  EXPECT_NEAR(resulting_end_wheel_vel.Right(), end_wheel_vel.Right(), kEpsilon);
  EXPECT_NEAR(resulting_end_arc_vel.Translation(), end_arc_vel.Translation(),
              kEpsilon);
  EXPECT_NEAR(resulting_end_arc_vel.Rotation(), end_arc_vel.Rotation(),
              kEpsilon);
}

TEST(TrajectoryLimits, StraightRelaxedNoTime) {
  Kinematics kinematics;
  DynamicLimits dd_limits;
  TrajectoryLimits traj_limits;
  CreateTrajectoryLimits(&kinematics, &dd_limits, &traj_limits);

  const WheelVector start_wheel_vel = {0.2, 0.2};
  const ArcVector start_arc_vel =
      kinematics.ComputeForwardKinematics(start_wheel_vel);

  double total_duration = 0.1;
  double aggressivity = 0.5;

  // Straight-line with half acceleration but without enough time:

  WheelVector end_wheel_vel = {0.8, 0.8};
  ArcVector end_arc_vel = kinematics.ComputeForwardKinematics(end_wheel_vel);

  double suitable_time_step = 0.0;
  WheelVector suitable_wheel_vel_inc = {0.0, 0.0};
  ArcVector suitable_arc_vel_inc = {0.0, 0.0};
  traj_limits.GetSuitableVelocityIncrements(
      kinematics, start_wheel_vel, start_arc_vel, end_wheel_vel, end_arc_vel,
      total_duration, aggressivity, &suitable_time_step,
      &suitable_wheel_vel_inc, &suitable_arc_vel_inc);

  double fraction = suitable_time_step / kCycleTime;
  EXPECT_NEAR(fraction, 1.0, 0.05);
  EXPECT_LE(std::abs(suitable_wheel_vel_inc.Left()),
            kMaxWheelAcceleration * kCycleTime * (fraction + kEpsilon));
  EXPECT_LE(std::abs(suitable_wheel_vel_inc.Right()),
            kMaxWheelAcceleration * kCycleTime * (fraction + kEpsilon));
  EXPECT_LE(std::abs(suitable_arc_vel_inc.Translation()),
            kMaxArcAcceleration * kCycleTime * (fraction + kEpsilon));
  EXPECT_LE(std::abs(suitable_arc_vel_inc.Rotation()),
            kMaxArcAcceleration * kCycleTime * (fraction + kEpsilon));
  double integral_num_of_steps_computed =
      (end_wheel_vel.Left() - start_wheel_vel.Left()) /
      suitable_wheel_vel_inc.Left();
  double minimum_num_of_steps =
      std::abs(end_wheel_vel.Left() - start_wheel_vel.Left()) /
      (kCycleTime * traj_limits.GetMaxWheelAcceleration().Left());
  EXPECT_GT(integral_num_of_steps_computed, minimum_num_of_steps);
  EXPECT_LE(integral_num_of_steps_computed, minimum_num_of_steps + 2.0);
  WheelVector resulting_end_wheel_vel =
      integral_num_of_steps_computed * suitable_wheel_vel_inc + start_wheel_vel;
  ArcVector resulting_end_arc_vel =
      integral_num_of_steps_computed * suitable_arc_vel_inc + start_arc_vel;
  EXPECT_NEAR(resulting_end_wheel_vel.Left(), end_wheel_vel.Left(), kEpsilon);
  EXPECT_NEAR(resulting_end_wheel_vel.Right(), end_wheel_vel.Right(), kEpsilon);
  EXPECT_NEAR(resulting_end_arc_vel.Translation(), end_arc_vel.Translation(),
              kEpsilon);
  EXPECT_NEAR(resulting_end_arc_vel.Rotation(), end_arc_vel.Rotation(),
              kEpsilon);
}

TEST(TrajectoryLimits, ToPureSpinRelaxedNoTime) {
  Kinematics kinematics;
  DynamicLimits dd_limits;
  TrajectoryLimits traj_limits;
  CreateTrajectoryLimits(&kinematics, &dd_limits, &traj_limits);

  const WheelVector start_wheel_vel = {0.2, 0.2};
  const ArcVector start_arc_vel =
      kinematics.ComputeForwardKinematics(start_wheel_vel);

  double total_duration = 0.1;
  double aggressivity = 0.5;

  // Towards a pure spin with half acceleration but without enough time:

  WheelVector end_wheel_vel = {-0.8, 0.8};
  ArcVector end_arc_vel = kinematics.ComputeForwardKinematics(end_wheel_vel);

  double suitable_time_step = 0.0;
  WheelVector suitable_wheel_vel_inc = {0.0, 0.0};
  ArcVector suitable_arc_vel_inc = {0.0, 0.0};
  traj_limits.GetSuitableVelocityIncrements(
      kinematics, start_wheel_vel, start_arc_vel, end_wheel_vel, end_arc_vel,
      total_duration, aggressivity, &suitable_time_step,
      &suitable_wheel_vel_inc, &suitable_arc_vel_inc);

  double fraction = suitable_time_step / kCycleTime;
  EXPECT_NEAR(fraction, 1.0, 0.05);
  EXPECT_LE(std::abs(suitable_wheel_vel_inc.Left()),
            kMaxWheelAcceleration * kCycleTime * (fraction + kEpsilon));
  EXPECT_LE(std::abs(suitable_wheel_vel_inc.Right()),
            kMaxWheelAcceleration * kCycleTime * (fraction + kEpsilon));
  EXPECT_LE(std::abs(suitable_arc_vel_inc.Translation()),
            kMaxArcAcceleration * kCycleTime * (fraction + kEpsilon));
  EXPECT_LE(std::abs(suitable_arc_vel_inc.Rotation()),
            kMaxArcAcceleration * kCycleTime * (fraction + kEpsilon));
  double integral_num_of_steps_computed =
      (end_wheel_vel.Left() - start_wheel_vel.Left()) /
      suitable_wheel_vel_inc.Left();
  double minimum_num_of_steps =
      std::abs(end_wheel_vel.Left() - start_wheel_vel.Left()) /
      (kCycleTime * traj_limits.GetMaxWheelAcceleration().Left());
  EXPECT_GT(integral_num_of_steps_computed, minimum_num_of_steps);
  EXPECT_LE(integral_num_of_steps_computed, minimum_num_of_steps + 2.0);
  WheelVector resulting_end_wheel_vel =
      integral_num_of_steps_computed * suitable_wheel_vel_inc + start_wheel_vel;
  ArcVector resulting_end_arc_vel =
      integral_num_of_steps_computed * suitable_arc_vel_inc + start_arc_vel;
  EXPECT_NEAR(resulting_end_wheel_vel.Left(), end_wheel_vel.Left(), kEpsilon);
  EXPECT_NEAR(resulting_end_wheel_vel.Right(), end_wheel_vel.Right(), kEpsilon);
  EXPECT_NEAR(resulting_end_arc_vel.Translation(), end_arc_vel.Translation(),
              kEpsilon);
  EXPECT_NEAR(resulting_end_arc_vel.Rotation(), end_arc_vel.Rotation(),
              kEpsilon);
}

TEST(TrajectoryLimits, MinimumStoppingTime) {
  Kinematics kinematics;
  DynamicLimits dd_limits;
  TrajectoryLimits traj_limits;
  CreateTrajectoryLimits(&kinematics, &dd_limits, &traj_limits);

  const WheelVector start_wheel_vel = {0.2, -0.9};
  const ArcVector start_arc_vel =
      kinematics.ComputeForwardKinematics(start_wheel_vel);

  const double min_stopping_time =
      traj_limits.GetMinimumStoppingTime(start_arc_vel, start_wheel_vel);
  EXPECT_NEAR(min_stopping_time, 0.9 / kMaxWheelAcceleration + kCycleTime,
              kEpsilon);
}

TEST(TrajectoryLimits, LinearBrakingDistance) {
  Kinematics kinematics;
  DynamicLimits dd_limits;
  TrajectoryLimits traj_limits;
  CreateTrajectoryLimits(&kinematics, &dd_limits, &traj_limits);

  WheelVector start_wheel_vel = {0.2, -0.8};
  ArcVector start_arc_vel =
      kinematics.ComputeForwardKinematics(start_wheel_vel);
  double aggressivity = 0.5;

  double suitable_time_step, suitable_vel_inc;
  double braking_distance = traj_limits.GetLinearBrakingDistance(
      start_arc_vel.Translation(), aggressivity, &suitable_time_step,
      &suitable_vel_inc);
  EXPECT_NEAR(suitable_time_step, 2.0 * kCycleTime, kEpsilon);
  EXPECT_NEAR(suitable_vel_inc, kMaxArcAcceleration * kCycleTime, kEpsilon);
  double expected_vel = start_arc_vel.Translation() + suitable_vel_inc;
  double expected_distance = 0.0;
  do {
    expected_distance += std::abs(expected_vel) * suitable_time_step;
    expected_vel += suitable_vel_inc;
  } while (expected_vel < 0.0);
  EXPECT_NEAR(braking_distance, expected_distance, kEpsilon);

  start_wheel_vel = WheelVector{0.29, 0.29};
  start_arc_vel = kinematics.ComputeForwardKinematics(start_wheel_vel);

  braking_distance = traj_limits.GetLinearBrakingDistance(
      start_arc_vel.Translation(), aggressivity, &suitable_time_step,
      &suitable_vel_inc);
  EXPECT_NEAR(suitable_time_step, 2.0 * kCycleTime, kEpsilon);
  EXPECT_NEAR(suitable_vel_inc, -kMaxArcAcceleration * kCycleTime, kEpsilon);
  expected_vel = start_arc_vel.Translation() + suitable_vel_inc;
  expected_distance = 0.0;
  do {
    expected_distance += std::abs(expected_vel) * suitable_time_step;
    expected_vel += suitable_vel_inc;
  } while (expected_vel > 0.0);
  EXPECT_NEAR(braking_distance, expected_distance, kEpsilon);
}

TEST(TrajectoryLimits, LimitedNextVelocity) {
  Kinematics kinematics;
  DynamicLimits dd_limits;
  TrajectoryLimits traj_limits;
  CreateTrajectoryLimits(&kinematics, &dd_limits, &traj_limits);

  WheelVector start_wheel_vel = {0.2, -0.8};
  ArcVector start_arc_vel =
      kinematics.ComputeForwardKinematics(start_wheel_vel);
  WheelVector desired_wheel_vel = {0.2, -0.8};
  ArcVector desired_arc_vel =
      kinematics.ComputeForwardKinematics(desired_wheel_vel);

  ArcVector next_velocity = traj_limits.ComputeLimitedNextVelocity(
      dd_limits, start_arc_vel, desired_arc_vel);
  EXPECT_NEAR(next_velocity.Translation(), -0.6, kEpsilon);
  EXPECT_NEAR(next_velocity.Rotation(), -1.0, kEpsilon);

  desired_wheel_vel = WheelVector{0.0, 0.0};
  desired_arc_vel = kinematics.ComputeForwardKinematics(desired_wheel_vel);

  next_velocity = traj_limits.ComputeLimitedNextVelocity(
      dd_limits, start_arc_vel, desired_arc_vel);
  EXPECT_NEAR(next_velocity.Translation(), -0.5775, kEpsilon);
  EXPECT_NEAR(next_velocity.Rotation(), -0.9625, kEpsilon);

  start_wheel_vel = WheelVector{0.749, 0.749};
  start_arc_vel = kinematics.ComputeForwardKinematics(start_wheel_vel);
  desired_wheel_vel = WheelVector{1.0, 1.0};
  desired_arc_vel = kinematics.ComputeForwardKinematics(desired_wheel_vel);

  next_velocity = traj_limits.ComputeLimitedNextVelocity(
      dd_limits, start_arc_vel, desired_arc_vel);
  EXPECT_NEAR(next_velocity.Translation(), 1.5, kEpsilon);
  EXPECT_NEAR(next_velocity.Rotation(), 0.0, kEpsilon);

  // Test infinite cases.
  start_arc_vel = ArcVector{1.0, 0.5};

  next_velocity = traj_limits.ComputeLimitedNextVelocity(
      dd_limits, start_arc_vel,
      ArcVector{start_arc_vel.Translation(),
                std::numeric_limits<double>::infinity()});
  ArcVector next_velocity_expected = traj_limits.ComputeLimitedNextVelocity(
      dd_limits, start_arc_vel, ArcVector{0.0, 1.0e6});
  EXPECT_NEAR(next_velocity.Translation(), next_velocity_expected.Translation(),
              kEpsilon);
  EXPECT_NEAR(next_velocity.Rotation(), next_velocity_expected.Rotation(),
              kEpsilon);

  next_velocity = traj_limits.ComputeLimitedNextVelocity(
      dd_limits, start_arc_vel,
      ArcVector{start_arc_vel.Translation(),
                -std::numeric_limits<double>::infinity()});
  next_velocity_expected = traj_limits.ComputeLimitedNextVelocity(
      dd_limits, start_arc_vel, ArcVector{0.0, -1.0e6});
  EXPECT_NEAR(next_velocity.Translation(), next_velocity_expected.Translation(),
              kEpsilon);
  EXPECT_NEAR(next_velocity.Rotation(), next_velocity_expected.Rotation(),
              kEpsilon);

  next_velocity = traj_limits.ComputeLimitedNextVelocity(
      dd_limits, start_arc_vel,
      ArcVector{std::numeric_limits<double>::infinity(),
                start_arc_vel.Rotation()});
  next_velocity_expected = traj_limits.ComputeLimitedNextVelocity(
      dd_limits, start_arc_vel, ArcVector{1.0e6, 0.0});
  EXPECT_NEAR(next_velocity.Translation(), next_velocity_expected.Translation(),
              kEpsilon);
  EXPECT_NEAR(next_velocity.Rotation(), next_velocity_expected.Rotation(),
              kEpsilon);

  next_velocity = traj_limits.ComputeLimitedNextVelocity(
      dd_limits, start_arc_vel,
      ArcVector{-std::numeric_limits<double>::infinity(),
                start_arc_vel.Rotation()});
  next_velocity_expected = traj_limits.ComputeLimitedNextVelocity(
      dd_limits, start_arc_vel, ArcVector{-1.0e6, 0.0});
  EXPECT_NEAR(next_velocity.Translation(), next_velocity_expected.Translation(),
              kEpsilon);
  EXPECT_NEAR(next_velocity.Rotation(), next_velocity_expected.Rotation(),
              kEpsilon);

  next_velocity = traj_limits.ComputeLimitedNextVelocity(
      dd_limits, start_arc_vel,
      ArcVector{std::numeric_limits<double>::infinity(),
                std::numeric_limits<double>::infinity()});
  next_velocity_expected = traj_limits.ComputeLimitedNextVelocity(
      dd_limits, start_arc_vel, ArcVector{0.0, 0.0});
  EXPECT_NEAR(next_velocity.Translation(), next_velocity_expected.Translation(),
              kEpsilon);
  EXPECT_NEAR(next_velocity.Rotation(), next_velocity_expected.Rotation(),
              kEpsilon);
}

TEST(TrajectoryLimits, GetLinearBangBangDisplacement) {
  Kinematics kinematics;
  DynamicLimits dd_limits;
  TrajectoryLimits traj_limits;
  CreateTrajectoryLimits(&kinematics, &dd_limits, &traj_limits);

  double start_velocity = 1.0;
  double velocity_step = 0.3;
  double time_step = 0.07;
  int steps_first = 10;
  double time_flat = 10.0 * time_step;
  int steps_second = 14;

  // Test that positive velocity and negative velocity traverse the same
  // distance.
  double distance1 = std::abs(traj_limits.GetLinearBangBangDisplacement(
      start_velocity, time_step, steps_first, velocity_step, time_flat,
      steps_second, -velocity_step));
  double distance2 = std::abs(traj_limits.GetLinearBangBangDisplacement(
      -start_velocity, time_step, steps_first, -velocity_step, time_flat,
      steps_second, velocity_step));
  EXPECT_NEAR(distance1, distance2, kEpsilon) << "symmetric velocity profile";
  // Test that the up-ramp does as expected.
  distance2 = 0.0;
  start_velocity = 1.0;
  distance1 = traj_limits.GetLinearBangBangDisplacement(
      start_velocity, time_step, steps_first, velocity_step, 0.0, 0, 0.0);
  for (int c = 0; c < steps_first; ++c) {
    start_velocity += velocity_step;
    distance2 += start_velocity * time_step;
  }
  EXPECT_NEAR(distance1, distance2, kEpsilon) << "ramp up distance only";
  // Test that the plateau region does as expected.
  start_velocity = 1.0;
  distance1 = traj_limits.GetLinearBangBangDisplacement(
      start_velocity, time_step, 0, velocity_step, time_flat, 0, 0.0);
  distance2 = start_velocity * time_flat;
  EXPECT_NEAR(distance1, distance2, kEpsilon) << "plateau distance only";
  // Test that the down ramp does as expected.
  distance2 = 0.0;
  start_velocity = 4.0;
  distance1 = traj_limits.GetLinearBangBangDisplacement(
      start_velocity, time_step, 0, 0.0, 0.0, steps_second, -velocity_step);
  for (int c = 0; c < steps_second; ++c) {
    start_velocity -= velocity_step;
    if (start_velocity < 0.0) {
      start_velocity = 0.0;
    }
    distance2 += start_velocity * time_step;
  }
  EXPECT_NEAR(distance1, distance2, kEpsilon) << "ramp down distance only";

  // Test that the distance matches the trajectory integration.
  start_velocity = 1.0;
  distance1 = traj_limits.GetLinearBangBangDisplacement(
      start_velocity, time_step, steps_first, velocity_step, time_flat,
      steps_second, -velocity_step);

  Trajectory traj(256);
  traj.AddState(0.0, State(eigenmath::Pose2d::Identity(),
                           ArcVector(start_velocity, 0.0)));
  StateAndTime next_finish = traj.GetFinish();
  for (int c = 0; c < steps_first; ++c) {
    next_finish.state.SetArcVelocity(ArcVector(
        next_finish.state.GetArcVelocity().Translation() + velocity_step, 0.0));
    traj.AddState(next_finish.time, next_finish.state);
    next_finish.state =
        next_finish.state.ExtrapolateConstantVelocityArc(time_step);
    next_finish.time += time_step;
  }
  {
    traj.AddState(next_finish.time, next_finish.state);
    next_finish.state =
        next_finish.state.ExtrapolateConstantVelocityArc(time_flat);
    next_finish.time += time_flat;
  }
  for (int c = 0; c < steps_second; ++c) {
    if (c < steps_second - 1) {
      next_finish.state.SetArcVelocity(ArcVector(
          next_finish.state.GetArcVelocity().Translation() - velocity_step,
          0.0));
    } else {
      next_finish.state.SetArcVelocity(ArcVector(0.0, 0.0));
    }
    traj.AddState(next_finish.time, next_finish.state);
    next_finish.state =
        next_finish.state.ExtrapolateConstantVelocityArc(time_step);
    next_finish.time += time_step;
  }
  traj.AddState(next_finish.time, next_finish.state);
  distance2 = traj.GetFinish().state.GetPose().translation()[0];
  EXPECT_NEAR(distance1, distance2, kEpsilon)
      << "matching against trajectory integration";
}

TEST(TrajectoryLimits, GetLinearVelocityRamp) {
  Kinematics kinematics;
  DynamicLimits dd_limits;
  TrajectoryLimits traj_limits;
  CreateTrajectoryLimits(&kinematics, &dd_limits, &traj_limits);

  double delta_in_velocity = 10.0;
  double velocity_step = 0.5;
  int num_velocity_steps;
  double last_velocity_step;
  traj_limits.GetLinearVelocityRamp(delta_in_velocity, velocity_step,
                                    &num_velocity_steps, &last_velocity_step);
  EXPECT_LE(last_velocity_step, velocity_step);
  EXPECT_NEAR(delta_in_velocity,
              num_velocity_steps * velocity_step + last_velocity_step,
              kEpsilon);

  delta_in_velocity = -10.0;
  traj_limits.GetLinearVelocityRamp(delta_in_velocity, velocity_step,
                                    &num_velocity_steps, &last_velocity_step);
  EXPECT_EQ(last_velocity_step, std::abs(last_velocity_step));
  EXPECT_LE(last_velocity_step, velocity_step);
  EXPECT_NEAR(delta_in_velocity,
              -num_velocity_steps * velocity_step - last_velocity_step,
              kEpsilon);

  delta_in_velocity = 0.5;
  traj_limits.GetLinearVelocityRamp(delta_in_velocity, velocity_step,
                                    &num_velocity_steps, &last_velocity_step);
  EXPECT_LE(last_velocity_step, velocity_step);
  EXPECT_NEAR(delta_in_velocity,
              num_velocity_steps * velocity_step + last_velocity_step,
              kEpsilon);

  delta_in_velocity = 0.01;
  traj_limits.GetLinearVelocityRamp(delta_in_velocity, velocity_step,
                                    &num_velocity_steps, &last_velocity_step);
  EXPECT_LE(last_velocity_step, velocity_step);
  EXPECT_NEAR(delta_in_velocity,
              num_velocity_steps * velocity_step + last_velocity_step,
              kEpsilon);

  delta_in_velocity = 11.0;
  velocity_step = 0.13;
  traj_limits.GetLinearVelocityRamp(delta_in_velocity, velocity_step,
                                    &num_velocity_steps, &last_velocity_step);
  EXPECT_LE(last_velocity_step, velocity_step);
  EXPECT_NEAR(delta_in_velocity,
              num_velocity_steps * velocity_step + last_velocity_step,
              kEpsilon);
}

class BangBangParametersTest : public ::testing::Test {
 public:
  BangBangParametersTest() {
    CreateTrajectoryLimits(&kinematics_, &dd_limits_, &traj_limits_);
  }

  // Calls GetLinearBangBangParameters and checks basic expectations on the
  // output.
  void CheckedGetLinearBangBangParameters(
      double delta_in_position, double start_velocity, double max_velocity,
      double velocity_step, double time_step, int *steps_first,
      double *velocity_step_first, double *time_flat, int *steps_second,
      double *velocity_step_second) {
    const Interval<double> velocity_range(-max_velocity, max_velocity);
    traj_limits_.GetLinearBangBangParameters(
        delta_in_position, start_velocity, velocity_range, velocity_step,
        time_step, steps_first, velocity_step_first, time_flat, steps_second,
        velocity_step_second);

    std::string parameters = absl::StrFormat(
        "start %v max %v step %v time %v\n steps_first: %v\n "
        "velocity_step_first: %v\n time_flat: %v\n steps_second: %v\n "
        "velocity_step_second: %v",
        start_velocity, max_velocity, velocity_step, time_step, *steps_first,
        *velocity_step_first, *time_flat, *steps_second, *velocity_step_second);
    ::testing::ScopedTrace trace(__FILE__, __LINE__, parameters);
    EXPECT_GE(*steps_first, 0);
    EXPECT_GE(*time_flat, 0);
    EXPECT_GE(*steps_second, 0);
    EXPECT_LE(std::abs(*velocity_step_first), velocity_step + kEpsilon);
    EXPECT_LE(std::abs(*velocity_step_second), velocity_step + kEpsilon);
    const double peak_velocity =
        start_velocity + *steps_first * *velocity_step_first;
    EXPECT_LE(std::abs(peak_velocity), max_velocity + kEpsilon);

    double actual_delta = traj_limits_.GetLinearBangBangDisplacement(
        start_velocity, time_step, *steps_first, *velocity_step_first,
        *time_flat, *steps_second, *velocity_step_second);
    EXPECT_NEAR(actual_delta, delta_in_position, kEpsilon);

    double actual_end_velocity = start_velocity +
                                 *velocity_step_first * *steps_first +
                                 *velocity_step_second * (*steps_second - 1);
    EXPECT_LE(std::abs(actual_end_velocity), velocity_step + kEpsilon);
    EXPECT_LE(std::abs(actual_end_velocity), max_velocity);

    // Get a gross upper bound on the time.
    const double duration_upper_bound =
        8 * time_step * (max_velocity / velocity_step) +
        2 * std::abs(delta_in_position) / velocity_step;
    const double actual_duration =
        (*steps_first + *steps_second) * time_step + *time_flat;
    EXPECT_LE(actual_duration, duration_upper_bound);
  }

 protected:
  Kinematics kinematics_;
  DynamicLimits dd_limits_;
  TrajectoryLimits traj_limits_;
};

TEST_F(BangBangParametersTest, GetLinearBangBangParameters) {
  double time_step = kCycleTime;
  double start_velocity = 0.0;
  double delta_in_rotation = M_PI;
  double max_velocity = dd_limits_.MaxArcVelocity().Rotation();
  double velocity_step = dd_limits_.MaxArcAcceleration().Rotation() * time_step;
  int steps_first = 0;
  double velocity_step_first = 0.0;
  double time_flat = 0.0;
  int steps_second = 0;
  double velocity_step_second = 0.0;
  // Check that we can create feasible parameters for a reasonable spread of
  // angular displacement.
  for (double desired_rotation = 0.0; desired_rotation < 2 * M_PI;
       desired_rotation += M_PI / 7) {
    CheckedGetLinearBangBangParameters(
        desired_rotation, start_velocity, max_velocity, velocity_step,
        time_step, &steps_first, &velocity_step_first, &time_flat,
        &steps_second, &velocity_step_second);
  }
  // Check that we can achieve feasible parameters for a reasonable spread of
  // starting velocity.
  for (double init_velocity = -max_velocity; init_velocity < max_velocity;
       init_velocity += max_velocity / 13) {
    CheckedGetLinearBangBangParameters(
        delta_in_rotation, init_velocity, max_velocity, velocity_step,
        time_step, &steps_first, &velocity_step_first, &time_flat,
        &steps_second, &velocity_step_second);
  }
  // Check that distance traveled is the same for symmetric distances.
  {
    start_velocity = 0.707 * max_velocity;
    CheckedGetLinearBangBangParameters(
        delta_in_rotation, start_velocity, max_velocity, velocity_step,
        time_step, &steps_first, &velocity_step_first, &time_flat,
        &steps_second, &velocity_step_second);
    const double rotation1 =
        std::abs(traj_limits_.GetLinearBangBangDisplacement(
            start_velocity, time_step, steps_first, velocity_step_first,
            time_flat, steps_second, velocity_step_second));
    CheckedGetLinearBangBangParameters(
        -delta_in_rotation, -start_velocity, max_velocity, velocity_step,
        time_step, &steps_first, &velocity_step_first, &time_flat,
        &steps_second, &velocity_step_second);
    const double rotation2 =
        std::abs(traj_limits_.GetLinearBangBangDisplacement(
            -start_velocity, time_step, steps_first, velocity_step_first,
            time_flat, steps_second, velocity_step_second));
    EXPECT_NEAR(rotation1, rotation2, kEpsilon) << "symmetric trajectory";
  }
  // Check that a long distance should have a trapezoid.
  {
    start_velocity = 0.0;
    CheckedGetLinearBangBangParameters(
        delta_in_rotation, start_velocity, max_velocity, velocity_step,
        time_step, &steps_first, &velocity_step_first, &time_flat,
        &steps_second, &velocity_step_second);
    EXPECT_GT(steps_first, 0) << "trapezoid";
    EXPECT_GT(time_flat, 0.0) << "trapezoid";
    EXPECT_GT(steps_second, 0) << "trapezoid";
  }
  // Check that starting at a velocity should produce a truncated trapezoid.
  {
    start_velocity = 0.5 * max_velocity;
    CheckedGetLinearBangBangParameters(
        delta_in_rotation, start_velocity, max_velocity, velocity_step,
        time_step, &steps_first, &velocity_step_first, &time_flat,
        &steps_second, &velocity_step_second);
    EXPECT_LT(steps_first, steps_second) << "truncated trapezoid";
  }  // Check that starting at max velocity should only ramp down.
  {
    start_velocity = max_velocity;
    CheckedGetLinearBangBangParameters(
        delta_in_rotation, start_velocity, max_velocity, velocity_step,
        time_step, &steps_first, &velocity_step_first, &time_flat,
        &steps_second, &velocity_step_second);
    EXPECT_EQ(steps_first, 0) << "ramp down only";
    EXPECT_GT(steps_second, 0) << "ramp down only";
  }
  // Check that distance limited trajectories have no plateau.
  {
    start_velocity = 0.0;
    delta_in_rotation =
        0.25 * time_step * max_velocity * max_velocity / velocity_step;
    CheckedGetLinearBangBangParameters(
        delta_in_rotation, start_velocity, max_velocity, velocity_step,
        time_step, &steps_first, &velocity_step_first, &time_flat,
        &steps_second, &velocity_step_second);
    EXPECT_EQ(steps_first, steps_second) << "triangular";
    EXPECT_LT(steps_first, max_velocity / velocity_step) << "triangular";
  }
  // Check that we overshoot if we have to reverse after a quick-stop. This is
  // because we depend on this behavior in its usage, so if it changes dependent
  // code should change.
  {
    start_velocity = max_velocity;
    delta_in_rotation = 0.0;
    CheckedGetLinearBangBangParameters(
        delta_in_rotation, start_velocity, max_velocity, velocity_step,
        time_step, &steps_first, &velocity_step_first, &time_flat,
        &steps_second, &velocity_step_second);
    const double displacement = traj_limits_.GetLinearBangBangDisplacement(
        start_velocity, time_step, steps_first, velocity_step_first, time_flat,
        steps_second, velocity_step_second);
    EXPECT_NEAR(displacement, delta_in_rotation, kEpsilon)
        << "expect no displacement overshoot";
  }
}

TEST_F(BangBangParametersTest,
       RegressionCasesSmallDisplacementNearZeroVelocity) {
  int steps_first = 0;
  double velocity_step_first = 0.0;
  double time_flat = 0.0;
  int steps_second = 0;
  double velocity_step_second = 0.0;
  const double max_velocity = 5.12821;  // 5.0;
  const double velocity_step = 0.04;
  const double time_step = 0.01;
  for (double start_velocity = -1e-5; start_velocity < 1e-5;
       start_velocity += 1e-7) {
    for (double delta = -1e-2; delta < 1e-2; delta += 1e-4) {
      CheckedGetLinearBangBangParameters(delta, start_velocity, max_velocity,
                                         velocity_step, time_step, &steps_first,
                                         &velocity_step_first, &time_flat,
                                         &steps_second, &velocity_step_second);
    }
  }
}

TEST_F(BangBangParametersTest, RegressionCasesSmallDisplacement) {
  int steps_first = 0;
  double velocity_step_first = 0.0;
  double time_flat = 0.0;
  int steps_second = 0;
  double velocity_step_second = 0.0;
  const double delta = -0.00634739;
  const double start_velocity = 0.389817;
  const double max_velocity = 1.2;
  const double velocity_step = 0.2;
  const double time_step = 0.05;
  CheckedGetLinearBangBangParameters(delta, start_velocity, max_velocity,
                                     velocity_step, time_step, &steps_first,
                                     &velocity_step_first, &time_flat,
                                     &steps_second, &velocity_step_second);
}

TEST_F(BangBangParametersTest, RandomSamples) {
  eigenmath::TestGenerator generator(eigenmath::kGeneratorTestSeed);

  int steps_first = 0;
  double velocity_step_first = 0.0;
  double time_flat = 0.0;
  int steps_second = 0;
  double velocity_step_second = 0.0;

  for (int i = 0; i < 1e6; ++i) {
    const double max_speed = 2.0;
    const double time_step = 0.05;
    const double velocity_step = absl::Uniform(generator, 0.01, 0.4);
    const double delta = absl::Uniform(generator, -4, 4);
    // Includes out-of-bounds start velocities.
    const double velocity = absl::Uniform(generator, -2.2, 2.2);
    CheckedGetLinearBangBangParameters(
        delta, velocity, max_speed, velocity_step, time_step, &steps_first,
        &velocity_step_first, &time_flat, &steps_second, &velocity_step_second);
  }
}

TEST(TrajectoryLimits, AppendMinimumTimeRotationTrajectory) {
  Kinematics kinematics;
  DynamicLimits dd_limits;
  TrajectoryLimits traj_limits;
  CreateTrajectoryLimits(&kinematics, &dd_limits, &traj_limits);
  Trajectory traj(256);
  // Counter-clockwise - slow.
  double start_angle = 0.0;
  double final_angle = 2.5;
  double aggressivity = 0.5;
  double time_step = kCycleTime;
  double start_linear_velocity = 5.0;
  double start_angular_velocity = 0.0;
  State start_state;
  eigenmath::Pose2d end_pose = eigenmath::Pose2d({0.0, 0.0}, final_angle);
  start_state.SetPose(eigenmath::Pose2d({0.0, 0.0}, start_angle));
  start_state.SetArcVelocity(
      ArcVector(start_linear_velocity, start_angular_velocity));
  traj.Clear();
  traj.AddState(0.0, start_state);
  EXPECT_TRUE(traj_limits.AppendBangBangRotationStoppingTrajectory(
      dd_limits, end_pose.so2(), aggressivity, aggressivity, time_step, &traj));
  State end_state = traj.GetFinish().state;
  EXPECT_NEAR(end_state.GetPose().angle(), final_angle, kEpsilon);
  EXPECT_NEAR(end_state.GetArcVelocity().Translation(), 0.0, kEpsilon);
  EXPECT_NEAR(end_state.GetArcVelocity().Rotation(), 0.0, kEpsilon);
  eigenmath::SO2dInterval orientation_span(start_angle, final_angle);
  for (auto &state_and_time : traj.GetStateIteratorRange()) {
    EXPECT_LT(state_and_time.state.GetArcVelocity().Rotation(),
              dd_limits.MaxArcVelocity().Rotation() + kEpsilon);
    EXPECT_TRUE(
        orientation_span.Contains(state_and_time.state.GetPose().so2(), 1e-3));
  }
  // Test that too small of a trajectory will return false.
  Trajectory traj_too_small(1);
  EXPECT_FALSE(traj_limits.AppendBangBangRotationStoppingTrajectory(
      dd_limits, end_pose.so2(), aggressivity, aggressivity, time_step,
      &traj_too_small));
  // Sweep through a range of angles - fast.
  aggressivity = 1.0;
  for (final_angle = -M_PI; final_angle < M_PI; final_angle += M_PI / 11) {
    end_pose = eigenmath::Pose2d({0.0, 0.0}, final_angle);
    traj.Clear();
    traj.AddState(0.0, start_state);
    EXPECT_TRUE(traj_limits.AppendBangBangRotationStoppingTrajectory(
        dd_limits, end_pose.so2(), aggressivity, aggressivity, time_step,
        &traj));
    end_state = traj.GetFinish().state;
    eigenmath::Pose2d end_pose_error = end_pose.inverse() * end_state.GetPose();
    EXPECT_NEAR(end_pose_error.angle(), 0.0, kEpsilon);
    EXPECT_NEAR(end_state.GetArcVelocity().Translation(), 0.0, kEpsilon);
    EXPECT_NEAR(end_state.GetArcVelocity().Rotation(), 0.0, kEpsilon);
    for (auto &state_and_time : traj.GetStateIteratorRange()) {
      EXPECT_LT(state_and_time.state.GetArcVelocity().Rotation(),
                dd_limits.MaxArcVelocity().Rotation() + kEpsilon);
    }
  }
  // Sweep through a range of starting velocities - fast.
  final_angle = 2.5;
  end_pose = eigenmath::Pose2d({0.0, 0.0}, final_angle);
  aggressivity = 1.0;
  for (start_angular_velocity = -dd_limits.MaxArcVelocity().Rotation();
       start_angular_velocity <= dd_limits.MaxArcVelocity().Rotation();
       start_angular_velocity += dd_limits.MaxArcVelocity().Rotation() / 11) {
    traj.Clear();
    traj.AddState(0.0, start_state);
    EXPECT_TRUE(traj_limits.AppendBangBangRotationStoppingTrajectory(
        dd_limits, end_pose.so2(), aggressivity, aggressivity, time_step,
        &traj));
    end_state = traj.GetFinish().state;
    eigenmath::Pose2d end_pose_error = end_pose.inverse() * end_state.GetPose();
    EXPECT_NEAR(end_pose_error.angle(), 0.0, kEpsilon);
    EXPECT_NEAR(end_state.GetArcVelocity().Translation(), 0.0, kEpsilon);
    EXPECT_NEAR(end_state.GetArcVelocity().Rotation(), 0.0, kEpsilon);
    for (auto &state_and_time : traj.GetStateIteratorRange()) {
      EXPECT_LT(state_and_time.state.GetArcVelocity().Rotation(),
                dd_limits.MaxArcVelocity().Rotation() + kEpsilon);
    }
  }
  // Called with no delta - expect only a single element.
  start_angle = 0.0;
  final_angle = 0.0;
  aggressivity = 0.5;
  time_step = kCycleTime;
  start_linear_velocity = 0.0;
  start_angular_velocity = 0.0;
  end_pose = eigenmath::Pose2d({0.0, 0.0}, final_angle);
  start_state.SetPose(eigenmath::Pose2d({0.0, 0.0}, start_angle));
  start_state.SetArcVelocity(
      ArcVector(start_linear_velocity, start_angular_velocity));
  traj.Clear();
  traj.AddState(0.0, start_state);
  EXPECT_TRUE(traj_limits.AppendBangBangRotationStoppingTrajectory(
      dd_limits, end_pose.so2(), aggressivity, aggressivity, time_step, &traj));
  end_state = traj.GetFinish().state;
  eigenmath::Pose2d end_pose_error = end_pose.inverse() * end_state.GetPose();
  EXPECT_NEAR(end_pose_error.angle(), 0.0, kEpsilon);
  EXPECT_NEAR(end_state.GetArcVelocity().Translation(), 0.0, kEpsilon);
  EXPECT_NEAR(end_state.GetArcVelocity().Rotation(), 0.0, kEpsilon);
  EXPECT_EQ(traj.GetSize(), 1);
}

// Similar as above, testing for opposite turning direction.  Trajectories can
// be longer than for short rotation.
TEST(TrajectoryLimits,
     AppendBangBangRotationStoppingTrajectoryReverseRotation) {
  Kinematics kinematics;
  DynamicLimits dd_limits;
  TrajectoryLimits traj_limits;
  CreateTrajectoryLimits(&kinematics, &dd_limits, &traj_limits);
  Trajectory traj(512);
  // Counter-clockwise - slow.
  double start_angle = 0.0;
  double final_angle = 2.5;
  double aggressivity = 0.5;
  double time_step = kCycleTime;
  double start_linear_velocity = 5.0;
  double start_angular_velocity = 0.0;
  State start_state;
  eigenmath::Pose2d end_pose = eigenmath::Pose2d({0.0, 0.0}, final_angle);
  start_state.SetPose(eigenmath::Pose2d({0.0, 0.0}, start_angle));
  start_state.SetArcVelocity(
      ArcVector(start_linear_velocity, start_angular_velocity));
  traj.Clear();
  traj.AddState(0.0, start_state);
  EXPECT_TRUE(traj_limits.AppendBangBangRotationStoppingTrajectory(
      dd_limits, end_pose.so2(), aggressivity, aggressivity, time_step, &traj,
      /*long_rotation=*/true));
  State end_state = traj.GetFinish().state;
  EXPECT_NEAR(end_state.GetPose().angle(), final_angle, kEpsilon);
  EXPECT_NEAR(end_state.GetArcVelocity().Translation(), 0.0, kEpsilon);
  EXPECT_NEAR(end_state.GetArcVelocity().Rotation(), 0.0, kEpsilon);
  eigenmath::SO2dInterval orientation_span(final_angle, start_angle);
  for (auto &state_and_time : traj.GetStateIteratorRange()) {
    EXPECT_LT(state_and_time.state.GetArcVelocity().Rotation(),
              dd_limits.MaxArcVelocity().Rotation() + kEpsilon);
    EXPECT_TRUE(
        orientation_span.Contains(state_and_time.state.GetPose().so2(), 1e-3));
  }
  // Test that too small of a trajectory will return false.
  Trajectory traj_too_small(1);
  EXPECT_FALSE(traj_limits.AppendBangBangRotationStoppingTrajectory(
      dd_limits, end_pose.so2(), aggressivity, aggressivity, time_step,
      &traj_too_small, /*long_rotation=*/true));
  // Sweep through a range of angles - fast.
  aggressivity = 1.0;
  for (final_angle = -M_PI; final_angle < M_PI; final_angle += M_PI / 11) {
    end_pose = eigenmath::Pose2d({0.0, 0.0}, final_angle);
    traj.Clear();
    traj.AddState(0.0, start_state);
    EXPECT_TRUE(traj_limits.AppendBangBangRotationStoppingTrajectory(
        dd_limits, end_pose.so2(), aggressivity, aggressivity, time_step, &traj,
        /*long_rotation=*/true));
    end_state = traj.GetFinish().state;
    eigenmath::Pose2d end_pose_error = end_pose.inverse() * end_state.GetPose();
    EXPECT_NEAR(end_pose_error.angle(), 0.0, kEpsilon);
    EXPECT_NEAR(end_state.GetArcVelocity().Translation(), 0.0, kEpsilon);
    EXPECT_NEAR(end_state.GetArcVelocity().Rotation(), 0.0, kEpsilon);
    for (auto &state_and_time : traj.GetStateIteratorRange()) {
      EXPECT_LT(state_and_time.state.GetArcVelocity().Rotation(),
                dd_limits.MaxArcVelocity().Rotation() + kEpsilon);
    }
  }
  // Sweep through a range of starting velocities - fast.
  final_angle = 2.5;
  end_pose = eigenmath::Pose2d({0.0, 0.0}, final_angle);
  aggressivity = 1.0;
  for (start_angular_velocity = -dd_limits.MaxArcVelocity().Rotation();
       start_angular_velocity <= dd_limits.MaxArcVelocity().Rotation();
       start_angular_velocity += dd_limits.MaxArcVelocity().Rotation() / 11) {
    traj.Clear();
    traj.AddState(0.0, start_state);
    EXPECT_TRUE(traj_limits.AppendBangBangRotationStoppingTrajectory(
        dd_limits, end_pose.so2(), aggressivity, aggressivity, time_step, &traj,
        /*long_rotation=*/true));
    end_state = traj.GetFinish().state;
    eigenmath::Pose2d end_pose_error = end_pose.inverse() * end_state.GetPose();
    EXPECT_NEAR(end_pose_error.angle(), 0.0, kEpsilon);
    EXPECT_NEAR(end_state.GetArcVelocity().Translation(), 0.0, kEpsilon);
    EXPECT_NEAR(end_state.GetArcVelocity().Rotation(), 0.0, kEpsilon);
    for (auto &state_and_time : traj.GetStateIteratorRange()) {
      EXPECT_LT(state_and_time.state.GetArcVelocity().Rotation(),
                dd_limits.MaxArcVelocity().Rotation() + kEpsilon);
    }
  }
}

TEST(TrajectoryLimits, AppendStoppingTrajectory) {
  Kinematics kinematics;
  DynamicLimits dd_limits;
  TrajectoryLimits traj_limits;
  CreateTrajectoryLimits(&kinematics, &dd_limits, &traj_limits);
  const StateAndTime initial_state{.time = 1.0, .state = State({}, {1.0, 0.0})};
  diff_drive::Trajectory trajectory(256);
  ASSERT_TRUE(trajectory.AddState(initial_state.time, initial_state.state));
  // trajectory.AppendStates(initial_state);
  constexpr double duration = 3.0;
  auto current_state = initial_state;
  ASSERT_TRUE(traj_limits.AppendStoppingTrajectory(
      kinematics, /*aggressivity_factor=*/1.0,
      /*total_duration=*/duration, &current_state, &trajectory));
  EXPECT_THAT(trajectory.GetStart().time, initial_state.time);
  EXPECT_THAT(trajectory.GetStart().state.GetPose(),
              IsApprox(initial_state.state.GetPose()));
  EXPECT_THAT(trajectory.GetStart().state.GetArcVelocity(),
              IsApprox(initial_state.state.GetArcVelocity()));

  EXPECT_THAT(trajectory.GetTimeSpan().Length(), ::testing::Ge(duration));
  EXPECT_THAT(trajectory.GetFinish().time,
              ::testing::Ge(initial_state.time + duration));
  EXPECT_THAT(trajectory.GetFinish().state.GetPose(),
              IsApprox(current_state.state.GetPose()));
  EXPECT_THAT(trajectory.GetFinish().state.GetArcVelocity(),
              IsApprox(current_state.state.GetArcVelocity()));
  // Want to meet zero exactly.
  EXPECT_THAT(current_state.state.GetArcVelocity(), ArcVector(0, 0));
}

TEST(TrajectoryLimits,
     AppendStoppingTrajectoryRegenerateFromIntermediateSampledPoint) {
  Kinematics kinematics;
  DynamicLimits dd_limits;
  TrajectoryLimits traj_limits;
  CreateTrajectoryLimits(&kinematics, &dd_limits, &traj_limits);
  StateAndTime current_state{.time = 1.0, .state = State({}, {1.0, 0.0})};
  diff_drive::Trajectory trajectory(256);
  ASSERT_TRUE(trajectory.AddState(current_state.time, current_state.state));
  constexpr double duration = 3.0;
  ASSERT_TRUE(traj_limits.AppendStoppingTrajectory(
      kinematics, /*aggressivity_factor=*/1.0,
      /*total_duration=*/duration, &current_state, &trajectory));
  const Trajectory first_trajectory = trajectory;
  auto it = first_trajectory.BeginState();
  const int offset = 4;
  ASSERT_THAT(first_trajectory.GetSize(), ::testing::Gt(offset));
  it += offset;
  current_state = *it;
  EXPECT_THAT(current_state.state.GetArcVelocity().Translation(),
              ::testing::Gt(0.5));
  const double time_offset = it->time - first_trajectory.GetTimeSpan().min();

  // Regenerate the trialing part of the trajectory.
  trajectory.Clear();
  ASSERT_TRUE(trajectory.AddState(current_state.time, current_state.state));
  current_state = trajectory.GetFinish();
  ASSERT_TRUE(traj_limits.AppendStoppingTrajectory(
      kinematics, /*aggressivity_factor=*/1.0,
      /*total_duration=*/duration - time_offset, &current_state, &trajectory));

  // Ensure that we get the same points.
  ASSERT_THAT(trajectory.GetSize() + offset,
              ::testing::Eq(first_trajectory.GetSize()));

  auto trailing_end = first_trajectory.EndState();
  auto trailing_range = genit::MakeIteratorRange(it, trailing_end);
  for (auto [regen, orig] :
       genit::ZipRange(trajectory.GetStateIteratorRange(), trailing_range)) {
    EXPECT_THAT(regen.time, testing::DoubleEq(orig.time));
    EXPECT_THAT(regen.state.GetPose(), IsApprox(orig.state.GetPose()));
    EXPECT_THAT(regen.state.GetArcVelocity(),
                IsApprox(orig.state.GetArcVelocity()));
  }
}

TEST(TrajectoryLimits,
     AppendStoppingTrajectoryRegenerateFromIntermediateInterpolatedPoints) {
  Kinematics kinematics;
  DynamicLimits dd_limits;
  TrajectoryLimits traj_limits;
  CreateTrajectoryLimits(&kinematics, &dd_limits, &traj_limits);
  StateAndTime current_state{.time = 1.0,
                             .state = State({}, {0.991832571623, 0.0})};
  diff_drive::Trajectory trajectory(256);
  ASSERT_TRUE(trajectory.AddState(current_state.time, current_state.state));
  constexpr double duration = 3.0;
  ASSERT_TRUE(traj_limits.AppendStoppingTrajectory(
      kinematics, /*aggressivity_factor=*/1.0,
      /*total_duration=*/duration, &current_state, &trajectory));
  ASSERT_THAT(trajectory.GetSize(), ::testing::Gt(2));
  // const double initial_stopping_distance =
  // trajectory.ComputeTotalCordLength();
  const double sampling_timestep =
      std::next(trajectory.BeginState())->time - trajectory.BeginState()->time;
  const Trajectory first_trajectory = trajectory;
  const double initial_stopping_distance =
      trajectory.GetFinish().state.GetPose().translation().x();

  // The stopping distance should at most increase by a factor of two.
  for (double multiple : {1.1, 1.9, 2.0}) {
    std::string loop_message = absl::StrFormat("multiple is %.2f", multiple);
    SCOPED_TRACE(loop_message.c_str());

    trajectory = first_trajectory;
    const double time_step = multiple * sampling_timestep;
    double time = 0.0;
    while (trajectory.GetStart().state.GetArcVelocity().squaredNorm() > 1e-3) {
      // Move forward along the trajectory.
      time += time_step;
      const auto interpolated_state = (trajectory.BeginInTime() + time_step);

      // Regenerate the stopping segment from the new state.
      trajectory.Clear();
      ASSERT_TRUE(trajectory.AddState(interpolated_state.GetTime(),
                                      interpolated_state.GetState()));
      current_state = trajectory.GetFinish();
      ASSERT_TRUE(traj_limits.AppendStoppingTrajectory(
          kinematics, /*aggressivity_factor=*/1.0,
          /*total_duration=*/duration - time, &current_state, &trajectory));
    }

    // The time aliasing delays the stop at every cycle by extending the
    // previous velocity segment.  Hence, we can only expect to stay below a
    // multiple of the initial stopping time.
    const double stopping_distance =
        trajectory.GetFinish().state.GetPose().translation().x();
    EXPECT_THAT(stopping_distance,
                ::testing::Lt(2 * initial_stopping_distance));
  }
}

TEST(TrajectoryLimits, AppendMinimumTimeRotationTrajectoryRegressions) {
  {
    const Kinematics kinematics(WheelVector(0.0754, 0.0754), 0.312);
    const DynamicLimits dd_limits(
        kinematics, WheelVector(-4, -4), WheelVector(4, 4),
        WheelVector(-120, -120), WheelVector(120, 120), ArcVector(-0.003, -0.6),
        ArcVector(0.3, 0.6), ArcVector(-2, -6), ArcVector(2, 6));
    const double time_step = 0.05;
    const TrajectoryLimits traj_limits(dd_limits, time_step);
    const double aggressivity = 0.5;

    eigenmath::Pose2d goal_pose =
        eigenmath::Pose2d({-4.151150256e-06, -1.936340012e-06}, -2.705136311);
    State initial_state;
    initial_state.SetPose(eigenmath::Pose2d({0.0, 0.0}, -0.0075));
    initial_state.SetArcVelocity(ArcVector(0.0, -0.3 + 1e-8));

    Trajectory traj(256);
    traj.AddState(0.0, initial_state);
    EXPECT_TRUE(traj_limits.AppendBangBangRotationStoppingTrajectory(
        dd_limits, goal_pose.so2(), aggressivity, aggressivity, time_step,
        &traj));

    for (auto &state_pt : traj.GetStateIteratorRange()) {
      EXPECT_TRUE(dd_limits.VelocityLimits().IsInBounds(
          state_pt.state.GetArcVelocity()));
    }
  }
  {
    const Kinematics kinematics(WheelVector(0.0754, 0.0754), 0.312);
    const DynamicLimits dd_limits(
        kinematics, WheelVector(-4, -4), WheelVector(4, 4),
        WheelVector(-120, -120), WheelVector(120, 120), ArcVector(-0.003, -0.6),
        ArcVector(0.3, 0.6), ArcVector(-2, -6), ArcVector(2, 6));
    const double time_step = 0.05;
    const TrajectoryLimits traj_limits(dd_limits, time_step);
    const double aggressivity = 0.5;

    eigenmath::Pose2d goal_pose =
        eigenmath::Pose2d({-4.151150256e-06, -1.936340012e-06}, -2.705136311);
    State initial_state;
    initial_state.SetPose(eigenmath::Pose2d({0.0, 0.0}, -0.0075));
    initial_state.SetArcVelocity(ArcVector(0.0, -0.3 - 1e-8));

    Trajectory traj(256);
    traj.AddState(0.0, initial_state);
    EXPECT_TRUE(traj_limits.AppendBangBangRotationStoppingTrajectory(
        dd_limits, goal_pose.so2(), aggressivity, aggressivity, time_step,
        &traj));

    for (auto &state_pt : traj.GetStateIteratorRange()) {
      EXPECT_TRUE(dd_limits.VelocityLimits().IsInBounds(
          state_pt.state.GetArcVelocity()));
    }
  }
  {
    const Kinematics kinematics(WheelVector(0.0754, 0.0754), 0.312);
    const DynamicLimits dd_limits(
        kinematics, WheelVector(-12.6, -12.6), WheelVector(12.6, 12.6),
        WheelVector(-120, -120), WheelVector(120, 120), ArcVector(-0.008, -1.2),
        ArcVector(0.8, 1.2), ArcVector(-8, -6), ArcVector(8, 6));
    const double time_step = 0.05;
    const TrajectoryLimits traj_limits(dd_limits, time_step);
    const double aggressivity = 0.5;
    const double realignment_aggressivity = 0.1;

    eigenmath::Pose2d goal_pose =
        eigenmath::Pose2d({1.29456950145, -26.9077603098}, 2.19105525027);
    State initial_state;
    initial_state.SetPose(
        eigenmath::Pose2d({1.29290149369, -26.9146945691}, 1.30548522602));
    initial_state.SetArcVelocity(ArcVector(0.142661432735, 1.2));

    Trajectory traj(256);
    traj.AddState(0.0, initial_state);
    EXPECT_TRUE(traj_limits.AppendBangBangRotationStoppingTrajectory(
        dd_limits, goal_pose.so2(), aggressivity, realignment_aggressivity,
        time_step, &traj));

    EXPECT_LT(traj.GetTimeSpan().max(), 4.0);

    for (auto &state_pt : traj.GetStateIteratorRange()) {
      EXPECT_TRUE(dd_limits.VelocityLimits().IsInBounds(
          state_pt.state.GetArcVelocity()));
    }
  }
}

void BM_GetLinearBangBangParameters(benchmark::State &state) {
  Kinematics kinematics;
  DynamicLimits dd_limits;
  TrajectoryLimits traj_limits;
  CreateTrajectoryLimits(&kinematics, &dd_limits, &traj_limits);

  const double max_speed = 2.0;
  const double velocity_step = 0.04;
  const double time_step = 0.01;
  eigenmath::TestGenerator generator(eigenmath::kGeneratorTestSeed);

  constexpr int kSamples = 10000;
  std::vector<double> displacements(kSamples);
  for (double &delta : displacements) {
    delta = absl::Uniform(generator, -2, 20);
  }
  std::vector<double> velocities(kSamples);
  for (double &velocity : velocities) {
    velocity = absl::Uniform(generator, -2, 2);
  }

  int steps_first = 0;
  double velocity_step_first = 0.0;
  double time_flat = 0.0;
  int steps_second = 0;
  double velocity_step_second = 0.0;

  int sample = 0;
  for (auto _ : state) {
    traj_limits.GetLinearBangBangParameters(
        displacements[sample], velocities[sample], {-max_speed, max_speed},
        velocity_step, time_step, &steps_first, &velocity_step_first,
        &time_flat, &steps_second, &velocity_step_second);
    if (++sample >= kSamples) {
      sample = 0;
    }
  }
}
BENCHMARK(BM_GetLinearBangBangParameters);

}  // namespace
}  // namespace mobility::diff_drive
