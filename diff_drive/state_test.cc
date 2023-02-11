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

#include "diff_drive/state.h"

#include <cmath>

#include "gtest/gtest.h"

namespace mobility::diff_drive {
namespace {

constexpr double kEpsilon = 1.0e-6;

TEST(State, BasicMemberFunctions) {
  State test_state(
      eigenmath::Pose2d(eigenmath::Vector2d(1.0, 3.0), 0.5 * M_PI));
  EXPECT_NEAR(test_state.GetPose().translation().x(), 1.0, kEpsilon);
  EXPECT_NEAR(test_state.GetPose().translation().y(), 3.0, kEpsilon);
  EXPECT_NEAR(test_state.GetPose().angle(), 0.5 * M_PI, kEpsilon);

  EXPECT_NEAR(test_state.GetArcVelocity().Translation(), 0.0, kEpsilon);
  EXPECT_NEAR(test_state.GetArcVelocity().Rotation(), 0.0, kEpsilon);

  test_state.SetArcVelocity(ArcVector{1.5, 0.5});
  EXPECT_NEAR(test_state.GetArcVelocity().Translation(), 1.5, kEpsilon);
  EXPECT_NEAR(test_state.GetArcVelocity().Rotation(), 0.5, kEpsilon);
}

TEST(State, Extrapolate) {
  State test_state(eigenmath::Pose2d(eigenmath::Vector2d(1.0, 3.0), 0.5 * M_PI),
                   ArcVector{1.0, 2.0 * M_PI});
  EXPECT_NEAR(test_state.GetPose().translation().x(), 1.0, kEpsilon);
  EXPECT_NEAR(test_state.GetPose().translation().y(), 3.0, kEpsilon);
  EXPECT_NEAR(test_state.GetPose().angle(), 0.5 * M_PI, kEpsilon);
  EXPECT_NEAR(test_state.GetArcVelocity().Translation(), 1.0, kEpsilon);
  EXPECT_NEAR(test_state.GetArcVelocity().Rotation(), 2.0 * M_PI, kEpsilon);

  State quarter_turn =
      test_state.ExtrapolateConstantVelocityArc(0.25 - kEpsilon / (2.0 * M_PI));
  EXPECT_NEAR(quarter_turn.GetPose().translation().x(), 1.0 - 0.5 / M_PI,
              kEpsilon);
  EXPECT_NEAR(quarter_turn.GetPose().translation().y(), 3.0 + 0.5 / M_PI,
              kEpsilon);
  EXPECT_NEAR(quarter_turn.GetPose().angle(), M_PI - kEpsilon, kEpsilon);
  EXPECT_NEAR(quarter_turn.GetArcVelocity().Translation(), 1.0, kEpsilon);
  EXPECT_NEAR(quarter_turn.GetArcVelocity().Rotation(), 2.0 * M_PI, kEpsilon);

  State half_turn = test_state.ExtrapolateConstantVelocityArc(0.5);
  EXPECT_NEAR(half_turn.GetPose().translation().x(), 1.0 - 1.0 / M_PI,
              kEpsilon);
  EXPECT_NEAR(half_turn.GetPose().translation().y(), 3.0, kEpsilon);
  EXPECT_NEAR(half_turn.GetPose().angle(), -0.5 * M_PI, kEpsilon);
  EXPECT_NEAR(half_turn.GetArcVelocity().Translation(), 1.0, kEpsilon);
  EXPECT_NEAR(half_turn.GetArcVelocity().Rotation(), 2.0 * M_PI, kEpsilon);

  State negquarter_turn = test_state.ExtrapolateConstantVelocityArc(-0.25);
  EXPECT_NEAR(negquarter_turn.GetPose().translation().x(), 1.0 - 0.5 / M_PI,
              kEpsilon);
  EXPECT_NEAR(negquarter_turn.GetPose().translation().y(), 3.0 - 0.5 / M_PI,
              kEpsilon);
  EXPECT_NEAR(negquarter_turn.GetPose().angle(), 0.0, kEpsilon);
  EXPECT_NEAR(negquarter_turn.GetArcVelocity().Translation(), 1.0, kEpsilon);
  EXPECT_NEAR(negquarter_turn.GetArcVelocity().Rotation(), 2.0 * M_PI,
              kEpsilon);

  State neghalf_turn = test_state.ExtrapolateConstantVelocityArc(-0.5);
  EXPECT_NEAR(neghalf_turn.GetPose().translation().x(), 1.0 - 1.0 / M_PI,
              kEpsilon);
  EXPECT_NEAR(neghalf_turn.GetPose().translation().y(), 3.0, kEpsilon);
  EXPECT_NEAR(neghalf_turn.GetPose().angle(), -0.5 * M_PI, kEpsilon);
  EXPECT_NEAR(neghalf_turn.GetArcVelocity().Translation(), 1.0, kEpsilon);
  EXPECT_NEAR(neghalf_turn.GetArcVelocity().Rotation(), 2.0 * M_PI, kEpsilon);
}

TEST(State, GetRelativeTo) {
  State s_a(eigenmath::Pose2d(eigenmath::Vector2d(1.0, 3.0), 0.5 * M_PI),
            ArcVector(1.5, 0.5));
  State s_b(eigenmath::Pose2d(eigenmath::Vector2d(-1.0, 2.0), -0.5 * M_PI),
            ArcVector(0.5, 1.0));
  State s_c(eigenmath::Pose2d(eigenmath::Vector2d(1.0, -2.0), M_PI),
            ArcVector(-0.5, -0.5));

  State s_ab = s_b.GetRelativeTo(s_a);
  EXPECT_NEAR(s_ab.GetPose().translation().x(), -1.0, kEpsilon);
  EXPECT_NEAR(s_ab.GetPose().translation().y(), 2.0, kEpsilon);
  EXPECT_NEAR(eigenmath::WrapAngle(s_ab.GetPose().angle() - M_PI), 0.0,
              kEpsilon);
  EXPECT_NEAR(s_ab.GetArcVelocity().Translation(), 2.0, kEpsilon);
  EXPECT_NEAR(s_ab.GetArcVelocity().Rotation(), 0.5, kEpsilon);

  State s_ac = s_c.GetRelativeTo(s_a);
  EXPECT_NEAR(s_ac.GetPose().translation().x(), -5.0, kEpsilon);
  EXPECT_NEAR(s_ac.GetPose().translation().y(), 0.0, kEpsilon);
  EXPECT_NEAR(s_ac.GetPose().angle(), 0.5 * M_PI, kEpsilon);
  EXPECT_NEAR(s_ac.GetArcVelocity().Translation(), -0.5, kEpsilon);
  EXPECT_NEAR(s_ac.GetArcVelocity().Rotation(), -1.0, kEpsilon);

  State s_bc = s_c.GetRelativeTo(s_b);
  EXPECT_NEAR(s_bc.GetPose().translation().x(), 4.0, kEpsilon);
  EXPECT_NEAR(s_bc.GetPose().translation().y(), 2.0, kEpsilon);
  EXPECT_NEAR(s_bc.GetPose().angle(), -0.5 * M_PI, kEpsilon);
  EXPECT_NEAR(s_bc.GetArcVelocity().Translation(), -0.5, kEpsilon);
  EXPECT_NEAR(s_bc.GetArcVelocity().Rotation(), -1.5, kEpsilon);

  State s_abc = s_ac.GetRelativeTo(s_ab);
  EXPECT_NEAR(s_abc.GetPose().translation().x(),
              s_bc.GetPose().translation().x(), kEpsilon);
  EXPECT_NEAR(s_abc.GetPose().translation().y(),
              s_bc.GetPose().translation().y(), kEpsilon);
  EXPECT_NEAR(s_abc.GetPose().angle(), s_bc.GetPose().angle(), kEpsilon);
  EXPECT_NEAR(s_abc.GetArcVelocity().Translation(),
              s_bc.GetArcVelocity().Translation(), kEpsilon);
  EXPECT_NEAR(s_abc.GetArcVelocity().Rotation(),
              s_bc.GetArcVelocity().Rotation(), kEpsilon);
}

}  // namespace
}  // namespace mobility::diff_drive
