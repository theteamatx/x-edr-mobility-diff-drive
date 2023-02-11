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

#include "diff_drive/kinematics.h"

#include <cmath>

#include "gtest/gtest.h"

namespace mobility::diff_drive {
namespace {

constexpr bool kDebug = false;
constexpr double kEpsilon = 1.0e-6;

TEST(Kinematics, FwdRevSystematic) {
  Kinematics kinematics({0.11, 0.99}, 0.401);

  ArcVector arc_in;
  for (arc_in.Translation() = -0.1; arc_in.Translation() < 0.11;
       arc_in.Translation() += 0.05) {
    for (arc_in.Rotation() = -0.1; arc_in.Rotation() < 0.11;
         arc_in.Rotation() += 0.05) {
      const auto wheel = kinematics.ComputeInverseKinematics(arc_in);
      const auto arc_out = kinematics.ComputeForwardKinematics(wheel);

      if (kDebug) {
        fprintf(stderr, "kinematics: %f %f --> %f %f --> %f %f\n",
                arc_in.Translation(), arc_in.Rotation(), wheel.Left(),
                wheel.Right(), arc_out.Translation(), arc_out.Rotation());
      }

      ASSERT_NEAR(arc_in.Translation(), arc_out.Translation(), kEpsilon);
      ASSERT_NEAR(arc_in.Rotation(), arc_out.Rotation(), kEpsilon);
    }
  }
}

TEST(Kinematics, StraightLineArc) {
  auto rel_motion = Kinematics::ComputeIncrement(ArcVector{1.0, 0.0});
  EXPECT_NEAR(rel_motion.translation().x(), 1.0, kEpsilon);
  EXPECT_NEAR(rel_motion.translation().y(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.angle(), 0.0, kEpsilon);

  rel_motion = Kinematics::ComputeIncrement(ArcVector{1.0, kEpsilon});
  EXPECT_NEAR(rel_motion.translation().x(), 1.0, kEpsilon);
  EXPECT_NEAR(rel_motion.translation().y(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.angle(), kEpsilon, kEpsilon);

  rel_motion = Kinematics::ComputeIncrement(ArcVector{1.0, -kEpsilon});
  EXPECT_NEAR(rel_motion.translation().x(), 1.0, kEpsilon);
  EXPECT_NEAR(rel_motion.translation().y(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.angle(), -kEpsilon, kEpsilon);
}

TEST(Kinematics, CircularArc) {
  auto rel_motion = Kinematics::ComputeIncrement(ArcVector{1.0, 2.0 * M_PI});
  EXPECT_NEAR(rel_motion.translation().x(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.translation().y(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.angle(), 0.0, kEpsilon);

  rel_motion = Kinematics::ComputeIncrement(ArcVector{0.5, M_PI - kEpsilon});
  EXPECT_NEAR(rel_motion.translation().x(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.translation().y(), 1.0 / M_PI, kEpsilon);
  EXPECT_NEAR(rel_motion.angle(), M_PI - kEpsilon, kEpsilon);

  rel_motion = Kinematics::ComputeIncrement(ArcVector{0.5, kEpsilon - M_PI});
  EXPECT_NEAR(rel_motion.translation().x(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.translation().y(), -1.0 / M_PI, kEpsilon);
  EXPECT_NEAR(rel_motion.angle(), kEpsilon - M_PI, kEpsilon);

  rel_motion = Kinematics::ComputeIncrement(ArcVector{0.25, 0.5 * M_PI});
  EXPECT_NEAR(rel_motion.translation().x(), 0.5 / M_PI, kEpsilon);
  EXPECT_NEAR(rel_motion.translation().y(), 0.5 / M_PI, kEpsilon);
  EXPECT_NEAR(rel_motion.angle(), 0.5 * M_PI, kEpsilon);

  rel_motion = Kinematics::ComputeIncrement(ArcVector{0.25, -0.5 * M_PI});
  EXPECT_NEAR(rel_motion.translation().x(), 0.5 / M_PI, kEpsilon);
  EXPECT_NEAR(rel_motion.translation().y(), -0.5 / M_PI, kEpsilon);
  EXPECT_NEAR(rel_motion.angle(), -0.5 * M_PI, kEpsilon);

  rel_motion = Kinematics::ComputeIncrement(ArcVector{0.75, 1.5 * M_PI});
  EXPECT_NEAR(rel_motion.translation().x(), -0.5 / M_PI, kEpsilon);
  EXPECT_NEAR(rel_motion.translation().y(), 0.5 / M_PI, kEpsilon);
  EXPECT_NEAR(rel_motion.angle(), -0.5 * M_PI, kEpsilon);

  rel_motion = Kinematics::ComputeIncrement(ArcVector{0.75, -1.5 * M_PI});
  EXPECT_NEAR(rel_motion.translation().x(), -0.5 / M_PI, kEpsilon);
  EXPECT_NEAR(rel_motion.translation().y(), -0.5 / M_PI, kEpsilon);
  EXPECT_NEAR(rel_motion.angle(), 0.5 * M_PI, kEpsilon);
}

TEST(Kinematics, PureSpin) {
  auto rel_motion = Kinematics::ComputeIncrement(ArcVector{0.0, 2.0 * M_PI});
  EXPECT_NEAR(rel_motion.translation().x(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.translation().y(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.angle(), 0.0, kEpsilon);

  rel_motion = Kinematics::ComputeIncrement(ArcVector{0.0, M_PI - kEpsilon});
  EXPECT_NEAR(rel_motion.translation().x(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.translation().y(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.angle(), M_PI - kEpsilon, kEpsilon);

  rel_motion = Kinematics::ComputeIncrement(ArcVector{0.0, kEpsilon - M_PI});
  EXPECT_NEAR(rel_motion.translation().x(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.translation().y(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.angle(), kEpsilon - M_PI, kEpsilon);

  rel_motion = Kinematics::ComputeIncrement(ArcVector{0.0, 0.5 * M_PI});
  EXPECT_NEAR(rel_motion.translation().x(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.translation().y(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.angle(), 0.5 * M_PI, kEpsilon);

  rel_motion = Kinematics::ComputeIncrement(ArcVector{0.0, -0.5 * M_PI});
  EXPECT_NEAR(rel_motion.translation().x(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.translation().y(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.angle(), -0.5 * M_PI, kEpsilon);

  rel_motion = Kinematics::ComputeIncrement(ArcVector{0.0, 1.5 * M_PI});
  EXPECT_NEAR(rel_motion.translation().x(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.translation().y(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.angle(), -0.5 * M_PI, kEpsilon);

  rel_motion = Kinematics::ComputeIncrement(ArcVector{0.0, -1.5 * M_PI});
  EXPECT_NEAR(rel_motion.translation().x(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.translation().y(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.angle(), 0.5 * M_PI, kEpsilon);
}

TEST(Kinematics, Configure) {
  Kinematics kinematics1({0.1, 0.1}, 0.2);

  auto orig_wheel =
      kinematics1.ComputeInverseKinematics(ArcVector{1.0, 2.0 * M_PI});

  Kinematics kinematics2({1.0, 1.0}, 2.0);
  kinematics2.Configure({0.1, 0.1}, 0.2);

  auto new_wheel =
      kinematics2.ComputeInverseKinematics(ArcVector{1.0, 2.0 * M_PI});

  EXPECT_NEAR(orig_wheel.Right(), new_wheel.Right(), kEpsilon);
  EXPECT_NEAR(orig_wheel.Left(), new_wheel.Left(), kEpsilon);
}

TEST(Kinematics, ComputeIncrementWheelVector) {
  Kinematics kinematics({0.1, 0.1}, 0.2);

  double delta_trans = 0;
  auto wheel = kinematics.ComputeInverseKinematics(ArcVector{1.0, 0.0});
  auto rel_motion = kinematics.ComputeIncrement(wheel, &delta_trans);
  EXPECT_NEAR(delta_trans, 1.0, kEpsilon);
  EXPECT_NEAR(rel_motion.translation().x(), 1.0, kEpsilon);
  EXPECT_NEAR(rel_motion.translation().y(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.angle(), 0.0, kEpsilon);

  wheel = kinematics.ComputeInverseKinematics(ArcVector{1.0, 2.0 * M_PI});
  rel_motion = kinematics.ComputeIncrement(wheel, &delta_trans);
  EXPECT_NEAR(delta_trans, 1.0, kEpsilon);
  EXPECT_NEAR(rel_motion.translation().x(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.translation().y(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.angle(), 0.0, kEpsilon);

  wheel = kinematics.ComputeInverseKinematics(ArcVector{0.0, 2.0 * M_PI});
  rel_motion = kinematics.ComputeIncrement(wheel, &delta_trans);
  EXPECT_NEAR(delta_trans, 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.translation().x(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.translation().y(), 0.0, kEpsilon);
  EXPECT_NEAR(rel_motion.angle(), 0.0, kEpsilon);
}

}  // namespace
}  // namespace mobility::diff_drive
