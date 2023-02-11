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

#include "diff_drive/dynamic_limits.h"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include "diff_drive/kinematics.h"
#include "diff_drive/type_aliases.h"
#include "gtest/gtest.h"
#include "absl/strings/str_format.h"

namespace mobility::diff_drive {

namespace {
constexpr bool kDebug = false;
constexpr double kEpsilon = 1.0e-6;

void ExpectEqualBoxConstraints(const BoxConstraints &limits1,
                               const BoxConstraints &limits2) {
  auto min_wheel_vel_1 = limits1.MinWheelVector();
  auto max_wheel_vel_1 = limits1.MaxWheelVector();
  auto min_arc_vel_1 = limits1.MinArcVector();
  auto max_arc_vel_1 = limits1.MaxArcVector();
  auto min_wheel_vel_2 = limits2.MinWheelVector();
  auto max_wheel_vel_2 = limits2.MaxWheelVector();
  auto min_arc_vel_2 = limits2.MinArcVector();
  auto max_arc_vel_2 = limits2.MaxArcVector();
  EXPECT_NEAR(min_wheel_vel_1.Left(), min_wheel_vel_2.Left(), kEpsilon);
  EXPECT_NEAR(min_wheel_vel_1.Right(), min_wheel_vel_2.Right(), kEpsilon);
  EXPECT_NEAR(max_wheel_vel_1.Left(), max_wheel_vel_2.Left(), kEpsilon);
  EXPECT_NEAR(max_wheel_vel_1.Right(), max_wheel_vel_2.Right(), kEpsilon);
  EXPECT_NEAR(min_arc_vel_1.Translation(), min_arc_vel_2.Translation(),
              kEpsilon);
  EXPECT_NEAR(min_arc_vel_1.Rotation(), min_arc_vel_2.Rotation(), kEpsilon);
  EXPECT_NEAR(max_arc_vel_1.Translation(), max_arc_vel_2.Translation(),
              kEpsilon);
  EXPECT_NEAR(max_arc_vel_1.Rotation(), max_arc_vel_2.Rotation(), kEpsilon);
}

}  // namespace

TEST(BoxConstraints, DefaultCtor) {
  BoxConstraints ddv_limits;
  auto min_wheel_vel = ddv_limits.MinWheelVector();
  auto max_wheel_vel = ddv_limits.MaxWheelVector();
  auto min_arc_vel = ddv_limits.MinArcVector();
  auto max_arc_vel = ddv_limits.MaxArcVector();
  EXPECT_FALSE(ddv_limits.IsEmpty());
  EXPECT_NEAR(min_wheel_vel.Left(), 0.0, kEpsilon);
  EXPECT_NEAR(min_wheel_vel.Right(), 0.0, kEpsilon);
  EXPECT_NEAR(max_wheel_vel.Left(), 0.0, kEpsilon);
  EXPECT_NEAR(max_wheel_vel.Right(), 0.0, kEpsilon);
  EXPECT_NEAR(min_arc_vel.Translation(), 0.0, kEpsilon);
  EXPECT_NEAR(min_arc_vel.Rotation(), 0.0, kEpsilon);
  EXPECT_NEAR(max_arc_vel.Translation(), 0.0, kEpsilon);
  EXPECT_NEAR(max_arc_vel.Rotation(), 0.0, kEpsilon);
}

TEST(BoxConstraints, SymmetricCtor) {
  BoxConstraints ddv_limits(WheelVector(0.5, 0.5), ArcVector(1.0, 0.5));
  auto min_wheel_vel = ddv_limits.MinWheelVector();
  auto max_wheel_vel = ddv_limits.MaxWheelVector();
  auto min_arc_vel = ddv_limits.MinArcVector();
  auto max_arc_vel = ddv_limits.MaxArcVector();
  EXPECT_FALSE(ddv_limits.IsEmpty());
  EXPECT_NEAR(min_wheel_vel.Left(), -0.5, kEpsilon);
  EXPECT_NEAR(min_wheel_vel.Right(), -0.5, kEpsilon);
  EXPECT_NEAR(max_wheel_vel.Left(), 0.5, kEpsilon);
  EXPECT_NEAR(max_wheel_vel.Right(), 0.5, kEpsilon);
  EXPECT_NEAR(min_arc_vel.Translation(), -1.0, kEpsilon);
  EXPECT_NEAR(min_arc_vel.Rotation(), -0.5, kEpsilon);
  EXPECT_NEAR(max_arc_vel.Translation(), 1.0, kEpsilon);
  EXPECT_NEAR(max_arc_vel.Rotation(), 0.5, kEpsilon);
}

TEST(BoxConstraints, AsymmetricCtor) {
  BoxConstraints ddv_limits(WheelVector(-0.25, -0.25), WheelVector(0.5, 0.5),
                            ArcVector(-0.75, -0.25), ArcVector(1.0, 0.5));
  auto min_wheel_vel = ddv_limits.MinWheelVector();
  auto max_wheel_vel = ddv_limits.MaxWheelVector();
  auto min_arc_vel = ddv_limits.MinArcVector();
  auto max_arc_vel = ddv_limits.MaxArcVector();
  EXPECT_FALSE(ddv_limits.IsEmpty());
  EXPECT_NEAR(min_wheel_vel.Left(), -0.25, kEpsilon);
  EXPECT_NEAR(min_wheel_vel.Right(), -0.25, kEpsilon);
  EXPECT_NEAR(max_wheel_vel.Left(), 0.5, kEpsilon);
  EXPECT_NEAR(max_wheel_vel.Right(), 0.5, kEpsilon);
  EXPECT_NEAR(min_arc_vel.Translation(), -0.75, kEpsilon);
  EXPECT_NEAR(min_arc_vel.Rotation(), -0.25, kEpsilon);
  EXPECT_NEAR(max_arc_vel.Translation(), 1.0, kEpsilon);
  EXPECT_NEAR(max_arc_vel.Rotation(), 0.5, kEpsilon);
}

TEST(BoxConstraints, Empty) {
  {
    BoxConstraints ddv_limits(WheelVector(0.6, 0.25), WheelVector(0.5, 0.5),
                              ArcVector(0.75, 0.25), ArcVector(1.0, 0.5));
    EXPECT_TRUE(ddv_limits.IsEmpty());
  }
  {
    BoxConstraints ddv_limits(WheelVector(0.25, 0.6), WheelVector(0.5, 0.5),
                              ArcVector(0.75, 0.25), ArcVector(1.0, 0.5));
    EXPECT_TRUE(ddv_limits.IsEmpty());
  }
  {
    BoxConstraints ddv_limits(WheelVector(0.25, 0.25), WheelVector(0.5, 0.5),
                              ArcVector(1.05, 0.25), ArcVector(1.0, 0.5));
    EXPECT_TRUE(ddv_limits.IsEmpty());
  }
  {
    BoxConstraints ddv_limits(WheelVector(0.25, 0.25), WheelVector(0.5, 0.5),
                              ArcVector(0.75, 0.6), ArcVector(1.0, 0.5));
    EXPECT_TRUE(ddv_limits.IsEmpty());
  }
}

TEST(BoxConstraints, IsInBoundsWheelOnly) {
  BoxConstraints ddv_limits(WheelVector(-0.25, -0.25), WheelVector(0.5, 0.5),
                            ArcVector(-0.75, -0.25), ArcVector(1.0, 0.5));
  EXPECT_FALSE(ddv_limits.IsInBounds(WheelVector(-0.5, 0.0)));
  EXPECT_FALSE(ddv_limits.IsInBounds(WheelVector(0.0, -0.5)));
  EXPECT_FALSE(ddv_limits.IsInBounds(WheelVector(0.75, 0.0)));
  EXPECT_FALSE(ddv_limits.IsInBounds(WheelVector(0.0, 0.75)));
  EXPECT_TRUE(ddv_limits.IsInBounds(WheelVector(-0.25, 0.0)));
  EXPECT_TRUE(ddv_limits.IsInBounds(WheelVector(0.0, -0.25)));
  EXPECT_TRUE(ddv_limits.IsInBounds(WheelVector(0.5, 0.0)));
  EXPECT_TRUE(ddv_limits.IsInBounds(WheelVector(0.0, 0.5)));
  EXPECT_TRUE(ddv_limits.IsInBounds(WheelVector(0.0, 0.0)));
}

TEST(BoxConstraints, IsInBoundsArcOnly) {
  BoxConstraints ddv_limits(WheelVector(-0.25, -0.25), WheelVector(0.5, 0.5),
                            ArcVector(-0.75, -0.25), ArcVector(1.0, 0.5));
  EXPECT_FALSE(ddv_limits.IsInBounds(ArcVector(-0.95, 0.0)));
  EXPECT_FALSE(ddv_limits.IsInBounds(ArcVector(0.0, -0.5)));
  EXPECT_FALSE(ddv_limits.IsInBounds(ArcVector(1.25, 0.0)));
  EXPECT_FALSE(ddv_limits.IsInBounds(ArcVector(0.0, 0.75)));
  EXPECT_TRUE(ddv_limits.IsInBounds(ArcVector(-0.75, 0.0)));
  EXPECT_TRUE(ddv_limits.IsInBounds(ArcVector(0.0, -0.25)));
  EXPECT_TRUE(ddv_limits.IsInBounds(ArcVector(1.0, 0.0)));
  EXPECT_TRUE(ddv_limits.IsInBounds(ArcVector(0.0, 0.5)));
  EXPECT_TRUE(ddv_limits.IsInBounds(ArcVector(0.0, 0.0)));
}

TEST(BoxConstraints, IsInBoundsFromWheel) {
  BoxConstraints ddv_limits(WheelVector(-0.25, -0.25), WheelVector(0.5, 0.5),
                            ArcVector(-0.1, -0.05), ArcVector(0.1, 0.05));
  Kinematics kinematics({2.0, 2.0}, 2.0);
  EXPECT_FALSE(ddv_limits.IsInBounds(kinematics, WheelVector(-0.5, 0.0)));
  EXPECT_FALSE(ddv_limits.IsInBounds(kinematics, WheelVector(0.0, -0.5)));
  EXPECT_FALSE(ddv_limits.IsInBounds(kinematics, WheelVector(0.75, 0.0)));
  EXPECT_FALSE(ddv_limits.IsInBounds(kinematics, WheelVector(0.0, 0.75)));
  EXPECT_FALSE(ddv_limits.IsInBounds(kinematics, WheelVector(-0.25, 0.0)));
  EXPECT_FALSE(ddv_limits.IsInBounds(kinematics, WheelVector(0.0, -0.25)));
  EXPECT_FALSE(ddv_limits.IsInBounds(kinematics, WheelVector(0.5, 0.0)));
  EXPECT_FALSE(ddv_limits.IsInBounds(kinematics, WheelVector(0.0, 0.5)));
  EXPECT_TRUE(ddv_limits.IsInBounds(kinematics, WheelVector(-0.05, 0.0)));
  EXPECT_TRUE(ddv_limits.IsInBounds(kinematics, WheelVector(0.0, -0.025)));
  EXPECT_TRUE(ddv_limits.IsInBounds(kinematics, WheelVector(0.05, 0.0)));
  EXPECT_TRUE(ddv_limits.IsInBounds(kinematics, WheelVector(0.0, 0.025)));
  EXPECT_TRUE(ddv_limits.IsInBounds(kinematics, WheelVector(0.0, 0.0)));
}

TEST(BoxConstraints, IsInBoundsFromArc) {
  BoxConstraints ddv_limits(WheelVector(-0.25, -0.25), WheelVector(0.25, 0.25),
                            ArcVector(-0.75, -0.75), ArcVector(1.0, 1.0));
  Kinematics kinematics({2.0, 2.0}, 2.0);
  EXPECT_FALSE(ddv_limits.IsInBounds(kinematics, ArcVector(-0.95, 0.0)));
  EXPECT_FALSE(ddv_limits.IsInBounds(kinematics, ArcVector(0.0, -0.95)));
  EXPECT_FALSE(ddv_limits.IsInBounds(kinematics, ArcVector(1.25, 0.0)));
  EXPECT_FALSE(ddv_limits.IsInBounds(kinematics, ArcVector(0.0, 0.75)));
  EXPECT_FALSE(ddv_limits.IsInBounds(kinematics, ArcVector(-0.75, 0.0)));
  EXPECT_FALSE(ddv_limits.IsInBounds(kinematics, ArcVector(0.0, -0.75)));
  EXPECT_FALSE(ddv_limits.IsInBounds(kinematics, ArcVector(0.75, 0.0)));
  EXPECT_FALSE(ddv_limits.IsInBounds(kinematics, ArcVector(0.0, 0.75)));
  EXPECT_TRUE(ddv_limits.IsInBounds(kinematics, ArcVector(-0.5, 0.0)));
  EXPECT_TRUE(ddv_limits.IsInBounds(kinematics, ArcVector(0.0, -0.5)));
  EXPECT_TRUE(ddv_limits.IsInBounds(kinematics, ArcVector(0.5, 0.0)));
  EXPECT_TRUE(ddv_limits.IsInBounds(kinematics, ArcVector(0.0, 0.5)));
  EXPECT_TRUE(ddv_limits.IsInBounds(kinematics, ArcVector(0.0, 0.0)));
}

TEST(BoxConstraints, ContainsOrigin) {
  BoxConstraints ddv_limits_centered(
      WheelVector(-0.25, -0.25), WheelVector(0.5, 0.5), ArcVector(-0.75, -0.25),
      ArcVector(1.0, 0.5));
  EXPECT_TRUE(ddv_limits_centered.ContainsOrigin());
  BoxConstraints ddv_limits_rot_shifted_min(
      WheelVector(-0.25, -0.25), WheelVector(0.5, 0.5), ArcVector(-0.75, -0.5),
      ArcVector(1.0, -0.25));
  EXPECT_FALSE(ddv_limits_rot_shifted_min.ContainsOrigin());
  BoxConstraints ddv_limits_rot_shifted_max(
      WheelVector(-0.25, -0.25), WheelVector(0.5, 0.5), ArcVector(-0.75, 0.25),
      ArcVector(1.0, 0.5));
  EXPECT_FALSE(ddv_limits_rot_shifted_max.ContainsOrigin());
  BoxConstraints ddv_limits_trans_shifted_min(
      WheelVector(-0.25, -0.25), WheelVector(0.5, 0.5), ArcVector(-0.75, -0.5),
      ArcVector(-0.25, 0.5));
  EXPECT_FALSE(ddv_limits_trans_shifted_min.ContainsOrigin());
  BoxConstraints ddv_limits_trans_shifted_max(
      WheelVector(-0.25, -0.25), WheelVector(0.5, 0.5), ArcVector(0.75, -0.5),
      ArcVector(1.0, 0.5));
  EXPECT_FALSE(ddv_limits_trans_shifted_max.ContainsOrigin());
  BoxConstraints ddv_limits_left_shifted_min(
      WheelVector(-0.25, -0.25), WheelVector(-0.1, 0.5),
      ArcVector(-0.75, -0.25), ArcVector(1.0, 0.5));
  EXPECT_FALSE(ddv_limits_left_shifted_min.ContainsOrigin());
  BoxConstraints ddv_limits_left_shifted_max(
      WheelVector(0.25, -0.25), WheelVector(0.5, 0.5), ArcVector(-0.75, -0.25),
      ArcVector(1.0, 0.5));
  EXPECT_FALSE(ddv_limits_left_shifted_max.ContainsOrigin());
  BoxConstraints ddv_limits_right_shifted_min(
      WheelVector(-0.25, -0.25), WheelVector(0.5, -0.1),
      ArcVector(-0.75, -0.25), ArcVector(1.0, 0.5));
  EXPECT_FALSE(ddv_limits_right_shifted_min.ContainsOrigin());
  BoxConstraints ddv_limits_right_shifted_max(
      WheelVector(-0.25, 0.25), WheelVector(0.5, 0.5), ArcVector(-0.75, -0.25),
      ArcVector(1.0, 0.5));
  EXPECT_FALSE(ddv_limits_right_shifted_max.ContainsOrigin());
}

TEST(BoxConstraints, BringInBoundsWheelOnly) {
  BoxConstraints ddv_limits(WheelVector(-0.25, -0.25), WheelVector(0.5, 0.5),
                            ArcVector(-0.75, -0.25), ArcVector(1.0, 0.5));
  WheelVector w1;
  bool r1 = ddv_limits.BringInBounds(WheelVector(-0.5, 0.0), &w1);
  EXPECT_TRUE(r1);
  EXPECT_NEAR(w1.Left(), -0.25, kEpsilon);
  EXPECT_NEAR(w1.Right(), 0.0, kEpsilon);
  WheelVector w2;
  bool r2 = ddv_limits.BringInBounds(WheelVector(0.0, -0.5), &w2);
  EXPECT_TRUE(r2);
  EXPECT_NEAR(w2.Left(), 0.0, kEpsilon);
  EXPECT_NEAR(w2.Right(), -0.25, kEpsilon);
  WheelVector w3;
  bool r3 = ddv_limits.BringInBounds(WheelVector(-0.5, -0.5), &w3);
  EXPECT_TRUE(r3);
  EXPECT_NEAR(w3.Left(), -0.25, kEpsilon);
  EXPECT_NEAR(w3.Right(), -0.25, kEpsilon);
  WheelVector w4;
  bool r4 = ddv_limits.BringInBounds(WheelVector(0.75, -0.5), &w4);
  EXPECT_TRUE(r4);
  EXPECT_NEAR(w4.Left(), 0.375, kEpsilon);
  EXPECT_NEAR(w4.Right(), -0.25, kEpsilon);
  WheelVector w5;
  bool r5 = ddv_limits.BringInBounds(WheelVector(0.25, -0.5), &w5);
  EXPECT_TRUE(r5);
  EXPECT_NEAR(w5.Left(), 0.125, kEpsilon);
  EXPECT_NEAR(w5.Right(), -0.25, kEpsilon);
  WheelVector w6;
  bool r6 = ddv_limits.BringInBounds(WheelVector(0.75, 0.0), &w6);
  EXPECT_TRUE(r6);
  EXPECT_NEAR(w6.Left(), 0.5, kEpsilon);
  EXPECT_NEAR(w6.Right(), 0.0, kEpsilon);
  WheelVector w7;
  bool r7 = ddv_limits.BringInBounds(WheelVector(0.0, 0.75), &w7);
  EXPECT_TRUE(r7);
  EXPECT_NEAR(w7.Left(), 0.0, kEpsilon);
  EXPECT_NEAR(w7.Right(), 0.5, kEpsilon);
  WheelVector w8;
  bool r8 = ddv_limits.BringInBounds(WheelVector(0.75, 0.75), &w8);
  EXPECT_TRUE(r8);
  EXPECT_NEAR(w8.Left(), 0.5, kEpsilon);
  EXPECT_NEAR(w8.Right(), 0.5, kEpsilon);
  WheelVector w9;
  bool r9 = ddv_limits.BringInBounds(WheelVector(0.75, 0.25), &w9);
  EXPECT_TRUE(r9);
  EXPECT_NEAR(w9.Left(), 0.5, kEpsilon);
  EXPECT_NEAR(w9.Right(), 0.166666666667, kEpsilon);
}

TEST(BoxConstraints, BringInBoundsArcOnly) {
  BoxConstraints ddv_limits(WheelVector(-0.25, -0.25), WheelVector(0.5, 0.5),
                            ArcVector(-0.75, -0.25), ArcVector(1.0, 0.5));
  ArcVector a1;
  bool r1 = ddv_limits.BringInBounds(ArcVector(-1.0, 0.0), &a1);
  EXPECT_TRUE(r1);
  EXPECT_NEAR(a1.Translation(), -0.75, kEpsilon);
  EXPECT_NEAR(a1.Rotation(), 0.0, kEpsilon);
  ArcVector a2;
  bool r2 = ddv_limits.BringInBounds(ArcVector(0.0, -0.5), &a2);
  EXPECT_TRUE(r2);
  EXPECT_NEAR(a2.Translation(), 0.0, kEpsilon);
  EXPECT_NEAR(a2.Rotation(), -0.25, kEpsilon);
  ArcVector a3;
  bool r3 = ddv_limits.BringInBounds(ArcVector(-0.5, -0.5), &a3);
  EXPECT_TRUE(r3);
  EXPECT_NEAR(a3.Translation(), -0.25, kEpsilon);
  EXPECT_NEAR(a3.Rotation(), -0.25, kEpsilon);
  ArcVector a4;
  bool r4 = ddv_limits.BringInBounds(ArcVector(0.75, -0.5), &a4);
  EXPECT_TRUE(r4);
  EXPECT_NEAR(a4.Translation(), 0.375, kEpsilon);
  EXPECT_NEAR(a4.Rotation(), -0.25, kEpsilon);
  ArcVector a5;
  bool r5 = ddv_limits.BringInBounds(ArcVector(0.25, -0.5), &a5);
  EXPECT_TRUE(r5);
  EXPECT_NEAR(a5.Translation(), 0.125, kEpsilon);
  EXPECT_NEAR(a5.Rotation(), -0.25, kEpsilon);
  ArcVector a6;
  bool r6 = ddv_limits.BringInBounds(ArcVector(1.25, 0.0), &a6);
  EXPECT_TRUE(r6);
  EXPECT_NEAR(a6.Translation(), 1.0, kEpsilon);
  EXPECT_NEAR(a6.Rotation(), 0.0, kEpsilon);
  ArcVector a7;
  bool r7 = ddv_limits.BringInBounds(ArcVector(0.0, 0.75), &a7);
  EXPECT_TRUE(r7);
  EXPECT_NEAR(a7.Translation(), 0.0, kEpsilon);
  EXPECT_NEAR(a7.Rotation(), 0.5, kEpsilon);
  ArcVector a8;
  bool r8 = ddv_limits.BringInBounds(ArcVector(0.75, 0.75), &a8);
  EXPECT_TRUE(r8);
  EXPECT_NEAR(a8.Translation(), 0.5, kEpsilon);
  EXPECT_NEAR(a8.Rotation(), 0.5, kEpsilon);
  ArcVector a9;
  bool r9 = ddv_limits.BringInBounds(ArcVector(1.25, 0.25), &a9);
  EXPECT_TRUE(r9);
  EXPECT_NEAR(a9.Translation(), 1.0, kEpsilon);
  EXPECT_NEAR(a9.Rotation(), 0.2, kEpsilon);
}

TEST(BoxConstraints, BringInBoundsFromWheel) {
  BoxConstraints ddv_limits(WheelVector(-0.25, -0.25), WheelVector(0.5, 0.5),
                            ArcVector(-0.1, -0.05), ArcVector(0.1, 0.05));
  Kinematics kinematics({2.0, 2.0}, 2.0);
  WheelVector w1;
  bool r1 = ddv_limits.BringInBounds(kinematics, WheelVector(-0.5, 0.0), &w1);
  EXPECT_TRUE(r1);
  EXPECT_NEAR(w1.Left(), -0.05, kEpsilon);
  EXPECT_NEAR(w1.Right(), 0.0, kEpsilon);
  WheelVector w2;
  bool r2 = ddv_limits.BringInBounds(kinematics, WheelVector(0.0, -0.5), &w2);
  EXPECT_TRUE(r2);
  EXPECT_NEAR(w2.Left(), 0.0, kEpsilon);
  EXPECT_NEAR(w2.Right(), -0.05, kEpsilon);
  WheelVector w3;
  bool r3 = ddv_limits.BringInBounds(kinematics, WheelVector(-0.5, -0.5), &w3);
  EXPECT_TRUE(r3);
  EXPECT_NEAR(w3.Left(), -0.05, kEpsilon);
  EXPECT_NEAR(w3.Right(), -0.05, kEpsilon);
  WheelVector w4;
  bool r4 = ddv_limits.BringInBounds(kinematics, WheelVector(0.75, -0.5), &w4);
  EXPECT_TRUE(r4);
  EXPECT_NEAR(w4.Left(), 0.03, kEpsilon);
  EXPECT_NEAR(w4.Right(), -0.02, kEpsilon);
  WheelVector w5;
  bool r5 = ddv_limits.BringInBounds(kinematics, WheelVector(0.25, -0.5), &w5);
  EXPECT_TRUE(r5);
  EXPECT_NEAR(w5.Left(), 0.016666666667, kEpsilon);
  EXPECT_NEAR(w5.Right(), -0.033333333333, kEpsilon);
  WheelVector w6;
  bool r6 = ddv_limits.BringInBounds(kinematics, WheelVector(0.75, 0.0), &w6);
  EXPECT_TRUE(r6);
  EXPECT_NEAR(w6.Left(), 0.05, kEpsilon);
  EXPECT_NEAR(w6.Right(), 0.0, kEpsilon);
  WheelVector w7;
  bool r7 = ddv_limits.BringInBounds(kinematics, WheelVector(0.0, 0.75), &w7);
  EXPECT_TRUE(r7);
  EXPECT_NEAR(w7.Left(), 0.0, kEpsilon);
  EXPECT_NEAR(w7.Right(), 0.05, kEpsilon);
  WheelVector w8;
  bool r8 = ddv_limits.BringInBounds(kinematics, WheelVector(0.75, 0.75), &w8);
  EXPECT_TRUE(r8);
  EXPECT_NEAR(w8.Left(), 0.05, kEpsilon);
  EXPECT_NEAR(w8.Right(), 0.05, kEpsilon);
  WheelVector w9;
  bool r9 = ddv_limits.BringInBounds(kinematics, WheelVector(0.75, 0.25), &w9);
  EXPECT_TRUE(r9);
  EXPECT_NEAR(w9.Left(), 0.075, kEpsilon);
  EXPECT_NEAR(w9.Right(), 0.025, kEpsilon);
}

TEST(BoxConstraints, BringInBoundsFromArc) {
  BoxConstraints ddv_limits(WheelVector(-0.25, -0.25), WheelVector(0.25, 0.25),
                            ArcVector(-0.75, -0.75), ArcVector(1.0, 1.0));
  Kinematics kinematics({2.0, 2.0}, 2.0);
  ArcVector a1;
  bool r1 = ddv_limits.BringInBounds(kinematics, ArcVector(-1.0, 0.0), &a1);
  EXPECT_TRUE(r1);
  EXPECT_NEAR(a1.Translation(), -0.5, kEpsilon);
  EXPECT_NEAR(a1.Rotation(), 0.0, kEpsilon);
  ArcVector a2;
  bool r2 = ddv_limits.BringInBounds(kinematics, ArcVector(0.0, -1.0), &a2);
  EXPECT_TRUE(r2);
  EXPECT_NEAR(a2.Translation(), 0.0, kEpsilon);
  EXPECT_NEAR(a2.Rotation(), -0.5, kEpsilon);
  ArcVector a3;
  bool r3 = ddv_limits.BringInBounds(kinematics, ArcVector(-1.0, -1.0), &a3);
  EXPECT_TRUE(r3);
  EXPECT_NEAR(a3.Translation(), -0.25, kEpsilon);
  EXPECT_NEAR(a3.Rotation(), -0.25, kEpsilon);
  ArcVector a4;
  bool r4 = ddv_limits.BringInBounds(kinematics, ArcVector(1.5, -1.0), &a4);
  EXPECT_TRUE(r4);
  EXPECT_NEAR(a4.Translation(), 0.3, kEpsilon);
  EXPECT_NEAR(a4.Rotation(), -0.2, kEpsilon);
  ArcVector a5;
  bool r5 = ddv_limits.BringInBounds(kinematics, ArcVector(0.5, -1.0), &a5);
  EXPECT_TRUE(r5);
  EXPECT_NEAR(a5.Translation(), 0.1666666666667, kEpsilon);
  EXPECT_NEAR(a5.Rotation(), -0.3333333333333, kEpsilon);
  ArcVector a6;
  bool r6 = ddv_limits.BringInBounds(kinematics, ArcVector(1.25, 0.0), &a6);
  EXPECT_TRUE(r6);
  EXPECT_NEAR(a6.Translation(), 0.5, kEpsilon);
  EXPECT_NEAR(a6.Rotation(), 0.0, kEpsilon);
  ArcVector a7;
  bool r7 = ddv_limits.BringInBounds(kinematics, ArcVector(0.0, 1.25), &a7);
  EXPECT_TRUE(r7);
  EXPECT_NEAR(a7.Translation(), 0.0, kEpsilon);
  EXPECT_NEAR(a7.Rotation(), 0.5, kEpsilon);
  ArcVector a8;
  bool r8 = ddv_limits.BringInBounds(kinematics, ArcVector(1.25, 1.25), &a8);
  EXPECT_TRUE(r8);
  EXPECT_NEAR(a8.Translation(), 0.25, kEpsilon);
  EXPECT_NEAR(a8.Rotation(), 0.25, kEpsilon);
  ArcVector a9;
  bool r9 = ddv_limits.BringInBounds(kinematics, ArcVector(1.25, 0.25), &a9);
  EXPECT_TRUE(r9);
  EXPECT_NEAR(a9.Translation(), 0.416666666666667, kEpsilon);
  EXPECT_NEAR(a9.Rotation(), 0.08333333333333, kEpsilon);
}

TEST(BoxConstraints, BringInBoundsPreserveDirectionWheelOnly) {
  BoxConstraints ddv_limits(WheelVector(0.25, 0.25), WheelVector(0.5, 0.5),
                            ArcVector(0.25, 0.25), ArcVector(1.0, 0.5));
  WheelVector w1;
  bool r1 = ddv_limits.BringInBounds(WheelVector(-0.5, 0.0), &w1,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r1);
  WheelVector w2;
  bool r2 = ddv_limits.BringInBounds(WheelVector(0.0, -0.5), &w2,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r2);
  WheelVector w3;
  bool r3 = ddv_limits.BringInBounds(WheelVector(-0.5, -0.5), &w3,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_TRUE(r3);
  EXPECT_NEAR(w3.Left(), 0.25, kEpsilon);
  EXPECT_NEAR(w3.Right(), 0.25, kEpsilon);
  WheelVector w4;
  bool r4 = ddv_limits.BringInBounds(WheelVector(0.75, -0.5), &w4,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r4);
  WheelVector w5;
  bool r5 = ddv_limits.BringInBounds(WheelVector(0.25, -0.5), &w5,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r5);
  WheelVector w6;
  bool r6 = ddv_limits.BringInBounds(WheelVector(0.75, 0.0), &w6,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r6);
  WheelVector w7;
  bool r7 = ddv_limits.BringInBounds(WheelVector(0.0, 0.75), &w7,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r7);
  WheelVector w8;
  bool r8 = ddv_limits.BringInBounds(WheelVector(0.75, 0.75), &w8,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_TRUE(r8);
  EXPECT_NEAR(w8.Left(), 0.5, kEpsilon);
  EXPECT_NEAR(w8.Right(), 0.5, kEpsilon);
  WheelVector w9;
  bool r9 = ddv_limits.BringInBounds(WheelVector(0.75, 0.25), &w9,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r9);
}

TEST(BoxConstraints, BringInBoundsPreserveDirectionArcOnly) {
  BoxConstraints ddv_limits(WheelVector(0.25, 0.25), WheelVector(0.5, 0.5),
                            ArcVector(0.25, 0.25), ArcVector(1.0, 0.5));
  ArcVector a1;
  bool r1 = ddv_limits.BringInBounds(ArcVector(-1.0, 0.0), &a1,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r1);
  ArcVector a2;
  bool r2 = ddv_limits.BringInBounds(ArcVector(0.0, -0.5), &a2,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r2);
  ArcVector a3;
  bool r3 = ddv_limits.BringInBounds(ArcVector(-0.5, -0.5), &a3,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_TRUE(r3);
  EXPECT_NEAR(a3.Translation(), 0.25, kEpsilon);
  EXPECT_NEAR(a3.Rotation(), 0.25, kEpsilon);
  ArcVector a4;
  bool r4 = ddv_limits.BringInBounds(ArcVector(0.75, -0.5), &a4,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r4);
  ArcVector a5;
  bool r5 = ddv_limits.BringInBounds(ArcVector(0.25, -0.5), &a5,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r5);
  ArcVector a6;
  bool r6 = ddv_limits.BringInBounds(ArcVector(1.25, 0.0), &a6,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r6);
  ArcVector a7;
  bool r7 = ddv_limits.BringInBounds(ArcVector(0.0, 0.75), &a7,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r7);
  ArcVector a8;
  bool r8 = ddv_limits.BringInBounds(ArcVector(0.75, 0.75), &a8,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_TRUE(r8);
  EXPECT_NEAR(a8.Translation(), 0.5, kEpsilon);
  EXPECT_NEAR(a8.Rotation(), 0.5, kEpsilon);
  ArcVector a9;
  bool r9 = ddv_limits.BringInBounds(ArcVector(1.25, 0.25), &a9,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r9);
}

TEST(BoxConstraints, BringInBoundsPreserveDirectionFromWheel) {
  BoxConstraints ddv_limits(WheelVector(-0.0625, 0.3125),
                            WheelVector(0.3125, 0.6875), ArcVector(0.25, 0.25),
                            ArcVector(1.0, 0.5));
  Kinematics kinematics({2.0, 2.0}, 2.0);
  WheelVector w1;
  bool r1 = ddv_limits.BringInBounds(kinematics, WheelVector(-0.5, 0.0), &w1,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r1);
  WheelVector w2;
  bool r2 = ddv_limits.BringInBounds(kinematics, WheelVector(0.0, -0.5), &w2,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_TRUE(r2);
  EXPECT_NEAR(w2.Left(), 0.0, kEpsilon);
  EXPECT_NEAR(w2.Right(), 0.3125, kEpsilon);
  WheelVector w3;
  bool r3 = ddv_limits.BringInBounds(kinematics, WheelVector(-0.5, -0.5), &w3,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r3);
  WheelVector w4;
  bool r4 = ddv_limits.BringInBounds(kinematics, WheelVector(0.75, -0.5), &w4,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r4);
  WheelVector w5;
  bool r5 = ddv_limits.BringInBounds(kinematics, WheelVector(0.25, -0.5), &w5,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r5);
  WheelVector w6;
  bool r6 = ddv_limits.BringInBounds(kinematics, WheelVector(0.75, 0.0), &w6,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r6);
  WheelVector w7;
  bool r7 = ddv_limits.BringInBounds(kinematics, WheelVector(0.0, 0.75), &w7,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_TRUE(r7);
  EXPECT_NEAR(w7.Left(), 0.0, kEpsilon);
  EXPECT_NEAR(w7.Right(), 0.5, kEpsilon);
  WheelVector w8;
  bool r8 = ddv_limits.BringInBounds(kinematics, WheelVector(0.25, 0.75), &w8,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_TRUE(r8);
  EXPECT_NEAR(w8.Left(), 0.229166667, kEpsilon);
  EXPECT_NEAR(w8.Right(), 0.6875, kEpsilon);
  WheelVector w9;
  bool r9 = ddv_limits.BringInBounds(kinematics, WheelVector(0.75, 0.25), &w9,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r9);
}

TEST(BoxConstraints, BringInBoundsPreserveDirectionFromArc) {
  BoxConstraints ddv_limits(WheelVector(-0.0625, 0.3125),
                            WheelVector(0.3125, 0.6875), ArcVector(0.25, 0.25),
                            ArcVector(1.0, 0.5));
  Kinematics kinematics({2.0, 2.0}, 2.0);
  ArcVector a1;
  bool r1 = ddv_limits.BringInBounds(kinematics, ArcVector(-1.0, 0.0), &a1,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r1);
  ArcVector a2;
  bool r2 = ddv_limits.BringInBounds(kinematics, ArcVector(0.0, -1.0), &a2,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r2);
  ArcVector a3;
  bool r3 = ddv_limits.BringInBounds(kinematics, ArcVector(-1.0, -1.0), &a3,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_TRUE(r3);
  EXPECT_NEAR(a3.Translation(), 0.3125, kEpsilon);
  EXPECT_NEAR(a3.Rotation(), 0.3125, kEpsilon);
  ArcVector a4;
  bool r4 = ddv_limits.BringInBounds(kinematics, ArcVector(1.5, -1.0), &a4,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r4);
  ArcVector a5;
  bool r5 = ddv_limits.BringInBounds(kinematics, ArcVector(0.5, -1.0), &a5,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r5);
  ArcVector a6;
  bool r6 = ddv_limits.BringInBounds(kinematics, ArcVector(1.25, 0.0), &a6,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r6);
  ArcVector a7;
  bool r7 = ddv_limits.BringInBounds(kinematics, ArcVector(0.0, 1.25), &a7,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r7);
  ArcVector a8;
  bool r8 = ddv_limits.BringInBounds(kinematics, ArcVector(1.25, 1.25), &a8,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_TRUE(r8);
  EXPECT_NEAR(a8.Translation(), 0.5, kEpsilon);
  EXPECT_NEAR(a8.Rotation(), 0.5, kEpsilon);
  ArcVector a9;
  bool r9 = ddv_limits.BringInBounds(kinematics, ArcVector(1.0, 0.25), &a9,
                                     BoxConstraints::kPreserveDirectionOrFail);
  EXPECT_FALSE(r9);
}

TEST(BoxConstraints, TranslateLimits) {
  BoxConstraints ddv_limits(WheelVector(-0.25, -0.25), WheelVector(0.5, 0.5),
                            ArcVector(-0.75, -0.25), ArcVector(1.0, 0.5));
  ddv_limits.TranslateLimits(WheelVector(0.2, 0.1), ArcVector(0.3, -0.1));
  auto min_wheel_vel = ddv_limits.MinWheelVector();
  auto max_wheel_vel = ddv_limits.MaxWheelVector();
  auto min_arc_vel = ddv_limits.MinArcVector();
  auto max_arc_vel = ddv_limits.MaxArcVector();
  EXPECT_NEAR(min_wheel_vel.Left(), -0.05, kEpsilon);
  EXPECT_NEAR(min_wheel_vel.Right(), -0.15, kEpsilon);
  EXPECT_NEAR(max_wheel_vel.Left(), 0.7, kEpsilon);
  EXPECT_NEAR(max_wheel_vel.Right(), 0.6, kEpsilon);
  EXPECT_NEAR(min_arc_vel.Translation(), -0.45, kEpsilon);
  EXPECT_NEAR(min_arc_vel.Rotation(), -0.35, kEpsilon);
  EXPECT_NEAR(max_arc_vel.Translation(), 1.3, kEpsilon);
  EXPECT_NEAR(max_arc_vel.Rotation(), 0.4, kEpsilon);
}

TEST(BoxConstraints, TranslateLimitsFromWheel) {
  BoxConstraints ddv_limits(WheelVector(-0.25, -0.25), WheelVector(0.5, 0.5),
                            ArcVector(-0.75, -0.25), ArcVector(1.0, 0.5));
  Kinematics kinematics({2.0, 2.0}, 2.0);
  ddv_limits.TranslateLimits(kinematics, WheelVector(0.2, 0.1));
  auto min_wheel_vel = ddv_limits.MinWheelVector();
  auto max_wheel_vel = ddv_limits.MaxWheelVector();
  auto min_arc_vel = ddv_limits.MinArcVector();
  auto max_arc_vel = ddv_limits.MaxArcVector();
  EXPECT_NEAR(min_wheel_vel.Left(), -0.05, kEpsilon);
  EXPECT_NEAR(min_wheel_vel.Right(), -0.15, kEpsilon);
  EXPECT_NEAR(max_wheel_vel.Left(), 0.7, kEpsilon);
  EXPECT_NEAR(max_wheel_vel.Right(), 0.6, kEpsilon);
  EXPECT_NEAR(min_arc_vel.Translation(), -0.45, kEpsilon);
  EXPECT_NEAR(min_arc_vel.Rotation(), -0.35, kEpsilon);
  EXPECT_NEAR(max_arc_vel.Translation(), 1.3, kEpsilon);
  EXPECT_NEAR(max_arc_vel.Rotation(), 0.4, kEpsilon);
}

TEST(BoxConstraints, TranslateLimitsFromArc) {
  BoxConstraints ddv_limits(WheelVector(-0.25, -0.25), WheelVector(0.5, 0.5),
                            ArcVector(-0.75, -0.25), ArcVector(1.0, 0.5));
  Kinematics kinematics({2.0, 2.0}, 2.0);
  ddv_limits.TranslateLimits(kinematics, ArcVector(0.3, -0.1));
  auto min_wheel_vel = ddv_limits.MinWheelVector();
  auto max_wheel_vel = ddv_limits.MaxWheelVector();
  auto min_arc_vel = ddv_limits.MinArcVector();
  auto max_arc_vel = ddv_limits.MaxArcVector();
  EXPECT_NEAR(min_wheel_vel.Left(), -0.05, kEpsilon);
  EXPECT_NEAR(min_wheel_vel.Right(), -0.15, kEpsilon);
  EXPECT_NEAR(max_wheel_vel.Left(), 0.7, kEpsilon);
  EXPECT_NEAR(max_wheel_vel.Right(), 0.6, kEpsilon);
  EXPECT_NEAR(min_arc_vel.Translation(), -0.45, kEpsilon);
  EXPECT_NEAR(min_arc_vel.Rotation(), -0.35, kEpsilon);
  EXPECT_NEAR(max_arc_vel.Translation(), 1.3, kEpsilon);
  EXPECT_NEAR(max_arc_vel.Rotation(), 0.4, kEpsilon);
}

TEST(BoxConstraints, ScaleLimits) {
  BoxConstraints ddv_limits(WheelVector(-0.25, -0.25), WheelVector(0.5, 0.5),
                            ArcVector(-0.75, -0.25), ArcVector(1.0, 0.5));
  ddv_limits.ScaleLimits(0.5);
  auto min_wheel_vel = ddv_limits.MinWheelVector();
  auto max_wheel_vel = ddv_limits.MaxWheelVector();
  auto min_arc_vel = ddv_limits.MinArcVector();
  auto max_arc_vel = ddv_limits.MaxArcVector();
  EXPECT_NEAR(min_wheel_vel.Left(), -0.125, kEpsilon);
  EXPECT_NEAR(min_wheel_vel.Right(), -0.125, kEpsilon);
  EXPECT_NEAR(max_wheel_vel.Left(), 0.25, kEpsilon);
  EXPECT_NEAR(max_wheel_vel.Right(), 0.25, kEpsilon);
  EXPECT_NEAR(min_arc_vel.Translation(), -0.375, kEpsilon);
  EXPECT_NEAR(min_arc_vel.Rotation(), -0.125, kEpsilon);
  EXPECT_NEAR(max_arc_vel.Translation(), 0.5, kEpsilon);
  EXPECT_NEAR(max_arc_vel.Rotation(), 0.25, kEpsilon);
}

TEST(BoxConstraints, Intersect) {
  BoxConstraints ddv_limits(WheelVector(-0.25, -0.25), WheelVector(0.5, 0.5),
                            ArcVector(-0.75, -0.25), ArcVector(1.0, 0.5));

  BoxConstraints ddv_limits_inside(
      WheelVector(-0.05, -0.05), WheelVector(0.05, 0.05),
      ArcVector(-0.05, -0.05), ArcVector(0.05, 0.05));
  ExpectEqualBoxConstraints(
      BoxConstraints::Intersect(ddv_limits, ddv_limits_inside),
      ddv_limits_inside);

  BoxConstraints ddv_limits_overlap(
      WheelVector(-0.5, -0.5), WheelVector(0.25, 0.25), ArcVector(-1.0, -0.5),
      ArcVector(0.75, 0.25));
  BoxConstraints ddv_limits_overlap_intersect(
      WheelVector(-0.25, -0.25), WheelVector(0.25, 0.25),
      ArcVector(-0.75, -0.25), ArcVector(0.75, 0.25));
  ExpectEqualBoxConstraints(
      BoxConstraints::Intersect(ddv_limits, ddv_limits_overlap),
      ddv_limits_overlap_intersect);
}

TEST(BoxConstraints, GrowToIntersect) {
  BoxConstraints ddv_limits(WheelVector(-0.5, -0.5), WheelVector(0.5, 0.5),
                            ArcVector(-1.0, -0.5), ArcVector(1.0, 0.5));

  BoxConstraints ddv_limits_in(WheelVector(-0.25, -0.25),
                               WheelVector(0.25, 0.25), ArcVector(-0.5, -0.25),
                               ArcVector(0.5, 0.25));
  ExpectEqualBoxConstraints(
      BoxConstraints::GrowToIntersect(ddv_limits, ddv_limits_in),
      ddv_limits_in);

  BoxConstraints ddv_limits_out1(WheelVector(0.75, -0.25),
                                 WheelVector(1.25, 0.25),
                                 ArcVector(-0.5, -0.25), ArcVector(0.5, 0.25));
  BoxConstraints ddv_limits_out1_expected(
      WheelVector(0.5, -0.25), WheelVector(0.5, 0.25), ArcVector(-0.5, -0.25),
      ArcVector(0.5, 0.25));
  ExpectEqualBoxConstraints(
      BoxConstraints::GrowToIntersect(ddv_limits, ddv_limits_out1),
      ddv_limits_out1_expected);

  BoxConstraints ddv_limits_out2(WheelVector(0.75, 0.75),
                                 WheelVector(1.25, 1.25),
                                 ArcVector(-0.5, -0.25), ArcVector(0.5, 0.25));
  BoxConstraints ddv_limits_out2_expected(
      WheelVector(0.5, 0.5), WheelVector(0.5, 0.5), ArcVector(-0.5, -0.25),
      ArcVector(0.5, 0.25));
  ExpectEqualBoxConstraints(
      BoxConstraints::GrowToIntersect(ddv_limits, ddv_limits_out2),
      ddv_limits_out2_expected);

  BoxConstraints ddv_limits_out3(WheelVector(-0.25, -0.25),
                                 WheelVector(0.25, 0.25), ArcVector(1.5, -0.25),
                                 ArcVector(2.0, 0.25));
  BoxConstraints ddv_limits_out3_expected(
      WheelVector(-0.25, -0.25), WheelVector(0.25, 0.25), ArcVector(1.0, -0.25),
      ArcVector(1.0, 0.25));
  ExpectEqualBoxConstraints(
      BoxConstraints::GrowToIntersect(ddv_limits, ddv_limits_out3),
      ddv_limits_out3_expected);

  BoxConstraints ddv_limits_out4(WheelVector(-0.25, -0.25),
                                 WheelVector(0.25, 0.25), ArcVector(1.5, 0.75),
                                 ArcVector(2.0, 1.25));
  BoxConstraints ddv_limits_out4_expected(
      WheelVector(-0.25, -0.25), WheelVector(0.25, 0.25), ArcVector(1.0, 0.5),
      ArcVector(1.0, 0.5));
  ExpectEqualBoxConstraints(
      BoxConstraints::GrowToIntersect(ddv_limits, ddv_limits_out4),
      ddv_limits_out4_expected);

  BoxConstraints ddv_limits_out5(WheelVector(-1.25, -0.25),
                                 WheelVector(-0.75, 0.25),
                                 ArcVector(-0.5, -0.25), ArcVector(0.5, 0.25));
  BoxConstraints ddv_limits_out5_expected(
      WheelVector(-0.5, -0.25), WheelVector(-0.5, 0.25), ArcVector(-0.5, -0.25),
      ArcVector(0.5, 0.25));
  ExpectEqualBoxConstraints(
      BoxConstraints::GrowToIntersect(ddv_limits, ddv_limits_out5),
      ddv_limits_out5_expected);

  BoxConstraints ddv_limits_out6(WheelVector(-1.25, -1.25),
                                 WheelVector(-0.75, -0.75),
                                 ArcVector(-0.5, -0.25), ArcVector(0.5, 0.25));
  BoxConstraints ddv_limits_out6_expected(
      WheelVector(-0.5, -0.5), WheelVector(-0.5, -0.5), ArcVector(-0.5, -0.25),
      ArcVector(0.5, 0.25));
  ExpectEqualBoxConstraints(
      BoxConstraints::GrowToIntersect(ddv_limits, ddv_limits_out6),
      ddv_limits_out6_expected);

  BoxConstraints ddv_limits_out7(WheelVector(-0.25, -0.25),
                                 WheelVector(0.25, 0.25),
                                 ArcVector(-2.0, -0.25), ArcVector(-1.5, 0.25));
  BoxConstraints ddv_limits_out7_expected(
      WheelVector(-0.25, -0.25), WheelVector(0.25, 0.25),
      ArcVector(-1.0, -0.25), ArcVector(-1.0, 0.25));
  ExpectEqualBoxConstraints(
      BoxConstraints::GrowToIntersect(ddv_limits, ddv_limits_out7),
      ddv_limits_out7_expected);

  BoxConstraints ddv_limits_out8(
      WheelVector(-0.25, -0.25), WheelVector(0.25, 0.25),
      ArcVector(-2.0, -1.25), ArcVector(-1.5, -0.75));
  BoxConstraints ddv_limits_out8_expected(
      WheelVector(-0.25, -0.25), WheelVector(0.25, 0.25), ArcVector(-1.0, -0.5),
      ArcVector(-1.0, -0.5));
  ExpectEqualBoxConstraints(
      BoxConstraints::GrowToIntersect(ddv_limits, ddv_limits_out8),
      ddv_limits_out8_expected);
}

TEST(BoxConstraints, ScaleDown) {
  // Choose dynamic constraints such that the wheel space constraints
  // are not wholly inside the arc space constraints, nor
  // vice-versa. Sanity check that the constraints actually have that
  // property. This is based on specially chosen kinematic model
  // parameters which make it more straightforward to construct such a
  // case.

  Kinematics kinematics({2.0, 2.0}, 2.0);

  const WheelVector max_wheels = {1.0, 1.0};
  const ArcVector max_arc = {1.5, 1.5};

  BoxConstraints ddv_limits(max_wheels, max_arc);

  std::vector<WheelVector> check_inside_wheels;
  check_inside_wheels.push_back(
      kinematics.ComputeInverseKinematics(max_arc.Translation(), 0.0));
  check_inside_wheels.push_back(
      kinematics.ComputeInverseKinematics(-max_arc.Translation(), 0.0));
  check_inside_wheels.push_back(
      kinematics.ComputeInverseKinematics(0.0, max_arc.Rotation()));
  check_inside_wheels.push_back(
      kinematics.ComputeInverseKinematics(0.0, -max_arc.Rotation()));
  for (int ii = 0; ii < check_inside_wheels.size(); ++ii) {
    ASSERT_TRUE(std::abs(check_inside_wheels[ii].Left()) <= max_wheels.Left() &&
                std::abs(check_inside_wheels[ii].Right()) <= max_wheels.Right())
        << "check_inside_wheels[" << ii << "] = { "
        << check_inside_wheels[ii].Left() << ", "
        << check_inside_wheels[ii].Right() << " } exceeds max_wheel = { "
        << max_wheels.Left() << ", " << max_wheels.Right() << " }";
  }

  std::vector<WheelVector> check_outside_wheels;
  check_outside_wheels.push_back(kinematics.ComputeInverseKinematics(max_arc));
  check_outside_wheels.push_back(kinematics.ComputeInverseKinematics(
      ArcVector{max_arc.Translation(), -max_arc.Rotation()}));
  check_outside_wheels.push_back(kinematics.ComputeInverseKinematics(
      ArcVector{-max_arc.Translation(), max_arc.Rotation()}));
  check_outside_wheels.push_back(kinematics.ComputeInverseKinematics(-max_arc));
  for (int ii = 0; ii < check_outside_wheels.size(); ++ii) {
    ASSERT_TRUE(
        (std::abs(check_outside_wheels[ii].Left()) > max_wheels.Left()) ||
        (std::abs(check_outside_wheels[ii].Right()) > max_wheels.Right()))
        << "check_outside_wheels[" << ii << "] = { "
        << check_outside_wheels[ii].Left() << ", "
        << check_outside_wheels[ii].Right()
        << " } does not exceed max_wheel = { " << max_wheels.Left() << ", "
        << max_wheels.Right() << " }";
  }

  std::vector<ArcVector> check_inside_arc;
  check_inside_arc.push_back(
      kinematics.ComputeForwardKinematics(max_wheels.Left(), 0.0));
  check_inside_arc.push_back(
      kinematics.ComputeForwardKinematics(-max_wheels.Left(), 0.0));
  check_inside_arc.push_back(
      kinematics.ComputeForwardKinematics(0.0, max_wheels.Right()));
  check_inside_arc.push_back(
      kinematics.ComputeForwardKinematics(0.0, -max_wheels.Right()));
  for (int ii = 0; ii < check_inside_arc.size(); ++ii) {
    ASSERT_TRUE(std::abs(check_inside_arc[ii].Translation()) <=
                    max_arc.Translation() &&
                std::abs(check_inside_arc[ii].Rotation()) <= max_arc.Rotation())
        << "check_inside_arc[" << ii << "] = { "
        << check_inside_arc[ii].Translation() << ", "
        << check_inside_arc[ii].Rotation() << " } exceeds max_arc = {"
        << max_arc.Translation() << ", " << max_arc.Rotation() << " }";
  }

  std::vector<ArcVector> check_outside_arc;
  check_outside_arc.push_back(kinematics.ComputeForwardKinematics(max_wheels));
  check_outside_arc.push_back(kinematics.ComputeForwardKinematics(
      WheelVector{max_wheels.Left(), -max_wheels.Right()}));
  check_outside_arc.push_back(kinematics.ComputeForwardKinematics(
      WheelVector{-max_wheels.Left(), max_wheels.Right()}));
  check_outside_arc.push_back(kinematics.ComputeForwardKinematics(-max_wheels));
  for (int ii = 0; ii < check_outside_arc.size(); ++ii) {
    ASSERT_TRUE(std::abs(check_outside_arc[ii].Translation()) >
                    max_arc.Translation() ||
                std::abs(check_outside_arc[ii].Rotation()) > max_arc.Rotation())
        << "check_outside_arc[" << ii << "] = { "
        << check_outside_arc[ii].Translation() << ", "
        << check_outside_arc[ii].Rotation() << " } does not exceed max_arc = {"
        << max_arc.Translation() << ", " << max_arc.Rotation() << " }";
  }

  // Check that scaling down things that exceed limits end up
  // maintaining the same curvature.

  constexpr int num_steps = 11;
  double max_magnitude = 5.0;
  for (int isample = 0; isample < num_steps; ++isample) {
    const double magnitude = isample * max_magnitude / num_steps;
    {  // Input in wheel space:
      std::vector<WheelVector> in_wheels;
      in_wheels.push_back({magnitude, max_magnitude});
      in_wheels.push_back({magnitude, -max_magnitude});
      in_wheels.push_back({-magnitude, max_magnitude});
      in_wheels.push_back({-magnitude, -max_magnitude});
      in_wheels.push_back({max_magnitude, magnitude});
      in_wheels.push_back({max_magnitude, -magnitude});
      in_wheels.push_back({-max_magnitude, magnitude});
      in_wheels.push_back({-max_magnitude, -magnitude});
      for (int jcheck = 0; jcheck < in_wheels.size(); ++jcheck) {
        const ArcVector in_arc =
            kinematics.ComputeForwardKinematics(in_wheels[jcheck]);
        WheelVector out_wheels;
        ASSERT_TRUE(ddv_limits.BringInBounds(kinematics, in_wheels[jcheck],
                                             &out_wheels));
        const ArcVector out_arc =
            kinematics.ComputeForwardKinematics(out_wheels);
        if (kDebug) {
          fprintf(stderr,
                  "wheels { % 6.4f  % 6.4f } -> arc { % 6.4f  % 6.4f } --> "
                  "scaled arc { % 6.4f  % 6.4f }\n",
                  in_wheels[jcheck].Left(), in_wheels[jcheck].Right(),
                  in_arc.Translation(), in_arc.Rotation(),
                  out_arc.Translation(), out_arc.Rotation());
        }
        ASSERT_LE(std::abs(out_arc.Translation()),
                  std::abs(in_arc.Translation()));
        ASSERT_LE(std::abs(out_arc.Rotation()), std::abs(in_arc.Rotation()));
        ASSERT_NEAR(in_arc.Translation() * out_arc.Rotation(),
                    out_arc.Translation() * in_arc.Rotation(), kEpsilon)
            << "The curvature changed by too much.";
      }
    }
    {  // Input in arc space:
      std::vector<ArcVector> in_arc;
      in_arc.push_back({magnitude, max_magnitude});
      in_arc.push_back({magnitude, -max_magnitude});
      in_arc.push_back({-magnitude, max_magnitude});
      in_arc.push_back({-magnitude, -max_magnitude});
      in_arc.push_back({max_magnitude, magnitude});
      in_arc.push_back({max_magnitude, -magnitude});
      in_arc.push_back({-max_magnitude, magnitude});
      in_arc.push_back({-max_magnitude, -magnitude});
      for (int jcheck = 0; jcheck < in_arc.size(); ++jcheck) {
        ArcVector out_arc;
        ASSERT_TRUE(
            ddv_limits.BringInBounds(kinematics, in_arc[jcheck], &out_arc));
        ASSERT_LE(std::abs(out_arc.Translation()),
                  std::abs(in_arc[jcheck].Translation()));
        ASSERT_LE(std::abs(out_arc.Rotation()),
                  std::abs(in_arc[jcheck].Rotation()));
        ASSERT_NEAR(in_arc[jcheck].Translation() * out_arc.Rotation(),
                    out_arc.Translation() * in_arc[jcheck].Rotation(), kEpsilon)
            << "The curvature changed by too much.";
      }
    }
  }

  // Check that scaling down things that already respect the limits does not
  // actually do anything.

  max_magnitude = 0.1;
  for (int isample = 0; isample < num_steps; ++isample) {
    const double magnitude = isample * max_magnitude / num_steps;
    {  // Input in wheel space:
      std::vector<WheelVector> in_wheels;
      in_wheels.push_back({magnitude, max_magnitude});
      in_wheels.push_back({magnitude, -max_magnitude});
      in_wheels.push_back({-magnitude, max_magnitude});
      in_wheels.push_back({-magnitude, -max_magnitude});
      in_wheels.push_back({max_magnitude, magnitude});
      in_wheels.push_back({max_magnitude, -magnitude});
      in_wheels.push_back({-max_magnitude, magnitude});
      in_wheels.push_back({-max_magnitude, -magnitude});
      for (int jcheck = 0; jcheck < in_wheels.size(); ++jcheck) {
        ASSERT_LT(std::abs(in_wheels[jcheck].Left()), max_wheels.Left())
            << "Sanity check failed on isample == " << isample
            << ", jcheck == " << jcheck << ".";
        ASSERT_LT(std::abs(in_wheels[jcheck].Right()), max_wheels.Right())
            << "Sanity check failed on isample == " << isample
            << ", jcheck == " << jcheck << ".";
        const ArcVector in_arc =
            kinematics.ComputeForwardKinematics(in_wheels[jcheck]);
        ASSERT_LT(std::abs(in_arc.Translation()), max_arc.Translation())
            << "Sanity check failed on isample == " << isample
            << ", jcheck == " << jcheck << ".";
        ASSERT_LT(std::abs(in_arc.Rotation()), max_arc.Rotation())
            << "Sanity check failed on isample == " << isample
            << ", jcheck == " << jcheck << ".";
        WheelVector out_wheels;
        ASSERT_TRUE(ddv_limits.BringInBounds(kinematics, in_wheels[jcheck],
                                             &out_wheels));
        const ArcVector out_arc =
            kinematics.ComputeForwardKinematics(out_wheels);
        ASSERT_NEAR(in_arc.Translation(), out_arc.Translation(), kEpsilon);
        ASSERT_NEAR(in_arc.Rotation(), out_arc.Rotation(), kEpsilon);
      }
    }
    {  // Input in arc space:
      std::vector<ArcVector> in_arc;
      in_arc.push_back({magnitude, max_magnitude});
      in_arc.push_back({magnitude, -max_magnitude});
      in_arc.push_back({-magnitude, max_magnitude});
      in_arc.push_back({-magnitude, -max_magnitude});
      in_arc.push_back({max_magnitude, magnitude});
      in_arc.push_back({max_magnitude, -magnitude});
      in_arc.push_back({-max_magnitude, magnitude});
      in_arc.push_back({-max_magnitude, -magnitude});
      for (int jcheck = 0; jcheck < in_arc.size(); ++jcheck) {
        ASSERT_LT(std::abs(in_arc[jcheck].Translation()), max_arc.Translation())
            << "Sanity check failed on isample == " << isample
            << ", jcheck == " << jcheck << ".";
        ASSERT_LT(std::abs(in_arc[jcheck].Rotation()), max_arc.Rotation())
            << "Sanity check failed on isample == " << isample
            << ", jcheck == " << jcheck << ".";
        const WheelVector in_wheels =
            kinematics.ComputeInverseKinematics(in_arc[jcheck]);
        ASSERT_LT(std::abs(in_wheels.Left()), max_wheels.Left())
            << "Sanity check failed on isample == " << isample
            << ", jcheck == " << jcheck << ".";
        ASSERT_LT(std::abs(in_wheels.Right()), max_wheels.Right())
            << "Sanity check failed on isample == " << isample
            << ", jcheck == " << jcheck << ".";
        ArcVector out_arc;
        ASSERT_TRUE(
            ddv_limits.BringInBounds(kinematics, in_arc[jcheck], &out_arc));
        const WheelVector out_wheels =
            kinematics.ComputeInverseKinematics(out_arc);
        ASSERT_NEAR(in_wheels.Left(), out_wheels.Left(), kEpsilon);
        ASSERT_NEAR(in_wheels.Right(), out_wheels.Right(), kEpsilon);
      }
    }
  }
}

TEST(BoxConstraints, Reverse) {
  const BoxConstraints constraints(WheelVector(-1.0, -2.0),
                                   WheelVector(3.0, 4.0), ArcVector(-5.0, -6.0),
                                   ArcVector(7.0, 8.0));
  const BoxConstraints reversed =
      BoxConstraints::ReverseConstraints(constraints);
  EXPECT_EQ(reversed.MinWheelVector(), WheelVector(-3, -4));
  EXPECT_EQ(reversed.MaxWheelVector(), WheelVector(1, 2));
  EXPECT_EQ(reversed.MinArcVector(), ArcVector(-7, -8));
  EXPECT_EQ(reversed.MaxArcVector(), ArcVector(5, 6));
}

TEST(DynamicLimits, ComputeSafetyDistances) {
  Kinematics kinematics({2.0, 2.0}, 2.0);

  const WheelVector min_wheel_velocity = {-1.0, -1.0};
  const WheelVector max_wheel_velocity = {1.0, 1.0};
  const WheelVector min_wheel_acceleration = {-3.0, -3.0};
  const WheelVector max_wheel_acceleration = {3.0, 3.0};
  const ArcVector min_arc_velocity = {-0.5, -1.5};
  const ArcVector max_arc_velocity = {2.0, 1.5};
  const ArcVector min_arc_acceleration = {-4.0, -4.0};
  const ArcVector max_arc_acceleration = {4.0, 4.0};

  DynamicLimits limits(kinematics, min_wheel_velocity, max_wheel_velocity,
                       min_wheel_acceleration, max_wheel_acceleration,
                       min_arc_velocity, max_arc_velocity, min_arc_acceleration,
                       max_arc_acceleration);

  double reaction_time = 0.45;
  double dist_forward, dist_backward, dist_sideways;
  limits.ComputeSafetyDistances(reaction_time, &dist_forward, &dist_backward,
                                &dist_sideways);
  EXPECT_NEAR(dist_forward, 2.3, kEpsilon);
  EXPECT_NEAR(dist_backward, 1.15625, kEpsilon);
  EXPECT_NEAR(dist_sideways, 0.9, kEpsilon);

  limits.MutableVelocityLimits()->ScaleLimits(0.5);
  limits.ComputeSafetyDistances(reaction_time, &dist_forward, &dist_backward,
                                &dist_sideways);
  EXPECT_NEAR(dist_forward, 1.025, kEpsilon);
  EXPECT_NEAR(dist_backward, 0.5703125, kEpsilon);
  EXPECT_NEAR(dist_sideways, 0.45, kEpsilon);
}

TEST(DynamicLimits, Clamp) {
  constexpr double kCycleTime = 0.01;
  constexpr int kMaxNumSteps = 10000;

  Kinematics kinematics({2.0, 2.0}, 2.0);

  const WheelVector max_wheel_velocity = {1.0, 1.0};
  const WheelVector max_wheel_acceleration = {3.0, 3.0};
  const ArcVector max_arc_velocity = {1.5, 1.5};
  const ArcVector max_arc_acceleration = {4.5, 4.5};

  DynamicLimits limits(kinematics, max_wheel_velocity, max_wheel_acceleration,
                       max_arc_velocity, max_arc_acceleration);

  BoxConstraints incremental_limits(max_wheel_acceleration,
                                    max_arc_acceleration);
  incremental_limits.ScaleLimits(kCycleTime);

  std::vector<WheelVector> wheel_velocity_samples;
  wheel_velocity_samples.push_back({0.1, 0.1});
  wheel_velocity_samples.push_back({0.1, 1.1});
  wheel_velocity_samples.push_back({1.1, 0.1});
  wheel_velocity_samples.push_back({0.9, 1.1});
  wheel_velocity_samples.push_back({1.1, 0.9});
  wheel_velocity_samples.push_back({1.1, 1.1});
  wheel_velocity_samples.push_back({0.5, 10.0});
  wheel_velocity_samples.push_back({10.0, 0.5});

  for (int max_factor_step = 0; max_factor_step <= 3; ++max_factor_step) {
    const double max_factor = max_factor_step * 1.0 / 3.0;

    BoxConstraints scaled_limits = limits.VelocityLimits();
    scaled_limits.ScaleLimits(max_factor);

    for (int from = 0; from < wheel_velocity_samples.size(); ++from) {
      for (int to = 0; from < wheel_velocity_samples.size(); ++from) {
        for (int from_sign_left = -1; from_sign_left < 2; from_sign_left += 2) {
          for (int from_sign_right = -1; from_sign_right < 2;
               from_sign_right += 2) {
            for (int to_sign_left = -1; to_sign_left < 2; to_sign_left += 2) {
              for (int to_sign_right = -1; to_sign_right < 2;
                   to_sign_right += 2) {
                WheelVector current_wheel_velocity = {
                    from_sign_left * wheel_velocity_samples[from].Left(),
                    from_sign_right * wheel_velocity_samples[from].Right()};
                ArcVector current_arc_velocity =
                    kinematics.ComputeForwardKinematics(current_wheel_velocity);
                const WheelVector desired_wheel_velocity = {
                    to_sign_left * wheel_velocity_samples[to].Left(),
                    to_sign_right * wheel_velocity_samples[to].Right()};
                const ArcVector desired_arc_velocity =
                    kinematics.ComputeForwardKinematics(desired_wheel_velocity);
                WheelVector scaled_desired_wheel_velocity;
                ASSERT_TRUE(scaled_limits.BringInBounds(
                    kinematics, desired_wheel_velocity,
                    &scaled_desired_wheel_velocity));
                const ArcVector scaled_desired_arc_velocity =
                    kinematics.ComputeForwardKinematics(
                        scaled_desired_wheel_velocity);
                // Sanity check scaled_desired_arc_velocity.
                if (desired_arc_velocity.Translation() > 0.0) {
                  ASSERT_LE(scaled_desired_arc_velocity.Translation(),
                            desired_arc_velocity.Translation());
                } else if (desired_arc_velocity.Translation() < 0.0) {
                  ASSERT_GE(scaled_desired_arc_velocity.Translation(),
                            desired_arc_velocity.Translation());
                } else {
                  ASSERT_NEAR(scaled_desired_arc_velocity.Translation(), 0.0,
                              kEpsilon);
                }
                if (desired_arc_velocity.Rotation() > 0.0) {
                  ASSERT_LE(scaled_desired_arc_velocity.Rotation(),
                            desired_arc_velocity.Rotation());
                } else if (desired_arc_velocity.Rotation() < 0.0) {
                  ASSERT_GE(scaled_desired_arc_velocity.Rotation(),
                            desired_arc_velocity.Rotation());
                } else {
                  ASSERT_NEAR(scaled_desired_arc_velocity.Rotation(), 0.0,
                              kEpsilon);
                }
                ASSERT_NEAR(desired_arc_velocity.Translation() *
                                scaled_desired_arc_velocity.Rotation(),
                            scaled_desired_arc_velocity.Translation() *
                                desired_arc_velocity.Rotation(),
                            kEpsilon);

                // Simulate running from initial to desired. We may
                // start outside of dynamic limits, in which case
                // DynamicLimits will get us into bounds as
                // fast as it can given the acceleration limits.
                bool violation_ok =
                    (std::abs(current_wheel_velocity.Left()) >
                     max_wheel_velocity.Left()) ||
                    (std::abs(current_wheel_velocity.Right()) >
                     max_wheel_velocity.Right()) ||
                    (std::abs(current_arc_velocity.Translation()) >
                     max_arc_velocity.Translation()) ||
                    (std::abs(current_arc_velocity.Rotation()) >
                     max_arc_velocity.Rotation());

                if (kDebug) {
                  fprintf(stderr, "% 6.4f  % 6.4f     % 6.4f  % 6.4f   %d\n",
                          current_wheel_velocity.Left(),
                          current_wheel_velocity.Right(),
                          current_arc_velocity.Translation(),
                          current_arc_velocity.Rotation(),
                          static_cast<int>(violation_ok));
                }

                for (int step = 0; step < kMaxNumSteps; ++step) {
                  ArcVector scaled_desired_arc_velocity;
                  ASSERT_TRUE(scaled_limits.BringInBounds(
                      kinematics, desired_arc_velocity,
                      &scaled_desired_arc_velocity));

                  const ArcVector desired_arc_velocity_change =
                      scaled_desired_arc_velocity - current_arc_velocity;

                  ArcVector scaled_arc_velocity_change;
                  ASSERT_TRUE(scaled_limits.BringInBounds(
                      kinematics, desired_arc_velocity_change,
                      &scaled_arc_velocity_change));

                  const WheelVector scaled_wheel_velocity_change =
                      kinematics.ComputeInverseKinematics(
                          scaled_arc_velocity_change);

                  const WheelVector next_wheel_velocity =
                      current_wheel_velocity + scaled_wheel_velocity_change;

                  if (std::abs(next_wheel_velocity.Left() -
                               current_wheel_velocity.Left()) <= kEpsilon &&
                      std::abs(next_wheel_velocity.Right() -
                               current_wheel_velocity.Right()) <= kEpsilon) {
                    // Reached final velocity.
                    break;
                  }
                  const ArcVector next_arc_velocity =
                      kinematics.ComputeForwardKinematics(next_wheel_velocity);

                  const bool violation =
                      (std::abs(next_wheel_velocity.Left()) >
                       max_wheel_velocity.Left()) ||
                      (std::abs(next_wheel_velocity.Right()) >
                       max_wheel_velocity.Right()) ||
                      (std::abs(next_arc_velocity.Translation()) >
                       max_arc_velocity.Translation()) ||
                      (std::abs(next_arc_velocity.Rotation()) >
                       max_arc_velocity.Rotation());
                  if (violation_ok) {
                    violation_ok = violation;
                  } else {
                    ASSERT_FALSE(violation);
                  }

                  if (kDebug) {
                    fprintf(stderr, "% 6.4f  % 6.4f     % 6.4f  % 6.4f   %d\n",
                            next_wheel_velocity.Left(),
                            next_wheel_velocity.Right(),
                            next_arc_velocity.Translation(),
                            next_arc_velocity.Rotation(),
                            static_cast<int>(violation));
                  }

                  current_wheel_velocity = next_wheel_velocity;
                  current_arc_velocity = next_arc_velocity;
                }

                // Verify we ended up where we wanted.
                const ArcVector final_arc_velocity =
                    kinematics.ComputeForwardKinematics(current_wheel_velocity);
                ASSERT_NEAR(final_arc_velocity.Translation(),
                            current_arc_velocity.Translation(), kEpsilon);
                ASSERT_NEAR(final_arc_velocity.Rotation(),
                            current_arc_velocity.Rotation(), kEpsilon);
              }
            }
          }
        }
      }
    }
  }
}

TEST(DynamicLimits, Reverse) {
  const Kinematics kinematics;
  const BoxConstraints velocity_constraints(
      WheelVector(-1.0, -2.0), WheelVector(3.0, 4.0), ArcVector(-5.0, -6.0),
      ArcVector(7.0, 8.0));
  const BoxConstraints acceleration_constraints(
      WheelVector(-1.5, -2.5), WheelVector(3.5, 4.5), ArcVector(-5.5, -6.5),
      ArcVector(7.5, 8.5));
  const DynamicLimits limits(kinematics, velocity_constraints,
                             acceleration_constraints);

  const DynamicLimits reversed = DynamicLimits::ReverseLimits(limits);

  EXPECT_EQ(reversed.VelocityLimits().MinWheelVector(),
            WheelVector(-3.0, -4.0));
  EXPECT_EQ(reversed.VelocityLimits().MaxWheelVector(), WheelVector(1.0, 2.0));
  EXPECT_EQ(reversed.VelocityLimits().MinArcVector(), ArcVector(-7.0, -8.0));
  EXPECT_EQ(reversed.VelocityLimits().MaxArcVector(), ArcVector(5.0, 6.0));

  EXPECT_EQ(reversed.AccelerationLimits().MinWheelVector(),
            WheelVector(-3.5, -4.5));
  EXPECT_EQ(reversed.AccelerationLimits().MaxWheelVector(),
            WheelVector(1.5, 2.5));
  EXPECT_EQ(reversed.AccelerationLimits().MinArcVector(),
            ArcVector(-7.5, -8.5));
  EXPECT_EQ(reversed.AccelerationLimits().MaxArcVector(), ArcVector(5.5, 6.5));
}

TEST(DynamicLimitsFilter, BringInBounds) {
  Kinematics kinematics({0.2, 0.2}, 0.6);

  // For each "dimension" (left/right wheel, translation, and
  // rotation), check that all 6 permutations of limiting velocity can
  // happen: velocity should end up either as desired, or as limited
  // by velocity_factor*max_velocity, or as
  // cycle_time*aggressivity*max_acceleration away from the initial
  // velocity. Check both positive and negative desired velocities.
  //
  // For simplicity, check each dimension individually, and use zero
  // as initial velocity. While not exhaustive over all (we'd need to
  // consider all combinations of dimension, and all permutations for
  // each combination, as well as check from various kinds of initial
  // velocities), this should cover all the cases where regressions
  // are likely to creep in.

  constexpr double kA = 0.1;
  constexpr double kB = 1.0;
  constexpr double kC = 10.0;
  static struct {
    double desired, max_v, max_a;
  } permutations[] = {{kA, kB, kC}, {kA, kC, kB}, {kB, kA, kC},
                      {kB, kC, kA}, {kC, kA, kB}, {kC, kB, kA}};
  constexpr double kTolerance = 1.0e-6;
  const double kExpected = std::min({kA, kB, kC});

  static const double velocity_scales[] = {0.5, 1.0};
  static const double aggressivities[] = {0.5, 1.0};
  static const double cycle_durations[] = {0.2, 2.0};
  enum Dimension { kWheelLeft, kWheelRight, kArcTranslation, kArcRotation };
  static const Dimension dimensions[] = {kWheelLeft, kWheelRight,
                                         kArcTranslation, kArcRotation};
  for (const auto &velocity_scale : velocity_scales) {
    for (const auto &aggressivity : aggressivities) {
      for (const auto &cycle_duration : cycle_durations) {
        for (const auto &permutation : permutations) {
          for (const auto &dimension : dimensions) {
            for (int sign = -1; sign < 2; sign += 2) {
              std::string blurb = absl::StrFormat(
                  "velocity_scale: %v\naggressivity: %v\ncycle_duration: "
                  "%v\npermutation: %v  %v  %v\ndimension: %v\nsign: %v\n",
                  velocity_scale, aggressivity, cycle_duration,
                  permutation.desired, permutation.max_v, permutation.max_a,
                  dimension, sign);
              const double effective_max_v = permutation.max_v / velocity_scale;
              const double effective_max_a =
                  permutation.max_a / cycle_duration / aggressivity;
              const WheelVector max_wheel_velocity = {
                  (dimension == kWheelLeft ? effective_max_v : 10.0),
                  (dimension == kWheelRight ? effective_max_v : 10.0)};
              const WheelVector max_wheel_acceleration = {
                  (dimension == kWheelLeft ? effective_max_a : 100.0),
                  (dimension == kWheelRight ? effective_max_a : 100.0)};
              const ArcVector max_arc_velocity = {
                  (dimension == kArcTranslation ? effective_max_v : 10.0),
                  (dimension == kArcRotation ? effective_max_v : 10.0)};
              const ArcVector max_arc_acceleration = {
                  (dimension == kArcTranslation ? effective_max_a : 100.0),
                  (dimension == kArcRotation ? effective_max_a : 100.0)};

              const DynamicLimits limits(
                  kinematics, max_wheel_velocity, max_wheel_acceleration,
                  max_arc_velocity, max_arc_acceleration);
              const DynamicLimitsFilter filter(limits, velocity_scale,
                                               aggressivity, cycle_duration);

              switch (dimension) {
                case kWheelLeft:
                case kWheelRight: {
                  const WheelVector initial_wheel_velocity = {0.0, 0.0};
                  const WheelVector desired_wheel_velocity = {
                      (dimension == kWheelLeft ? sign * permutation.desired
                                               : 0.0),
                      (dimension == kWheelRight ? sign * permutation.desired
                                                : 0.0)};
                  const WheelVector expected_wheel_velocity = {
                      (dimension == kWheelLeft ? sign * kExpected : 0.0),
                      (dimension == kWheelRight ? sign * kExpected : 0.0)};
                  WheelVector clamped_wheel_velocity;
                  ASSERT_TRUE(filter.BringInBounds(initial_wheel_velocity,
                                                   desired_wheel_velocity,
                                                   &clamped_wheel_velocity))
                      << blurb;
                  ASSERT_TRUE(expected_wheel_velocity.isApprox(
                      clamped_wheel_velocity, kTolerance))
                      << "expected: " << expected_wheel_velocity.transpose()
                      << "\n"
                      << "clamped: " << clamped_wheel_velocity.transpose()
                      << "\n"
                      << blurb;
                } break;
                case kArcTranslation:
                case kArcRotation: {
                  const ArcVector initial_arc_velocity = {0.0, 0.0};
                  const ArcVector desired_arc_velocity = {
                      (dimension == kArcTranslation ? sign * permutation.desired
                                                    : 0.0),
                      (dimension == kArcRotation ? sign * permutation.desired
                                                 : 0.0)};
                  const ArcVector expected_arc_velocity = {
                      (dimension == kArcTranslation ? sign * kExpected : 0.0),
                      (dimension == kArcRotation ? sign * kExpected : 0.0)};
                  ArcVector clamped_arc_velocity;
                  ASSERT_TRUE(filter.BringInBounds(initial_arc_velocity,
                                                   desired_arc_velocity,
                                                   &clamped_arc_velocity))
                      << blurb;
                  ASSERT_TRUE(expected_arc_velocity.isApprox(
                      clamped_arc_velocity, kTolerance))
                      << "expected: " << expected_arc_velocity.transpose()
                      << "\n"
                      << "clamped: " << clamped_arc_velocity.transpose() << "\n"
                      << blurb;
                } break;
                default:
                  FAIL() << "Should never reach here.";
              }
            }  // for (sign)
          }    // for (dimension)
        }      // for (permutation)
      }        // for (cycle_duration)
    }          // for (aggressivity)
  }            // for (velocity_scale)
}

}  // namespace mobility::diff_drive
