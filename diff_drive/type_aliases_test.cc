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

#include "diff_drive/type_aliases.h"

#include <random>
#include <vector>

#include "gtest/gtest.h"

namespace mobility {
namespace {

TEST(VectorTypes, Initialize) {
  const double left = 1.2;
  const double right = 3.4;

  const double translation = 2.1;
  const double rotation = 4.3;

  const WheelVector wheels{left, right};
  ASSERT_DOUBLE_EQ(wheels.Left(), left);
  ASSERT_DOUBLE_EQ(wheels.Right(), right);

  const ArcVector arc{translation, rotation};
  ASSERT_DOUBLE_EQ(arc.Translation(), translation);
  ASSERT_DOUBLE_EQ(arc.Rotation(), rotation);
}

TEST(VectorTypes, DefaultConstruct) {
  std::mt19937 rnd;
  std::uniform_int_distribution<int> dist(100, 500);
  for (int ii = 0; ii < 100; ++ii) {
    std::vector<int> some_other_memory;
    some_other_memory.reserve(dist(rnd));
    const WheelVector wheels;
    const ArcVector arc;
    ASSERT_DOUBLE_EQ(wheels.Left(), 0.0);
    ASSERT_DOUBLE_EQ(wheels.Right(), 0.0);
    ASSERT_DOUBLE_EQ(arc.Translation(), 0.0);
    ASSERT_DOUBLE_EQ(arc.Rotation(), 0.0);
  }
}

TEST(VectorTypes, Multiply) {
  const double left = 1.2;
  const double right = 3.4;
  const WheelVector wheels(left, right);

  const double translation = 2.1;
  const double rotation = 4.3;
  const ArcVector arc(translation, rotation);

  const double scalar = 2.0;

  const WheelVector aa = scalar * wheels;
  ASSERT_DOUBLE_EQ(aa.Left(), scalar * left);
  ASSERT_DOUBLE_EQ(aa.Right(), scalar * right);

  const ArcVector bb = scalar * arc;
  ASSERT_DOUBLE_EQ(bb.Translation(), scalar * translation);
  ASSERT_DOUBLE_EQ(bb.Rotation(), scalar * rotation);

  const WheelVector cc = wheels * scalar;
  ASSERT_DOUBLE_EQ(cc.Left(), scalar * left);
  ASSERT_DOUBLE_EQ(cc.Right(), scalar * right);

  const ArcVector dd = arc * scalar;
  ASSERT_DOUBLE_EQ(dd.Translation(), scalar * translation);
  ASSERT_DOUBLE_EQ(dd.Rotation(), scalar * rotation);
}

TEST(VectorTypes, Add) {
  const double left = 1.2;
  const double right = 3.4;
  const WheelVector wheels(left, right);

  const double translation = 2.1;
  const double rotation = 4.3;
  const ArcVector arc(translation, rotation);

  const double scalar = 2.0;

  const WheelVector aa = scalar * wheels;
  const WheelVector ee = aa + wheels;
  ASSERT_DOUBLE_EQ(ee.Left(), (1.0 + scalar) * left);
  ASSERT_DOUBLE_EQ(ee.Right(), (1.0 + scalar) * right);

  const ArcVector bb = scalar * arc;
  const ArcVector ff = bb + arc;
  ASSERT_DOUBLE_EQ(ff.Translation(), (1.0 + scalar) * translation);
  ASSERT_DOUBLE_EQ(ff.Rotation(), (1.0 + scalar) * rotation);
}

TEST(VectorTypes, Assign) {
  const double left = 1.2;
  const double right = 3.4;
  const WheelVector wheels(left, right);

  const double translation = 2.1;
  const double rotation = 4.3;
  const ArcVector arc(translation, rotation);

  const eigenmath::Vector2d v2d{left, right};
  const WheelVector ii = v2d;
  ASSERT_DOUBLE_EQ(ii.Left(), left);
  ASSERT_DOUBLE_EQ(ii.Right(), right);
}

}  // namespace
}  // namespace mobility
