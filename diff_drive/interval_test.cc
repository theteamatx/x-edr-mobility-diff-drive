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

#include "diff_drive/interval.h"

#include <cstdint>
#include <string>
#include <utility>

#include "gtest/gtest.h"

namespace mobility {
namespace {

// Test intersection between the two intervals i1 and i2.  Tries
// i1.IntersectWith(i2) and vice versa. The intersection should change i1 iff
// changes_i1 is true, and the same for changes_i2.  The resulting
// intersection should be result.
void TestIntersect(const Interval<int> &i1, const Interval<int> &i2,
                   bool changes_i1, bool changes_i2,
                   const Interval<int> &result) {
  Interval<int> i;
  i = i1;
  EXPECT_TRUE(i.IntersectWith(i2) == changes_i1 && i == result);
  i = i2;
  EXPECT_TRUE(i.IntersectWith(i1) == changes_i2 && i == result);
}

TEST(IntervalTest, ConstructorsCopyAndClear) {
  Interval<int> empty;
  EXPECT_TRUE(empty.Empty());

  Interval<int> d2(0, 100);
  EXPECT_EQ(0, d2.min());
  EXPECT_EQ(100, d2.max());
  EXPECT_EQ(Interval<int>(0, 100), d2);
  EXPECT_NE(Interval<int>(0, 99), d2);

  empty = d2;
  EXPECT_EQ(0, d2.min());
  EXPECT_EQ(100, d2.max());
  EXPECT_TRUE(empty == d2);
  EXPECT_EQ(empty, d2);
  EXPECT_TRUE(d2 == empty);
  EXPECT_EQ(d2, empty);

  Interval<int> max_less_than_min(40, 20);
  EXPECT_TRUE(max_less_than_min.Empty());
  EXPECT_EQ(40, max_less_than_min.min());
  EXPECT_EQ(20, max_less_than_min.max());

  Interval<int> d3(10, 20);
  d3.Clear();
  EXPECT_TRUE(d3.Empty());
}

TEST(IntervalTest, GettersSetters) {
  Interval<int> d1(100, 200);

  // SetMin:
  d1.SetMin(30);
  EXPECT_EQ(30, d1.min());
  EXPECT_EQ(200, d1.max());

  // SetMax:
  d1.SetMax(220);
  EXPECT_EQ(30, d1.min());
  EXPECT_EQ(220, d1.max());

  // Set:
  d1.Clear();
  d1.Set(30, 220);
  EXPECT_EQ(30, d1.min());
  EXPECT_EQ(220, d1.max());

  // SpanningUnion:
  Interval<int32_t> d2;
  EXPECT_TRUE(!d1.SpanningUnion(d2));
  EXPECT_EQ(30, d1.min());
  EXPECT_EQ(220, d1.max());

  EXPECT_TRUE(d2.SpanningUnion(d1));
  EXPECT_EQ(30, d2.min());
  EXPECT_EQ(220, d2.max());

  d2.SetMin(40);
  d2.SetMax(100);
  EXPECT_TRUE(!d1.SpanningUnion(d2));
  EXPECT_EQ(30, d1.min());
  EXPECT_EQ(220, d1.max());

  d2.SetMin(20);
  d2.SetMax(100);
  EXPECT_TRUE(d1.SpanningUnion(d2));
  EXPECT_EQ(20, d1.min());
  EXPECT_EQ(220, d1.max());

  d2.SetMin(50);
  d2.SetMax(300);
  EXPECT_TRUE(d1.SpanningUnion(d2));
  EXPECT_EQ(20, d1.min());
  EXPECT_EQ(300, d1.max());

  d2.SetMin(0);
  d2.SetMax(500);
  EXPECT_TRUE(d1.SpanningUnion(d2));
  EXPECT_EQ(0, d1.min());
  EXPECT_EQ(500, d1.max());

  d2.SetMin(100);
  d2.SetMax(0);
  EXPECT_TRUE(!d1.SpanningUnion(d2));
  EXPECT_EQ(0, d1.min());
  EXPECT_EQ(500, d1.max());
  EXPECT_TRUE(d2.SpanningUnion(d1));
  EXPECT_EQ(0, d2.min());
  EXPECT_EQ(500, d2.max());
}

TEST(IntervalTest, CoveringOps) {
  const Interval<int> empty;
  const Interval<int> d(100, 200);
  const Interval<int> d1(0, 50);
  const Interval<int> d2(50, 110);
  const Interval<int> d3(110, 180);
  const Interval<int> d4(180, 220);
  const Interval<int> d5(220, 300);
  const Interval<int> d6(100, 150);
  const Interval<int> d7(150, 200);
  const Interval<int> d8(0, 300);

  // Intersection:
  EXPECT_TRUE(d.Intersects(d));
  EXPECT_TRUE(!empty.Intersects(d) && !d.Intersects(empty));
  EXPECT_TRUE(!d.Intersects(d1) && !d1.Intersects(d));
  EXPECT_TRUE(d.Intersects(d2) && d2.Intersects(d));
  EXPECT_TRUE(d.Intersects(d3) && d3.Intersects(d));
  EXPECT_TRUE(d.Intersects(d4) && d4.Intersects(d));
  EXPECT_TRUE(!d.Intersects(d5) && !d5.Intersects(d));
  EXPECT_TRUE(d.Intersects(d6) && d6.Intersects(d));
  EXPECT_TRUE(d.Intersects(d7) && d7.Intersects(d));
  EXPECT_TRUE(d.Intersects(d8) && d8.Intersects(d));

  Interval<int> i;
  EXPECT_TRUE(d.Intersects(d, &i) && d == i);
  EXPECT_TRUE(!empty.Intersects(d, nullptr) && !d.Intersects(empty, nullptr));
  EXPECT_TRUE(!d.Intersects(d1, nullptr) && !d1.Intersects(d, nullptr));
  EXPECT_TRUE(d.Intersects(d2, &i) && i == Interval<int>(100, 110));
  EXPECT_TRUE(d2.Intersects(d, &i) && i == Interval<int>(100, 110));
  EXPECT_TRUE(d.Intersects(d3, &i) && i == d3);
  EXPECT_TRUE(d3.Intersects(d, &i) && i == d3);
  EXPECT_TRUE(d.Intersects(d4, &i) && i == Interval<int>(180, 200));
  EXPECT_TRUE(d4.Intersects(d, &i) && i == Interval<int>(180, 200));
  EXPECT_TRUE(!d.Intersects(d5, nullptr) && !d5.Intersects(d, nullptr));
  EXPECT_TRUE(d.Intersects(d6, &i) && i == d6);
  EXPECT_TRUE(d6.Intersects(d, &i) && i == d6);
  EXPECT_TRUE(d.Intersects(d7, &i) && i == d7);
  EXPECT_TRUE(d7.Intersects(d, &i) && i == d7);
  EXPECT_TRUE(d.Intersects(d8, &i) && i == d);
  EXPECT_TRUE(d8.Intersects(d, &i) && i == d);

  // Test IntersectsWith().
  // Arguments are TestIntersect(i1, i2, changes_i1, changes_i2, result).
  TestIntersect(empty, d, false, true, empty);
  TestIntersect(d, d1, true, true, empty);
  TestIntersect(d1, d2, true, true, empty);
  TestIntersect(d, d2, true, true, Interval<int>(100, 110));
  TestIntersect(d8, d, true, false, d);
  TestIntersect(d8, d1, true, false, d1);
  TestIntersect(d8, d5, true, false, d5);

  // Contains:
  EXPECT_TRUE(!empty.Contains(d) && !d.Contains(empty));
  EXPECT_TRUE(d.Contains(d));
  EXPECT_TRUE(!d.Contains(d1) && !d1.Contains(d));
  EXPECT_TRUE(!d.Contains(d2) && !d2.Contains(d));
  EXPECT_TRUE(d.Contains(d3) && !d3.Contains(d));
  EXPECT_TRUE(!d.Contains(d4) && !d4.Contains(d));
  EXPECT_TRUE(!d.Contains(d5) && !d5.Contains(d));
  EXPECT_TRUE(d.Contains(d6) && !d6.Contains(d));
  EXPECT_TRUE(d.Contains(d7) && !d7.Contains(d));
  EXPECT_TRUE(!d.Contains(d8) && d8.Contains(d));

  EXPECT_TRUE(d.Contains(100));
  EXPECT_TRUE(!d.Contains(200));
  EXPECT_TRUE(d.Contains(150));
  EXPECT_TRUE(!d.Contains(99));
  EXPECT_TRUE(!d.Contains(201));
}

TEST(IntervalTest, Length) {
  const Interval<int> empty1;
  const Interval<int> empty2(1, 1);
  const Interval<int> empty3(1, 0);
  const Interval<double> empty4(1.0, 0.0);
  const Interval<int> d1(1, 2);
  const Interval<int> d2(0, 50);
  const Interval<double> d3(0.0, 1.0);
  const Interval<double> d4(1.0, 1.5);

  EXPECT_EQ(0, empty1.Length());
  EXPECT_EQ(0, empty2.Length());
  EXPECT_EQ(0, empty3.Length());
  EXPECT_EQ(0.0, empty4.Length());
  EXPECT_EQ(1, d1.Length());
  EXPECT_EQ(50, d2.Length());
  EXPECT_EQ(1.0, d3.Length());
  EXPECT_EQ(0.5, d4.Length());
}

TEST(IntervalTest, IntervalOfTypeWithNoOperatorMinus) {
  // Interval<T> should work even if T does not support operator-().  We just
  // can't call Interval<T>::Length() for such types.
  const Interval<std::string> d1("a", "b");
  const Interval<std::pair<int, int>> d2({1, 2}, {4, 3});
  EXPECT_EQ("a", d1.min());
  EXPECT_EQ("b", d1.max());
  EXPECT_EQ(std::make_pair(1, 2), d2.min());
  EXPECT_EQ(std::make_pair(4, 3), d2.max());
}

struct NoEquals {
  NoEquals(int v) : value(v) {}  // NOLINT
  int value;
  bool operator<(const NoEquals &other) const { return value < other.value; }
};

TEST(IntervalTest, OrderedComparisonForTypeWithoutEquals) {
  const Interval<NoEquals> d1(0, 4);
  const Interval<NoEquals> d2(0, 3);
  const Interval<NoEquals> d3(1, 4);
  const Interval<NoEquals> d4(1, 5);
  const Interval<NoEquals> d6(0, 4);
  EXPECT_TRUE(d1 < d2);
  EXPECT_TRUE(d1 < d3);
  EXPECT_TRUE(d1 < d4);
  EXPECT_FALSE(d1 < d6);
}

TEST(IntervalTest, ConstexprAccess) {
  constexpr Interval<int> interval{5, 10};
  static_assert(interval.min() == 5, "");
  static_assert(interval.max() == 10, "");
  static_assert(!interval.Empty(), "");
  static_assert(interval.Contains(7), "");
  static_assert(!interval.Contains(Interval<int>{0, 1}), "");
  static_assert(interval.Length() == 5, "");
  static_assert(interval.Intersects(Interval<int>{3, 6}), "");
  static_assert(interval == interval, "");
  static_assert(interval != Interval<int>{0, 1}, "");
}

}  // unnamed namespace
}  // namespace mobility
