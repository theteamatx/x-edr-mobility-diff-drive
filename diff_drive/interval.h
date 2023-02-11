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


#ifndef MOBILITY_DIFF_DRIVE_DIFF_DRIVE_INTERVAL_H_
#define MOBILITY_DIFF_DRIVE_DIFF_DRIVE_INTERVAL_H_

#include <algorithm>
#include <ostream>

namespace mobility {

template <typename T>
class Interval {
 public:
  // Construct an Interval representing an empty interval.
  constexpr Interval() : min_(), max_() {}

  // Construct an Interval representing the interval [min, max). If min < max,
  // the constructed object will represent the non-empty interval containing all
  // values from min up to (but not including) max. On the other hand, if min >=
  // max, the constructed object will represent the empty interval.
  constexpr Interval(const T& min, const T& max) : min_(min), max_(max) {}

  constexpr const T& min() const { return min_; }
  constexpr const T& max() const { return max_; }
  void SetMin(const T& t) { min_ = t; }
  void SetMax(const T& t) { max_ = t; }

  void Set(const T& min, const T& max) {
    SetMin(min);
    SetMax(max);
  }

  void Clear() { *this = {}; }

  constexpr bool Empty() const { return min() >= max(); }

  // Returns the length of this interval. The value returned is zero if
  // Empty() is true; otherwise the value returned is max() - min().
  constexpr auto Length() const { return (Empty() ? min() : max()) - min(); }

  // Returns true iff t >= min() && t < max().
  constexpr bool Contains(const T& t) const { return min() <= t && max() > t; }

  // Returns true iff *this and i are non-empty, and *this includes i. "*this
  // includes i" means that for all t, if i.Contains(t) then this->Contains(t).
  // Note the unintuitive consequence of this definition: this method always
  // returns false when i is the empty interval.
  constexpr bool Contains(const Interval& i) const {
    return !Empty() && !i.Empty() && min() <= i.min() && max() >= i.max();
  }

  // Returns true iff there exists some point t for which this->Contains(t) &&
  // i.Contains(t) evaluates to true, i.e. if the intersection is non-empty.
  constexpr bool Intersects(const Interval& i) const {
    return !Empty() && !i.Empty() && min() < i.max() && max() > i.min();
  }

  // Returns true iff there exists some point t for which this->Contains(t) &&
  // i.Contains(t) evaluates to true, i.e. if the intersection is non-empty.
  // Furthermore, if the intersection is non-empty and the intersection pointer
  // is not null, this method stores the calculated intersection in
  // *intersection.
  bool Intersects(const Interval& i, Interval* out) const;

  // Sets *this to be the intersection of itself with i. Returns true iff
  // *this was modified.
  bool IntersectWith(const Interval& i);

  // Calculates the smallest interval containing both *this i, and updates *this
  // to represent that interval, and returns true iff *this was modified.
  bool SpanningUnion(const Interval& i);

  constexpr friend bool operator==(const Interval& a, const Interval& b) {
    return (a.Empty() && b.Empty()) ||
           ((!a.Empty() && !b.Empty()) &&
            (a.min() == b.min() && a.max() == b.max()));
  }

  constexpr friend bool operator!=(const Interval& a, const Interval& b) {
    return !(a == b);
  }

  // Defines a comparator which can be used to induce an order on Intervals, so
  // that, for example, they can be stored in an ordered container such as
  // std::set. The ordering is arbitrary, but does provide the guarantee that,
  // for non-empty intervals X and Y, if X contains Y, then X <= Y.
  friend bool operator<(const Interval& a, const Interval& b) {
    return a.min() < b.min() || (!(b.min() < a.min()) && b.max() < a.max());
  }

  friend std::ostream& operator<<(std::ostream& out, const Interval& i) {
    return out << "[" << i.min() << ", " << i.max() << ")";
  }

 private:
  T min_;  // Inclusive lower bound.
  T max_;  // Exclusive upper bound.
};

template <typename T>
bool Interval<T>::Intersects(const Interval& i, Interval* out) const {
  if (!Intersects(i)) return false;
  if (out != nullptr) {
    *out = Interval(std::max(min(), i.min()), std::min(max(), i.max()));
  }
  return true;
}

template <typename T>
bool Interval<T>::IntersectWith(const Interval& i) {
  if (Empty()) return false;
  bool modified = false;
  if (i.min() > min()) {
    SetMin(i.min());
    modified = true;
  }
  if (i.max() < max()) {
    SetMax(i.max());
    modified = true;
  }
  return modified;
}

template <typename T>
bool Interval<T>::SpanningUnion(const Interval& i) {
  if (i.Empty()) return false;
  if (Empty()) {
    *this = i;
    return true;
  }
  bool modified = false;
  if (i.min() < min()) {
    SetMin(i.min());
    modified = true;
  }
  if (i.max() > max()) {
    SetMax(i.max());
    modified = true;
  }
  return modified;
}

}  // namespace mobility

#endif  // MOBILITY_DIFF_DRIVE_DIFF_DRIVE_INTERVAL_H_
