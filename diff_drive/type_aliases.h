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

#ifndef MOBILITY_DIFF_DRIVE_DIFF_DRIVE_TYPE_ALIASES_H_
#define MOBILITY_DIFF_DRIVE_DIFF_DRIVE_TYPE_ALIASES_H_

#include <cmath>

#include "eigenmath/mean_and_covariance.h"  // IWYU pragma: export
#include "eigenmath/pose2.h"                // IWYU pragma: export
#include "eigenmath/pose3.h"                // IWYU pragma: export
#include "absl/strings/str_format.h"

namespace mobility {

#define MOBILITY_DEFINE_VECTOR2D_ALIAS(Vector2dAlias, FirstFieldAlias,       \
                                       SecondFieldAlias)                     \
  class Vector2dAlias : public eigenmath::Vector2d {                         \
   public:                                                                   \
    typedef eigenmath::Vector2d Base;                                        \
                                                                             \
    Vector2dAlias() : eigenmath::Vector2d(0.0, 0.0) {}                       \
                                                                             \
    template <typename OtherDerived>                                         \
    Vector2dAlias(const Eigen::MatrixBase<OtherDerived> &other) /* NOLINT */ \
        : eigenmath::Vector2d(other) {}                                      \
                                                                             \
    template <typename OtherDerived>                                         \
    Vector2dAlias &operator=(const Eigen::MatrixBase<OtherDerived> &other) { \
      this->Base::operator=(other);                                          \
      return *this;                                                          \
    }                                                                        \
                                                                             \
    Vector2dAlias(double FirstFieldAlias, double SecondFieldAlias)           \
        : eigenmath::Vector2d(FirstFieldAlias, SecondFieldAlias) {}          \
                                                                             \
    void Set(double FirstFieldAlias, double SecondFieldAlias) {              \
      *this << FirstFieldAlias, SecondFieldAlias;                            \
    }                                                                        \
                                                                             \
    friend Vector2dAlias operator-(const Vector2dAlias &in) {                \
      return Vector2dAlias(-static_cast<const Base &>(in));                  \
    }                                                                        \
                                                                             \
    double FirstFieldAlias() const { return (*this)(0); }                    \
    double SecondFieldAlias() const { return (*this)(1); }                   \
                                                                             \
    double &FirstFieldAlias() { return (*this)(0); }                         \
    double &SecondFieldAlias() { return (*this)(1); }                        \
  };

// Struct for passing around things that represent both wheels of a
// differentially driven base. E.g. velocities, encoder values,
// encoder deltas.
MOBILITY_DEFINE_VECTOR2D_ALIAS(WheelVector, Left, Right)

// Struct for passing around things that represent a planar motion
// that follows an arc of circle. The translation part is the velocity
// (or arc length) along the curve, and rotation is the angular
// velocity (or angular increment). Straight-line motions and pure
// rotations can also be represented naturally, the former have
// rotation==0 and the latter translation==0.
MOBILITY_DEFINE_VECTOR2D_ALIAS(ArcVector, Translation, Rotation)

// ComputeCurvature computes the curvature for a given arc_velocity. For all
// speeds smaller or equal min_speed_threshold, it uses that threshold instead.
// This limits curvature from going to infinity.
inline double ComputeCurvature(ArcVector arc_velocity,
                               double min_speed_threshold = 0.) {
  if (std::abs(arc_velocity.Translation()) > min_speed_threshold) {
    return arc_velocity.Rotation() / arc_velocity.Translation();
  } else {
    return arc_velocity.Rotation() /
           std::copysign(min_speed_threshold, arc_velocity.Translation());
  }
}

template <typename Sink>
void AbslStringify(Sink &sink, const ArcVector &arc) {
  absl::Format(&sink, "(t: %f, r: %f)", arc.Translation(), arc.Rotation());
}

template <typename Sink>
void AbslStringify(Sink &sink, const WheelVector &wheel) {
  absl::Format(&sink, "(l: %f, r: %f)", wheel.Left(), wheel.Right());
}

#undef MOBILITY_DEFINE_VECTOR2D_ALIAS
#undef MOBILITY_DEFINE_VECTOR2D_ALIAS_OPS

}  // namespace mobility

#endif  // MOBILITY_DIFF_DRIVE_DIFF_DRIVE_TYPE_ALIASES_H_
