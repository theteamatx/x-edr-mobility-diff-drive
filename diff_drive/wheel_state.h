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

#ifndef MOBILITY_DIFF_DRIVE_DIFF_DRIVE_WHEEL_STATE_H_
#define MOBILITY_DIFF_DRIVE_DIFF_DRIVE_WHEEL_STATE_H_

#include "diff_drive/type_aliases.h"

namespace mobility::diff_drive {

// Represents a wheel state for a differential-drive robot, including
// the wheel positions and motion rates (fraction of total wheel motion
// imparted on each wheel).
// The wheel motions are parametrized by the total wheel motion (w) defined as:
//   w = abs(left_wheel_rotation) + abs(right_wheel_rotation)
// The relative wheel "speeds" are described using the motion rates (fl, fr)
// which are fractions of the motion for each wheel:
//   fl = d(left_wheel_rotation) / dw
//   fr = d(right_wheel_rotation) / dw
class WheelState {
 public:
  // Default constructor. Leaves data members in undefined values.
  WheelState() = default;

  // Create a diff-drive wheel state as a wheel position vector and optional
  // motion rates.
  explicit WheelState(const WheelVector& wheel_positions,
                      const WheelVector& motion_rates = WheelVector{0.5, 0.5})
      : wheel_positions_(wheel_positions), motion_rates_(motion_rates) {}

  // Get a const-reference to the wheel positions at this wheel state.
  const WheelVector& GetPositions() const { return wheel_positions_; }
  // Sets the wheel positions at this wheel state.
  void SetPositions(const WheelVector& wheel_positions) {
    wheel_positions_ = wheel_positions;
  }

  // Get a const-reference to the motion rates at this wheel state.
  const WheelVector& GetMotionRates() const { return motion_rates_; }
  // Sets the motion rates at this wheel state.
  void SetMotionRates(const WheelVector& motion_rates) {
    motion_rates_ = motion_rates;
  }

  // Get the curvature of the path of the robot at this wheel state.
  double GetCurvature(double wheel_base) const {
    return (motion_rates_.Right() - motion_rates_.Left()) /
           (motion_rates_.Left() + motion_rates_.Right()) * 2.0 / wheel_base;
  }

  // Normalize the motion rates such that their absolute values sum to 1.
  void NormalizeMotionRates() { motion_rates_ /= motion_rates_.lpNorm<1>(); }

  // Extrapolates from the robot's stored wheel positions by assuming constant
  // wheel motion rates over a given amount of total wheel motion.
  WheelState Extrapolate(double dw) const {
    const WheelVector rel_motion = motion_rates_ * dw;
    return WheelState(wheel_positions_ + rel_motion, motion_rates_);
  }

 private:
  WheelVector wheel_positions_;
  WheelVector motion_rates_;
};

// Represents a wheel state for a differential-drive robot along with a
// total wheel motion value marking its position along a curve, parametrized by
// the total wheel motion along the curve.
// As defined above, the total wheel motion (w) is defined as:
//   w = abs(left_wheel_rotation) + abs(right_wheel_rotation)
class WheelStateAndTotalMotion {
 public:
  double total_motion;
  WheelState state;

  // This comparator class is used when searching or sorting wheel states
  // according to their total wheel motion parameter. This is a less-than
  // comparison (per C++ convention).
  class TotalMotionComparator {
   public:
    bool operator()(const WheelStateAndTotalMotion& lhs,
                    const WheelStateAndTotalMotion& rhs) const {
      return lhs.total_motion < rhs.total_motion;
    }
    bool operator()(const WheelStateAndTotalMotion& pt, double w) const {
      return pt.total_motion < w;
    }
    bool operator()(double w, const WheelStateAndTotalMotion& pt) const {
      return w < pt.total_motion;
    }
  };
};

}  // namespace mobility::diff_drive

#endif  // MOBILITY_DIFF_DRIVE_DIFF_DRIVE_WHEEL_STATE_H_
