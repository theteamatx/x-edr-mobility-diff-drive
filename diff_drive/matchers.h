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

#ifndef MOBILITY_DIFF_DRIVE_DIFF_DRIVE_MATCHERS_H_
#define MOBILITY_DIFF_DRIVE_DIFF_DRIVE_MATCHERS_H_

#include <type_traits>

#include "diff_drive/curve_trajectory_utils.h"
#include "diff_drive/dynamic_limits.h"
#include "diff_drive/state.h"
#include "diff_drive/trajectory.h"
#include "diff_drive/type_aliases.h"
#include "gmock/gmock.h"

namespace mobility::diff_drive {

MATCHER_P(SatisfiesLimits, limits, "satisfies velocity limits") {
  using ArgType = std::decay_t<decltype(arg)>;
  auto check_state = [](const diff_drive::State& state,
                        const diff_drive::DynamicLimits& limits,
                        auto* result_listener) {
    if (!limits.VelocityLimits().IsInBounds(limits.GetKinematics(),
                                            state.GetArcVelocity())) {
      ArcVector limited_arc_velocity;
      limits.VelocityLimits().BringInBounds(limits.GetKinematics(),
                                            state.GetArcVelocity(),
                                            &limited_arc_velocity);
      *result_listener << " velocity limits " << limits.VelocityLimits()
                       << " are violated. Limited velocity is {v: "
                       << limited_arc_velocity.Translation()
                       << ", w: " << limited_arc_velocity.Rotation()
                       << "} from original {v: "
                       << state.GetArcVelocity().Translation()
                       << ", w: " << state.GetArcVelocity().Rotation() << "}.";
      return false;
    }
    return true;
  };
  if constexpr (std::is_same_v<diff_drive::Trajectory, ArgType>) {
    const diff_drive::Trajectory& traj = arg;
    for (const auto& pt : traj.GetStateIteratorRange()) {
      if (!check_state(pt.state, limits, result_listener)) {
        *result_listener << " At time t=" << pt.time << ".";
        return false;
      }
    }
  } else if constexpr (std::is_same_v<diff_drive::StateAndTime, ArgType>) {
    const diff_drive::StateAndTime& state_and_time = arg;
    if (!check_state(state_and_time.state, limits, result_listener)) {
      *result_listener << " At time t=" << state_and_time.time << ".";
      return false;
    }
  } else if constexpr (std::is_same_v<diff_drive::State, ArgType>) {
    const diff_drive::State& state = arg;
    return check_state(state, limits, result_listener);
  } else {
    static_assert(sizeof(ArgType) == 0, "Unsupported type.");
  }
  return true;
}

MATCHER_P2(IsApproxState, expected, tolerance, "") {
  const diff_drive::State& actual = arg;
  if (!actual.GetPose().isApprox(expected.GetPose(), tolerance)) {
    return false;
  }
  if (!actual.GetArcVelocity().isApprox(expected.GetArcVelocity(), tolerance)) {
    return false;
  }
  return true;
}

}  // namespace mobility::diff_drive

#endif  // MOBILITY_DIFF_DRIVE_DIFF_DRIVE_MATCHERS_H_
