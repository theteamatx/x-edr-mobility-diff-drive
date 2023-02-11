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
#include <numeric>

#include "absl/log/check.h"

namespace mobility::diff_drive {

Kinematics::Kinematics(const WheelVector &wheel_radius, double wheel_base)
    : wheel_radius_(wheel_radius), wheel_base_(wheel_base) {
  CHECK_GT(wheel_radius.Left(), 0.0) << "Wheel radius (left) must be positive";
  CHECK_GT(wheel_radius.Right(), 0.0) << "Wheel radius (right) must be positive";
  CHECK_GT(wheel_base, 0.0) << "Wheel base must be positive";
}

void Kinematics::Configure(const WheelVector &wheel_radius, double wheel_base) {
  CHECK_GT(wheel_radius.Left(), 0.0) << "Wheel radius (left) must be positive";
  CHECK_GT(wheel_radius.Right(), 0.0) << "Wheel radius (right) must be positive";
  CHECK_GT(wheel_base, 0.0) << "Wheel base must be positive";
  wheel_radius_ = wheel_radius;
  wheel_base_ = wheel_base;
}

}  // namespace mobility::diff_drive
