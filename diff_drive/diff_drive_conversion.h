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

#ifndef MOBILITY_DIFF_DRIVE_DIFF_DRIVE_DIFF_DRIVE_CONVERSION_H_
#define MOBILITY_DIFF_DRIVE_DIFF_DRIVE_DIFF_DRIVE_CONVERSION_H_

#include <memory>
#include <vector>

#include "absl/status/status.h"
#include "diff_drive/diff_drive.pb.h"
#include "diff_drive/type_aliases.h"

namespace mobility::diff_drive {

// forward-declarations
class Kinematics;
class BoxConstraints;
class DynamicLimits;
class TrajectoryLimits;
class CurvePoint;
class CurvePtAndCord;
class Curve;
class State;
class StateAndTime;
class Trajectory;
class WheelState;
class WheelStateAndTotalMotion;
class WheelCurve;

absl::Status FromProto(const ArcVectorProto &proto, ArcVector *data_out);
absl::Status ToProto(const ArcVector &data_in, ArcVectorProto *proto_out);

absl::Status FromProto(const WheelVectorProto &proto, WheelVector *data_out);
absl::Status ToProto(const WheelVector &data_in, WheelVectorProto *proto_out);

absl::Status FromProto(const KinematicsProto &proto, Kinematics *data_out);
absl::Status ToProto(const Kinematics &data_in, KinematicsProto *proto_out);

absl::Status FromProto(const BoxConstraintsProto &proto,
                       BoxConstraints *data_out);
absl::Status ToProto(const BoxConstraints &data_in,
                     BoxConstraintsProto *proto_out);

absl::Status FromProto(const DynamicLimitsProto &proto,
                       DynamicLimits *data_out);
absl::Status ToProto(const DynamicLimits &data_in,
                     DynamicLimitsProto *proto_out);

absl::Status FromProto(const TrajectoryLimitsProto &proto,
                       TrajectoryLimits *data_out);
absl::Status ToProto(const TrajectoryLimits &data_in,
                     TrajectoryLimitsProto *proto_out);

absl::Status FromProto(const CurvePointProto &proto, CurvePoint *data_out);
absl::Status ToProto(const CurvePoint &data_in, CurvePointProto *proto_out);

absl::Status FromProto(const CurvePtAndCordProto &proto,
                       CurvePtAndCord *data_out);
absl::Status ToProto(const CurvePtAndCord &data_in,
                     CurvePtAndCordProto *proto_out);

absl::Status FromProto(const CurveProto &proto, Curve *data_out);
absl::Status ToProto(const Curve &data_in, CurveProto *proto_out);

absl::Status FromProto(const PiecewiseLinearCurveProto &proto, Curve *data_out);
absl::Status ToProto(const Curve &data_in,
                     PiecewiseLinearCurveProto *proto_out);

absl::Status FromProto(const StateProto &proto, State *data_out);
absl::Status ToProto(const State &data_in, StateProto *proto_out);

absl::Status FromProto(const StateAndTimeProto &proto, StateAndTime *data_out);
absl::Status ToProto(const StateAndTime &data_in, StateAndTimeProto *proto_out);

absl::Status FromProto(const TrajectoryProto &proto, Trajectory *data_out);
absl::Status ToProto(const Trajectory &data_in, TrajectoryProto *proto_out);

absl::Status FromProto(const WheelStateProto &proto, WheelState *data_out);
absl::Status ToProto(const WheelState &data_in, WheelStateProto *proto_out);

absl::Status FromProto(const WheelStateAndTotalMotionProto &proto,
                       WheelStateAndTotalMotion *data_out);
absl::Status ToProto(const WheelStateAndTotalMotion &data_in,
                     WheelStateAndTotalMotionProto *proto_out);

absl::Status FromProto(const WheelCurveProto &proto, WheelCurve *data_out);
absl::Status ToProto(const WheelCurve &data_in, WheelCurveProto *proto_out);

}  // namespace mobility::diff_drive

#endif  // MOBILITY_DIFF_DRIVE_DIFF_DRIVE_DIFF_DRIVE_CONVERSION_H_
