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

#include "diff_drive/diff_drive_conversion.h"

#include <algorithm>
#include <cmath>
#include <utility>

#include "absl/log/check.h"
#include "absl/status/status.h"
#include "absl/strings/str_cat.h"
#include "diff_drive/curve.h"
#include "diff_drive/curve_point.h"
#include "diff_drive/dynamic_limits.h"
#include "diff_drive/kinematics.h"
#include "diff_drive/state.h"
#include "diff_drive/trajectory.h"
#include "diff_drive/trajectory_limits.h"
#include "diff_drive/wheel_curve.h"
#include "diff_drive/wheel_state.h"
#include "eigenmath/conversions.h"
#include "eigenmath/eigenmath.pb.h"

namespace mobility::diff_drive {

namespace {
constexpr double kEpsilon = 1.0e-3;
}  // namespace

#define MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(expr) \
  {                                               \
    auto status = (expr);                         \
    if (!status.ok()) return status;              \
  }

absl::Status FromProto(const ArcVectorProto &proto, ArcVector *data_out) {
  data_out->Translation() = proto.translation();
  data_out->Rotation() = proto.rotation();
  return absl::OkStatus();
}

absl::Status ToProto(const ArcVector &data_in, ArcVectorProto *proto_out) {
  proto_out->set_translation(data_in.Translation());
  proto_out->set_rotation(data_in.Rotation());
  return absl::OkStatus();
}

absl::Status FromProto(const WheelVectorProto &proto, WheelVector *data_out) {
  data_out->Left() = proto.left();
  data_out->Right() = proto.right();
  return absl::OkStatus();
}

absl::Status ToProto(const WheelVector &data_in, WheelVectorProto *proto_out) {
  proto_out->set_left(data_in.Left());
  proto_out->set_right(data_in.Right());
  return absl::OkStatus();
}

absl::Status FromProto(const KinematicsProto &proto, Kinematics *data_out) {
  WheelVector wheel_radius;
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(
      FromProto(proto.wheel_radius(), &wheel_radius));
  *data_out = Kinematics(wheel_radius, proto.wheel_base());
  return absl::OkStatus();
}

absl::Status ToProto(const Kinematics &data_in, KinematicsProto *proto_out) {
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(
      ToProto(data_in.GetWheelRadius(), proto_out->mutable_wheel_radius()));
  proto_out->set_wheel_base(data_in.GetWheelBase());
  return absl::OkStatus();
}

absl::Status FromProto(const BoxConstraintsProto &proto,
                       BoxConstraints *data_out) {
  WheelVector min_wheel, max_wheel;
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(FromProto(proto.min_wheel(), &min_wheel));
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(FromProto(proto.max_wheel(), &max_wheel));
  ArcVector min_arc, max_arc;
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(FromProto(proto.min_arc(), &min_arc));
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(FromProto(proto.max_arc(), &max_arc));
  *data_out = BoxConstraints(min_wheel, max_wheel, min_arc, max_arc);
  return absl::OkStatus();
}

absl::Status ToProto(const BoxConstraints &data_in,
                     BoxConstraintsProto *proto_out) {
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(
      ToProto(data_in.MinWheelVector(), proto_out->mutable_min_wheel()));
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(
      ToProto(data_in.MaxWheelVector(), proto_out->mutable_max_wheel()));
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(
      ToProto(data_in.MinArcVector(), proto_out->mutable_min_arc()));
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(
      ToProto(data_in.MaxArcVector(), proto_out->mutable_max_arc()));
  return absl::OkStatus();
}

absl::Status FromProto(const DynamicLimitsProto &proto,
                       DynamicLimits *data_out) {
  Kinematics kinematics;
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(
      FromProto(proto.kinematics(), &kinematics));
  BoxConstraints vel_limits, accel_limits;
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(
      FromProto(proto.velocity_limits(), &vel_limits));
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(
      FromProto(proto.acceleration_limits(), &accel_limits));
  *data_out = DynamicLimits(kinematics, vel_limits, accel_limits);
  return absl::OkStatus();
}

absl::Status ToProto(const DynamicLimits &data_in,
                     DynamicLimitsProto *proto_out) {
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(
      ToProto(data_in.GetKinematics(), proto_out->mutable_kinematics()));
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(
      ToProto(data_in.VelocityLimits(), proto_out->mutable_velocity_limits()));
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(ToProto(
      data_in.AccelerationLimits(), proto_out->mutable_acceleration_limits()));
  return absl::OkStatus();
}

absl::Status FromProto(const TrajectoryLimitsProto &proto,
                       TrajectoryLimits *data_out) {
  CHECK_NE(data_out, nullptr);
  WheelVector max_wheel_velocity_jump;
  ArcVector max_arc_velocity_jump;
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(
      FromProto(proto.max_wheel_velocity_jump(), &max_wheel_velocity_jump));
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(
      FromProto(proto.max_arc_velocity_jump(), &max_arc_velocity_jump));
  *data_out = TrajectoryLimits(max_wheel_velocity_jump, max_arc_velocity_jump,
                               proto.min_cycle_duration());
  return absl::OkStatus();
}

absl::Status ToProto(const TrajectoryLimits &data_in,
                     TrajectoryLimitsProto *proto_out) {
  CHECK_NE(proto_out, nullptr);
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(
      ToProto(data_in.GetMaxWheelVelocityJump(),
              proto_out->mutable_max_wheel_velocity_jump()));
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(
      ToProto(data_in.GetMaxArcVelocityJump(),
              proto_out->mutable_max_arc_velocity_jump()));
  proto_out->set_min_cycle_duration(data_in.GetMinCycleDuration());
  return absl::OkStatus();
}

absl::Status FromProto(const CurvePointProto &proto, CurvePoint *data_out) {
  eigenmath::Pose2d pose = eigenmath::conversions::PoseFromProto(proto.pose());
  *data_out = CurvePoint(pose, proto.curvature());
  return absl::OkStatus();
}

absl::Status ToProto(const CurvePoint &data_in, CurvePointProto *proto_out) {
  *proto_out->mutable_pose() =
      eigenmath::conversions::ProtoFromPose(data_in.GetPose());
  proto_out->set_curvature(data_in.GetCurvature());
  return absl::OkStatus();
}

absl::Status FromProto(const CurvePtAndCordProto &proto,
                       CurvePtAndCord *data_out) {
  CurvePoint point;
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(FromProto(proto.point(), &point));
  *data_out = CurvePtAndCord{proto.cord_length(), point};
  return absl::OkStatus();
}

absl::Status ToProto(const CurvePtAndCord &data_in,
                     CurvePtAndCordProto *proto_out) {
  proto_out->set_cord_length(data_in.cord_length);
  return ToProto(data_in.point, proto_out->mutable_point());
}

absl::Status FromProto(const CurveProto &proto, Curve *data_out) {
  if (proto.points_size() < 1) {
    return absl::InvalidArgumentError(
        absl::StrCat("Number of points is too small: ", proto.points_size()));
  }
  Curve tmp_curve(std::max(proto.points_size(), data_out->GetCapacity()));
  for (int i = 0; i < proto.points_size(); ++i) {
    CurvePtAndCord new_pt;
    MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(FromProto(proto.points(i), &new_pt));
    tmp_curve.AddPoint(new_pt.cord_length, new_pt.point);
  }
  *data_out = std::move(tmp_curve);
  return absl::OkStatus();
}

absl::Status ToProto(const Curve &data_in, CurveProto *proto_out) {
  for (auto &pt : data_in.GetCurvePointIteratorRange()) {
    MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(ToProto(pt, proto_out->add_points()));
  }
  return absl::OkStatus();
}

absl::Status FromProto(const PiecewiseLinearCurveProto &proto,
                       Curve *data_out) {
  if (proto.points_size() < 1) {
    return absl::InvalidArgumentError(
        absl::StrCat("Number of points is too small: ", proto.points_size()));
  }
  Curve tmp_curve(std::max(proto.points_size(), data_out->GetCapacity()));
  eigenmath::Vector2d point_i =
      eigenmath::conversions::EigenVectorFromProto(proto.points(0));
  double accum_s = 0.0;
  for (int i = 0; i < proto.points_size(); ++i) {
    if (i + 1 == proto.points_size()) {
      // treat end point differently
      tmp_curve.AddPoint(accum_s, diff_drive::CurvePoint(eigenmath::Pose2d(
                                      point_i, proto.final_angle())));
    } else {
      eigenmath::Vector2d point_i_1 =
          eigenmath::conversions::EigenVectorFromProto(proto.points(i + 1));
      const eigenmath::Vector2d delta = point_i_1 - point_i;
      const double delta_norm = delta.norm();
      if (delta_norm < kEpsilon) {
        continue;
      }
      const double new_angle = std::atan2(delta.y(), delta.x());
      tmp_curve.AddPoint(accum_s, diff_drive::CurvePoint(
                                      eigenmath::Pose2d(point_i, new_angle)));
      accum_s += delta_norm;
      point_i = point_i_1;
    }
  }
  *data_out = std::move(tmp_curve);
  return absl::OkStatus();
}

absl::Status ToProto(const Curve &data_in,
                     PiecewiseLinearCurveProto *proto_out) {
  for (auto &pt : data_in.GetCurvePointIteratorRange()) {
    const auto &translation = pt.point.GetPose().translation();
    *proto_out->add_points() =
        eigenmath::conversions::ProtoFromVector2d(translation);
  }
  proto_out->set_final_angle(data_in.GetFinish().point.GetPose().angle());
  return absl::OkStatus();
}

absl::Status FromProto(const StateProto &proto, State *data_out) {
  eigenmath::Pose2d pose = eigenmath::conversions::PoseFromProto(proto.pose());
  ArcVector arc_vel;
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(
      FromProto(proto.arc_velocity(), &arc_vel));
  *data_out = State(pose, arc_vel);
  return absl::OkStatus();
}

absl::Status ToProto(const State &data_in, StateProto *proto_out) {
  *proto_out->mutable_pose() =
      eigenmath::conversions::ProtoFromPose(data_in.GetPose());
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(
      ToProto(data_in.GetArcVelocity(), proto_out->mutable_arc_velocity()));
  return absl::OkStatus();
}

absl::Status FromProto(const StateAndTimeProto &proto, StateAndTime *data_out) {
  State state;
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(FromProto(proto.state(), &state));
  *data_out = StateAndTime{proto.time(), state};
  return absl::OkStatus();
}

absl::Status ToProto(const StateAndTime &data_in,
                     StateAndTimeProto *proto_out) {
  proto_out->set_time(data_in.time);
  return ToProto(data_in.state, proto_out->mutable_state());
}

absl::Status FromProto(const TrajectoryProto &proto, Trajectory *data_out) {
  Trajectory tmp_traj(std::max(proto.states_size(), data_out->GetCapacity()));
  for (int i = 0; i < proto.states_size(); ++i) {
    StateAndTime new_st;
    MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(FromProto(proto.states(i), &new_st));
    tmp_traj.AddState(new_st.time, new_st.state);
  }
  *data_out = std::move(tmp_traj);
  return absl::OkStatus();
}

absl::Status ToProto(const Trajectory &data_in, TrajectoryProto *proto_out) {
  for (auto &st : data_in.GetStateIteratorRange()) {
    MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(ToProto(st, proto_out->add_states()));
  }
  return absl::OkStatus();
}

absl::Status FromProto(const WheelStateProto &proto, WheelState *data_out) {
  CHECK_NE(data_out, nullptr);
  WheelVector positions;
  WheelVector rates;
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(
      FromProto(proto.wheel_positions(), &positions));
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(
      FromProto(proto.wheel_motion_rates(), &rates));
  data_out->SetPositions(positions);
  data_out->SetMotionRates(rates);
  return absl::OkStatus();
}

absl::Status ToProto(const WheelState &data_in, WheelStateProto *proto_out) {
  CHECK_NE(proto_out, nullptr);
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(
      ToProto(data_in.GetPositions(), proto_out->mutable_wheel_positions()));
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(ToProto(
      data_in.GetMotionRates(), proto_out->mutable_wheel_motion_rates()));
  return absl::OkStatus();
}

absl::Status FromProto(const WheelStateAndTotalMotionProto &proto,
                       WheelStateAndTotalMotion *data_out) {
  CHECK_NE(data_out, nullptr);
  WheelState state;
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(FromProto(proto.state(), &state));
  *data_out = WheelStateAndTotalMotion{proto.total_motion(), state};
  return absl::OkStatus();
}

absl::Status ToProto(const WheelStateAndTotalMotion &data_in,
                     WheelStateAndTotalMotionProto *proto_out) {
  CHECK_NE(proto_out, nullptr);
  MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(
      ToProto(data_in.state, proto_out->mutable_state()));
  proto_out->set_total_motion(data_in.total_motion);
  return absl::OkStatus();
}

absl::Status FromProto(const WheelCurveProto &proto, WheelCurve *data_out) {
  CHECK_NE(data_out, nullptr);
  if (proto.states_size() < 1) {
    return absl::InvalidArgumentError(
        absl::StrCat("Number of states is too small: ", proto.states_size()));
  }
  WheelCurve curve(std::max(proto.states_size(), data_out->GetCapacity()));
  for (int i = 0; i < proto.states_size(); ++i) {
    WheelStateAndTotalMotion state;
    MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(FromProto(proto.states(i), &state));
    curve.AddPoint(state.total_motion, state.state);
  }
  *data_out = std::move(curve);
  return absl::OkStatus();
}

absl::Status ToProto(const WheelCurve &data_in, WheelCurveProto *proto_out) {
  CHECK_NE(proto_out, nullptr);
  for (const auto &state : data_in.GetWheelStateIteratorRange()) {
    MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR(
        ToProto(state, proto_out->add_states()));
  }
  return absl::OkStatus();
}

#undef MOBILITY_DIFF_DRIVE_RETURN_IF_ERROR

}  // namespace mobility::diff_drive
