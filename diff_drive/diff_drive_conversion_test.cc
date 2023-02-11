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

#include <cmath>
#include <string>

#include "eigenmath/eigenmath.pb.h"
#include "diff_drive/curve.h"
#include "diff_drive/curve_point.h"
#include "diff_drive/diff_drive.pb.h"
#include "diff_drive/dynamic_limits.h"
#include "diff_drive/kinematics.h"
#include "diff_drive/state.h"
#include "diff_drive/trajectory.h"
#include "diff_drive/trajectory_limits.h"
#include "diff_drive/wheel_curve.h"
#include "diff_drive/wheel_state.h"
#include "google/protobuf/text_format.h"
#include "google/protobuf/util/message_differencer.h"
#include "gmock/gmock.h"
#include "gtest/gtest.h"

namespace mobility::diff_drive {
namespace {

constexpr double kEpsilon = 1e-6;

TEST(DiffDriveConversion, ArcVector) {
  ArcVectorProto proto;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(R"""(
    translation: 2.0
    rotation: 3.0
    )""", &proto));

  ArcVector data_out;
  ASSERT_TRUE(FromProto(proto, &data_out).ok());
  EXPECT_NEAR(data_out.Translation(), 2.0, kEpsilon);
  EXPECT_NEAR(data_out.Rotation(), 3.0, kEpsilon);

  ArcVectorProto proto_out;
  ASSERT_TRUE(ToProto(data_out, &proto_out).ok());
  EXPECT_NEAR(proto_out.translation(), 2.0, kEpsilon);
  EXPECT_NEAR(proto_out.rotation(), 3.0, kEpsilon);
}

TEST(DiffDriveConversion, WheelVector) {
  WheelVectorProto proto;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(R"""(
    left: 2.0
    right: 3.0
    )""", &proto));

  WheelVector data_out;
  ASSERT_TRUE(FromProto(proto, &data_out).ok());
  EXPECT_NEAR(data_out.Left(), 2.0, kEpsilon);
  EXPECT_NEAR(data_out.Right(), 3.0, kEpsilon);

  WheelVectorProto proto_out;
  ASSERT_TRUE(ToProto(data_out, &proto_out).ok());
  EXPECT_NEAR(proto_out.left(), 2.0, kEpsilon);
  EXPECT_NEAR(proto_out.right(), 3.0, kEpsilon);
}

TEST(DiffDriveConversion, Kinematics) {
  KinematicsProto proto;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(R"""(
    wheel_radius {
      left: 2.0
      right: 3.0
    }
    wheel_base: 1.0
    )""", &proto));

  Kinematics data_out;
  ASSERT_TRUE(FromProto(proto, &data_out).ok());
  EXPECT_NEAR(data_out.GetWheelBase(), 1.0, kEpsilon);
  EXPECT_NEAR(data_out.GetWheelRadius().Left(), 2.0, kEpsilon);
  EXPECT_NEAR(data_out.GetWheelRadius().Right(), 3.0, kEpsilon);

  KinematicsProto proto_out;
  ASSERT_TRUE(ToProto(data_out, &proto_out).ok());
  EXPECT_NEAR(proto_out.wheel_base(), 1.0, kEpsilon);
  EXPECT_NEAR(proto_out.wheel_radius().left(), 2.0, kEpsilon);
  EXPECT_NEAR(proto_out.wheel_radius().right(), 3.0, kEpsilon);
}

TEST(DiffDriveConversion, BoxConstraints) {
  BoxConstraintsProto proto;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(R"""(
    min_wheel {
      left: -2.0
      right: -3.0
    }
    max_wheel {
      left: 4.0
      right: 5.0
    }
    min_arc {
      translation: -0.5
      rotation: -1.0
    }
    max_arc {
      translation: 0.25
      rotation: 0.75
    }
    )""", &proto));

  BoxConstraints data_out;
  ASSERT_TRUE(FromProto(proto, &data_out).ok());
  EXPECT_NEAR(data_out.MinWheelVector().Left(), -2.0, kEpsilon);
  EXPECT_NEAR(data_out.MinWheelVector().Right(), -3.0, kEpsilon);
  EXPECT_NEAR(data_out.MaxWheelVector().Left(), 4.0, kEpsilon);
  EXPECT_NEAR(data_out.MaxWheelVector().Right(), 5.0, kEpsilon);
  EXPECT_NEAR(data_out.MinArcVector().Translation(), -0.5, kEpsilon);
  EXPECT_NEAR(data_out.MinArcVector().Rotation(), -1.0, kEpsilon);
  EXPECT_NEAR(data_out.MaxArcVector().Translation(), 0.25, kEpsilon);
  EXPECT_NEAR(data_out.MaxArcVector().Rotation(), 0.75, kEpsilon);

  BoxConstraintsProto proto_out;
  ASSERT_TRUE(ToProto(data_out, &proto_out).ok());
  EXPECT_NEAR(proto_out.min_wheel().left(), -2.0, kEpsilon);
  EXPECT_NEAR(proto_out.min_wheel().right(), -3.0, kEpsilon);
  EXPECT_NEAR(proto_out.max_wheel().left(), 4.0, kEpsilon);
  EXPECT_NEAR(proto_out.max_wheel().right(), 5.0, kEpsilon);
  EXPECT_NEAR(proto_out.min_arc().translation(), -0.5, kEpsilon);
  EXPECT_NEAR(proto_out.min_arc().rotation(), -1.0, kEpsilon);
  EXPECT_NEAR(proto_out.max_arc().translation(), 0.25, kEpsilon);
  EXPECT_NEAR(proto_out.max_arc().rotation(), 0.75, kEpsilon);
}

TEST(DiffDriveConversion, DynamicLimits) {
  DynamicLimitsProto proto;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(R"""(
    kinematics {
      wheel_radius {
        left: 2.0
        right: 3.0
      }
      wheel_base: 1.0
    }
    velocity_limits {
      min_wheel {
        left: -2.0
        right: -3.0
      }
      max_wheel {
        left: 4.0
        right: 5.0
      }
      min_arc {
        translation: -0.5
        rotation: -1.0
      }
      max_arc {
        translation: 0.25
        rotation: 0.75
      }
    }
    acceleration_limits {
      min_wheel {
        left: -20.0
        right: -30.0
      }
      max_wheel {
        left: 40.0
        right: 50.0
      }
      min_arc {
        translation: -5.0
        rotation: -10.0
      }
      max_arc {
        translation: 2.5
        rotation: 7.5
      }
    }
    )""", &proto));

  DynamicLimits data_out;
  ASSERT_TRUE(FromProto(proto, &data_out).ok());
  EXPECT_NEAR(data_out.GetKinematics().GetWheelBase(), 1.0, kEpsilon);
  EXPECT_NEAR(data_out.GetKinematics().GetWheelRadius().Left(), 2.0, kEpsilon);
  EXPECT_NEAR(data_out.GetKinematics().GetWheelRadius().Right(), 3.0, kEpsilon);

  EXPECT_NEAR(data_out.VelocityLimits().MinWheelVector().Left(), -2.0,
              kEpsilon);
  EXPECT_NEAR(data_out.VelocityLimits().MinWheelVector().Right(), -3.0,
              kEpsilon);
  EXPECT_NEAR(data_out.VelocityLimits().MaxWheelVector().Left(), 4.0, kEpsilon);
  EXPECT_NEAR(data_out.VelocityLimits().MaxWheelVector().Right(), 5.0,
              kEpsilon);
  EXPECT_NEAR(data_out.VelocityLimits().MinArcVector().Translation(), -0.5,
              kEpsilon);
  EXPECT_NEAR(data_out.VelocityLimits().MinArcVector().Rotation(), -1.0,
              kEpsilon);
  EXPECT_NEAR(data_out.VelocityLimits().MaxArcVector().Translation(), 0.25,
              kEpsilon);
  EXPECT_NEAR(data_out.VelocityLimits().MaxArcVector().Rotation(), 0.75,
              kEpsilon);

  EXPECT_NEAR(data_out.AccelerationLimits().MinWheelVector().Left(), -20.0,
              kEpsilon);
  EXPECT_NEAR(data_out.AccelerationLimits().MinWheelVector().Right(), -30.0,
              kEpsilon);
  EXPECT_NEAR(data_out.AccelerationLimits().MaxWheelVector().Left(), 40.0,
              kEpsilon);
  EXPECT_NEAR(data_out.AccelerationLimits().MaxWheelVector().Right(), 50.0,
              kEpsilon);
  EXPECT_NEAR(data_out.AccelerationLimits().MinArcVector().Translation(), -5.0,
              kEpsilon);
  EXPECT_NEAR(data_out.AccelerationLimits().MinArcVector().Rotation(), -10.0,
              kEpsilon);
  EXPECT_NEAR(data_out.AccelerationLimits().MaxArcVector().Translation(), 2.5,
              kEpsilon);
  EXPECT_NEAR(data_out.AccelerationLimits().MaxArcVector().Rotation(), 7.5,
              kEpsilon);

  DynamicLimitsProto proto_out;
  ASSERT_TRUE(ToProto(data_out, &proto_out).ok());
  EXPECT_NEAR(proto_out.kinematics().wheel_base(), 1.0, kEpsilon);
  EXPECT_NEAR(proto_out.kinematics().wheel_radius().left(), 2.0, kEpsilon);
  EXPECT_NEAR(proto_out.kinematics().wheel_radius().right(), 3.0, kEpsilon);

  EXPECT_NEAR(proto_out.velocity_limits().min_wheel().left(), -2.0, kEpsilon);
  EXPECT_NEAR(proto_out.velocity_limits().min_wheel().right(), -3.0, kEpsilon);
  EXPECT_NEAR(proto_out.velocity_limits().max_wheel().left(), 4.0, kEpsilon);
  EXPECT_NEAR(proto_out.velocity_limits().max_wheel().right(), 5.0, kEpsilon);
  EXPECT_NEAR(proto_out.velocity_limits().min_arc().translation(), -0.5,
              kEpsilon);
  EXPECT_NEAR(proto_out.velocity_limits().min_arc().rotation(), -1.0, kEpsilon);
  EXPECT_NEAR(proto_out.velocity_limits().max_arc().translation(), 0.25,
              kEpsilon);
  EXPECT_NEAR(proto_out.velocity_limits().max_arc().rotation(), 0.75, kEpsilon);

  EXPECT_NEAR(proto_out.acceleration_limits().min_wheel().left(), -20.0,
              kEpsilon);
  EXPECT_NEAR(proto_out.acceleration_limits().min_wheel().right(), -30.0,
              kEpsilon);
  EXPECT_NEAR(proto_out.acceleration_limits().max_wheel().left(), 40.0,
              kEpsilon);
  EXPECT_NEAR(proto_out.acceleration_limits().max_wheel().right(), 50.0,
              kEpsilon);
  EXPECT_NEAR(proto_out.acceleration_limits().min_arc().translation(), -5.0,
              kEpsilon);
  EXPECT_NEAR(proto_out.acceleration_limits().min_arc().rotation(), -10.0,
              kEpsilon);
  EXPECT_NEAR(proto_out.acceleration_limits().max_arc().translation(), 2.5,
              kEpsilon);
  EXPECT_NEAR(proto_out.acceleration_limits().max_arc().rotation(), 7.5,
              kEpsilon);
}

TEST(DiffDriveConversion, TrajectoryLimits) {
  TrajectoryLimitsProto proto_in;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(R"""(
    max_wheel_velocity_jump {
      left: 1.23
      right: 4.23
    }
    max_arc_velocity_jump {
      translation: 45.2
      rotation: -2.1
    }
    min_cycle_duration: 0.02
    )""", &proto_in));

  TrajectoryLimits data_out;
  ASSERT_TRUE(FromProto(proto_in, &data_out).ok());
  TrajectoryLimitsProto proto_out;
  ASSERT_TRUE(ToProto(data_out, &proto_out).ok());

  EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equivalent(proto_in, proto_out));
}

TEST(DiffDriveConversion, CurvePoint) {
  CurvePointProto proto;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(R"""(
    pose {
      tx: 2.0
      ty: 3.0
      rotation: 1.57
    }
    curvature: 1.0
    )""", &proto));

  CurvePoint data_out;
  ASSERT_TRUE(FromProto(proto, &data_out).ok());
  EXPECT_NEAR(data_out.GetCurvature(), 1.0, kEpsilon);
  EXPECT_NEAR(data_out.GetPose().translation().x(), 2.0, kEpsilon);
  EXPECT_NEAR(data_out.GetPose().translation().y(), 3.0, kEpsilon);
  EXPECT_NEAR(data_out.GetPose().angle(), 1.57, kEpsilon);

  CurvePointProto proto_out;
  ASSERT_TRUE(ToProto(data_out, &proto_out).ok());
  EXPECT_NEAR(proto_out.curvature(), 1.0, kEpsilon);
  EXPECT_NEAR(proto_out.pose().tx(), 2.0, kEpsilon);
  EXPECT_NEAR(proto_out.pose().ty(), 3.0, kEpsilon);
  EXPECT_NEAR(proto_out.pose().rotation(), 1.57, kEpsilon);
}

TEST(DiffDriveConversion, CurvePtAndCord) {
  CurvePtAndCordProto proto;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(R"""(
    point {
      pose {
        tx: 2.0
        ty: 3.0
        rotation: 1.57
      }
      curvature: 1.0
    }
    cord_length: 5.0
    )""", &proto));

  CurvePtAndCord data_out;
  ASSERT_TRUE(FromProto(proto, &data_out).ok());
  EXPECT_NEAR(data_out.cord_length, 5.0, kEpsilon);
  EXPECT_NEAR(data_out.point.GetCurvature(), 1.0, kEpsilon);
  EXPECT_NEAR(data_out.point.GetPose().translation().x(), 2.0, kEpsilon);
  EXPECT_NEAR(data_out.point.GetPose().translation().y(), 3.0, kEpsilon);
  EXPECT_NEAR(data_out.point.GetPose().angle(), 1.57, kEpsilon);

  CurvePtAndCordProto proto_out;
  ASSERT_TRUE(ToProto(data_out, &proto_out).ok());
  EXPECT_NEAR(proto_out.cord_length(), 5.0, kEpsilon);
  EXPECT_NEAR(proto_out.point().curvature(), 1.0, kEpsilon);
  EXPECT_NEAR(proto_out.point().pose().tx(), 2.0, kEpsilon);
  EXPECT_NEAR(proto_out.point().pose().ty(), 3.0, kEpsilon);
  EXPECT_NEAR(proto_out.point().pose().rotation(), 1.57, kEpsilon);
}

TEST(DiffDriveConversion, Curve) {
  CurveProto proto;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(R"""(
points {
  point {
    pose {
      tx: 0.0
      ty: 0.0
      rotation: 0.4636476
    }
    curvature: 0.0
  }
  cord_length: 0.0
}
points {
  point {
    pose {
      tx: 2.0
      ty: 1.0
      rotation: 3.1415926
    }
    curvature: 0.0
  }
  cord_length: 2.236068
}
points {
  point {
    pose {
      tx: -2.0
      ty: 1.0
      rotation: -1.57
    }
    curvature: 0.0
  }
  cord_length: 6.236068
}
)""", &proto));
  ASSERT_NE(0, proto.points_size());

  Curve data_out(21);
  ASSERT_TRUE(FromProto(proto, &data_out).ok());
  EXPECT_EQ(data_out.GetCapacity(), 21);
  ASSERT_EQ(data_out.GetSize(), 3);
  auto pt_it = data_out.BeginCurvePoint();
  auto pt0 = *pt_it;
  ++pt_it;
  auto pt1 = *pt_it;
  ++pt_it;
  auto pt2 = *pt_it;
  ++pt_it;

  EXPECT_NEAR(pt0.cord_length, 0.0, kEpsilon);
  EXPECT_NEAR(pt0.point.GetPose().translation().x(), 0.0, kEpsilon);
  EXPECT_NEAR(pt0.point.GetPose().translation().y(), 0.0, kEpsilon);
  EXPECT_NEAR(pt0.point.GetPose().angle(), std::atan2(1.0, 2.0), kEpsilon);
  EXPECT_NEAR(pt0.point.GetCurvature(), 0.0, kEpsilon);

  EXPECT_NEAR(pt1.cord_length, std::sqrt(5.0), kEpsilon);
  EXPECT_NEAR(pt1.point.GetPose().translation().x(), 2.0, kEpsilon);
  EXPECT_NEAR(pt1.point.GetPose().translation().y(), 1.0, kEpsilon);
  EXPECT_NEAR(pt1.point.GetPose().angle(), std::atan2(0.0, -4.0), kEpsilon);
  EXPECT_NEAR(pt1.point.GetCurvature(), 0.0, kEpsilon);

  EXPECT_NEAR(pt2.cord_length, std::sqrt(5.0) + 4.0, kEpsilon);
  EXPECT_NEAR(pt2.point.GetPose().translation().x(), -2.0, kEpsilon);
  EXPECT_NEAR(pt2.point.GetPose().translation().y(), 1.0, kEpsilon);
  EXPECT_NEAR(pt2.point.GetPose().angle(), -1.57, kEpsilon);
  EXPECT_NEAR(pt2.point.GetCurvature(), 0.0, kEpsilon);

  CurveProto proto_out;
  ASSERT_TRUE(ToProto(data_out, &proto_out).ok());
  EXPECT_EQ(proto_out.points_size(), 3);

  EXPECT_NEAR(proto_out.points(0).cord_length(), 0.0, kEpsilon);
  EXPECT_NEAR(proto_out.points(0).point().pose().tx(), 0.0, kEpsilon);
  EXPECT_NEAR(proto_out.points(0).point().pose().ty(), 0.0, kEpsilon);
  EXPECT_NEAR(proto_out.points(0).point().pose().rotation(),
              std::atan2(1.0, 2.0), kEpsilon);
  EXPECT_NEAR(proto_out.points(0).point().curvature(), 0.0, kEpsilon);

  EXPECT_NEAR(proto_out.points(1).cord_length(), std::sqrt(5.0), kEpsilon);
  EXPECT_NEAR(proto_out.points(1).point().pose().tx(), 2.0, kEpsilon);
  EXPECT_NEAR(proto_out.points(1).point().pose().ty(), 1.0, kEpsilon);
  EXPECT_NEAR(proto_out.points(1).point().pose().rotation(),
              std::atan2(0.0, -4.0), kEpsilon);
  EXPECT_NEAR(proto_out.points(1).point().curvature(), 0.0, kEpsilon);

  EXPECT_NEAR(proto_out.points(2).cord_length(), std::sqrt(5.0) + 4.0,
              kEpsilon);
  EXPECT_NEAR(proto_out.points(2).point().pose().tx(), -2.0, kEpsilon);
  EXPECT_NEAR(proto_out.points(2).point().pose().ty(), 1.0, kEpsilon);
  EXPECT_NEAR(proto_out.points(2).point().pose().rotation(), -1.57, kEpsilon);
  EXPECT_NEAR(proto_out.points(2).point().curvature(), 0.0, kEpsilon);
}

TEST(DiffDriveConversion, PiecewiseLinearCurve) {
  PiecewiseLinearCurveProto proto;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(R"""(
points {
  vec: 0.0
  vec: 0.0
}
points {
  vec: 2.0
  vec: 1.0
}
points {
  vec: -2.0
  vec: 1.0
}
final_angle: -1.57
)""", &proto));
  ASSERT_NE(0, proto.points_size());

  Curve data_out(21);
  ASSERT_TRUE(FromProto(proto, &data_out).ok());
  EXPECT_EQ(data_out.GetCapacity(), 21);
  ASSERT_EQ(data_out.GetSize(), 3);
  auto pt_it = data_out.BeginCurvePoint();
  auto pt0 = *pt_it;
  ++pt_it;
  auto pt1 = *pt_it;
  ++pt_it;
  auto pt2 = *pt_it;
  ++pt_it;

  EXPECT_NEAR(pt0.cord_length, 0.0, kEpsilon);
  EXPECT_NEAR(pt0.point.GetPose().translation().x(), 0.0, kEpsilon);
  EXPECT_NEAR(pt0.point.GetPose().translation().y(), 0.0, kEpsilon);
  EXPECT_NEAR(pt0.point.GetPose().angle(), std::atan2(1.0, 2.0), kEpsilon);
  EXPECT_NEAR(pt0.point.GetCurvature(), 0.0, kEpsilon);

  EXPECT_NEAR(pt1.cord_length, std::sqrt(5.0), kEpsilon);
  EXPECT_NEAR(pt1.point.GetPose().translation().x(), 2.0, kEpsilon);
  EXPECT_NEAR(pt1.point.GetPose().translation().y(), 1.0, kEpsilon);
  EXPECT_NEAR(pt1.point.GetPose().angle(), std::atan2(0.0, -4.0), kEpsilon);
  EXPECT_NEAR(pt1.point.GetCurvature(), 0.0, kEpsilon);

  EXPECT_NEAR(pt2.cord_length, std::sqrt(5.0) + 4.0, kEpsilon);
  EXPECT_NEAR(pt2.point.GetPose().translation().x(), -2.0, kEpsilon);
  EXPECT_NEAR(pt2.point.GetPose().translation().y(), 1.0, kEpsilon);
  EXPECT_NEAR(pt2.point.GetPose().angle(), -1.57, kEpsilon);
  EXPECT_NEAR(pt2.point.GetCurvature(), 0.0, kEpsilon);

  PiecewiseLinearCurveProto proto_out;
  ASSERT_TRUE(ToProto(data_out, &proto_out).ok());
  EXPECT_EQ(proto_out.points_size(), 3);
  EXPECT_NEAR(proto_out.final_angle(), -1.57, kEpsilon);

  EXPECT_NEAR(proto_out.points(0).vec(0), 0.0, kEpsilon);
  EXPECT_NEAR(proto_out.points(0).vec(1), 0.0, kEpsilon);

  EXPECT_NEAR(proto_out.points(1).vec(0), 2.0, kEpsilon);
  EXPECT_NEAR(proto_out.points(1).vec(1), 1.0, kEpsilon);

  EXPECT_NEAR(proto_out.points(2).vec(0), -2.0, kEpsilon);
  EXPECT_NEAR(proto_out.points(2).vec(1), 1.0, kEpsilon);
}

TEST(DiffDriveConversion, State) {
  StateProto proto;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(R"""(
    pose {
      tx: 2.0
      ty: 3.0
      rotation: 1.57
    }
    arc_velocity {
      translation: 4.0
      rotation: 5.0
    }
    )""", &proto));

  State data_out;
  ASSERT_TRUE(FromProto(proto, &data_out).ok());
  EXPECT_NEAR(data_out.GetPose().translation().x(), 2.0, kEpsilon);
  EXPECT_NEAR(data_out.GetPose().translation().y(), 3.0, kEpsilon);
  EXPECT_NEAR(data_out.GetPose().angle(), 1.57, kEpsilon);
  EXPECT_NEAR(data_out.GetArcVelocity().Translation(), 4.0, kEpsilon);
  EXPECT_NEAR(data_out.GetArcVelocity().Rotation(), 5.0, kEpsilon);

  StateProto proto_out;
  ASSERT_TRUE(ToProto(data_out, &proto_out).ok());
  EXPECT_NEAR(proto_out.pose().tx(), 2.0, kEpsilon);
  EXPECT_NEAR(proto_out.pose().ty(), 3.0, kEpsilon);
  EXPECT_NEAR(proto_out.pose().rotation(), 1.57, kEpsilon);
  EXPECT_NEAR(proto_out.arc_velocity().translation(), 4.0, kEpsilon);
  EXPECT_NEAR(proto_out.arc_velocity().rotation(), 5.0, kEpsilon);
}

TEST(DiffDriveConversion, StateAndTime) {
  StateAndTimeProto proto;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(R"""(
    state {
      pose {
        tx: 2.0
        ty: 3.0
        rotation: 1.57
      }
      arc_velocity {
        translation: 4.0
        rotation: 5.0
      }
    }
    time: 1.0
    )""", &proto));

  StateAndTime data_out;
  ASSERT_TRUE(FromProto(proto, &data_out).ok());
  EXPECT_NEAR(data_out.time, 1.0, kEpsilon);
  EXPECT_NEAR(data_out.state.GetPose().translation().x(), 2.0, kEpsilon);
  EXPECT_NEAR(data_out.state.GetPose().translation().y(), 3.0, kEpsilon);
  EXPECT_NEAR(data_out.state.GetPose().angle(), 1.57, kEpsilon);
  EXPECT_NEAR(data_out.state.GetArcVelocity().Translation(), 4.0, kEpsilon);
  EXPECT_NEAR(data_out.state.GetArcVelocity().Rotation(), 5.0, kEpsilon);

  StateAndTimeProto proto_out;
  ASSERT_TRUE(ToProto(data_out, &proto_out).ok());
  EXPECT_NEAR(proto_out.time(), 1.0, kEpsilon);
  EXPECT_NEAR(proto_out.state().pose().tx(), 2.0, kEpsilon);
  EXPECT_NEAR(proto_out.state().pose().ty(), 3.0, kEpsilon);
  EXPECT_NEAR(proto_out.state().pose().rotation(), 1.57, kEpsilon);
  EXPECT_NEAR(proto_out.state().arc_velocity().translation(), 4.0, kEpsilon);
  EXPECT_NEAR(proto_out.state().arc_velocity().rotation(), 5.0, kEpsilon);
}

TEST(DiffDriveConversion, Trajectory) {
  TrajectoryProto proto;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(R"""(
states {
  state {
    pose {
        tx: 0.0
        ty: 0.0
      rotation: 0.4636476
    }
    arc_velocity {
      translation: 1.0
      rotation: 0.0
    }
  }
  time: 0.0
}
states {
  state {
    pose {
        tx: 2.0
        ty: 1.0
      rotation: 3.1415926
    }
    arc_velocity {
      translation: 1.0
      rotation: 0.0
    }
  }
  time: 2.236068
}
states {
  state {
    pose {
        tx: -2.0
        ty: 1.0
      rotation: -1.57
    }
    arc_velocity {
      translation: 1.0
      rotation: 0.0
    }
  }
  time: 6.236068
}
)""", &proto));
  ASSERT_NE(0, proto.states_size());

  Trajectory data_out(21);
  ASSERT_TRUE(FromProto(proto, &data_out).ok());
  EXPECT_EQ(data_out.GetCapacity(), 21);
  ASSERT_EQ(data_out.GetSize(), 3);
  auto pt_it = data_out.BeginState();
  auto pt0 = *pt_it;
  ++pt_it;
  auto pt1 = *pt_it;
  ++pt_it;
  auto pt2 = *pt_it;
  ++pt_it;

  EXPECT_NEAR(pt0.time, 0.0, kEpsilon);
  EXPECT_NEAR(pt0.state.GetPose().translation().x(), 0.0, kEpsilon);
  EXPECT_NEAR(pt0.state.GetPose().translation().y(), 0.0, kEpsilon);
  EXPECT_NEAR(pt0.state.GetPose().angle(), std::atan2(1.0, 2.0), kEpsilon);
  EXPECT_NEAR(pt0.state.GetArcVelocity().Translation(), 1.0, kEpsilon);
  EXPECT_NEAR(pt0.state.GetArcVelocity().Rotation(), 0.0, kEpsilon);

  EXPECT_NEAR(pt1.time, std::sqrt(5.0), kEpsilon);
  EXPECT_NEAR(pt1.state.GetPose().translation().x(), 2.0, kEpsilon);
  EXPECT_NEAR(pt1.state.GetPose().translation().y(), 1.0, kEpsilon);
  EXPECT_NEAR(pt1.state.GetPose().angle(), std::atan2(0.0, -4.0), kEpsilon);
  EXPECT_NEAR(pt1.state.GetArcVelocity().Translation(), 1.0, kEpsilon);
  EXPECT_NEAR(pt1.state.GetArcVelocity().Rotation(), 0.0, kEpsilon);

  EXPECT_NEAR(pt2.time, std::sqrt(5.0) + 4.0, kEpsilon);
  EXPECT_NEAR(pt2.state.GetPose().translation().x(), -2.0, kEpsilon);
  EXPECT_NEAR(pt2.state.GetPose().translation().y(), 1.0, kEpsilon);
  EXPECT_NEAR(pt2.state.GetPose().angle(), -1.57, kEpsilon);
  EXPECT_NEAR(pt2.state.GetArcVelocity().Translation(), 1.0, kEpsilon);
  EXPECT_NEAR(pt2.state.GetArcVelocity().Rotation(), 0.0, kEpsilon);

  TrajectoryProto proto_out;
  ASSERT_TRUE(ToProto(data_out, &proto_out).ok());
  EXPECT_EQ(proto_out.states_size(), 3);

  EXPECT_NEAR(proto_out.states(0).time(), 0.0, kEpsilon);
  EXPECT_NEAR(proto_out.states(0).state().pose().tx(), 0.0, kEpsilon);
  EXPECT_NEAR(proto_out.states(0).state().pose().ty(), 0.0, kEpsilon);
  EXPECT_NEAR(proto_out.states(0).state().pose().rotation(),
              std::atan2(1.0, 2.0), kEpsilon);
  EXPECT_NEAR(proto_out.states(0).state().arc_velocity().translation(), 1.0,
              kEpsilon);
  EXPECT_NEAR(proto_out.states(0).state().arc_velocity().rotation(), 0.0,
              kEpsilon);

  EXPECT_NEAR(proto_out.states(1).time(), std::sqrt(5.0), kEpsilon);
  EXPECT_NEAR(proto_out.states(1).state().pose().tx(), 2.0, kEpsilon);
  EXPECT_NEAR(proto_out.states(1).state().pose().ty(), 1.0, kEpsilon);
  EXPECT_NEAR(proto_out.states(1).state().pose().rotation(),
              std::atan2(0.0, -4.0), kEpsilon);
  EXPECT_NEAR(proto_out.states(1).state().arc_velocity().translation(), 1.0,
              kEpsilon);
  EXPECT_NEAR(proto_out.states(1).state().arc_velocity().rotation(), 0.0,
              kEpsilon);

  EXPECT_NEAR(proto_out.states(2).time(), std::sqrt(5.0) + 4.0, kEpsilon);
  EXPECT_NEAR(proto_out.states(2).state().pose().tx(), -2.0, kEpsilon);
  EXPECT_NEAR(proto_out.states(2).state().pose().ty(), 1.0, kEpsilon);
  EXPECT_NEAR(proto_out.states(2).state().pose().rotation(), -1.57, kEpsilon);
  EXPECT_NEAR(proto_out.states(2).state().arc_velocity().translation(), 1.0,
              kEpsilon);
  EXPECT_NEAR(proto_out.states(2).state().arc_velocity().rotation(), 0.0,
              kEpsilon);
}

TEST(DiffDriveConversion, WheelState) {
  WheelStateProto proto_in;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(R"""(
    wheel_positions {
      left: 2.0
      right: 3.0
    }
    wheel_motion_rates {
      left: 4.0
      right: 5.0
    }
    )""", &proto_in));

  WheelState data_out;
  ASSERT_TRUE(FromProto(proto_in, &data_out).ok());
  WheelStateProto proto_out;
  ASSERT_TRUE(ToProto(data_out, &proto_out).ok());

  EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equivalent(proto_in, proto_out));
}

TEST(DiffDriveConversion, WheelStateAndTotalMotion) {
  WheelStateAndTotalMotionProto proto_in;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(R"""(
    total_motion: 1.234
    state {
      wheel_positions {
        left: 2.0
        right: 3.0
      }
      wheel_motion_rates {
        left: 4.0
        right: 5.0
      }
    }
    )""", &proto_in));

  WheelStateAndTotalMotion data_out;
  ASSERT_TRUE(FromProto(proto_in, &data_out).ok());
  WheelStateAndTotalMotionProto proto_out;
  ASSERT_TRUE(ToProto(data_out, &proto_out).ok());

  EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equivalent(proto_in, proto_out));
}

TEST(DiffDriveConversion, WheelCurve) {
  WheelCurveProto proto_in;
  ASSERT_TRUE(google::protobuf::TextFormat::ParseFromString(R"""(
    states {
      total_motion: 1.234
        state {
          wheel_positions {
            left: 2.0
            right: 3.0
          }
          wheel_motion_rates {
            left: 4.0
            right: 5.0
          }
        }
    }
    states {
      total_motion: 5.234
        state {
          wheel_positions {
            left: -2.0
            right: -3.0
          }
          wheel_motion_rates {
            left: -4.0
            right: -5.0
          }
        }
    }
    states {
      total_motion: 12.234
        state {
          wheel_positions {
            left: 12.0
            right: 35.0
          }
          wheel_motion_rates {
            left: 42.0
            right: 55.0
          }
        }
      }
    )""", &proto_in));

  WheelCurve data_out(21);
  ASSERT_TRUE(FromProto(proto_in, &data_out).ok());
  EXPECT_EQ(data_out.GetCapacity(), 21);
  ASSERT_EQ(data_out.GetSize(), 3);
  WheelCurveProto proto_out;
  ASSERT_TRUE(ToProto(data_out, &proto_out).ok());

  EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equivalent(proto_in, proto_out));
}

}  // namespace
}  // namespace mobility::diff_drive
