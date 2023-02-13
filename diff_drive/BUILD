# Copyright 2023 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_proto_library")

licenses(["notice"])

exports_files(["LICENSE"])

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "interval",
    hdrs = [
        "interval.h",
    ],
)

cc_test(
    name = "interval_test",
    srcs = [
        "interval_test.cc",
    ],
    deps = [
        ":interval",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_library(
    name = "differential_drive",
    srcs = [
        "curve.cc",
        "curve_trajectory_utils.cc",
        "dynamic_limits.cc",
        "kinematics.cc",
        "trajectory.cc",
        "trajectory_limits.cc",
        "wheel_curve.cc",
    ],
    hdrs = [
        "continuous_curve.h",
        "curve.h",
        "curve_point.h",
        "curve_trajectory_utils.h",
        "dynamic_limits.h",
        "kinematics.h",
        "state.h",
        "trajectory.h",
        "trajectory_limits.h",
        "transformed_curve_view.h",
        "type_aliases.h",
        "wheel_curve.h",
        "wheel_state.h",
    ],
    deps = [
        ":interval",
        "@eigenmath//eigenmath",
        "@genit//genit:iterators",
        "@com_google_absl//absl/log:check",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_library(
    name = "motion_control",
    srcs = [
        "path_tracker.cc",
        "state_feedback_control.cc",
        "state_tracking_control.cc",
    ],
    hdrs = [
        "path_tracker.h",
        "state_feedback_control.h",
        "state_tracking_control.h",
    ],
    deps = [
        ":base_feedback_control_config_cc_proto",
        ":differential_drive",
        "@eigenmath//eigenmath",
        "@com_google_absl//absl/types:span",
    ],
)

proto_library(
    name = "diff_drive_proto",
    srcs = [
        "diff_drive.proto",
    ],
    deps = [
        "@eigenmath//eigenmath:eigenmath_proto",
    ],
)

cc_proto_library(
    name = "diff_drive_cc_proto",
    deps = [":diff_drive_proto"],
)

cc_library(
    name = "diff_drive_conversion",
    srcs = [
        "diff_drive_conversion.cc",
    ],
    hdrs = [
        "diff_drive_conversion.h",
    ],
    deps = [
        ":diff_drive_cc_proto",
        ":differential_drive",
        "@eigenmath//eigenmath:eigenmath_cc_proto",
        "@eigenmath//eigenmath:conversions",
        "@com_google_absl//absl/log",
        "@com_google_absl//absl/status",
        "@com_google_absl//absl/strings",
    ],
)

cc_test(
    name = "diff_drive_conversion_test",
    srcs = [
        "diff_drive_conversion_test.cc",
    ],
    deps = [
        ":diff_drive_cc_proto",
        ":diff_drive_conversion",
        ":differential_drive",
        "@eigenmath//eigenmath:eigenmath_cc_proto",
        "@com_google_protobuf//:protobuf",
        "@com_google_googletest//:gtest_main",
    ],
)

proto_library(
    name = "base_kinematics_config_proto",
    srcs = [
        "base_kinematics_config.proto",
    ],
    deps = [
        ":diff_drive_proto",
    ],
)

cc_proto_library(
    name = "base_kinematics_config_cc_proto",
    deps = [":base_kinematics_config_proto"],
)

proto_library(
    name = "base_feedback_control_config_proto",
    srcs = [
        "base_feedback_control_config.proto",
    ],
)

cc_proto_library(
    name = "base_feedback_control_config_cc_proto",
    deps = [":base_feedback_control_config_proto"],
)

cc_library(
    name = "test_fixtures",
    testonly = True,
    srcs = [
        "test_curves.cc",
        "test_trajectories.cc",
        "test_wheel_curves.cc",
    ],
    hdrs = [
        "test_curves.h",
        "test_trajectories.h",
        "test_utils.h",
        "test_wheel_curves.h",
    ],
    deps = [
        ":differential_drive",
        "@eigenmath//eigenmath:matchers",
        "@com_google_googletest//:gtest",
    ],
)

cc_library(
    name = "matchers",
    testonly = True,
    hdrs = ["matchers.h"],
    deps = [
        ":differential_drive",
        "@com_google_googletest//:gtest",
    ],
)

cc_test(
    name = "type_aliases_test",
    srcs = [
        "type_aliases_test.cc",
    ],
    deps = [
        ":differential_drive",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "curve_test",
    srcs = [
        "curve_test.cc",
    ],
    deps = [
        ":differential_drive",
        ":test_fixtures",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "curve_trajectory_utils_test",
    srcs = [
        "curve_trajectory_utils_test.cc",
    ],
    deps = [
        ":differential_drive",
        ":matchers",
        ":test_fixtures",
        "@eigenmath//eigenmath:matchers",
        "@eigenmath//eigenmath:sampling",
        "@com_google_googletest//:gtest_main",
        "@com_google_absl//absl/flags:flag",
    ],
)

cc_test(
    name = "dynamic_limits_test",
    srcs = [
        "dynamic_limits_test.cc",
    ],
    deps = [
        ":differential_drive",
        "@com_google_googletest//:gtest_main",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_test(
    name = "kinematics_test",
    srcs = [
        "kinematics_test.cc",
    ],
    deps = [
        ":differential_drive",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "state_test",
    srcs = [
        "state_test.cc",
    ],
    deps = [
        ":differential_drive",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "trajectory_test",
    srcs = [
        "trajectory_test.cc",
    ],
    deps = [
        ":differential_drive",
        ":test_fixtures",
        "@eigenmath//eigenmath:matchers",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "trajectory_limits_test",
    srcs = [
        "trajectory_limits_test.cc",
    ],
    deps = [
        ":differential_drive",
        "@eigenmath//eigenmath",
        "@eigenmath//eigenmath:matchers",
        "@eigenmath//eigenmath:sampling",
        "@genit//genit:iterators",
        "@com_google_benchmark//:benchmark",
        "@com_google_googletest//:gtest_main",
        "@com_google_absl//absl/random:distributions",
        "@com_google_absl//absl/strings:str_format",
    ],
)

cc_test(
    name = "transformed_curve_view_test",
    srcs = [
        "transformed_curve_view_test.cc",
    ],
    deps = [
        ":differential_drive",
        ":test_fixtures",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "wheel_curve_test",
    srcs = [
        "wheel_curve_test.cc",
    ],
    deps = [
        ":differential_drive",
        ":test_fixtures",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "path_tracker_test",
    srcs = [
        "path_tracker_test.cc",
    ],
    deps = [
        ":motion_control",
        "@eigenmath//eigenmath:matchers",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "state_feedback_control_test",
    srcs = [
        "state_feedback_control_test.cc",
    ],
    deps = [
        ":differential_drive",
        ":motion_control",
        "@eigenmath//eigenmath:matchers",
        "@com_google_googletest//:gtest_main",
    ],
)

cc_test(
    name = "state_tracking_control_test",
    srcs = [
        "state_tracking_control_test.cc",
    ],
    deps = [
        ":differential_drive",
        ":motion_control",
        "@eigenmath//eigenmath:matchers",
        "@com_google_googletest//:gtest_main",
    ],
)
