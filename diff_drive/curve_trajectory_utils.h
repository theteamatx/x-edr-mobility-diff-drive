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

#ifndef MOBILITY_DIFF_DRIVE_DIFF_DRIVE_CURVE_TRAJECTORY_UTILS_H_
#define MOBILITY_DIFF_DRIVE_DIFF_DRIVE_CURVE_TRAJECTORY_UTILS_H_

#include <iosfwd>
#include <ostream>
#include <string>

#include "eigenmath/quintic_spline.h"
#include "diff_drive/curve.h"
#include "diff_drive/dynamic_limits.h"
#include "diff_drive/kinematics.h"
#include "diff_drive/trajectory.h"
#include "diff_drive/trajectory_limits.h"
#include "diff_drive/type_aliases.h"
#include "diff_drive/wheel_curve.h"

namespace mobility::diff_drive {

// Converts a trajectory to a curve. This is a pretty straight-forward
// conversion, which mainly involves re-parameterizing from time to cord-length.
// If there are pure rotation stages in the trajectory, then they will be
// converted to instantaneous jumps in orientation in the curve (i.e., hard
// corners).
// Except for skipped pure-spins, the output curve will have the same number
// of points as the input trajectory.
bool ConvertTrajectoryToCurve(const Trajectory& traj, Curve* curve);

// Converts a curve to a trajectory with a given constant speed along it.
// This mainly involves re-parameterizing from cord-length to time. The
// rotational velocity is computed along the trajectory so as to respect
// the curvature of the curve and maintain the given tangential speed.
// If there are instantaneous jumps in orientation in the curve, then they
// will be converted to pure rotation stages in the trajectory (i.e., hard
// corners).
// Except for added pure-spins, the output trajectory will have the same number
// of points as the input curve.
bool ConvertCurveToTrajectory(const Curve& curve, double desired_speed,
                              const Kinematics& kinematics, Trajectory* traj);

// Same as above, but with dynamic limits being respected.
bool ConvertCurveToTrajectory(const Curve& curve, double desired_speed,
                              const DynamicLimits& limits, Trajectory* traj);

// Same as above, but over a specific cord-length span only.
bool ConvertCurveToTrajectory(const Curve& curve,
                              Interval<double> cord_length_span,
                              double desired_speed,
                              const Kinematics& kinematics, Trajectory* traj);

// Same as above, but over a specific cord-length span only and with dynamic
// limits being respected.
bool ConvertCurveToTrajectory(const Curve& curve,
                              Interval<double> cord_length_span,
                              double desired_speed, const DynamicLimits& limits,
                              Trajectory* traj);

// Resamples a curve to approximate cord-length steps. The points of the
// original curve are preserved because this is the only way to preserve the
// exact shape of the curve. In other words, each constant arc segment of the
// original curve will be split into a integer number of segments with at most
// the cord-length of 'cord_step' each.
// If 'cord_step' is larger than the cord-length of the original arc segments
// of the curve, then this function has no effect.
// This function is mainly useful to ensure that all segments are shorter
// than 'cord_step' in a curve.
bool ResampleCurve(const Curve& curve_in, double cord_step, Curve* curve_out);

// Converts a curve to a polyline. This is a pretty straight-forward
// conversion, which mainly involves evaluating the curve at given cord-length
// increments and stringing together the evaluated poses into a polyline.
bool ResampleCurveToPolyline(const Curve& curve_in, double cord_step,
                             Curve* polyline_out);

// Checks that a curve is a polyline.
bool CheckPolylineProperties(const Curve& polyline);

// Resamples a trajectory to approximate time steps. The points of the
// original trajectory are preserved because this is the only way to preserve
// the exact shape of the trajectory. In other words, each constant arc
// segment of the original trajectory will be split into a integer number of
// segments with at most the duration of 'time_step' each.
// If 'time_step' is larger than the duration of the original arc segments
// of the trajectory, then this function has no effect.
// This function is mainly useful to ensure that all segments are shorter
// than 'time_step' in a trajectory.
bool ResampleTrajectory(const Trajectory& traj_in, double time_step,
                        Trajectory* traj_out);

// Warps time on a whole trajectory such that all velocities fall within
// the given dynamic limits (velocity limits). This function will not change
// the path followed by the robot because it only slows down segments that
// drive too fast but should follow the same underlying curve.
void TimeWarpTrajectoryIntoLimits(const DynamicLimits& limits,
                                  Trajectory* traj);

// Warps time on a trajectory such that velocities are decayed by the given
// decay rate, without creating jumps in velocity that would violate the
// trajectory limits. This function will not change the path followed by the
// robot because it only slows it down while following the same curve.
void DecaySpeedOfTrajectory(const Kinematics& kinematics,
                            const TrajectoryLimits& traj_limits,
                            double decay_rate_per_second, Trajectory* traj);

// Brings a whole trajectory within limits such that all velocity jumps
// respect the given trajectory limits. This function will change the path
// followed by the robot because it replaces any infeasible velocity jumps
// with a sequence of feasible jumps. In effect, this function makes the
// trajectory follow what would be the result of executing this trajectory
// on a controller that limited the acceleration itself.
// It is highly recommended to use the `TimeWarpTrajectoryIntoLimits` function
// first to bring the velocities into the absolute limits first.
// The `start_arc_vel` parameter is used to specify what velocity comes prior
// to the start of the trajectory. Pass in the start velocity of the trajectory
// if you don't have a meaningful start velocity.
// Returns false if it ran out of capacity in the trajectory, which can happen
// if too many sub-segments have to be added.
bool BringTrajectoryIntoLimits(const ArcVector& start_arc_vel,
                               const Trajectory& traj_in,
                               const DynamicLimits& limits,
                               const TrajectoryLimits& traj_limits,
                               Trajectory* traj_out);

// Amends the end of the trajectory such that it comes to a full stop at the
// end while remaining within the given trajectory limits (acceleration). If
// the original trajectory has state transitions that are impossible to meet
// within the constraints, this function will return false. Also, if it runs
// out of capacity at any point, it will return false.
// When this function returns false, the end will still contain a full stop,
// but the ramp to come to a full stop might not respect the constraints.
// If this function returns true, then the trajectory should end with a
// feasible ramp down to a full stop.
bool AmendTrajectoryToEndWithStop(const Kinematics& kinematics,
                                  const TrajectoryLimits& traj_limits,
                                  Trajectory* traj);

// Amends the end of the trajectory such that it comes to a given slowed-down
// speed at the end while remaining within the given trajectory limits
// (acceleration). If the original trajectory has state transitions that are
// impossible to meet within the constraints, this function will return false.
// If the given speed is higher (in magnitude) than the speed at the end of
// the trajectory, this function will return false.
// Also, if it runs out of capacity in the output trajectory, it returns false.
// When this function returns false, the end will still contain a reduced speed,
// but the ramp to come to that speed might not respect the constraints.
// If this function returns true, then the trajectory should end with a
// feasible ramp down to the given speed.
bool AmendTrajectoryToEndWithSlowdown(const Kinematics& kinematics,
                                      double desired_speed,
                                      const TrajectoryLimits& traj_limits,
                                      Trajectory* traj);

// Finds a slowed-down trajectory that can be amended at the end to meet a
// given end speed, using AmendTrajectoryToEndWithSlowdown().
bool FindSlowerTrajectoryToMeetEndSpeed(
    const Trajectory& original_traj, double original_speed,
    double desired_end_speed, double tolerance, const Kinematics& kinematics,
    const TrajectoryLimits& traj_limits, Trajectory* result_traj);

// Finds a slowed-down trajectory that can be amended at the end to end with
// zero speed (aka stop), using AmendTrajectoryToEndWithStop().
bool FindSlowerTrajectoryToEndWithStop(const Trajectory& original_traj,
                                       double tolerance,
                                       const Kinematics& kinematics,
                                       const TrajectoryLimits& traj_limits,
                                       Trajectory* result_traj);

// Rounds off the hard corners of a polyline curve to produce a curve with
// continuous angles. The rounding off method is simply to add constant
// radius arcs across each hard corner of the input polyline. The radius
// of the arcs are chosen to be as large as possible between the two mid-points
// of the line segments adjacent to a given hard corner.
bool RoundOffCornersOfPolyline(const Curve& polyline, Curve* curve);

// Converts a curve into a discretized clothoid trajectory that respects the
// given dynamic and trajectory limits, and maintains a given desired speed.
// The conversion is done by approximating each circular arc segment of the
// curve by a sequence of velocity "ticks" that attempt to reach sufficient
// curvature to meet the other side of the arc. In other words, it produces
// curvature steps that approximate the curvature ramps of a symmetric
// clothoid arc.
// The desired speed can be negative, in which case, the orientation of the
// trajectory poses are reversed from those of the curve.
bool ConvertCurveToClothoidTrajectory(const Curve& curve, double desired_speed,
                                      const DynamicLimits& limits,
                                      const TrajectoryLimits& traj_limits,
                                      Trajectory* traj);

// Searches for a desired speed that is as high as possible (up to given
// desired speed) for which the given curve can be successfully converted to
// a clothoid trajectory using ConvertCurveToClothoidTrajectory().
// The tolerance parameter specifies how close to the limit speed we should get.
bool FindClothoidTrajectoryForCurve(const Curve& curve, double desired_speed,
                                    double tolerance,
                                    const DynamicLimits& limits,
                                    const TrajectoryLimits& traj_limits,
                                    Trajectory* traj,
                                    double* sub_desired_speed);

// Appends a curve, after finding a suitable clothoid trajectory for it, to a
// given resulting trajectory.
// The `desired_speed` is the absolute value of the velocity that the clothoid
// trajectory segment should attempt to reach (max).
// The `limits` and `traj_limits` are the diff-drive limits to apply. They
// should already have the aggressivity factor applied to them if desired.
// The `curve_segment` is the curve to be converted to a clothoid. If the curve
// only contains one point, that point is treated as the goal of a pure-spin
// trajectory to be generated. Otherwise, the start pose of the curve should
// be equal to last pose already in the `result_traj`, if non-empty.
// The `going_backwards` flag signifies that the curve goes from the finish pose
// to the start pose and that the trajectory segment should then be reversed
// (thus, giving it negative velocity) before appending it.
// The `traj_segment` trajectory will contain the trajectory segment found.
// The `traj_scratch_space` trajectory will be used internally as a
// scratch-space to avoid repeated memory allocations. It should have enough
// capacity to store the resulting trajectory. The `result_traj` trajectory is
// the accumulated combined trajectory which will have the trajectory segment
// appended to it after this. The function returns false if any of the
// intermediate steps have failed, which is typically caused by running out of
// capacity in the output.
bool AppendClothoidFromCurveToTrajectory(
    double desired_speed, const DynamicLimits& limits,
    const TrajectoryLimits& traj_limits, const Curve& curve_segment,
    bool going_backwards, Trajectory* traj_segment,
    Trajectory* traj_scratch_space, Trajectory* result_traj);

// Computes two circular arcs that join the origin to a given final position
// and orientation. Outputs (kappa1, arc_length1) and (kappa2, arc_length2)
// specify first and second arc, respectively, and the median_pose is the
// pose that is reached where the two arcs meet.
// Returns false if it could not solve for two arcs that meet the end
// conditions given.
bool ComputeTwoArcsToFinalPose(const eigenmath::Vector2d& final_pt,
                               const eigenmath::SO2d& final_so2, double* kappa1,
                               double* arc_length1, double* kappa2,
                               double* arc_length2,
                               eigenmath::Pose2d* median_pose);

// Computes a curve made of two circular arcs that join one pose to another.
// The curve must have a capacity for at least 3 curve points.
// This function relies on ComputeTwoArcsToFinalPose to solve for the arcs.
// Returns false if it could not solve for two arcs that meet the end
// conditions given.
bool ComputeTwoArcsToFinalPose(const eigenmath::Pose2d& start_pose,
                               const eigenmath::Pose2d& final_pose,
                               Curve* output_curve);

// Converts a quintic spline into a curve.
// This is accomplished by sampling a number of points on the spline and
// creating circular arcs that approximately join them.
// Converting a spline to a curve of piece-wise circular arcs will never be
// perfect, and this function is a "best effort" conversion. Increasing the
// density of samples will achieve a better approximation, for values above
// 10 samples for segment are reasonable.
// Returns false if it could not construct a curve that approximates the
// given spline or if it ran out of preallocated capacity in the output curve.
bool ConvertQuinticSplineToCurve(const eigenmath::QuinticSpline& spline,
                                 int num_samples_per_segment, Curve* curve);

// Append a ramp with constant acceleration that ramps up to a given desired
// arc velocity. The trajectory will be generated up to the given desired
// duration. If the desired velocity can be reached using the given aggressivity
// factor before the desired duration, then the remainder of the trajectory
// will have constant velocity. If the desired velocity cannot be reached
// within the desired duration using the given aggressivity, then the
// trajectory will have increased aggressivity (up to 1.0) to reach the desired
// velocity within the desired duration. If even with maximum aggressivity, the
// desired velocity cannot be reached, it will go as close as it can.
// Returns false if the trajectory capacity is exhausted.
bool AppendRampToSpeedWithConstantAccel(const ArcVector& desired_arc_velocity,
                                        const Kinematics& kinematics,
                                        const TrajectoryLimits& traj_limits,
                                        double desired_duration,
                                        double aggressivity_factor,
                                        Trajectory* result_traj);

// Compute the total amount of wheel motion (additive of both wheels) over
// a given trajectory. Optionally, over a given time-interval.
double ComputeTotalWheelMotion(const Trajectory& traj,
                               const Kinematics& kinematics);
double ComputeTotalWheelMotion(const Trajectory& traj,
                               const Kinematics& kinematics,
                               const Interval<double>& time_interval);

// Compute the total amount of wheel motion (additive of both wheels) over
// a given trajectory. Optionally, over a given time-interval.
double ComputeTotalAbsoluteWheelMotion(const Trajectory& traj,
                                       const Kinematics& kinematics);
double ComputeTotalAbsoluteWheelMotion(const Trajectory& traj,
                                       const Kinematics& kinematics,
                                       const Interval<double>& time_interval);

bool ConvertCurveToWheelCurve(const Curve& curve, const Kinematics& kinematics,
                              const WheelVector& initial_wheel_position,
                              double initial_total_motion,
                              WheelCurve* wheel_curve);

inline bool ConvertCurveToWheelCurve(const Curve& curve,
                                     const Kinematics& kinematics,
                                     WheelCurve* wheel_curve) {
  return ConvertCurveToWheelCurve(curve, kinematics, WheelVector{0.0, 0.0}, 0.0,
                                  wheel_curve);
}

bool ConvertTrajectoryToWheelCurve(const Trajectory& traj,
                                   const Kinematics& kinematics,
                                   const WheelVector& initial_wheel_position,
                                   double initial_total_motion,
                                   WheelCurve* wheel_curve);

inline bool ConvertTrajectoryToWheelCurve(const Trajectory& traj,
                                          const Kinematics& kinematics,
                                          WheelCurve* wheel_curve) {
  return ConvertTrajectoryToWheelCurve(traj, kinematics, WheelVector{0.0, 0.0},
                                       0.0, wheel_curve);
}

// This class can be used to iterate over a trajectory by
// wheel motion increments.
class TrajectoryWheelMotionIterator {
 public:
  // Create a wheel-motion-iterator pointing exactly at a given state iterator.
  TrajectoryWheelMotionIterator(const Kinematics* kinematics,
                                Trajectory::StateIterator it, double max_time)
      : kinematics_(kinematics),
        current_it_(it),
        current_time_(it->time),
        max_time_(max_time) {}

  // Create a wheel-motion-iterator pointing at a given state iterator and time.
  TrajectoryWheelMotionIterator(const Kinematics* kinematics,
                                Trajectory::StateIterator it,
                                double current_time, double max_time)
      : kinematics_(kinematics),
        current_it_(it),
        current_time_(current_time),
        max_time_(max_time) {}

  // Get the time at which this wheel-motion-iterator points.
  double GetTime() const { return current_time_; }

  // Compute the state to which this wheel-motion-iterator points.
  State ComputeState() const;

  // Advance the wheel-motion-iterator by a given delta in wheel-motion.
  TrajectoryWheelMotionIterator& operator+=(double dw);

  // Advance the wheel-motion-iterator by a given delta in wheel-motion.
  friend TrajectoryWheelMotionIterator operator+(
      const TrajectoryWheelMotionIterator& it, double dw) {
    TrajectoryWheelMotionIterator result = it;
    result += dw;
    return result;  // NRVO
  }

  // Advance the wheel-motion-iterator by a given delta in wheel-motion.
  friend TrajectoryWheelMotionIterator operator+(
      double dw, const TrajectoryWheelMotionIterator& it) {
    TrajectoryWheelMotionIterator result = it;
    result += dw;
    return result;  // NRVO
  }

  // Compare two wheel-motion-iterators to see if LHS comes before RHS.
  // Note, it is important to use less-than to terminate iterations.
  bool operator<(const TrajectoryWheelMotionIterator& rhs) const {
    return current_time_ < rhs.current_time_;
  }

 private:
  const Kinematics* kinematics_;
  Trajectory::StateIterator current_it_;
  double current_time_;
  double max_time_;
};

// Get a wheel-motion-iterator to the beginning of the trajectory.
inline TrajectoryWheelMotionIterator BeginByWheelMotion(
    const Kinematics* kinematics, const Trajectory& traj) {
  if (traj.IsEmpty()) {
    return TrajectoryWheelMotionIterator(kinematics, traj.BeginState(), 0.0,
                                         0.0);
  }
  return TrajectoryWheelMotionIterator(kinematics, traj.BeginState(),
                                       traj.GetTimeSpan().max());
}

// Get a wheel-motion-iterator to the end of the trajectory.
inline TrajectoryWheelMotionIterator EndByWheelMotion(
    const Kinematics* kinematics, const Trajectory& traj) {
  if (traj.IsEmpty()) {
    return TrajectoryWheelMotionIterator(kinematics, traj.BeginState(), 0.0,
                                         0.0);
  }
  return TrajectoryWheelMotionIterator(kinematics, traj.EndState() - 1,
                                       traj.GetTimeSpan().max());
}

// Prints the curve points, evaluated at ds cord-length intervals, to the
// given output stream.
void PrintCurveTrace(const ContinuousCurve& curve, double ds,
                     std::ostream* out);
std::string CurveTraceToString(const ContinuousCurve& curve, double ds);

// Prints the wheel curve states, evaluated at dw total motion intervals, to the
// given output stream.
void PrintWheelCurveTrace(const WheelCurve& curve, double dw,
                          std::ostream* out);
std::string WheelCurveTraceToString(const WheelCurve& curve, double dw);

// Prints the trajectory points, evaluated at ds cord-length intervals, to the
// given output stream.
void PrintTrajectoryTrace(const Trajectory& traj, double dt, std::ostream* out);
std::string TrajectoryTraceToString(const Trajectory& traj, double dt);

struct FullPrecisionDouble {
  double v;
  explicit FullPrecisionDouble(double d) : v(d) {}
  template <typename Sink>
  friend void AbslStringify(Sink& sink, const FullPrecisionDouble& value) {
    absl::Format(&sink, "%.16f", value.v);
  }
};

template <typename Sink>
void AbslStringify(Sink& sink, const CurvePoint& pt) {
  absl::Format(&sink, "x: %v y: %v theta: %v kappa: %v",
               FullPrecisionDouble(pt.GetPose().translation().x()),
               FullPrecisionDouble(pt.GetPose().translation().y()),
               FullPrecisionDouble(pt.GetPose().angle()),
               FullPrecisionDouble(pt.GetCurvature()));
}

template <typename Sink>
void AbslStringify(Sink& sink, const CurvePtAndCord& pt) {
  absl::Format(&sink, "s: %v %v", FullPrecisionDouble(pt.cord_length),
               pt.point);
}

template <typename Sink>
void AbslStringify(Sink& sink, const Curve& curve) {
  for (const auto& curve_pt : curve.GetCurvePointIteratorRange()) {
    absl::Format(&sink, "%v\n", curve_pt);
  }
}

template <typename Sink>
void AbslStringify(Sink& sink, const WheelState& state) {
  absl::Format(&sink, "l: %v r: %v fl: %v fr: %v",
               FullPrecisionDouble(state.GetPositions().Left()),
               FullPrecisionDouble(state.GetPositions().Right()),
               FullPrecisionDouble(state.GetMotionRates().Left()),
               FullPrecisionDouble(state.GetMotionRates().Right()));
}

template <typename Sink>
void AbslStringify(Sink& sink, const WheelStateAndTotalMotion& state) {
  absl::Format(&sink, "w: %v %v", FullPrecisionDouble(state.total_motion),
               state.state);
}

template <typename Sink>
void AbslStringify(Sink& sink, const WheelCurve& curve) {
  for (const auto& curve_pt : curve.GetWheelStateIteratorRange()) {
    absl::Format(&sink, "%v\n", curve_pt);
  }
}

template <typename Sink>
void AbslStringify(Sink& sink, const State& state) {
  absl::Format(&sink, "x: %v y: %v theta: %v v: %v w: %v",
               FullPrecisionDouble(state.GetPose().translation().x()),
               FullPrecisionDouble(state.GetPose().translation().y()),
               FullPrecisionDouble(state.GetPose().angle()),
               FullPrecisionDouble(state.GetArcVelocity().Translation()),
               FullPrecisionDouble(state.GetArcVelocity().Rotation()));
}

template <typename Sink>
void AbslStringify(Sink& sink, const StateAndTime& state) {
  absl::Format(&sink, "t: %v %v", FullPrecisionDouble(state.time), state.state);
}

template <typename Sink>
void AbslStringify(Sink& sink, const Trajectory& traj) {
  for (const auto& traj_pt : traj.GetStateIteratorRange()) {
    absl::Format(&sink, "%v\n", traj_pt);
  }
}

template <typename Sink>
void AbslStringify(Sink& sink, const Kinematics& kinematics) {
  absl::Format(&sink,
               "wheel radius left: %v wheel radius right: %v wheel base: %v",
               kinematics.GetWheelRadius().Left(),
               kinematics.GetWheelRadius().Right(), kinematics.GetWheelBase());
}

template <typename Sink>
void AbslStringify(Sink& sink, const BoxConstraints& box_constraints) {
  absl::Format(
      &sink,
      "min wheel: (l: %v, r: %v) max wheel: (l: %v, r: %v) min arc: (t: %v, r: "
      "%v) max arc: (t: %v, r: %v)",
      box_constraints.MinWheelVector().Left(),
      box_constraints.MinWheelVector().Right(),
      box_constraints.MaxWheelVector().Left(),
      box_constraints.MaxWheelVector().Right(),
      box_constraints.MinArcVector().Translation(),
      box_constraints.MinArcVector().Rotation(),
      box_constraints.MaxArcVector().Translation(),
      box_constraints.MaxArcVector().Rotation());
}

template <typename Sink>
void AbslStringify(Sink& sink, const DynamicLimits& dd_limits) {
  absl::Format(&sink,
               "kinematics: %v velocity limits: %v acceleration limits: %v",
               dd_limits.GetKinematics(), dd_limits.VelocityLimits(),
               dd_limits.AccelerationLimits());
}

template <typename Sink>
void AbslStringify(Sink& sink, const TrajectoryLimits& traj_limits) {
  absl::Format(
      &sink,
      "max wheel velocity jump: (l: %v, r: %v) max arc velocity jump: (t: %v, "
      "r: %v) min cycle duration: %v",
      traj_limits.GetMaxWheelVelocityJump().Left(),
      traj_limits.GetMaxWheelVelocityJump().Right(),
      traj_limits.GetMaxArcVelocityJump().Translation(),
      traj_limits.GetMaxArcVelocityJump().Rotation(),
      traj_limits.GetMinCycleDuration());
}

#define MOBILITY_DIFF_DRIVE_DEFINE_OSTREAM_OPERATOR(TYPE)        \
  inline std::ostream& operator<<(std::ostream& out, const TYPE& v) { \
    return out << absl::StrCat(v);                                    \
  }

MOBILITY_DIFF_DRIVE_DEFINE_OSTREAM_OPERATOR(Curve)
MOBILITY_DIFF_DRIVE_DEFINE_OSTREAM_OPERATOR(WheelCurve)
MOBILITY_DIFF_DRIVE_DEFINE_OSTREAM_OPERATOR(Trajectory)

MOBILITY_DIFF_DRIVE_DEFINE_OSTREAM_OPERATOR(CurvePtAndCord);
MOBILITY_DIFF_DRIVE_DEFINE_OSTREAM_OPERATOR(WheelStateAndTotalMotion);
MOBILITY_DIFF_DRIVE_DEFINE_OSTREAM_OPERATOR(StateAndTime);

MOBILITY_DIFF_DRIVE_DEFINE_OSTREAM_OPERATOR(CurvePoint);
MOBILITY_DIFF_DRIVE_DEFINE_OSTREAM_OPERATOR(WheelState);
MOBILITY_DIFF_DRIVE_DEFINE_OSTREAM_OPERATOR(State);

MOBILITY_DIFF_DRIVE_DEFINE_OSTREAM_OPERATOR(Kinematics);
MOBILITY_DIFF_DRIVE_DEFINE_OSTREAM_OPERATOR(BoxConstraints);
MOBILITY_DIFF_DRIVE_DEFINE_OSTREAM_OPERATOR(DynamicLimits);
MOBILITY_DIFF_DRIVE_DEFINE_OSTREAM_OPERATOR(TrajectoryLimits);

#undef MOBILITY_DIFF_DRIVE_DEFINE_OSTREAM_OPERATOR

}  // namespace mobility::diff_drive

#endif  // MOBILITY_DIFF_DRIVE_DIFF_DRIVE_CURVE_TRAJECTORY_UTILS_H_
