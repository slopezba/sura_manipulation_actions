#pragma once

#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>
#include <Eigen/Geometry>
#include <Eigen/LU>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace sura_manipulation_actions
{

// Position and ZYX Euler coordinates in the trajectory header's fixed frame.
// The final three derivatives are Euler rates, rather than angular velocity.
using BaseCoordinates = Eigen::Matrix<double, 6, 1>;

struct BaseTrajectorySample
{
  BaseCoordinates coordinates = BaseCoordinates::Zero();
  BaseCoordinates velocity = BaseCoordinates::Zero();
  BaseCoordinates acceleration = BaseCoordinates::Zero();
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  geometry_msgs::msg::Twist twist;
  geometry_msgs::msg::Twist twist_acceleration;
};

struct BaseTrajectoryKnot
{
  double time = 0.0;
  BaseCoordinates coordinates = BaseCoordinates::Zero();
  BaseCoordinates velocity = BaseCoordinates::Zero();
  BaseCoordinates acceleration = BaseCoordinates::Zero();
  bool has_velocity = false;
  bool has_acceleration = false;
};

inline double baseTrajectorySeconds(const builtin_interfaces::msg::Duration & value)
{
  return static_cast<double>(value.sec) + 1.0e-9 * value.nanosec;
}

inline BaseCoordinates baseCoordinates(
  const Eigen::Isometry3d & pose,
  double yaw_reference = std::numeric_limits<double>::quiet_NaN())
{
  BaseCoordinates result;
  result.head<3>() = pose.translation();
  const auto & rotation = pose.rotation();
  result(3) = std::atan2(rotation(2, 1), rotation(2, 2));
  result(4) = std::atan2(-rotation(2, 0), std::hypot(rotation(0, 0), rotation(1, 0)));
  result(5) = std::atan2(rotation(1, 0), rotation(0, 0));
  if (std::isfinite(yaw_reference)) {
    result(5) = yaw_reference + std::remainder(result(5) - yaw_reference, 2.0 * std::acos(-1.0));
  }
  return result;
}

inline BaseCoordinates baseCoordinates(
  const Eigen::Isometry3d & pose, const BaseCoordinates & angular_reference)
{
  auto result = baseCoordinates(pose, angular_reference(5));
  result(3) = angular_reference(3) + std::remainder(
    result(3) - angular_reference(3), 2.0 * std::acos(-1.0));
  return result;
}

inline Eigen::Isometry3d basePose(const BaseCoordinates & coordinates)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = coordinates.head<3>();
  result.linear() = (
    Eigen::AngleAxisd(coordinates(5), Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(coordinates(4), Eigen::Vector3d::UnitY()) *
    Eigen::AngleAxisd(coordinates(3), Eigen::Vector3d::UnitX())).toRotationMatrix();
  return result;
}

inline Eigen::Matrix3d baseAngularVelocityMap(const BaseCoordinates & coordinates)
{
  const double cp = std::cos(coordinates(4)), sp = std::sin(coordinates(4));
  const double cy = std::cos(coordinates(5)), sy = std::sin(coordinates(5));
  Eigen::Matrix3d result;
  result << cy * cp, -sy, 0.0, sy * cp, cy, 0.0, -sp, 0.0, 1.0;
  return result;
}

inline Eigen::Matrix3d baseAngularVelocityMapDerivative(
  const BaseCoordinates & coordinates, const BaseCoordinates & velocity)
{
  const double cp = std::cos(coordinates(4)), sp = std::sin(coordinates(4));
  const double cy = std::cos(coordinates(5)), sy = std::sin(coordinates(5));
  const double pd = velocity(4), yd = velocity(5);
  Eigen::Matrix3d result = Eigen::Matrix3d::Zero();
  result.col(0) = Eigen::Vector3d(-sy * cp * yd - cy * sp * pd,
    cy * cp * yd - sy * sp * pd, -cp * pd);
  result.col(1) = Eigen::Vector3d(-cy * yd, -sy * yd, 0.0);
  return result;
}

inline Eigen::Vector3d baseLinearVector(const geometry_msgs::msg::Twist & value)
{
  return Eigen::Vector3d(value.linear.x, value.linear.y, value.linear.z);
}

inline Eigen::Vector3d baseAngularVector(const geometry_msgs::msg::Twist & value)
{
  return Eigen::Vector3d(value.angular.x, value.angular.y, value.angular.z);
}

inline geometry_msgs::msg::Twist baseTwist(
  const Eigen::Vector3d & linear, const Eigen::Vector3d & angular)
{
  geometry_msgs::msg::Twist result;
  result.linear.x = linear.x(); result.linear.y = linear.y(); result.linear.z = linear.z();
  result.angular.x = angular.x(); result.angular.y = angular.y(); result.angular.z = angular.z();
  return result;
}

inline void completeBaseSample(BaseTrajectorySample & sample)
{
  sample.pose = basePose(sample.coordinates);
  const auto map = baseAngularVelocityMap(sample.coordinates);
  sample.twist = baseTwist(sample.velocity.head<3>(), map * sample.velocity.tail<3>());
  sample.twist_acceleration = baseTwist(sample.acceleration.head<3>(),
    map * sample.acceleration.tail<3>() +
    baseAngularVelocityMapDerivative(sample.coordinates, sample.velocity) * sample.velocity.tail<3>());
}

inline trajectory_msgs::msg::MultiDOFJointTrajectoryPoint baseTrajectoryPoint(
  const BaseCoordinates & coordinates, const BaseCoordinates & velocity,
  const BaseCoordinates & acceleration, const builtin_interfaces::msg::Duration & time)
{
  BaseTrajectorySample sample;
  sample.coordinates = coordinates; sample.velocity = velocity; sample.acceleration = acceleration;
  completeBaseSample(sample);
  geometry_msgs::msg::Transform transform;
  transform.translation.x = sample.pose.translation().x();
  transform.translation.y = sample.pose.translation().y();
  transform.translation.z = sample.pose.translation().z();
  const Eigen::Quaterniond quaternion(sample.pose.rotation());
  transform.rotation.x = quaternion.x(); transform.rotation.y = quaternion.y();
  transform.rotation.z = quaternion.z(); transform.rotation.w = quaternion.w();
  trajectory_msgs::msg::MultiDOFJointTrajectoryPoint point;
  point.transforms.push_back(transform);
  point.velocities.push_back(sample.twist);
  point.accelerations.push_back(sample.twist_acceleration);
  point.time_from_start = time;
  return point;
}

// Normalize time to [0, 1]. The same polynomial is used by planning and control.
inline std::array<double, 6> baseHermitePolynomial(
  double p0, double p1, double v0, double v1, double a0, double a1,
  double duration, bool has_velocity = true, bool has_acceleration = true)
{
  std::array<double, 6> c{{p0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  if (!has_velocity) {c[1] = p1 - p0; return c;}
  c[1] = duration * v0;
  if (!has_acceleration) {
    c[2] = 3.0 * (p1 - p0) - duration * (2.0 * v0 + v1);
    c[3] = 2.0 * (p0 - p1) + duration * (v0 + v1);
    return c;
  }
  c[2] = 0.5 * duration * duration * a0;
  const double d = p1 - c[0] - c[1] - c[2];
  const double v = duration * v1 - c[1] - 2.0 * c[2];
  const double a = duration * duration * a1 - 2.0 * c[2];
  c[3] = 10.0 * d - 4.0 * v + 0.5 * a;
  c[4] = -15.0 * d + 7.0 * v - a;
  c[5] = 6.0 * d - 3.0 * v + 0.5 * a;
  return c;
}

inline double evaluateBasePolynomial(const std::array<double, 6> & coefficients, double u, int derivative = 0)
{
  double result = 0.0;
  for (int power = 5; power >= derivative; --power) {
    double coefficient = coefficients[static_cast<size_t>(power)];
    for (int d = 0; d < derivative; ++d) {coefficient *= power - d;}
    result = result * u + coefficient;
  }
  return result;
}

inline std::vector<double> basePolynomialExtrema(const std::array<double, 6> & c, int derivative)
{
  std::vector<double> coefficients;
  for (int power = derivative; power <= 5; ++power) {
    double value = c[static_cast<size_t>(power)];
    for (int d = 0; d < derivative; ++d) {value *= power - d;}
    coefficients.push_back(value);
  }
  while (coefficients.size() > 1 && std::abs(coefficients.back()) < 1.0e-13) {coefficients.pop_back();}
  std::vector<double> points{0.0, 1.0};
  const int degree = static_cast<int>(coefficients.size()) - 1;
  if (degree <= 0) {return points;}
  Eigen::MatrixXd companion = Eigen::MatrixXd::Zero(degree, degree);
  for (int i = 1; i < degree; ++i) {companion(i, i - 1) = 1.0;}
  for (int i = 0; i < degree; ++i) {companion(i, degree - 1) = -coefficients[static_cast<size_t>(i)] / coefficients.back();}
  Eigen::EigenSolver<Eigen::MatrixXd> solver(companion, false);
  if (solver.info() != Eigen::Success) {return {};}
  for (const auto & root : solver.eigenvalues()) {
    if (std::abs(root.imag()) < 1.0e-8 && root.real() > 0.0 && root.real() < 1.0) {points.push_back(root.real());}
  }
  return points;
}

// Prepare once at admission. Control updates can then sample without allocating
// or scanning the quaternion history to recover continuous Euler coordinates.
inline bool prepareBaseTrajectory(
  const trajectory_msgs::msg::MultiDOFJointTrajectory & trajectory,
  std::vector<BaseTrajectoryKnot> & knots, std::string & error)
{
  knots.clear();
  if (trajectory.header.frame_id.empty() || trajectory.joint_names.size() != 1 ||
    trajectory.joint_names.front().empty() || trajectory.points.empty())
  {error = "Base trajectory needs a fixed frame, one named joint and points"; return false;}
  knots.reserve(trajectory.points.size());
  for (const auto & point : trajectory.points) {
    if (point.transforms.size() != 1 || point.velocities.size() > 1 || point.accelerations.size() > 1 ||
      (!point.accelerations.empty() && point.velocities.empty()))
    {error = "Base trajectory point dimensions are inconsistent"; knots.clear(); return false;}
    const auto & transform = point.transforms.front();
    Eigen::Quaterniond quaternion(transform.rotation.w, transform.rotation.x,
      transform.rotation.y, transform.rotation.z);
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(transform.translation.x, transform.translation.y, transform.translation.z);
    if (!quaternion.coeffs().allFinite() || std::abs(quaternion.norm() - 1.0) > 0.001 ||
      !pose.translation().allFinite())
    {error = "Base trajectory contains an invalid transform"; knots.clear(); return false;}
    pose.linear() = quaternion.normalized().toRotationMatrix();
    BaseTrajectoryKnot knot;
    knot.time = baseTrajectorySeconds(point.time_from_start);
    if (point.time_from_start.nanosec >= 1000000000U || !std::isfinite(knot.time) || knot.time < 0.0 ||
      (!knots.empty() && knot.time <= knots.back().time))
    {error = "Base trajectory times must be nonnegative and strictly increasing"; knots.clear(); return false;}
    knot.coordinates = baseCoordinates(pose, knots.empty() ?
      std::numeric_limits<double>::quiet_NaN() : knots.back().coordinates(5));
    if (!knots.empty()) {
      const double previous_roll = knots.back().coordinates(3);
      knot.coordinates(3) = previous_roll + std::remainder(
        knot.coordinates(3) - previous_roll, 2.0 * std::acos(-1.0));
    }
    if (std::abs(std::cos(knot.coordinates(4))) < 1.0e-4)
    {error = "Base trajectory pitch is too close to an Euler singularity"; knots.clear(); return false;}
    // Adjacent samples must identify a unique continuous orientation branch.
    if (!knots.empty() &&
      (knot.coordinates.tail<3>() - knots.back().coordinates.tail<3>()).cwiseAbs().maxCoeff() >= std::acos(-1.0) - 1.0e-6)
    {error = "Base trajectory has an ambiguous angular step"; knots.clear(); return false;}
    knot.has_velocity = !point.velocities.empty();
    knot.has_acceleration = !point.accelerations.empty();
    const auto inverse = baseAngularVelocityMap(knot.coordinates).inverse().eval();
    if (knot.has_velocity) {
      knot.velocity.head<3>() = baseLinearVector(point.velocities.front());
      knot.velocity.tail<3>() = inverse * baseAngularVector(point.velocities.front());
    }
    if (knot.has_acceleration) {
      knot.acceleration.head<3>() = baseLinearVector(point.accelerations.front());
      knot.acceleration.tail<3>() = inverse * (baseAngularVector(point.accelerations.front()) -
        baseAngularVelocityMapDerivative(knot.coordinates, knot.velocity) * knot.velocity.tail<3>());
    }
    if (!knot.velocity.allFinite() || !knot.acceleration.allFinite() ||
      (!knots.empty() && (knot.has_velocity != knots.back().has_velocity ||
      knot.has_acceleration != knots.back().has_acceleration)))
    {error = "Base trajectory derivatives must be finite and consistently present"; knots.clear(); return false;}
    knots.push_back(knot);
  }
  return true;
}

// Admission-time analytic checks. Public execution endpoints must apply them
// even when a trajectory did not originate in the Cartesian planner.
inline bool validateBaseTrajectoryLimits(
  const std::vector<BaseTrajectoryKnot> & knots,
  const std::vector<double> & lower, const std::vector<double> & upper,
  const std::vector<double> & velocity_limits, const std::vector<double> & acceleration_limits,
  const std::vector<bool> & mask, std::string & error)
{
  if (knots.empty() || (!lower.empty() && lower.size() != 6) || (!upper.empty() && upper.size() != 6) ||
    velocity_limits.size() != 6 || acceleration_limits.size() != 6 || (!mask.empty() && mask.size() < 6))
  {error = "Base polynomial validation needs six-axis limits and prepared points"; return false;}
  const double pitch_bound = std::acos(1.0e-4) - 1.0e-8;
  BaseCoordinates lo, hi;
  for (size_t axis = 0; axis < 6; ++axis) {
    lo(static_cast<Eigen::Index>(axis)) = lower.empty() ? -std::numeric_limits<double>::infinity() : lower[axis];
    hi(static_cast<Eigen::Index>(axis)) = upper.empty() ? std::numeric_limits<double>::infinity() : upper[axis];
    if ((!lower.empty() && !std::isfinite(lower[axis])) || (!upper.empty() && !std::isfinite(upper[axis])) ||
      lo(static_cast<Eigen::Index>(axis)) > hi(static_cast<Eigen::Index>(axis)) ||
      !std::isfinite(velocity_limits[axis]) || velocity_limits[axis] <= 0.0 ||
      !std::isfinite(acceleration_limits[axis]) || acceleration_limits[axis] <= 0.0)
    {error = "Base polynomial limits must be ordered with positive finite dynamics"; return false;}
  }
  lo(4) = std::max(lo(4), -pitch_bound); hi(4) = std::min(hi(4), pitch_bound);
  if (!mask.empty()) {
    for (Eigen::Index axis = 0; axis < 6; ++axis) {
      if (!mask[static_cast<size_t>(axis)]) {
        lo(axis) = std::max(lo(axis), knots.front().coordinates(axis));
        hi(axis) = std::min(hi(axis), knots.front().coordinates(axis));
      }
    }
  }
  for (size_t i = 0; i < knots.size(); ++i) {
    const auto & knot = knots[i];
    if (!knot.coordinates.allFinite() || !knot.velocity.allFinite() || !knot.acceleration.allFinite() ||
      !std::isfinite(knot.time) || knot.time < 0.0 || (i && knot.time <= knots[i - 1].time))
    {error = "Invalid prepared base trajectory knot"; return false;}
    for (Eigen::Index axis = 0; axis < 6; ++axis) {
      const size_t index = static_cast<size_t>(axis);
      if (knot.coordinates(axis) < lo(axis) - 1.0e-9 || knot.coordinates(axis) > hi(axis) + 1.0e-9 ||
        std::abs(knot.velocity(axis)) > velocity_limits[index] * (1.0 + 1.0e-6) ||
        std::abs(knot.acceleration(axis)) > acceleration_limits[index] * (1.0 + 1.0e-6) ||
        (!mask.empty() && !mask[index] &&
        (std::abs(knot.velocity(axis)) > 1.0e-9 || std::abs(knot.acceleration(axis)) > 1.0e-9)))
      {error = "Base trajectory knot violates pose, mask, velocity or acceleration limits"; return false;}
    }
    if (!i) {continue;}
    const auto & before = knots[i - 1];
    const double span = knot.time - before.time;
    for (Eigen::Index axis = 0; axis < 6; ++axis) {
      const auto c = baseHermitePolynomial(before.coordinates(axis), knot.coordinates(axis),
        before.velocity(axis), knot.velocity(axis), before.acceleration(axis), knot.acceleration(axis),
        span, before.has_velocity, before.has_acceleration);
      const auto positions = basePolynomialExtrema(c, 1);
      const auto velocities = basePolynomialExtrema(c, 2);
      const auto accelerations = basePolynomialExtrema(c, 3);
      if (positions.empty() || velocities.empty() || accelerations.empty())
      {error = "Could not compute base interpolation extrema"; return false;}
      for (const double u : positions) {
        const double value = evaluateBasePolynomial(c, u);
        if (value < lo(axis) - 1.0e-9 || value > hi(axis) + 1.0e-9)
        {error = "Base interpolation violates pose bounds or a locked axis"; return false;}
      }
      for (const double u : velocities) {
        if (std::abs(evaluateBasePolynomial(c, u, 1) / span) >
          velocity_limits[static_cast<size_t>(axis)] * (1.0 + 1.0e-6))
        {error = "Base interpolation violates a velocity limit"; return false;}
      }
      for (const double u : accelerations) {
        if (std::abs(evaluateBasePolynomial(c, u, 2) / (span * span)) >
          acceleration_limits[static_cast<size_t>(axis)] * (1.0 + 1.0e-6))
        {error = "Base interpolation violates an acceleration limit"; return false;}
      }
    }
  }
  return true;
}

inline bool validateBaseTrajectory(
  const trajectory_msgs::msg::MultiDOFJointTrajectory & trajectory, std::string & error)
{
  std::vector<BaseTrajectoryKnot> knots;
  return prepareBaseTrajectory(trajectory, knots, error);
}

inline bool sampleBaseTrajectory(
  const std::vector<BaseTrajectoryKnot> & knots, double elapsed,
  BaseTrajectorySample & sample, std::string & error)
{
  if (knots.empty() || !std::isfinite(elapsed))
  {error = "Cannot sample an empty base trajectory or invalid time"; return false;}
  if (elapsed <= knots.front().time || elapsed >= knots.back().time) {
    const auto & knot = elapsed <= knots.front().time ? knots.front() : knots.back();
    sample.coordinates = knot.coordinates;
    sample.velocity = elapsed < knots.front().time || elapsed > knots.back().time ?
      BaseCoordinates::Zero().eval() : knot.velocity;
    sample.acceleration = elapsed < knots.front().time || elapsed > knots.back().time ?
      BaseCoordinates::Zero().eval() : knot.acceleration;
    completeBaseSample(sample);
    return true;
  }
  const auto after = std::upper_bound(knots.begin(), knots.end(), elapsed,
    [](double time, const BaseTrajectoryKnot & knot) {return time < knot.time;});
  const auto & a = *(after - 1);
  const auto & b = *after;
  const double span = b.time - a.time, u = (elapsed - a.time) / span;
  for (Eigen::Index axis = 0; axis < 6; ++axis) {
    const auto c = baseHermitePolynomial(a.coordinates(axis), b.coordinates(axis),
      a.velocity(axis), b.velocity(axis), a.acceleration(axis), b.acceleration(axis),
      span, a.has_velocity, a.has_acceleration);
    sample.coordinates(axis) = evaluateBasePolynomial(c, u);
    sample.velocity(axis) = evaluateBasePolynomial(c, u, 1) / span;
    sample.acceleration(axis) = evaluateBasePolynomial(c, u, 2) / (span * span);
  }
  if (!sample.coordinates.allFinite() || !sample.velocity.allFinite() || !sample.acceleration.allFinite() ||
    std::abs(std::cos(sample.coordinates(4))) < 1.0e-4)
  {error = "Base interpolation is invalid or near an Euler singularity"; return false;}
  completeBaseSample(sample);
  return true;
}

inline bool sampleBaseTrajectory(
  const trajectory_msgs::msg::MultiDOFJointTrajectory & trajectory, double elapsed,
  BaseTrajectorySample & sample, std::string & error)
{
  std::vector<BaseTrajectoryKnot> knots;
  return prepareBaseTrajectory(trajectory, knots, error) && sampleBaseTrajectory(knots, elapsed, sample, error);
}

inline bool sampleBaseTrajectory(
  const trajectory_msgs::msg::MultiDOFJointTrajectory & trajectory, double elapsed,
  BaseCoordinates & coordinates, BaseCoordinates & velocity, BaseCoordinates & acceleration,
  std::string & error)
{
  BaseTrajectorySample sample;
  if (!sampleBaseTrajectory(trajectory, elapsed, sample, error)) {return false;}
  coordinates = sample.coordinates; velocity = sample.velocity; acceleration = sample.acceleration;
  return true;
}

}  // namespace sura_manipulation_actions
