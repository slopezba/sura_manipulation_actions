#include "sura_manipulation_actions/cartesian_trajectory_timing.hpp"
#include "sura_manipulation_actions/base_trajectory_sampling.hpp"

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <stdexcept>

namespace sura_manipulation_actions::bimanual_cartesian::timing
{

double seconds(const builtin_interfaces::msg::Duration & time)
{
  return static_cast<double>(time.sec) + static_cast<double>(time.nanosec) * 1.0e-9;
}

builtin_interfaces::msg::Duration duration(double time)
{
  if (!std::isfinite(time) || time < 0.0 ||
    time > static_cast<double>(std::numeric_limits<int32_t>::max()) - 1.0)
  {throw std::runtime_error("Trajectory duration exceeds the ROS duration representation");}
  builtin_interfaces::msg::Duration out;
  const auto ns = static_cast<int64_t>(std::llround(time * 1.0e9));
  out.sec = static_cast<int32_t>(ns / 1000000000LL);
  out.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
  return out;
}

Polynomial polynomial(
  const trajectory_msgs::msg::JointTrajectoryPoint & a,
  const trajectory_msgs::msg::JointTrajectoryPoint & b,
  std::size_t joint, double span)
{
  const bool has_velocity = !a.velocities.empty() && !b.velocities.empty();
  const bool has_acceleration = !a.accelerations.empty() && !b.accelerations.empty();
  return baseHermitePolynomial(a.positions[joint], b.positions[joint],
    has_velocity ? a.velocities[joint] : 0.0, has_velocity ? b.velocities[joint] : 0.0,
    has_acceleration ? a.accelerations[joint] : 0.0, has_acceleration ? b.accelerations[joint] : 0.0,
    span, has_velocity, has_acceleration);
}

double evaluate(const Polynomial & c, double u, int derivative)
{
  return evaluateBasePolynomial(c, u, derivative);
}

// Extrema are checked analytically rather than inferred from temporal samples.
std::vector<double> extrema(const Polynomial & c, int derivative)
{
  return basePolynomialExtrema(c, derivative);
}

bool validMessage(const trajectory_msgs::msg::JointTrajectory & msg, std::string & error)
{
  if (msg.joint_names.empty() || msg.points.empty()) {
    error = "Joint trajectory is empty";
    return false;
  }
  if (std::set<std::string>(msg.joint_names.begin(), msg.joint_names.end()).size() != msg.joint_names.size()) {
    error = "Joint trajectory contains duplicate joint names";
    return false;
  }
  double previous = -1.0;
  for (const auto & point : msg.points) {
    const auto size = msg.joint_names.size();
    if (point.positions.size() != size ||
      (!point.velocities.empty() && point.velocities.size() != size) ||
      (!point.accelerations.empty() && point.accelerations.size() != size) ||
      (!point.accelerations.empty() && point.velocities.empty()))
    {
      error = "Joint trajectory derivative/position sizes are inconsistent";
      return false;
    }
    for (const auto * values : {&point.positions, &point.velocities, &point.accelerations}) {
      if (!std::all_of(values->begin(), values->end(), [](double x) {return std::isfinite(x);})) {
        error = "Joint trajectory contains non-finite values";
        return false;
      }
    }
    const double time = seconds(point.time_from_start);
    if (point.time_from_start.nanosec >= 1000000000U || time < 0.0 || time <= previous) {
      error = "Joint trajectory times must be non-negative and strictly increasing";
      return false;
    }
    previous = time;
  }
  return true;
}

void interpolateSegment(
  const trajectory_msgs::msg::JointTrajectoryPoint & a,
  const trajectory_msgs::msg::JointTrajectoryPoint & b,
  double u, double span, trajectory_msgs::msg::JointTrajectoryPoint & output)
{
  output.positions.resize(a.positions.size());
  output.velocities.resize(a.positions.size());
  output.accelerations.resize(a.positions.size());
  for (std::size_t j = 0; j < a.positions.size(); ++j) {
    const auto c = polynomial(a, b, j, span);
    output.positions[j] = evaluate(c, u);
    output.velocities[j] = evaluate(c, u, 1) / span;
    output.accelerations[j] = evaluate(c, u, 2) / (span * span);
  }
}


bool parameterize(
  robot_trajectory::RobotTrajectory & trajectory,
  const std::vector<std::string> & joint_names,
  double velocity_scaling, double acceleration_scaling, std::string & error)
{
  if (joint_names.empty() || trajectory.getWayPointCount() < 2 ||
    !std::isfinite(velocity_scaling) || velocity_scaling <= 0.0 || velocity_scaling > 1.0 ||
    !std::isfinite(acceleration_scaling) || acceleration_scaling <= 0.0 || acceleration_scaling > 1.0)
  {error = "Time parameterization needs joints, two waypoints and scaling in (0,1]"; return false;}
  if (std::set<std::string>(joint_names.begin(), joint_names.end()).size() != joint_names.size()) {
    error = "Time parameterization joint names must be unique"; return false;
  }
  const auto model = trajectory.getRobotModel();
  const auto & model_names = model->getVariableNames();
  for (const auto & name : joint_names) {
    if (std::find(model_names.begin(), model_names.end(), name) == model_names.end()) {
      error = "Unknown time parameterization joint: " + name; return false;
    }
    const auto & bounds = model->getVariableBounds(name);
    if (!bounds.velocity_bounded_ || !bounds.acceleration_bounded_ ||
      !std::isfinite(bounds.max_velocity_) || !std::isfinite(bounds.max_acceleration_) ||
      bounds.max_velocity_ <= 0.0 || bounds.max_acceleration_ <= 0.0)
    {error = "Time parameterization requires loaded velocity/acceleration bounds: " + name; return false;}
  }
  bool stationary = true;
  for (std::size_t i = 0; i < trajectory.getWayPointCount(); ++i) {
    for (const auto & name : joint_names) {
      const auto value = trajectory.getWayPoint(i).getVariablePosition(name);
      if (!std::isfinite(value)) {error = "Time parameterization position is not finite"; return false;}
      if (std::abs(value - trajectory.getFirstWayPoint().getVariablePosition(name)) > 1.0e-12) {
        stationary = false;
      }
    }
  }
  for (std::size_t i = 0; i < trajectory.getWayPointCount(); ++i) {
    for (const auto & name : joint_names) {
      trajectory.getWayPointPtr(i)->setVariableVelocity(name, 0.0);
      trajectory.getWayPointPtr(i)->setVariableAcceleration(name, 0.0);
    }
  }
  trajectory.setWayPointDurationFromPrevious(0, 0.0);
  if (stationary) {
    for (std::size_t i = 1; i < trajectory.getWayPointCount(); ++i) {
      trajectory.setWayPointDurationFromPrevious(i, 0.1);
    }
    return true;
  }
  try {
    trajectory_processing::IterativeParabolicTimeParameterization timing;
    if (!timing.computeTimeStamps(trajectory, velocity_scaling, acceleration_scaling)) {
      error = "MoveIt trajectory time parameterization failed"; return false;
    }
    for (std::size_t i = 1; i < trajectory.getWayPointCount(); ++i) {
      const double span = trajectory.getWayPointDurationFromPrevious(i);
      if (!std::isfinite(span) || span < 0.0) {
        error = "Time parameterization returned a non-finite or negative interval"; return false;
      }
      if (span > 1.0e-9) {continue;}
      for (const auto & name : joint_names) {
        if (std::abs(trajectory.getWayPoint(i).getVariablePosition(name) -
          trajectory.getWayPoint(i - 1).getVariablePosition(name)) > 1.0e-12)
        {error = "Time parameterization returned zero duration for a moving segment"; return false;}
      }
      // Repeated feasible poses are a real hold. Adjacent boundary derivatives
      // become zero; the calling planner rechecks all affected interpolation.
      trajectory.setWayPointDurationFromPrevious(i, 0.01);
      for (const auto & name : joint_names) {
        for (const auto index : {i - 1, i}) {
          trajectory.getWayPointPtr(index)->setVariableVelocity(name, 0.0);
          trajectory.getWayPointPtr(index)->setVariableAcceleration(name, 0.0);
        }
      }
    }
    (void)duration(trajectory.getDuration());
    return true;
  } catch (const std::exception & exception) {
    error = std::string("Time parameterization failed: ") + exception.what();
    return false;
  }
}

}  // namespace sura_manipulation_actions::bimanual_cartesian::timing
