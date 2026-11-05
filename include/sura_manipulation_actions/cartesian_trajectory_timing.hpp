#pragma once

#include <builtin_interfaces/msg/duration.hpp>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace sura_manipulation_actions::bimanual_cartesian::timing
{

// Coefficients in normalized segment time u in [0,1].
using Polynomial = std::array<double, 6>;

double seconds(const builtin_interfaces::msg::Duration & time);
builtin_interfaces::msg::Duration duration(double time);

// Validate a message before calling the per-segment helpers; span must be positive.
bool validMessage(const trajectory_msgs::msg::JointTrajectory & message, std::string & error);
Polynomial polynomial(
  const trajectory_msgs::msg::JointTrajectoryPoint & first,
  const trajectory_msgs::msg::JointTrajectoryPoint & second,
  std::size_t joint, double span);
double evaluate(const Polynomial & coefficients, double u, int derivative = 0);
std::vector<double> extrema(const Polynomial & coefficients, int derivative);
void interpolateSegment(
  const trajectory_msgs::msg::JointTrajectoryPoint & first,
  const trajectory_msgs::msg::JointTrajectoryPoint & second,
  double u, double span, trajectory_msgs::msg::JointTrajectoryPoint & output);

// Uses loaded robot bounds and MoveIt IPTP. Stationary segments receive real
// hold durations and zero boundary derivatives. Geometric/collision validation
// and quintic limit checks remain the calling planner's responsibility.
bool parameterize(
  robot_trajectory::RobotTrajectory & trajectory,
  const std::vector<std::string> & joint_names,
  double velocity_scaling,
  double acceleration_scaling,
  std::string & error);

}  // namespace sura_manipulation_actions::bimanual_cartesian::timing
