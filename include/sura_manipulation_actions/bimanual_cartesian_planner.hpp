#pragma once

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory.hpp>

#include <Eigen/Geometry>

#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace sura_manipulation_actions::bimanual_cartesian
{

struct Arm
{
  std::string name;
  std::string group_name;
  std::string tip_frame;
  std::string controller_action;
  std::string controller_name;
  std::string manager_service;
  std::vector<std::string> joint_names;
  // Optional group containing this arm and the floating vehicle root.
  std::string whole_body_group_name;
};

struct Pair
{
  std::vector<std::string> arm_names;
  std::string group_name;
  std::string task_action;
  std::string task_controller_name;
  std::string task_manager_service;
};

struct GoalSpec
{
  // Empty selects the model frame. The reference must not move with either arm.
  std::string reference_frame;
  // p = p_arm_b - p_arm_a, R = R_arm_b * R_arm_a^T in reference axes.
  Eigen::Isometry3d relative_target = Eigen::Isometry3d::Identity();
  Eigen::Vector3d relative_position_tolerance = Eigen::Vector3d::Zero();
  Eigen::Vector3d relative_orientation_tolerance = Eigen::Vector3d::Zero();
  double planning_time = 5.0;
  double cartesian_step = 0.005;
  // Single-arm order: world XYZ, world ZYX Euler roll/pitch/yaw, arm joints.
  // Empty fixes the base and permits every selected-arm articulation.
  std::vector<bool> available_dofs;
  // Absolute targets and nominal paths remain in reference_frame. Cartesian
  // tolerances are evaluated in these fixed, snapshot world axes.
  Eigen::Isometry3d world_from_tolerance = Eigen::Isometry3d::Identity();
  // Resolved absolute XYZ/RPY limits; empty means no additional pose bounds.
  std::vector<double> base_lower_limits;
  std::vector<double> base_upper_limits;
  std::vector<double> base_velocity_limits;
  std::vector<double> base_acceleration_limits;
};

struct Options
{
  int max_iterations = 100;
  int max_waypoints = 2000;
  int max_subdivision_depth = 6;
  int recovery_seeds = 8;
  int smoothing_passes = 2;
  int max_timing_refinements = 6;
  double angular_step = 0.05;
  double position_weight = 10.0;
  double orientation_weight = 2.0;
  double continuity_weight = 5.0;
  // Active arm variable order of the combined MoveIt group, independent of A/B.
  // Empty means zero additional movement penalties.
  std::vector<double> weights;
  double nominal_weight = 0.1;
  double smoothness_weight = 0.5;
  double damping = 0.001;
  double max_joint_step = 0.05;
  double max_prismatic_step = 0.005;
  double validation_joint_step = 0.01;
  double validation_prismatic_step = 0.001;
  double validation_time_step = 0.01;
  double velocity_scaling = 0.1;
  double acceleration_scaling = 0.1;
  double position_epsilon = 1.0e-6;
  double orientation_epsilon = 1.0e-6;
  bool optimize_trajectory = true;
  std::map<std::string, std::pair<double, double>> tightened_limits;
  std::map<std::string, double> nominal_positions;
  std::string base_joint_name = "world_joint";
  std::string base_frame = "cirtesub/base_link";
  std::vector<double> base_velocity_limits{0.2, 0.2, 0.2, 0.3, 0.3, 0.3};
  std::vector<double> base_acceleration_limits{0.2, 0.2, 0.2, 0.3, 0.3, 0.3};
  std::vector<double> base_weights{10.0, 10.0, 10.0, 20.0, 20.0, 10.0};
};

struct PlannedTrajectory
{
  robot_trajectory::RobotTrajectoryPtr trajectory;
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  trajectory_msgs::msg::MultiDOFJointTrajectory base_trajectory;
  Eigen::Isometry3d base_reference_pose = Eigen::Isometry3d::Identity();
  std::vector<bool> available_dofs;
  // One nominal-path parameter per final (possibly refined) trajectory waypoint.
  std::vector<double> progress_values;
  std::string reference_frame;
  Eigen::Isometry3d model_from_reference = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d initial_arm_a_pose = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d initial_relative_pose = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d relative_target = Eigen::Isometry3d::Identity();
};

class BimanualCartesianPlanner
{
public:
  using CancelCheck = std::function<bool()>;
  using Progress = std::function<void(double, const std::string &)>;

  bool plan(
    const planning_scene::PlanningSceneConstPtr & scene,
    const moveit::core::RobotState & start,
    const Arm & arm_a,
    const Arm & arm_b,
    const Pair & pair,
    const GoalSpec & goal,
    const Options & options,
    PlannedTrajectory & output,
    std::string & message,
    const CancelCheck & canceled = {},
    const Progress & progress = {}) const;

  bool validatePlan(
    const planning_scene::PlanningSceneConstPtr & scene,
    const Arm & arm_a,
    const Arm & arm_b,
    const Pair & pair,
    const GoalSpec & goal,
    const Options & options,
    const PlannedTrajectory & plan,
    std::string & message,
    const CancelCheck & canceled = {}) const;

  // Same boundary polynomials as joint_trajectory_controller: q/v/a -> quintic,
  // q/v -> cubic, positions -> linear. Times are seconds from trajectory start.
  static bool sampleTrajectory(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    double elapsed,
    trajectory_msgs::msg::JointTrajectoryPoint & sample,
    std::string & message);

  static Eigen::Isometry3d interpolateRelativePose(
    const Eigen::Isometry3d & initial,
    const Eigen::Isometry3d & target,
    double progress);

  static Eigen::Isometry3d relativePoseInReference(
    const Eigen::Isometry3d & model_from_arm_a,
    const Eigen::Isometry3d & model_from_arm_b,
    const Eigen::Isometry3d & model_from_reference);

  static Eigen::Vector3d orientationError(
    const Eigen::Matrix3d & target, const Eigen::Matrix3d & current);
};

}  // namespace sura_manipulation_actions::bimanual_cartesian
