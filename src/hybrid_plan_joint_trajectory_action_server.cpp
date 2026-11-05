#include "sura_manipulation_actions/hybrid_plan_joint_trajectory_action_server.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <stdexcept>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "moveit_msgs/msg/joint_constraint.hpp"
#include "moveit_msgs/msg/motion_sequence_item.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "yaml-cpp/yaml.h"

namespace sura_manipulation_actions
{
namespace
{
constexpr std::size_t kAlphaJointCount = 4;

template<typename T, typename NodeT>
T getOrDeclareParameter(NodeT & node, const std::string & name, const T & default_value)
{
  if (node.has_parameter(name)) {
    return node.get_parameter(name).template get_value<T>();
  }
  return node.template declare_parameter<T>(name, default_value);
}

bool allFinite(const std::array<double, kAlphaJointCount> & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {
      return std::isfinite(value);
    });
}

std::string normalizeName(const std::string & raw_name)
{
  std::string name = raw_name;
  name.erase(
    std::remove_if(name.begin(), name.end(), [](unsigned char c) { return std::isspace(c); }),
    name.end());
  std::transform(name.begin(), name.end(), name.begin(), [](unsigned char c) {
      return static_cast<char>(std::tolower(c));
    });
  return name;
}

std::string defaultPredefinedPosesFile()
{
  try {
    return ament_index_cpp::get_package_share_directory("sura_manipulation_actions") +
           "/config/predefined_poses.yaml";
  } catch (const std::exception &) {
    return "config/predefined_poses.yaml";
  }
}
}  // namespace

HybridPlanJointTrajectoryActionServer::HybridPlanJointTrajectoryActionServer(
  const rclcpp::NodeOptions & options)
: Node("hybrid_plan_joint_trajectory_action_server", options)
{
  configure();
}

HybridPlanJointTrajectoryActionServer::~HybridPlanJointTrajectoryActionServer()
{
  cleanupResources();
}

void HybridPlanJointTrajectoryActionServer::configure()
{
  robot_namespace_ = getOrDeclareParameter(*this, "robot_namespace", std::string("cirtesub"));
  action_name_ = getOrDeclareParameter(
    *this, "action_name", namespacedTopic("manipulation/hybrid_plan_joint_trajectory"));
  left_group_name_ = getOrDeclareParameter(*this, "left_group_name", std::string("alpha_left"));
  right_group_name_ = getOrDeclareParameter(*this, "right_group_name", std::string("alpha_right"));
  left_hybrid_action_ = getOrDeclareParameter(
    *this, "left_hybrid_action", std::string("/hybrid_planning_left/run_hybrid_planning"));
  right_hybrid_action_ = getOrDeclareParameter(
    *this, "right_hybrid_action", std::string("/hybrid_planning_right/run_hybrid_planning"));
  execution_backend_ = getOrDeclareParameter(*this, "execution_backend", std::string("controller"));
  predefined_poses_file_ =
    getOrDeclareParameter(*this, "predefined_poses_file", defaultPredefinedPosesFile());
  default_planning_time_ = getOrDeclareParameter(*this, "default_planning_time", 5.0);
  default_goal_tolerance_ = getOrDeclareParameter(*this, "default_goal_tolerance", 0.02);
  hybrid_server_timeout_ = getOrDeclareParameter(*this, "hybrid_server_timeout", 5.0);
  hybrid_result_timeout_ = getOrDeclareParameter(*this, "hybrid_result_timeout", 15.0);
  max_velocity_scaling_factor_ =
    getOrDeclareParameter(*this, "max_velocity_scaling_factor", 0.05);
  max_acceleration_scaling_factor_ =
    getOrDeclareParameter(*this, "max_acceleration_scaling_factor", 0.05);

  left_joint_names_ = getOrDeclareParameter<std::vector<std::string>>(
    *this, "left_joint_names",
    {
      "cirtesub/alpha_left/axis_b",
      "cirtesub/alpha_left/axis_c",
      "cirtesub/alpha_left/axis_d",
      "cirtesub/alpha_left/axis_e",
    });
  right_joint_names_ = getOrDeclareParameter<std::vector<std::string>>(
    *this, "right_joint_names",
    {
      "cirtesub/alpha_right/axis_b",
      "cirtesub/alpha_right/axis_c",
      "cirtesub/alpha_right/axis_d",
      "cirtesub/alpha_right/axis_e",
    });

  if (execution_backend_ != "controller" && execution_backend_ != "task") {
    throw std::runtime_error(
            "Invalid execution_backend '" + execution_backend_ + "'. Expected 'controller' or 'task'.");
  }

  std::string error_message;
  if (!loadPredefinedPoses(error_message)) {
    throw std::runtime_error(error_message);
  }

  left_hybrid_client_ = rclcpp_action::create_client<HybridPlanner>(this, left_hybrid_action_);
  right_hybrid_client_ = rclcpp_action::create_client<HybridPlanner>(this, right_hybrid_action_);
  action_server_ = rclcpp_action::create_server<PlanJointTrajectory>(
    this,
    action_name_,
    std::bind(
      &HybridPlanJointTrajectoryActionServer::handleGoal,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&HybridPlanJointTrajectoryActionServer::handleCancel, this, std::placeholders::_1),
    std::bind(&HybridPlanJointTrajectoryActionServer::handleAccepted, this, std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "Hybrid PlanJointTrajectory action server ready on '%s'. Backend='%s', result_timeout=%.1fs.",
    action_name_.c_str(),
    execution_backend_.c_str(),
    hybrid_result_timeout_);
}

bool HybridPlanJointTrajectoryActionServer::loadPredefinedPoses(std::string & error_message)
{
  predefined_poses_.clear();
  try {
    const YAML::Node root = YAML::LoadFile(predefined_poses_file_);
    const YAML::Node poses_root = root["predefined_poses"];
    if (!poses_root || !poses_root.IsMap()) {
      error_message = "Predefined poses file must contain a 'predefined_poses' map";
      return false;
    }

    for (const auto & arm_entry : poses_root) {
      const std::string arm_name = normalizeName(arm_entry.first.as<std::string>());
      const YAML::Node arm_poses = arm_entry.second;
      if (!arm_poses.IsMap()) {
        error_message = "Predefined poses for arm '" + arm_name + "' must be a map";
        return false;
      }

      auto & poses = predefined_poses_[arm_name];
      for (const auto & pose_entry : arm_poses) {
        const std::string pose_name = normalizeName(pose_entry.first.as<std::string>());
        const YAML::Node positions = pose_entry.second;
        if (!positions.IsSequence() || positions.size() != kAlphaJointCount) {
          error_message = "Predefined pose '" + pose_name + "' must contain exactly 4 joints";
          return false;
        }

        std::array<double, kAlphaJointCount> target{};
        for (std::size_t i = 0; i < kAlphaJointCount; ++i) {
          target[i] = positions[i].as<double>();
        }
        if (!allFinite(target)) {
          error_message = "Predefined pose '" + pose_name + "' contains non-finite values";
          return false;
        }
        poses[pose_name] = target;
      }
    }
  } catch (const std::exception & ex) {
    error_message = "Could not load predefined poses file '" + predefined_poses_file_ + "': " + ex.what();
    return false;
  }
  return true;
}

std::optional<HybridPlanJointTrajectoryActionServer::ArmConfig>
HybridPlanJointTrajectoryActionServer::resolveArm(const std::string & arm_name) const
{
  const auto name = normalizeName(arm_name);
  if (name == "left" || name == "alpha_left" || name == "cirtesub/alpha_left") {
    return ArmConfig{"alpha_left", left_group_name_, left_hybrid_action_, left_joint_names_};
  }
  if (name == "right" || name == "alpha_right" || name == "cirtesub/alpha_right") {
    return ArmConfig{"alpha_right", right_group_name_, right_hybrid_action_, right_joint_names_};
  }
  return std::nullopt;
}

bool HybridPlanJointTrajectoryActionServer::validateGoal(
  const PlanJointTrajectory::Goal & goal,
  ArmConfig & arm,
  std::array<double, kAlphaJointCount> & target,
  std::string & error_message) const
{
  const auto resolved_arm = resolveArm(goal.arm_name);
  if (!resolved_arm) {
    error_message =
      "Invalid arm_name '" + goal.arm_name + "'. Valid names are left/right or alpha_left/alpha_right";
    return false;
  }
  arm = *resolved_arm;
  if (arm.joint_names.size() != kAlphaJointCount) {
    error_message = "Configured joint list for " + arm.canonical_name + " must contain exactly 4 joints";
    return false;
  }

  const std::string pose_name = normalizeName(goal.predefined_pose);
  if (!pose_name.empty()) {
    const auto arm_poses = predefined_poses_.find(arm.canonical_name);
    if (arm_poses == predefined_poses_.end()) {
      error_message = "No predefined poses are configured for arm '" + arm.canonical_name + "'";
      return false;
    }
    const auto pose = arm_poses->second.find(pose_name);
    if (pose == arm_poses->second.end()) {
      error_message = "Unknown predefined_pose '" + goal.predefined_pose + "'";
      return false;
    }
    target = pose->second;
  } else {
    std::copy(goal.joint_positions.begin(), goal.joint_positions.end(), target.begin());
  }

  if (!allFinite(target)) {
    error_message = "All joint_positions must be finite";
    return false;
  }
  if (goal.planning_time < 0.0 || goal.goal_tolerance < 0.0) {
    error_message = "planning_time and goal_tolerance must be non-negative";
    return false;
  }
  return true;
}

HybridPlanJointTrajectoryActionServer::HybridPlanner::Goal
HybridPlanJointTrajectoryActionServer::buildHybridGoal(
  const ArmConfig & arm,
  const std::array<double, kAlphaJointCount> & target,
  double planning_time,
  double goal_tolerance) const
{
  moveit_msgs::msg::MotionPlanRequest request;
  request.group_name = arm.group_name;
  request.pipeline_id = "ompl";
  request.num_planning_attempts = 1;
  request.allowed_planning_time = planning_time;
  request.max_velocity_scaling_factor = max_velocity_scaling_factor_;
  request.max_acceleration_scaling_factor = max_acceleration_scaling_factor_;

  moveit_msgs::msg::Constraints constraints;
  constraints.name = arm.canonical_name + "_joint_goal";
  for (std::size_t i = 0; i < kAlphaJointCount; ++i) {
    moveit_msgs::msg::JointConstraint constraint;
    constraint.joint_name = arm.joint_names[i];
    constraint.position = target[i];
    constraint.tolerance_above = goal_tolerance;
    constraint.tolerance_below = goal_tolerance;
    constraint.weight = 1.0;
    constraints.joint_constraints.push_back(constraint);
  }
  request.goal_constraints.push_back(constraints);

  moveit_msgs::msg::MotionSequenceItem item;
  item.req = request;
  item.blend_radius = 0.0;

  HybridPlanner::Goal hybrid_goal;
  hybrid_goal.planning_group = arm.group_name;
  hybrid_goal.motion_sequence.items.push_back(item);
  return hybrid_goal;
}

rclcpp_action::GoalResponse HybridPlanJointTrajectoryActionServer::handleGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const PlanJointTrajectory::Goal> goal)
{
  if (busy_) {
    RCLCPP_WARN(get_logger(), "Rejecting hybrid goal because another goal is running");
    return rclcpp_action::GoalResponse::REJECT;
  }

  ArmConfig arm;
  std::array<double, kAlphaJointCount> target{};
  std::string error_message;
  if (!validateGoal(*goal, arm, target, error_message)) {
    RCLCPP_WARN(get_logger(), "Rejecting hybrid goal: %s", error_message.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(
    get_logger(),
    "Accepted hybrid goal for '%s'. execute=%s is ignored because Hybrid Planning runs online.",
    arm.canonical_name.c_str(),
    goal->execute ? "true" : "false");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse HybridPlanJointTrajectoryActionServer::handleCancel(
  const std::shared_ptr<GoalHandlePlanJointTrajectory>)
{
  requestStop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void HybridPlanJointTrajectoryActionServer::handleAccepted(
  const std::shared_ptr<GoalHandlePlanJointTrajectory> goal_handle)
{
  if (busy_.exchange(true)) {
    auto result = std::make_shared<PlanJointTrajectory::Result>();
    result->success = false;
    result->message = "Another hybrid planning goal is already running";
    goal_handle->abort(result);
    return;
  }

  requestStop();
  joinWorkerThread();
  stop_requested_ = false;
  worker_thread_ = std::thread{
    std::bind(&HybridPlanJointTrajectoryActionServer::executeGoal, this, std::placeholders::_1),
    goal_handle};
}

void HybridPlanJointTrajectoryActionServer::executeGoal(
  const std::shared_ptr<GoalHandlePlanJointTrajectory> goal_handle)
{
  auto result = std::make_shared<PlanJointTrajectory::Result>();

  ArmConfig arm;
  std::array<double, kAlphaJointCount> target{};
  std::string error_message;
  if (!validateGoal(*goal_handle->get_goal(), arm, target, error_message)) {
    result->success = false;
    result->message = error_message;
    goal_handle->abort(result);
    busy_ = false;
    return;
  }

  const double planning_time = goal_handle->get_goal()->planning_time > 0.0 ?
    goal_handle->get_goal()->planning_time : default_planning_time_;
  const double goal_tolerance = goal_handle->get_goal()->goal_tolerance > 0.0 ?
    goal_handle->get_goal()->goal_tolerance : default_goal_tolerance_;
  const auto hybrid_goal = buildHybridGoal(arm, target, planning_time, goal_tolerance);

  RCLCPP_INFO(
    get_logger(),
    "Forwarding '%s' goal to MoveIt HybridPlanner action '%s'",
    arm.canonical_name.c_str(),
    arm.hybrid_action.c_str());
  if (!sendHybridGoal(arm, hybrid_goal, goal_handle, error_message)) {
    result->success = false;
    result->message = error_message;
    result->final_error_norm = 0.0;
    if (goal_handle->is_canceling() || stop_requested_) {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
    busy_ = false;
    return;
  }

  result->success = true;
  result->message = "MoveIt Hybrid Planning completed successfully";
  result->final_error_norm = 0.0;
  goal_handle->succeed(result);
  busy_ = false;
}

bool HybridPlanJointTrajectoryActionServer::sendHybridGoal(
  const ArmConfig & arm,
  const HybridPlanner::Goal & hybrid_goal,
  const std::shared_ptr<GoalHandlePlanJointTrajectory> & goal_handle,
  std::string & error_message)
{
  auto client = arm.canonical_name == "alpha_left" ? left_hybrid_client_ : right_hybrid_client_;
  if (!client || !client->wait_for_action_server(std::chrono::duration<double>(hybrid_server_timeout_))) {
    error_message = "Hybrid planner action server is unavailable: " + arm.hybrid_action;
    return false;
  }

  auto goal_future = client->async_send_goal(hybrid_goal);
  if (goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    error_message = "Timed out sending goal to hybrid planner: " + arm.hybrid_action;
    return false;
  }

  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    active_hybrid_goal_ = goal_future.get();
    active_hybrid_client_ = client;
  }
  if (!active_hybrid_goal_) {
    error_message = "Hybrid planner rejected goal: " + arm.hybrid_action;
    return false;
  }

  auto result_future = client->async_get_result(active_hybrid_goal_);
  const auto start_time = std::chrono::steady_clock::now();
  rclcpp::Rate rate(20.0);
  while (rclcpp::ok()) {
    if (stop_requested_ || goal_handle->is_canceling()) {
      client->async_cancel_goal(active_hybrid_goal_);
      error_message = "Hybrid planning canceled";
      return false;
    }

    const double elapsed =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count();
    if (hybrid_result_timeout_ > 0.0 && elapsed > hybrid_result_timeout_) {
      client->async_cancel_goal(active_hybrid_goal_);
      error_message =
        "Timed out waiting for MoveIt HybridPlanner result after " +
        std::to_string(hybrid_result_timeout_) + " seconds";
      return false;
    }

    if (result_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      const auto wrapped_result = result_future.get();
      if (wrapped_result.code != rclcpp_action::ResultCode::SUCCEEDED) {
        error_message =
          "Hybrid planner finished with result code " +
          std::to_string(static_cast<int>(wrapped_result.code));
        return false;
      }
      if (!wrapped_result.result) {
        error_message = "Hybrid planner returned no result";
        return false;
      }
      if (wrapped_result.result->error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
        error_message = wrapped_result.result->error_message;
        if (error_message.empty()) {
          error_message =
            "Hybrid planner failed with MoveIt error code " +
            std::to_string(wrapped_result.result->error_code.val);
        }
        return false;
      }
      return true;
    }
    rate.sleep();
  }

  error_message = "ROS shutdown during hybrid planning";
  return false;
}

void HybridPlanJointTrajectoryActionServer::requestStop()
{
  stop_requested_ = true;
  std::lock_guard<std::mutex> lock(active_goal_mutex_);
  if (active_hybrid_client_ && active_hybrid_goal_) {
    active_hybrid_client_->async_cancel_goal(active_hybrid_goal_);
  }
}

void HybridPlanJointTrajectoryActionServer::joinWorkerThread()
{
  if (worker_thread_.joinable() && worker_thread_.get_id() != std::this_thread::get_id()) {
    worker_thread_.join();
  }
}

void HybridPlanJointTrajectoryActionServer::cleanupResources()
{
  requestStop();
  joinWorkerThread();
  action_server_.reset();
  left_hybrid_client_.reset();
  right_hybrid_client_.reset();
  {
    std::lock_guard<std::mutex> lock(active_goal_mutex_);
    active_hybrid_client_.reset();
    active_hybrid_goal_.reset();
  }
  busy_ = false;
  stop_requested_ = false;
}

std::string HybridPlanJointTrajectoryActionServer::namespacedTopic(
  const std::string & suffix) const
{
  std::string prefix = robot_namespace_;
  if (!prefix.empty() && prefix.front() != '/') {
    prefix = "/" + prefix;
  }
  if (!prefix.empty() && prefix.back() == '/') {
    prefix.pop_back();
  }
  if (suffix.empty()) {
    return prefix.empty() ? "/" : prefix;
  }
  if (suffix.front() == '/') {
    return suffix;
  }
  return prefix + "/" + suffix;
}

}  // namespace sura_manipulation_actions

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node =
    std::make_shared<sura_manipulation_actions::HybridPlanJointTrajectoryActionServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
