#include "sura_manipulation_actions/plan_bimanual_joint_trajectory_action_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <numeric>
#include <sstream>
#include <unordered_set>

#include "builtin_interfaces/msg/duration.hpp"
#include "moveit/collision_detection/collision_common.h"
#include "moveit/planning_interface/planning_interface.h"
#include "moveit/robot_state/conversions.h"
#include "moveit_msgs/msg/joint_constraint.hpp"
#include "moveit_msgs/msg/motion_plan_request.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "visualization_msgs/msg/marker.hpp"

namespace sura_manipulation_actions
{
namespace
{
template<typename T, typename NodeT>
T getOrDeclareParameter(NodeT & node, const std::string & name, const T & default_value)
{
  if (node.has_parameter(name)) {
    return node.get_parameter(name).template get_value<T>();
  }
  return node.template declare_parameter<T>(name, default_value);
}

std::vector<std::string> splitAdapters(const std::string & adapters)
{
  std::istringstream stream(adapters);
  std::vector<std::string> result;
  std::string adapter;
  while (stream >> adapter) {
    result.push_back(adapter);
  }
  return result;
}

builtin_interfaces::msg::Duration secondsToDuration(double seconds)
{
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<int32_t>(std::floor(seconds));
  duration.nanosec = static_cast<uint32_t>((seconds - duration.sec) * 1.0e9);
  return duration;
}

std::string contactName(const collision_detection::CollisionResult & result)
{
  if (!result.contacts.empty()) {
    const auto & pair = result.contacts.begin()->first;
    return pair.first + " <-> " + pair.second;
  }
  return "robot";
}

bool allFinite(const std::vector<double> & values)
{
  return std::all_of(
    values.begin(), values.end(), [](double value) {
      return std::isfinite(value);
    });
}
}  // namespace

PlanBimanualJointTrajectoryActionServer::PlanBimanualJointTrajectoryActionServer(
  const rclcpp::NodeOptions & options)
: LifecycleNode("plan_bimanual_joint_trajectory_lifecycle_action_server", options)
{
}

PlanBimanualJointTrajectoryActionServer::~PlanBimanualJointTrajectoryActionServer()
{
  cleanupResources();
}

PlanBimanualJointTrajectoryActionServer::CallbackReturn
PlanBimanualJointTrajectoryActionServer::on_configure(const rclcpp_lifecycle::State &)
{
  robot_namespace_ = getOrDeclareParameter(*this, "robot_namespace", std::string("cirtesub"));
  moveit_namespace_ = getOrDeclareParameter(*this, "moveit_namespace", std::string("moveit"));
  action_name_ = getOrDeclareParameter(
    *this,
    "action_name",
    namespacedTopic("manipulation/plan_bimanual_joint_trajectory"));
  joint_states_topic_ = getOrDeclareParameter(
    *this,
    "joint_states_topic",
    namespacedTopic("alpha/joint_states"));
  display_trajectory_topic_ = getOrDeclareParameter(
    *this,
    "display_trajectory_topic",
    namespacedMoveItTopic("display_planned_path"));
  planning_scene_topic_ = getOrDeclareParameter(
    *this,
    "planning_scene_topic",
    namespacedMoveItTopic("monitored_planning_scene"));
  collision_debug_topic_ = getOrDeclareParameter(
    *this,
    "collision_debug_topic",
    namespacedTopic("manipulation/collision_debug"));
  group_name_ = getOrDeclareParameter(*this, "group_name", std::string("dual_alpha"));
  left_controller_action_ = getOrDeclareParameter(
    *this,
    "left_controller_action",
    namespacedTopic("alpha/controller/alpha_left_joint_trajectory_controller/follow_joint_trajectory"));
  right_controller_action_ = getOrDeclareParameter(
    *this,
    "right_controller_action",
    namespacedTopic("alpha/controller/alpha_right_joint_trajectory_controller/follow_joint_trajectory"));
  planning_pipeline_namespace_ =
    getOrDeclareParameter(*this, "planning_pipeline_namespace", std::string("ompl"));
  planning_plugin_ =
    getOrDeclareParameter(*this, "planning_plugin", std::string("ompl_interface/OMPLPlanner"));
  request_adapters_ = getOrDeclareParameter(
    *this,
    "request_adapters",
    std::string(
      "default_planner_request_adapters/AddTimeOptimalParameterization "
      "default_planner_request_adapters/FixWorkspaceBounds "
      "default_planner_request_adapters/FixStartStateBounds "
      "default_planner_request_adapters/FixStartStateCollision "
      "default_planner_request_adapters/FixStartStatePathConstraints"));
  default_planning_time_ = getOrDeclareParameter(*this, "default_planning_time", 5.0);
  default_goal_tolerance_ = getOrDeclareParameter(*this, "default_goal_tolerance", 0.02);
  controller_wait_timeout_ = getOrDeclareParameter(*this, "controller_wait_timeout", 5.0);
  current_state_timeout_ = getOrDeclareParameter(*this, "current_state_timeout", 2.0);
  planning_scene_publish_rate_ =
    getOrDeclareParameter(*this, "planning_scene_publish_rate", 10.0);
  execution_feedback_rate_ = getOrDeclareParameter(*this, "execution_feedback_rate", 10.0);
  left_joint_names_ = getOrDeclareParameter<std::vector<std::string>>(
    *this,
    "left_joint_names",
    {
      "cirtesub/alpha_left/axis_b",
      "cirtesub/alpha_left/axis_c",
      "cirtesub/alpha_left/axis_d",
      "cirtesub/alpha_left/axis_e",
    });
  right_joint_names_ = getOrDeclareParameter<std::vector<std::string>>(
    *this,
    "right_joint_names",
    {
      "cirtesub/alpha_right/axis_b",
      "cirtesub/alpha_right/axis_c",
      "cirtesub/alpha_right/axis_d",
      "cirtesub/alpha_right/axis_e",
    });

  std::string error_message;
  if (!initializeMoveIt(error_message)) {
    RCLCPP_ERROR(get_logger(), "Failed to configure bimanual planner: %s", error_message.c_str());
    cleanupResources();
    return CallbackReturn::FAILURE;
  }

  left_controller_client_ =
    rclcpp_action::create_client<FollowJointTrajectory>(this, left_controller_action_);
  right_controller_client_ =
    rclcpp_action::create_client<FollowJointTrajectory>(this, right_controller_action_);

  action_server_ = rclcpp_action::create_server<PlanBimanualJointTrajectory>(
    this,
    action_name_,
    std::bind(
      &PlanBimanualJointTrajectoryActionServer::handleGoal,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&PlanBimanualJointTrajectoryActionServer::handleCancel, this, std::placeholders::_1),
    std::bind(
      &PlanBimanualJointTrajectoryActionServer::handleAccepted,
      this,
      std::placeholders::_1));

  RCLCPP_INFO(
    get_logger(),
    "Configured PlanBimanualJointTrajectory lifecycle action server on '%s'. group='%s'.",
    action_name_.c_str(),
    group_name_.c_str());
  return CallbackReturn::SUCCESS;
}

PlanBimanualJointTrajectoryActionServer::CallbackReturn
PlanBimanualJointTrajectoryActionServer::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "PlanBimanualJointTrajectory lifecycle node activated");
  return CallbackReturn::SUCCESS;
}

PlanBimanualJointTrajectoryActionServer::CallbackReturn
PlanBimanualJointTrajectoryActionServer::on_deactivate(const rclcpp_lifecycle::State &)
{
  requestStop();
  joinWorkerThread();
  return CallbackReturn::SUCCESS;
}

PlanBimanualJointTrajectoryActionServer::CallbackReturn
PlanBimanualJointTrajectoryActionServer::on_cleanup(const rclcpp_lifecycle::State &)
{
  cleanupResources();
  return CallbackReturn::SUCCESS;
}

PlanBimanualJointTrajectoryActionServer::CallbackReturn
PlanBimanualJointTrajectoryActionServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  cleanupResources();
  return CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse PlanBimanualJointTrajectoryActionServer::handleGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const PlanBimanualJointTrajectory::Goal> goal)
{
  if (busy_) {
    RCLCPP_WARN(get_logger(), "Rejecting bimanual goal because another goal is active");
    return rclcpp_action::GoalResponse::REJECT;
  }

  std::string error_message;
  if (!validateGoal(*goal, error_message)) {
    RCLCPP_WARN(get_logger(), "Rejecting bimanual goal: %s", error_message.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlanBimanualJointTrajectoryActionServer::handleCancel(
  const std::shared_ptr<GoalHandlePlanBimanualJointTrajectory>)
{
  requestStop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlanBimanualJointTrajectoryActionServer::handleAccepted(
  const std::shared_ptr<GoalHandlePlanBimanualJointTrajectory> goal_handle)
{
  if (busy_.exchange(true)) {
    auto result = std::make_shared<PlanBimanualJointTrajectory::Result>();
    result->success = false;
    result->message = "Another PlanBimanualJointTrajectory goal is already active";
    goal_handle->abort(result);
    return;
  }

  joinWorkerThread();
  stop_requested_.store(false);
  worker_thread_ = std::thread{
    std::bind(&PlanBimanualJointTrajectoryActionServer::executeGoal, this, std::placeholders::_1),
    goal_handle};
}

void PlanBimanualJointTrajectoryActionServer::executeGoal(
  const std::shared_ptr<GoalHandlePlanBimanualJointTrajectory> goal_handle)
{
  auto result = std::make_shared<PlanBimanualJointTrajectory::Result>();
  const auto goal = goal_handle->get_goal();
  const double planning_time = goal->planning_time > 0.0 ?
    goal->planning_time : default_planning_time_;
  const double goal_tolerance = goal->goal_tolerance > 0.0 ?
    goal->goal_tolerance : default_goal_tolerance_;

  std::string error_message;
  robot_trajectory::RobotTrajectoryPtr robot_trajectory;
  publishFeedback(goal_handle, "planning", "Planning bimanual trajectory", CollisionSummary{});
  if (goal_handle->is_canceling() || stop_requested_) {
    result->success = false;
    result->message = "Planning goal canceled";
    goal_handle->canceled(result);
    busy_.store(false);
    return;
  }
  if (!planTrajectory(*goal, planning_time, goal_tolerance, robot_trajectory, error_message)) {
    result->success = false;
    result->message = error_message;
    goal_handle->abort(result);
    busy_.store(false);
    return;
  }

  result->trajectory = extractJointTrajectory(*robot_trajectory, goal->joint_names);
  if (result->trajectory.points.empty()) {
    result->success = false;
    result->message = "Planned bimanual trajectory is empty";
    goal_handle->abort(result);
    busy_.store(false);
    return;
  }
  publishDisplayTrajectory(*robot_trajectory);

  if (goal->execute) {
    publishFeedback(goal_handle, "executing", "Executing bimanual trajectory", CollisionSummary{});
    if (!executeTrajectory(result->trajectory, error_message)) {
      result->success = false;
      result->message = error_message;
      if (goal_handle->is_canceling() || stop_requested_) {
        goal_handle->canceled(result);
      } else {
        goal_handle->abort(result);
      }
      busy_.store(false);
      return;
    }
  }

  moveit::core::RobotState current_state(planning_scene_monitor_->getRobotModel());
  if (getCurrentState(current_state, error_message)) {
    result->final_error_norm =
      computeFinalErrorNorm(current_state, goal->joint_names, goal->joint_positions);
  } else {
    result->final_error_norm = std::numeric_limits<double>::infinity();
  }
  result->success = true;
  result->message = goal->execute ?
    "Bimanual trajectory planned and executed" :
    "Bimanual trajectory planned";
  goal_handle->succeed(result);
  busy_.store(false);
}

bool PlanBimanualJointTrajectoryActionServer::initializeMoveIt(std::string & error_message)
{
  std::lock_guard<std::mutex> lock(moveit_mutex_);
  if (planning_pipeline_ && planning_scene_monitor_) {
    return true;
  }

  try {
    moveit_node_ = std::make_shared<rclcpp::Node>(
      "plan_bimanual_joint_trajectory_moveit_context",
      namespacedMoveItTopic(""),
      rclcpp::NodeOptions().allow_undeclared_parameters(true));
    copyParametersToMoveItNode();

    robot_model_loader_ =
      std::make_shared<robot_model_loader::RobotModelLoader>(moveit_node_, "robot_description");
    const auto model = robot_model_loader_->getModel();
    if (!model) {
      error_message = "Failed to load MoveIt robot model from robot_description";
      return false;
    }

    planning_scene_monitor_ =
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(
      moveit_node_,
      robot_model_loader_);
    if (!planning_scene_monitor_->getPlanningScene()) {
      error_message = "Failed to create MoveIt planning scene";
      return false;
    }

    moveit_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    moveit_executor_->add_node(moveit_node_);
    moveit_spin_thread_ = std::thread{[this]() {moveit_executor_->spin();}};

    planning_scene_monitor_->startStateMonitor(joint_states_topic_);
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor();
    planning_scene_monitor_->setPlanningScenePublishingFrequency(planning_scene_publish_rate_);
    planning_scene_monitor_->startPublishingPlanningScene(
      planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE,
      planning_scene_topic_);

    display_trajectory_pub_ =
      moveit_node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      display_trajectory_topic_,
      rclcpp::QoS(1).reliable().transient_local());
    collision_debug_pub_ =
      moveit_node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      collision_debug_topic_,
      rclcpp::SystemDefaultsQoS());

    planning_pipeline_ = std::make_shared<planning_pipeline::PlanningPipeline>(
      model,
      moveit_node_,
      planning_pipeline_namespace_,
      planning_plugin_,
      splitAdapters(request_adapters_));
    planning_pipeline_->displayComputedMotionPlans(false);
    planning_pipeline_->publishReceivedRequests(false);
    planning_pipeline_->checkSolutionPaths(true);
  } catch (const std::exception & ex) {
    error_message = std::string("Failed to initialize MoveIt: ") + ex.what();
    return false;
  }

  return true;
}

void PlanBimanualJointTrajectoryActionServer::copyParametersToMoveItNode()
{
  const auto parameter_names = this->list_parameters({}, 10).names;
  for (const auto & name : parameter_names) {
    rclcpp::Parameter parameter;
    if (!this->get_parameter(name, parameter)) {
      continue;
    }
    try {
      if (!moveit_node_->has_parameter(name)) {
        moveit_node_->declare_parameter(name, parameter.get_parameter_value());
      } else {
        moveit_node_->set_parameter(parameter);
      }
    } catch (const std::exception & exception) {
      RCLCPP_DEBUG(
        get_logger(),
        "Could not copy parameter '%s' to MoveIt context node: %s",
        name.c_str(),
        exception.what());
    }
  }
}

bool PlanBimanualJointTrajectoryActionServer::validateGoal(
  const PlanBimanualJointTrajectory::Goal & goal,
  std::string & error_message) const
{
  if (goal.joint_names.empty()) {
    error_message = "joint_names must not be empty";
    return false;
  }
  if (goal.joint_names.size() != goal.joint_positions.size()) {
    error_message = "joint_names and joint_positions must have the same size";
    return false;
  }
  if (!allFinite(goal.joint_positions)) {
    error_message = "All joint_positions must be finite";
    return false;
  }
  if (goal.planning_time < 0.0 || goal.goal_tolerance < 0.0) {
    error_message = "planning_time and goal_tolerance must be non-negative";
    return false;
  }
  std::unordered_set<std::string> unique_names;
  for (const auto & joint_name : goal.joint_names) {
    if (joint_name.empty()) {
      error_message = "joint_names must not contain empty names";
      return false;
    }
    if (!unique_names.insert(joint_name).second) {
      error_message = "joint_names contains duplicate joint '" + joint_name + "'";
      return false;
    }
  }
  return true;
}

bool PlanBimanualJointTrajectoryActionServer::getCurrentState(
  moveit::core::RobotState & state,
  std::string & error_message) const
{
  if (!planning_scene_monitor_) {
    error_message = "MoveIt planning scene is not initialized";
    return false;
  }

  const auto start_time = now();
  while (rclcpp::ok() && (now() - start_time).seconds() < current_state_timeout_) {
    {
      planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
      if (scene) {
        state = scene->getCurrentState();
        state.update();
        return true;
      }
    }
    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }

  error_message = "Timed out waiting for current planning scene state";
  return false;
}

PlanBimanualJointTrajectoryActionServer::CollisionSummary
PlanBimanualJointTrajectoryActionServer::collisionSummary(
  const moveit::core::RobotState & state) const
{
  CollisionSummary summary;
  if (!planning_scene_monitor_) {
    summary.in_collision = true;
    summary.closest_distance = 0.0;
    summary.closest_object = "planning_scene_unavailable";
    return summary;
  }

  planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
  if (!scene) {
    summary.in_collision = true;
    summary.closest_distance = 0.0;
    summary.closest_object = "planning_scene_unavailable";
    return summary;
  }

  collision_detection::CollisionRequest request;
  collision_detection::CollisionResult result;
  request.contacts = true;
  request.max_contacts = 1;
  request.max_contacts_per_pair = 1;
  request.distance = true;
  scene->checkCollision(request, result, state);

  summary.in_collision = result.collision;
  summary.closest_distance = result.collision ? 0.0 : result.distance;
  summary.closest_object = result.collision ? contactName(result) : "robot";
  publishCollisionDebug(summary);
  return summary;
}

bool PlanBimanualJointTrajectoryActionServer::isStateCollisionFree(
  const moveit::core::RobotState & state,
  CollisionSummary & summary) const
{
  summary = collisionSummary(state);
  return !summary.in_collision;
}

bool PlanBimanualJointTrajectoryActionServer::planTrajectory(
  const PlanBimanualJointTrajectory::Goal & goal,
  double planning_time,
  double goal_tolerance,
  robot_trajectory::RobotTrajectoryPtr & trajectory,
  std::string & error_message)
{
  moveit::core::RobotState start_state(planning_scene_monitor_->getRobotModel());
  if (!getCurrentState(start_state, error_message)) {
    return false;
  }

  const auto * joint_model_group = start_state.getJointModelGroup(group_name_);
  if (!joint_model_group) {
    error_message = "MoveIt group '" + group_name_ + "' does not exist";
    return false;
  }

  const auto & variable_names = start_state.getVariableNames();
  const auto group_variable_names = joint_model_group->getVariableNames();
  for (const auto & joint_name : goal.joint_names) {
    if (std::find(
        variable_names.begin(), variable_names.end(),
        joint_name) == variable_names.end())
    {
      error_message = "Joint '" + joint_name + "' does not exist in robot_description";
      return false;
    }
    if (std::find(group_variable_names.begin(), group_variable_names.end(), joint_name) ==
      group_variable_names.end())
    {
      error_message =
        "Joint '" + joint_name + "' is not part of MoveIt group '" + group_name_ + "'";
      return false;
    }
  }

  CollisionSummary start_collision;
  if (!isStateCollisionFree(start_state, start_collision)) {
    error_message = "Current state is in collision: " + start_collision.closest_object;
    return false;
  }

  moveit::core::RobotState goal_state(start_state);
  for (std::size_t i = 0; i < goal.joint_names.size(); ++i) {
    goal_state.setVariablePosition(goal.joint_names[i], goal.joint_positions[i]);
  }
  goal_state.update();

  if (!goal_state.satisfiesBounds(joint_model_group, 0.0)) {
    error_message = "Target joint_positions violate joint limits for group '" + group_name_ + "'";
    return false;
  }

  CollisionSummary goal_collision;
  if (!isStateCollisionFree(goal_state, goal_collision)) {
    error_message = "Target state is in collision: " + goal_collision.closest_object;
    return false;
  }

  planning_interface::MotionPlanRequest request;
  moveit::core::robotStateToRobotStateMsg(start_state, request.start_state);
  request.group_name = group_name_;
  request.num_planning_attempts = 1;
  request.allowed_planning_time = planning_time;
  request.max_velocity_scaling_factor = 0.1;
  request.max_acceleration_scaling_factor = 0.1;

  moveit_msgs::msg::Constraints goal_constraints;
  goal_constraints.name = group_name_ + "_joint_goal";
  for (std::size_t i = 0; i < goal.joint_names.size(); ++i) {
    moveit_msgs::msg::JointConstraint constraint;
    constraint.joint_name = goal.joint_names[i];
    constraint.position = goal.joint_positions[i];
    constraint.tolerance_above = goal_tolerance;
    constraint.tolerance_below = goal_tolerance;
    constraint.weight = 1.0;
    goal_constraints.joint_constraints.push_back(constraint);
  }
  request.goal_constraints.push_back(goal_constraints);

  planning_interface::MotionPlanResponse response;
  {
    planning_scene_monitor::LockedPlanningSceneRO scene(planning_scene_monitor_);
    if (!scene) {
      error_message = "Planning scene is unavailable";
      return false;
    }
    const bool solved = planning_pipeline_->generatePlan(scene, request, response);
    if (!solved || response.error_code_.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
      error_message =
        "MoveIt failed to plan for group '" + group_name_ +
        "' with error code " + std::to_string(response.error_code_.val);
      return false;
    }
  }

  trajectory = response.trajectory_;
  if (!trajectory || trajectory->empty()) {
    error_message = "MoveIt returned an empty trajectory";
    return false;
  }
  return true;
}

trajectory_msgs::msg::JointTrajectory
PlanBimanualJointTrajectoryActionServer::extractJointTrajectory(
  const robot_trajectory::RobotTrajectory & trajectory,
  const std::vector<std::string> & joint_names) const
{
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp = now();
  msg.joint_names = joint_names;

  for (std::size_t point_index = 0; point_index < trajectory.getWayPointCount(); ++point_index) {
    const auto & state = trajectory.getWayPoint(point_index);
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.reserve(joint_names.size());
    point.velocities.reserve(joint_names.size());
    for (const auto & joint_name : joint_names) {
      point.positions.push_back(state.getVariablePosition(joint_name));
      point.velocities.push_back(state.getVariableVelocity(joint_name));
    }
    double time_from_start = trajectory.getWayPointDurationFromStart(point_index);
    if (point_index > 0 && time_from_start <= 0.0) {
      time_from_start = 0.5 * static_cast<double>(point_index);
    }
    point.time_from_start = secondsToDuration(time_from_start);
    msg.points.push_back(point);
  }
  return msg;
}

bool PlanBimanualJointTrajectoryActionServer::executeTrajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  std::string & error_message)
{
  const auto left = filterTrajectory(trajectory, left_joint_names_);
  const auto right = filterTrajectory(trajectory, right_joint_names_);
  if (left.points.empty() || right.points.empty()) {
    error_message = "Could not split bimanual trajectory into left/right controller trajectories";
    return false;
  }
  if (!sendTrajectoryToController(
      left_controller_client_, left_controller_action_, left,
      error_message))
  {
    return false;
  }
  if (!sendTrajectoryToController(
      right_controller_client_, right_controller_action_, right,
      error_message))
  {
    return false;
  }
  return true;
}

trajectory_msgs::msg::JointTrajectory PlanBimanualJointTrajectoryActionServer::filterTrajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::vector<std::string> & joint_names) const
{
  trajectory_msgs::msg::JointTrajectory filtered;
  filtered.header = trajectory.header;
  filtered.joint_names = joint_names;
  std::vector<std::size_t> source_indices;
  source_indices.reserve(joint_names.size());
  for (const auto & joint_name : joint_names) {
    const auto it = std::find(
      trajectory.joint_names.begin(),
      trajectory.joint_names.end(), joint_name);
    if (it == trajectory.joint_names.end()) {
      return trajectory_msgs::msg::JointTrajectory();
    }
    source_indices.push_back(
      static_cast<std::size_t>(std::distance(
        trajectory.joint_names.begin(),
        it)));
  }

  for (const auto & source_point : trajectory.points) {
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.time_from_start = source_point.time_from_start;
    for (const auto index : source_indices) {
      if (index >= source_point.positions.size()) {
        return trajectory_msgs::msg::JointTrajectory();
      }
      point.positions.push_back(source_point.positions[index]);
      if (index < source_point.velocities.size()) {
        point.velocities.push_back(source_point.velocities[index]);
      }
      if (index < source_point.accelerations.size()) {
        point.accelerations.push_back(source_point.accelerations[index]);
      }
    }
    filtered.points.push_back(point);
  }
  return filtered;
}

bool PlanBimanualJointTrajectoryActionServer::sendTrajectoryToController(
  const std::shared_ptr<rclcpp_action::Client<FollowJointTrajectory>> & client,
  const std::string & action_name,
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  std::string & error_message)
{
  if (!client->wait_for_action_server(std::chrono::duration<double>(controller_wait_timeout_))) {
    error_message = "Controller action server is unavailable: " + action_name;
    return false;
  }

  FollowJointTrajectory::Goal controller_goal;
  controller_goal.trajectory = trajectory;
  controller_goal.trajectory.header.stamp = now();
  auto goal_future = client->async_send_goal(controller_goal);
  if (goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    error_message = "Timed out sending trajectory to controller: " + action_name;
    return false;
  }

  const auto controller_goal_handle = goal_future.get();
  if (!controller_goal_handle) {
    error_message = "Controller rejected trajectory goal: " + action_name;
    return false;
  }

  rclcpp::Rate rate(std::max(1.0, execution_feedback_rate_));
  auto result_future = client->async_get_result(controller_goal_handle);
  while (rclcpp::ok()) {
    if (stop_requested_) {
      client->async_cancel_goal(controller_goal_handle);
      error_message = "Trajectory execution canceled";
      return false;
    }
    if (result_future.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
      const auto wrapped_result = result_future.get();
      if (wrapped_result.code == rclcpp_action::ResultCode::SUCCEEDED) {
        return true;
      }
      error_message =
        "Controller failed trajectory execution with result code " +
        std::to_string(static_cast<int>(wrapped_result.code));
      if (wrapped_result.result) {
        error_message += " and controller error code " +
          std::to_string(wrapped_result.result->error_code);
      }
      return false;
    }
    rate.sleep();
  }

  error_message = "ROS shutdown during trajectory execution";
  return false;
}

double PlanBimanualJointTrajectoryActionServer::computeFinalErrorNorm(
  const moveit::core::RobotState & state,
  const std::vector<std::string> & joint_names,
  const std::vector<double> & target) const
{
  if (joint_names.size() != target.size() || joint_names.empty()) {
    return std::numeric_limits<double>::infinity();
  }
  double squared_sum = 0.0;
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    const double error = target[i] - state.getVariablePosition(joint_names[i]);
    squared_sum += error * error;
  }
  return std::sqrt(squared_sum / static_cast<double>(joint_names.size()));
}

bool PlanBimanualJointTrajectoryActionServer::publishFeedback(
  const std::shared_ptr<GoalHandlePlanBimanualJointTrajectory> goal_handle,
  const std::string & state,
  const std::string & message,
  const CollisionSummary & collision) const
{
  if (!goal_handle) {
    return false;
  }
  auto feedback = std::make_shared<PlanBimanualJointTrajectory::Feedback>();
  feedback->state = state;
  feedback->message = message;
  feedback->closest_distance = collision.closest_distance;
  feedback->closest_object = collision.closest_object;
  goal_handle->publish_feedback(feedback);
  return true;
}

void PlanBimanualJointTrajectoryActionServer::publishDisplayTrajectory(
  const robot_trajectory::RobotTrajectory & trajectory) const
{
  if (!display_trajectory_pub_) {
    return;
  }
  moveit_msgs::msg::DisplayTrajectory display;
  moveit::core::robotStateToRobotStateMsg(trajectory.getFirstWayPoint(), display.trajectory_start);
  trajectory.getRobotTrajectoryMsg(display.trajectory.emplace_back());
  display_trajectory_pub_->publish(display);
}

void PlanBimanualJointTrajectoryActionServer::publishCollisionDebug(
  const CollisionSummary & summary,
  const std::string & frame_id) const
{
  if (!collision_debug_pub_) {
    return;
  }
  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = now();
  marker.ns = "bimanual_collision_state";
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.pose.position.z = 0.2;
  marker.scale.z = 0.06;
  marker.color.a = 1.0;
  marker.color.r = summary.in_collision ? 1.0 : 0.0;
  marker.color.g = summary.in_collision ? 0.0 : 1.0;
  marker.text =
    summary.in_collision ?
    "collision: " + summary.closest_object :
    "clear distance: " + std::to_string(summary.closest_distance);
  markers.markers.push_back(marker);
  collision_debug_pub_->publish(markers);
}

void PlanBimanualJointTrajectoryActionServer::requestStop()
{
  stop_requested_.store(true);
}

void PlanBimanualJointTrajectoryActionServer::joinWorkerThread()
{
  if (worker_thread_.joinable()) {
    worker_thread_.join();
  }
}

void PlanBimanualJointTrajectoryActionServer::cleanupResources()
{
  requestStop();
  joinWorkerThread();
  action_server_.reset();
  left_controller_client_.reset();
  right_controller_client_.reset();
  planning_pipeline_.reset();
  if (planning_scene_monitor_) {
    planning_scene_monitor_->stopPublishingPlanningScene();
    planning_scene_monitor_->stopWorldGeometryMonitor();
    planning_scene_monitor_->stopSceneMonitor();
    planning_scene_monitor_->stopStateMonitor();
    planning_scene_monitor_.reset();
  }
  display_trajectory_pub_.reset();
  collision_debug_pub_.reset();
  robot_model_loader_.reset();
  if (moveit_executor_) {
    moveit_executor_->cancel();
  }
  if (moveit_spin_thread_.joinable()) {
    moveit_spin_thread_.join();
  }
  moveit_executor_.reset();
  moveit_node_.reset();
  busy_.store(false);
  stop_requested_.store(false);
}

std::string PlanBimanualJointTrajectoryActionServer::namespacedTopic(
  const std::string & suffix) const
{
  if (robot_namespace_.empty()) {
    return "/" + suffix;
  }
  return "/" + robot_namespace_ + "/" + suffix;
}

std::string PlanBimanualJointTrajectoryActionServer::namespacedMoveItTopic(
  const std::string & suffix) const
{
  std::string base;
  if (!robot_namespace_.empty()) {
    base += "/" + robot_namespace_;
  }
  if (!moveit_namespace_.empty()) {
    base += "/" + moveit_namespace_;
  }
  if (suffix.empty()) {
    return base.empty() ? "/" : base;
  }
  return base + "/" + suffix;
}

}  // namespace sura_manipulation_actions

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node =
    std::make_shared<sura_manipulation_actions::PlanBimanualJointTrajectoryActionServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
