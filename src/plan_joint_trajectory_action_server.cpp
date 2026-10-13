#include "sura_manipulation_actions/plan_joint_trajectory_action_server.hpp"

#include "controller_manager_msgs/srv/configure_controller.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <numeric>
#include <sstream>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "moveit/collision_detection/collision_common.h"
#include "moveit/planning_interface/planning_interface.h"
#include "moveit/robot_state/conversions.h"
#include "moveit_msgs/msg/joint_constraint.hpp"
#include "moveit_msgs/msg/motion_plan_request.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "visualization_msgs/msg/marker.hpp"
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

bool allFinite(const std::array<double, kAlphaJointCount> & values)
{
  return std::all_of(values.begin(), values.end(), [](double value) {
      return std::isfinite(value);
    });
}

builtin_interfaces::msg::Duration secondsToDuration(double seconds)
{
  builtin_interfaces::msg::Duration duration;
  duration.sec = static_cast<int32_t>(std::floor(seconds));
  duration.nanosec = static_cast<uint32_t>((seconds - duration.sec) * 1.0e9);
  return duration;
}

std::string normalizeArmName(const std::string & raw_name)
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

std::string normalizePoseName(const std::string & raw_name)
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

std::string contactName(const collision_detection::CollisionResult & result)
{
  if (!result.contacts.empty()) {
    const auto & pair = result.contacts.begin()->first;
    return pair.first + " <-> " + pair.second;
  }
  return "robot";
}
}  // namespace

PlanJointTrajectoryActionServer::PlanJointTrajectoryActionServer(
  const rclcpp::NodeOptions & options)
: LifecycleNode("plan_joint_trajectory_lifecycle_action_server", options)
{
}

PlanJointTrajectoryActionServer::~PlanJointTrajectoryActionServer()
{
  cleanupResources();
}

PlanJointTrajectoryActionServer::CallbackReturn
PlanJointTrajectoryActionServer::on_configure(const rclcpp_lifecycle::State &)
{
  robot_namespace_ = getOrDeclareParameter(*this, "robot_namespace", std::string("cirtesub"));
  moveit_namespace_ = getOrDeclareParameter(*this, "moveit_namespace", std::string("moveit"));
  action_name_ = getOrDeclareParameter(
    *this,
    "action_name",
    namespacedTopic("manipulation/plan_joint_trajectory"));
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
    namespacedTopic("manipulation/marker_plans"));
  left_group_name_ = getOrDeclareParameter(*this, "left_group_name", std::string("alpha_left"));
  right_group_name_ = getOrDeclareParameter(*this, "right_group_name", std::string("alpha_right"));
  left_controller_action_ = getOrDeclareParameter(
    *this,
    "left_controller_action",
    namespacedTopic("alpha/controller/alpha_left_joint_trajectory_controller/follow_joint_trajectory"));
  right_controller_action_ = getOrDeclareParameter(
    *this,
    "right_controller_action",
    namespacedTopic("alpha/controller/alpha_right_joint_trajectory_controller/follow_joint_trajectory"));
  controller_list_service_ = getOrDeclareParameter(
    *this,
    "controller_list_service",
    namespacedTopic("alpha/controller/controller_manager/list_controllers"));
  controller_switch_service_ = getOrDeclareParameter(
    *this,
    "controller_switch_service",
    namespacedTopic("alpha/controller/controller_manager/switch_controller"));
  vehicle_controller_list_service_ = getOrDeclareParameter(
    *this, "vehicle_controller_list_service", namespacedTopic("controller/controller_manager/list_controllers"));
  vehicle_controller_switch_service_ = getOrDeclareParameter(
    *this, "vehicle_controller_switch_service", namespacedTopic("controller/controller_manager/switch_controller"));
  body_velocity_controller_ = getOrDeclareParameter(*this, "body_velocity_controller", std::string("body_velocity"));
  left_velocity_controller_ = getOrDeclareParameter(
    *this,
    "left_velocity_controller",
    std::string("alpha_left_forward_velocity_controller"));
  right_velocity_controller_ = getOrDeclareParameter(
    *this,
    "right_velocity_controller",
    std::string("alpha_right_forward_velocity_controller"));
  left_trajectory_controller_ = getOrDeclareParameter(
    *this,
    "left_trajectory_controller",
    std::string("alpha_left_joint_trajectory_controller"));
  right_trajectory_controller_ = getOrDeclareParameter(
    *this,
    "right_trajectory_controller",
    std::string("alpha_right_joint_trajectory_controller"));
  task_priority_controller_ = getOrDeclareParameter(
    *this,
    "task_priority_controller",
    std::string("task_priority_controller"));
  execution_backend_ = getOrDeclareParameter(*this, "execution_backend", std::string("controller"));
  left_task_action_ = getOrDeclareParameter(
    *this,
    "left_task_action",
    namespacedTopic("controller/task_priority/tasks/left_joint_trajectory/follow_joint_trajectory"));
  right_task_action_ = getOrDeclareParameter(
    *this,
    "right_task_action",
    namespacedTopic("controller/task_priority/tasks/right_joint_trajectory/follow_joint_trajectory"));
  execute_service_name_ = getOrDeclareParameter(
    *this,
    "execute_service_name",
    std::string("~/execute_pending_trajectory"));
  cancel_service_name_ = getOrDeclareParameter(
    *this,
    "cancel_service_name",
    std::string("~/cancel_pending_trajectory"));
  planning_pipeline_namespace_ =
    getOrDeclareParameter(*this, "planning_pipeline_namespace", std::string("ompl"));
  planning_plugin_ =
    getOrDeclareParameter(*this, "planning_plugin", std::string("ompl_interface/OMPLPlanner"));
  predefined_poses_file_ =
    getOrDeclareParameter(*this, "predefined_poses_file", defaultPredefinedPosesFile());
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
  controller_switch_timeout_ = getOrDeclareParameter(*this, "controller_switch_timeout", 2.0);
  execution_feedback_rate_ = getOrDeclareParameter(*this, "execution_feedback_rate", 10.0);
  current_state_timeout_ = getOrDeclareParameter(*this, "current_state_timeout", 2.0);
  planning_scene_publish_rate_ =
    getOrDeclareParameter(*this, "planning_scene_publish_rate", 10.0);

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
  left_task_joint_names_ = getOrDeclareParameter<std::vector<std::string>>(
    *this,
    "left_task_joint_names",
    {
      "cirtesub/alpha_left/axis_b",
      "cirtesub/alpha_left/axis_c",
      "cirtesub/alpha_left/axis_d",
      "cirtesub/alpha_left/axis_e",
    });
  right_task_joint_names_ = getOrDeclareParameter<std::vector<std::string>>(
    *this,
    "right_task_joint_names",
    {
      "cirtesub/alpha_right/axis_b",
      "cirtesub/alpha_right/axis_c",
      "cirtesub/alpha_right/axis_d",
      "cirtesub/alpha_right/axis_e",
    });

  if (execution_backend_ != "controller" && execution_backend_ != "task") {
    RCLCPP_ERROR(
      get_logger(),
      "Invalid execution_backend '%s'. Expected 'controller' or 'task'.",
      execution_backend_.c_str());
    cleanupResources();
    return CallbackReturn::FAILURE;
  }

  std::string error_message;
  if (!loadPredefinedPoses(error_message)) {
    RCLCPP_ERROR(get_logger(), "Failed to configure manipulation action server: %s", error_message.c_str());
    cleanupResources();
    return CallbackReturn::FAILURE;
  }

  if (!initializeMoveIt(error_message)) {
    RCLCPP_ERROR(get_logger(), "Failed to configure manipulation action server: %s", error_message.c_str());
    cleanupResources();
    return CallbackReturn::FAILURE;
  }

  trajectory_executor_ = std::make_shared<TrajectoryExecutor>(this);
  list_controllers_client_ = create_client<ListControllers>(controller_list_service_);
  switch_controller_client_ = create_client<SwitchController>(controller_switch_service_);
  vehicle_list_client_ = create_client<ListControllers>(vehicle_controller_list_service_);
  vehicle_switch_client_ = create_client<SwitchController>(vehicle_controller_switch_service_);

  action_server_ = rclcpp_action::create_server<PlanJointTrajectory>(
    this,
    action_name_,
    std::bind(
      &PlanJointTrajectoryActionServer::handleGoal,
      this,
      std::placeholders::_1,
      std::placeholders::_2),
    std::bind(&PlanJointTrajectoryActionServer::handleCancel, this, std::placeholders::_1),
    std::bind(&PlanJointTrajectoryActionServer::handleAccepted, this, std::placeholders::_1));
  execute_service_ = create_service<Trigger>(
    execute_service_name_,
    std::bind(
      &PlanJointTrajectoryActionServer::handleExecuteTrigger,
      this,
      std::placeholders::_1,
      std::placeholders::_2));
  cancel_service_ = create_service<Trigger>(
    cancel_service_name_,
    std::bind(
      &PlanJointTrajectoryActionServer::handleCancelTrigger,
      this,
      std::placeholders::_1,
      std::placeholders::_2));

  RCLCPP_INFO(
    get_logger(),
    "Configured PlanJointTrajectory lifecycle action server on '%s'. Execute service='%s', cancel service='%s'.",
    action_name_.c_str(),
    execute_service_name_.c_str(),
    cancel_service_name_.c_str());
  return CallbackReturn::SUCCESS;
}

PlanJointTrajectoryActionServer::CallbackReturn
PlanJointTrajectoryActionServer::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(get_logger(), "PlanJointTrajectory lifecycle node activated");
  return CallbackReturn::SUCCESS;
}

PlanJointTrajectoryActionServer::CallbackReturn
PlanJointTrajectoryActionServer::on_deactivate(const rclcpp_lifecycle::State &)
{
  requestStop();
  joinExecutionThread();
  RCLCPP_INFO(get_logger(), "PlanJointTrajectory lifecycle node deactivated");
  return CallbackReturn::SUCCESS;
}

PlanJointTrajectoryActionServer::CallbackReturn
PlanJointTrajectoryActionServer::on_cleanup(const rclcpp_lifecycle::State &)
{
  cleanupResources();
  return CallbackReturn::SUCCESS;
}

PlanJointTrajectoryActionServer::CallbackReturn
PlanJointTrajectoryActionServer::on_shutdown(const rclcpp_lifecycle::State &)
{
  cleanupResources();
  return CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse PlanJointTrajectoryActionServer::handleGoal(
  const rclcpp_action::GoalUUID &,
  std::shared_ptr<const PlanJointTrajectory::Goal> goal)
{
  if (isPlanning()) {
    RCLCPP_WARN(get_logger(), "Rejecting goal because another trajectory is being planned");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (isExecuting()) {
    RCLCPP_WARN(get_logger(), "Rejecting goal because a trajectory is executing");
    return rclcpp_action::GoalResponse::REJECT;
  }

  ArmConfig arm;
  std::array<double, kAlphaJointCount> target{};
  std::string error_message;
  if (!validateGoal(*goal, arm, target, error_message)) {
    RCLCPP_WARN(get_logger(), "Rejecting goal: %s", error_message.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(
    get_logger(),
    "Accepted planning goal for '%s'. execute=%s.",
    arm.canonical_name.c_str(),
    goal->execute ? "true" : "false");
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlanJointTrajectoryActionServer::handleCancel(
  const std::shared_ptr<GoalHandlePlanJointTrajectory> goal_handle)
{
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (pending_goal_handle_ == goal_handle && !isExecuting()) {
      pending_goal_handle_.reset();
      pending_robot_trajectory_.reset();
      pending_joint_trajectory_ = trajectory_msgs::msg::JointTrajectory();
      has_pending_plan_ = false;
      pending_plan_id_.clear();
      finishPendingActionCancel(goal_handle, "Planned trajectory canceled before execution");
      return rclcpp_action::CancelResponse::ACCEPT;
    }
  }

  requestStop();
  RCLCPP_INFO(get_logger(), "Cancel requested for executing PlanJointTrajectory goal");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlanJointTrajectoryActionServer::handleExecuteTrigger(
  const std::shared_ptr<Trigger::Request>,
  std::shared_ptr<Trigger::Response> response)
{
  std::string message;
  response->success = startPendingExecution(message);
  response->message = message;
}

void PlanJointTrajectoryActionServer::handleCancelTrigger(
  const std::shared_ptr<Trigger::Request>,
  std::shared_ptr<Trigger::Response> response)
{
  std::string message;
  response->success = cancelPendingOrActive("Trajectory canceled by trigger service", message);
  response->message = message;
}

void PlanJointTrajectoryActionServer::handleAccepted(
  const std::shared_ptr<GoalHandlePlanJointTrajectory> goal_handle)
{
  if (!tryStartPlanning()) {
    auto result = std::make_shared<PlanJointTrajectory::Result>();
    result->success = false;
    result->message = "Another PlanJointTrajectory goal is already being planned";
    goal_handle->abort(result);
    return;
  }

  clearPendingPlan(true, "Replaced by newer trajectory goal");
  joinPlanningThread();
  planning_thread_ = std::thread{
    std::bind(&PlanJointTrajectoryActionServer::planAcceptedGoal, this, std::placeholders::_1),
    goal_handle};
}

void PlanJointTrajectoryActionServer::planAcceptedGoal(
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
    finishPlanning();
    return;
  }

  robot_trajectory::RobotTrajectoryPtr robot_trajectory;
  const double planning_time =
    goal_handle->get_goal()->planning_time > 0.0 ?
    goal_handle->get_goal()->planning_time :
    default_planning_time_;
  const double goal_tolerance =
    goal_handle->get_goal()->goal_tolerance > 0.0 ?
    goal_handle->get_goal()->goal_tolerance :
    default_goal_tolerance_;

  publishFeedback(goal_handle, arm, target, "planning", "Switching arm controllers for planning");
  const bool switched = execution_backend_ == "task" ?
    switchControllersForTaskExecution(error_message) :
    switchControllersForArm(arm, error_message);
  if (!switched) {
    result->success = false;
    result->message = error_message;
    goal_handle->abort(result);
    finishPlanning();
    return;
  }

  publishFeedback(goal_handle, arm, target, "planning", "Planning manipulation trajectory");
  if (goal_handle->is_canceling() || stop_requested_) {
    result->success = false;
    result->message = "Planning goal canceled";
    goal_handle->canceled(result);
    finishPlanning();
    return;
  }

  if (!planTrajectory(arm, target, planning_time, goal_tolerance, robot_trajectory, error_message)) {
    result->success = false;
    result->message = error_message;
    goal_handle->abort(result);
    finishPlanning();
    return;
  }

  const auto joint_trajectory = extractJointTrajectory(*robot_trajectory, arm);
  if (joint_trajectory.points.empty()) {
    result->success = false;
    result->message = "Planned trajectory is empty";
    goal_handle->abort(result);
    finishPlanning();
    return;
  }

  publishDisplayTrajectory(*robot_trajectory);
  const auto plan_id = "plan_" + std::to_string(next_plan_id_++);
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    pending_goal_handle_ = goal_handle;
    pending_arm_ = arm;
    pending_target_ = target;
    pending_robot_trajectory_ = robot_trajectory;
    pending_joint_trajectory_ = joint_trajectory;
    pending_plan_id_ = plan_id;
    has_pending_plan_ = true;
  }

  finishPlanning();
  if (goal_handle->get_goal()->execute) {
    std::string execute_message;
    if (!startPendingExecution(execute_message)) {
      result->success = false;
      result->message = execute_message;
      goal_handle->abort(result);
      clearPendingPlan(false, "");
    }
    return;
  }

  publishFeedback(
    goal_handle,
    arm,
    target,
    "waiting_for_execute",
    "Trajectory planned and waiting for execute trigger");
  RCLCPP_INFO(
    get_logger(),
    "Trajectory '%s' planned and published to RViz. Waiting for execute trigger.",
    plan_id.c_str());
}

void PlanJointTrajectoryActionServer::executePendingPlan()
{
  std::shared_ptr<GoalHandlePlanJointTrajectory> goal_handle;
  ArmConfig arm;
  std::array<double, kAlphaJointCount> target{};
  trajectory_msgs::msg::JointTrajectory joint_trajectory;
  std::string plan_id;

  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    goal_handle = pending_goal_handle_;
    arm = pending_arm_;
    target = pending_target_;
    joint_trajectory = pending_joint_trajectory_;
    plan_id = pending_plan_id_;
  }

  auto result = std::make_shared<PlanJointTrajectory::Result>();
  if (!goal_handle || joint_trajectory.points.empty()) {
    result->success = false;
    result->message = "No valid planned trajectory is available for execution";
    finishExecution();
    return;
  }

  publishFeedback(goal_handle, arm, target, "executing", "Executing approved manipulation trajectory");
  std::string error_message;
  double final_error_norm = std::numeric_limits<double>::infinity();
  if (!sendTrajectory(arm, joint_trajectory, goal_handle, target, error_message, final_error_norm)) {
    result->success = false;
    result->message = error_message;
    result->final_error_norm = final_error_norm;
    if (goal_handle->is_canceling() || stop_requested_) {
      goal_handle->canceled(result);
    } else {
      goal_handle->abort(result);
    }
    clearPendingPlan(false, "");
    finishExecution();
    return;
  }

  result->success = true;
  result->message = "Trajectory '" + plan_id + "' executed successfully";
  result->final_error_norm = final_error_norm;
  goal_handle->succeed(result);
  clearPendingPlan(false, "");
  finishExecution();
}

bool PlanJointTrajectoryActionServer::initializeMoveIt(std::string & error_message)
{
  std::lock_guard<std::mutex> lock(moveit_mutex_);
  if (planning_pipeline_ && planning_scene_monitor_) {
    return true;
  }

  try {
    moveit_node_ = std::make_shared<rclcpp::Node>(
      "plan_joint_trajectory_moveit_context",
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
      std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(moveit_node_, robot_model_loader_);
    if (!planning_scene_monitor_->getPlanningScene()) {
      error_message = "Failed to create MoveIt planning scene";
      return false;
    }

    moveit_executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    moveit_executor_->add_node(moveit_node_);
    moveit_spin_thread_ = std::thread{[this]() { moveit_executor_->spin(); }};

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
      rclcpp::QoS(10).reliable());

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

bool PlanJointTrajectoryActionServer::loadPredefinedPoses(std::string & error_message)
{
  predefined_poses_.clear();

  try {
    const YAML::Node root = YAML::LoadFile(predefined_poses_file_);
    const YAML::Node poses_root = root["predefined_poses"];
    if (!poses_root || !poses_root.IsMap()) {
      error_message =
        "Predefined poses file '" + predefined_poses_file_ +
        "' must contain a 'predefined_poses' map";
      return false;
    }

    for (const auto & arm_entry : poses_root) {
      const std::string arm_name = normalizeArmName(arm_entry.first.as<std::string>());
      const YAML::Node arm_poses = arm_entry.second;
      if (!arm_poses.IsMap()) {
        error_message =
          "Predefined poses for arm '" + arm_name + "' must be a map of pose names";
        return false;
      }

      auto & poses = predefined_poses_[arm_name];
      for (const auto & pose_entry : arm_poses) {
        const std::string pose_name = normalizePoseName(pose_entry.first.as<std::string>());
        const YAML::Node positions = pose_entry.second;
        if (!positions.IsSequence() || positions.size() != kAlphaJointCount) {
          error_message =
            "Predefined pose '" + pose_name + "' for arm '" + arm_name +
            "' must contain exactly 4 joint positions";
          return false;
        }

        std::array<double, kAlphaJointCount> target{};
        for (std::size_t i = 0; i < kAlphaJointCount; ++i) {
          target[i] = positions[i].as<double>();
        }
        if (!allFinite(target)) {
          error_message =
            "Predefined pose '" + pose_name + "' for arm '" + arm_name +
            "' contains non-finite joint positions";
          return false;
        }
        poses[pose_name] = target;
      }
    }
  } catch (const std::exception & ex) {
    error_message =
      "Could not load predefined poses file '" + predefined_poses_file_ + "': " + ex.what();
    return false;
  }

  RCLCPP_INFO(
    get_logger(),
    "Loaded predefined manipulation poses from '%s'",
    predefined_poses_file_.c_str());
  return true;
}

void PlanJointTrajectoryActionServer::copyParametersToMoveItNode()
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

std::optional<PlanJointTrajectoryActionServer::ArmConfig>
PlanJointTrajectoryActionServer::resolveArm(const std::string & arm_name) const
{
  const auto name = normalizeArmName(arm_name);
  if (name == "left" || name == "alpha_left" || name == "cirtesub/alpha_left") {
    return ArmConfig{"alpha_left", left_group_name_, left_controller_action_, left_joint_names_};
  }
  if (name == "right" || name == "alpha_right" || name == "cirtesub/alpha_right") {
    return ArmConfig{"alpha_right", right_group_name_, right_controller_action_, right_joint_names_};
  }
  return std::nullopt;
}

bool PlanJointTrajectoryActionServer::validateGoal(
  const PlanJointTrajectory::Goal & goal,
  ArmConfig & arm,
  std::array<double, kAlphaJointCount> & target,
  std::string & error_message) const
{
  const auto resolved_arm = resolveArm(goal.arm_name);
  if (!resolved_arm) {
    error_message =
      "Invalid arm_name '" + goal.arm_name +
      "'. Valid names are left, alpha_left, cirtesub/alpha_left, right, alpha_right, cirtesub/alpha_right";
    return false;
  }
  arm = *resolved_arm;
  if (arm.joint_names.size() != kAlphaJointCount) {
    error_message = "Configured joint list for " + arm.canonical_name + " must contain exactly 4 arm joints";
    return false;
  }

  const std::string pose_name = normalizePoseName(goal.predefined_pose);
  if (!pose_name.empty()) {
    const auto arm_poses = predefined_poses_.find(arm.canonical_name);
    if (arm_poses == predefined_poses_.end()) {
      error_message =
        "No predefined poses are configured for arm '" + arm.canonical_name + "'";
      return false;
    }

    const auto pose = arm_poses->second.find(pose_name);
    if (pose == arm_poses->second.end()) {
      error_message =
        "Unknown predefined_pose '" + goal.predefined_pose + "' for arm '" +
        arm.canonical_name + "'";
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

bool PlanJointTrajectoryActionServer::getCurrentState(
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

bool PlanJointTrajectoryActionServer::isStateCollisionFree(
  const moveit::core::RobotState & state,
  CollisionSummary & summary) const
{
  summary = collisionSummary(state);
  return !summary.in_collision;
}

PlanJointTrajectoryActionServer::CollisionSummary
PlanJointTrajectoryActionServer::collisionSummary(const moveit::core::RobotState & state) const
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

std::array<double, kAlphaJointCount> PlanJointTrajectoryActionServer::computeJointError(
  const moveit::core::RobotState & state,
  const ArmConfig & arm,
  const std::array<double, kAlphaJointCount> & target) const
{
  std::array<double, kAlphaJointCount> error{};
  for (std::size_t i = 0; i < kAlphaJointCount; ++i) {
    error[i] = target[i] - state.getVariablePosition(arm.joint_names[i]);
  }
  return error;
}

double PlanJointTrajectoryActionServer::computeErrorNorm(
  const std::array<double, kAlphaJointCount> & error) const
{
  const double squared_sum = std::inner_product(error.begin(), error.end(), error.begin(), 0.0);
  return std::sqrt(squared_sum / static_cast<double>(error.size()));
}

bool PlanJointTrajectoryActionServer::planTrajectory(
  const ArmConfig & arm,
  const std::array<double, kAlphaJointCount> & target,
  double planning_time,
  double goal_tolerance,
  robot_trajectory::RobotTrajectoryPtr & trajectory,
  std::string & error_message)
{
  moveit::core::RobotState start_state(planning_scene_monitor_->getRobotModel());
  if (!getCurrentState(start_state, error_message)) {
    return false;
  }

  const auto * joint_model_group = start_state.getJointModelGroup(arm.group_name);
  if (!joint_model_group) {
    error_message = "MoveIt group '" + arm.group_name + "' does not exist";
    return false;
  }

  const auto & variable_names = start_state.getVariableNames();
  for (const auto & joint_name : arm.joint_names) {
    if (std::find(variable_names.begin(), variable_names.end(), joint_name) == variable_names.end()) {
      error_message = "Joint '" + joint_name + "' does not exist in robot_description";
      return false;
    }
  }

  CollisionSummary start_collision;
  if (!isStateCollisionFree(start_state, start_collision)) {
    error_message = "Current state is in collision: " + start_collision.closest_object;
    return false;
  }

  moveit::core::RobotState goal_state(start_state);
  for (std::size_t i = 0; i < kAlphaJointCount; ++i) {
    goal_state.setVariablePosition(arm.joint_names[i], target[i]);
  }
  goal_state.update();

  if (!goal_state.satisfiesBounds(joint_model_group, 0.0)) {
    error_message = "Target joint_positions violate joint limits for group '" + arm.group_name + "'";
    return false;
  }

  CollisionSummary goal_collision;
  if (!isStateCollisionFree(goal_state, goal_collision)) {
    error_message = "Target state is in collision: " + goal_collision.closest_object;
    return false;
  }

  planning_interface::MotionPlanRequest request;
  moveit::core::robotStateToRobotStateMsg(start_state, request.start_state);
  request.group_name = arm.group_name;
  request.num_planning_attempts = 1;
  request.allowed_planning_time = planning_time;
  request.max_velocity_scaling_factor = 0.1;
  request.max_acceleration_scaling_factor = 0.1;

  moveit_msgs::msg::Constraints goal_constraints;
  goal_constraints.name = arm.canonical_name + "_joint_goal";
  for (std::size_t i = 0; i < kAlphaJointCount; ++i) {
    moveit_msgs::msg::JointConstraint constraint;
    constraint.joint_name = arm.joint_names[i];
    constraint.position = target[i];
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
        "MoveIt failed to plan for group '" + arm.group_name +
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

trajectory_msgs::msg::JointTrajectory PlanJointTrajectoryActionServer::extractJointTrajectory(
  const robot_trajectory::RobotTrajectory & trajectory,
  const ArmConfig & arm) const
{
  trajectory_msgs::msg::JointTrajectory msg;
  msg.header.stamp = now();
  msg.joint_names = arm.joint_names;

  for (std::size_t point_index = 0; point_index < trajectory.getWayPointCount(); ++point_index) {
    const auto & state = trajectory.getWayPoint(point_index);
    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.reserve(kAlphaJointCount);
    point.velocities.reserve(kAlphaJointCount);
    for (const auto & joint_name : arm.joint_names) {
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

bool PlanJointTrajectoryActionServer::sendTrajectory(
  const ArmConfig & arm,
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::shared_ptr<GoalHandlePlanJointTrajectory> goal_handle,
  const std::array<double, kAlphaJointCount> & target,
  std::string & error_message,
  double & final_error_norm)
{
  if (execution_backend_ == "task") {
    return sendTrajectoryToTask(
      arm, trajectory, goal_handle, target, error_message, final_error_norm);
  }
  return sendTrajectoryToController(
    arm, trajectory, goal_handle, target, error_message, final_error_norm);
}

bool PlanJointTrajectoryActionServer::sendTrajectoryToController(
  const ArmConfig & arm,
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::shared_ptr<GoalHandlePlanJointTrajectory> goal_handle,
  const std::array<double, kAlphaJointCount> & target,
  std::string & error_message,
  double & final_error_norm)
{
  return executeTransport(
    arm, trajectory, {arm.canonical_name, arm.controller_action, arm.joint_names},
    goal_handle, target, "Executing approved manipulation trajectory", error_message, final_error_norm);
}

bool PlanJointTrajectoryActionServer::sendTrajectoryToTask(
  const ArmConfig & arm,
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::shared_ptr<GoalHandlePlanJointTrajectory> goal_handle,
  const std::array<double, kAlphaJointCount> & target,
  std::string & error_message,
  double & final_error_norm)
{
  const bool left = arm.canonical_name == "alpha_left";
  const ExecutionTarget task{
    arm.canonical_name, left ? left_task_action_ : right_task_action_,
    left ? left_task_joint_names_ : right_task_joint_names_};
  return executeTransport(
    arm, trajectory, task, goal_handle, target,
    "Executing trajectory through task priority", error_message, final_error_norm);
}

bool PlanJointTrajectoryActionServer::executeTransport(
  const ArmConfig & arm,
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const ExecutionTarget & execution_target,
  const std::shared_ptr<GoalHandlePlanJointTrajectory> goal_handle,
  const std::array<double, kAlphaJointCount> & target,
  const std::string & feedback_message,
  std::string & error_message,
  double & final_error_norm)
{
  if (!trajectory_executor_) {
    error_message = "Trajectory transport is not configured";
    return false;
  }
  ExecutionOptions options;
  options.controller_wait_timeout = controller_wait_timeout_;
  options.acceptance_timeout = 5.0;
  options.start_delay = 0.0;
  options.execution_wait_margin = 5.0;
  options.feedback_rate = std::max(1.0, execution_feedback_rate_);
  const auto execution = trajectory_executor_->execute(
    trajectory, {execution_target}, options,
    [this, goal_handle]() { return stop_requested_.load() || goal_handle->is_canceling(); },
    [this, goal_handle, arm, target, feedback_message]() {
      publishFeedback(goal_handle, arm, target, "executing", feedback_message);
    });
  if (execution.accepted) {
    moveit::core::RobotState current_state(planning_scene_monitor_->getRobotModel());
    std::string state_error;
    if (getCurrentState(current_state, state_error)) {
      final_error_norm = computeErrorNorm(computeJointError(current_state, arm, target));
    }
  }
  error_message = execution.success ? std::string{} : execution.message;
  return execution.success;
}

bool PlanJointTrajectoryActionServer::switchControllersForArm(
  const ArmConfig & arm,
  std::string & error_message)
{
  if (!list_controllers_client_) {
    error_message = "Controller list client is not initialized";
    return false;
  }
  if (!switch_controller_client_) {
    error_message = "Controller switch client is not initialized";
    return false;
  }
  if (!list_controllers_client_->wait_for_service(
      std::chrono::duration<double>(controller_switch_timeout_)))
  {
    error_message = "Controller list service is unavailable: " + controller_list_service_;
    return false;
  }
  if (!switch_controller_client_->wait_for_service(
      std::chrono::duration<double>(controller_switch_timeout_)))
  {
    error_message = "Controller switch service is unavailable: " + controller_switch_service_;
    return false;
  }

  std::string trajectory_controller;
  std::string velocity_controller;
  if (arm.canonical_name == "alpha_left") {
    trajectory_controller = left_trajectory_controller_;
    velocity_controller = left_velocity_controller_;
  } else if (arm.canonical_name == "alpha_right") {
    trajectory_controller = right_trajectory_controller_;
    velocity_controller = right_velocity_controller_;
  } else {
    error_message = "Cannot switch controllers for unknown arm '" + arm.canonical_name + "'";
    return false;
  }

  auto list_request = std::make_shared<ListControllers::Request>();
  auto list_future = list_controllers_client_->async_send_request(list_request);
  if (list_future.wait_for(std::chrono::duration<double>(controller_switch_timeout_ + 0.5)) !=
    std::future_status::ready)
  {
    error_message = "Timed out listing controllers before switching '" + arm.canonical_name + "'";
    return false;
  }

  bool trajectory_exists = false;
  bool trajectory_active = false;
  bool velocity_exists = false;
  bool velocity_active = false;
  std::vector<std::string> conflicting_controllers;
  const auto list_response = list_future.get();
  for (const auto & controller : list_response->controller) {
    const std::string prefix = arm.canonical_name + "_";
    if (controller.state == "active" && controller.name != trajectory_controller &&
        (controller.name == task_priority_controller_ ||
         controller.name == prefix + "forward_position_controller" ||
         controller.name == prefix + "cartesian_velocity_controller")) {
      conflicting_controllers.push_back(controller.name);
    }
    if (controller.name == trajectory_controller) {
      trajectory_exists = true;
      trajectory_active = controller.state == "active";
    } else if (controller.name == velocity_controller) {
      velocity_exists = true;
      velocity_active = controller.state == "active";
    }
  }

  if (!trajectory_exists) {
    error_message = "Trajectory controller '" + trajectory_controller + "' is not loaded";
    return false;
  }
  if (!velocity_exists) {
    error_message = "Velocity controller '" + velocity_controller + "' is not loaded";
    return false;
  }

  auto request = std::make_shared<SwitchController::Request>();
  if (!trajectory_active) {
    request->activate_controllers.push_back(trajectory_controller);
  }
  request->deactivate_controllers = conflicting_controllers;
  if (velocity_active) {
    request->deactivate_controllers.push_back(velocity_controller);
  }

  if (request->activate_controllers.empty() && request->deactivate_controllers.empty()) {
    RCLCPP_DEBUG(
      get_logger(),
      "Controllers for '%s' are already in trajectory mode",
      arm.canonical_name.c_str());
    return true;
  }

  request->strictness = SwitchController::Request::STRICT;
  request->activate_asap = true;
  request->timeout = secondsToDuration(controller_switch_timeout_);

  auto future = switch_controller_client_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(controller_switch_timeout_ + 0.5)) !=
    std::future_status::ready)
  {
    error_message = "Timed out switching controllers for '" + arm.canonical_name + "'";
    return false;
  }

  const auto response = future.get();
  if (!response->ok) {
    error_message = "Controller manager rejected switch request for '" + arm.canonical_name + "'";
    return false;
  }
  return true;
}

bool PlanJointTrajectoryActionServer::switchControllersForTaskExecution(std::string & error_message)
{
  if (!list_controllers_client_) {
    error_message = "Controller list client is not initialized";
    return false;
  }
  if (!switch_controller_client_) {
    error_message = "Controller switch client is not initialized";
    return false;
  }
  if (!list_controllers_client_->wait_for_service(
      std::chrono::duration<double>(controller_switch_timeout_)))
  {
    error_message = "Controller list service is unavailable: " + controller_list_service_;
    return false;
  }
  if (!switch_controller_client_->wait_for_service(
      std::chrono::duration<double>(controller_switch_timeout_)))
  {
    error_message = "Controller switch service is unavailable: " + controller_switch_service_;
    return false;
  }

  auto list_request = std::make_shared<ListControllers::Request>();
  auto list_future = list_controllers_client_->async_send_request(list_request);
  if (list_future.wait_for(std::chrono::duration<double>(controller_switch_timeout_ + 0.5)) !=
    std::future_status::ready)
  {
    error_message = "Timed out listing controllers before switching to task execution";
    return false;
  }

  const std::vector<std::string> controllers_to_deactivate{
    left_velocity_controller_,
    right_velocity_controller_,
    left_trajectory_controller_,
    right_trajectory_controller_,
    "alpha_left_forward_position_controller", "alpha_right_forward_position_controller",
    "alpha_left_cartesian_velocity_controller", "alpha_right_cartesian_velocity_controller",
  };

  bool task_priority_exists = false;
  bool task_priority_active = false;
  bool task_priority_unconfigured = false;
  std::vector<std::string> active_to_deactivate;
  const auto list_response = list_future.get();
  for (const auto & controller : list_response->controller) {
    if (controller.name == task_priority_controller_) {
      task_priority_exists = true;
      task_priority_active = controller.state == "active";
      task_priority_unconfigured = controller.state == "unconfigured";
      continue;
    }
    if (controller.state != "active") {
      continue;
    }
    if (std::find(
        controllers_to_deactivate.begin(),
        controllers_to_deactivate.end(),
        controller.name) != controllers_to_deactivate.end())
    {
      active_to_deactivate.push_back(controller.name);
    }
  }

  if (!task_priority_exists) {
    error_message = "Task priority controller '" + task_priority_controller_ + "' is not loaded";
    return false;
  }

  if (task_priority_unconfigured) {
    const auto manager_prefix = controller_switch_service_.substr(0, controller_switch_service_.rfind('/'));
    auto configure_client = create_client<controller_manager_msgs::srv::ConfigureController>(
      manager_prefix + "/configure_controller");
    if (!configure_client->wait_for_service(std::chrono::duration<double>(controller_switch_timeout_))) {
      error_message = "Alpha configure_controller service is unavailable";
      return false;
    }
    auto configure_request = std::make_shared<controller_manager_msgs::srv::ConfigureController::Request>();
    configure_request->name = task_priority_controller_;
    auto configured = configure_client->async_send_request(configure_request);
    if (configured.wait_for(std::chrono::duration<double>(controller_wait_timeout_)) !=
        std::future_status::ready || !configured.get()->ok) {
      error_message = "Failed to configure task priority controller";
      return false;
    }
  }
  if (!vehicle_list_client_->wait_for_service(std::chrono::duration<double>(controller_switch_timeout_)) ||
      !vehicle_switch_client_->wait_for_service(std::chrono::duration<double>(controller_switch_timeout_))) {
    error_message = "Vehicle controller manager is unavailable";
    return false;
  }
  auto vehicle_future = vehicle_list_client_->async_send_request(std::make_shared<ListControllers::Request>());
  if (vehicle_future.wait_for(std::chrono::duration<double>(controller_switch_timeout_ + 0.5)) !=
      std::future_status::ready) {
    error_message = "Timed out listing vehicle controllers";
    return false;
  }
  std::vector<std::string> vehicle_deactivate;
  bool body_velocity_exists = false;
  bool body_velocity_active = false;
  const auto vehicle_response = vehicle_future.get();
  for (const auto & controller : vehicle_response->controller) {
    if (controller.name == body_velocity_controller_) {
      body_velocity_exists = true;
      body_velocity_active = controller.state == "active";
    } else if (controller.state == "active" &&
        (controller.name == "position_hold" || controller.name == "stabilize" ||
         controller.name == "depth_hold" || controller.name == "mpc_4dof")) {
      vehicle_deactivate.push_back(controller.name);
    }
  }
  if (!body_velocity_exists) {
    error_message = "Vehicle body_velocity controller is not loaded";
    return false;
  }
  const auto switch_vehicle = [this](const std::vector<std::string> & activate,
      const std::vector<std::string> & deactivate) {
      if (activate.empty() && deactivate.empty()) {return true;}
      auto change = std::make_shared<SwitchController::Request>();
      change->activate_controllers = activate;
      change->deactivate_controllers = deactivate;
      change->strictness = SwitchController::Request::STRICT;
      change->activate_asap = true;
      change->timeout = secondsToDuration(controller_switch_timeout_);
      auto result = vehicle_switch_client_->async_send_request(change);
      return result.wait_for(std::chrono::duration<double>(controller_switch_timeout_ + 0.5)) ==
             std::future_status::ready && result.get()->ok;
    };
  const std::vector<std::string> vehicle_activate = body_velocity_active ?
    std::vector<std::string>{} : std::vector<std::string>{body_velocity_controller_};
  if (!switch_vehicle(vehicle_activate, vehicle_deactivate)) {
    error_message = "Vehicle controller manager rejected task execution setup";
    return false;
  }
  bool committed = false;
  const auto rollback = std::shared_ptr<void>(nullptr, [&](void *) {
      if (!committed && !switch_vehicle(vehicle_deactivate, vehicle_activate)) {
        RCLCPP_ERROR(get_logger(), "Failed to restore vehicle controllers after Alpha activation failure");
      }
    });

  auto request = std::make_shared<SwitchController::Request>();
  if (!task_priority_active) {
    request->activate_controllers.push_back(task_priority_controller_);
  }
  request->deactivate_controllers = active_to_deactivate;

  if (request->activate_controllers.empty() && request->deactivate_controllers.empty()) {
    RCLCPP_DEBUG(get_logger(), "Task priority controller is already active for trajectory execution");
    committed = true;
    return true;
  }

  request->strictness = SwitchController::Request::STRICT;
  request->activate_asap = true;
  request->timeout = secondsToDuration(controller_switch_timeout_);

  auto future = switch_controller_client_->async_send_request(request);
  if (future.wait_for(std::chrono::duration<double>(controller_switch_timeout_ + 0.5)) !=
    std::future_status::ready)
  {
    // A timed-out request may still complete later. Confirm the producer is
    // stopped before restoring vehicle controllers from the previous mode.
    auto stop = std::make_shared<SwitchController::Request>();
    stop->deactivate_controllers = {task_priority_controller_};
    stop->strictness = SwitchController::Request::BEST_EFFORT;
    stop->timeout = secondsToDuration(controller_switch_timeout_);
    auto stopped = switch_controller_client_->async_send_request(stop);
    error_message = "Timed out switching controllers for task trajectory execution";
    if (stopped.wait_for(std::chrono::duration<double>(controller_switch_timeout_ + 0.5)) !=
        std::future_status::ready || !stopped.get()->ok) {
      committed = true;
      error_message += "; could not confirm Alpha stopped, vehicle rollback skipped";
    }
    return false;
  }

  const auto response = future.get();
  if (!response->ok) {
    error_message = "Controller manager rejected switch to task trajectory execution";
    return false;
  }
  committed = true;
  return true;
}

bool PlanJointTrajectoryActionServer::startPendingExecution(std::string & message)
{
  if (isPlanning()) {
    message = "Cannot execute while a trajectory is still being planned";
    return false;
  }
  if (!hasPendingPlan()) {
    message = "No pending trajectory is waiting for execute trigger";
    return false;
  }
  if (!tryStartExecution()) {
    message = "A trajectory is already executing";
    return false;
  }

  joinExecutionThread();
  execution_thread_ = std::thread{std::bind(&PlanJointTrajectoryActionServer::executePendingPlan, this)};
  message = "Started pending trajectory execution";
  return true;
}

bool PlanJointTrajectoryActionServer::cancelPendingOrActive(
  const std::string & message,
  std::string & response_message)
{
  if (isExecuting()) {
    requestStop();
    response_message = "Cancel requested for active trajectory execution";
    return true;
  }

  std::shared_ptr<GoalHandlePlanJointTrajectory> goal_handle;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    if (has_pending_plan_ && pending_goal_handle_) {
      goal_handle = pending_goal_handle_;
      pending_goal_handle_.reset();
      pending_robot_trajectory_.reset();
      pending_joint_trajectory_ = trajectory_msgs::msg::JointTrajectory();
      has_pending_plan_ = false;
      pending_plan_id_.clear();
    }
  }

  if (goal_handle) {
    auto result = std::make_shared<PlanJointTrajectory::Result>();
    result->success = false;
    result->message = message;
    goal_handle->abort(result);
    response_message = message;
    return true;
  }

  if (isPlanning()) {
    requestStop();
    response_message = "Cancel requested for active trajectory planning";
    return true;
  }

  response_message = "No pending or active trajectory to cancel";
  return false;
}

void PlanJointTrajectoryActionServer::finishPendingActionCancel(
  const std::shared_ptr<GoalHandlePlanJointTrajectory> goal_handle,
  const std::string & message) const
{
  const auto logger = get_logger();
  std::thread(
    [goal_handle, message, logger]() {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
      while (
        rclcpp::ok() && !goal_handle->is_canceling() &&
        std::chrono::steady_clock::now() < deadline)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }

      auto result = std::make_shared<PlanJointTrajectory::Result>();
      result->success = false;
      result->message = message;

      try {
        if (goal_handle->is_canceling()) {
          goal_handle->canceled(result);
          RCLCPP_INFO(logger, "Canceled pending planned trajectory");
        } else {
          goal_handle->abort(result);
          RCLCPP_WARN(
            logger,
            "Pending planned trajectory cancel did not reach canceling state; aborted goal instead");
        }
      } catch (const rclcpp::exceptions::RCLError & ex) {
        RCLCPP_ERROR(logger, "Failed to finish pending planned trajectory cancel: %s", ex.what());
      }
    }).detach();
}

bool PlanJointTrajectoryActionServer::publishFeedback(
  const std::shared_ptr<GoalHandlePlanJointTrajectory> goal_handle,
  const ArmConfig & arm,
  const std::array<double, kAlphaJointCount> & target,
  const std::string & state_label,
  const std::string & message) const
{
  moveit::core::RobotState state(planning_scene_monitor_->getRobotModel());
  std::string error_message;
  if (!getCurrentState(state, error_message)) {
    return false;
  }

  const auto collision = collisionSummary(state);
  const auto joint_error = computeJointError(state, arm, target);
  auto feedback = std::make_shared<PlanJointTrajectory::Feedback>();
  feedback->state = state_label;
  feedback->message = message;
  feedback->closest_distance = collision.closest_distance;
  feedback->closest_object = collision.closest_object;
  std::copy(joint_error.begin(), joint_error.end(), feedback->joint_error.begin());
  goal_handle->publish_feedback(feedback);
  return true;
}

void PlanJointTrajectoryActionServer::publishDisplayTrajectory(
  const robot_trajectory::RobotTrajectory & trajectory) const
{
  if (!display_trajectory_pub_ || trajectory.empty()) {
    return;
  }

  moveit_msgs::msg::DisplayTrajectory display;
  moveit::core::robotStateToRobotStateMsg(trajectory.getFirstWayPoint(), display.trajectory_start);
  moveit_msgs::msg::RobotTrajectory trajectory_msg;
  trajectory.getRobotTrajectoryMsg(trajectory_msg);
  display.trajectory.push_back(trajectory_msg);
  display_trajectory_pub_->publish(display);
}

void PlanJointTrajectoryActionServer::publishCollisionDebug(
  const CollisionSummary & summary,
  const std::string & frame_id) const
{
  if (!collision_debug_pub_) {
    return;
  }

  visualization_msgs::msg::MarkerArray markers;
  visualization_msgs::msg::Marker text;
  text.header.frame_id = frame_id;
  text.header.stamp = now();
  text.ns = "joint/collision_summary";
  text.id = 0;
  text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  text.action = visualization_msgs::msg::Marker::ADD;
  text.pose.orientation.w = 1.0;
  text.pose.position.z = 0.6;
  text.scale.z = 0.18;
  text.color.r = summary.in_collision ? 1.0F : 1.0F;
  text.color.g = summary.in_collision ? 0.0F : 0.8F;
  text.color.b = 0.0F;
  text.color.a = 1.0F;
  text.text = summary.in_collision ?
    "Collision: " + summary.closest_object :
    "Closest: " + summary.closest_object + " d=" + std::to_string(summary.closest_distance);
  markers.markers.push_back(text);
  collision_debug_pub_->publish(markers);
}

void PlanJointTrajectoryActionServer::requestStop()
{
  stop_requested_.store(true);
  if (planning_pipeline_) {
    planning_pipeline_->terminate();
  }
  if (trajectory_executor_) {
    trajectory_executor_->cancel();
  }
}

void PlanJointTrajectoryActionServer::finishExecution()
{
  stop_requested_.store(false);
  executing_.store(false);
}

void PlanJointTrajectoryActionServer::finishPlanning()
{
  stop_requested_.store(false);
  planning_.store(false);
}

bool PlanJointTrajectoryActionServer::tryStartExecution()
{
  bool expected = false;
  stop_requested_.store(false);
  return executing_.compare_exchange_strong(expected, true);
}

bool PlanJointTrajectoryActionServer::tryStartPlanning()
{
  bool expected = false;
  stop_requested_.store(false);
  return planning_.compare_exchange_strong(expected, true);
}

bool PlanJointTrajectoryActionServer::isExecuting() const
{
  return executing_.load();
}

bool PlanJointTrajectoryActionServer::isPlanning() const
{
  return planning_.load();
}

bool PlanJointTrajectoryActionServer::isInactive()
{
  return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE;
}

bool PlanJointTrajectoryActionServer::isActive()
{
  return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

bool PlanJointTrajectoryActionServer::hasPendingPlan() const
{
  std::lock_guard<std::mutex> lock(pending_mutex_);
  return has_pending_plan_ && pending_goal_handle_ && !pending_joint_trajectory_.points.empty();
}

void PlanJointTrajectoryActionServer::clearPendingPlan(bool finish_goal, const std::string & message)
{
  std::shared_ptr<GoalHandlePlanJointTrajectory> goal_handle;
  {
    std::lock_guard<std::mutex> lock(pending_mutex_);
    goal_handle = pending_goal_handle_;
    pending_goal_handle_.reset();
    pending_robot_trajectory_.reset();
    pending_joint_trajectory_ = trajectory_msgs::msg::JointTrajectory();
    has_pending_plan_ = false;
    pending_plan_id_.clear();
  }

  if (finish_goal && goal_handle) {
    auto result = std::make_shared<PlanJointTrajectory::Result>();
    result->success = false;
    result->message = message;
    goal_handle->abort(result);
  }
}

void PlanJointTrajectoryActionServer::scheduleDeactivateAfterGoal()
{
  std::thread{
    [this]()
    {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      if (!isActive()) {
        return;
      }
      try {
        this->deactivate();
      } catch (const std::exception & exception) {
        RCLCPP_WARN(
          get_logger(),
          "Failed to deactivate lifecycle node after trajectory execution: %s",
          exception.what());
      }
    }}.detach();
}

void PlanJointTrajectoryActionServer::joinExecutionThread()
{
  if (execution_thread_.joinable() && execution_thread_.get_id() != std::this_thread::get_id()) {
    execution_thread_.join();
  }
}

void PlanJointTrajectoryActionServer::joinPlanningThread()
{
  if (planning_thread_.joinable() && planning_thread_.get_id() != std::this_thread::get_id()) {
    planning_thread_.join();
  }
}

void PlanJointTrajectoryActionServer::cleanupResources()
{
  requestStop();
  joinPlanningThread();
  joinExecutionThread();
  clearPendingPlan(true, "Manipulation lifecycle node cleaned up");

  if (planning_scene_monitor_) {
    planning_scene_monitor_->stopPublishingPlanningScene();
    planning_scene_monitor_->stopSceneMonitor();
    planning_scene_monitor_->stopWorldGeometryMonitor();
    planning_scene_monitor_->stopStateMonitor();
  }
  planning_pipeline_.reset();
  planning_scene_monitor_.reset();
  robot_model_loader_.reset();
  display_trajectory_pub_.reset();
  collision_debug_pub_.reset();
  trajectory_executor_.reset();
  list_controllers_client_.reset();
  switch_controller_client_.reset();
  vehicle_list_client_.reset();
  vehicle_switch_client_.reset();
  execute_service_.reset();
  cancel_service_.reset();
  action_server_.reset();

  if (moveit_executor_) {
    moveit_executor_->cancel();
  }
  if (moveit_spin_thread_.joinable()) {
    moveit_spin_thread_.join();
  }
  if (moveit_executor_ && moveit_node_) {
    moveit_executor_->remove_node(moveit_node_);
  }
  moveit_executor_.reset();
  moveit_node_.reset();
  planning_.store(false);
  executing_.store(false);
  stop_requested_.store(false);
}

std::string PlanJointTrajectoryActionServer::namespacedTopic(const std::string & suffix) const
{
  const std::string ns = robot_namespace_.empty() ? "" : "/" + robot_namespace_;
  return ns + "/" + suffix;
}

std::string PlanJointTrajectoryActionServer::namespacedMoveItTopic(
  const std::string & suffix) const
{
  std::string ns = robot_namespace_.empty() ? "" : "/" + robot_namespace_;
  if (!moveit_namespace_.empty()) {
    ns += "/" + moveit_namespace_;
  }
  if (suffix.empty()) {
    return ns.empty() ? "/" : ns;
  }
  return ns + "/" + suffix;
}

}  // namespace sura_manipulation_actions

#ifndef SURA_MANIPULATION_ACTIONS_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sura_manipulation_actions::PlanJointTrajectoryActionServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}

#endif
