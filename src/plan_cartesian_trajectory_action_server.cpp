#include "sura_manipulation_actions/plan_cartesian_trajectory_action_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <set>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "moveit/robot_model/revolute_joint_model.h"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "sura_manipulation_actions/base_trajectory_sampling.hpp"

namespace sura_manipulation_actions
{
namespace
{
template<typename T>
T parameter(rclcpp_lifecycle::LifecycleNode & node, const std::string & name, const T & fallback)
{
  if (node.has_parameter(name)) {
    const auto parameter = node.get_parameter(name);
    if constexpr (std::is_same_v<T, std::vector<std::string>> ||
      std::is_same_v<T, std::vector<double>>)
    {
      // ROS/launch can infer a different array type for an untyped YAML [].
      // An empty array still means no configured names/limits/nominal entries.
      const auto value = parameter.get_parameter_value().to_value_msg();
      if (parameter.get_type() >= rclcpp::ParameterType::PARAMETER_BYTE_ARRAY &&
        value.byte_array_value.empty() && value.bool_array_value.empty() &&
        value.integer_array_value.empty() && value.double_array_value.empty() &&
        value.string_array_value.empty()) {return T{};}
    }
    return parameter.get_value<T>();
  }
  return node.declare_parameter<T>(name, fallback);
}

Eigen::Vector3d vector(const geometry_msgs::msg::Vector3 & msg)
{
  return {msg.x, msg.y, msg.z};
}

std::vector<std::string> activeVariables(const moveit::core::JointModelGroup * group)
{
  std::vector<std::string> names;
  for (const auto * joint : group->getActiveJointModels()) {
    const auto & variables = joint->getVariableNames();
    names.insert(names.end(), variables.begin(), variables.end());
  }
  return names;
}

geometry_msgs::msg::PoseStamped stampedPose(
  const Eigen::Isometry3d & value, const std::string & frame)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = frame;
  result.pose.position.x = value.translation().x();
  result.pose.position.y = value.translation().y();
  result.pose.position.z = value.translation().z();
  const Eigen::Quaterniond q(value.rotation());
  result.pose.orientation.x = q.x(); result.pose.orientation.y = q.y();
  result.pose.orientation.z = q.z(); result.pose.orientation.w = q.w();
  return result;
}

bool finiteArray(const std::vector<double> & values, std::size_t size, bool positive = false)
{
  return values.size() == size && std::all_of(values.begin(), values.end(),
    [positive](double v) {return std::isfinite(v) && (!positive || v > 0.0);});
}

bool resolveTolerance(
  const std::vector<double> & input, const std::vector<double> & fallback,
  std::size_t size, std::vector<double> & output)
{
  output = input.empty() ? fallback : input;
  return finiteArray(output, size) && std::all_of(output.begin(), output.end(),
    [](double v) {return v >= 0.0;});
}
}  // namespace

PlanCartesianTrajectoryActionServer::PlanCartesianTrajectoryActionServer(
  const rclcpp::NodeOptions & options)
: LifecycleNode(
    "plan_cartesian_trajectory_lifecycle_action_server",
    rclcpp::NodeOptions(options).automatically_declare_parameters_from_overrides(true)) {}

PlanCartesianTrajectoryActionServer::~PlanCartesianTrajectoryActionServer()
{
  cleanupResources();
}

std::string PlanCartesianTrajectoryActionServer::topic(const std::string & suffix) const
{
  std::string prefix = robot_namespace_;
  while (!prefix.empty() && prefix.front() == '/') {prefix.erase(0, 1);}
  while (!prefix.empty() && prefix.back() == '/') {prefix.pop_back();}
  return (prefix.empty() ? std::string() : "/" + prefix) + "/" + suffix;
}

bool PlanCartesianTrajectoryActionServer::loadConfiguration(std::string & error)
{
  robot_namespace_ = parameter<std::string>(*this, "robot_namespace", "cirtesub");
  moveit_namespace_ = parameter<std::string>(*this, "moveit_namespace", "moveit");
  action_name_ = parameter<std::string>(*this, "action_name", topic("manipulation/plan_cartesian_trajectory"));
  execution_backend_ = parameter<std::string>(*this, "execution_backend", "controller");
  if (execution_backend_ != "controller" && execution_backend_ != "task" && execution_backend_ != "pose") {
    error = "execution_backend must be controller, task or pose"; return false;
  }
  fixed_frame_ = parameter<std::string>(*this, "visualization.fixed_frame", "");
  reference_frame_ = parameter<std::string>(*this, "reference_frame", "");
  reference_frame_ = parameter<std::string>(*this, "planning_frame",
    reference_frame_.empty() ? "world_ned" : reference_frame_);
  base_frame_ = parameter<std::string>(*this, "base_frame", topic("base_link").substr(1));
  coordinated_action_ = parameter<std::string>(*this, "execution.coordinated_action",
    topic("controller/task_priority/follow_cartesian_trajectory"));
  base_manager_service_ = parameter<std::string>(*this, "execution.base_manager_service",
    topic("controller/controller_manager/list_controllers"));
  base_controller_name_ = parameter<std::string>(*this, "execution.base_controller_name", "body_velocity");
  planning_time_ = parameter<double>(*this, "planning.default_planning_time", 5.0);
  cartesian_step_ = parameter<double>(*this, "planning.cartesian_step", 0.02);
  state_timeout_ = parameter<double>(*this, "current_state_timeout", 2.0);
  state_max_age_ = parameter<double>(*this, "current_state_max_age", 2.0);
  start_joint_tolerance_ = parameter<double>(*this, "execution.start_joint_tolerance", 0.001);
  start_prismatic_tolerance_ = parameter<double>(*this, "execution.start_prismatic_tolerance", 0.0001);
  base_position_tolerance_ = parameter<double>(*this, "execution.base_position_tolerance", 0.005);
  base_orientation_tolerance_ = parameter<double>(*this, "execution.base_orientation_tolerance", 0.01);
  pose_linear_velocity_ = parameter<double>(*this, "pose.max_linear_velocity", 1.0);
  pose_angular_velocity_ = parameter<double>(*this, "pose.max_angular_velocity", 0.5);
  pose_linear_acceleration_ = parameter<double>(*this, "pose.max_linear_acceleration", 0.4);
  pose_angular_acceleration_ = parameter<double>(*this, "pose.max_angular_acceleration", 0.2);
  const std::vector<double> positive{planning_time_, cartesian_step_, state_timeout_, state_max_age_,
    start_joint_tolerance_, start_prismatic_tolerance_, base_position_tolerance_, base_orientation_tolerance_,
    pose_linear_velocity_, pose_angular_velocity_, pose_linear_acceleration_, pose_angular_acceleration_};
  if (std::any_of(positive.begin(), positive.end(), [](double value) {return !std::isfinite(value) || value <= 0.0;})) {
    error = "Planning, state, and execution tolerance defaults must be finite and positive"; return false;
  }
  auto & p = planner_options_;
  p.max_iterations = static_cast<int>(parameter<int64_t>(*this, "planning.max_iterations", 100));
  p.max_waypoints = static_cast<int>(parameter<int64_t>(*this, "planning.max_waypoints", 2000));
  p.max_subdivision_depth = static_cast<int>(parameter<int64_t>(*this, "planning.max_subdivision_depth", 6));
  p.recovery_seeds = static_cast<int>(parameter<int64_t>(*this, "planning.recovery_seeds", 8));
  p.smoothing_passes = static_cast<int>(parameter<int64_t>(*this, "planning.smoothing_passes", 2));
  p.max_timing_refinements = static_cast<int>(parameter<int64_t>(*this, "planning.max_timing_refinements", 6));
  p.angular_step = parameter<double>(*this, "planning.angular_step", 0.05);
  p.position_weight = parameter<double>(*this, "planning.position_weight", 10.0);
  p.orientation_weight = parameter<double>(*this, "planning.orientation_weight", 2.0);
  p.continuity_weight = parameter<double>(*this, "planning.continuity_weight", 5.0);
  p.nominal_weight = parameter<double>(*this, "planning.nominal_weight", 0.1);
  p.smoothness_weight = parameter<double>(*this, "planning.smoothness_weight", 0.5);
  p.damping = parameter<double>(*this, "planning.damping", 0.001);
  p.max_joint_step = parameter<double>(*this, "planning.max_joint_step", 0.05);
  p.max_prismatic_step = parameter<double>(*this, "planning.max_prismatic_step", 0.005);
  p.validation_joint_step = parameter<double>(*this, "planning.validation_joint_step", 0.01);
  p.validation_prismatic_step = parameter<double>(*this, "planning.validation_prismatic_step", 0.001);
  p.validation_time_step = parameter<double>(*this, "planning.validation_time_step", 0.01);
  p.position_epsilon = parameter<double>(*this, "planning.position_epsilon", 1e-6);
  p.orientation_epsilon = parameter<double>(*this, "planning.orientation_epsilon", 1e-6);
  p.velocity_scaling = parameter<double>(*this, "planning.velocity_scaling", 0.8);
  p.acceleration_scaling = parameter<double>(*this, "planning.acceleration_scaling", 0.4);
  p.optimize_trajectory = parameter<bool>(*this, "planning.optimize_trajectory", true);
  p.base_joint_name = parameter<std::string>(*this, "planning.base_joint_name", "world_joint");
  p.base_frame = base_frame_;
  p.base_velocity_limits = parameter<std::vector<double>>(*this, "planning.base_velocity_limits", {1.0, 1.0, 1.0, 0.5, 0.5, 0.5});
  p.base_acceleration_limits = parameter<std::vector<double>>(*this, "planning.base_acceleration_limits", {0.4, 0.4, 0.4, 0.2, 0.2, 0.2});
  if (!finiteArray(p.base_velocity_limits, 6, true) || !finiteArray(p.base_acceleration_limits, 6, true)) {
    error = "Base velocity/acceleration defaults must contain six positive finite values"; return false;
  }
  auto & t = default_tracking_tolerances_;
  t.base_path = parameter<std::vector<double>>(*this, "execution.base_path_tolerance", {0.05, 0.05, 0.05, 0.3, 0.3, 0.3});
  t.base_goal = parameter<std::vector<double>>(*this, "execution.base_goal_tolerance", t.base_path);
  t.tip_path = parameter<std::vector<double>>(*this, "execution.tip_path_tolerance", {0.02, 0.02, 0.02, 0.3, 0.3, 0.3});
  t.tip_goal = parameter<std::vector<double>>(*this, "execution.tip_goal_tolerance", t.tip_path);
  for (const auto * values : {&t.base_path, &t.base_goal, &t.tip_path, &t.tip_goal}) {
    if (!finiteArray(*values, 6) || std::any_of(values->begin(), values->end(), [](double v) {return v < 0.0;})) {
      error = "Execution pose tolerances must contain six finite nonnegative values"; return false;
    }
  }
  const auto goal_time = parameter<double>(*this, "execution.goal_time_tolerance", 0.5);
  if (!std::isfinite(goal_time) || goal_time <= 0.0) {error = "Invalid goal convergence time"; return false;}
  t.goal_time_tolerance = rclcpp::Duration::from_seconds(goal_time);
  const auto limit_names = parameter<std::vector<std::string>>(*this, "planning.limit_joint_names", {});
  const auto lower = parameter<std::vector<double>>(*this, "planning.lower_limits", {});
  const auto upper = parameter<std::vector<double>>(*this, "planning.upper_limits", {});
  if (limit_names.size() != lower.size() || lower.size() != upper.size()) {
    error = "Named tightened joint limits have inconsistent lengths"; return false;
  }
  for (std::size_t i = 0; i < limit_names.size(); ++i) {
    if (!std::isfinite(lower[i]) || !std::isfinite(upper[i]) || lower[i] > upper[i] ||
      !p.tightened_limits.emplace(limit_names[i], std::make_pair(lower[i], upper[i])).second)
    {error = "Invalid or duplicate tightened joint limit: " + limit_names[i]; return false;}
  }
  const auto nominal_names = parameter<std::vector<std::string>>(*this, "planning.nominal_joint_names", {});
  const auto nominal = parameter<std::vector<double>>(*this, "planning.nominal_positions", {});
  if (nominal_names.size() != nominal.size()) {error = "Named nominal posture lengths differ"; return false;}
  for (std::size_t i = 0; i < nominal.size(); ++i) {
    if (!std::isfinite(nominal[i]) || !p.nominal_positions.emplace(nominal_names[i], nominal[i]).second) {
      error = "Invalid or duplicate nominal joint: " + nominal_names[i]; return false;
    }
  }
  execution_options_.controller_wait_timeout = parameter<double>(*this, "execution.controller_wait_timeout", 5.0);
  execution_options_.acceptance_timeout = parameter<double>(*this, "execution.acceptance_timeout", 2.0);
  execution_options_.start_delay = parameter<double>(*this, "execution.start_delay", 3.0);
  execution_options_.execution_wait_margin = parameter<double>(*this, "execution.wait_margin", 5.0);
  execution_options_.feedback_rate = parameter<double>(*this, "execution.feedback_rate", 10.0);
  const auto & execution = execution_options_;
  if (!std::isfinite(execution.controller_wait_timeout) || execution.controller_wait_timeout <= 0.0 ||
    !std::isfinite(execution.acceptance_timeout) || execution.acceptance_timeout <= 0.0 ||
    !std::isfinite(execution.start_delay) || execution.start_delay <= execution.acceptance_timeout ||
    !std::isfinite(execution.execution_wait_margin) || execution.execution_wait_margin <= 0.0 ||
    !std::isfinite(execution.feedback_rate) || execution.feedback_rate <= 0.0)
  {error = "Invalid execution timing; start_delay must exceed acceptance_timeout"; return false;}
  const auto arm_names = parameter<std::vector<std::string>>(*this, "arm_names", {});
  for (const auto & name : arm_names) {
    const auto prefix = "arms." + name + ".";
    bimanual_cartesian::Arm arm;
    arm.name = name;
    arm.group_name = parameter<std::string>(*this, prefix + "group", name);
    arm.whole_body_group_name = parameter<std::string>(*this, prefix + "whole_body_group", "base_" + arm.group_name);
    arm.tip_frame = parameter<std::string>(*this, prefix + "tip_frame", "");
    arm.controller_action = parameter<std::string>(*this, prefix + "controller_action", "");
    arm.controller_name = parameter<std::string>(*this, prefix + "controller_name", "");
    arm.manager_service = parameter<std::string>(*this, prefix + "manager_service", "");
    if (name.empty() || !arms_.emplace(name, arm).second) {error = "Empty or duplicate arm name"; return false;}
    tasks_[name] = {
      parameter<std::string>(*this, prefix + "task_action", ""),
      parameter<std::string>(*this, prefix + "task_controller_name", ""),
      parameter<std::string>(*this, prefix + "task_manager_service", "")};
    const auto & task = tasks_.at(name);
    auto controller_namespace = topic("controller");
    const auto manager_suffix = task.manager_service.rfind("/controller_manager/list_controllers");
    if (manager_suffix != std::string::npos) {
      controller_namespace = task.manager_service.substr(0, manager_suffix);
    }
    PoseEndpoint pose;
    pose.controller_node = parameter<std::string>(*this, prefix + "pose_controller_node",
      controller_namespace + "/" + (task.controller_name.empty() ? "task_priority_controller" : task.controller_name));
    pose.task_id = parameter<std::string>(*this, prefix + "pose_task",
      name.find("right") != std::string::npos ? "right_pose" :
      name.find("left") != std::string::npos ? "left_pose" :
      arms_.size() == 1 ? "left_pose" : "right_pose");
    const auto node_separator = pose.controller_node.rfind('/');
    const auto pose_namespace = node_separator == std::string::npos ? std::string() :
      pose.controller_node.substr(0, node_separator);
    const auto pose_prefix = pose_namespace + "/task_priority/tasks/" + pose.task_id;
    pose.target_topic = parameter<std::string>(*this, prefix + "pose_target_topic", pose_prefix + "/target");
    auto feedforward_prefix = pose.target_topic;
    if (feedforward_prefix.size() >= 7 && feedforward_prefix.compare(feedforward_prefix.size() - 7, 7, "/target") == 0) {
      feedforward_prefix.resize(feedforward_prefix.size() - 7);
    } else {feedforward_prefix = pose_prefix;}
    pose.feedforward_topic = parameter<std::string>(*this, prefix + "pose_feedforward_topic", feedforward_prefix + "/feedforward");
    if (pose.controller_node.empty() || pose.task_id.empty() || pose.target_topic.empty() || pose.feedforward_topic.empty()) {
      error = "Pose task endpoints cannot be empty: " + name; return false;
    }
    pose_endpoints_.emplace(name, std::move(pose));
  }
  if (arms_.empty()) {error = "Configure at least one arm"; return false;}
  return true;
}

PlanCartesianTrajectoryActionServer::CallbackReturn
PlanCartesianTrajectoryActionServer::on_configure(const rclcpp_lifecycle::State &)
{
  cleanupResources();
  std::string error;
  try {
    if (!loadConfiguration(error)) {throw std::runtime_error(error);}
    const auto moveit_prefix = topic(moveit_namespace_);
    moveit_node_ = std::make_shared<rclcpp::Node>(
      "single_arm_cartesian_moveit_context", moveit_prefix,
      rclcpp::NodeOptions().allow_undeclared_parameters(true));
    for (const auto & name : list_parameters({}, 20).names) {
      const auto value = get_parameter(name);
      if (!moveit_node_->has_parameter(name)) {
        moveit_node_->declare_parameter(name, value.get_parameter_value());
      } else {moveit_node_->set_parameter(value);}
    }
    context_ = std::make_unique<CartesianMoveItContext>(moveit_node_);
    if (!context_->start(
        parameter<std::string>(*this, "joint_states_topic", topic("alpha/joint_states")),
        parameter<std::string>(*this, "planning_scene_input_topic", moveit_prefix + "/planning_scene"),
        parameter<std::string>(*this, "planning_scene_topic", moveit_prefix + "/monitored_planning_scene"),
        parameter<std::string>(*this, "display_trajectory_topic", moveit_prefix + "/display_planned_path"),
        parameter<double>(*this, "planning_scene_publish_rate", 10.0), error))
    {throw std::runtime_error(error);}
    pose_tf_buffer_ = std::make_unique<tf2_ros::Buffer>(moveit_node_->get_clock());
    pose_tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*pose_tf_buffer_, moveit_node_, false);
    for (auto & entry : pose_endpoints_) {
      entry.second.target = moveit_node_->create_publisher<geometry_msgs::msg::PoseStamped>(
        entry.second.target_topic, rclcpp::QoS(1).reliable());
      entry.second.feedforward = moveit_node_->create_publisher<geometry_msgs::msg::TwistStamped>(
        entry.second.feedforward_topic, rclcpp::QoS(1).reliable());
    }
    const auto & model = context_->model();
    if (reference_frame_ != model->getModelFrame()) {
      throw std::runtime_error("planning_frame must equal the MoveIt model frame; express goals in any TF frame instead");
    }
    if (!model->hasLinkModel(base_frame_)) {base_frame_ = model->getRootLinkName();}
    planner_options_.base_frame = base_frame_;
    if (model->getRootJoint()->getType() == moveit::core::JointModel::FLOATING &&
      model->getRootJointName() != planner_options_.base_joint_name) {
      throw std::runtime_error("planning.base_joint_name must identify the floating model root");
    }
    const auto & variables = model->getVariableNames();
    for (const auto & entry : planner_options_.tightened_limits) {
      if (std::find(variables.begin(), variables.end(), entry.first) == variables.end()) {
        throw std::runtime_error("Unknown joint variable in tightened limits: " + entry.first);
      }
    }
    for (const auto & entry : planner_options_.nominal_positions) {
      if (std::find(variables.begin(), variables.end(), entry.first) == variables.end()) {
        throw std::runtime_error("Unknown joint variable in nominal posture: " + entry.first);
      }
    }
    for (auto & entry : arms_) {
      auto & arm = entry.second;
      const auto * group = model->getJointModelGroup(arm.group_name);
      if (!group || !group->isChain() || !model->hasLinkModel(arm.tip_frame) ||
        !group->getUpdatedLinkModelsSet().count(model->getLinkModel(arm.tip_frame)))
      {throw std::runtime_error("Invalid serial group or tip for arm '" + arm.name + "'");}
      for (const auto * joint : group->getActiveJointModels()) {
        if (joint->getVariableCount() != 1) {throw std::runtime_error("Only scalar arm joints are supported");}
        if (joint->isPassive()) {throw std::runtime_error("Arm group contains a passive joint: " + joint->getName());}
      }
      arm.joint_names = activeVariables(group);
      if (arm.joint_names.empty()) {throw std::runtime_error("Arm group contains no active joints: " + arm.name);}
      const auto metadata_name = "arms." + arm.name + ".joint_names";
      if (has_parameter(metadata_name) && get_parameter(metadata_name).as_string_array() != arm.joint_names) {
        throw std::runtime_error("Configured joint-name metadata differs from the MoveIt group: " + arm.name);
      }
      if (has_parameter(metadata_name) && !describe_parameter(metadata_name).read_only) {
        undeclare_parameter(metadata_name);
      }
      if (!has_parameter(metadata_name)) {
        rcl_interfaces::msg::ParameterDescriptor descriptor;
        descriptor.read_only = true;
        descriptor.description = "Canonical planning mask order, derived from the active MoveIt group";
        declare_parameter(metadata_name, arm.joint_names, descriptor);
      }
    }
    for (const auto & entry : arms_) {
      const auto * group = model->getJointModelGroup(entry.second.group_name);
      if (reference_frame_ != model->getModelFrame() &&
        group->getUpdatedLinkModelsSet().count(model->getLinkModel(reference_frame_)))
      {throw std::runtime_error("reference_frame must be independent of the selected arm");}
    }
    if (fixed_frame_.empty()) {fixed_frame_ = model->getModelFrame();}
    visualization_ = std::make_unique<CartesianTrajectoryVisualization>(
      moveit_node_, topic("cartesian_planner"), topic("manipulation"), fixed_frame_,
      parameter<bool>(*this, "visualization.publish_paths", true),
      parameter<bool>(*this, "visualization.publish_markers", true), true);
    executor_ = std::make_unique<TrajectoryExecutor>(this);
    clients_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    coordinated_client_ = rclcpp_action::create_client<FollowAction>(this, coordinated_action_);
    action_ = rclcpp_action::create_server<Action>(
      this, action_name_,
      std::bind(&PlanCartesianTrajectoryActionServer::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&PlanCartesianTrajectoryActionServer::handleCancel, this, std::placeholders::_1),
      std::bind(&PlanCartesianTrajectoryActionServer::handleAccepted, this, std::placeholders::_1));
    execute_service_ = create_service<std_srvs::srv::Trigger>(
      "~/execute_pending_trajectory",
      std::bind(&PlanCartesianTrajectoryActionServer::executeTrigger, this, std::placeholders::_1, std::placeholders::_2));
    cancel_service_ = create_service<std_srvs::srv::Trigger>(
      "~/cancel_pending_trajectory",
      std::bind(&PlanCartesianTrajectoryActionServer::cancelTrigger, this, std::placeholders::_1, std::placeholders::_2));
    cancel_timer_ = create_wall_timer(std::chrono::milliseconds(50), [this]() {
        std::shared_ptr<PendingPlan> pending;
        {std::lock_guard<std::mutex> lock(mutex_);
          if (stage_ == Stage::Pending && pending_ && pending_->handle->is_canceling()) {pending = pending_;}}
        if (pending) {finish(pending, false, "Pending plan canceled");}
      });
    stopped_.store(false);
    RCLCPP_INFO(get_logger(), "Configured '%s', execution backend '%s', manual activation", action_name_.c_str(), execution_backend_.c_str());
    return CallbackReturn::SUCCESS;
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Configuring Cartesian planner: %s", ex.what());
    cleanupResources();
    return CallbackReturn::FAILURE;
  }
}

PlanCartesianTrajectoryActionServer::CallbackReturn
PlanCartesianTrajectoryActionServer::on_activate(const rclcpp_lifecycle::State &)
{execution_active_.store(true); return CallbackReturn::SUCCESS;}

PlanCartesianTrajectoryActionServer::CallbackReturn
PlanCartesianTrajectoryActionServer::on_deactivate(const rclcpp_lifecycle::State &)
{execution_active_.store(false); stop("Cartesian planner deactivated"); return CallbackReturn::SUCCESS;}

PlanCartesianTrajectoryActionServer::CallbackReturn
PlanCartesianTrajectoryActionServer::on_cleanup(const rclcpp_lifecycle::State &)
{cleanupResources(); return CallbackReturn::SUCCESS;}

PlanCartesianTrajectoryActionServer::CallbackReturn
PlanCartesianTrajectoryActionServer::on_shutdown(const rclcpp_lifecycle::State &)
{cleanupResources(); return CallbackReturn::SUCCESS;}

bool PlanCartesianTrajectoryActionServer::validateGoal(
  const Action::Goal & goal, PendingPlan & pending, std::string & error) const
{
  const auto arm = arms_.find(goal.arm);
  if (arm == arms_.end()) {error = "Unknown arm: " + goal.arm; return false;}
  pending.arm = arm->second;
  pending.backend = goal.execution_backend.empty() ? execution_backend_ : goal.execution_backend;
  if (pending.backend != "controller" && pending.backend != "task" && pending.backend != "pose") {
    error = "execution_backend must be empty, controller, task or pose"; return false;
  }
  for (const auto scaling : {goal.velocity_scaling, goal.acceleration_scaling}) {
    if (!std::isfinite(scaling) || scaling < 0.0 || scaling > 1.0) {
      error = "velocity_scaling and acceleration_scaling must be finite in [0, 1]; zero uses server defaults";
      return false;
    }
  }
  // Capture the effective timing limits with the pending plan. Revalidation
  // must use these same per-goal limits when execution is requested later.
  pending.options = planner_options_;
  if (goal.velocity_scaling > 0.0) {pending.options.velocity_scaling = goal.velocity_scaling;}
  if (goal.acceleration_scaling > 0.0) {pending.options.acceleration_scaling = goal.acceleration_scaling;}
  auto & spec = pending.goal;
  spec.reference_frame = reference_frame_;
  if (pending.backend != "pose") {
    spec.available_dofs.assign(goal.available_dofs.begin(), goal.available_dofs.end());
    if (spec.available_dofs.empty()) {
      spec.available_dofs.assign(6 + pending.arm.joint_names.size(), true);
      std::fill_n(spec.available_dofs.begin(), 6, false);
    }
    if (spec.available_dofs.size() != 6 + pending.arm.joint_names.size()) {
      error = "available_dofs must contain six base entries plus every selected arm joint"; return false;
    }
    const bool moving_base = std::any_of(spec.available_dofs.begin(), spec.available_dofs.begin() + 6,
      [](bool value) {return value;});
    if (moving_base && goal.execution_backend.empty()) {pending.backend = "task";}
    if (moving_base && pending.backend != "task") {
      error = "Base motion requires execution_backend=task"; return false;
    }
    if (moving_base && (!context_->model()->getJointModelGroup(pending.arm.whole_body_group_name) ||
      context_->model()->getRootJoint()->getType() != moveit::core::JointModel::FLOATING)) {
      error = "Base motion requires a floating root and the configured base+arm planning group"; return false;
    }
    pending.base_limits = goal.base_limits;
    auto & limits = pending.base_limits;
    if (limits.lower_limits.empty() != limits.upper_limits.empty() ||
      (!limits.lower_limits.empty() && (!finiteArray(limits.lower_limits, 6) || !finiteArray(limits.upper_limits, 6)))) {
      error = "Base pose bounds must both be empty or contain six finite values"; return false;
    }
    for (std::size_t i = 0; i < limits.lower_limits.size(); ++i) {
      if (limits.lower_limits[i] > limits.upper_limits[i]) {error = "Base lower bound exceeds upper bound"; return false;}
    }
    if (limits.max_velocity.empty()) {limits.max_velocity = pending.options.base_velocity_limits;}
    if (limits.max_acceleration.empty()) {limits.max_acceleration = pending.options.base_acceleration_limits;}
    if (!finiteArray(limits.max_velocity, 6, true) || !finiteArray(limits.max_acceleration, 6, true)) {
      error = "Base dynamic limits must contain six positive finite values"; return false;
    }
    spec.base_velocity_limits = limits.max_velocity;
    spec.base_acceleration_limits = limits.max_acceleration;
  }
  auto & tol = pending.tracking_tolerances;
  const auto & requested = goal.tracking_tolerances;
  const auto & defaults = default_tracking_tolerances_;
  const auto joint_defaults = std::vector<double>(pending.arm.joint_names.size(), 0.05);
  if ((pending.backend != "pose" &&
    (!resolveTolerance(requested.base_path, defaults.base_path, 6, tol.base_path) ||
    !resolveTolerance(requested.base_goal, defaults.base_goal, 6, tol.base_goal) ||
    !resolveTolerance(requested.joint_path, joint_defaults, joint_defaults.size(), tol.joint_path) ||
    !resolveTolerance(requested.joint_goal, joint_defaults, joint_defaults.size(), tol.joint_goal))) ||
    !resolveTolerance(requested.tip_path, defaults.tip_path, 6, tol.tip_path) ||
    !resolveTolerance(requested.tip_goal, defaults.tip_goal, 6, tol.tip_goal)) {
    error = "Tracking tolerances must match their coordinate counts and be finite and nonnegative"; return false;
  }
  tol.goal_time_tolerance = requested.goal_time_tolerance;
  if (tol.goal_time_tolerance.sec < 0 || tol.goal_time_tolerance.nanosec >= 1000000000U) {
    error = "Invalid goal_time_tolerance"; return false;
  }
  if (tol.goal_time_tolerance.sec == 0 && tol.goal_time_tolerance.nanosec == 0) {
    tol.goal_time_tolerance = defaults.goal_time_tolerance;
  }
  const auto & pose = goal.target_pose.pose;
  Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
  spec.relative_target.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
  spec.relative_position_tolerance = vector(goal.position_tolerance);
  spec.relative_orientation_tolerance = vector(goal.orientation_tolerance);
  if (!quaternion.coeffs().allFinite() || std::abs(quaternion.norm() - 1.0) > 0.001 ||
    !spec.relative_target.translation().allFinite() ||
    !spec.relative_position_tolerance.allFinite() || (spec.relative_position_tolerance.array() < 0.0).any() ||
    !spec.relative_orientation_tolerance.allFinite() || (spec.relative_orientation_tolerance.array() < 0.0).any() ||
    !std::isfinite(goal.cartesian_step) || !std::isfinite(goal.planning_time))
  {error = "Pose must have a unit quaternion and finite coordinates; constraints must be finite and non-negative"; return false;}
  spec.relative_target.linear() = quaternion.normalized().toRotationMatrix();
  spec.cartesian_step = goal.cartesian_step > 0.0 ? goal.cartesian_step : cartesian_step_;
  spec.planning_time = goal.planning_time > 0.0 ? goal.planning_time : planning_time_;
  return true;
}

rclcpp_action::GoalResponse PlanCartesianTrajectoryActionServer::handleGoal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (stage_ == Stage::Planning || stage_ == Stage::Executing) {return rclcpp_action::GoalResponse::REJECT;}
  PendingPlan pending;
  std::string error;
  if (!validateGoal(*goal, pending, error)) {
    RCLCPP_WARN(get_logger(), "Rejected Cartesian goal: %s", error.c_str());
    return rclcpp_action::GoalResponse::REJECT;
  }
  stage_ = Stage::Planning;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlanCartesianTrajectoryActionServer::handleCancel(
  const std::shared_ptr<Handle> & handle)
{
  {std::lock_guard<std::mutex> lock(mutex_);
    if (!pending_ || pending_->handle != handle) {return rclcpp_action::CancelResponse::REJECT;}}
  stopped_.store(true);
  executor_->cancel();
  cancelCoordinated();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlanCartesianTrajectoryActionServer::handleAccepted(const std::shared_ptr<Handle> & handle)
{
  if (worker_.joinable()) {worker_.join();}
  auto pending = std::make_shared<PendingPlan>();
  std::string error;
  if (!validateGoal(*handle->get_goal(), *pending, error)) {
    pending->handle = handle;
    {std::lock_guard<std::mutex> lock(mutex_); stage_ = pending_ ? Stage::Pending : Stage::Idle;}
    finish(pending, false, error); return;
  }
  pending->handle = handle;
  std::shared_ptr<PendingPlan> previous;
  {std::lock_guard<std::mutex> lock(mutex_); previous = pending_; pending_ = pending; stage_ = Stage::Planning;}
  if (previous) {finish(previous, false, "Pending plan replaced by newer Cartesian goal");}
  stopped_.store(false);
  worker_ = std::thread([this, pending]() {planGoal(pending);});
}

void PlanCartesianTrajectoryActionServer::planGoal(const std::shared_ptr<PendingPlan> & pending)
{
  if (pending->backend == "pose") {planPoseGoal(pending); return;}
  try {
    const auto canceled = [this, pending]() {return stopped_.load() || pending->handle->is_canceling() || !pending->handle->is_active();};
    const auto & names = pending->arm.joint_names;
    planning_scene::PlanningScenePtr scene;
    std::string error;
    feedback(pending->handle, 0.0F, "acquiring_state", "Reading current joint states and planning scene");
    if (!context_->snapshot(names, state_timeout_, state_max_age_, scene, error, canceled) ||
      !visualization_->modelTransform(context_->model()->getModelFrame(), pending->fixed_from_model, error))
    {finish(pending, false, error); return;}
    auto input = pending->handle->get_goal()->target_pose;
    if (input.header.frame_id.empty()) {input.header.frame_id = reference_frame_;}
    if (!context_->transformTarget(scene, input, state_max_age_, pending->goal.relative_target,
        pending->goal.world_from_tolerance, error))
    {finish(pending, false, error); return;}
    pending->base_reference = scene->getCurrentState().getGlobalLinkTransform(base_frame_);
    pending->goal.base_lower_limits = pending->base_limits.lower_limits;
    pending->goal.base_upper_limits = pending->base_limits.upper_limits;
    for (std::size_t i = 0; i < pending->goal.base_lower_limits.size() && i < 3; ++i) {
      pending->goal.base_lower_limits[i] += pending->base_reference.translation()[static_cast<Eigen::Index>(i)];
      pending->goal.base_upper_limits[i] += pending->base_reference.translation()[static_cast<Eigen::Index>(i)];
    }
    const bimanual_cartesian::SingleArmCartesianPlanner planner;
    if (!planner.plan(scene, scene->getCurrentState(), pending->arm,
        pending->goal, pending->options, pending->plan, error, canceled,
        [this, pending](double value, const std::string & stage) {
          feedback(pending->handle, static_cast<float>(0.1 + 0.7 * value), stage, "Planning single-arm Cartesian trajectory");
        }))
    {finish(pending, false, error); return;}
    if (canceled()) {finish(pending, false, "Planning canceled"); return;}
    // Fixed-root models still expose a constant base reference for the common
    // execution interface; floating models provide their planned base samples.
    if (pending->plan.base_trajectory.points.empty()) {
      pending->plan.base_trajectory.header.frame_id = reference_frame_;
      pending->plan.base_trajectory.joint_names = {pending->options.base_joint_name};
      for (const auto & point : pending->plan.joint_trajectory.points) {
        pending->plan.base_trajectory.points.push_back(baseTrajectoryPoint(
          baseCoordinates(pending->base_reference), BaseCoordinates::Zero(),
          BaseCoordinates::Zero(), point.time_from_start));
      }
    }
    pending->views = visualization_->publishSingleArm(pending->plan, pending->arm);
    context_->display(*pending->plan.trajectory);
    retain(pending, "Trajectory planned and published; waiting for execute trigger");
    if (pending->handle->get_goal()->execute && !canceled()) {
      {std::lock_guard<std::mutex> lock(mutex_);
        if (pending_ != pending || stage_ != Stage::Pending) {return;}
        stage_ = Stage::Executing;}
      executePlan(pending);
    }
  } catch (const std::exception & ex) {finish(pending, false, std::string("Planning exception: ") + ex.what());}
}

bool PlanCartesianTrajectoryActionServer::backendReady(const PendingPlan & pending, std::string & error)
{
  if (!execution_active_.load()) {
    error = "Operator must activate the Cartesian planner lifecycle before execution";
    return false;
  }
  std::map<std::string, std::set<std::string>> required;
  if (pending.backend == "task" || pending.backend == "pose") {
    if (pending.backend == "task" && coordinated_action_.empty()) {error = "No coordinated Cartesian execution action configured"; return false;}
    if (!tasks_.at(pending.arm.name).manager_service.empty()) {
      required[tasks_.at(pending.arm.name).manager_service].insert(tasks_.at(pending.arm.name).controller_name);
    }
    if (pending.backend == "task" && !base_manager_service_.empty()) {
      required[base_manager_service_].insert(base_controller_name_);
    }
  } else {
    for (const auto & arm : {pending.arm}) {
      if (arm.controller_action.empty()) {error = "No controller action for arm '" + arm.name + "'"; return false;}
      if (!arm.manager_service.empty()) {required[arm.manager_service].insert(arm.controller_name);}
    }
  }
  for (const auto & entry : required) {
    auto & client = managers_[entry.first];
    if (!client) {client = create_client<controller_manager_msgs::srv::ListControllers>(
        entry.first, rmw_qos_profile_services_default, clients_group_);}
    if (!client->wait_for_service(std::chrono::duration<double>(execution_options_.controller_wait_timeout))) {
      error = "Controller manager unavailable: " + entry.first; return false;
    }
    auto future = client->async_send_request(std::make_shared<controller_manager_msgs::srv::ListControllers::Request>());
    if (future.wait_for(std::chrono::duration<double>(execution_options_.acceptance_timeout)) != std::future_status::ready) {
      client->remove_pending_request(future); error = "Timed out checking controller readiness"; return false;
    }
    const auto response = future.get();
    for (const auto & name : entry.second) {
      const auto it = std::find_if(response->controller.begin(), response->controller.end(),
        [&name](const auto & controller) {return controller.name == name && controller.state == "active";});
      if (name.empty() || it == response->controller.end()) {
        error = "Operator must activate controller '" + name + "'"; return false;
      }
    }
  }
  return true;
}

bool PlanCartesianTrajectoryActionServer::validatePending(const PendingPlan & pending, std::string & error) const
{
  if (pending.backend == "pose") {return validatePosePending(pending, error);}
  const auto canceled = [this, &pending]() {return stopped_.load() || pending.handle->is_canceling() || !pending.handle->is_active();};
  planning_scene::PlanningScenePtr scene;
  if (!context_->snapshot(pending.plan.joint_trajectory.joint_names, state_timeout_, state_max_age_, scene, error, canceled)) {return false;}
  const auto & measured = scene->getCurrentState();
  const auto & planned_start = pending.plan.trajectory->getFirstWayPoint();
  for (const auto & arm : {pending.arm}) {
    const auto * group = context_->model()->getJointModelGroup(arm.group_name);
    const auto * mount = group->getJointModels().front()->getParentLinkModel();
    if (!mount) {continue;}
    const auto & planned_mount = planned_start.getGlobalLinkTransform(mount);
    const auto & measured_mount = measured.getGlobalLinkTransform(mount);
    if ((measured_mount.translation() - planned_mount.translation()).norm() > base_position_tolerance_ ||
      Eigen::AngleAxisd(planned_mount.rotation().transpose() * measured_mount.rotation()).angle() > base_orientation_tolerance_)
    {
      error = "Plan obsolete: arm mount moved ('" + arm.name + "'); send a new goal";
      return false;
    }
  }
  const auto & first = pending.plan.joint_trajectory.points.front();
  for (std::size_t i = 0; i < pending.plan.joint_trajectory.joint_names.size(); ++i) {
    const auto & name = pending.plan.joint_trajectory.joint_names[i];
    const auto * joint = context_->model()->getJointOfVariable(name);
    const double tolerance = joint->getType() == moveit::core::JointModel::PRISMATIC ?
      start_prismatic_tolerance_ : start_joint_tolerance_;
    double delta = measured.getVariablePosition(name) - first.positions[i];
    if (joint->getType() == moveit::core::JointModel::REVOLUTE &&
      static_cast<const moveit::core::RevoluteJointModel *>(joint)->isContinuous()) {delta = std::remainder(delta, 2.0 * std::acos(-1.0));}
    if (std::abs(delta) > tolerance) {error = "Plan obsolete: initial joint moved ('" + name + "'); send a new goal"; return false;}
  }
  Eigen::Isometry3d current_transform;
  if (!visualization_->modelTransform(context_->model()->getModelFrame(), current_transform, error)) {return false;}
  if ((current_transform.translation() - pending.fixed_from_model.translation()).norm() > base_position_tolerance_ ||
    Eigen::AngleAxisd(pending.fixed_from_model.rotation().transpose() * current_transform.rotation()).angle() > base_orientation_tolerance_)
  {error = "Plan obsolete: robot base moved; send a new goal"; return false;}
  return bimanual_cartesian::SingleArmCartesianPlanner().validatePlan(
    scene, pending.arm, pending.goal,
    pending.options, pending.plan, error, canceled);
}

void PlanCartesianTrajectoryActionServer::executePlan(const std::shared_ptr<PendingPlan> & pending)
{
  if (pending->backend == "pose") {executePosePlan(pending); return;}
  try {
    std::string error;
    if (!backendReady(*pending, error)) {
      if (stopped_.load() || pending->handle->is_canceling()) {finish(pending, false, "Execution request canceled");}
      else {retain(pending, error + "; plan retained. Activate the backend and call execute_pending_trajectory");}
      return;
    }
    if (!validatePending(*pending, error)) {finish(pending, false, error); return;}
    if (pending->backend == "task") {executeCoordinated(pending); return;}
    std::vector<ExecutionTarget> targets;
    auto options = execution_options_;
    rclcpp::Time start(0, 0, get_clock()->get_clock_type());
    options.on_start = [this, &start](const builtin_interfaces::msg::Time & stamp) {
      start = rclcpp::Time(stamp, get_clock()->get_clock_type());
    };
    options.goal_time_tolerance = pending->tracking_tolerances.goal_time_tolerance;
    for (std::size_t i = 0; i < pending->arm.joint_names.size(); ++i) {
      control_msgs::msg::JointTolerance path, goal;
      path.name = pending->arm.joint_names[i]; path.position = pending->tracking_tolerances.joint_path[i];
      goal.name = path.name; goal.position = pending->tracking_tolerances.joint_goal[i];
      options.path_tolerance.push_back(path); options.goal_tolerance.push_back(goal);
    }
    for (const auto & arm : {pending->arm}) {
      targets.push_back({arm.name, arm.controller_action, arm.joint_names});
    }
    feedback(pending->handle, 0.8F, "executing", "Sending joint trajectory to manually activated backend");
    std::string tracking_failure;
    const double duration = baseTrajectorySeconds(pending->plan.joint_trajectory.points.back().time_from_start);
    const double goal_allowance = baseTrajectorySeconds(pending->tracking_tolerances.goal_time_tolerance);
    auto result = executor_->execute(pending->plan.joint_trajectory, targets, options,
      [this, pending]() {return stopped_.load() || !execution_active_.load() || pending->handle->is_canceling() || !pending->handle->is_active();},
      [this, pending, &start, &tracking_failure, duration, goal_allowance]() {
        const double elapsed = std::max(0.0, (now() - start).seconds());
        const bool at_end = elapsed >= duration;
        if (!checkControllerTracking(*pending, elapsed, at_end, tracking_failure)) {
          const bool converging = at_end && elapsed <= duration + goal_allowance &&
            tracking_failure.find("tracking tolerance violated") != std::string::npos;
          if (converging) {tracking_failure.clear();}
          else {executor_->cancel();}
        }
      });
    if (!tracking_failure.empty()) {result.success = false; result.retryable = false; result.message = tracking_failure;}
    if (result.success) {
      while (!checkControllerTracking(*pending, duration, true, tracking_failure)) {
        if (tracking_failure.find("tracking tolerance violated") == std::string::npos ||
          (now() - start).seconds() > duration + goal_allowance || stopped_.load() ||
          pending->handle->is_canceling() || !execution_active_.load()) {
          result.success = false; result.message = tracking_failure; break;
        }
        tracking_failure.clear();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    }
    if (!result.success && result.retryable && !stopped_.load() && !pending->handle->is_canceling()) {
      retain(pending, result.message + "; plan retained. Check the enabled joint trajectory task and retry the execute service");
    } else {finish(pending, result.success, result.message);}
  } catch (const std::exception & ex) {
    executor_->cancel(); cancelCoordinated();
    finish(pending, false, std::string("Execution exception: ") + ex.what());
  }
}

bool PlanCartesianTrajectoryActionServer::checkControllerTracking(
  const PendingPlan & pending, double elapsed, bool final, std::string & error) const
{
  planning_scene::PlanningScenePtr scene;
  if (!context_->snapshot(pending.arm.joint_names, 0.05, state_max_age_, scene, error,
      [this]() {return stopped_.load() || !execution_active_.load();})) {return false;}
  trajectory_msgs::msg::JointTrajectoryPoint desired_joints;
  if (!bimanual_cartesian::BimanualCartesianPlanner::sampleTrajectory(
      pending.plan.joint_trajectory, elapsed, desired_joints, error)) {return false;}
  BaseTrajectorySample desired_base;
  if (!sampleBaseTrajectory(pending.plan.base_trajectory, elapsed, desired_base, error)) {return false;}
  const auto & measured = scene->getCurrentState();
  const auto measured_base = baseCoordinates(measured.getGlobalLinkTransform(base_frame_), desired_base.coordinates);
  const BaseCoordinates base_error = desired_base.coordinates - measured_base;
  auto desired_state = pending.plan.trajectory->getFirstWayPoint();
  std::vector<double> joint_error(pending.arm.joint_names.size());
  for (std::size_t i = 0; i < pending.arm.joint_names.size(); ++i) {
    const auto & name = pending.arm.joint_names[i];
    desired_state.setVariablePosition(name, desired_joints.positions[i]);
    joint_error[i] = desired_joints.positions[i] - measured.getVariablePosition(name);
    const auto * joint = context_->model()->getJointOfVariable(name);
    if (joint->getType() == moveit::core::JointModel::REVOLUTE &&
      static_cast<const moveit::core::RevoluteJointModel *>(joint)->isContinuous()) {
      joint_error[i] = std::remainder(joint_error[i], 2.0 * std::acos(-1.0));
    }
  }
  desired_state.update();
  const Eigen::Isometry3d desired_tip = final ? pending.goal.relative_target :
    desired_state.getGlobalLinkTransform(pending.arm.tip_frame);
  const auto & actual_tip = measured.getGlobalLinkTransform(pending.arm.tip_frame);
  BaseCoordinates tip_error;
  const Eigen::Matrix3d rotation = pending.goal.world_from_tolerance.rotation().transpose();
  tip_error.head<3>() = rotation * (desired_tip.translation() - actual_tip.translation());
  tip_error.tail<3>() = rotation * bimanual_cartesian::BimanualCartesianPlanner::orientationError(
    desired_tip.rotation(), actual_tip.rotation());
  const auto & tolerance = pending.tracking_tolerances;
  const auto & base_tolerance = final ? tolerance.base_goal : tolerance.base_path;
  const auto & joint_tolerance = final ? tolerance.joint_goal : tolerance.joint_path;
  const auto & tip_tolerance = final ? tolerance.tip_goal : tolerance.tip_path;
  for (std::size_t i = 0; i < 6; ++i) {
    if (std::abs(base_error[static_cast<Eigen::Index>(i)]) > base_tolerance[i] + 1.0e-6) {
      error = "Base tracking tolerance violated at coordinate " + std::to_string(i); break;
    }
    if (std::abs(tip_error[static_cast<Eigen::Index>(i)]) > tip_tolerance[i] + 1.0e-6) {
      error = "Tool tracking tolerance violated at coordinate " + std::to_string(i); break;
    }
    if (!pending.goal.base_lower_limits.empty() &&
      (measured_base[static_cast<Eigen::Index>(i)] < pending.goal.base_lower_limits[i] - 1.0e-6 ||
      measured_base[static_cast<Eigen::Index>(i)] > pending.goal.base_upper_limits[i] + 1.0e-6)) {
      error = "Measured base is outside goal pose bounds"; break;
    }
  }
  for (std::size_t i = 0; error.empty() && i < joint_error.size(); ++i) {
    if (std::abs(joint_error[i]) > joint_tolerance[i] + 1.0e-9) {
      error = "Joint tracking tolerance violated: " + pending.arm.joint_names[i];
    }
  }
  if (pending.handle->is_active() && !pending.handle->is_canceling()) {
    auto output = std::make_shared<Action::Feedback>();
    output->progress = final ? 1.0F : 0.85F;
    output->current_stage = "executing";
    output->message = error.empty() ? "Tracking arm and fixed-base reference" : error;
    output->joint_error = joint_error;
    std::copy_n(base_error.data(), 6, output->base_error.begin());
    std::copy_n(tip_error.data(), 6, output->tip_error.begin());
    pending.handle->publish_feedback(output);
  }
  return error.empty();
}

void PlanCartesianTrajectoryActionServer::cancelCoordinated()
{
  std::shared_ptr<FollowDispatch> dispatch;
  {std::lock_guard<std::mutex> lock(mutex_); dispatch = follow_dispatch_;}
  if (!dispatch) {return;}
  FollowHandle::SharedPtr handle;
  {std::lock_guard<std::mutex> lock(dispatch->mutex); dispatch->canceled = true; handle = dispatch->handle;}
  if (handle && coordinated_client_) {
    try {coordinated_client_->async_cancel_goal(handle);} catch (const std::exception &) {}
  }
}

void PlanCartesianTrajectoryActionServer::executeCoordinated(const std::shared_ptr<PendingPlan> & pending)
{
  const auto canceled = [this, pending]() {
      return stopped_.load() || !execution_active_.load() || pending->handle->is_canceling() ||
             !pending->handle->is_active();
    };
  if (!coordinated_client_->wait_for_action_server(
      std::chrono::duration<double>(execution_options_.controller_wait_timeout))) {
    retain(pending, "Coordinated execution unavailable; plan retained"); return;
  }
  if (canceled()) {finish(pending, false, "Execution canceled before coordinated dispatch"); return;}
  FollowAction::Goal goal;
  goal.arm = pending->arm.name;
  goal.tip_frame = pending->arm.tip_frame;
  goal.available_dofs.assign(pending->goal.available_dofs.begin(), pending->goal.available_dofs.end());
  goal.base_reference_pose = stampedPose(pending->base_reference, reference_frame_);
  goal.target_pose = stampedPose(pending->goal.relative_target, reference_frame_);
  goal.tolerance_reference_pose = stampedPose(pending->goal.world_from_tolerance, reference_frame_);
  goal.joint_trajectory = pending->plan.joint_trajectory;
  goal.base_trajectory = pending->plan.base_trajectory;
  const auto start = now() + rclcpp::Duration::from_seconds(execution_options_.start_delay);
  goal.joint_trajectory.header.stamp = start;
  goal.base_trajectory.header.stamp = start;
  goal.base_trajectory.header.frame_id = reference_frame_;
  goal.base_limits = pending->base_limits;
  goal.tracking_tolerances = pending->tracking_tolerances;
  auto dispatch = std::make_shared<FollowDispatch>();
  {std::lock_guard<std::mutex> lock(mutex_); follow_dispatch_ = dispatch;}
  typename rclcpp_action::Client<FollowAction>::SendGoalOptions options;
  const auto client = coordinated_client_;
  options.goal_response_callback = [dispatch, client](FollowHandle::SharedPtr handle) {
      bool cancel;
      {std::lock_guard<std::mutex> lock(dispatch->mutex); dispatch->handle = handle; cancel = dispatch->canceled;}
      if (cancel && handle) {client->async_cancel_goal(handle);}
    };
  options.feedback_callback = [handle = pending->handle](
    FollowHandle::SharedPtr, const std::shared_ptr<const FollowAction::Feedback> input) {
      if (!handle->is_active() || handle->is_canceling()) {return;}
      auto output = std::make_shared<Action::Feedback>();
      output->progress = 0.8F + 0.2F * input->progress;
      output->current_stage = input->current_stage;
      output->message = input->message;
      output->base_error = input->base_error;
      output->joint_error = input->joint_error;
      output->tip_error = input->tip_error;
      try {handle->publish_feedback(output);} catch (const std::exception &) {}
    };
  feedback(pending->handle, 0.8F, "executing", "Reserving synchronized base and arm tasks");
  auto acceptance = client->async_send_goal(goal, options);
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(execution_options_.acceptance_timeout);
  while (acceptance.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
    if (canceled() || std::chrono::steady_clock::now() >= deadline) {
      cancelCoordinated();
      if (canceled()) {finish(pending, false, "Coordinated execution canceled before acceptance");}
      else {finish(pending, false, "Coordinated acceptance timed out; late acceptance will be canceled");}
      return;
    }
  }
  auto handle = acceptance.get();
  if (!handle) {retain(pending, "Coordinated execution rejected; plan retained. Check controller state and task configuration"); return;}
  if (canceled()) {cancelCoordinated();}
  auto result = client->async_get_result(handle);
  // Execution progress follows ROS time; pausing simulation must not time it out.
  while (rclcpp::ok() && result.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
    if (canceled()) {cancelCoordinated(); finish(pending, false, "Coordinated execution canceled"); return;}
  }
  if (!rclcpp::ok()) {cancelCoordinated(); finish(pending, false, "ROS shut down during execution"); return;}
  const auto response = result.get();
  const bool success = response.code == rclcpp_action::ResultCode::SUCCEEDED && response.result && response.result->success;
  finish(pending, success, response.result ? response.result->message : "Coordinated execution returned no result");
}

void PlanCartesianTrajectoryActionServer::executeTrigger(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (!execution_active_.load()) {
    response->message = "Activate the Cartesian planner lifecycle before requesting execution";
    return;
  }
  std::shared_ptr<PendingPlan> pending;
  {std::lock_guard<std::mutex> lock(mutex_);
    if (stage_ != Stage::Pending || !pending_) {response->message = "No pending plan is available"; return;}
    pending = pending_; stage_ = Stage::Executing;}
  if (worker_.joinable()) {worker_.join();}
  if (pending->handle->is_canceling() || !pending->handle->is_active()) {
    finish(pending, false, "Execution request canceled");
    response->message = "Execution request canceled";
    return;
  }
  stopped_.store(false);
  response->success = true;
  response->message = "Requested pending trajectory execution; readiness and current scene will be checked";
  worker_ = std::thread([this, pending]() {executePlan(pending);});
}

void PlanCartesianTrajectoryActionServer::cancelTrigger(
  const std::shared_ptr<std_srvs::srv::Trigger::Request>,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  std::shared_ptr<PendingPlan> pending;
  Stage stage;
  {std::lock_guard<std::mutex> lock(mutex_); pending = pending_; stage = stage_;}
  if (!pending) {response->message = "No pending or active Cartesian goal"; return;}
  stopped_.store(true); executor_->cancel();
  cancelCoordinated();
  if (stage == Stage::Pending) {finish(pending, false, "Plan canceled by operator service");}
  response->success = true; response->message = "Cancellation requested";
}

void PlanCartesianTrajectoryActionServer::feedback(
  const std::shared_ptr<Handle> & handle, float progress,
  const std::string & stage, const std::string & message) const
{
  if (!handle || !handle->is_active()) {return;}
  auto feedback = std::make_shared<Action::Feedback>();
  feedback->progress = progress; feedback->current_stage = stage; feedback->message = message;
  try {
    handle->publish_feedback(feedback);
  } catch (const std::exception & ex) {
    RCLCPP_DEBUG(get_logger(), "Cartesian feedback unavailable: %s", ex.what());
  }
}

void PlanCartesianTrajectoryActionServer::retain(
  const std::shared_ptr<PendingPlan> & pending, const std::string & message)
{
  {std::lock_guard<std::mutex> lock(mutex_); if (pending_ != pending) {return;} stage_ = Stage::Pending;}
  feedback(pending->handle, 0.8F, "waiting_for_execute", message);
}

void PlanCartesianTrajectoryActionServer::finish(
  const std::shared_ptr<PendingPlan> & pending, bool success, const std::string & message)
{
  if (pending->finished.exchange(true)) {return;}
  auto result = std::make_shared<Action::Result>();
  result->success = success;
  result->message = message;
  result->joint_trajectory = pending->plan.joint_trajectory;
  result->base_trajectory = pending->plan.base_trajectory;
  result->base_path = pending->views.base_path;
  result->path = pending->views.arm_b_path;
  result->desired_path = pending->views.desired_relative_path;
  {std::lock_guard<std::mutex> lock(mutex_);
    if (pending_ == pending || !pending_) {pending_.reset(); stage_ = Stage::Idle;}}
  try {
    if (pending->handle && pending->handle->is_active()) {
      if (pending->handle->is_canceling()) {result->success = false; pending->handle->canceled(result);}
      else if (success) {feedback(pending->handle, 1.0F, "completed", message); pending->handle->succeed(result);}
      else {pending->handle->abort(result);}
    }
  } catch (const std::exception & ex) {
    RCLCPP_WARN(get_logger(), "Could not report Cartesian result: %s", ex.what());
  }
}

void PlanCartesianTrajectoryActionServer::stop(const std::string & message)
{
  stopped_.store(true);
  if (executor_) {executor_->cancel();}
  cancelCoordinated();
  if (worker_.joinable()) {worker_.join();}
  std::shared_ptr<PendingPlan> pending;
  {std::lock_guard<std::mutex> lock(mutex_); pending = pending_;}
  if (pending) {finish(pending, false, message);}
}

void PlanCartesianTrajectoryActionServer::cleanupResources()
{
  execution_active_.store(false);
  stop("Cartesian planner cleaned up");
  cancel_timer_.reset(); action_.reset(); execute_service_.reset(); cancel_service_.reset();
  if (context_) {context_->stop();}
  managers_.clear(); coordinated_client_.reset(); follow_dispatch_.reset(); clients_group_.reset();
  pose_endpoints_.clear(); pose_tf_listener_.reset(); pose_tf_buffer_.reset();
  executor_.reset(); visualization_.reset(); context_.reset(); moveit_node_.reset();
  arms_.clear(); tasks_.clear();
  planner_options_.tightened_limits.clear(); planner_options_.nominal_positions.clear();
}

}  // namespace sura_manipulation_actions

#ifndef SURA_MANIPULATION_ACTIONS_NO_MAIN
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sura_manipulation_actions::PlanCartesianTrajectoryActionServer>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
#endif
