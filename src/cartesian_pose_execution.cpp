#include "sura_manipulation_actions/plan_cartesian_trajectory_action_server.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <stdexcept>
#include <thread>

#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl_interfaces/srv/set_parameters_atomically.hpp"

namespace sura_manipulation_actions
{
namespace
{
using GetParameters = rcl_interfaces::srv::GetParameters;
using SetParameters = rcl_interfaces::srv::SetParametersAtomically;
using PoseError = Eigen::Matrix<double, 6, 1>;

template<class Service>
typename Service::Response::SharedPtr request(
  const typename rclcpp::Client<Service>::SharedPtr & client,
  const typename Service::Request::SharedPtr & input, double timeout,
  std::string & error, const std::function<bool()> & canceled = {})
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
  while (!client->service_is_ready()) {
    if (!rclcpp::ok() || (canceled && canceled())) {error = "Pose task preparation canceled"; return {};}
    if (std::chrono::steady_clock::now() >= deadline) {
      error = "Pose task parameter service unavailable: " + std::string(client->get_service_name()); return {};
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  auto future = client->async_send_request(input);
  while (future.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
    if (!rclcpp::ok() || (canceled && canceled()) || std::chrono::steady_clock::now() >= deadline) {
      client->remove_pending_request(future);
      error = "Pose task parameter request interrupted or timed out: " + std::string(client->get_service_name());
      return {};
    }
  }
  return future.get();
}

bool setAutoUpdate(
  const rclcpp::Client<SetParameters>::SharedPtr & client,
  const std::string & name, bool enabled, double timeout, std::string & error)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
  do {
    auto input = std::make_shared<SetParameters::Request>();
    input->parameters.push_back(rclcpp::Parameter(name, enabled).to_parameter_msg());
    const double remaining = std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count();
    const auto response = request<SetParameters>(client, input, std::max(0.0, remaining), error);
    if (!response) {return false;}
    if (response->result.successful) {return true;}
    error = response->result.reason;
    // The standalone host can briefly reject parameters while updating tasks.
    if (error.find("retry the parameter change") == std::string::npos) {return false;}
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  } while (std::chrono::steady_clock::now() < deadline);
  return false;
}

Eigen::Isometry3d transformPose(const geometry_msgs::msg::Transform & value)
{
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() << value.translation.x, value.translation.y, value.translation.z;
  Eigen::Quaterniond rotation(value.rotation.w, value.rotation.x, value.rotation.y, value.rotation.z);
  if (!result.translation().allFinite() || !rotation.coeffs().allFinite() || rotation.norm() < 1e-12) {
    throw std::runtime_error("Non-finite pose or invalid quaternion in TF");
  }
  result.linear() = rotation.normalized().toRotationMatrix();
  return result;
}

geometry_msgs::msg::PoseStamped poseMessage(
  const Eigen::Isometry3d & value, const std::string & frame, const rclcpp::Time & time)
{
  geometry_msgs::msg::PoseStamped result;
  result.header.frame_id = frame; result.header.stamp = time;
  result.pose.position.x = value.translation().x();
  result.pose.position.y = value.translation().y();
  result.pose.position.z = value.translation().z();
  const Eigen::Quaterniond q(value.rotation());
  result.pose.orientation.x = q.x(); result.pose.orientation.y = q.y();
  result.pose.orientation.z = q.z(); result.pose.orientation.w = q.w();
  return result;
}

PoseError poseError(
  const Eigen::Isometry3d & desired, const Eigen::Isometry3d & measured,
  const Eigen::Matrix3d & reference)
{
  PoseError result;
  result.head<3>() = reference.transpose() * (desired.translation() - measured.translation());
  const Eigen::AngleAxisd difference(desired.rotation() * measured.rotation().transpose());
  result.tail<3>() = reference.transpose() * difference.axis() * difference.angle();
  return result;
}

bool within(const PoseError & error, const std::vector<double> & tolerance)
{
  for (Eigen::Index i = 0; i < error.size(); ++i) {
    if (!std::isfinite(error(i)) || std::abs(error(i)) > tolerance[static_cast<std::size_t>(i)] + 1e-9) {return false;}
  }
  return true;
}
}  // namespace

bool PlanCartesianTrajectoryActionServer::readTipPose(
  const PendingPlan & pending, Eigen::Isometry3d & pose, std::string & error) const
{
  try {
    const auto transform = pose_tf_buffer_->lookupTransform(reference_frame_, pending.arm.tip_frame, tf2::TimePointZero);
    const auto & stamp = transform.header.stamp;
    if (stamp.sec != 0 || stamp.nanosec != 0) {
      const double age = (now() - rclcpp::Time(stamp, get_clock()->get_clock_type())).seconds();
      if (std::abs(age) > state_max_age_) {error = "Tool TF is stale or ahead of ROS time"; return false;}
    }
    pose = transformPose(transform.transform);
    return true;
  } catch (const std::exception & exception) {
    error = "Tool TF unavailable: " + std::string(exception.what()); return false;
  }
}

void PlanCartesianTrajectoryActionServer::planPoseGoal(const std::shared_ptr<PendingPlan> & pending)
{
  try {
    std::string error;
    const auto canceled = [this, &pending]() {
        return stopped_.load() || pending->handle->is_canceling() || !pending->handle->is_active();
      };
    feedback(pending->handle, 0.0F, "acquiring_pose", "Reading tool TF; generating a geometric pose reference");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(state_timeout_);
    while (!readTipPose(*pending, pending->pose_start, error)) {
      if (canceled() || std::chrono::steady_clock::now() >= deadline) {finish(pending, false, error); return;}
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    auto input = pending->handle->get_goal()->target_pose;
    if (input.header.frame_id.empty()) {input.header.frame_id = reference_frame_;}
    Eigen::Isometry3d world_from_input = Eigen::Isometry3d::Identity();
    if (input.header.frame_id != reference_frame_) {
      world_from_input = transformPose(pose_tf_buffer_->lookupTransform(
        reference_frame_, input.header.frame_id,
        rclcpp::Time(input.header.stamp, get_clock()->get_clock_type())).transform);
    }
    pending->goal.relative_target = world_from_input * pending->goal.relative_target;
    pending->goal.world_from_tolerance = world_from_input;
    const auto & options = pending->options;
    if (!pending->pose_path.init(pending->pose_start, pending->goal.relative_target,
        pose_linear_velocity_ * options.velocity_scaling,
        pose_angular_velocity_ * options.velocity_scaling,
        pose_linear_acceleration_ * options.acceleration_scaling,
        pose_angular_acceleration_ * options.acceleration_scaling,
        pending->goal.cartesian_step, options.angular_step,
        static_cast<std::size_t>(std::max(0, options.max_waypoints)), error))
    {finish(pending, false, error); return;}
    if (canceled()) {finish(pending, false, "Pose planning canceled"); return;}
    auto desired = pending->pose_path.desiredPath(reference_frame_);
    desired.header.stamp = now();
    for (auto & point : desired.poses) {point.header.stamp = desired.header.stamp;}
    pending->views = visualization_->publishPosePath(desired);
    retain(pending, "Geometric tool path published; waiting for execute (no IK or collision validation)");
    if (canceled()) {finish(pending, false, "Pose planning canceled"); return;}
    if (pending->handle->get_goal()->execute && !canceled()) {
      {std::lock_guard<std::mutex> lock(mutex_);
        if (pending_ != pending || stage_ != Stage::Pending) {return;}
        stage_ = Stage::Executing;}
      executePosePlan(pending);
    }
  } catch (const std::exception & exception) {
    finish(pending, false, "Pose planning exception: " + std::string(exception.what()));
  }
}

bool PlanCartesianTrajectoryActionServer::validatePosePending(const PendingPlan & pending, std::string & error) const
{
  Eigen::Isometry3d measured;
  if (!readTipPose(pending, measured, error)) {return false;}
  if (!within(poseError(pending.pose_start, measured, pending.goal.world_from_tolerance.rotation()),
      pending.tracking_tolerances.tip_path))
  {error = "Plan obsolete: tool moved since planning; send a new goal"; return false;}
  return true;
}

bool PlanCartesianTrajectoryActionServer::poseBackendReady(
  const PendingPlan & pending, bool & auto_update, std::string & error)
{
  if (!backendReady(pending, error)) {return false;}
  const auto & endpoint = pose_endpoints_.at(pending.arm.name);
  const auto client = moveit_node_->create_client<GetParameters>(endpoint.controller_node + "/get_parameters");
  auto input = std::make_shared<GetParameters::Request>();
  const auto prefix = "tasks." + endpoint.task_id + ".";
  input->names = {"world_frame", prefix + "frame_id", prefix + "plugin", prefix + "auto_update_target_pose"};
  const auto response = request<GetParameters>(client, input, execution_options_.controller_wait_timeout, error,
    [this, &pending]() {return stopped_.load() || pending.handle->is_canceling() || !execution_active_.load();});
  if (!response) {return false;}
  const auto & values = response->values;
  using Type = rcl_interfaces::msg::ParameterType;
  if (values.size() != 4 || values[0].type != Type::PARAMETER_STRING ||
    values[1].type != Type::PARAMETER_STRING || values[2].type != Type::PARAMETER_STRING ||
    values[3].type != Type::PARAMETER_BOOL)
  {error = "Pose task parameters are missing or have invalid types"; return false;}
  if (values[0].string_value != reference_frame_ || values[1].string_value != pending.arm.tip_frame) {
    error = "Pose task world/tool frames must match planner world/tool frames"; return false;
  }
  if (values[2].string_value != "task_priority_kinematic_control/FramePoseTask" &&
    values[2].string_value != "task_priority_kinematic_control/EndEffectorPoseTask")
  {error = "Selected task must be FramePoseTask or EndEffectorPoseTask"; return false;}
  auto_update = values[3].bool_value;
  if (endpoint.target->get_subscription_count() == 0 || endpoint.feedforward->get_subscription_count() == 0) {
    error = "Pose target/feedforward subscribers are unavailable; check controller namespace and task ID"; return false;
  }
  return true;
}

void PlanCartesianTrajectoryActionServer::executePosePlan(const std::shared_ptr<PendingPlan> & pending)
{
  std::string error;
  bool auto_update = true;
  bool touched = false;
  bool success = false;
  Eigen::Isometry3d measured = pending->pose_start;
  const auto & endpoint = pose_endpoints_.at(pending->arm.name);
  const auto parameter_name = "tasks." + endpoint.task_id + ".auto_update_target_pose";
  rclcpp::Client<SetParameters>::SharedPtr setter;
  const auto canceled = [this, &pending]() {
      return !rclcpp::ok() || stopped_.load() || !execution_active_.load() ||
        pending->handle->is_canceling() || !pending->handle->is_active();
    };
  const auto publish_zero = [&]() {
      geometry_msgs::msg::TwistStamped zero;
      zero.header.frame_id = reference_frame_; zero.header.stamp = now();
      endpoint.feedforward->publish(zero);
    };
  try {
    if (!poseBackendReady(*pending, auto_update, error)) {
      if (canceled()) {finish(pending, false, "Pose execution canceled");}
      else {retain(pending, error + "; plan retained. Check the pose task and retry execution");}
      return;
    }
    if (!validatePosePending(*pending, error)) {finish(pending, false, error); return;}
    setter = moveit_node_->create_client<SetParameters>(endpoint.controller_node + "/set_parameters_atomically");
    if (canceled()) {throw std::runtime_error("Pose execution canceled");}
    if (!readTipPose(*pending, measured, error)) {throw std::runtime_error(error);}
    // A response can time out after a remote change; always attempt restoration.
    touched = true;
    publish_zero();
    endpoint.target->publish(poseMessage(measured, reference_frame_, now()));
    if (!setAutoUpdate(setter, parameter_name, false, execution_options_.controller_wait_timeout, error)) {
      throw std::runtime_error("Cannot disable automatic pose capture: " + error);
    }
    if (canceled()) {throw std::runtime_error("Pose execution canceled");}
    if (!validatePosePending(*pending, error)) {throw std::runtime_error(error);}
    const auto start = now();
    auto previous = start;
    auto next_tick = std::chrono::steady_clock::now();
    const double duration = pending->pose_path.duration();
    const double allowance = rclcpp::Duration(pending->tracking_tolerances.goal_time_tolerance).seconds();
    while (!canceled()) {
      const auto time = now();
      if (time < previous) {error = "ROS time moved backwards during pose execution"; break;}
      const bool advancing = time > previous;
      previous = time;
      const double elapsed = (time - start).seconds();
      if (!readTipPose(*pending, measured, error)) {break;}
      const auto desired = pending->pose_path.sample(elapsed);
      auto velocity = pending->pose_path.sampleVelocity(elapsed);
      if (!advancing) {velocity.setZero();}
      geometry_msgs::msg::TwistStamped feedforward;
      feedforward.header.frame_id = reference_frame_;
      feedforward.header.stamp = time;
      feedforward.twist.linear.x = velocity(0);
      feedforward.twist.linear.y = velocity(1);
      feedforward.twist.linear.z = velocity(2);
      feedforward.twist.angular.x = velocity(3);
      feedforward.twist.angular.y = velocity(4);
      feedforward.twist.angular.z = velocity(5);
      endpoint.target->publish(poseMessage(desired, reference_frame_, time));
      endpoint.feedforward->publish(feedforward);
      const auto tracking = poseError(desired, measured, pending->goal.world_from_tolerance.rotation());
      auto output = std::make_shared<Action::Feedback>();
      output->progress = static_cast<float>(0.8 + 0.2 * (duration > 0.0 ? std::clamp(elapsed / duration, 0.0, 1.0) : 1.0));
      output->current_stage = elapsed >= duration ? "converging_pose" : "tracking_pose";
      output->message = "Streaming Cartesian references to " + endpoint.task_id;
      std::copy(tracking.data(), tracking.data() + 6, output->tip_error.begin());
      pending->handle->publish_feedback(output);
      if (elapsed >= duration && within(tracking, pending->tracking_tolerances.tip_goal)) {
        success = true; error = "Cartesian pose target reached"; break;
      }
      if (elapsed < duration && !within(tracking, pending->tracking_tolerances.tip_path)) {
        error = "Cartesian pose path tracking tolerance violated"; break;
      }
      if (elapsed > duration + allowance) {error = "Cartesian pose goal tracking tolerance timed out"; break;}
      // Wall scheduling remains cancelable when ROS/simulation time is paused.
      next_tick = std::max(next_tick + std::chrono::milliseconds(10), std::chrono::steady_clock::now());
      std::this_thread::sleep_until(next_tick);
    }
    if (canceled()) {success = false; error = "Pose execution canceled or controller deactivated";}
  } catch (const std::exception & exception) {success = false; error = exception.what();}

  if (touched) {
    try {
      publish_zero();
      std::string measured_error;
      // On TF loss, retain the last valid measurement rather than a future target.
      readTipPose(*pending, measured, measured_error);
      endpoint.target->publish(poseMessage(measured, reference_frame_, now()));
      std::string restore_error;
      if (!setAutoUpdate(setter, parameter_name, auto_update, execution_options_.controller_wait_timeout, restore_error)) {
        success = false; error += "; restoring auto_update_target_pose failed: " + restore_error;
      }
    } catch (const std::exception & exception) {
      success = false; error += "; pose cleanup failed: " + std::string(exception.what());
    }
  }
  finish(pending, success, error);
}

}  // namespace sura_manipulation_actions
