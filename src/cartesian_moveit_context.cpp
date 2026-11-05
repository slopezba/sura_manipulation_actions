#include "sura_manipulation_actions/cartesian_moveit_context.hpp"

#include <chrono>
#include <cmath>
#include <set>
#include <sstream>
#include <utility>

#include "moveit/robot_state/conversions.h"
#include "tf2/exceptions.h"

namespace sura_manipulation_actions
{

CartesianMoveItContext::CartesianMoveItContext(rclcpp::Node::SharedPtr node)
: node_(std::move(node)) {}

CartesianMoveItContext::~CartesianMoveItContext()
{
  stop();
  observation_sub_.reset();
  monitor_.reset();
}

void CartesianMoveItContext::stop()
{
  if (executor_) {executor_->cancel();}
  if (spin_thread_.joinable()) {spin_thread_.join();}
}

bool CartesianMoveItContext::start(
  const std::string & joint_states_topic, const std::string & scene_input_topic,
  const std::string & scene_output_topic, const std::string & display_topic,
  double scene_publish_rate, std::string & error)
{
  try {
    loader_ = std::make_shared<robot_model_loader::RobotModelLoader>(node_, "robot_description");
    model_ = loader_->getModel();
    if (!model_) {error = "MoveIt could not load robot_description / SRDF"; return false;}
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(node_->get_clock());
    // Timestamped Buffer lookups use the timeout-capable overload. Let the
    // listener service TF independently and enable the buffer's dedicated-thread flag.
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_, node_, true);
    monitor_ = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node_, loader_);
    if (!monitor_->getPlanningScene()) {error = "MoveIt planning scene unavailable"; return false;}
    observation_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        const auto stamp = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
          node_->now() : rclcpp::Time(msg->header.stamp, node_->get_clock()->get_clock_type());
        std::lock_guard<std::mutex> lock(observation_mutex_);
        for (std::size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
          if (std::isfinite(msg->position[i])) {
            observations_.insert_or_assign(msg->name[i], Observation{msg->position[i], stamp});
          }
        }
      });
    monitor_->startStateMonitor(joint_states_topic);
    monitor_->startSceneMonitor(scene_input_topic);
    monitor_->startWorldGeometryMonitor();
    monitor_->setPlanningScenePublishingFrequency(scene_publish_rate);
    monitor_->startPublishingPlanningScene(
      planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE, scene_output_topic);
    display_pub_ = node_->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      display_topic, rclcpp::QoS(1).reliable().transient_local());
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() {executor_->spin();});
    // Cleanup can follow immediately when later model validation fails.
    // Wait for spin() to start so an early cancel() cannot be lost.
    while (!executor_->is_spinning() && rclcpp::ok()) {std::this_thread::yield();}
    return true;
  } catch (const std::exception & ex) {
    error = std::string("Initializing MoveIt: ") + ex.what();
    return false;
  }
}

bool CartesianMoveItContext::snapshot(
  const std::vector<std::string> & joint_names, double timeout, double max_age,
  planning_scene::PlanningScenePtr & scene, std::string & error,
  const std::function<bool()> & canceled) const
{
  // Uncommanded joints still affect collision geometry. Do not use unobserved
  // default positions for a gripper, torso, or another arm in the scene.
  std::set<std::string> required(joint_names.begin(), joint_names.end());
  for (const auto * joint : model_->getActiveJointModels()) {
    if (joint->getVariableCount() == 1) {
      required.insert(joint->getVariableNames().front());
    }
  }
  const auto deadline = std::chrono::steady_clock::now() +
    std::chrono::duration<double>(timeout);
  do {
    if (canceled && canceled()) {error = "State acquisition canceled"; return false;}
    std::map<std::string, double> positions;
    std::ostringstream missing;
    {
      std::lock_guard<std::mutex> lock(observation_mutex_);
      const auto current = node_->now();
      for (const auto & name : required) {
        const auto it = observations_.find(name);
        if (it == observations_.end() ||
          (current - it->second.stamp).seconds() > max_age ||
          (it->second.stamp - current).seconds() > max_age)
        {
          missing << name << " ";
        } else {
          positions[name] = it->second.position;
        }
      }
    }
    if (positions.size() == required.size()) {
      planning_scene_monitor::LockedPlanningSceneRO locked(monitor_);
      if (locked) {
        scene = locked->diff();
        scene->decoupleParent();
        auto & state = scene->getCurrentStateNonConst();
        state.setVariablePositions(positions);
        if (model_->getRootJoint()->getType() == moveit::core::JointModel::FLOATING) {
          try {
            const auto base = tf_buffer_->lookupTransform(
              model_->getModelFrame(), model_->getRootLinkName(), tf2::TimePointZero);
            const rclcpp::Time stamp(base.header.stamp, node_->get_clock()->get_clock_type());
            if (stamp.nanoseconds() != 0 && std::abs((node_->now() - stamp).seconds()) > max_age) {
              error = "Missing or stale floating-base TF";
              std::this_thread::sleep_for(std::chrono::milliseconds(10));
              continue;
            }
            const auto & t = base.transform.translation;
            const auto & q = base.transform.rotation;
            Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
            if (!rotation.coeffs().allFinite() || rotation.norm() < 1e-12 ||
              !Eigen::Vector3d(t.x, t.y, t.z).allFinite()) {
              error = "Floating-base TF is not finite";
              return false;
            }
            Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
            pose.translation() = Eigen::Vector3d(t.x, t.y, t.z);
            pose.linear() = rotation.normalized().toRotationMatrix();
            state.setJointPositions(model_->getRootJoint(), pose);
          } catch (const tf2::TransformException & ex) {
            error = std::string("Floating-base TF unavailable: ") + ex.what();
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
          }
        }
        state.update();
        error.clear();
        return true;
      }
      error = "Planning scene unavailable";
    } else {
      error = "Missing or stale joint states: " + missing.str();
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  } while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline);
  return false;
}

bool CartesianMoveItContext::transformTarget(
  planning_scene::PlanningScenePtr & scene,
  const geometry_msgs::msg::PoseStamped & target, double max_age,
  Eigen::Isometry3d & model_target, Eigen::Isometry3d & model_from_reference,
  std::string & error) const
{
  try {
    const auto clock_type = node_->get_clock()->get_clock_type();
    const auto frame = target.header.frame_id.empty() ? model_->getModelFrame() : target.header.frame_id;
    rclcpp::Time stamp(target.header.stamp, clock_type);
    const bool floating = model_->getRootJoint()->getType() == moveit::core::JointModel::FLOATING;
    if (stamp.nanoseconds() == 0) {
      // Select the oldest latest dynamic stamp, then read every transform there.
      for (const auto & source : {frame, floating ? model_->getRootLinkName() : model_->getModelFrame()}) {
        if (source == model_->getModelFrame()) {continue;}
        const auto latest = tf_buffer_->lookupTransform(model_->getModelFrame(), source, tf2::TimePointZero);
        const rclcpp::Time candidate(latest.header.stamp, clock_type);
        if (candidate.nanoseconds() && (!stamp.nanoseconds() || candidate < stamp)) {stamp = candidate;}
      }
    }
    if (stamp.nanoseconds() && std::abs((node_->now() - stamp).seconds()) > max_age) {
      error = "Target/base TF timestamp is outside current_state_max_age";
      return false;
    }
    auto transform = [&](const std::string & source) {
        Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
        if (source == model_->getModelFrame()) {return result;}
        const auto msg = tf_buffer_->lookupTransform(model_->getModelFrame(), source, stamp);
        const auto & t = msg.transform.translation;
        const auto & q = msg.transform.rotation;
        Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
        if (!rotation.coeffs().allFinite() || rotation.norm() < 1e-12 ||
          !Eigen::Vector3d(t.x, t.y, t.z).allFinite()) {throw std::runtime_error("Invalid TF transform");}
        result.translation() = Eigen::Vector3d(t.x, t.y, t.z);
        result.linear() = rotation.normalized().toRotationMatrix();
        return result;
      };
    model_from_reference = transform(frame);
    const auto & p = target.pose.position;
    const auto & q = target.pose.orientation;
    Eigen::Isometry3d input = Eigen::Isometry3d::Identity();
    input.translation() = Eigen::Vector3d(p.x, p.y, p.z);
    input.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
    model_target = model_from_reference * input;
    if (floating) {
      auto & state = scene->getCurrentStateNonConst();
      state.setJointPositions(model_->getRootJoint(), transform(model_->getRootLinkName()));
      state.update();
    }
    error.clear();
    return true;
  } catch (const std::exception & ex) {
    error = std::string("Resolving target/base frames: ") + ex.what();
    return false;
  }
}

void CartesianMoveItContext::display(const robot_trajectory::RobotTrajectory & trajectory) const
{
  moveit_msgs::msg::DisplayTrajectory msg;
  msg.model_id = model_->getName();
  moveit::core::robotStateToRobotStateMsg(trajectory.getFirstWayPoint(), msg.trajectory_start);
  moveit_msgs::msg::RobotTrajectory motion;
  trajectory.getRobotTrajectoryMsg(motion);
  msg.trajectory.push_back(std::move(motion));
  display_pub_->publish(msg);
}

}  // namespace sura_manipulation_actions
