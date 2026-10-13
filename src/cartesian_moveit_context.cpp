#include "sura_manipulation_actions/cartesian_moveit_context.hpp"

#include <chrono>
#include <cmath>
#include <set>
#include <sstream>
#include <utility>

#include "moveit/robot_state/conversions.h"

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
        state.update();
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
