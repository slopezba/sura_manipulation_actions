#pragma once

#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "moveit/planning_scene_monitor/planning_scene_monitor.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace sura_manipulation_actions
{

// Owns only MoveIt observation and display. It never sends robot commands.
class CartesianMoveItContext
{
public:
  explicit CartesianMoveItContext(rclcpp::Node::SharedPtr node);
  ~CartesianMoveItContext();
  void stop();
  bool start(
    const std::string & joint_states_topic, const std::string & scene_input_topic,
    const std::string & scene_output_topic, const std::string & display_topic,
    double scene_publish_rate, std::string & error);
  const moveit::core::RobotModelConstPtr & model() const {return model_;}
  bool snapshot(
    const std::vector<std::string> & joint_names, double timeout, double max_age,
    planning_scene::PlanningScenePtr & scene, std::string & error,
    const std::function<bool()> & canceled = {}) const;
  void display(const robot_trajectory::RobotTrajectory & trajectory) const;

private:
  struct Observation
  {
    double position;
    rclcpp::Time stamp;
  };
  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelConstPtr model_;
  robot_model_loader::RobotModelLoaderPtr loader_;
  planning_scene_monitor::PlanningSceneMonitorPtr monitor_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr observation_sub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_pub_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
  std::thread spin_thread_;
  mutable std::mutex observation_mutex_;
  std::map<std::string, Observation> observations_;
};

}  // namespace sura_manipulation_actions
