#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sura_manipulation_actions/bimanual_cartesian_planner.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "visualization_msgs/msg/marker_array.hpp"

namespace sura_manipulation_actions
{

struct CartesianViews
{
  nav_msgs::msg::Path leader_path;
  nav_msgs::msg::Path follower_path;
  nav_msgs::msg::Path relative_path;
  nav_msgs::msg::Path desired_relative_path;
};

class CartesianTrajectoryVisualization
{
public:
  CartesianTrajectoryVisualization(
    rclcpp::Node::SharedPtr node, const std::string & topic_prefix,
    const std::string & visualization_topic_prefix,
    const std::string & fixed_frame, bool publish_paths, bool publish_markers,
    bool single_arm = false);
  bool modelTransform(
    const std::string & model_frame, Eigen::Isometry3d & transform, std::string & error) const;
  CartesianViews publish(
    const bimanual_cartesian::PlannedTrajectory & plan,
    const bimanual_cartesian::Arm & leader, const bimanual_cartesian::Arm & follower,
    double leader_radius);
  CartesianViews publishSingleArm(
    const bimanual_cartesian::PlannedTrajectory & plan,
    const bimanual_cartesian::Arm & arm);

private:
  struct PathCache
  {
    std::mutex mutex;
    CartesianViews views;
    bool ready = false;
  };
  rclcpp::Node::SharedPtr node_;
  std::string fixed_frame_;
  std::string marker_namespace_;
  bool publish_paths_;
  bool publish_markers_;
  mutable tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr leader_pub_, follower_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr relative_pub_, desired_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::shared_ptr<PathCache> path_cache_;
  rclcpp::TimerBase::SharedPtr path_refresh_timer_;
};

}  // namespace sura_manipulation_actions
