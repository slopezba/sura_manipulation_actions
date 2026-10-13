#include "sura_manipulation_actions/cartesian_trajectory_visualization.hpp"

#include <algorithm>
#include <chrono>
#include <utility>

#include "tf2/exceptions.h"
#include "tf2/time.h"

namespace sura_manipulation_actions
{
namespace
{
geometry_msgs::msg::Pose poseMessage(const Eigen::Isometry3d & transform)
{
  geometry_msgs::msg::Pose pose;
  pose.position.x = transform.translation().x();
  pose.position.y = transform.translation().y();
  pose.position.z = transform.translation().z();
  const Eigen::Quaterniond quaternion(transform.rotation());
  pose.orientation.x = quaternion.x();
  pose.orientation.y = quaternion.y();
  pose.orientation.z = quaternion.z();
  pose.orientation.w = quaternion.w();
  return pose;
}

visualization_msgs::msg::Marker pathMarker(
  const nav_msgs::msg::Path & path, const std::string & ns,
  float red, float green, float blue, bool desired = false)
{
  visualization_msgs::msg::Marker marker;
  marker.header = path.header;
  marker.ns = ns;
  marker.id = 0;
  marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.orientation.w = 1.0;
  marker.frame_locked = true;
  // The wider desired line remains visible behind a coincident planned line.
  marker.scale.x = desired ? 0.006 : 0.003;
  marker.color.r = red;
  marker.color.g = green;
  marker.color.b = blue;
  marker.color.a = desired ? 0.65F : 1.0F;
  for (const auto & pose : path.poses) {marker.points.push_back(pose.pose.position);}
  return marker;
}

void appendPoseAxes(
  visualization_msgs::msg::MarkerArray & markers,
  const std_msgs::msg::Header & header, const std::string & ns,
  const Eigen::Isometry3d & pose, double length)
{
  for (int axis = 0; axis < 3; ++axis) {
    visualization_msgs::msg::Marker marker;
    marker.header = header;
    marker.ns = ns;
    marker.id = axis;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.pose.orientation.w = 1.0;
    marker.frame_locked = true;
    marker.scale.x = 0.004; marker.scale.y = 0.008; marker.scale.z = 0.008;
    marker.color.a = 1.0F;
    if (axis == 0) {marker.color.r = 1.0F;}
    if (axis == 1) {marker.color.g = 1.0F;}
    if (axis == 2) {marker.color.b = 1.0F;}
    marker.points.push_back(poseMessage(pose).position);
    Eigen::Isometry3d end = pose;
    end.translation() += length * pose.linear().col(axis);
    marker.points.push_back(poseMessage(end).position);
    markers.markers.push_back(marker);
  }
}
}  // namespace

CartesianTrajectoryVisualization::CartesianTrajectoryVisualization(
  rclcpp::Node::SharedPtr node, const std::string & topic_prefix,
  const std::string & visualization_topic_prefix,
  const std::string & fixed_frame, bool publish_paths, bool publish_markers, bool single_arm)
: node_(std::move(node)), fixed_frame_(fixed_frame),
  marker_namespace_(single_arm ? "cartesian" : "bimanual_cartesian"),
  publish_paths_(publish_paths), publish_markers_(publish_markers),
  tf_buffer_(node_->get_clock()), tf_listener_(tf_buffer_, node_, false)
{
  const auto qos = rclcpp::QoS(1).reliable().transient_local();
  joint_pub_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    topic_prefix + "/joint_trajectory", qos);
  if (!single_arm) {
    leader_pub_ = node_->create_publisher<nav_msgs::msg::Path>(topic_prefix + "/leader_path", qos);
    relative_pub_ = node_->create_publisher<nav_msgs::msg::Path>(topic_prefix + "/relative_path", qos);
  }
  follower_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    topic_prefix + (single_arm ? "/path" : "/follower_path"), qos);
  desired_pub_ = node_->create_publisher<nav_msgs::msg::Path>(
    topic_prefix + (single_arm ? "/desired_path" : "/desired_relative_path"), qos);
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
    visualization_topic_prefix + "/marker_plans", rclcpp::QoS(10).reliable());
  path_cache_ = std::make_shared<PathCache>();
  if (publish_paths_) {
    // RViz transforms a Path when it receives it. Refresh with stamp zero to
    // follow the existing reference TF even when RViz's fixed frame is world.
    // Capture shared state/publishers, never the visualization object's lifetime.
    path_refresh_timer_ = node_->create_wall_timer(std::chrono::milliseconds(100),
      [cache = path_cache_, leader_pub = leader_pub_, follower_pub = follower_pub_,
      relative_pub = relative_pub_, desired_pub = desired_pub_]() {
        std::lock_guard<std::mutex> lock(cache->mutex);
        if (!cache->ready) {return;}
        if (leader_pub) {leader_pub->publish(cache->views.leader_path);}
        follower_pub->publish(cache->views.follower_path);
        if (relative_pub) {relative_pub->publish(cache->views.relative_path);}
        desired_pub->publish(cache->views.desired_relative_path);
      });
  }
}

bool CartesianTrajectoryVisualization::modelTransform(
  const std::string & model_frame, Eigen::Isometry3d & transform, std::string & error) const
{
  transform.setIdentity();
  if (model_frame == fixed_frame_) {return true;}
  try {
    const auto msg = tf_buffer_.lookupTransform(fixed_frame_, model_frame, tf2::TimePointZero);
    transform.translation() = Eigen::Vector3d(
      msg.transform.translation.x, msg.transform.translation.y, msg.transform.translation.z);
    const auto & q = msg.transform.rotation;
    transform.linear() = Eigen::Quaterniond(q.w, q.x, q.y, q.z).normalized().toRotationMatrix();
    return true;
  } catch (const tf2::TransformException & ex) {
    error = "Missing TF from '" + model_frame + "' to '" + fixed_frame_ + "': " + ex.what();
    return false;
  }
}

CartesianViews CartesianTrajectoryVisualization::publish(
  const bimanual_cartesian::PlannedTrajectory & plan,
  const bimanual_cartesian::Arm & leader, const bimanual_cartesian::Arm & follower,
  double leader_radius)
{
  CartesianViews views;
  views.leader_path.header.frame_id = plan.reference_frame;
  // Zero requests the latest transform from the already existing base frame.
  views.leader_path.header.stamp = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  views.follower_path.header = views.leader_path.header;
  views.relative_path.header = views.leader_path.header;
  views.desired_relative_path.header = views.relative_path.header;

  const Eigen::Isometry3d reference_from_model = plan.model_from_reference.inverse();
  const Eigen::Isometry3d initial_leader = reference_from_model * plan.initial_leader_pose;
  // Embed the old drawing origin as a position offset in the existing frame.
  // Its axes are the base axes; the offset stays fixed relative to that base.
  Eigen::Isometry3d anchor = Eigen::Isometry3d::Identity();
  anchor.translation() = initial_leader.translation();
  const auto anchor_pose = poseMessage(anchor);

  const auto count = plan.trajectory->getWayPointCount();
  for (std::size_t i = 0; i < count; ++i) {
    const auto & state = plan.trajectory->getWayPoint(i);
    const auto & leader_pose = state.getGlobalLinkTransform(leader.tip_frame);
    const auto & follower_pose = state.getGlobalLinkTransform(follower.tip_frame);
    geometry_msgs::msg::PoseStamped pose;
    pose.header = views.leader_path.header;
    pose.pose = poseMessage(reference_from_model * leader_pose);
    views.leader_path.poses.push_back(pose);
    pose.pose = poseMessage(reference_from_model * follower_pose);
    views.follower_path.poses.push_back(pose);
    pose.header = views.relative_path.header;
    pose.pose = poseMessage(anchor *
      bimanual_cartesian::BimanualCartesianPlanner::relativePoseInReference(
        leader_pose, follower_pose, plan.model_from_reference));
    views.relative_path.poses.push_back(pose);
    const auto nominal = bimanual_cartesian::BimanualCartesianPlanner::interpolateRelativePose(
      plan.initial_relative_pose, plan.relative_target, plan.progress_values.at(i));
    pose.pose = poseMessage(anchor * nominal);
    views.desired_relative_path.poses.push_back(pose);
  }
  auto joints = plan.joint_trajectory;
  joints.header.stamp = node_->now();
  joint_pub_->publish(joints);
  // One complete snapshot contains every enabled layer of this action's plan.
  // Stable, distinct (namespace, ID) keys update RViz without a DELETE/ADD pair.
  visualization_msgs::msg::MarkerArray markers;
  if (publish_paths_) {
    std::lock_guard<std::mutex> lock(path_cache_->mutex);
    path_cache_->views = views;
    path_cache_->ready = true;
    leader_pub_->publish(views.leader_path);
    follower_pub_->publish(views.follower_path);
    relative_pub_->publish(views.relative_path);
    desired_pub_->publish(views.desired_relative_path);
    markers.markers.push_back(pathMarker(views.leader_path,
      marker_namespace_ + "/leader_path", 0.1F, 0.9F, 0.8F));
    markers.markers.push_back(pathMarker(views.follower_path,
      marker_namespace_ + "/follower_path", 1.0F, 0.5F, 0.1F));
    markers.markers.push_back(pathMarker(views.relative_path,
      marker_namespace_ + "/planned_path", 0.0F, 1.0F, 0.5F));
    markers.markers.push_back(pathMarker(views.desired_relative_path,
      marker_namespace_ + "/desired_path", 0.7F, 0.3F, 1.0F, true));
  }
  if (publish_markers_) {
    visualization_msgs::msg::Marker sphere;
    sphere.header = views.leader_path.header;
    sphere.frame_locked = true;
    sphere.ns = marker_namespace_ + "/leader_workspace";
    sphere.id = 0;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.pose = anchor_pose;
    sphere.scale.x = sphere.scale.y = sphere.scale.z = std::max(0.001, 2.0 * leader_radius);
    sphere.color.g = 0.8F;
    sphere.color.b = 0.8F;
    sphere.color.a = 0.2F;
    markers.markers.push_back(sphere);
    // Pose triads show the target and both actual final effectors.
    Eigen::Isometry3d target_pose = anchor * plan.relative_target;
    target_pose.linear() = plan.relative_target.rotation() * initial_leader.rotation();
    appendPoseAxes(markers, views.leader_path.header,
      marker_namespace_ + "/target", target_pose, 0.06);
    appendPoseAxes(markers, views.leader_path.header, marker_namespace_ + "/leader_final",
      reference_from_model * plan.trajectory->getLastWayPoint().getGlobalLinkTransform(leader.tip_frame), 0.04);
    appendPoseAxes(markers, views.leader_path.header, marker_namespace_ + "/follower_final",
      reference_from_model * plan.trajectory->getLastWayPoint().getGlobalLinkTransform(follower.tip_frame), 0.04);
  }
  if (!markers.markers.empty()) {marker_pub_->publish(markers);}
  return views;
}

CartesianViews CartesianTrajectoryVisualization::publishSingleArm(
  const bimanual_cartesian::PlannedTrajectory & plan, const bimanual_cartesian::Arm & arm)
{
  CartesianViews views;
  views.follower_path.header.frame_id = plan.reference_frame;
  views.follower_path.header.stamp = rclcpp::Time(0, 0, node_->get_clock()->get_clock_type());
  views.desired_relative_path.header = views.follower_path.header;
  const Eigen::Isometry3d reference_from_model = plan.model_from_reference.inverse();
  for (std::size_t i = 0; i < plan.trajectory->getWayPointCount(); ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = views.follower_path.header;
    pose.pose = poseMessage(reference_from_model *
      plan.trajectory->getWayPoint(i).getGlobalLinkTransform(arm.tip_frame));
    views.follower_path.poses.push_back(pose);
    pose.pose = poseMessage(bimanual_cartesian::BimanualCartesianPlanner::interpolateRelativePose(
      plan.initial_relative_pose, plan.relative_target, plan.progress_values.at(i)));
    views.desired_relative_path.poses.push_back(pose);
  }
  auto joints = plan.joint_trajectory;
  joints.header.stamp = node_->now();
  joint_pub_->publish(joints);
  visualization_msgs::msg::MarkerArray markers;
  if (publish_paths_) {
    std::lock_guard<std::mutex> lock(path_cache_->mutex);
    path_cache_->views = views;
    path_cache_->ready = true;
    follower_pub_->publish(views.follower_path);
    desired_pub_->publish(views.desired_relative_path);
    markers.markers.push_back(pathMarker(views.follower_path,
      marker_namespace_ + "/planned_path", 0.0F, 1.0F, 0.5F));
    markers.markers.push_back(pathMarker(views.desired_relative_path,
      marker_namespace_ + "/desired_path", 0.7F, 0.3F, 1.0F, true));
  }
  if (publish_markers_) {
    appendPoseAxes(markers, views.follower_path.header,
      marker_namespace_ + "/target", plan.relative_target, 0.06);
    appendPoseAxes(markers, views.follower_path.header, marker_namespace_ + "/final",
      reference_from_model * plan.trajectory->getLastWayPoint().getGlobalLinkTransform(arm.tip_frame), 0.04);
  }
  if (!markers.markers.empty()) {marker_pub_->publish(markers);}
  return views;
}

}  // namespace sura_manipulation_actions
