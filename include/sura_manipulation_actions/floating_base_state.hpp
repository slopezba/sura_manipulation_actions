#pragma once

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2_ros/buffer.h>
#include <cmath>
#include <string>

namespace sura_manipulation_actions
{

// Existing arm-only actions share the floating model but never command its root.
// Their collision checks must still use the measured vehicle pose.
inline bool refreshFloatingBase(
  const planning_scene_monitor::PlanningSceneMonitorPtr & monitor,
  moveit::core::RobotState & state, const rclcpp::Time & now,
  double max_age, std::string & error)
{
  const auto model = state.getRobotModel();
  if (model->getRootJoint()->getType() != moveit::core::JointModel::FLOATING) {error.clear(); return true;}
  try {
    if (!monitor->getTFClient()) {error = "Floating-base TF buffer is unavailable"; return false;}
    const auto transform = monitor->getTFClient()->lookupTransform(
      model->getModelFrame(), model->getRootLinkName(), tf2::TimePointZero);
    const rclcpp::Time stamp(transform.header.stamp, now.get_clock_type());
    if (stamp.nanoseconds() && std::abs((now - stamp).seconds()) > max_age) {
      error = "Floating-base TF is stale"; return false;
    }
    const auto & t = transform.transform.translation;
    const auto & q = transform.transform.rotation;
    const Eigen::Quaterniond rotation(q.w, q.x, q.y, q.z);
    if (!Eigen::Vector3d(t.x, t.y, t.z).allFinite() || !rotation.coeffs().allFinite() || rotation.norm() < 1e-12) {
      error = "Floating-base TF is invalid"; return false;
    }
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation() = Eigen::Vector3d(t.x, t.y, t.z);
    pose.linear() = rotation.normalized().toRotationMatrix();
    state.setJointPositions(model->getRootJoint(), pose);
    state.update();
    error.clear();
    return true;
  } catch (const std::exception & ex) {
    error = std::string("Floating-base TF unavailable: ") + ex.what();
    return false;
  }
}

}  // namespace sura_manipulation_actions
