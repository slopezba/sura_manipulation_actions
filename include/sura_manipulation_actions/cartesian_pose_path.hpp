#pragma once

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>

#include "nav_msgs/msg/path.hpp"
#include "sura_manipulation_actions/msg/cartesian_pose_trajectory.hpp"

namespace sura_manipulation_actions
{

// A geometric end-effector path with a rest-to-rest quintic time law. This class
// deliberately does not solve inverse kinematics or validate robot collisions.
class CartesianPosePath
{
public:
  using Velocity = Eigen::Matrix<double, 6, 1>;

  bool init(
    const Eigen::Isometry3d & start, const Eigen::Isometry3d & target,
    double linear_velocity, double angular_velocity,
    double linear_acceleration, double angular_acceleration,
    double linear_step, double angular_step, std::size_t max_waypoints,
    std::string & error)
  {
    waypoint_count_ = 0;
    duration_ = 0.0;
    start_.setIdentity();
    target_.setIdentity();
    start_orientation_.setIdentity();
    target_orientation_.setIdentity();
    angular_displacement_.setZero();
    error.clear();
    if (!validPose(start) || !validPose(target)) {
      error = "Cartesian path endpoints must be finite rigid poses";
      return false;
    }
    for (const double limit : {linear_velocity, angular_velocity,
      linear_acceleration, angular_acceleration, linear_step, angular_step})
    {
      if (!std::isfinite(limit) || limit <= 0.0) {
        error = "Cartesian velocity, acceleration and sampling limits must be positive and finite";
        return false;
      }
    }
    if (max_waypoints == 0) {
      error = "Cartesian path max_waypoints must be positive";
      return false;
    }
    Eigen::Quaterniond begin(start.rotation()), end(target.rotation());
    begin.normalize();
    end.normalize();
    if (begin.dot(end) < 0.0) {end.coeffs() *= -1.0;}
    const auto relative = begin.conjugate() * end;
    const double angle = start.rotation().isApprox(target.rotation(), 1.0e-15) ? 0.0 :
      2.0 * std::atan2(relative.vec().norm(), std::abs(relative.w()));
    const double distance = (target.translation() - start.translation()).stableNorm();
    if (!std::isfinite(distance) || !std::isfinite(angle)) {
      error = "Cartesian path displacement is not finite";
      return false;
    }
    const long double segments = std::ceil(std::max(
      static_cast<long double>(distance) / linear_step,
      static_cast<long double>(angle) / angular_step));
    if (segments > static_cast<long double>(max_waypoints - 1)) {
      error = "Cartesian path exceeds max_waypoints at the requested sampling resolution";
      return false;
    }
    // max(s') = 15/8; max(|s''|) = 10/sqrt(3), for
    // s(u) = 10u^3 - 15u^4 + 6u^5 on u in [0, 1].
    constexpr double max_progress_velocity = 1.875;
    const double max_progress_acceleration = 10.0 / std::sqrt(3.0);
    const double duration = std::max({
      max_progress_velocity * distance / linear_velocity,
      max_progress_velocity * angle / angular_velocity,
      std::sqrt(max_progress_acceleration * distance / linear_acceleration),
      std::sqrt(max_progress_acceleration * angle / angular_acceleration)});
    if (!std::isfinite(duration)) {
      error = "Cartesian path duration is not finite";
      return false;
    }
    start_ = start;
    target_ = target;
    start_orientation_ = begin;
    target_orientation_ = end;
    start_.linear() = begin.toRotationMatrix();
    target_.linear() = end.toRotationMatrix();
    if (angle > 0.0) {
      // Use the same signed quaternion arc as SLERP. Rotate its local axis
      // into the path's reference frame for a spatial angular velocity.
      angular_displacement_ = begin * (relative.vec() * (angle / relative.vec().stableNorm()));
    }
    duration_ = duration;
    waypoint_count_ = static_cast<std::size_t>(segments) + 1;
    return true;
  }

  double duration() const {return duration_;}

  msg::CartesianPoseTrajectory descriptor(const std::string & frame) const
  {
    msg::CartesianPoseTrajectory out;
    out.header.frame_id = frame;
    const auto fill = [](geometry_msgs::msg::Pose & p, const Eigen::Isometry3d & t,
        const Eigen::Quaterniond & q) {
        p.position.x = t.translation().x(); p.position.y = t.translation().y();
        p.position.z = t.translation().z();
        p.orientation.x = q.x(); p.orientation.y = q.y();
        p.orientation.z = q.z(); p.orientation.w = q.w();
      };
    fill(out.start_pose, start_, start_orientation_);
    fill(out.target_pose, target_, target_orientation_);
    out.duration = duration_;
    return out;
  }

  // Deserialize without recomputing timing or choosing a different arc at pi.
  bool initTimed(const msg::CartesianPoseTrajectory & input, std::string & error)
  {
    const auto decode = [](const geometry_msgs::msg::Pose & p,
        Eigen::Isometry3d & t, Eigen::Quaterniond & q) {
        t.setIdentity(); t.translation() << p.position.x, p.position.y, p.position.z;
        q = Eigen::Quaterniond(p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z);
        if (!t.translation().allFinite() || !q.coeffs().allFinite() ||
          std::abs(q.norm() - 1.0) > 1e-12) {return false;}
        // Preserve exact coefficients: renormalizing again can reverse SLERP's
        // sign choice through roundoff at an exactly pi rotation.
        t.linear() = q.toRotationMatrix(); return true;
      };
    if (!decode(input.start_pose, start_, start_orientation_) ||
      !decode(input.target_pose, target_, target_orientation_) ||
      !std::isfinite(input.duration) || input.duration < 0.0 ||
      start_orientation_.dot(target_orientation_) < 0.0)
    {error = "Invalid timed Cartesian profile or signed quaternion arc"; return false;}
    const Eigen::Quaterniond relative = start_orientation_.conjugate() * target_orientation_;
    const double angle = start_.rotation().isApprox(target_.rotation(), 1e-15) ? 0.0 :
      2.0 * std::atan2(relative.vec().norm(), std::abs(relative.w()));
    angular_displacement_.setZero();
    if (angle > 0.0) {
      angular_displacement_ = start_orientation_ * (relative.vec() * (angle / relative.vec().stableNorm()));
    }
    if (input.duration == 0.0 &&
      ((start_.translation() - target_.translation()).norm() > 1e-12 || angle > 1e-12))
    {error = "A moving Cartesian profile needs a positive duration"; return false;}
    duration_ = input.duration; waypoint_count_ = duration_ == 0.0 ? 1U : 2U;
    error.clear(); return true;
  }

  Eigen::Isometry3d sample(double elapsed) const
  {
    if (duration_ == 0.0 || elapsed >= duration_) {return target_;}
    if (!std::isfinite(elapsed) || elapsed <= 0.0) {return start_;}
    const double u = elapsed / duration_;
    const double progress = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
    return interpolate(progress);
  }

  // Linear velocity at the tool origin and spatial angular velocity, both
  // expressed in the same reference frame as sample().
  Velocity sampleVelocity(double elapsed) const
  {
    Velocity velocity = Velocity::Zero();
    if (duration_ == 0.0 || !std::isfinite(elapsed) || elapsed <= 0.0 || elapsed >= duration_) {
      return velocity;
    }
    const double u = elapsed / duration_;
    const double remaining = 1.0 - u;
    const double progress_velocity = 30.0 * u * u * remaining * remaining / duration_;
    velocity.head<3>() = (target_.translation() - start_.translation()) * progress_velocity;
    velocity.tail<3>() = angular_displacement_ * progress_velocity;
    return velocity;
  }

  nav_msgs::msg::Path desiredPath(const std::string & frame) const
  {
    nav_msgs::msg::Path path;
    path.header.frame_id = frame;
    path.poses.reserve(waypoint_count_);
    for (std::size_t i = 0; i < waypoint_count_; ++i) {
      const double progress = waypoint_count_ == 1 ? 1.0 :
        static_cast<double>(i) / static_cast<double>(waypoint_count_ - 1);
      const auto pose = interpolate(progress);
      geometry_msgs::msg::PoseStamped message;
      message.header = path.header;
      message.pose.position.x = pose.translation().x();
      message.pose.position.y = pose.translation().y();
      message.pose.position.z = pose.translation().z();
      const Eigen::Quaterniond orientation(pose.rotation());
      message.pose.orientation.x = orientation.x();
      message.pose.orientation.y = orientation.y();
      message.pose.orientation.z = orientation.z();
      message.pose.orientation.w = orientation.w();
      path.poses.push_back(message);
    }
    return path;
  }

private:
  static bool validPose(const Eigen::Isometry3d & pose)
  {
    if (!pose.matrix().allFinite()) {return false;}
    const Eigen::Matrix3d rotation = pose.rotation();
    return (rotation.transpose() * rotation).isApprox(Eigen::Matrix3d::Identity(), 1.0e-6) &&
           std::abs(rotation.determinant() - 1.0) <= 1.0e-6 &&
           pose.matrix().row(3).isApprox(Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0), 1.0e-12);
  }

  Eigen::Isometry3d interpolate(double progress) const
  {
    if (progress <= 0.0) {return start_;}
    if (progress >= 1.0) {return target_;}
    Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
    result.translation() = (1.0 - progress) * start_.translation() +
      progress * target_.translation();
    result.linear() = start_orientation_.slerp(progress, target_orientation_).normalized().
      toRotationMatrix();
    return result;
  }

  Eigen::Isometry3d start_ = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d target_ = Eigen::Isometry3d::Identity();
  Eigen::Quaterniond start_orientation_ = Eigen::Quaterniond::Identity();
  Eigen::Quaterniond target_orientation_ = Eigen::Quaterniond::Identity();
  Eigen::Vector3d angular_displacement_ = Eigen::Vector3d::Zero();
  double duration_ = 0.0;
  std::size_t waypoint_count_ = 0;
};

}  // namespace sura_manipulation_actions
