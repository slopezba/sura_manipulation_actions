#pragma once

#include <array>
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "moveit_msgs/action/hybrid_planner.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sura_manipulation_actions/action/plan_joint_trajectory.hpp"

namespace sura_manipulation_actions
{

class HybridPlanJointTrajectoryActionServer : public rclcpp::Node
{
public:
  using PlanJointTrajectory = sura_manipulation_actions::action::PlanJointTrajectory;
  using GoalHandlePlanJointTrajectory =
    rclcpp_action::ServerGoalHandle<PlanJointTrajectory>;
  using HybridPlanner = moveit_msgs::action::HybridPlanner;
  using GoalHandleHybridPlanner = rclcpp_action::ClientGoalHandle<HybridPlanner>;

  explicit HybridPlanJointTrajectoryActionServer(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~HybridPlanJointTrajectoryActionServer() override;

private:
  struct ArmConfig
  {
    std::string canonical_name;
    std::string group_name;
    std::string hybrid_action;
    std::vector<std::string> joint_names;
  };

  using PredefinedPoseMap =
    std::unordered_map<std::string, std::unordered_map<std::string, std::array<double, 4>>>;

  void configure();
  bool loadPredefinedPoses(std::string & error_message);
  std::optional<ArmConfig> resolveArm(const std::string & arm_name) const;
  bool validateGoal(
    const PlanJointTrajectory::Goal & goal,
    ArmConfig & arm,
    std::array<double, 4> & target,
    std::string & error_message) const;
  HybridPlanner::Goal buildHybridGoal(
    const ArmConfig & arm,
    const std::array<double, 4> & target,
    double planning_time,
    double goal_tolerance) const;
  bool sendHybridGoal(
    const ArmConfig & arm,
    const HybridPlanner::Goal & hybrid_goal,
    const std::shared_ptr<GoalHandlePlanJointTrajectory> & goal_handle,
    std::string & error_message);

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PlanJointTrajectory::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandlePlanJointTrajectory> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandlePlanJointTrajectory> goal_handle);
  void executeGoal(const std::shared_ptr<GoalHandlePlanJointTrajectory> goal_handle);

  void requestStop();
  void joinWorkerThread();
  void cleanupResources();
  std::string namespacedTopic(const std::string & suffix) const;

  rclcpp_action::Server<PlanJointTrajectory>::SharedPtr action_server_;
  std::shared_ptr<rclcpp_action::Client<HybridPlanner>> left_hybrid_client_;
  std::shared_ptr<rclcpp_action::Client<HybridPlanner>> right_hybrid_client_;

  std::thread worker_thread_;
  std::atomic_bool busy_{false};
  std::atomic_bool stop_requested_{false};
  std::mutex active_goal_mutex_;
  std::shared_ptr<rclcpp_action::Client<HybridPlanner>> active_hybrid_client_;
  GoalHandleHybridPlanner::SharedPtr active_hybrid_goal_;

  std::string robot_namespace_;
  std::string action_name_;
  std::string left_group_name_;
  std::string right_group_name_;
  std::string left_hybrid_action_;
  std::string right_hybrid_action_;
  std::string execution_backend_;
  std::string predefined_poses_file_;
  PredefinedPoseMap predefined_poses_;
  std::vector<std::string> left_joint_names_;
  std::vector<std::string> right_joint_names_;
  double default_planning_time_{5.0};
  double default_goal_tolerance_{0.02};
  double hybrid_server_timeout_{5.0};
  double hybrid_result_timeout_{15.0};
  double max_velocity_scaling_factor_{0.05};
  double max_acceleration_scaling_factor_{0.05};
};

}  // namespace sura_manipulation_actions
