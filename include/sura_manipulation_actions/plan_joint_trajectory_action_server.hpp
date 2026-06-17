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

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "moveit/planning_pipeline/planning_pipeline.h"
#include "moveit/planning_scene/planning_scene.h"
#include "moveit/planning_scene_monitor/planning_scene_monitor.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sura_manipulation_actions/action/plan_joint_trajectory.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace sura_manipulation_actions
{

class PlanJointTrajectoryActionServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using PlanJointTrajectory = sura_manipulation_actions::action::PlanJointTrajectory;
  using GoalHandlePlanJointTrajectory =
    rclcpp_action::ServerGoalHandle<PlanJointTrajectory>;
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory =
    rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;
  using ListControllers = controller_manager_msgs::srv::ListControllers;
  using SwitchController = controller_manager_msgs::srv::SwitchController;
  using Trigger = std_srvs::srv::Trigger;

  explicit PlanJointTrajectoryActionServer(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PlanJointTrajectoryActionServer() override;

private:
  struct ArmConfig
  {
    std::string canonical_name;
    std::string group_name;
    std::string controller_action;
    std::vector<std::string> joint_names;
  };

  struct CollisionSummary
  {
    bool in_collision{false};
    double closest_distance{0.0};
    std::string closest_object{"unknown"};
  };

  using PredefinedPoseMap =
    std::unordered_map<std::string, std::unordered_map<std::string, std::array<double, 5>>>;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PlanJointTrajectory::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandlePlanJointTrajectory> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandlePlanJointTrajectory> goal_handle);
  void planAcceptedGoal(const std::shared_ptr<GoalHandlePlanJointTrajectory> goal_handle);
  void executePendingPlan();
  void handleExecuteTrigger(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);
  void handleCancelTrigger(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response> response);

  bool initializeMoveIt(std::string & error_message);
  bool loadPredefinedPoses(std::string & error_message);
  void copyParametersToMoveItNode();
  std::optional<ArmConfig> resolveArm(const std::string & arm_name) const;
  bool validateGoal(
    const PlanJointTrajectory::Goal & goal,
    ArmConfig & arm,
    std::array<double, 5> & target,
    std::string & error_message) const;
  bool getCurrentState(moveit::core::RobotState & state, std::string & error_message) const;
  bool isStateCollisionFree(
    const moveit::core::RobotState & state,
    CollisionSummary & summary) const;
  CollisionSummary collisionSummary(const moveit::core::RobotState & state) const;
  std::array<double, 5> computeJointError(
    const moveit::core::RobotState & state,
    const ArmConfig & arm,
    const std::array<double, 5> & target) const;
  double computeErrorNorm(const std::array<double, 5> & error) const;
  bool planTrajectory(
    const ArmConfig & arm,
    const std::array<double, 5> & target,
    double planning_time,
    double goal_tolerance,
    robot_trajectory::RobotTrajectoryPtr & trajectory,
    std::string & error_message);
  trajectory_msgs::msg::JointTrajectory extractJointTrajectory(
    const robot_trajectory::RobotTrajectory & trajectory,
    const ArmConfig & arm) const;
  bool sendTrajectory(
    const ArmConfig & arm,
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const std::shared_ptr<GoalHandlePlanJointTrajectory> goal_handle,
    const std::array<double, 5> & target,
    std::string & error_message,
    double & final_error_norm);
  bool switchControllersForArm(const ArmConfig & arm, std::string & error_message);
  bool startPendingExecution(std::string & message);
  bool cancelPendingOrActive(const std::string & message, std::string & response_message);
  void finishPendingActionCancel(
    const std::shared_ptr<GoalHandlePlanJointTrajectory> goal_handle,
    const std::string & message) const;
  bool publishFeedback(
    const std::shared_ptr<GoalHandlePlanJointTrajectory> goal_handle,
    const ArmConfig & arm,
    const std::array<double, 5> & target,
    const std::string & state,
    const std::string & message) const;
  void publishDisplayTrajectory(const robot_trajectory::RobotTrajectory & trajectory) const;
  void publishCollisionDebug(
    const CollisionSummary & summary,
    const std::string & frame_id = "world_ned") const;
  void requestStop();
  void finishExecution();
  void finishPlanning();
  bool tryStartExecution();
  bool tryStartPlanning();
  bool isExecuting() const;
  bool isPlanning() const;
  bool isInactive();
  bool isActive();
  bool hasPendingPlan() const;
  void clearPendingPlan(bool finish_goal, const std::string & message);
  void scheduleDeactivateAfterGoal();
  void joinExecutionThread();
  void joinPlanningThread();
  void cleanupResources();
  std::string namespacedTopic(const std::string & suffix) const;
  std::string namespacedMoveItTopic(const std::string & suffix) const;

  rclcpp_action::Server<PlanJointTrajectory>::SharedPtr action_server_;
  std::shared_ptr<rclcpp_action::Client<FollowJointTrajectory>> left_controller_client_;
  std::shared_ptr<rclcpp_action::Client<FollowJointTrajectory>> right_controller_client_;
  rclcpp::Client<ListControllers>::SharedPtr list_controllers_client_;
  rclcpp::Client<SwitchController>::SharedPtr switch_controller_client_;
  rclcpp::Service<Trigger>::SharedPtr execute_service_;
  rclcpp::Service<Trigger>::SharedPtr cancel_service_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr collision_debug_pub_;

  rclcpp::Node::SharedPtr moveit_node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> moveit_executor_;
  std::thread moveit_spin_thread_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  planning_pipeline::PlanningPipelinePtr planning_pipeline_;

  mutable std::mutex moveit_mutex_;
  mutable std::mutex pending_mutex_;
  std::thread planning_thread_;
  std::thread execution_thread_;
  std::atomic_bool planning_{false};
  std::atomic_bool executing_{false};
  std::atomic_bool stop_requested_{false};
  std::shared_ptr<rclcpp_action::Client<FollowJointTrajectory>> active_controller_client_;
  GoalHandleFollowJointTrajectory::SharedPtr active_controller_goal_;
  std::shared_ptr<GoalHandlePlanJointTrajectory> pending_goal_handle_;
  ArmConfig pending_arm_;
  std::array<double, 5> pending_target_{};
  robot_trajectory::RobotTrajectoryPtr pending_robot_trajectory_;
  trajectory_msgs::msg::JointTrajectory pending_joint_trajectory_;
  std::string pending_plan_id_;
  bool has_pending_plan_{false};
  uint64_t next_plan_id_{1};

  std::string robot_namespace_;
  std::string moveit_namespace_;
  std::string action_name_;
  std::string joint_states_topic_;
  std::string display_trajectory_topic_;
  std::string planning_scene_topic_;
  std::string collision_debug_topic_;
  std::string left_group_name_;
  std::string right_group_name_;
  std::string left_controller_action_;
  std::string right_controller_action_;
  std::string controller_list_service_;
  std::string controller_switch_service_;
  std::string left_velocity_controller_;
  std::string right_velocity_controller_;
  std::string left_trajectory_controller_;
  std::string right_trajectory_controller_;
  std::string execute_service_name_;
  std::string cancel_service_name_;
  std::string planning_pipeline_namespace_;
  std::string planning_plugin_;
  std::string request_adapters_;
  std::string predefined_poses_file_;
  PredefinedPoseMap predefined_poses_;
  std::vector<std::string> left_joint_names_;
  std::vector<std::string> right_joint_names_;
  double default_planning_time_{5.0};
  double default_goal_tolerance_{0.02};
  double controller_wait_timeout_{5.0};
  double execution_feedback_rate_{10.0};
  double current_state_timeout_{2.0};
  double planning_scene_publish_rate_{10.0};
  double controller_switch_timeout_{2.0};
};

}  // namespace sura_manipulation_actions
