#pragma once

#include <atomic>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "moveit/planning_pipeline/planning_pipeline.h"
#include "moveit/planning_scene_monitor/planning_scene_monitor.h"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_trajectory/robot_trajectory.h"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "sura_manipulation_actions/action/plan_bimanual_joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace sura_manipulation_actions
{

class PlanBimanualJointTrajectoryActionServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  using PlanBimanualJointTrajectory =
    sura_manipulation_actions::action::PlanBimanualJointTrajectory;
  using GoalHandlePlanBimanualJointTrajectory =
    rclcpp_action::ServerGoalHandle<PlanBimanualJointTrajectory>;
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory =
    rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

  explicit PlanBimanualJointTrajectoryActionServer(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PlanBimanualJointTrajectoryActionServer() override;

private:
  struct CollisionSummary
  {
    bool in_collision{false};
    double closest_distance{0.0};
    std::string closest_object{"unknown"};
  };

  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const PlanBimanualJointTrajectory::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandlePlanBimanualJointTrajectory> goal_handle);
  void handleAccepted(const std::shared_ptr<GoalHandlePlanBimanualJointTrajectory> goal_handle);
  void executeGoal(const std::shared_ptr<GoalHandlePlanBimanualJointTrajectory> goal_handle);

  bool initializeMoveIt(std::string & error_message);
  void copyParametersToMoveItNode();
  bool validateGoal(
    const PlanBimanualJointTrajectory::Goal & goal,
    std::string & error_message) const;
  bool getCurrentState(moveit::core::RobotState & state, std::string & error_message) const;
  CollisionSummary collisionSummary(const moveit::core::RobotState & state) const;
  bool isStateCollisionFree(
    const moveit::core::RobotState & state,
    CollisionSummary & summary) const;
  bool planTrajectory(
    const PlanBimanualJointTrajectory::Goal & goal,
    double planning_time,
    double goal_tolerance,
    robot_trajectory::RobotTrajectoryPtr & trajectory,
    std::string & error_message);
  trajectory_msgs::msg::JointTrajectory extractJointTrajectory(
    const robot_trajectory::RobotTrajectory & trajectory,
    const std::vector<std::string> & joint_names) const;
  bool executeTrajectory(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    std::string & error_message);
  trajectory_msgs::msg::JointTrajectory filterTrajectory(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const std::vector<std::string> & joint_names) const;
  bool sendTrajectoryToController(
    const std::shared_ptr<rclcpp_action::Client<FollowJointTrajectory>> & client,
    const std::string & action_name,
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    std::string & error_message);
  double computeFinalErrorNorm(
    const moveit::core::RobotState & state,
    const std::vector<std::string> & joint_names,
    const std::vector<double> & target) const;
  bool publishFeedback(
    const std::shared_ptr<GoalHandlePlanBimanualJointTrajectory> goal_handle,
    const std::string & state,
    const std::string & message,
    const CollisionSummary & collision) const;
  void publishDisplayTrajectory(const robot_trajectory::RobotTrajectory & trajectory) const;
  void publishCollisionDebug(
    const CollisionSummary & summary,
    const std::string & frame_id = "world_ned") const;
  void requestStop();
  void joinWorkerThread();
  void cleanupResources();
  std::string namespacedTopic(const std::string & suffix) const;
  std::string namespacedMoveItTopic(const std::string & suffix) const;

  rclcpp_action::Server<PlanBimanualJointTrajectory>::SharedPtr action_server_;
  std::shared_ptr<rclcpp_action::Client<FollowJointTrajectory>> left_controller_client_;
  std::shared_ptr<rclcpp_action::Client<FollowJointTrajectory>> right_controller_client_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_trajectory_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr collision_debug_pub_;

  rclcpp::Node::SharedPtr moveit_node_;
  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> moveit_executor_;
  std::thread moveit_spin_thread_;
  robot_model_loader::RobotModelLoaderPtr robot_model_loader_;
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
  planning_pipeline::PlanningPipelinePtr planning_pipeline_;

  mutable std::mutex moveit_mutex_;
  std::thread worker_thread_;
  std::atomic_bool busy_{false};
  std::atomic_bool stop_requested_{false};

  std::string robot_namespace_;
  std::string moveit_namespace_;
  std::string action_name_;
  std::string joint_states_topic_;
  std::string display_trajectory_topic_;
  std::string planning_scene_topic_;
  std::string collision_debug_topic_;
  std::string group_name_;
  std::string left_controller_action_;
  std::string right_controller_action_;
  std::string planning_pipeline_namespace_;
  std::string planning_plugin_;
  std::string request_adapters_;
  std::vector<std::string> left_joint_names_;
  std::vector<std::string> right_joint_names_;
  double default_planning_time_{5.0};
  double default_goal_tolerance_{0.02};
  double controller_wait_timeout_{5.0};
  double current_state_timeout_{2.0};
  double planning_scene_publish_rate_{10.0};
  double execution_feedback_rate_{10.0};
};

}  // namespace sura_manipulation_actions
