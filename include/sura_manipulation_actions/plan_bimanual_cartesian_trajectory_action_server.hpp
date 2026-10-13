#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sura_manipulation_actions/action/plan_bimanual_cartesian_trajectory.hpp"
#include "sura_manipulation_actions/bimanual_cartesian_planner.hpp"
#include "sura_manipulation_actions/cartesian_moveit_context.hpp"
#include "sura_manipulation_actions/cartesian_trajectory_visualization.hpp"
#include "sura_manipulation_actions/trajectory_execution.hpp"

namespace sura_manipulation_actions
{

class PlanBimanualCartesianTrajectoryActionServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  using Action = sura_manipulation_actions::action::PlanBimanualCartesianTrajectory;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  explicit PlanBimanualCartesianTrajectoryActionServer(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PlanBimanualCartesianTrajectoryActionServer() override;

private:
  enum class Stage {Idle, Planning, Pending, Executing};
  struct PendingPlan
  {
    std::shared_ptr<Handle> handle;
    bimanual_cartesian::Arm arm_a, arm_b;
    bimanual_cartesian::Pair pair;
    bimanual_cartesian::GoalSpec goal;
    bimanual_cartesian::Options options;
    bimanual_cartesian::PlannedTrajectory plan;
    CartesianViews views;
    Eigen::Isometry3d fixed_from_model = Eigen::Isometry3d::Identity();
    std::atomic_bool finished{false};
  };

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override;
  bool loadConfiguration(std::string & error);
  bool validateGoal(const Action::Goal & goal, PendingPlan & pending, std::string & error) const;
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal);
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<Handle> & handle);
  void handleAccepted(const std::shared_ptr<Handle> & handle);
  void planGoal(const std::shared_ptr<PendingPlan> & pending);
  void executePlan(const std::shared_ptr<PendingPlan> & pending);
  bool backendReady(const PendingPlan & pending, std::string & error);
  bool validatePending(const PendingPlan & pending, std::string & error) const;
  void executeTrigger(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void cancelTrigger(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void feedback(
    const std::shared_ptr<Handle> & handle, float progress,
    const std::string & stage, const std::string & message) const;
  void finish(const std::shared_ptr<PendingPlan> & pending, bool success, const std::string & message);
  void retain(const std::shared_ptr<PendingPlan> & pending, const std::string & message);
  void stop(const std::string & message);
  void cleanupResources();
  std::string topic(const std::string & suffix) const;

  std::string robot_namespace_, moveit_namespace_, action_name_, fixed_frame_, execution_backend_;
  std::string relative_reference_frame_;
  double state_timeout_{2.0}, state_max_age_{2.0}, planning_time_{5.0}, cartesian_step_{0.005};
  double start_joint_tolerance_{0.001}, start_prismatic_tolerance_{0.0001};
  double base_position_tolerance_{0.005}, base_orientation_tolerance_{0.01};
  bimanual_cartesian::Options planner_options_;
  ExecutionOptions execution_options_;
  std::map<std::string, bimanual_cartesian::Arm> arms_;
  std::map<std::string, bimanual_cartesian::Pair> pairs_;
  std::unique_ptr<CartesianMoveItContext> context_;
  std::unique_ptr<CartesianTrajectoryVisualization> visualization_;
  std::unique_ptr<TrajectoryExecutor> executor_;
  rclcpp::Node::SharedPtr moveit_node_;
  rclcpp::CallbackGroup::SharedPtr clients_group_;
  rclcpp_action::Server<Action>::SharedPtr action_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_service_, cancel_service_;
  rclcpp::TimerBase::SharedPtr cancel_timer_;
  std::map<std::string, rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr> managers_;
  mutable std::mutex mutex_;
  Stage stage_{Stage::Idle};
  std::shared_ptr<PendingPlan> pending_;
  std::thread worker_;
  std::atomic_bool stopped_{false};
  std::atomic_bool execution_active_{false};
};

}  // namespace sura_manipulation_actions
