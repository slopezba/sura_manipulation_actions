#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sura_manipulation_actions/action/plan_cartesian_trajectory.hpp"
#include "sura_manipulation_actions/action/follow_cartesian_trajectory.hpp"
#include "sura_manipulation_actions/single_arm_cartesian_planner.hpp"
#include "sura_manipulation_actions/cartesian_moveit_context.hpp"
#include "sura_manipulation_actions/cartesian_trajectory_visualization.hpp"
#include "sura_manipulation_actions/trajectory_execution.hpp"
#include "sura_manipulation_actions/cartesian_pose_path.hpp"

namespace sura_manipulation_actions
{

class PlanCartesianTrajectoryActionServer : public rclcpp_lifecycle::LifecycleNode
{
public:
  using Action = sura_manipulation_actions::action::PlanCartesianTrajectory;
  using Handle = rclcpp_action::ServerGoalHandle<Action>;
  using FollowAction = sura_manipulation_actions::action::FollowCartesianTrajectory;
  using FollowHandle = rclcpp_action::ClientGoalHandle<FollowAction>;
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
  explicit PlanCartesianTrajectoryActionServer(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~PlanCartesianTrajectoryActionServer() override;

private:
  enum class Stage {Idle, Planning, Pending, Executing};
  struct TaskEndpoint
  {
    std::string action, controller_name, manager_service;
  };
  struct PoseEndpoint
  {
    std::string controller_node, task_id, target_topic, feedforward_topic;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr feedforward;
  };
  struct PendingPlan
  {
    std::shared_ptr<Handle> handle;
    bimanual_cartesian::Arm arm;
    std::string backend;
    bimanual_cartesian::GoalSpec goal;
    bimanual_cartesian::Options options;
    bimanual_cartesian::PlannedTrajectory plan;
    CartesianViews views;
    CartesianPosePath pose_path;
    Eigen::Isometry3d pose_start = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d fixed_from_model = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d base_reference = Eigen::Isometry3d::Identity();
    sura_manipulation_actions::msg::BaseMotionLimits base_limits;
    sura_manipulation_actions::msg::TrajectoryTrackingTolerances tracking_tolerances;
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
  void planPoseGoal(const std::shared_ptr<PendingPlan> & pending);
  void executePosePlan(const std::shared_ptr<PendingPlan> & pending);
  bool poseBackendReady(const PendingPlan & pending, bool & auto_update, std::string & error);
  bool validatePosePending(const PendingPlan & pending, std::string & error) const;
  bool readTipPose(const PendingPlan & pending, Eigen::Isometry3d & pose, std::string & error) const;
  void executeCoordinated(const std::shared_ptr<PendingPlan> & pending);
  void cancelCoordinated();
  bool checkControllerTracking(
    const PendingPlan & pending, double elapsed, bool final, std::string & error) const;
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
  std::string reference_frame_;
  std::string base_frame_, coordinated_action_, base_manager_service_, base_controller_name_;
  double state_timeout_{2.0}, state_max_age_{2.0}, planning_time_{5.0}, cartesian_step_{0.02};
  double start_joint_tolerance_{0.001}, start_prismatic_tolerance_{0.0001};
  double base_position_tolerance_{0.005}, base_orientation_tolerance_{0.01};
  double pose_linear_velocity_{1.0}, pose_angular_velocity_{0.5};
  double pose_linear_acceleration_{0.4}, pose_angular_acceleration_{0.2};
  bimanual_cartesian::Options planner_options_;
  sura_manipulation_actions::msg::TrajectoryTrackingTolerances default_tracking_tolerances_;
  ExecutionOptions execution_options_;
  std::map<std::string, bimanual_cartesian::Arm> arms_;
  std::map<std::string, TaskEndpoint> tasks_;
  std::map<std::string, PoseEndpoint> pose_endpoints_;
  std::unique_ptr<tf2_ros::Buffer> pose_tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> pose_tf_listener_;
  std::unique_ptr<CartesianMoveItContext> context_;
  std::unique_ptr<CartesianTrajectoryVisualization> visualization_;
  std::unique_ptr<TrajectoryExecutor> executor_;
  struct FollowDispatch
  {
    std::mutex mutex;
    bool canceled{false};
    FollowHandle::SharedPtr handle;
  };
  rclcpp_action::Client<FollowAction>::SharedPtr coordinated_client_;
  std::shared_ptr<FollowDispatch> follow_dispatch_;
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
