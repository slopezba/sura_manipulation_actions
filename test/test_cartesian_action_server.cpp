#include "sura_manipulation_actions/plan_cartesian_trajectory_action_server.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "rcl_interfaces/srv/get_parameters.hpp"
#include "rcl/time.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sura_manipulation_actions/action/follow_cartesian_trajectory.hpp"
#include "sura_manipulation_actions/cartesian_trajectory_timing.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace sura_manipulation_actions
{
namespace
{
using PlanAction = sura_manipulation_actions::action::PlanCartesianTrajectory;
using PlanHandle = rclcpp_action::ClientGoalHandle<PlanAction>;
using FollowAction = control_msgs::action::FollowJointTrajectory;
using FollowHandle = rclcpp_action::ServerGoalHandle<FollowAction>;
using CartesianFollowAction = sura_manipulation_actions::action::FollowCartesianTrajectory;
using CartesianFollowHandle = rclcpp_action::ServerGoalHandle<CartesianFollowAction>;
using Trigger = std_srvs::srv::Trigger;

const char * kUrdf = R"(
<robot name="cartesian_action_fixture">
  <link name="base"/>
  <link name="left_tip"/>
  <link name="right_tip"/>
  <joint name="left_joint" type="prismatic">
    <parent link="base"/><child link="left_tip"/>
    <origin xyz="-1 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-0.5" upper="0.5" velocity="1" effort="10"/>
  </joint>
  <joint name="right_joint" type="prismatic">
    <parent link="base"/><child link="right_tip"/>
    <origin xyz="1 0 0"/><axis xyz="1 0 0"/>
    <limit lower="-0.5" upper="0.5" velocity="1" effort="10"/>
  </joint>
</robot>)";

const char * kSrdf = R"(
<robot name="cartesian_action_fixture">
  <group name="left"><chain base_link="base" tip_link="left_tip"/></group>
  <group name="right"><chain base_link="base" tip_link="right_tip"/></group>
  <group name="both"><group name="left"/><group name="right"/></group>
</robot>)";

bool waitUntil(const std::function<bool()> & predicate, double timeout = 5.0)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {return true;}
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return predicate();
}

bool zeroTwist(const geometry_msgs::msg::Twist & twist)
{
  return twist.linear.x == 0.0 && twist.linear.y == 0.0 && twist.linear.z == 0.0 &&
         twist.angular.x == 0.0 && twist.angular.y == 0.0 && twist.angular.z == 0.0;
}

class SingleCartesianActionServerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    static unsigned sequence = 0;
    suffix_ = std::to_string(++sequence);
    ns_ = "/single_cartesian_action_fixture_" + suffix_;
    server_name_ = "single_cartesian_action_server_" + suffix_;
    fake_ = std::make_shared<rclcpp::Node>("single_cartesian_action_clients_" + suffix_);
    fake_->declare_parameter("world_frame", std::string("base"));
    for (const auto & side : {"left", "right"}) {
      const auto prefix = "tasks." + std::string(side) + "_pose.";
      fake_->declare_parameter(prefix + "frame_id", std::string(side) + "_tip");
      fake_->declare_parameter(prefix + "plugin", std::string("task_priority_kinematic_control/FramePoseTask"));
      fake_->declare_parameter(prefix + "auto_update_target_pose", true);
    }
    dynamic_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(fake_);
    auto node_options = rclcpp::NodeOptions();
    node_options.arguments({"--ros-args", "-r", "__node:=" + server_name_});
    node_options.parameter_overrides({
      rclcpp::Parameter("robot_namespace", ns_),
      rclcpp::Parameter("robot_description", std::string(kUrdf)),
      rclcpp::Parameter("robot_description_semantic", std::string(kSrdf)),
      rclcpp::Parameter("joint_states_topic", ns_ + "/joint_states"),
      rclcpp::Parameter("execution_backend", "controller"),
      rclcpp::Parameter("execution.coordinated_action", ns_ + "/follow_cartesian_trajectory"),
      rclcpp::Parameter("execution.base_manager_service", ns_ + "/controller_manager/list_controllers"),
      rclcpp::Parameter("arm_names", std::vector<std::string>{"A", "B"}),
      rclcpp::Parameter("arms.A.group", "left"),
      rclcpp::Parameter("arms.A.tip_frame", "left_tip"),
      rclcpp::Parameter("arms.A.task_action", ns_ + "/left_task/follow_joint_trajectory"),
      rclcpp::Parameter("arms.A.task_controller_name", "task_priority_controller"),
      rclcpp::Parameter("arms.A.task_manager_service", ns_ + "/controller_manager/list_controllers"),
      rclcpp::Parameter("arms.A.pose_controller_node", std::string(fake_->get_fully_qualified_name())),
      rclcpp::Parameter("arms.A.pose_task", "left_pose"),
      rclcpp::Parameter("arms.A.pose_target_topic", ns_ + "/task_priority/tasks/left_pose/target"),
      rclcpp::Parameter("arms.A.controller_action", ns_ + "/left/follow_joint_trajectory"),
      rclcpp::Parameter("arms.A.controller_name", "left_controller"),
      rclcpp::Parameter("arms.A.manager_service", ns_ + "/controller_manager/list_controllers"),
      rclcpp::Parameter("arms.B.group", "right"),
      rclcpp::Parameter("arms.B.tip_frame", "right_tip"),
      rclcpp::Parameter("arms.B.task_action", ns_ + "/right_task/follow_joint_trajectory"),
      rclcpp::Parameter("arms.B.task_controller_name", "task_priority_controller"),
      rclcpp::Parameter("arms.B.task_manager_service", ns_ + "/controller_manager/list_controllers"),
      rclcpp::Parameter("arms.B.pose_controller_node", std::string(fake_->get_fully_qualified_name())),
      rclcpp::Parameter("arms.B.pose_task", "right_pose"),
      rclcpp::Parameter("arms.B.pose_target_topic", ns_ + "/task_priority/tasks/right_pose/target"),
      rclcpp::Parameter("arms.B.controller_action", ns_ + "/right/follow_joint_trajectory"),
      rclcpp::Parameter("arms.B.controller_name", "right_controller"),
      rclcpp::Parameter("arms.B.manager_service", ns_ + "/controller_manager/list_controllers"),
      rclcpp::Parameter("visualization.fixed_frame", "base"),
      rclcpp::Parameter("reference_frame", "base"),
      rclcpp::Parameter("planning_frame", "base"),
      rclcpp::Parameter("base_frame", "base"),
      rclcpp::Parameter("planning.default_planning_time", 3.0),
      rclcpp::Parameter("planning.optimize_trajectory", false),
      // Untyped empty YAML arrays may arrive as another ROS array type.
      rclcpp::Parameter("planning.limit_joint_names", std::vector<uint8_t>{}),
      rclcpp::Parameter("planning.lower_limits", std::vector<uint8_t>{}),
      rclcpp::Parameter("planning.upper_limits", std::vector<uint8_t>{}),
      rclcpp::Parameter("execution.controller_wait_timeout", 0.5),
      rclcpp::Parameter("execution.acceptance_timeout", 0.5),
      rclcpp::Parameter("execution.start_delay", 1.0),
      rclcpp::Parameter("pose.max_linear_velocity", 0.5),
      rclcpp::Parameter("pose.max_angular_velocity", 1.0),
      rclcpp::Parameter("pose.max_linear_acceleration", 1.0),
      rclcpp::Parameter("pose.max_angular_acceleration", 2.0),
      rclcpp::Parameter("robot_description_planning.joint_limits.left_joint.has_acceleration_limits", true),
      rclcpp::Parameter("robot_description_planning.joint_limits.left_joint.max_acceleration", 1.0),
      rclcpp::Parameter("robot_description_planning.joint_limits.right_joint.has_acceleration_limits", true),
      rclcpp::Parameter("robot_description_planning.joint_limits.right_joint.max_acceleration", 1.0),
    });
    server_ = std::make_shared<PlanCartesianTrajectoryActionServer>(node_options);
    joint_pub_ = fake_->create_publisher<sensor_msgs::msg::JointState>(ns_ + "/joint_states", rclcpp::SensorDataQoS());
    state_timer_ = fake_->create_wall_timer(std::chrono::milliseconds(5), [this]() {
      updateBackendState();
      publishMeasuredPoses();
      sensor_msgs::msg::JointState state;
      state.header.stamp = fake_->now();
      state.name = {"left_joint", "right_joint"};
      state.position = {left_position_.load(), right_position_.load()};
      joint_pub_->publish(state);
    });
    list_service_ = fake_->create_service<controller_manager_msgs::srv::ListControllers>(
      ns_ + "/controller_manager/list_controllers",
      [this](std::shared_ptr<controller_manager_msgs::srv::ListControllers::Request>,
      std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response> response) {
        for (const auto & name : {"left_controller", "right_controller", "task_priority_controller", "body_velocity"}) {
          controller_manager_msgs::msg::ControllerState state;
          state.name = name;
          state.state = backend_active_.load() ? "active" : "inactive";
          response->controller.push_back(state);
        }
      });
    switch_service_ = fake_->create_service<controller_manager_msgs::srv::SwitchController>(
      ns_ + "/controller_manager/switch_controller",
      [this](std::shared_ptr<controller_manager_msgs::srv::SwitchController::Request>,
      std::shared_ptr<controller_manager_msgs::srv::SwitchController::Response> response) {
        switch_requests_.fetch_add(1);
        response->ok = true;
      });
    for (const auto & arm : {"left", "right"}) {
      const auto endpoint = ns_ + "/" + arm + "/follow_joint_trajectory";
      backends_.push_back(rclcpp_action::create_server<FollowAction>(fake_, endpoint,
        [this, endpoint](const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowAction::Goal> goal) {
          backend_requests_.fetch_add(1);
          {std::lock_guard<std::mutex> lock(observation_mutex_);
            endpoint_ = endpoint; dispatched_ = goal->trajectory;}
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](std::shared_ptr<FollowHandle>) {
          backend_cancels_.fetch_add(1); return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<FollowHandle> handle) {
          std::lock_guard<std::mutex> lock(observation_mutex_); controller_handle_ = handle;
        }));
    }
    coordinated_backend_ = rclcpp_action::create_server<CartesianFollowAction>(fake_,
      ns_ + "/follow_cartesian_trajectory",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const CartesianFollowAction::Goal> goal) {
        backend_requests_.fetch_add(1);
        {std::lock_guard<std::mutex> lock(observation_mutex_);
          endpoint_ = ns_ + "/follow_cartesian_trajectory";
          dispatched_ = goal->joint_trajectory; coordinated_goal_ = *goal;}
        return task_enabled_.load() ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
          rclcpp_action::GoalResponse::REJECT;
      },
      [this](std::shared_ptr<CartesianFollowHandle>) {
        backend_cancels_.fetch_add(1); return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<CartesianFollowHandle> handle) {
        std::lock_guard<std::mutex> lock(observation_mutex_); coordinated_handle_ = handle;
      });
    tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(fake_);
    for (std::size_t side = 0; side < 2; ++side) {
      const std::string prefix = ns_ + "/task_priority/tasks/" + (side == 0 ? "left_pose" : "right_pose");
      pose_target_subs_.push_back(fake_->create_subscription<geometry_msgs::msg::PoseStamped>(
        prefix + "/target", rclcpp::QoS(10),
        [this, side](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(observation_mutex_);
          pose_targets_[side].push_back(*msg);
          if (pose_tracks_.load()) {measured_poses_[side] = msg->pose;}
        }));
      pose_feedforward_subs_.push_back(fake_->create_subscription<geometry_msgs::msg::TwistStamped>(
        prefix + "/feedforward", rclcpp::QoS(10),
        [this, side](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
          std::lock_guard<std::mutex> lock(observation_mutex_);
          pose_feedforwards_[side].push_back(*msg);
        }));
    }
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    trajectory_sub_ = fake_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      ns_ + "/cartesian_planner/joint_trajectory", qos,
      [this](trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(observation_mutex_); published_ = *msg;
      });
    relative_sub_ = fake_->create_subscription<nav_msgs::msg::Path>(
      ns_ + "/cartesian_planner/path", qos,
      [this](nav_msgs::msg::Path::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(observation_mutex_);
        relative_view_ = *msg;
        visual_refreshes_.fetch_add(1);
      });
    markers_sub_ = fake_->create_subscription<visualization_msgs::msg::MarkerArray>(
      ns_ + "/manipulation/marker_plans", rclcpp::QoS(10).reliable(),
      [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(observation_mutex_); markers_ = *msg;
      });
    display_sub_ = fake_->create_subscription<moveit_msgs::msg::DisplayTrajectory>(
      ns_ + "/moveit/display_planned_path", qos,
      [this](moveit_msgs::msg::DisplayTrajectory::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(observation_mutex_); display_ = *msg;
      });
    client_ = rclcpp_action::create_client<PlanAction>(fake_, ns_ + "/manipulation/plan_cartesian_trajectory");
    execute_ = fake_->create_client<Trigger>("/" + server_name_ + "/execute_pending_trajectory");
    cancel_ = fake_->create_client<Trigger>("/" + server_name_ + "/cancel_pending_trajectory");
    executor_.add_node(fake_);
    executor_.add_node(server_->get_node_base_interface());
    spin_ = std::thread([this]() {executor_.spin();});
    ASSERT_EQ(server_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(server_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    ASSERT_TRUE(client_->wait_for_action_server(std::chrono::seconds(2)));
  }

  void TearDown() override
  {
    // Stop the worker while ROS callbacks are still serviced.
    server_->deactivate();
    static_cast<rclcpp_lifecycle::LifecycleNode &>(*server_).cleanup();
    executor_.cancel();
    spin_.join();
    executor_.remove_node(server_->get_node_base_interface());
    executor_.remove_node(fake_);
    server_.reset();
    fake_.reset();
  }

  PlanAction::Goal goal(bool execute = false, bool right = false, const std::string & backend = "") const
  {
    PlanAction::Goal result;
    result.arm = right ? "B" : "A";
    result.target_pose.header.frame_id = "base";
    result.target_pose.pose.position.x = right ? 1.01 : -0.99;
    result.target_pose.pose.orientation.w = 1.0;
    result.position_tolerance.x = 0.001;
    result.cartesian_step = 0.005;
    result.planning_time = 3.0;
    result.execution_backend = backend;
    result.execute = execute;
    return result;
  }

  PlanHandle::SharedPtr sendGoal(const PlanAction::Goal & goal)
  {
    rclcpp_action::Client<PlanAction>::SendGoalOptions options;
    options.feedback_callback = [this](PlanHandle::SharedPtr, std::shared_ptr<const PlanAction::Feedback> msg) {
        std::lock_guard<std::mutex> lock(observation_mutex_);
        stage_ = msg->current_stage; message_ = msg->message; last_feedback_ = *msg;
      };
    auto sent = client_->async_send_goal(goal, options);
    if (sent.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {return {};}
    return sent.get();
  }

  PlanAction::Goal poseGoal(bool execute = false, bool right = false) const
  {
    auto result = goal(execute, right, "pose");
    result.velocity_scaling = result.acceleration_scaling = 1.0;
    return result;
  }

  void publishMeasuredPoses()
  {
    std::vector<geometry_msgs::msg::TransformStamped> transforms(2);
    std::lock_guard<std::mutex> lock(observation_mutex_);
    for (std::size_t side = 0; side < transforms.size(); ++side) {
      auto & transform = transforms[side];
      transform.header.frame_id = "base";
      transform.header.stamp = fake_->now();
      transform.child_frame_id = side == 0 ? "left_tip" : "right_tip";
      transform.transform.translation.x = measured_poses_[side].position.x;
      transform.transform.translation.y = measured_poses_[side].position.y;
      transform.transform.translation.z = measured_poses_[side].position.z;
      transform.transform.rotation = measured_poses_[side].orientation;
    }
    dynamic_tf_broadcaster_->sendTransform(transforms);
  }

  bool automaticTarget(bool right = false) const
  {
    return fake_->get_parameter(right ? "tasks.right_pose.auto_update_target_pose" :
      "tasks.left_pose.auto_update_target_pose").as_bool();
  }

  bool overrideClock(const rclcpp::Clock::SharedPtr & clock, int64_t time_ns)
  {
    std::lock_guard<std::mutex> lock(clock->get_clock_mutex());
    return rcl_set_ros_time_override(clock->get_clock_handle(), time_ns) == RCL_RET_OK &&
      rcl_enable_ros_time_override(clock->get_clock_handle()) == RCL_RET_OK;
  }

  bool waiting(const std::string & diagnostic = {}) const
  {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    return stage_ == "waiting_for_execute" && message_.find(diagnostic) != std::string::npos;
  }

  std::shared_ptr<Trigger::Response> trigger(const rclcpp::Client<Trigger>::SharedPtr & client)
  {
    if (!client->wait_for_service(std::chrono::seconds(1))) {return {};}
    auto response = client->async_send_request(std::make_shared<Trigger::Request>());
    if (response.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {return {};}
    return response.get();
  }

  void setMeasuredJoints(const trajectory_msgs::msg::JointTrajectory & trajectory,
    const std::vector<double> & positions)
  {
    for (std::size_t i = 0; i < trajectory.joint_names.size(); ++i) {
      if (trajectory.joint_names[i] == "left_joint") {left_position_.store(positions.at(i));}
      if (trajectory.joint_names[i] == "right_joint") {right_position_.store(positions.at(i));}
    }
  }

  void updateBackendState()
  {
    std::shared_ptr<FollowHandle> controller;
    std::shared_ptr<CartesianFollowHandle> coordinated;
    {std::lock_guard<std::mutex> lock(observation_mutex_);
      controller = controller_handle_; coordinated = coordinated_handle_;}
    if (controller) {
      auto result = std::make_shared<FollowAction::Result>();
      if (controller->is_canceling()) {
        result->error_code = FollowAction::Result::SUCCESSFUL;
        controller->canceled(result);
        std::lock_guard<std::mutex> lock(observation_mutex_); controller_handle_.reset();
      } else if (controller->is_active()) {
        const auto & trajectory = controller->get_goal()->trajectory;
        const double elapsed = (fake_->now() - rclcpp::Time(
          trajectory.header.stamp, fake_->get_clock()->get_clock_type())).seconds();
        const auto & points = trajectory.points;
        if (!points.empty() && elapsed >= 0.0 && controller_tracks_.load()) {
          auto sample = points.back();
          for (std::size_t i = 1; i < points.size(); ++i) {
            const double begin = bimanual_cartesian::timing::seconds(points[i - 1].time_from_start);
            const double end = bimanual_cartesian::timing::seconds(points[i].time_from_start);
            if (elapsed <= end) {
              bimanual_cartesian::timing::interpolateSegment(points[i - 1], points[i],
                std::clamp((elapsed - begin) / (end - begin), 0.0, 1.0), end - begin, sample);
              break;
            }
          }
          setMeasuredJoints(trajectory, sample.positions);
        }
        // Let the state publication at the final waypoint reach MoveIt before success.
        if (!points.empty() && elapsed >= bimanual_cartesian::timing::seconds(
            points.back().time_from_start) + 0.1) {
          result->error_code = FollowAction::Result::SUCCESSFUL;
          controller->succeed(result);
          std::lock_guard<std::mutex> lock(observation_mutex_); controller_handle_.reset();
        }
      }
    }
    if (coordinated && coordinated->is_active()) {
      auto result = std::make_shared<CartesianFollowAction::Result>();
      if (coordinated->is_canceling()) {
        result->message = "Fake coordinated execution canceled";
        coordinated->canceled(result);
        std::lock_guard<std::mutex> lock(observation_mutex_); coordinated_handle_.reset();
      } else if (!hold_coordinated_.load()) {
        const auto & joints = coordinated->get_goal()->joint_trajectory;
        if (!joints.points.empty()) {setMeasuredJoints(joints, joints.points.back().positions);}
        result->success = true; result->message = "Fake synchronized execution completed";
        coordinated->succeed(result);
        std::lock_guard<std::mutex> lock(observation_mutex_); coordinated_handle_.reset();
      } else {
        auto feedback = std::make_shared<CartesianFollowAction::Feedback>();
        feedback->progress = 0.5F; feedback->current_stage = "tracking";
        feedback->message = "Fake synchronized tracking";
        feedback->base_error[0] = 0.01; feedback->joint_error = {0.02}; feedback->tip_error[1] = 0.03;
        coordinated->publish_feedback(feedback);
      }
    }
  }

  std::string suffix_, ns_, server_name_;
  std::shared_ptr<PlanCartesianTrajectoryActionServer> server_;
  rclcpp::Node::SharedPtr fake_;
  rclcpp::executors::MultiThreadedExecutor executor_{rclcpp::ExecutorOptions(), 4};
  std::thread spin_;
  rclcpp_action::Client<PlanAction>::SharedPtr client_;
  rclcpp::Client<Trigger>::SharedPtr execute_, cancel_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr state_timer_;
  rclcpp::Service<controller_manager_msgs::srv::ListControllers>::SharedPtr list_service_;
  rclcpp::Service<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_service_;
  std::vector<rclcpp_action::Server<FollowAction>::SharedPtr> backends_;
  rclcpp_action::Server<CartesianFollowAction>::SharedPtr coordinated_backend_;
  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> pose_target_subs_;
  std::vector<rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr> pose_feedforward_subs_;
  std::shared_ptr<FollowHandle> controller_handle_;
  std::shared_ptr<CartesianFollowHandle> coordinated_handle_;
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr relative_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr markers_sub_;
  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_sub_;
  std::atomic_bool backend_active_{false}, task_enabled_{true};
  std::atomic_bool controller_tracks_{true}, hold_coordinated_{false};
  std::atomic_bool pose_tracks_{true};
  std::atomic_int backend_requests_{0}, switch_requests_{0}, backend_cancels_{0};
  std::atomic_int visual_refreshes_{0};
  std::atomic<double> left_position_{0.0}, right_position_{0.0};
  mutable std::mutex observation_mutex_;
  std::string stage_, message_;
  trajectory_msgs::msg::JointTrajectory published_, dispatched_;
  std::string endpoint_;
  nav_msgs::msg::Path relative_view_;
  visualization_msgs::msg::MarkerArray markers_;
  moveit_msgs::msg::DisplayTrajectory display_;
  CartesianFollowAction::Goal coordinated_goal_;
  PlanAction::Feedback last_feedback_;
  std::array<geometry_msgs::msg::Pose, 2> measured_poses_ = []() {
      std::array<geometry_msgs::msg::Pose, 2> poses;
      poses[0].position.x = -1.0; poses[1].position.x = 1.0;
      poses[0].orientation.w = poses[1].orientation.w = 1.0;
      return poses;
    }();
  std::vector<geometry_msgs::msg::PoseStamped> pose_targets_[2];
  std::vector<geometry_msgs::msg::TwistStamped> pose_feedforwards_[2];
};

TEST_F(SingleCartesianActionServerTest, PendingPlanHasOnlySelectedArmAndAbsoluteFkPath)
{
  auto handle = sendGoal(goal());
  ASSERT_TRUE(handle);
  auto result = client_->async_get_result(handle);
  ASSERT_TRUE(waitUntil([this]() {return waiting();}));
  EXPECT_EQ(backend_requests_.load(), 0);
  ASSERT_TRUE(waitUntil([this]() {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    return !markers_.markers.empty() && visual_refreshes_.load() >= 2;
  }));
  auto cancel = client_->async_cancel_goal(handle);
  ASSERT_EQ(cancel.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  ASSERT_EQ(result.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  const auto finished = result.get();
  ASSERT_TRUE(finished.result);
  EXPECT_EQ(finished.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_EQ(finished.result->joint_trajectory.joint_names, (std::vector<std::string>{"left_joint"}));
  ASSERT_FALSE(finished.result->path.poses.empty());
  EXPECT_EQ(finished.result->path.header.frame_id, "base");
  EXPECT_NEAR(finished.result->path.poses.back().pose.position.x, -0.99, 0.001001);
  EXPECT_EQ(finished.result->desired_path.poses.size(), finished.result->path.poses.size());
  std::lock_guard<std::mutex> lock(observation_mutex_);
  EXPECT_EQ(std::count_if(markers_.markers.begin(), markers_.markers.end(),
    [](const auto & marker) {return marker.action == visualization_msgs::msg::Marker::ADD &&
      marker.type == visualization_msgs::msg::Marker::ARROW;}), 6);
  ASSERT_EQ(markers_.markers.size(), 9u);  // Base/tool paths and XYZ triads.
  std::set<std::pair<std::string, int>> marker_keys;
  for (const auto & marker : markers_.markers) {
    EXPECT_TRUE(marker_keys.emplace(marker.ns, marker.id).second)
      << "Duplicate marker in the same message: " << marker.ns << "/" << marker.id;
    EXPECT_NE(marker.action, visualization_msgs::msg::Marker::DELETEALL);
    EXPECT_EQ(marker.ns.find("cartesian/"), 0u);
    EXPECT_EQ(marker.header.frame_id, "base");
    EXPECT_TRUE(marker.frame_locked);
    if (marker.action != visualization_msgs::msg::Marker::ADD ||
      marker.type != visualization_msgs::msg::Marker::ARROW) {continue;}
    EXPECT_EQ(marker.type, visualization_msgs::msg::Marker::ARROW);
    ASSERT_EQ(marker.points.size(), 2u);
    EXPECT_FLOAT_EQ(marker.color.r, marker.id == 0 ? 1.0F : 0.0F);
    EXPECT_FLOAT_EQ(marker.color.g, marker.id == 1 ? 1.0F : 0.0F);
    EXPECT_FLOAT_EQ(marker.color.b, marker.id == 2 ? 1.0F : 0.0F);
    const auto & pose = marker.ns == "cartesian/target" ?
      goal().target_pose.pose : finished.result->path.poses.back().pose;
    EXPECT_EQ(marker.points.front(), pose.position);
    const double length = marker.ns == "cartesian/target" ? 0.06 : 0.04;
    // This fixture has identity orientation: the triad endpoints must be XYZ.
    EXPECT_NEAR(marker.points.back().x - pose.position.x, marker.id == 0 ? length : 0.0, 1e-9);
    EXPECT_NEAR(marker.points.back().y - pose.position.y, marker.id == 1 ? length : 0.0, 1e-9);
    EXPECT_NEAR(marker.points.back().z - pose.position.z, marker.id == 2 ? length : 0.0, 1e-9);
  }
  EXPECT_EQ(std::count_if(markers_.markers.begin(), markers_.markers.end(),
    [](const auto & marker) {return marker.action == visualization_msgs::msg::Marker::ADD &&
      marker.type == visualization_msgs::msg::Marker::LINE_STRIP;}), 3);
  for (const auto & path : markers_.markers) {
    EXPECT_NE(path.action, visualization_msgs::msg::Marker::DELETEALL);
    EXPECT_EQ(path.ns.find("cartesian/"), 0u);
    EXPECT_TRUE(path.frame_locked);
    if (path.action != visualization_msgs::msg::Marker::ADD ||
      path.type != visualization_msgs::msg::Marker::LINE_STRIP) {continue;}
    EXPECT_EQ(path.type, visualization_msgs::msg::Marker::LINE_STRIP);
    const auto & poses = path.ns == "cartesian/base_path" ? finished.result->base_path.poses :
      path.ns == "cartesian/planned_path" ? finished.result->path.poses : finished.result->desired_path.poses;
    ASSERT_EQ(path.points.size(), poses.size());
    for (std::size_t i = 0; i < poses.size(); ++i) {
      EXPECT_EQ(path.points[i], poses[i].pose.position);
    }
    if (path.ns == "cartesian/planned_path") {EXPECT_FLOAT_EQ(path.color.g, 1.0F);}
    else if (path.ns == "cartesian/desired_path") {
      EXPECT_FLOAT_EQ(path.color.r, 0.7F);
      EXPECT_FLOAT_EQ(path.color.b, 1.0F);
      EXPECT_DOUBLE_EQ(path.scale.x, 0.006);
    }
  }
}

TEST_F(SingleCartesianActionServerTest, PendingPosePlanContainsOnlyCartesianReferences)
{
  auto handle = sendGoal(poseGoal());
  ASSERT_TRUE(handle);
  auto result = client_->async_get_result(handle);
  ASSERT_TRUE(waitUntil([this]() {return waiting();}));
  EXPECT_EQ(backend_requests_.load(), 0);
  EXPECT_TRUE(automaticTarget());
  ASSERT_TRUE(waitUntil([this]() {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    return std::any_of(markers_.markers.begin(), markers_.markers.end(), [](const auto & marker) {
      return marker.ns == "cartesian/desired_path" &&
             marker.action == visualization_msgs::msg::Marker::ADD;
    });
  }));
  {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    EXPECT_TRUE(pose_targets_[0].empty());
    EXPECT_TRUE(pose_targets_[1].empty());
    // The shared publisher accepts complete ADD-only snapshots and handles
    // removal of the previous plan on manipulation/markers.
    EXPECT_EQ(markers_.markers.size(), 4u);  // Desired path and target XYZ triad.
    for (const auto & marker : markers_.markers) {
      EXPECT_EQ(marker.action, visualization_msgs::msg::Marker::ADD);
    }
  }
  auto canceled = client_->async_cancel_goal(handle);
  ASSERT_EQ(canceled.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  ASSERT_EQ(result.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  const auto finished = result.get();
  ASSERT_TRUE(finished.result);
  EXPECT_EQ(finished.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_TRUE(finished.result->joint_trajectory.joint_names.empty());
  EXPECT_TRUE(finished.result->joint_trajectory.points.empty());
  EXPECT_TRUE(finished.result->base_trajectory.joint_names.empty());
  EXPECT_TRUE(finished.result->base_trajectory.points.empty());
  const auto & path = finished.result->desired_path;
  ASSERT_GE(path.poses.size(), 2u);
  EXPECT_EQ(path.header.frame_id, "base");
  EXPECT_NEAR(path.poses.front().pose.position.x, -1.0, 1e-12);
  EXPECT_NEAR(path.poses.back().pose.position.x, -0.99, 1e-12);
}

TEST_F(SingleCartesianActionServerTest, PoseExecutionSelectsEachArmAndRestoresAutomaticTargets)
{
  backend_active_.store(true);
  for (std::size_t side = 0; side < 2; ++side) {
    SCOPED_TRACE(side == 0 ? "left_pose" : "right_pose");
    auto request = poseGoal(false, side == 1);
    const double angle = side == 0 ? 0.08 : -0.06;
    request.target_pose.pose.orientation.z = std::sin(angle / 2.0);
    request.target_pose.pose.orientation.w = std::cos(angle / 2.0);
    auto handle = sendGoal(request);
    ASSERT_TRUE(handle);
    auto result = client_->async_get_result(handle);
    ASSERT_TRUE(waitUntil([this]() {return waiting();}));
    ASSERT_TRUE(automaticTarget(side == 1));
    auto executed = trigger(execute_);
    ASSERT_TRUE(executed);
    ASSERT_TRUE(executed->success) << executed->message;
    ASSERT_TRUE(waitUntil([this, side]() {return !automaticTarget(side == 1);}));
    ASSERT_EQ(result.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    const auto finished = result.get();
    ASSERT_TRUE(finished.result);
    EXPECT_EQ(finished.code, rclcpp_action::ResultCode::SUCCEEDED);
    EXPECT_TRUE(finished.result->success) << finished.result->message;
    EXPECT_TRUE(automaticTarget(side == 1));
    ASSERT_TRUE(waitUntil([this, side]() {
      std::lock_guard<std::mutex> lock(observation_mutex_);
      return !pose_targets_[side].empty() && !pose_feedforwards_[side].empty() &&
             zeroTwist(pose_feedforwards_[side].back().twist) &&
             std::any_of(pose_feedforwards_[side].begin(), pose_feedforwards_[side].end(),
        [](const auto & reference) {return !zeroTwist(reference.twist);});
    }));
    std::lock_guard<std::mutex> lock(observation_mutex_);
    EXPECT_GE(pose_targets_[side].size(), 10u);
    EXPECT_EQ(pose_targets_[side].back().header.frame_id, "base");
    EXPECT_NEAR(measured_poses_[side].position.x, side == 0 ? -0.99 : 1.01, 0.001);
    EXPECT_NEAR(measured_poses_[side].orientation.z, std::sin(angle / 2.0), 0.001);
    EXPECT_TRUE(zeroTwist(pose_feedforwards_[side].front().twist));
    EXPECT_TRUE(zeroTwist(pose_feedforwards_[side].back().twist));
    bool moving = false;
    for (const auto & reference : pose_feedforwards_[side]) {
      EXPECT_EQ(reference.header.frame_id, "base");
      EXPECT_GE(reference.twist.linear.x, 0.0);
      EXPECT_LE(reference.twist.linear.x, 0.5 + 1.0e-9);
      EXPECT_DOUBLE_EQ(reference.twist.linear.y, 0.0);
      EXPECT_DOUBLE_EQ(reference.twist.linear.z, 0.0);
      EXPECT_DOUBLE_EQ(reference.twist.angular.x, 0.0);
      EXPECT_DOUBLE_EQ(reference.twist.angular.y, 0.0);
      EXPECT_LE(std::abs(reference.twist.angular.z), 1.0 + 1.0e-9);
      if (reference.twist.linear.x > 0.0) {
        moving = true;
        // Both velocities must share the pose trajectory's progress rate.
        EXPECT_NEAR(reference.twist.angular.z / reference.twist.linear.x, angle / 0.01, 1.0e-8);
      }
    }
    EXPECT_TRUE(moving);
    if (side == 0) {EXPECT_TRUE(pose_targets_[1].empty());}
  }
  EXPECT_EQ(backend_requests_.load(), 0);
  EXPECT_EQ(switch_requests_.load(), 0);
}

TEST_F(SingleCartesianActionServerTest, PoseExecutionPreservesInitiallyDisabledAutomaticTarget)
{
  backend_active_.store(true);
  ASSERT_TRUE(fake_->set_parameter(rclcpp::Parameter(
    "tasks.right_pose.auto_update_target_pose", false)).successful);
  auto handle = sendGoal(poseGoal(true, true));
  ASSERT_TRUE(handle);
  auto result = client_->async_get_result(handle);
  ASSERT_EQ(result.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  const auto finished = result.get();
  ASSERT_TRUE(finished.result);
  EXPECT_TRUE(finished.result->success) << finished.result->message;
  EXPECT_FALSE(automaticTarget(true));
  EXPECT_TRUE(automaticTarget());
  EXPECT_EQ(backend_requests_.load(), 0);
}

TEST_F(SingleCartesianActionServerTest, CancelingPoseExecutionHoldsMeasuredPoseAndRestoresTask)
{
  backend_active_.store(true);
  pose_tracks_.store(false);
  auto request = poseGoal(true);
  request.target_pose.pose.position.x = -0.8;
  request.tracking_tolerances.tip_path = std::vector<double>(6, 0.5);
  auto handle = sendGoal(request);
  ASSERT_TRUE(handle);
  auto result = client_->async_get_result(handle);
  ASSERT_TRUE(waitUntil([this]() {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    return !pose_targets_[0].empty() && pose_targets_[0].back().pose.position.x > -0.995;
  }));
  EXPECT_FALSE(automaticTarget());
  auto canceled = client_->async_cancel_goal(handle);
  ASSERT_EQ(canceled.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  ASSERT_EQ(result.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  const auto finished = result.get();
  EXPECT_EQ(finished.code, rclcpp_action::ResultCode::CANCELED);
  ASSERT_TRUE(finished.result);
  EXPECT_FALSE(finished.result->success);
  EXPECT_TRUE(automaticTarget());
  ASSERT_TRUE(waitUntil([this]() {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    return !pose_targets_[0].empty() && pose_targets_[0].back().pose == measured_poses_[0] &&
           !pose_feedforwards_[0].empty() && zeroTwist(pose_feedforwards_[0].back().twist);
  }));
  EXPECT_EQ(backend_requests_.load(), 0);
  EXPECT_EQ(backend_cancels_.load(), 0);
}

TEST_F(SingleCartesianActionServerTest, PoseExecutionRetainsPendingPlanWhenTaskFrameDoesNotMatch)
{
  backend_active_.store(true);
  ASSERT_TRUE(fake_->set_parameter(rclcpp::Parameter(
    "tasks.left_pose.frame_id", "wrong_controlled_frame")).successful);
  auto handle = sendGoal(poseGoal(true));
  ASSERT_TRUE(handle);
  auto result = client_->async_get_result(handle);
  ASSERT_TRUE(waitUntil([this]() {return waiting("frame");}));
  EXPECT_EQ(result.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);
  EXPECT_TRUE(automaticTarget());
  {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    EXPECT_TRUE(pose_targets_[0].empty());
  }
  ASSERT_TRUE(fake_->set_parameter(rclcpp::Parameter("tasks.left_pose.frame_id", "left_tip")).successful);
  auto executed = trigger(execute_);
  ASSERT_TRUE(executed);
  EXPECT_TRUE(executed->success) << executed->message;
  ASSERT_EQ(result.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  const auto finished = result.get();
  ASSERT_TRUE(finished.result);
  EXPECT_TRUE(finished.result->success) << finished.result->message;
  EXPECT_TRUE(automaticTarget());
  EXPECT_EQ(backend_requests_.load(), 0);
}

TEST_F(SingleCartesianActionServerTest, PoseExecutionAbortsOnCartesianTrackingErrorAndRestoresTask)
{
  backend_active_.store(true);
  pose_tracks_.store(false);
  auto request = poseGoal(true);
  request.target_pose.pose.position.x = -0.8;
  auto handle = sendGoal(request);
  ASSERT_TRUE(handle);
  auto result = client_->async_get_result(handle);
  ASSERT_EQ(result.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  const auto finished = result.get();
  ASSERT_TRUE(finished.result);
  EXPECT_EQ(finished.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_FALSE(finished.result->success);
  EXPECT_NE(finished.result->message.find("tracking tolerance"), std::string::npos);
  EXPECT_TRUE(automaticTarget());
  ASSERT_TRUE(waitUntil([this]() {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    return !pose_targets_[0].empty() && pose_targets_[0].back().pose == measured_poses_[0] &&
           !pose_feedforwards_[0].empty() && zeroTwist(pose_feedforwards_[0].back().twist);
  }));
  EXPECT_EQ(backend_requests_.load(), 0);
}

TEST_F(SingleCartesianActionServerTest, PoseExecutionPausesWithRosTimeAndCanStillBeCanceled)
{
  backend_active_.store(true);
  const auto initial_ns = fake_->now().nanoseconds();
  ASSERT_TRUE(overrideClock(fake_->get_clock(), initial_ns));
  ASSERT_TRUE(overrideClock(server_->get_clock(), initial_ns));
  auto request = poseGoal(true);
  request.target_pose.pose.position.x = -0.8;
  request.tracking_tolerances.tip_path = std::vector<double>(6, 0.5);
  auto handle = sendGoal(request);
  ASSERT_TRUE(handle);
  auto result = client_->async_get_result(handle);
  ASSERT_TRUE(waitUntil([this]() {return !automaticTarget();}));
  ASSERT_TRUE(waitUntil([this]() {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    return pose_targets_[0].size() >= 3;
  }));
  // Jump into the path, then leave ROS time frozen while wall callbacks run.
  ASSERT_TRUE(overrideClock(fake_->get_clock(), initial_ns + 200000000LL));
  ASSERT_TRUE(overrideClock(server_->get_clock(), initial_ns + 200000000LL));
  ASSERT_TRUE(waitUntil([this]() {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    return !pose_targets_[0].empty() && pose_targets_[0].back().pose.position.x > -0.999 &&
           !pose_feedforwards_[0].empty() && zeroTwist(pose_feedforwards_[0].back().twist) &&
           pose_feedforwards_[0].back().header.stamp == pose_targets_[0].back().header.stamp;
  }));
  geometry_msgs::msg::Pose frozen;
  std::size_t before, before_feedforward;
  {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    frozen = pose_targets_[0].back().pose;
    before = pose_targets_[0].size();
    before_feedforward = pose_feedforwards_[0].size();
  }
  ASSERT_TRUE(waitUntil([this, before, before_feedforward]() {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    return pose_targets_[0].size() >= before + 10 &&
           pose_feedforwards_[0].size() >= before_feedforward + 10;
  }));
  {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    for (std::size_t i = before; i < pose_targets_[0].size(); ++i) {
      EXPECT_EQ(pose_targets_[0][i].pose, frozen);
    }
    for (std::size_t i = before_feedforward; i < pose_feedforwards_[0].size(); ++i) {
      EXPECT_TRUE(zeroTwist(pose_feedforwards_[0][i].twist));
    }
  }
  EXPECT_EQ(result.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);
  EXPECT_FALSE(automaticTarget());
  auto canceled = client_->async_cancel_goal(handle);
  ASSERT_EQ(canceled.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  ASSERT_EQ(result.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  EXPECT_EQ(result.get().code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_TRUE(automaticTarget());
  EXPECT_EQ(backend_requests_.load(), 0);
}

TEST_F(SingleCartesianActionServerTest, SelectedControllerReceivesOnlyLeftTrajectory)
{
  backend_active_.store(true);
  auto request = goal(true, false, "controller");
  request.velocity_scaling = request.acceleration_scaling = 0.5;
  auto handle = sendGoal(request);
  ASSERT_TRUE(handle);
  auto result = client_->async_get_result(handle);
  ASSERT_EQ(result.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  ASSERT_TRUE(result.get().result->success);
  EXPECT_EQ(backend_requests_.load(), 1);
  std::lock_guard<std::mutex> lock(observation_mutex_);
  EXPECT_EQ(endpoint_, ns_ + "/left/follow_joint_trajectory");
  EXPECT_EQ(dispatched_.joint_names, (std::vector<std::string>{"left_joint"}));
  EXPECT_GT(dispatched_.header.stamp.sec, 0);
  EXPECT_EQ(switch_requests_.load(), 0);
}

TEST_F(SingleCartesianActionServerTest, SelectedTaskReceivesRightArmAndFixedBaseWithCommonFutureStart)
{
  backend_active_.store(true);
  auto handle = sendGoal(goal(false, true, "task"));
  ASSERT_TRUE(handle);
  auto result = client_->async_get_result(handle);
  ASSERT_TRUE(waitUntil([this]() {return waiting();}));
  auto executed = trigger(execute_);
  ASSERT_TRUE(executed);
  EXPECT_TRUE(executed->success);
  ASSERT_EQ(result.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  ASSERT_TRUE(result.get().result->success);
  EXPECT_EQ(backend_requests_.load(), 1);
  std::lock_guard<std::mutex> lock(observation_mutex_);
  EXPECT_EQ(endpoint_, ns_ + "/follow_cartesian_trajectory");
  EXPECT_EQ(dispatched_.joint_names, (std::vector<std::string>{"right_joint"}));
  EXPECT_GT(dispatched_.header.stamp.sec, 0);
  EXPECT_EQ(coordinated_goal_.base_trajectory.header.stamp, dispatched_.header.stamp);
  EXPECT_EQ(coordinated_goal_.available_dofs, (std::vector<bool>{false, false, false, false, false, false, true}));
  EXPECT_EQ(coordinated_goal_.arm, "B");
  EXPECT_EQ(coordinated_goal_.tip_frame, "right_tip");
  EXPECT_EQ(coordinated_goal_.target_pose.header.frame_id, "base");
  EXPECT_EQ(coordinated_goal_.base_trajectory.header.frame_id, "base");
  ASSERT_EQ(coordinated_goal_.base_trajectory.points.size(), dispatched_.points.size());
  for (std::size_t i = 0; i < dispatched_.points.size(); ++i) {
    EXPECT_EQ(coordinated_goal_.base_trajectory.points[i].time_from_start, dispatched_.points[i].time_from_start);
    ASSERT_EQ(coordinated_goal_.base_trajectory.points[i].transforms.size(), 1u);
    EXPECT_DOUBLE_EQ(coordinated_goal_.base_trajectory.points[i].transforms[0].translation.x, 0.0);
  }
  EXPECT_EQ(switch_requests_.load(), 0);
}

TEST_F(SingleCartesianActionServerTest, DisabledTaskRetainsPlanUntilManualRetry)
{
  backend_active_.store(true);
  task_enabled_.store(false);
  auto handle = sendGoal(goal(true, true, "task"));
  ASSERT_TRUE(handle);
  auto result = client_->async_get_result(handle);
  ASSERT_TRUE(waitUntil([this]() {return waiting("plan retained");}));
  EXPECT_EQ(backend_requests_.load(), 1);
  task_enabled_.store(true);
  EXPECT_EQ(result.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);
  EXPECT_EQ(backend_requests_.load(), 1);
  auto retry = trigger(execute_);
  ASSERT_TRUE(retry);
  EXPECT_TRUE(retry->success);
  ASSERT_EQ(result.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  EXPECT_TRUE(result.get().result->success);
  EXPECT_EQ(backend_requests_.load(), 2);
}

TEST_F(SingleCartesianActionServerTest, RejectsUnknownBackendAndObsoleteStart)
{
  auto invalid = client_->async_send_goal(goal(false, true, "unknown"));
  ASSERT_EQ(invalid.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_FALSE(invalid.get());
  backend_active_.store(true);
  auto handle = sendGoal(goal(false, true, "controller"));
  ASSERT_TRUE(handle);
  auto result = client_->async_get_result(handle);
  ASSERT_TRUE(waitUntil([this]() {return waiting();}));
  right_position_.store(0.02);
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  ASSERT_TRUE(trigger(execute_));
  ASSERT_EQ(result.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  const auto finished = result.get();
  EXPECT_FALSE(finished.result->success);
  EXPECT_NE(finished.result->message.find("Plan obsolete"), std::string::npos);
  EXPECT_EQ(backend_requests_.load(), 0);
}

TEST_F(SingleCartesianActionServerTest, JointOrderMetadataIsReadableAndImmutable)
{
  auto parameters = fake_->create_client<rcl_interfaces::srv::GetParameters>(
    "/" + server_name_ + "/get_parameters");
  ASSERT_TRUE(parameters->wait_for_service(std::chrono::seconds(1)));
  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names = {"arms.A.joint_names", "arms.B.joint_names"};
  auto future = parameters->async_send_request(request);
  ASSERT_EQ(future.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  const auto response = future.get();
  ASSERT_EQ(response->values.size(), 2u);
  EXPECT_EQ(response->values[0].string_array_value, (std::vector<std::string>{"left_joint"}));
  EXPECT_EQ(response->values[1].string_array_value, (std::vector<std::string>{"right_joint"}));
  EXPECT_FALSE(server_->set_parameter(rclcpp::Parameter(
    "arms.A.joint_names", std::vector<std::string>{"wrong_joint"})).successful);
}

TEST_F(SingleCartesianActionServerTest, RejectsWrongMaskLengthsAndUnavailableBase)
{
  for (const std::size_t count : {1u, 6u, 8u}) {
    auto request = goal();
    request.available_dofs.assign(count, false);
    auto sent = client_->async_send_goal(request);
    ASSERT_EQ(sent.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    EXPECT_FALSE(sent.get()) << "Accepted a mask with " << count << " entries";
  }
  for (const auto & backend : {"controller", "task", ""}) {
    auto request = goal(false, false, backend);
    request.available_dofs = {true, false, false, false, false, false, true};
    auto sent = client_->async_send_goal(request);
    ASSERT_EQ(sent.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    EXPECT_FALSE(sent.get()) << "Fixed-root model accepted base planning with " << backend;
  }
  EXPECT_EQ(backend_requests_.load(), 0);
}

TEST_F(SingleCartesianActionServerTest, LockedArmCannotReachDifferentTarget)
{
  auto request = goal();
  request.available_dofs = {false, false, false, false, false, false, false};
  auto handle = sendGoal(request);
  ASSERT_TRUE(handle);
  auto result = client_->async_get_result(handle);
  ASSERT_EQ(result.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  const auto finished = result.get();
  ASSERT_TRUE(finished.result);
  EXPECT_EQ(finished.code, rclcpp_action::ResultCode::ABORTED);
  EXPECT_FALSE(finished.result->success);
  EXPECT_EQ(backend_requests_.load(), 0);
}

TEST_F(SingleCartesianActionServerTest, RejectsInvalidBaseBoundsAndDynamicLimits)
{
  std::vector<PlanAction::Goal> invalid;
  invalid.push_back(goal()); invalid.back().base_limits.lower_limits = std::vector<double>(6, -1.0);
  invalid.push_back(goal()); invalid.back().base_limits.lower_limits = {-1.0};
  invalid.back().base_limits.upper_limits = {1.0};
  invalid.push_back(goal()); invalid.back().base_limits.lower_limits = std::vector<double>(6, -1.0);
  invalid.back().base_limits.upper_limits = std::vector<double>(6, 1.0);
  invalid.back().base_limits.lower_limits[2] = 2.0;
  invalid.push_back(goal()); invalid.back().base_limits.lower_limits = std::vector<double>(6, -1.0);
  invalid.back().base_limits.upper_limits = std::vector<double>(6, 1.0);
  invalid.back().base_limits.upper_limits[4] = std::numeric_limits<double>::infinity();
  invalid.push_back(goal()); invalid.back().base_limits.max_velocity = {0.2};
  invalid.push_back(goal()); invalid.back().base_limits.max_velocity = std::vector<double>(6, 0.2);
  invalid.back().base_limits.max_velocity[0] = 0.0;
  invalid.push_back(goal()); invalid.back().base_limits.max_acceleration = std::vector<double>(6, 0.2);
  invalid.back().base_limits.max_acceleration[3] = std::numeric_limits<double>::quiet_NaN();
  for (const auto & request : invalid) {
    auto sent = client_->async_send_goal(request);
    ASSERT_EQ(sent.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    EXPECT_FALSE(sent.get());
  }
  EXPECT_EQ(backend_requests_.load(), 0);
}

TEST_F(SingleCartesianActionServerTest, RejectsInvalidTrackingToleranceDimensionsAndValues)
{
  std::vector<PlanAction::Goal> invalid;
  invalid.push_back(goal()); invalid.back().tracking_tolerances.base_path = {0.05};
  invalid.push_back(goal()); invalid.back().tracking_tolerances.joint_goal = {0.05, 0.05};
  invalid.push_back(goal()); invalid.back().tracking_tolerances.tip_goal = std::vector<double>(6, 0.03);
  invalid.back().tracking_tolerances.tip_goal[2] = -0.01;
  invalid.push_back(goal()); invalid.back().tracking_tolerances.base_goal = std::vector<double>(6, 0.05);
  invalid.back().tracking_tolerances.base_goal[0] = std::numeric_limits<double>::quiet_NaN();
  invalid.push_back(goal()); invalid.back().tracking_tolerances.goal_time_tolerance.sec = -1;
  invalid.push_back(goal()); invalid.back().tracking_tolerances.goal_time_tolerance.nanosec = 1000000000U;
  for (const auto & request : invalid) {
    auto sent = client_->async_send_goal(request);
    ASSERT_EQ(sent.wait_for(std::chrono::seconds(2)), std::future_status::ready);
    EXPECT_FALSE(sent.get());
  }
}

TEST_F(SingleCartesianActionServerTest, MissingTargetTfAbortsAcceptedGoalWithoutDispatch)
{
  auto request = goal();
  request.target_pose.header.frame_id = "missing_target_frame_" + suffix_;
  auto handle = sendGoal(request);
  ASSERT_TRUE(handle);  // Frames are resolved asynchronously after goal admission.
  auto result = client_->async_get_result(handle);
  ASSERT_EQ(result.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  const auto finished = result.get();
  EXPECT_EQ(finished.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(finished.result);
  EXPECT_FALSE(finished.result->success);
  EXPECT_NE(finished.result->message.find("frames"), std::string::npos);
  EXPECT_EQ(backend_requests_.load(), 0);
}

TEST_F(SingleCartesianActionServerTest, ArbitraryTargetFrameIsTransformedAndFrozenBeforeExecution)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.frame_id = "base";
  transform.child_frame_id = "target_reference_" + suffix_;
  transform.transform.translation.x = 1.0;
  transform.transform.translation.y = 2.0;
  transform.transform.rotation.z = std::sqrt(0.5);
  transform.transform.rotation.w = std::sqrt(0.5);
  tf_broadcaster_->sendTransform(transform);
  // A local TF listener confirms that the retained transform has been delivered.
  tf2_ros::Buffer buffer(fake_->get_clock());
  tf2_ros::TransformListener listener(buffer, fake_, false);
  ASSERT_TRUE(waitUntil([&]() {
    return buffer.canTransform("base", transform.child_frame_id, tf2::TimePointZero);
  }));
  auto request = goal(false, false, "task");
  request.target_pose.header.frame_id = transform.child_frame_id;
  request.target_pose.pose.position.x = -2.0;
  request.target_pose.pose.position.y = 1.99;
  request.target_pose.pose.orientation.z = -std::sqrt(0.5);
  request.target_pose.pose.orientation.w = std::sqrt(0.5);
  auto handle = sendGoal(request);
  ASSERT_TRUE(handle);
  auto result = client_->async_get_result(handle);
  ASSERT_TRUE(waitUntil([this]() {return waiting();}));
  transform.transform.translation.x = 3.0;
  tf_broadcaster_->sendTransform(transform);
  backend_active_.store(true);
  ASSERT_TRUE(trigger(execute_));
  ASSERT_EQ(result.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  const auto finished = result.get();
  ASSERT_TRUE(finished.result);
  EXPECT_TRUE(finished.result->success) << finished.result->message;
  ASSERT_FALSE(finished.result->path.poses.empty());
  EXPECT_EQ(finished.result->path.header.frame_id, "base");
  EXPECT_NEAR(finished.result->path.poses.back().pose.position.x, -0.99, 1e-6);
  std::lock_guard<std::mutex> lock(observation_mutex_);
  EXPECT_NEAR(coordinated_goal_.target_pose.pose.position.x, -0.99, 1e-12);
  EXPECT_NEAR(coordinated_goal_.target_pose.pose.position.y, 0.0, 1e-12);
  EXPECT_NEAR(coordinated_goal_.target_pose.pose.orientation.w, 1.0, 1e-12);
  EXPECT_NEAR(coordinated_goal_.tolerance_reference_pose.pose.orientation.z, std::sqrt(0.5), 1e-12);
}

TEST_F(SingleCartesianActionServerTest, CoordinatedExecutionForwardsToleranceAndLimitOverrides)
{
  backend_active_.store(true);
  auto request = goal(true, false, "task");
  request.base_limits.max_velocity = std::vector<double>(6, 0.12);
  request.base_limits.max_acceleration = std::vector<double>(6, 0.18);
  request.tracking_tolerances.base_path = std::vector<double>(6, 0.08);
  request.tracking_tolerances.joint_goal = {0.015};
  request.tracking_tolerances.tip_goal = std::vector<double>(6, 0.02);
  request.tracking_tolerances.goal_time_tolerance.sec = 1;
  request.tracking_tolerances.goal_time_tolerance.nanosec = 250000000U;
  auto handle = sendGoal(request);
  ASSERT_TRUE(handle);
  auto result = client_->async_get_result(handle);
  ASSERT_EQ(result.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  ASSERT_TRUE(result.get().result->success);
  std::lock_guard<std::mutex> lock(observation_mutex_);
  EXPECT_EQ(coordinated_goal_.base_limits.max_velocity, request.base_limits.max_velocity);
  EXPECT_EQ(coordinated_goal_.base_limits.max_acceleration, request.base_limits.max_acceleration);
  EXPECT_EQ(coordinated_goal_.tracking_tolerances.base_path, request.tracking_tolerances.base_path);
  EXPECT_EQ(coordinated_goal_.tracking_tolerances.joint_goal, request.tracking_tolerances.joint_goal);
  EXPECT_EQ(coordinated_goal_.tracking_tolerances.tip_goal, request.tracking_tolerances.tip_goal);
  EXPECT_EQ(coordinated_goal_.tracking_tolerances.goal_time_tolerance,
    request.tracking_tolerances.goal_time_tolerance);
}

TEST_F(SingleCartesianActionServerTest, CancelingCoordinatedExecutionCancelsBackendAndForwardsErrors)
{
  backend_active_.store(true); hold_coordinated_.store(true);
  auto handle = sendGoal(goal(true, false, "task"));
  ASSERT_TRUE(handle);
  auto result = client_->async_get_result(handle);
  ASSERT_TRUE(waitUntil([this]() {
    std::lock_guard<std::mutex> lock(observation_mutex_); return stage_ == "tracking";
  }));
  {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    EXPECT_DOUBLE_EQ(last_feedback_.base_error[0], 0.01);
    ASSERT_EQ(last_feedback_.joint_error.size(), 1u);
    EXPECT_DOUBLE_EQ(last_feedback_.joint_error[0], 0.02);
    EXPECT_DOUBLE_EQ(last_feedback_.tip_error[1], 0.03);
  }
  auto canceled = client_->async_cancel_goal(handle);
  ASSERT_EQ(canceled.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  ASSERT_EQ(result.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  const auto finished = result.get();
  EXPECT_EQ(finished.code, rclcpp_action::ResultCode::CANCELED);
  EXPECT_FALSE(finished.result->success);
  EXPECT_TRUE(waitUntil([this]() {return backend_cancels_.load() > 0;}));
  EXPECT_EQ(switch_requests_.load(), 0);
}

TEST_F(SingleCartesianActionServerTest, ControllerSuccessRequiresMeasuredToolTracking)
{
  backend_active_.store(true); controller_tracks_.store(false);
  auto request = goal(true, false, "controller");
  request.velocity_scaling = request.acceleration_scaling = 0.5;
  auto handle = sendGoal(request);
  ASSERT_TRUE(handle);
  auto result = client_->async_get_result(handle);
  ASSERT_EQ(result.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  const auto finished = result.get();
  ASSERT_TRUE(finished.result);
  EXPECT_FALSE(finished.result->success);
  EXPECT_NE(finished.result->message.find("tolerance"), std::string::npos);
}

}  // namespace
}  // namespace sura_manipulation_actions

