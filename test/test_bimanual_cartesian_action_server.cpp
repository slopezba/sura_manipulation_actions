#include "sura_manipulation_actions/plan_bimanual_cartesian_trajectory_action_server.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <limits>
#include <mutex>
#include <set>
#include <thread>

#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "moveit_msgs/msg/display_trajectory.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace sura_manipulation_actions
{
namespace
{
using PlanAction = sura_manipulation_actions::action::PlanBimanualCartesianTrajectory;
using PlanHandle = rclcpp_action::ClientGoalHandle<PlanAction>;
using FollowAction = control_msgs::action::FollowJointTrajectory;
using FollowHandle = rclcpp_action::ServerGoalHandle<FollowAction>;
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

class CartesianActionServerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    static unsigned sequence = 0;
    suffix_ = std::to_string(++sequence);
    ns_ = "/cartesian_action_fixture_" + suffix_;
    server_name_ = "cartesian_action_server_" + suffix_;
    fake_ = std::make_shared<rclcpp::Node>("cartesian_action_clients_" + suffix_);
    auto node_options = rclcpp::NodeOptions();
    node_options.arguments({"--ros-args", "-r", "__node:=" + server_name_});
    node_options.parameter_overrides({
      rclcpp::Parameter("robot_namespace", ns_),
      rclcpp::Parameter("robot_description", std::string(kUrdf)),
      rclcpp::Parameter("robot_description_semantic", std::string(kSrdf)),
      rclcpp::Parameter("joint_states_topic", ns_ + "/joint_states"),
      rclcpp::Parameter("execution_backend", "controller"),
      rclcpp::Parameter("arm_names", std::vector<std::string>{"A", "B"}),
      rclcpp::Parameter("arms.A.group", "left"),
      rclcpp::Parameter("arms.A.tip_frame", "left_tip"),
      rclcpp::Parameter("arms.A.controller_action", ns_ + "/left/follow_joint_trajectory"),
      rclcpp::Parameter("arms.A.controller_name", "left_controller"),
      rclcpp::Parameter("arms.A.manager_service", ns_ + "/controller_manager/list_controllers"),
      rclcpp::Parameter("arms.B.group", "right"),
      rclcpp::Parameter("arms.B.tip_frame", "right_tip"),
      rclcpp::Parameter("arms.B.controller_action", ns_ + "/right/follow_joint_trajectory"),
      rclcpp::Parameter("arms.B.controller_name", "right_controller"),
      rclcpp::Parameter("arms.B.manager_service", ns_ + "/controller_manager/list_controllers"),
      rclcpp::Parameter("pair_names", std::vector<std::string>{"AB"}),
      rclcpp::Parameter("pairs.AB.arms", std::vector<std::string>{"A", "B"}),
      rclcpp::Parameter("pairs.AB.group", "both"),
      rclcpp::Parameter("visualization.fixed_frame", "base"),
      rclcpp::Parameter("relative_reference_frame", "base"),
      rclcpp::Parameter("planning.default_planning_time", 3.0),
      rclcpp::Parameter("planning.optimize_trajectory", false),
      // Untyped empty YAML arrays may arrive as another ROS array type.
      rclcpp::Parameter("planning.limit_joint_names", std::vector<uint8_t>{}),
      rclcpp::Parameter("planning.lower_limits", std::vector<uint8_t>{}),
      rclcpp::Parameter("planning.upper_limits", std::vector<uint8_t>{}),
      rclcpp::Parameter("execution.controller_wait_timeout", 0.5),
      rclcpp::Parameter("execution.acceptance_timeout", 0.5),
      rclcpp::Parameter("execution.start_delay", 1.0),
      rclcpp::Parameter("robot_description_planning.joint_limits.left_joint.has_acceleration_limits", true),
      rclcpp::Parameter("robot_description_planning.joint_limits.left_joint.max_acceleration", 1.0),
      rclcpp::Parameter("robot_description_planning.joint_limits.right_joint.has_acceleration_limits", true),
      rclcpp::Parameter("robot_description_planning.joint_limits.right_joint.max_acceleration", 1.0),
    });
    server_ = std::make_shared<PlanBimanualCartesianTrajectoryActionServer>(node_options);
    joint_pub_ = fake_->create_publisher<sensor_msgs::msg::JointState>(ns_ + "/joint_states", rclcpp::SensorDataQoS());
    state_timer_ = fake_->create_wall_timer(std::chrono::milliseconds(20), [this]() {
      sensor_msgs::msg::JointState state;
      state.header.stamp = fake_->now();
      state.name = {"left_joint", "right_joint"};
      state.position = {0.0, right_position_.load()};
      joint_pub_->publish(state);
    });
    list_service_ = fake_->create_service<controller_manager_msgs::srv::ListControllers>(
      ns_ + "/controller_manager/list_controllers",
      [this](std::shared_ptr<controller_manager_msgs::srv::ListControllers::Request>,
      std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response> response) {
        for (const auto & name : {"left_controller", "right_controller"}) {
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
      backends_.push_back(rclcpp_action::create_server<FollowAction>(
        fake_, ns_ + "/" + arm + "/follow_joint_trajectory",
        [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowAction::Goal>) {
          backend_requests_.fetch_add(1);
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](std::shared_ptr<FollowHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
        [](std::shared_ptr<FollowHandle> handle) {
          auto result = std::make_shared<FollowAction::Result>();
          result->error_code = FollowAction::Result::SUCCESSFUL;
          handle->succeed(result);
        }));
    }
    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    trajectory_sub_ = fake_->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      ns_ + "/bimanual_planner/joint_trajectory", qos,
      [this](trajectory_msgs::msg::JointTrajectory::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(observation_mutex_); published_ = *msg;
      });
    relative_sub_ = fake_->create_subscription<nav_msgs::msg::Path>(
      ns_ + "/bimanual_planner/relative_path", qos,
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
    client_ = rclcpp_action::create_client<PlanAction>(fake_, ns_ + "/manipulation/plan_bimanual_cartesian_trajectory");
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

  PlanAction::Goal goal(bool execute = false, bool swap = false) const
  {
    PlanAction::Goal result;
    result.arm_a = swap ? "B" : "A";
    result.arm_b = swap ? "A" : "B";
    result.relative_target.header.frame_id = "base";
    result.relative_target.pose.position.x = swap ? -1.99 : 2.01;
    result.relative_target.pose.orientation.w = 1.0;
    result.relative_position_tolerance.x = 0.001;
    result.cartesian_step = 0.005;
    result.planning_time = 3.0;
    result.execute = execute;
    return result;
  }

  PlanHandle::SharedPtr sendGoal(const PlanAction::Goal & goal)
  {
    rclcpp_action::Client<PlanAction>::SendGoalOptions options;
    options.feedback_callback = [this](PlanHandle::SharedPtr, std::shared_ptr<const PlanAction::Feedback> msg) {
        std::lock_guard<std::mutex> lock(observation_mutex_);
        stage_ = msg->current_stage; message_ = msg->message;
      };
    auto sent = client_->async_send_goal(goal, options);
    if (sent.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {return {};}
    return sent.get();
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

  std::string suffix_, ns_, server_name_;
  std::shared_ptr<PlanBimanualCartesianTrajectoryActionServer> server_;
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
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr relative_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr markers_sub_;
  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_sub_;
  std::atomic_bool backend_active_{false};
  std::atomic_int backend_requests_{0}, switch_requests_{0};
  std::atomic_int visual_refreshes_{0};
  std::atomic<double> right_position_{0.0};
  mutable std::mutex observation_mutex_;
  std::string stage_, message_;
  trajectory_msgs::msg::JointTrajectory published_;
  nav_msgs::msg::Path relative_view_;
  visualization_msgs::msg::MarkerArray markers_;
  moveit_msgs::msg::DisplayTrajectory display_;
};

TEST_F(CartesianActionServerTest, PlanOnlyPublishesTheRobotTrajectoryAndKeepsGoalPending)
{
  auto handle = sendGoal(goal());
  ASSERT_TRUE(handle);
  auto result = client_->async_get_result(handle);
  ASSERT_TRUE(waitUntil([this]() {return waiting();}));
  EXPECT_EQ(result.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
  ASSERT_TRUE(waitUntil([this]() {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    return !published_.points.empty() && !display_.trajectory.empty() &&
      !markers_.markers.empty() && visual_refreshes_.load() >= 2;
  }));
  {
    std::lock_guard<std::mutex> lock(observation_mutex_);
    EXPECT_EQ(published_.joint_names.size(), 2u);
    EXPECT_EQ(published_.points.front().positions, (std::vector<double>{0.0, 0.0}));
    EXPECT_EQ(display_.trajectory.front().joint_trajectory.joint_names, published_.joint_names);
    EXPECT_EQ(display_.trajectory.front().joint_trajectory.points.size(), published_.points.size());
    EXPECT_EQ(relative_view_.header.frame_id, "base");
    EXPECT_EQ(relative_view_.header.stamp.sec, 0);
    EXPECT_EQ(relative_view_.header.stamp.nanosec, 0u);
    ASSERT_EQ(markers_.markers.size(), 13u);  // Four paths and three triads.
    std::set<std::pair<std::string, int>> marker_keys;
    for (const auto & marker : markers_.markers) {
      EXPECT_TRUE(marker_keys.emplace(marker.ns, marker.id).second)
        << "Duplicate marker in the same message: " << marker.ns << "/" << marker.id;
      EXPECT_EQ(marker.header.frame_id, "base");
      EXPECT_TRUE(marker.frame_locked);
      EXPECT_NE(marker.action, visualization_msgs::msg::Marker::DELETEALL);
      EXPECT_EQ(marker.ns.find("bimanual_cartesian/"), 0u);
      if (marker.action == visualization_msgs::msg::Marker::ADD &&
        marker.type == visualization_msgs::msg::Marker::ARROW)
      {
        ASSERT_EQ(marker.points.size(), 2u);
        EXPECT_FLOAT_EQ(marker.color.r, marker.id == 0 ? 1.0F : 0.0F);
        EXPECT_FLOAT_EQ(marker.color.g, marker.id == 1 ? 1.0F : 0.0F);
        EXPECT_FLOAT_EQ(marker.color.b, marker.id == 2 ? 1.0F : 0.0F);
      }
    }
    EXPECT_EQ(std::count_if(markers_.markers.begin(), markers_.markers.end(),
      [](const auto & marker) {return marker.action == visualization_msgs::msg::Marker::ADD &&
        marker.type == visualization_msgs::msg::Marker::ARROW;}), 9);
    EXPECT_EQ(std::count_if(markers_.markers.begin(), markers_.markers.end(),
      [](const auto & marker) {return marker.action == visualization_msgs::msg::Marker::ADD &&
        marker.type == visualization_msgs::msg::Marker::LINE_STRIP;}), 4);
    for (const auto & path : markers_.markers) {
      EXPECT_NE(path.action, visualization_msgs::msg::Marker::DELETEALL);
      EXPECT_EQ(path.ns.find("bimanual_cartesian/"), 0u);
      EXPECT_TRUE(path.frame_locked);
      if (path.action != visualization_msgs::msg::Marker::ADD ||
        path.type != visualization_msgs::msg::Marker::LINE_STRIP) {continue;}
      EXPECT_EQ(path.type, visualization_msgs::msg::Marker::LINE_STRIP);
      if (path.ns == "bimanual_cartesian/planned_path") {
        ASSERT_EQ(path.points.size(), relative_view_.poses.size());
        for (std::size_t i = 0; i < path.points.size(); ++i) {
          EXPECT_EQ(path.points[i], relative_view_.poses[i].pose.position);
        }
        EXPECT_FLOAT_EQ(path.color.g, 1.0F);
      }
      if (path.ns == "bimanual_cartesian/desired_path") {
        EXPECT_FLOAT_EQ(path.color.r, 0.7F);
        EXPECT_FLOAT_EQ(path.color.b, 1.0F);
        EXPECT_DOUBLE_EQ(path.scale.x, 0.006);
      }
    }
  }
  for (const auto & publisher : fake_->get_publishers_info_by_topic("/tf_static")) {
    EXPECT_FALSE(publisher.node_name() == "bimanual_cartesian_moveit_context" &&
      publisher.node_namespace() == ns_ + "/moveit");
  }
  EXPECT_EQ(backend_requests_.load(), 0);
  EXPECT_EQ(switch_requests_.load(), 0);
  const auto canceled = trigger(cancel_);
  ASSERT_TRUE(canceled);
  EXPECT_TRUE(canceled->success);
  ASSERT_EQ(result.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  const auto finished = result.get();
  EXPECT_EQ(finished.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(finished.result);
  EXPECT_FALSE(finished.result->success);
  EXPECT_FALSE(finished.result->arm_a_path.poses.empty());
  EXPECT_EQ(finished.result->relative_path.header.frame_id,
    finished.result->desired_relative_path.header.frame_id);
  EXPECT_EQ(finished.result->relative_path.header.frame_id, "base");
  EXPECT_EQ(finished.result->arm_a_path.header.frame_id, "base");
  EXPECT_EQ(finished.result->arm_b_path.header.frame_id, "base");
  ASSERT_FALSE(finished.result->relative_path.poses.empty());
  // The drawing offset is the initial arm_a at -1; the requested delta is 2.01.
  EXPECT_NEAR(finished.result->relative_path.poses.back().pose.position.x, 1.01,
    goal().relative_position_tolerance.x + 1e-6);
  EXPECT_NEAR(finished.result->relative_path.poses.back().pose.position.x,
    -1.0 + finished.result->arm_b_path.poses.back().pose.position.x -
    finished.result->arm_a_path.poses.back().pose.position.x, 1e-9);
  for (const auto & pose : finished.result->relative_path.poses) {
    EXPECT_EQ(pose.header.frame_id, "base");
    EXPECT_EQ(pose.header.stamp.sec, 0);
    EXPECT_EQ(pose.header.stamp.nanosec, 0u);
  }
}

TEST_F(CartesianActionServerTest, RejectsToolFrameAndAcceptsTheConfiguredBaseFrame)
{
  auto wrong = goal();
  wrong.relative_target.header.frame_id = "left_tip";
  auto rejected = client_->async_send_goal(wrong);
  ASSERT_EQ(rejected.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_FALSE(rejected.get());
  auto accepted = sendGoal(goal());
  ASSERT_TRUE(accepted);
  ASSERT_TRUE(waitUntil([this]() {return waiting();}));
  EXPECT_EQ(backend_requests_.load(), 0);
}

TEST_F(CartesianActionServerTest, ConfigurationRejectsWrongLengthAndInvalidWeights)
{
  ASSERT_EQ(server_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(static_cast<rclcpp_lifecycle::LifecycleNode &>(*server_).cleanup().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  const std::vector<std::vector<double>> invalid_weights{
    {1.0}, {0.0, 0.0, 0.0}, {-1.0, 0.0},
    {std::numeric_limits<double>::quiet_NaN(), 0.0},
    {0.0, std::numeric_limits<double>::infinity()}};
  for (const auto & weights : invalid_weights) {
    ASSERT_TRUE(server_->set_parameter(rclcpp::Parameter("planning.weights", weights)).successful);
    EXPECT_EQ(server_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  }
  ASSERT_TRUE(server_->set_parameter(
      rclcpp::Parameter("planning.weights", std::vector<double>{0.0, 0.0})).successful);
  EXPECT_EQ(server_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
}

TEST_F(CartesianActionServerTest, ConfiguredWeightsRemainOnPhysicalJointsWhenArmsAreSwapped)
{
  ASSERT_EQ(server_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(static_cast<rclcpp_lifecycle::LifecycleNode &>(*server_).cleanup().id(),
    lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED);
  ASSERT_TRUE(server_->set_parameter(
      rclcpp::Parameter("planning.weights", std::vector<double>{100.0, 0.0})).successful);
  ASSERT_EQ(server_->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  ASSERT_EQ(server_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  std::vector<double> original_final;
  for (const bool swap : {false, true}) {
    {std::lock_guard<std::mutex> lock(observation_mutex_); stage_.clear();}
    auto target = goal(false, swap);
    target.relative_target.pose.position.x = swap ? -2.01 : 2.01;
    target.relative_position_tolerance.x = 0.0;
    auto handle = sendGoal(target);
    ASSERT_TRUE(handle);
    auto result = client_->async_get_result(handle);
    ASSERT_TRUE(waitUntil([this]() {return waiting();}));
    if (!swap) {
      // Parameter changes apply on the next configure, not to this pending plan.
      ASSERT_TRUE(server_->set_parameter(
          rclcpp::Parameter("planning.weights", std::vector<double>{0.0, 100.0})).successful);
      backend_active_.store(true);
      const auto executed = trigger(execute_);
      ASSERT_TRUE(executed);
      ASSERT_TRUE(executed->success) << executed->message;
    } else {
      const auto canceled = trigger(cancel_);
      ASSERT_TRUE(canceled);
      ASSERT_TRUE(canceled->success);
    }
    ASSERT_EQ(result.wait_for(std::chrono::seconds(5)), std::future_status::ready);
    const auto finished = result.get();
    ASSERT_TRUE(finished.result);
    ASSERT_FALSE(finished.result->joint_trajectory.points.empty());
    const auto & trajectory = finished.result->joint_trajectory;
    ASSERT_EQ(trajectory.joint_names, (std::vector<std::string>{"left_joint", "right_joint"}));
    const auto & final = trajectory.points.back().positions;
    ASSERT_EQ(final.size(), 2u);
    EXPECT_LT(std::abs(final[0]), 0.25 * std::abs(final[1]));
    EXPECT_NEAR(final[1] - final[0], 0.01, 1e-6);
    if (!swap) {original_final = final;}
    else {
      EXPECT_NEAR(final[0], original_final[0], 1e-6);
      EXPECT_NEAR(final[1], original_final[1], 1e-6);
    }
  }
}

TEST_F(CartesianActionServerTest, InactiveBackendRetainsPlanAndManualActivationRequiresNewExecuteTrigger)
{
  auto handle = sendGoal(goal(true));
  ASSERT_TRUE(handle);
  auto result = client_->async_get_result(handle);
  ASSERT_TRUE(waitUntil([this]() {return waiting("activate controller");}));
  EXPECT_EQ(backend_requests_.load(), 0);
  EXPECT_EQ(switch_requests_.load(), 0);
  backend_active_.store(true);
  EXPECT_EQ(result.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);
  EXPECT_EQ(backend_requests_.load(), 0);
  const auto executed = trigger(execute_);
  ASSERT_TRUE(executed);
  EXPECT_TRUE(executed->success) << executed->message;
  ASSERT_EQ(result.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  const auto finished = result.get();
  EXPECT_EQ(finished.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(finished.result);
  EXPECT_TRUE(finished.result->success) << finished.result->message;
  EXPECT_EQ(backend_requests_.load(), 2);
  EXPECT_EQ(switch_requests_.load(), 0);
}

TEST_F(CartesianActionServerTest, RejectsInvalidVelocityAndAccelerationFactors)
{
  for (double value : {-0.01, 1.01, std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::infinity()})
  {
    for (bool acceleration : {false, true}) {
      auto invalid = goal();
      if (acceleration) {invalid.acceleration_scaling = value;}
      else {invalid.velocity_scaling = value;}
      auto rejected = client_->async_send_goal(invalid);
      ASSERT_EQ(rejected.wait_for(std::chrono::seconds(2)), std::future_status::ready);
      EXPECT_FALSE(rejected.get());
    }
  }
  auto defaults = sendGoal(goal());
  ASSERT_TRUE(defaults);
  ASSERT_TRUE(waitUntil([this]() {return waiting();}));
}

TEST_F(CartesianActionServerTest, PerGoalTimingFactorsChangeDurationAndSurvivePendingExecution)
{
  double slow_duration = 0.0;
  auto slow = goal();
  slow.velocity_scaling = slow.acceleration_scaling = 0.05;
  auto slow_handle = sendGoal(slow);
  ASSERT_TRUE(slow_handle);
  auto slow_result = client_->async_get_result(slow_handle);
  ASSERT_TRUE(waitUntil([this]() {return waiting();}));
  ASSERT_TRUE(trigger(cancel_));
  ASSERT_EQ(slow_result.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  const auto slow_finished = slow_result.get();
  ASSERT_TRUE(slow_finished.result);
  ASSERT_FALSE(slow_finished.result->joint_trajectory.points.empty());
  const auto & slow_end = slow_finished.result->joint_trajectory.points.back().time_from_start;
  slow_duration = slow_end.sec + slow_end.nanosec * 1e-9;

  // Above the configured 0.1 defaults: execution revalidation must retain 0.5.
  backend_active_.store(true);
  {std::lock_guard<std::mutex> lock(observation_mutex_); stage_.clear();}
  auto fast = goal();
  fast.velocity_scaling = fast.acceleration_scaling = 0.5;
  auto fast_handle = sendGoal(fast);
  ASSERT_TRUE(fast_handle);
  auto fast_result = client_->async_get_result(fast_handle);
  ASSERT_TRUE(waitUntil([this]() {return waiting();}));
  const auto executed = trigger(execute_);
  ASSERT_TRUE(executed);
  EXPECT_TRUE(executed->success) << executed->message;
  ASSERT_EQ(fast_result.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  const auto fast_finished = fast_result.get();
  ASSERT_TRUE(fast_finished.result);
  EXPECT_TRUE(fast_finished.result->success) << fast_finished.result->message;
  ASSERT_FALSE(fast_finished.result->joint_trajectory.points.empty());
  const auto & fast_end = fast_finished.result->joint_trajectory.points.back().time_from_start;
  EXPECT_LT(fast_end.sec + fast_end.nanosec * 1e-9, slow_duration);
  EXPECT_EQ(backend_requests_.load(), 2);
}

TEST_F(CartesianActionServerTest, CancelingPendingActionFinishesCanceledWithoutBackendRequests)
{
  auto handle = sendGoal(goal(false, true));
  ASSERT_TRUE(handle);
  auto result = client_->async_get_result(handle);
  ASSERT_TRUE(waitUntil([this]() {return waiting();}));
  auto cancellation = client_->async_cancel_goal(handle);
  ASSERT_EQ(cancellation.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  EXPECT_FALSE(cancellation.get()->goals_canceling.empty());
  ASSERT_EQ(result.wait_for(std::chrono::seconds(2)), std::future_status::ready);
  const auto finished = result.get();
  EXPECT_EQ(finished.code, rclcpp_action::ResultCode::CANCELED);
  ASSERT_TRUE(finished.result);
  EXPECT_FALSE(finished.result->success);
  EXPECT_EQ(backend_requests_.load(), 0);
  EXPECT_EQ(switch_requests_.load(), 0);
}

TEST_F(CartesianActionServerTest, ChangedInitialStateInvalidatesPendingPlanBeforeDispatch)
{
  backend_active_.store(true);
  auto handle = sendGoal(goal());
  ASSERT_TRUE(handle);
  auto result = client_->async_get_result(handle);
  ASSERT_TRUE(waitUntil([this]() {return waiting();}));
  right_position_.store(0.02);
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  const auto executed = trigger(execute_);
  ASSERT_TRUE(executed);
  ASSERT_EQ(result.wait_for(std::chrono::seconds(3)), std::future_status::ready);
  const auto finished = result.get();
  EXPECT_EQ(finished.code, rclcpp_action::ResultCode::ABORTED);
  ASSERT_TRUE(finished.result);
  EXPECT_NE(finished.result->message.find("Plan obsolete"), std::string::npos);
  EXPECT_EQ(backend_requests_.load(), 0);
  EXPECT_EQ(switch_requests_.load(), 0);
}

TEST_F(CartesianActionServerTest, InactiveLifecycleKeepsNewPlanPendingUntilActivationAndExplicitTrigger)
{
  backend_active_.store(true);
  ASSERT_EQ(server_->deactivate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);
  auto handle = sendGoal(goal(true));
  ASSERT_TRUE(handle);
  auto result = client_->async_get_result(handle);
  ASSERT_TRUE(waitUntil([this]() {return waiting("planner lifecycle");}));
  EXPECT_EQ(backend_requests_.load(), 0);
  EXPECT_EQ(switch_requests_.load(), 0);
  const auto rejected = trigger(execute_);
  ASSERT_TRUE(rejected);
  EXPECT_FALSE(rejected->success);
  EXPECT_NE(rejected->message.find("lifecycle"), std::string::npos);
  EXPECT_EQ(result.wait_for(std::chrono::milliseconds(50)), std::future_status::timeout);
  ASSERT_EQ(server_->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
  EXPECT_EQ(result.wait_for(std::chrono::milliseconds(100)), std::future_status::timeout);
  EXPECT_EQ(backend_requests_.load(), 0);
  const auto executed = trigger(execute_);
  ASSERT_TRUE(executed);
  EXPECT_TRUE(executed->success) << executed->message;
  ASSERT_EQ(result.wait_for(std::chrono::seconds(5)), std::future_status::ready);
  const auto finished = result.get();
  EXPECT_EQ(finished.code, rclcpp_action::ResultCode::SUCCEEDED);
  ASSERT_TRUE(finished.result);
  EXPECT_TRUE(finished.result->success) << finished.result->message;
  EXPECT_EQ(backend_requests_.load(), 2);
  EXPECT_EQ(switch_requests_.load(), 0);
}

}  // namespace
}  // namespace sura_manipulation_actions
