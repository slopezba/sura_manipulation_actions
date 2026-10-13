#include "sura_manipulation_actions/plan_cartesian_trajectory_action_server.hpp"

#include <gtest/gtest.h>

#include <algorithm>
#include <atomic>
#include <chrono>
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
using PlanAction = sura_manipulation_actions::action::PlanCartesianTrajectory;
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
      rclcpp::Parameter("arms.A.task_action", ns_ + "/left_task/follow_joint_trajectory"),
      rclcpp::Parameter("arms.A.task_controller_name", "task_priority_controller"),
      rclcpp::Parameter("arms.A.task_manager_service", ns_ + "/controller_manager/list_controllers"),
      rclcpp::Parameter("arms.A.controller_action", ns_ + "/left/follow_joint_trajectory"),
      rclcpp::Parameter("arms.A.controller_name", "left_controller"),
      rclcpp::Parameter("arms.A.manager_service", ns_ + "/controller_manager/list_controllers"),
      rclcpp::Parameter("arms.B.group", "right"),
      rclcpp::Parameter("arms.B.tip_frame", "right_tip"),
      rclcpp::Parameter("arms.B.task_action", ns_ + "/right_task/follow_joint_trajectory"),
      rclcpp::Parameter("arms.B.task_controller_name", "task_priority_controller"),
      rclcpp::Parameter("arms.B.task_manager_service", ns_ + "/controller_manager/list_controllers"),
      rclcpp::Parameter("arms.B.controller_action", ns_ + "/right/follow_joint_trajectory"),
      rclcpp::Parameter("arms.B.controller_name", "right_controller"),
      rclcpp::Parameter("arms.B.manager_service", ns_ + "/controller_manager/list_controllers"),
      rclcpp::Parameter("visualization.fixed_frame", "base"),
      rclcpp::Parameter("reference_frame", "base"),
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
    server_ = std::make_shared<PlanCartesianTrajectoryActionServer>(node_options);
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
        for (const auto & name : {"left_controller", "right_controller", "task_priority_controller"}) {
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
      for (bool task : {false, true}) {
        const auto endpoint = ns_ + "/" + arm + (task ? "_task" : "") + "/follow_joint_trajectory";
        backends_.push_back(rclcpp_action::create_server<FollowAction>(fake_, endpoint,
          [this, endpoint, task](const rclcpp_action::GoalUUID &, std::shared_ptr<const FollowAction::Goal> goal) {
            backend_requests_.fetch_add(1);
            {std::lock_guard<std::mutex> lock(observation_mutex_);
              endpoint_ = endpoint; dispatched_ = goal->trajectory;}
            return task && !task_enabled_.load() ? rclcpp_action::GoalResponse::REJECT :
              rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
          },
          [](std::shared_ptr<FollowHandle>) {return rclcpp_action::CancelResponse::ACCEPT;},
          [](std::shared_ptr<FollowHandle> handle) {
            auto result = std::make_shared<FollowAction::Result>();
            result->error_code = FollowAction::Result::SUCCESSFUL;
            handle->succeed(result);
          }));
      }
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
  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr relative_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr markers_sub_;
  rclcpp::Subscription<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_sub_;
  std::atomic_bool backend_active_{false}, task_enabled_{true};
  std::atomic_int backend_requests_{0}, switch_requests_{0};
  std::atomic_int visual_refreshes_{0};
  std::atomic<double> right_position_{0.0};
  mutable std::mutex observation_mutex_;
  std::string stage_, message_;
  trajectory_msgs::msg::JointTrajectory published_, dispatched_;
  std::string endpoint_;
  nav_msgs::msg::Path relative_view_;
  visualization_msgs::msg::MarkerArray markers_;
  moveit_msgs::msg::DisplayTrajectory display_;
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
  ASSERT_EQ(markers_.markers.size(), 8u);  // Paths and XYZ triads in one message.
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
      marker.type == visualization_msgs::msg::Marker::LINE_STRIP;}), 2);
  for (const auto & path : markers_.markers) {
    EXPECT_NE(path.action, visualization_msgs::msg::Marker::DELETEALL);
    EXPECT_EQ(path.ns.find("cartesian/"), 0u);
    EXPECT_TRUE(path.frame_locked);
    if (path.action != visualization_msgs::msg::Marker::ADD ||
      path.type != visualization_msgs::msg::Marker::LINE_STRIP) {continue;}
    EXPECT_EQ(path.type, visualization_msgs::msg::Marker::LINE_STRIP);
    const auto & poses = path.ns == "cartesian/planned_path" ?
      finished.result->path.poses : finished.result->desired_path.poses;
    ASSERT_EQ(path.points.size(), poses.size());
    for (std::size_t i = 0; i < poses.size(); ++i) {
      EXPECT_EQ(path.points[i], poses[i].pose.position);
    }
    if (path.ns == "cartesian/planned_path") {EXPECT_FLOAT_EQ(path.color.g, 1.0F);}
    else {
      EXPECT_FLOAT_EQ(path.color.r, 0.7F);
      EXPECT_FLOAT_EQ(path.color.b, 1.0F);
      EXPECT_DOUBLE_EQ(path.scale.x, 0.006);
    }
  }
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

TEST_F(SingleCartesianActionServerTest, SelectedTaskReceivesOnlyRightTrajectoryAtStampZero)
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
  EXPECT_EQ(endpoint_, ns_ + "/right_task/follow_joint_trajectory");
  EXPECT_EQ(dispatched_.joint_names, (std::vector<std::string>{"right_joint"}));
  EXPECT_EQ(dispatched_.header.stamp.sec, 0);
  EXPECT_EQ(dispatched_.header.stamp.nanosec, 0u);
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

}  // namespace
}  // namespace sura_manipulation_actions

