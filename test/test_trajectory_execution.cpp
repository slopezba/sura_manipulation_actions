#include "sura_manipulation_actions/trajectory_execution.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <future>
#include <mutex>
#include <thread>
#include <utility>

namespace sura_manipulation_actions
{
namespace
{
using Action = control_msgs::action::FollowJointTrajectory;
using ServerHandle = rclcpp_action::ServerGoalHandle<Action>;

bool waitUntil(const std::function<bool()> & predicate, double timeout = 2.0)
{
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout);
  while (std::chrono::steady_clock::now() < deadline) {
    if (predicate()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return predicate();
}

class FakeTrajectoryServer
{
public:
  explicit FakeTrajectoryServer(const std::string & name)
  : node(std::make_shared<rclcpp::Node>(name)), action_name("/" + name + "/follow_joint_trajectory")
  {
    server_ = rclcpp_action::create_server<Action>(node, action_name,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Action::Goal> goal) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          trajectory_ = goal->trajectory;
        }
        received.store(true);
        std::this_thread::sleep_for(std::chrono::milliseconds(acceptance_delay_ms.load()));
        return reject.load() ? rclcpp_action::GoalResponse::REJECT :
               rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](const std::shared_ptr<ServerHandle>) {
        canceled.fetch_add(1);
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](const std::shared_ptr<ServerHandle> handle) {
        accepted.store(true);
        std::lock_guard<std::mutex> lock(mutex_);
        workers_.emplace_back([this, handle]() {
          while (!stop.load() && !finish.load() && !handle->is_canceling()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
          }
          auto result = std::make_shared<Action::Result>();
          result->error_code = result_error.load();
          if (handle->is_canceling()) {
            handle->canceled(result);
          } else if (stop.load()) {
            handle->abort(result);
          } else {
            handle->succeed(result);
          }
        });
      });
  }

  ~FakeTrajectoryServer()
  {
    stop.store(true);
    for (auto & worker : workers_) {
      if (worker.joinable()) {
        worker.join();
      }
    }
  }

  trajectory_msgs::msg::JointTrajectory trajectory() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return trajectory_;
  }

  rclcpp::Node::SharedPtr node;
  const std::string action_name;
  std::atomic_bool reject{false}, received{false}, accepted{false}, finish{false}, stop{false};
  std::atomic_int acceptance_delay_ms{0}, canceled{0}, result_error{Action::Result::SUCCESSFUL};

private:
  mutable std::mutex mutex_;
  trajectory_msgs::msg::JointTrajectory trajectory_;
  rclcpp_action::Server<Action>::SharedPtr server_;
  std::vector<std::thread> workers_;
};

class TrajectoryTransportTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    static unsigned sequence = 0;
    const auto suffix = std::to_string(++sequence);
    node_ = std::make_shared<rclcpp::Node>("trajectory_transport_client_" + suffix);
    first_ = std::make_unique<FakeTrajectoryServer>("trajectory_transport_first_" + suffix);
    second_ = std::make_unique<FakeTrajectoryServer>("trajectory_transport_second_" + suffix);
    transport_ = std::make_unique<TrajectoryExecutor>(node_.get());
    executor_.add_node(node_);
    executor_.add_node(first_->node);
    executor_.add_node(second_->node);
    spin_ = std::thread([this]() {executor_.spin();});
    // Validation-only tests may finish before the new thread enters spin().
    // Ensure cancel() cannot run before spin() initializes its running flag.
    ASSERT_TRUE(waitUntil([this]() {return executor_.is_spinning();}));
    options_.controller_wait_timeout = 2.0;
    options_.acceptance_timeout = 0.5;
    options_.start_delay = 0.75;
    options_.execution_wait_margin = 1.0;
    targets_ = {{"first", first_->action_name, {"left_1"}},
      {"second", second_->action_name, {"right_1", "right_2"}}};
  }

  void TearDown() override
  {
    if (transport_) {transport_->cancel();}
    first_->stop.store(true);
    second_->stop.store(true);
    executor_.cancel();
    spin_.join();
    transport_.reset();
    executor_.remove_node(node_);
    executor_.remove_node(first_->node);
    executor_.remove_node(second_->node);
    first_.reset();
    second_.reset();
    node_.reset();
  }

  static trajectory_msgs::msg::JointTrajectory trajectory()
  {
    trajectory_msgs::msg::JointTrajectory result;
    result.joint_names = {"left_1", "right_1", "right_2"};
    trajectory_msgs::msg::JointTrajectoryPoint initial;
    initial.positions = {0.1, 0.2, 0.3};
    initial.velocities = {0.4, 0.5, 0.6};
    initial.accelerations = {0.7, 0.8, 0.9};
    result.points.push_back(initial);
    initial.time_from_start.sec = 1;
    result.points.push_back(initial);
    return result;
  }

  rclcpp::Node::SharedPtr node_;
  std::unique_ptr<FakeTrajectoryServer> first_, second_;
  std::unique_ptr<TrajectoryExecutor> transport_;
  rclcpp::executors::MultiThreadedExecutor executor_{rclcpp::ExecutorOptions(), 4};
  std::thread spin_;
  ExecutionOptions options_;
  std::vector<ExecutionTarget> targets_;
};

TEST_F(TrajectoryTransportTest, DispatchesBothArmsBeforeWaitingAndUsesOneFutureTimestamp)
{
  const auto before = node_->now();
  auto execution = std::async(std::launch::async, [this]() {
      return transport_->execute(trajectory(), targets_, options_);
    });
  const bool both = waitUntil([this]() {return first_->accepted && second_->accepted;});
  EXPECT_TRUE(both);
  first_->finish = true;
  second_->finish = true;
  const auto result = execution.get();
  EXPECT_TRUE(result.success) << result.message;
  EXPECT_TRUE(result.dispatched);
  EXPECT_TRUE(result.accepted);
  const auto left = first_->trajectory();
  const auto right = second_->trajectory();
  EXPECT_EQ(left.header.stamp, right.header.stamp);
  EXPECT_GT(rclcpp::Time(left.header.stamp).nanoseconds(), before.nanoseconds());
  ASSERT_EQ(right.points.size(), 2u);
  EXPECT_EQ(right.points[0].positions, (std::vector<double>{0.2, 0.3}));
  EXPECT_EQ(right.points[0].velocities, (std::vector<double>{0.5, 0.6}));
  EXPECT_EQ(right.points[0].accelerations, (std::vector<double>{0.8, 0.9}));
  EXPECT_EQ(right.points[1].time_from_start, left.points[1].time_from_start);
}

TEST_F(TrajectoryTransportTest, CancelsAcceptedPeerWhenOtherArmRejects)
{
  second_->reject = true;
  second_->acceptance_delay_ms = 100;
  const auto result = transport_->execute(trajectory(), targets_, options_);
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.retryable);
  EXPECT_TRUE(result.dispatched);
  EXPECT_TRUE(waitUntil([this]() {return first_->canceled > 0;}));
}

TEST_F(TrajectoryTransportTest, CancelsLateAcceptanceAfterTransportAlreadyReturned)
{
  second_->acceptance_delay_ms = 150;
  options_.acceptance_timeout = 0.05;
  const auto result = transport_->execute(trajectory(), targets_, options_);
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.retryable);
  EXPECT_TRUE(waitUntil([this]() {return first_->canceled > 0 && second_->canceled > 0;}));
}

TEST_F(TrajectoryTransportTest, CancelsLateAcceptanceEvenAfterTransportOwnerIsDestroyed)
{
  second_->acceptance_delay_ms = 150;
  options_.acceptance_timeout = 0.05;
  const auto result = transport_->execute(trajectory(), targets_, options_);
  EXPECT_FALSE(result.success);
  transport_.reset();
  EXPECT_TRUE(waitUntil([this]() {return first_->canceled > 0 && second_->canceled > 0;}));
}

TEST_F(TrajectoryTransportTest, ExplicitCancellationCancelsAllGoals)
{
  auto execution = std::async(std::launch::async, [this]() {
      return transport_->execute(trajectory(), targets_, options_);
    });
  EXPECT_TRUE(waitUntil([this]() {return first_->accepted && second_->accepted;}));
  transport_->cancel();
  const auto result = execution.get();
  EXPECT_TRUE(result.canceled);
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(waitUntil([this]() {return first_->canceled > 0 && second_->canceled > 0;}));
}

TEST_F(TrajectoryTransportTest, UnavailableBackendIsRetryableAndDoesNotDispatch)
{
  options_.controller_wait_timeout = 0.05;
  targets_[1].action_name = "/missing_trajectory_backend/follow_joint_trajectory";
  const auto result = transport_->execute(trajectory(), targets_, options_);
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.retryable);
  EXPECT_FALSE(result.dispatched);
  EXPECT_FALSE(first_->received.load());
}

TEST_F(TrajectoryTransportTest, SingleTaskAdmissionRejectionIsRetryable)
{
  first_->reject = true;
  const auto result = transport_->execute(trajectory(), {targets_[0]}, options_);
  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.retryable);
  EXPECT_TRUE(result.dispatched);
  EXPECT_FALSE(result.accepted);
}

TEST_F(TrajectoryTransportTest, JointTaskReceivesCompleteTrajectoryAndImmediateStamp)
{
  options_.use_common_start_stamp = false;
  first_->finish = true;
  const auto result = transport_->execute(
    trajectory(), {{"joint_task", first_->action_name, {}}}, options_);
  EXPECT_TRUE(result.success) << result.message;
  const auto sent = first_->trajectory();
  EXPECT_EQ(sent.joint_names, trajectory().joint_names);
  EXPECT_EQ(sent.header.stamp.sec, 0);
  EXPECT_EQ(sent.header.stamp.nanosec, 0u);
}

TEST_F(TrajectoryTransportTest, SuccessfulActionCodeWithControllerErrorStillFailsAndCancelsPeer)
{
  first_->result_error = Action::Result::INVALID_GOAL;
  first_->finish = true;
  const auto result = transport_->execute(trajectory(), targets_, options_);
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.retryable);
  EXPECT_TRUE(waitUntil([this]() {return second_->canceled > 0;}));
}

TEST_F(TrajectoryTransportTest, RejectsMissingJointsAndMalformedFieldsBeforeSending)
{
  auto source = trajectory();
  trajectory_msgs::msg::JointTrajectory selected;
  std::string error;
  EXPECT_FALSE(TrajectoryExecutor::partitionTrajectory(source, {"absent"}, selected, error));
  source.points[0].velocities.pop_back();
  const auto result = transport_->execute(source, targets_, options_);
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.dispatched);
  EXPECT_FALSE(first_->received.load());
  EXPECT_FALSE(second_->received.load());
}

TEST_F(TrajectoryTransportTest, RejectsCoordinatedStartBeforeAcceptanceDeadline)
{
  options_.start_delay = options_.acceptance_timeout;
  const auto result = transport_->execute(trajectory(), targets_, options_);
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.dispatched);
  EXPECT_FALSE(first_->received.load());
  EXPECT_FALSE(second_->received.load());
}

TEST_F(TrajectoryTransportTest, RejectsMultipleControllersWithoutCommonStartStamp)
{
  options_.use_common_start_stamp = false;
  const auto result = transport_->execute(trajectory(), targets_, options_);
  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.dispatched);
  EXPECT_FALSE(first_->received.load());
  EXPECT_FALSE(second_->received.load());
}

}  // namespace
}  // namespace sura_manipulation_actions
