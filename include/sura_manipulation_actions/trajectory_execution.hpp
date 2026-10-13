#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

namespace sura_manipulation_actions
{

struct ExecutionTarget
{
  std::string name;
  std::string action_name;
  // Empty means send the complete trajectory (a single joint task).
  std::vector<std::string> joint_names;
};

struct ExecutionOptions
{
  double controller_wait_timeout{5.0};
  double acceptance_timeout{2.0};
  double start_delay{3.0};
  double execution_wait_margin{5.0};
  double feedback_rate{10.0};
  // Controllers share a future timestamp; tasks can use stamp zero instead.
  bool use_common_start_stamp{true};
};

struct ExecutionResult
{
  bool success{false};
  bool canceled{false};
  bool dispatched{false};
  bool accepted{false};
  // Only unavailable pre-dispatch, or a single target explicitly rejecting admission.
  bool retryable{false};
  std::string message;
};

/** FollowJointTrajectory transport only: it never activates tasks or controllers.
 * execute() runs on a worker while the node executor services action callbacks.
 * Clients remain alive to cancel a goal accepted after an acceptance timeout.
 */
class TrajectoryExecutor
{
public:
  using Action = control_msgs::action::FollowJointTrajectory;
  using Client = rclcpp_action::Client<Action>;

  template<typename NodeT>
  explicit TrajectoryExecutor(NodeT * node)
  : clock_(node->get_clock())
  {
    const auto base = node->get_node_base_interface();
    const auto graph = node->get_node_graph_interface();
    const auto logging = node->get_node_logging_interface();
    const auto waitables = node->get_node_waitables_interface();
    client_factory_ = [base, graph, logging, waitables](const std::string & action_name) {
        return rclcpp_action::create_client<Action>(base, graph, logging, waitables, action_name);
      };
  }

  ~TrajectoryExecutor();

  ExecutionResult execute(
    const trajectory_msgs::msg::JointTrajectory & trajectory,
    const std::vector<ExecutionTarget> & targets,
    const ExecutionOptions & options = ExecutionOptions{},
    std::function<bool()> canceled = {},
    std::function<void()> feedback = {});

  void cancel();

  static bool partitionTrajectory(
    const trajectory_msgs::msg::JointTrajectory & source,
    const std::vector<std::string> & joint_names,
    trajectory_msgs::msg::JointTrajectory & partition,
    std::string & error);

private:
  struct SharedExecution;
  static void cancelExecution(const std::shared_ptr<SharedExecution> & execution);
  Client::SharedPtr clientFor(const std::string & action_name);

  rclcpp::Clock::SharedPtr clock_;
  std::function<Client::SharedPtr(const std::string &)> client_factory_;
  std::mutex mutex_;
  std::unordered_map<std::string, Client::SharedPtr> clients_;
  std::shared_ptr<SharedExecution> active_;
};

}  // namespace sura_manipulation_actions
