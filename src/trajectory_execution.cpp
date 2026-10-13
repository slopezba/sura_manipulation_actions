#include "sura_manipulation_actions/trajectory_execution.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <future>
#include <set>
#include <thread>
#include <utility>

namespace sura_manipulation_actions
{
namespace
{
using SteadyClock = std::chrono::steady_clock;
using GoalHandle = rclcpp_action::ClientGoalHandle<TrajectoryExecutor::Action>;

double durationSeconds(const builtin_interfaces::msg::Duration & duration)
{
  return static_cast<double>(duration.sec) + static_cast<double>(duration.nanosec) * 1.0e-9;
}

bool validField(const std::vector<double> & field, size_t count, bool required)
{
  return ((!required && field.empty()) || field.size() == count) &&
         std::all_of(field.begin(), field.end(), [](double value) { return std::isfinite(value); });
}
}  // namespace

struct TrajectoryExecutor::SharedExecution
{
  struct Entry
  {
    std::weak_ptr<Client> client;
    GoalHandle::SharedPtr goal;
    bool response_received{false};
  };

  std::atomic_bool cancel_requested{false};
  std::atomic_bool finished{false};
  std::mutex mutex;
  std::vector<Entry> entries;
};

TrajectoryExecutor::~TrajectoryExecutor()
{
  cancel();
}

TrajectoryExecutor::Client::SharedPtr TrajectoryExecutor::clientFor(const std::string & action_name)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto & client = clients_[action_name];
  if (!client) {
    client = client_factory_(action_name);
  }
  return client;
}

void TrajectoryExecutor::cancelExecution(const std::shared_ptr<SharedExecution> & execution)
{
  if (!execution) {
    return;
  }
  execution->cancel_requested.store(true);
  std::vector<std::pair<Client::SharedPtr, GoalHandle::SharedPtr>> goals;
  {
    std::lock_guard<std::mutex> lock(execution->mutex);
    for (const auto & entry : execution->entries) {
      auto client = entry.client.lock();
      if (client && entry.goal) {
        goals.emplace_back(std::move(client), entry.goal);
      }
    }
  }
  for (const auto & item : goals) {
    try {
      item.first->async_cancel_goal(item.second);
    } catch (const std::exception &) {
      // ROS may already be shutting down; retain cancellation for late responses.
    }
  }
}

void TrajectoryExecutor::cancel()
{
  std::shared_ptr<SharedExecution> execution;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    execution = active_;
  }
  cancelExecution(execution);
}

bool TrajectoryExecutor::partitionTrajectory(
  const trajectory_msgs::msg::JointTrajectory & source,
  const std::vector<std::string> & joint_names,
  trajectory_msgs::msg::JointTrajectory & partition,
  std::string & error)
{
  partition = trajectory_msgs::msg::JointTrajectory();
  if (source.joint_names.empty() || source.points.empty()) {
    error = "Trajectory must contain joint names and points";
    return false;
  }
  std::set<std::string> source_names(source.joint_names.begin(), source.joint_names.end());
  if (source_names.size() != source.joint_names.size() || source_names.count("") != 0) {
    error = "Trajectory contains duplicate or empty joint names";
    return false;
  }
  const auto & selected = joint_names.empty() ? source.joint_names : joint_names;
  const std::set<std::string> selected_names(selected.begin(), selected.end());
  if (selected_names.size() != selected.size()) {
    error = "Execution target contains duplicate joint names";
    return false;
  }
  std::vector<size_t> indices;
  for (const auto & name : selected) {
    auto found = std::find(source.joint_names.begin(), source.joint_names.end(), name);
    if (found == source.joint_names.end()) {
      error = "Execution target joint is absent from trajectory: " + name;
      return false;
    }
    indices.push_back(static_cast<size_t>(std::distance(source.joint_names.begin(), found)));
  }
  partition.header = source.header;
  partition.joint_names = selected;
  double previous_time = -1.0;
  for (const auto & point : source.points) {
    const auto count = source.joint_names.size();
    if (!validField(point.positions, count, true) ||
      !validField(point.velocities, count, false) ||
      !validField(point.accelerations, count, false) ||
      !validField(point.effort, count, false))
    {
      error = "Trajectory point has inconsistent field sizes or nonfinite values";
      partition = trajectory_msgs::msg::JointTrajectory();
      return false;
    }
    const double time = durationSeconds(point.time_from_start);
    if (point.time_from_start.sec < 0 || point.time_from_start.nanosec >= 1000000000u ||
      time <= previous_time)
    {
      error = "Trajectory times must be nonnegative and strictly increasing";
      partition = trajectory_msgs::msg::JointTrajectory();
      return false;
    }
    previous_time = time;
    trajectory_msgs::msg::JointTrajectoryPoint selected_point;
    selected_point.time_from_start = point.time_from_start;
    for (auto index : indices) {
      selected_point.positions.push_back(point.positions[index]);
      if (!point.velocities.empty()) {
        selected_point.velocities.push_back(point.velocities[index]);
      }
      if (!point.accelerations.empty()) {
        selected_point.accelerations.push_back(point.accelerations[index]);
      }
      if (!point.effort.empty()) {
        selected_point.effort.push_back(point.effort[index]);
      }
    }
    partition.points.push_back(std::move(selected_point));
  }
  error.clear();
  return true;
}

ExecutionResult TrajectoryExecutor::execute(
  const trajectory_msgs::msg::JointTrajectory & trajectory,
  const std::vector<ExecutionTarget> & targets,
  const ExecutionOptions & options,
  std::function<bool()> canceled,
  std::function<void()> feedback)
{
  ExecutionResult result;
  if (targets.empty()) {
    result.message = "No trajectory execution targets configured";
    return result;
  }
  const std::vector<double> limits = {options.controller_wait_timeout, options.acceptance_timeout,
    options.start_delay, options.execution_wait_margin, options.feedback_rate};
  if (!std::all_of(limits.begin(), limits.end(), [](double value) {
      return std::isfinite(value) && value >= 0.0;
    }) || options.acceptance_timeout == 0.0 || options.feedback_rate == 0.0)
  {
    result.message = "Invalid trajectory execution timeout or feedback rate";
    return result;
  }
  if (targets.size() > 1 && (!options.use_common_start_stamp ||
    options.start_delay <= options.acceptance_timeout))
  {
    result.message =
      "Coordinated controllers require a common start timestamp after the goal acceptance timeout";
    return result;
  }
  std::vector<trajectory_msgs::msg::JointTrajectory> partitions(targets.size());
  std::set<std::string> endpoints;
  for (size_t index = 0; index < targets.size(); ++index) {
    if (targets[index].action_name.empty() || !endpoints.insert(targets[index].action_name).second) {
      result.message = "Execution action names must be nonempty and distinct";
      return result;
    }
    if (!partitionTrajectory(trajectory, targets[index].joint_names, partitions[index], result.message)) {
      return result;
    }
  }
  auto state = std::make_shared<SharedExecution>();
  state->entries.resize(targets.size());
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (active_ && !active_->finished.load()) {
      result.message = "Another trajectory execution is active";
      return result;
    }
    active_ = state;
  }
  auto finish = [&](bool success, const std::string & message) {
      result.success = success;
      result.message = message;
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        result.accepted = std::any_of(state->entries.begin(), state->entries.end(),
          [](const auto & entry) { return static_cast<bool>(entry.goal); });
      }
      if (!success) {
        cancelExecution(state);
      }
      state->finished.store(true);
      return result;
    };
  auto is_canceled = [&]() {
      return state->cancel_requested.load() || (canceled && canceled());
    };
  try {
    std::vector<Client::SharedPtr> clients;
    const auto discovery_deadline = SteadyClock::now() +
      std::chrono::duration<double>(options.controller_wait_timeout);
    for (size_t index = 0; index < targets.size(); ++index) {
      auto client = clientFor(targets[index].action_name);
      while (!client->action_server_is_ready()) {
        if (is_canceled() || !rclcpp::ok()) {
          result.canceled = is_canceled();
          return finish(false, "Trajectory execution canceled before dispatch");
        }
        if (SteadyClock::now() >= discovery_deadline) {
          result.retryable = true;
          return finish(false, "Trajectory action server unavailable: " + targets[index].action_name);
        }
        client->wait_for_action_server(std::chrono::milliseconds(25));
      }
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        state->entries[index].client = client;
      }
      clients.push_back(std::move(client));
    }
    if (is_canceled() || !rclcpp::ok()) {
      result.canceled = is_canceled();
      return finish(false, "Trajectory execution canceled before dispatch");
    }
    const auto dispatch_time = SteadyClock::now();
    builtin_interfaces::msg::Time start_stamp;
    if (options.use_common_start_stamp) {
      start_stamp = clock_->now() + rclcpp::Duration::from_seconds(options.start_delay);
    }
    // All requests are dispatched before waiting for any goal acceptance or result.
    for (size_t index = 0; index < targets.size(); ++index) {
      if (is_canceled()) {
        result.canceled = true;
        return finish(false, "Trajectory execution canceled during dispatch");
      }
      Action::Goal goal;
      goal.trajectory = std::move(partitions[index]);
      goal.trajectory.header.stamp = start_stamp;
      typename Client::SendGoalOptions send_options;
      // Keep this action client registered even if the owner cleans up while a
      // delayed admission reply is pending. The client releases this callback
      // after delivering the reply, breaking the temporary self reference.
      const auto response_client = clients[index];
      send_options.goal_response_callback = [state, response_client, index](GoalHandle::SharedPtr handle) {
          {
            std::lock_guard<std::mutex> lock(state->mutex);
            state->entries[index].goal = handle;
            state->entries[index].response_received = true;
          }
          // The shared token outlives execute(), including an acceptance timeout.
          if (handle && state->cancel_requested.load()) {
            try {
              response_client->async_cancel_goal(handle);
            } catch (const std::exception &) {
            }
          }
        };
      clients[index]->async_send_goal(goal, send_options);
      result.dispatched = true;
    }
    const auto acceptance_deadline = dispatch_time +
      std::chrono::duration<double>(options.acceptance_timeout);
    std::vector<GoalHandle::SharedPtr> handles(targets.size());
    while (rclcpp::ok()) {
      if (is_canceled()) {
        result.canceled = true;
        return finish(false, "Trajectory execution canceled during goal acceptance");
      }
      bool all_accepted = true;
      {
        std::lock_guard<std::mutex> lock(state->mutex);
        for (size_t index = 0; index < targets.size(); ++index) {
          const auto & entry = state->entries[index];
          if (entry.response_received && !entry.goal) {
            // Release the state mutex before canceling the other accepted goals.
            result.message = "Trajectory goal rejected: " + targets[index].action_name;
            result.retryable = targets.size() == 1;
            break;
          }
          handles[index] = entry.goal;
          all_accepted = all_accepted && entry.response_received;
        }
      }
      if (!result.message.empty()) {
        return finish(false, result.message);
      }
      if (all_accepted) {
        break;
      }
      if (SteadyClock::now() >= acceptance_deadline) {
        return finish(false, "Timed out waiting for trajectory goal acceptance");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    if (!rclcpp::ok()) {
      return finish(false, "ROS shutdown during trajectory goal acceptance");
    }
    using ResultFuture = std::shared_future<GoalHandle::WrappedResult>;
    std::vector<ResultFuture> results;
    for (size_t index = 0; index < targets.size(); ++index) {
      results.push_back(clients[index]->async_get_result(handles[index]));
    }
    std::vector<bool> complete(targets.size(), false);
    const double start_delay = options.use_common_start_stamp ? options.start_delay : 0.0;
    const auto execution_deadline = dispatch_time + std::chrono::duration<double>(
      start_delay + durationSeconds(trajectory.points.back().time_from_start) +
      options.execution_wait_margin);
    const auto feedback_interval = std::chrono::duration_cast<SteadyClock::duration>(
      std::chrono::duration<double>(1.0 / options.feedback_rate));
    auto next_feedback = SteadyClock::now();
    while (rclcpp::ok()) {
      if (is_canceled()) {
        result.canceled = true;
        return finish(false, "Trajectory execution canceled");
      }
      for (size_t index = 0; index < targets.size(); ++index) {
        if (complete[index] || results[index].wait_for(std::chrono::seconds(0)) !=
          std::future_status::ready)
        {
          continue;
        }
        const auto wrapped = results[index].get();
        if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result ||
          wrapped.result->error_code != Action::Result::SUCCESSFUL)
        {
          std::string diagnostic = "Trajectory execution failed: " + targets[index].action_name +
            " (result " + std::to_string(static_cast<int>(wrapped.code)) + ")";
          if (wrapped.result) {
            diagnostic += " error " + std::to_string(wrapped.result->error_code) +
              ": " + wrapped.result->error_string;
          }
          return finish(false, diagnostic);
        }
        complete[index] = true;
      }
      if (std::all_of(complete.begin(), complete.end(), [](bool value) { return value; })) {
        return finish(true, "Trajectory execution completed");
      }
      if (SteadyClock::now() >= execution_deadline) {
        return finish(false, "Trajectory execution timed out");
      }
      if (feedback && SteadyClock::now() >= next_feedback) {
        feedback();
        next_feedback = SteadyClock::now() + feedback_interval;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return finish(false, "ROS shutdown during trajectory execution");
  } catch (const std::exception & exception) {
    return finish(false, "Trajectory transport error: " + std::string(exception.what()));
  }
}

}  // namespace sura_manipulation_actions
