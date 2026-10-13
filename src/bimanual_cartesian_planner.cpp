#include "sura_manipulation_actions/bimanual_cartesian_planner.hpp"
#include "sura_manipulation_actions/cartesian_trajectory_timing.hpp"
#include "sura_manipulation_actions/single_arm_cartesian_planner.hpp"

#include <moveit/robot_model/joint_model_group.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include "osqp.h"

#include <Eigen/Cholesky>
#include <Eigen/SVD>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

namespace sura_manipulation_actions::bimanual_cartesian
{
namespace
{
using Vector6 = Eigen::Matrix<double, 6, 1>;
using timing::seconds;
using timing::duration;
using timing::polynomial;
using timing::extrema;
using timing::validMessage;
using timing::interpolateSegment;
bool growSampleCount(double requested, int cap, int & count, std::string & error)
{
  if (!std::isfinite(requested) || requested < 0.0 || requested > cap) {
    error = "Segment exceeds the bounded interpolation validation budget";
    return false;
  }
  count = std::max(count, static_cast<int>(std::ceil(requested)));
  return true;
}

struct SparseMatrix
{
  std::vector<c_float> values;
  std::vector<c_int> rows;
  std::vector<c_int> columns;
  csc matrix{};

  SparseMatrix(const Eigen::MatrixXd & dense, bool upper)
  {
    columns.push_back(0);
    for (Eigen::Index column = 0; column < dense.cols(); ++column) {
      for (Eigen::Index row = 0; row < dense.rows(); ++row) {
        if ((!upper || row <= column) && std::abs(dense(row, column)) > 1.0e-14) {
          values.push_back(static_cast<c_float>(dense(row, column)));
          rows.push_back(static_cast<c_int>(row));
        }
      }
      columns.push_back(static_cast<c_int>(values.size()));
    }
    matrix.nzmax = static_cast<c_int>(values.size());
    matrix.m = static_cast<c_int>(dense.rows());
    matrix.n = static_cast<c_int>(dense.cols());
    matrix.p = columns.data();
    matrix.i = rows.data();
    matrix.x = values.data();
    matrix.nz = -1;
  }
};

bool solveQp(
  const Eigen::MatrixXd & hessian, const Eigen::VectorXd & gradient,
  const Eigen::MatrixXd & constraints, const Eigen::VectorXd & lower,
  const Eigen::VectorXd & upper, double remaining, Eigen::VectorXd & solution)
{
  if (!hessian.allFinite() || !gradient.allFinite() || !constraints.allFinite() ||
    !lower.allFinite() || !upper.allFinite() || (lower.array() > upper.array()).any())
  {return false;}
  SparseMatrix p(0.5 * (hessian + hessian.transpose()), true), a(constraints, false);
  std::vector<c_float> q(gradient.data(), gradient.data() + gradient.size());
  std::vector<c_float> l(lower.data(), lower.data() + lower.size());
  std::vector<c_float> u(upper.data(), upper.data() + upper.size());
  OSQPData data{};
  data.n = static_cast<c_int>(gradient.size());
  data.m = static_cast<c_int>(lower.size());
  data.P = &p.matrix;
  data.A = &a.matrix;
  data.q = q.data();
  data.l = l.data();
  data.u = u.data();
  OSQPSettings settings;
  osqp_set_default_settings(&settings);
  settings.verbose = 0;
  settings.polish = 1;
  settings.max_iter = 1200;
  settings.eps_abs = 1.0e-8;
  settings.eps_rel = 1.0e-8;
#ifdef PROFILING
  settings.time_limit = std::max(1.0e-4, std::min(remaining, 0.05));
#else
  (void)remaining;
#endif
  OSQPWorkspace * workspace = nullptr;
  if (osqp_setup(&workspace, &data, &settings) != 0 || !workspace) {
    if (workspace) {osqp_cleanup(workspace);}
    return false;
  }
  const auto status = osqp_solve(workspace);
  const bool solved = status == 0 && workspace->info && workspace->solution &&
    workspace->solution->x && (workspace->info->status_val == OSQP_SOLVED ||
    workspace->info->status_val == OSQP_SOLVED_INACCURATE);
  if (solved) {
    solution.resize(gradient.size());
    for (Eigen::Index i = 0; i < solution.size(); ++i) {
      solution(i) = workspace->solution->x[i];
    }
  }
  osqp_cleanup(workspace);
  return solved && solution.allFinite();
}

struct Waypoint
{
  Eigen::VectorXd q;
  double progress = 0.0;
};

struct Evaluation
{
  Eigen::Isometry3d arm_a = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d arm_b = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d relative = Eigen::Isometry3d::Identity();
  Eigen::MatrixXd arm_a_jacobian;
  Eigen::MatrixXd relative_jacobian;
};

class PlanningContext
{
public:
  PlanningContext(
    const planning_scene::PlanningSceneConstPtr & scene_input,
    const moveit::core::RobotState & start_input,
    const Arm & arm_a_input, const Arm & arm_b_input, const Pair & pair_input,
    const GoalSpec & goal_input, const Options & options_input,
    const BimanualCartesianPlanner::CancelCheck & cancel_input,
    const BimanualCartesianPlanner::Progress & progress_input, bool single_arm_input = false)
  : scene(scene_input), start(start_input), arm_a(arm_a_input), arm_b(arm_b_input),
    pair(pair_input), goal(goal_input), options(options_input), canceled(cancel_input),
    feedback(progress_input), single_arm(single_arm_input), deadline(std::chrono::steady_clock::now() +
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(std::max(0.0, goal.planning_time)))) {}

  bool initialize(std::string & error)
  {
    if (!scene || scene->getRobotModel().get() != start.getRobotModel().get()) {
      error = "Planning scene and initial state must use the same robot model";
      return false;
    }
    if (!single_arm && (arm_a.name == arm_b.name || arm_a.group_name == arm_b.group_name)) {
      error = "Arm A and arm B must be distinct manipulators";
      return false;
    }
    const auto model = start.getRobotModel();
    if (!Eigen::Map<const Eigen::VectorXd>(start.getVariablePositions(),
      static_cast<Eigen::Index>(start.getVariableCount())).allFinite())
    {error = "Initial robot state contains non-finite joint positions"; return false;}
    const auto * arm_a_group = single_arm ? nullptr : model->getJointModelGroup(arm_a.group_name);
    const auto * arm_b_group = model->getJointModelGroup(arm_b.group_name);
    const auto * joint_group = model->getJointModelGroup(pair.group_name);
    if ((!single_arm && (!arm_a_group || !arm_a_group->isChain())) ||
      !arm_b_group || !joint_group || !arm_b_group->isChain())
    {
      error = "Unknown planning group or manipulator group is not a serial chain";
      return false;
    }
    arm_a_tip = single_arm ? nullptr : model->getLinkModel(arm_a.tip_frame);
    arm_b_tip = model->getLinkModel(arm_b.tip_frame);
    if ((!single_arm && !arm_a_tip) || !arm_b_tip) {
      error = "Unknown manipulator tip frame";
      return false;
    }
    reference_frame = goal.reference_frame.empty() ? model->getModelFrame() : goal.reference_frame;
    if (reference_frame != model->getModelFrame()) {
      const auto * reference = model->getLinkModel(reference_frame);
      if (!reference || joint_group->getUpdatedLinkModelsSet().count(reference)) {
        error = "Relative reference frame must be a model link independent of the commanded arms";
        return false;
      }
      start.update();
      model_from_reference = start.getGlobalLinkTransform(reference);
    }
    auto active_names = [](const moveit::core::JointModelGroup * group) {
        std::set<std::string> out;
        for (const auto * joint : group->getActiveJointModels()) {out.insert(joint->getName());}
        return out;
      };
    const auto arm_a_names = single_arm ? std::set<std::string>() : active_names(arm_a_group);
    const auto arm_b_names = active_names(arm_b_group);
    if ((!single_arm && arm_a_names.empty()) || arm_b_names.empty()) {
      error = single_arm ? "Selected manipulator group must contain active joints" :
        "Both selected manipulator groups must contain active joints";
      return false;
    }
    const auto tip_contains = [](const moveit::core::LinkModel * tip,
      const std::set<std::string> & required) {
        std::set<std::string> ancestors;
        for (auto link = tip; link; link = link->getParentLinkModel()) {
          if (link->getParentJointModel()) {ancestors.insert(link->getParentJointModel()->getName());}
        }
        return std::includes(ancestors.begin(), ancestors.end(), required.begin(), required.end());
      };
    if ((!single_arm && !tip_contains(arm_a_tip, arm_a_names)) || !tip_contains(arm_b_tip, arm_b_names)) {
      error = "Configured tip must be downstream of every active joint of its manipulator";
      return false;
    }
    for (const auto * tip : {arm_a_tip, arm_b_tip}) {
      for (auto link = tip; link; link = link->getParentLinkModel()) {
        const auto * joint = link->getParentJointModel();
        if (joint && joint->getMimic()) {
          error = "Mimic joints inside an arm's tip chain are not supported: " + joint->getName();
          return false;
        }
      }
    }
    for (const auto * arm : {&arm_a, &arm_b}) {
      if (single_arm && arm == &arm_a) {continue;}
      const auto & expected = arm == &arm_a ? arm_a_names : arm_b_names;
      if (!arm->joint_names.empty() &&
        (std::set<std::string>(arm->joint_names.begin(), arm->joint_names.end()) != expected ||
        arm->joint_names.size() != expected.size()))
      {
        error = "Configured arm joints do not match the MoveIt group: " + arm->name;
        return false;
      }
    }
    for (const auto & name : arm_a_names) {
      if (arm_b_names.count(name)) {
        error = "Manipulator groups share an active joint: " + name;
        return false;
      }
    }
    std::set<std::string> combined = arm_a_names;
    combined.insert(arm_b_names.begin(), arm_b_names.end());
    if (combined.empty() || combined != active_names(joint_group)) {
      error = "Combined planning group must contain exactly the selected arms' active joints";
      return false;
    }
    for (const auto * joint : joint_group->getActiveJointModels()) {
      if (joint->isPassive()) {
        error = "Passive joints cannot be commanded by a Cartesian arm trajectory: " + joint->getName();
        return false;
      }
      if (joint->getVariableCount() != 1 ||
        (joint->getType() != moveit::core::JointModel::REVOLUTE &&
        joint->getType() != moveit::core::JointModel::PRISMATIC))
      {
        error = "Only independent revolute/prismatic arm joints are supported: " + joint->getName();
        return false;
      }
      joints.push_back(joint);
      joint_names.push_back(joint->getVariableNames().front());
      is_prismatic.push_back(joint->getType() == moveit::core::JointModel::PRISMATIC);
    }
    const Eigen::Index size = static_cast<Eigen::Index>(joints.size());
    if (!options.weights.empty() && options.weights.size() != joints.size()) {
      error = "planning.weights must match the combined group's active joint count";
      return false;
    }
    joint_weights = Eigen::VectorXd::Zero(size);
    for (std::size_t i = 0; i < options.weights.size(); ++i) {
      joint_weights(static_cast<Eigen::Index>(i)) = options.weights[i];
    }
    q0.resize(size); qnom.resize(size); lower.resize(size); upper.resize(size);
    vmax.resize(size); amax.resize(size);
    for (Eigen::Index i = 0; i < size; ++i) {
      const auto & name = joint_names[static_cast<std::size_t>(i)];
      const auto & bounds = model->getVariableBounds(name);
      q0(i) = start.getVariablePosition(name);
      lower(i) = bounds.position_bounded_ ? bounds.min_position_ : -OSQP_INFTY;
      upper(i) = bounds.position_bounded_ ? bounds.max_position_ : OSQP_INFTY;
      const auto tightened = options.tightened_limits.find(name);
      if (tightened != options.tightened_limits.end()) {
        if (!std::isfinite(tightened->second.first) || !std::isfinite(tightened->second.second)) {
          error = "Non-finite configured position bounds: " + name; return false;
        }
        lower(i) = std::max(lower(i), tightened->second.first);
        upper(i) = std::min(upper(i), tightened->second.second);
      }
      if (lower(i) > upper(i) || !std::isfinite(q0(i)) || q0(i) < lower(i) || q0(i) > upper(i)) {
        error = "Initial state is outside the effective position bounds: " + name; return false;
      }
      if (!bounds.velocity_bounded_ || !bounds.acceleration_bounded_ ||
        !std::isfinite(bounds.max_velocity_) || !std::isfinite(bounds.max_acceleration_) ||
        bounds.max_velocity_ <= 0.0 || bounds.max_acceleration_ <= 0.0)
      {
        error = "Robot model is missing positive velocity/acceleration limits: " + name; return false;
      }
      vmax(i) = bounds.max_velocity_ * options.velocity_scaling;
      amax(i) = bounds.max_acceleration_ * options.acceleration_scaling;
      if (!std::isfinite(vmax(i)) || !std::isfinite(amax(i)) || vmax(i) <= 0.0 || amax(i) <= 0.0) {
        error = "Scaled dynamic joint limits cannot be represented: " + name; return false;
      }
      qnom(i) = q0(i);
      const auto nominal = options.nominal_positions.find(name);
      if (nominal != options.nominal_positions.end()) {
        if (!std::isfinite(nominal->second) || nominal->second < lower(i) || nominal->second > upper(i)) {
          error = "Nominal joint posture is outside position bounds: " + name; return false;
        }
        qnom(i) = nominal->second;
      }
    }
    start.update();
    const auto current = evaluate(q0);
    arm_a0 = current.arm_a;
    relative0 = current.relative;
    if (!valid(q0, 0.0, error)) {error = "Invalid initial state: " + error; return false;}
    return true;
  }

  bool stopped(std::string & error) const
  {
    if (canceled && canceled()) {error = "Planning canceled"; return true;}
    if (std::chrono::steady_clock::now() >= deadline) {error = "Planning time budget exhausted"; return true;}
    return false;
  }

  Evaluation evaluate(const Eigen::VectorXd & q, bool jacobians = false) const
  {
    auto state = stateAt(q);
    Evaluation result;
    result.arm_a = single_arm ? model_from_reference : state.getGlobalLinkTransform(arm_a_tip);
    result.arm_b = state.getGlobalLinkTransform(arm_b_tip);
    result.relative = BimanualCartesianPlanner::relativePoseInReference(
      result.arm_a, result.arm_b, model_from_reference);
    if (!jacobians) {return result;}
    result.arm_a_jacobian = Eigen::MatrixXd::Zero(6, q.size());
    if (!single_arm) {result.arm_a_jacobian = jacobian(state, arm_a_tip, arm_a.group_name);}
    const Eigen::MatrixXd arm_b_jacobian = jacobian(state, arm_b_tip, arm_b.group_name);
    result.relative_jacobian = Eigen::MatrixXd::Zero(6, q.size());
    const Eigen::Matrix3d rotation = model_from_reference.rotation().transpose();
    result.relative_jacobian.topRows(3) = rotation * (arm_b_jacobian.topRows(3) -
      result.arm_a_jacobian.topRows(3));
    // Spatial angular velocity of R_B R_A^T is omega_B - (R_B R_A^T) omega_A.
    const Eigen::Matrix3d delta = result.arm_b.rotation() * result.arm_a.rotation().transpose();
    result.relative_jacobian.bottomRows(3) = rotation *
      (arm_b_jacobian.bottomRows(3) - delta * result.arm_a_jacobian.bottomRows(3));
    return result;
  }

  moveit::core::RobotState stateAt(const Eigen::VectorXd & q) const
  {
    auto state = start;
    for (Eigen::Index i = 0; i < q.size(); ++i) {
      state.setVariablePosition(joint_names[static_cast<std::size_t>(i)], q(i));
    }
    state.update();
    return state;
  }

  Vector6 errorAt(const Evaluation & current, double progress) const
  {
    const auto target = BimanualCartesianPlanner::interpolateRelativePose(
      relative0, goal.relative_target, progress);
    Vector6 error;
    error.head<3>() = target.translation() - current.relative.translation();
    error.tail<3>() = BimanualCartesianPlanner::orientationError(
      target.rotation(), current.relative.rotation());
    return error;
  }

  bool valid(const Eigen::VectorXd & q, double progress, std::string & error) const
  {
    if (q.size() != q0.size() || !q.allFinite() ||
      (q.array() < lower.array() - 1.0e-9).any() || (q.array() > upper.array() + 1.0e-9).any())
    {error = "Joint position limits exceeded"; return false;}
    const auto current = evaluate(q);
    const auto residual = errorAt(current, progress);
    if ((residual.head<3>().cwiseAbs().array() >
      goal.relative_position_tolerance.array() + options.position_epsilon).any() ||
      (residual.tail<3>().cwiseAbs().array() >
      goal.relative_orientation_tolerance.array() + options.orientation_epsilon).any())
    {error = single_arm ? "Tool pose is outside the Cartesian tolerances" :
      "Relative pose is outside the Cartesian tolerances"; return false;}
    auto state = stateAt(q);
    if (scene->isStateColliding(state)) {error = "Trajectory intersects the planning scene"; return false;}
    return true;
  }

  bool segmentValid(const Waypoint & a, const Waypoint & b, std::string & error) const
  {
    int samples = 4;
    for (Eigen::Index i = 0; i < a.q.size(); ++i) {
      const double resolution = is_prismatic[static_cast<std::size_t>(i)] ?
        options.validation_prismatic_step : options.validation_joint_step;
      if (!growSampleCount(std::abs(b.q(i) - a.q(i)) / resolution, 4096, samples, error)) {return false;}
    }
    if (samples > 4096) {error = "Segment exceeds the bounded continuity validation budget"; return false;}
    for (int i = 0; i <= samples; ++i) {
      if (stopped(error)) {return false;}
      const double u = static_cast<double>(i) / samples;
      const Eigen::VectorXd q = a.q + u * (b.q - a.q);
      const double s = i == 0 ? a.progress :
        (i == samples ? b.progress : projectedProgress(q, a.progress, b.progress));
      if (!valid(q, s, error)) {
        return false;
      }
    }
    return true;
  }

  bool solveWaypoint(
    double progress, const Eigen::VectorXd & previous, const Eigen::VectorXd & seed,
    Eigen::VectorXd & solution, std::string & error) const
  {
    Eigen::VectorXd q = seed.cwiseMax(lower).cwiseMin(upper);
    const Eigen::Index n = q.size();
    double trust = 1.0;
    double slack_weight = 1.0e4;
    for (int iteration = 0; iteration < options.max_iterations; ++iteration) {
      if (stopped(error)) {return false;}
      const auto current = evaluate(q, true);
      const Vector6 residual = errorAt(current, progress);
      if (valid(q, progress, error)) {solution = q; return true;}
      const int variables = static_cast<int>(n) + 6;
      const int count = static_cast<int>(n) + 18;
      Eigen::MatrixXd h = Eigen::MatrixXd::Zero(variables, variables);
      Eigen::VectorXd g = Eigen::VectorXd::Zero(variables);
      Vector6 weights;
      weights.head<3>().setConstant(options.position_weight);
      weights.tail<3>().setConstant(options.orientation_weight);
      h.topLeftCorner(n, n) = 2.0 * current.relative_jacobian.transpose() *
        weights.asDiagonal() * current.relative_jacobian;
      g.head(n) = -2.0 * current.relative_jacobian.transpose() * weights.asDiagonal() * residual;
      for (Eigen::Index i = 0; i < n; ++i) {
        const double joint_weight = joint_weights(i);
        const double regularization = options.continuity_weight + joint_weight + options.nominal_weight;
        h(i, i) += 2.0 * regularization + options.damping * options.damping;
        g(i) += 2.0 * (options.continuity_weight * (q(i) - previous(i)) +
          joint_weight * (q(i) - q0(i)) + options.nominal_weight * (q(i) - qnom(i)));
      }
      // Slack is exclusively an internal SQP restoration aid. Final acceptance
      // always checks the original nonlinear constraints without slack.
      h.bottomRightCorner(6, 6).diagonal().setConstant(2.0 * slack_weight);
      Eigen::MatrixXd a = Eigen::MatrixXd::Zero(count, variables);
      Eigen::VectorXd l = Eigen::VectorXd::Constant(count, -OSQP_INFTY);
      Eigen::VectorXd u = Eigen::VectorXd::Constant(count, OSQP_INFTY);
      int row = 0;
      for (Eigen::Index i = 0; i < n; ++i, ++row) {
        a(row, i) = 1.0;
        const double step = trust * (is_prismatic[static_cast<std::size_t>(i)] ?
          options.max_prismatic_step : options.max_joint_step);
        l(row) = std::max(lower(i) - q(i), -step);
        u(row) = std::min(upper(i) - q(i), step);
      }
      Vector6 tolerances;
      tolerances.head<3>() = goal.relative_position_tolerance;
      tolerances.tail<3>() = goal.relative_orientation_tolerance;
      for (int axis = 0; axis < 6; ++axis) {
        a.block(row, 0, 1, n) = current.relative_jacobian.row(axis);
        a(row, n + axis) = 1.0;
        l(row++) = residual(axis) - tolerances(axis);
        a.block(row, 0, 1, n) = current.relative_jacobian.row(axis);
        a(row, n + axis) = -1.0;
        u(row++) = residual(axis) + tolerances(axis);
        a(row, n + axis) = 1.0;
        l(row++) = 0.0;
      }
      Eigen::VectorXd increment;
      const double remaining = std::chrono::duration<double>(deadline - std::chrono::steady_clock::now()).count();
      if (!solveQp(h, g, a, l, u, remaining, increment)) {
        trust *= 0.5;
        if (trust < 1.0e-4) {error = "Local constrained IK could not solve its quadratic subproblem"; return false;}
        continue;
      }
      const double old_merit = merit(q, previous, progress, slack_weight);
      bool accepted = false;
      for (int line = 0; line < 12; ++line) {
        if (stopped(error)) {return false;}
        Eigen::VectorXd candidate = q + std::ldexp(1.0, -line) * increment.head(n);
        candidate = candidate.cwiseMax(lower).cwiseMin(upper);
        auto state = stateAt(candidate);
        if (scene->isStateColliding(state)) {continue;}
        if (valid(candidate, progress, error) || merit(candidate, previous, progress, slack_weight) < old_merit - 1.0e-12) {
          q = std::move(candidate);
          accepted = true;
          break;
        }
      }
      if (!accepted) {
        // At a slack-penalized optimum the remaining residual can still exceed
        // a strict zero tolerance. Increase restoration pressure before
        // interpreting that plateau as an exhausted geometric trust region.
        if (slack_weight < 1.0e9) {slack_weight = std::min(1.0e9, slack_weight * 10.0);}
        else {
          trust *= 0.5;
          if (trust < 1.0e-4) {break;}
        }
      } else {
        trust = std::min(1.0, trust * 1.25);
      }
    }
    if (valid(q, progress, error)) {solution = q; return true;}
    error = "No valid local constrained IK solution found: " + error;
    return false;
  }

  bool appendSegment(std::vector<Waypoint> & points, double target_progress, int depth, std::string & error) const
  {
    if (stopped(error)) {return false;}
    if (points.size() >= static_cast<std::size_t>(options.max_waypoints)) {
      error = "Maximum trajectory waypoint count reached"; return false;
    }
    const Waypoint previous = points.back();
    for (int seed = 0; seed <= options.recovery_seeds; ++seed) {
      if (stopped(error)) {return false;}
      Eigen::VectorXd initial = previous.q;
      if (seed > 0) {
        const Eigen::Index joint = ((seed - 1) / 2) % previous.q.size();
        const double sign = seed % 2 ? 1.0 : -1.0;
        const double max_offset = is_prismatic[static_cast<std::size_t>(joint)] ? 0.005 : 0.05;
        const double range = upper(joint) - lower(joint);
        initial(joint) += sign * std::min(max_offset, 0.02 * range);
      }
      Eigen::VectorXd result;
      if (solveWaypoint(target_progress, previous.q, initial, result, error)) {
        Waypoint waypoint{result, target_progress};
        if (segmentValid(previous, waypoint, error)) {points.push_back(std::move(waypoint)); return true;}
      }
    }
    if (depth >= options.max_subdivision_depth || target_progress - previous.progress < 1.0e-7) {
      error = "No valid continuous segment found after bounded local recovery: " + error;
      return false;
    }
    const double midpoint = 0.5 * (previous.progress + target_progress);
    return appendSegment(points, midpoint, depth + 1, error) &&
           appendSegment(points, target_progress, depth + 1, error);
  }

  void smooth(std::vector<Waypoint> & points, std::string & error) const
  {
    if (!options.optimize_trajectory || options.smoothness_weight == 0.0) {return;}
    for (int pass = 0; pass < options.smoothing_passes; ++pass) {
      for (std::size_t i = 1; i + 1 < points.size(); ++i) {
        if (stopped(error)) {return;}
        const auto midpoint = 0.5 * (points[i - 1].q + points[i + 1].q);
        const double strength = options.smoothness_weight / (1.0 + options.smoothness_weight);
        const Eigen::VectorXd seed = points[i].q + strength * (midpoint - points[i].q);
        Eigen::VectorXd q;
        if (!solveWaypoint(points[i].progress, midpoint, seed, q, error)) {continue;}
        Waypoint candidate{q, points[i].progress};
        const double old_cost = (points[i + 1].q - 2.0 * points[i].q + points[i - 1].q).squaredNorm();
        const double new_cost = (points[i + 1].q - 2.0 * q + points[i - 1].q).squaredNorm();
        if (new_cost < old_cost && segmentValid(points[i - 1], candidate, error) &&
          segmentValid(candidate, points[i + 1], error)) {points[i] = std::move(candidate);}
      }
    }
  }

  bool timeAndValidate(std::vector<Waypoint> & points, PlannedTrajectory & output, std::string & error) const
  {
    for (int refinement = 0; refinement <= options.max_timing_refinements; ++refinement) {
      if (stopped(error)) {return false;}
      auto trajectory = std::make_shared<robot_trajectory::RobotTrajectory>(start.getRobotModel(), pair.group_name);
      for (const auto & waypoint : points) {
        auto state = stateAt(waypoint.q);
        trajectory->addSuffixWayPoint(state, 0.0);
      }
      if (!timing::parameterize(*trajectory, joint_names, options.velocity_scaling,
        options.acceleration_scaling, error)) {return false;}
      moveit_msgs::msg::RobotTrajectory robot_message;
      trajectory->getRobotTrajectoryMsg(robot_message, joint_names);
      auto & msg = robot_message.joint_trajectory;
      if (!validMessage(msg, error) || msg.points.size() != points.size()) {return false;}
      std::vector<Eigen::Index> order;
      for (const auto & name : msg.joint_names) {
        const auto found = std::find(joint_names.begin(), joint_names.end(), name);
        if (found == joint_names.end()) {error = "Time parameterization returned an unexpected joint"; return false;}
        order.push_back(std::distance(joint_names.begin(), found));
      }
      if (order.size() != joint_names.size()) {error = "Time parameterization dropped active joints"; return false;}
      double stretch = 1.0;
      std::size_t bad_segment = points.size();
      for (std::size_t segment = 1; segment < points.size(); ++segment) {
        if (stopped(error)) {return false;}
        const auto & a = msg.points[segment - 1];
        const auto & b = msg.points[segment];
        if (a.velocities.size() != order.size() || a.accelerations.size() != order.size() ||
          b.velocities.size() != order.size() || b.accelerations.size() != order.size())
        {error = "Time parameterization omitted velocity/acceleration values"; return false;}
        const double span = seconds(b.time_from_start) - seconds(a.time_from_start);
        int samples = 4;
        if (!growSampleCount(span / options.validation_time_step, 20000, samples, error)) {return false;}
        for (std::size_t j = 0; j < order.size(); ++j) {
          const auto c = polynomial(a, b, j, span);
          const auto position_extrema = extrema(c, 1);
          const auto velocity_extrema = extrema(c, 2);
          const auto acceleration_extrema = extrema(c, 3);
          if (position_extrema.empty() || velocity_extrema.empty() || acceleration_extrema.empty()) {
            error = "Could not compute interpolation extrema"; return false;
          }
          for (const double u : position_extrema) {
            const double position = timing::evaluate(c, u);
            if (position < lower(order[j]) - 1.0e-9 || position > upper(order[j]) + 1.0e-9) {
              bad_segment = segment; error = "Quintic interpolation exceeds joint position bounds";
            }
          }
          double maximum_velocity = 0.0;
          for (const double u : velocity_extrema) {maximum_velocity = std::max(maximum_velocity, std::abs(timing::evaluate(c, u, 1) / span));}
          for (const double u : acceleration_extrema) {
            stretch = std::max(stretch, std::sqrt(std::abs(timing::evaluate(c, u, 2) / (span * span)) / amax(order[j])));
          }
          stretch = std::max(stretch, maximum_velocity / vmax(order[j]));
          const double resolution = is_prismatic[static_cast<std::size_t>(order[j])] ?
            options.validation_prismatic_step : options.validation_joint_step;
          if (!growSampleCount(maximum_velocity * span / resolution, 20000, samples, error)) {return false;}
        }
        if (samples > 20000) {error = "Timed segment exceeds the bounded validation budget"; return false;}
        trajectory_msgs::msg::JointTrajectoryPoint sample;
        for (int sample_index = 0; sample_index <= samples; ++sample_index) {
          if (stopped(error)) {return false;}
          const double u = static_cast<double>(sample_index) / samples;
          interpolateSegment(a, b, u, span, sample);
          Eigen::VectorXd q(q0.size());
          for (std::size_t j = 0; j < order.size(); ++j) {q(order[j]) = sample.positions[j];}
          const double s = sample_index == 0 ? points[segment - 1].progress :
            (sample_index == samples ? points[segment].progress :
            projectedProgress(q, points[segment - 1].progress, points[segment].progress));
          if (!valid(q, s, error)) {bad_segment = segment; break;}
        }
        if (bad_segment != points.size()) {break;}
      }
      if (bad_segment == points.size()) {
        // Uniform dilation preserves every boundary polynomial in normalized time.
        if (stretch > 1.0 + 1.0e-8) {
          stretch *= 1.001;
          for (auto & point : msg.points) {
            point.time_from_start = duration(seconds(point.time_from_start) * stretch);
            for (auto & value : point.velocities) {value /= stretch;}
            for (auto & value : point.accelerations) {value /= stretch * stretch;}
          }
        }
        if (!validMessage(msg, error)) {return false;}
        trajectory->setRobotTrajectoryMsg(start, msg);
        output.trajectory = std::move(trajectory);
        output.joint_trajectory = msg;
        output.progress_values.clear();
        for (const auto & point : points) {output.progress_values.push_back(point.progress);}
        return true;
      }
      if (refinement == options.max_timing_refinements ||
        points.size() >= static_cast<std::size_t>(options.max_waypoints))
      {error = "Timed trajectory remains invalid after bounded refinement: " + error; return false;}
      const auto & before = points[bad_segment - 1];
      const auto & after = points[bad_segment];
      Waypoint midpoint{0.5 * (before.q + after.q), 0.5 * (before.progress + after.progress)};
      if (!valid(midpoint.q, midpoint.progress, error)) {
        Eigen::VectorXd replacement;
        if (!solveWaypoint(midpoint.progress, before.q, midpoint.q, replacement, error)) {return false;}
        midpoint.q = std::move(replacement);
      }
      if (!segmentValid(before, midpoint, error) || !segmentValid(midpoint, after, error)) {return false;}
      points.insert(points.begin() + static_cast<std::ptrdiff_t>(bad_segment), std::move(midpoint));
    }
    error = "Time parameterization validation failed";
    return false;
  }

  bool validateTimed(const PlannedTrajectory & plan, std::string & error) const
  {
    const auto & msg = plan.joint_trajectory;
    if (!validMessage(msg, error) || plan.progress_values.size() != msg.points.size() ||
      plan.progress_values.size() < 2 || seconds(msg.points.front().time_from_start) != 0.0 ||
      plan.progress_values.front() != 0.0 || plan.progress_values.back() != 1.0)
    {error = "Stored plan has inconsistent trajectory times or nominal-path parameters: " + error; return false;}
    std::vector<Eigen::Index> order;
    for (const auto & name : msg.joint_names) {
      const auto found = std::find(joint_names.begin(), joint_names.end(), name);
      if (found == joint_names.end()) {error = "Stored plan contains an unexpected joint"; return false;}
      order.push_back(std::distance(joint_names.begin(), found));
    }
    if (order.size() != joint_names.size()) {error = "Stored plan omits active joints"; return false;}
    for (std::size_t i = 0; i < plan.progress_values.size(); ++i) {
      const auto s = plan.progress_values[i];
      if (!std::isfinite(s) || s < 0.0 || s > 1.0 ||
        (i > 0 && s <= plan.progress_values[i - 1]))
      {error = "Stored plan nominal-path parameters are not strictly increasing"; return false;}
    }
    for (std::size_t segment = 1; segment < msg.points.size(); ++segment) {
      if (stopped(error)) {return false;}
      const auto & a = msg.points[segment - 1];
      const auto & b = msg.points[segment];
      const double span = seconds(b.time_from_start) - seconds(a.time_from_start);
      if (a.velocities.size() != order.size() || b.velocities.size() != order.size() ||
        a.accelerations.size() != order.size() || b.accelerations.size() != order.size())
      {error = "Stored plan is missing quintic boundary derivatives"; return false;}
      int samples = 4;
      if (!growSampleCount(span / options.validation_time_step, 20000, samples, error)) {return false;}
      for (std::size_t j = 0; j < order.size(); ++j) {
        const auto c = polynomial(a, b, j, span);
        const auto p_roots = extrema(c, 1), v_roots = extrema(c, 2), a_roots = extrema(c, 3);
        if (p_roots.empty() || v_roots.empty() || a_roots.empty()) {
          error = "Could not determine stored interpolation extrema"; return false;
        }
        for (double u : p_roots) {
          const double q = timing::evaluate(c, u);
          if (q < lower(order[j]) - 1.0e-9 || q > upper(order[j]) + 1.0e-9) {
            error = "Stored quintic curve violates joint position bounds"; return false;
          }
        }
        double peak = 0.0;
        for (double u : v_roots) {
          const double velocity = std::abs(timing::evaluate(c, u, 1) / span);
          if (velocity > vmax(order[j]) * (1.0 + 1.0e-6)) {
            error = "Stored quintic curve violates joint velocity bounds"; return false;
          }
          peak = std::max(peak, velocity);
        }
        for (double u : a_roots) {
          if (std::abs(timing::evaluate(c, u, 2) / (span * span)) > amax(order[j]) * (1.0 + 1.0e-6)) {
            error = "Stored quintic curve violates joint acceleration bounds"; return false;
          }
        }
        const double resolution = is_prismatic[static_cast<std::size_t>(order[j])] ?
          options.validation_prismatic_step : options.validation_joint_step;
        if (!growSampleCount(peak * span / resolution, 20000, samples, error)) {return false;}
      }
      if (samples > 20000) {error = "Stored segment exceeds the bounded validation budget"; return false;}
      trajectory_msgs::msg::JointTrajectoryPoint sample;
      for (int k = 0; k <= samples; ++k) {
        if (stopped(error)) {return false;}
        const double u = static_cast<double>(k) / samples;
        interpolateSegment(a, b, u, span, sample);
        Eigen::VectorXd q(q0.size());
        for (std::size_t j = 0; j < order.size(); ++j) {q(order[j]) = sample.positions[j];}
        if (segment == 1 && k == 0 && (q - q0).cwiseAbs().maxCoeff() > 1.0e-10) {
          error = "Stored trajectory does not begin at its recorded initial state"; return false;
        }
        const double s = k == 0 ? plan.progress_values[segment - 1] :
          (k == samples ? plan.progress_values[segment] :
          projectedProgress(q, plan.progress_values[segment - 1], plan.progress_values[segment]));
        if (!valid(q, s, error)) {return false;}
      }
    }
    return true;
  }

  planning_scene::PlanningSceneConstPtr scene;
  moveit::core::RobotState start;
  const Arm & arm_a;
  const Arm & arm_b;
  const Pair & pair;
  const GoalSpec & goal;
  const Options & options;
  const BimanualCartesianPlanner::CancelCheck & canceled;
  const BimanualCartesianPlanner::Progress & feedback;
  bool single_arm;
  std::chrono::steady_clock::time_point deadline;
  const moveit::core::LinkModel * arm_a_tip = nullptr;
  const moveit::core::LinkModel * arm_b_tip = nullptr;
  std::vector<const moveit::core::JointModel *> joints;
  std::vector<std::string> joint_names;
  std::vector<bool> is_prismatic;
  Eigen::VectorXd q0, qnom, lower, upper, vmax, amax, joint_weights;
  Eigen::Isometry3d arm_a0 = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d relative0 = Eigen::Isometry3d::Identity();
  std::string reference_frame;
  Eigen::Isometry3d model_from_reference = Eigen::Isometry3d::Identity();

private:
  double projectedProgress(const Eigen::VectorXd & q, double before, double after) const
  {
    if (!q.allFinite()) {return before;}
    const auto current = evaluate(q);
    const Eigen::Vector3d translation = goal.relative_target.translation() - relative0.translation();
    double progress = 0.5 * (before + after);
    if (translation.squaredNorm() > 1.0e-24) {
      progress = (current.relative.translation() - relative0.translation()).dot(translation) /
        translation.squaredNorm();
    } else {
      const Eigen::Vector3d desired_rotation = BimanualCartesianPlanner::orientationError(
        relative0.rotation().transpose() * goal.relative_target.rotation(), Eigen::Matrix3d::Identity());
      if (desired_rotation.squaredNorm() > 1.0e-24) {
        const Eigen::Vector3d actual_rotation = BimanualCartesianPlanner::orientationError(
          relative0.rotation().transpose() * current.relative.rotation(), Eigen::Matrix3d::Identity());
        progress = actual_rotation.dot(desired_rotation) / desired_rotation.squaredNorm();
      }
    }
    return std::isfinite(progress) ? std::clamp(progress, before, after) : before;
  }

  Eigen::MatrixXd jacobian(
    const moveit::core::RobotState & state, const moveit::core::LinkModel * tip,
    const std::string & group_name) const
  {
    const auto * group = state.getJointModelGroup(group_name);
    Eigen::MatrixXd local;
    if (!group || group->getJointModels().empty() ||
      !state.getJacobian(group, tip, Eigen::Vector3d::Zero(), local, false) ||
      local.rows() != 6 || local.cols() != static_cast<Eigen::Index>(group->getVariableNames().size()) ||
      !local.allFinite())
    {throw std::runtime_error("MoveIt could not compute the manipulator Jacobian: " + group_name);}
    // Humble RobotState::getJacobian uses the parent link of
    // group->getJointModels().front() as its reference. Rotate both twist blocks
    // into the common model frame before assembling the joint-relative task.
    const auto * reference = group->getJointModels().front()->getParentLinkModel();
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    if (reference) {rotation = state.getGlobalLinkTransform(reference).rotation();}
    Eigen::MatrixXd result = Eigen::MatrixXd::Zero(6, static_cast<Eigen::Index>(joints.size()));
    const auto & variables = group->getVariableNames();
    for (std::size_t i = 0; i < variables.size(); ++i) {
      const auto found = std::find(joint_names.begin(), joint_names.end(), variables[i]);
      if (found == joint_names.end()) {continue;}
      const Eigen::Index column = std::distance(joint_names.begin(), found);
      result.col(column).head<3>() = rotation * local.col(static_cast<Eigen::Index>(i)).head<3>();
      result.col(column).tail<3>() = rotation * local.col(static_cast<Eigen::Index>(i)).tail<3>();
    }
    return result;
  }

  double merit(const Eigen::VectorXd & q, const Eigen::VectorXd & previous, double s, double penalty) const
  {
    const auto residual = errorAt(evaluate(q), s);
    double result = options.position_weight * residual.head<3>().squaredNorm() +
      options.orientation_weight * residual.tail<3>().squaredNorm() +
      options.continuity_weight * (q - previous).squaredNorm() +
      options.nominal_weight * (q - qnom).squaredNorm();
    for (Eigen::Index i = 0; i < q.size(); ++i) {
      result += joint_weights(i) * std::pow(q(i) - q0(i), 2);
    }
    const Eigen::Vector3d p_excess = (residual.head<3>().cwiseAbs() - goal.relative_position_tolerance).cwiseMax(0.0);
    const Eigen::Vector3d r_excess = (residual.tail<3>().cwiseAbs() - goal.relative_orientation_tolerance).cwiseMax(0.0);
    return result + penalty * (p_excess.squaredNorm() + r_excess.squaredNorm());
  }

};

bool validOptions(const GoalSpec & goal, const Options & options, std::string & error)
{
  const auto finite_positive = [](double value) {return std::isfinite(value) && value > 0.0;};
  if (!goal.relative_target.matrix().allFinite() ||
    (goal.relative_target.rotation().transpose() * goal.relative_target.rotation() - Eigen::Matrix3d::Identity()).norm() > 1.0e-6 ||
    std::abs(goal.relative_target.rotation().determinant() - 1.0) > 1.0e-6 ||
    (goal.relative_target.matrix().row(3) - Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)).norm() > 1.0e-9)
  {error = "Relative target is not a finite rigid transform"; return false;}
  for (const auto * values : {&goal.relative_position_tolerance, &goal.relative_orientation_tolerance})
  {
    if (!values->allFinite() || (values->array() < 0.0).any()) {
      error = "Cartesian tolerances must be finite and non-negative"; return false;
    }
  }
  if (!finite_positive(goal.planning_time) || goal.planning_time > 3600.0 ||
    !finite_positive(goal.cartesian_step) ||
    options.max_iterations <= 0 || options.max_waypoints < 2 ||
    options.max_subdivision_depth < 0 || options.recovery_seeds < 0 ||
    options.smoothing_passes < 0 || options.max_timing_refinements < 0)
  {error = "Invalid planning time, resolution or iteration budget"; return false;}
  for (double value : {options.angular_step, options.position_weight, options.orientation_weight,
    options.damping, options.max_joint_step, options.max_prismatic_step,
    options.validation_joint_step, options.validation_prismatic_step, options.validation_time_step,
    options.velocity_scaling, options.acceleration_scaling, options.position_epsilon, options.orientation_epsilon})
  {
    if (!finite_positive(value)) {error = "Planner resolution, weights, scaling and numerical epsilon must be positive"; return false;}
  }
  for (double value : {options.continuity_weight, options.nominal_weight, options.smoothness_weight})
  {
    if (!std::isfinite(value) || value < 0.0) {error = "Optimization weights must be finite and non-negative"; return false;}
  }
  for (const double weight : options.weights) {
    if (!std::isfinite(weight) || weight < 0.0) {
      error = "planning.weights must be finite and non-negative";
      return false;
    }
  }
  if (options.velocity_scaling > 1.0 || options.acceleration_scaling > 1.0) {
    error = "Velocity and acceleration scaling must be in (0,1]"; return false;
  }
  return true;
}
}  // namespace

Eigen::Vector3d BimanualCartesianPlanner::orientationError(
  const Eigen::Matrix3d & target, const Eigen::Matrix3d & current)
{
  Eigen::Quaterniond error(target * current.transpose());
  error.normalize();
  if (error.w() < 0.0) {error.coeffs() *= -1.0;}
  if (error.vec().norm() < 1.0e-12) {return 2.0 * error.vec();}
  const Eigen::AngleAxisd axis_angle(error);
  return axis_angle.axis() * axis_angle.angle();
}

Eigen::Isometry3d BimanualCartesianPlanner::interpolateRelativePose(
  const Eigen::Isometry3d & initial, const Eigen::Isometry3d & target, double progress)
{
  const double u = std::clamp(progress, 0.0, 1.0);
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = initial.translation() + u * (target.translation() - initial.translation());
  result.linear() = Eigen::Quaterniond(initial.rotation()).slerp(u, Eigen::Quaterniond(target.rotation())).normalized().toRotationMatrix();
  return result;
}

Eigen::Isometry3d BimanualCartesianPlanner::relativePoseInReference(
  const Eigen::Isometry3d & model_from_arm_a,
  const Eigen::Isometry3d & model_from_arm_b,
  const Eigen::Isometry3d & model_from_reference)
{
  const Eigen::Matrix3d reference_from_model = model_from_reference.rotation().transpose();
  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.translation() = reference_from_model *
    (model_from_arm_b.translation() - model_from_arm_a.translation());
  result.linear() = reference_from_model * model_from_arm_b.rotation() *
    model_from_arm_a.rotation().transpose() * model_from_reference.rotation();
  return result;
}

bool BimanualCartesianPlanner::sampleTrajectory(
  const trajectory_msgs::msg::JointTrajectory & trajectory, double elapsed,
  trajectory_msgs::msg::JointTrajectoryPoint & sample, std::string & message)
{
  if (!std::isfinite(elapsed) || !validMessage(trajectory, message)) {return false;}
  if (elapsed <= seconds(trajectory.points.front().time_from_start)) {sample = trajectory.points.front(); return true;}
  if (elapsed >= seconds(trajectory.points.back().time_from_start)) {sample = trajectory.points.back(); return true;}
  for (std::size_t i = 1; i < trajectory.points.size(); ++i) {
    const double finish = seconds(trajectory.points[i].time_from_start);
    if (elapsed > finish) {continue;}
    const double begin = seconds(trajectory.points[i - 1].time_from_start);
    const double span = finish - begin;
    interpolateSegment(trajectory.points[i - 1], trajectory.points[i], (elapsed - begin) / span, span, sample);
    sample.time_from_start = duration(elapsed);
    return true;
  }
  message = "No interpolation segment found";
  return false;
}

namespace
{
bool planCartesian(
  const planning_scene::PlanningSceneConstPtr & scene, const moveit::core::RobotState & start,
  const Arm & arm_a, const Arm & arm_b, const Pair & pair, const GoalSpec & goal,
  const Options & options, PlannedTrajectory & output, std::string & message,
  const BimanualCartesianPlanner::CancelCheck & canceled,
  const BimanualCartesianPlanner::Progress & progress, bool single_arm)
{
  output = PlannedTrajectory();
  if (!validOptions(goal, options, message)) {return false;}
  try {
    PlanningContext context(scene, start, arm_a, arm_b, pair, goal, options, canceled, progress, single_arm);
    if (context.stopped(message) || !context.initialize(message)) {return false;}
    const double distance = (goal.relative_target.translation() - context.relative0.translation()).norm();
    const double angle = BimanualCartesianPlanner::orientationError(goal.relative_target.rotation(), context.relative0.rotation()).norm();
    const double required = std::max(distance / goal.cartesian_step, angle / options.angular_step);
    if (!std::isfinite(required) || required > options.max_waypoints - 1) {
      message = "Requested Cartesian resolution exceeds the maximum waypoint count"; return false;
    }
    const int count = std::max(1, static_cast<int>(std::ceil(required)));
    std::vector<Waypoint> waypoints{{context.q0, 0.0}};
    for (int i = 1; i <= count; ++i) {
      const double s = static_cast<double>(i) / count;
      if (progress) {progress(0.1 + 0.65 * s, single_arm ? "solving_constrained_cartesian_ik" : "solving_constrained_bimanual_ik");}
      if (!context.appendSegment(waypoints, s, 0, message)) {return false;}
    }
    if (progress) {progress(0.8, "smoothing_and_validating_segments");}
    context.smooth(waypoints, message);
    if (context.stopped(message)) {return false;}
    if (progress) {progress(0.9, "time_parameterization_and_interpolation_validation");}
    PlannedTrajectory completed;
    if (!context.timeAndValidate(waypoints, completed, message)) {return false;}
    if (!context.validateTimed(completed, message)) {return false;}
    if (context.stopped(message)) {return false;}
    completed.initial_arm_a_pose = context.arm_a0;
    completed.reference_frame = context.reference_frame;
    completed.model_from_reference = context.model_from_reference;
    completed.initial_relative_pose = context.relative0;
    completed.relative_target = goal.relative_target;
    output = std::move(completed);
    message = single_arm ? "Valid single-arm Cartesian joint trajectory planned" :
      "Valid constrained bimanual Cartesian joint trajectory planned";
    return true;
  } catch (const std::exception & exception) {
    message = std::string("Cartesian planning failed: ") + exception.what();
    return false;
  }
}

bool validateCartesian(
  const planning_scene::PlanningSceneConstPtr & scene, const Arm & arm_a,
  const Arm & arm_b, const Pair & pair, const GoalSpec & goal, const Options & options,
  const PlannedTrajectory & plan, std::string & message,
  const BimanualCartesianPlanner::CancelCheck & canceled, bool single_arm)
{
  if (!validOptions(goal, options, message) || !plan.trajectory ||
    plan.trajectory->getWayPointCount() == 0)
  {message = "Cannot validate an empty or malformed stored plan: " + message; return false;}
  try {
    if (!scene || scene->getRobotModel().get() != plan.trajectory->getRobotModel().get()) {
      message = "Stored plan and current planning scene use different robot models"; return false;
    }
    // Retain planned arm starts, but use live non-commanded joints (grippers,
    // other arms, torso) when checking collisions against the updated scene.
    auto baseline = scene->getCurrentState();
    const auto * group = scene->getRobotModel()->getJointModelGroup(pair.group_name);
    if (!group) {message = "Stored combined planning group no longer exists"; return false;}
    for (const auto * joint : group->getActiveJointModels()) {
      for (const auto & name : joint->getVariableNames()) {
        baseline.setVariablePosition(name, plan.trajectory->getFirstWayPoint().getVariablePosition(name));
      }
    }
    baseline.update();
    const BimanualCartesianPlanner::Progress progress;
    PlanningContext context(scene, baseline, arm_a, arm_b,
      pair, goal, options, canceled, progress, single_arm);
    if (context.stopped(message) || !context.initialize(message)) {return false;}
    if (context.reference_frame != plan.reference_frame ||
      (context.model_from_reference.matrix() - plan.model_from_reference.matrix()).norm() > 1.0e-8 ||
      (context.arm_a0.matrix() - plan.initial_arm_a_pose.matrix()).norm() > 1.0e-8 ||
      (context.relative0.matrix() - plan.initial_relative_pose.matrix()).norm() > 1.0e-8 ||
      (goal.relative_target.matrix() - plan.relative_target.matrix()).norm() > 1.0e-8)
    {message = "Stored plan pose references no longer match its initial state or goal"; return false;}
    if (!context.validateTimed(plan, message)) {return false;}
    message = "Stored quintic trajectory is valid in the current planning scene";
    return true;
  } catch (const std::exception & exception) {
    message = std::string("Stored trajectory validation failed: ") + exception.what();
    return false;
  }
}

}  // namespace

bool BimanualCartesianPlanner::plan(
  const planning_scene::PlanningSceneConstPtr & scene, const moveit::core::RobotState & start,
  const Arm & arm_a, const Arm & arm_b, const Pair & pair, const GoalSpec & goal,
  const Options & options, PlannedTrajectory & output, std::string & message,
  const CancelCheck & canceled, const Progress & progress) const
{
  return planCartesian(scene, start, arm_a, arm_b, pair, goal, options, output, message, canceled, progress, false);
}

bool BimanualCartesianPlanner::validatePlan(
  const planning_scene::PlanningSceneConstPtr & scene, const Arm & arm_a, const Arm & arm_b,
  const Pair & pair, const GoalSpec & goal, const Options & options, const PlannedTrajectory & plan,
  std::string & message, const CancelCheck & canceled) const
{
  return validateCartesian(scene, arm_a, arm_b, pair, goal, options, plan, message, canceled, false);
}

bool SingleArmCartesianPlanner::plan(
  const planning_scene::PlanningSceneConstPtr & scene, const moveit::core::RobotState & start,
  const Arm & arm, const GoalSpec & goal, const Options & options, PlannedTrajectory & output,
  std::string & message, const CancelCheck & canceled, const Progress & progress) const
{
  Pair group;
  group.group_name = arm.group_name;
  return planCartesian(scene, start, Arm{}, arm, group, goal, options, output, message, canceled, progress, true);
}

bool SingleArmCartesianPlanner::validatePlan(
  const planning_scene::PlanningSceneConstPtr & scene, const Arm & arm, const GoalSpec & goal,
  const Options & options, const PlannedTrajectory & plan, std::string & message,
  const CancelCheck & canceled) const
{
  Pair group;
  group.group_name = arm.group_name;
  return validateCartesian(scene, Arm{}, arm, group, goal, options, plan, message, canceled, true);
}

}  // namespace sura_manipulation_actions::bimanual_cartesian
