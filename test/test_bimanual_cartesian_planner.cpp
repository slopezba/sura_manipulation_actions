#include "sura_manipulation_actions/bimanual_cartesian_planner.hpp"
#include "sura_manipulation_actions/cartesian_trajectory_timing.hpp"
#include "sura_manipulation_actions/single_arm_cartesian_planner.hpp"

#include <gtest/gtest.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace sura_manipulation_actions::bimanual_cartesian
{
namespace
{

struct SyntheticRobot
{
  moveit::core::RobotModelPtr model;
  planning_scene::PlanningScenePtr scene;
  Arm left, right;
  Pair pair;
  std::string spare_joint = "spare_slide";

  explicit SyntheticRobot(
    int left_dofs = 4, int right_dofs = 6, double left_yaw = 0.0, double right_yaw = 0.0)
  {
    std::ostringstream urdf;
    urdf << "<robot name='synthetic'><link name='world'/>"
         << "<link name='reference'/><joint name='reference_joint' type='fixed'>"
         << "<parent link='world'/><child link='reference'/>"
         << "<origin xyz='1.2 -0.7 0.3' rpy='0.4 -0.3 0.5'/></joint>";
    auto add_arm = [&](const std::string & name, int count, double x, double yaw) {
        Arm arm;
        arm.name = name;
        arm.group_name = name;
        arm.tip_frame = name + "_tip";
        urdf << "<link name='" << name << "_mount'/><joint name='" << name
             << "_mount_joint' type='fixed'><parent link='world'/><child link='" << name
             << "_mount'/><origin xyz='" << x << " 0 0' rpy='0 0 " << yaw << "'/></joint>";
        std::string parent = name + "_mount";
        for (int i = 0; i < count; ++i) {
          const std::string child = i + 1 == count ? arm.tip_frame : name + "_link_" + std::to_string(i);
          const std::string joint = name + "_joint_" + std::to_string(i);
          arm.joint_names.push_back(joint);
          urdf << "<link name='" << child << "'>";
          if (i + 1 == count) {
            urdf << "<collision><geometry><sphere radius='0.004'/></geometry></collision>";
          }
          urdf << "</link><joint name='" << joint << "' type='"
               << (i < 3 ? "prismatic" : "revolute") << "'><parent link='" << parent
               << "'/><child link='" << child << "'/><axis xyz='";
          if (i == 0 || i == 5) {urdf << "1 0 0";}
          else if (i == 1 || i == 4) {urdf << "0 1 0";}
          else {urdf << "0 0 1";}
          urdf << "'/><limit lower='-1' upper='1' effort='1' velocity='1'/></joint>";
          parent = child;
        }
        return arm;
      };
    left = add_arm("left", left_dofs, -0.3, left_yaw);
    right = add_arm("right", right_dofs, 0.3, right_yaw);
    // A non-commanded robot joint allows execution revalidation to exercise
    // current full-body collision state, independently of the planned arms.
    urdf << "<link name='spare_tool'><collision><geometry><sphere radius='0.004'/>"
         << "</geometry></collision></link><joint name='spare_slide' type='prismatic'>"
         << "<parent link='world'/><child link='spare_tool'/><origin xyz='2 0 0'/>"
         << "<axis xyz='1 0 0'/><limit lower='-2' upper='2' effort='1' velocity='1'/>"
         << "</joint></robot>";
    const auto urdf_model = urdf::parseURDF(urdf.str());
    if (!urdf_model) {throw std::runtime_error("Synthetic URDF could not be parsed");}
    auto semantic = std::make_shared<srdf::Model>();
    const std::string srdf =
      "<robot name='synthetic'><group name='left'><chain base_link='left_mount' tip_link='left_tip'/></group>"
      "<group name='right'><chain base_link='right_mount' tip_link='right_tip'/></group>"
      "<group name='both'><group name='left'/><group name='right'/></group></robot>";
    if (!semantic->initString(*urdf_model, srdf)) {throw std::runtime_error("Synthetic SRDF could not be parsed");}
    model = std::make_shared<moveit::core::RobotModel>(urdf_model, semantic);
    for (auto * joint : model->getActiveJointModels()) {
      const auto & name = joint->getVariableNames().front();
      auto bounds = joint->getVariableBounds(name);
      bounds.velocity_bounded_ = true;
      bounds.min_velocity_ = -1.0;
      bounds.max_velocity_ = 1.0;
      bounds.acceleration_bounded_ = true;
      bounds.min_acceleration_ = -2.0;
      bounds.max_acceleration_ = 2.0;
      joint->setVariableBounds(name, bounds);
    }
    scene = std::make_shared<planning_scene::PlanningScene>(model);
    scene->getCurrentStateNonConst().setToDefaultValues();
    for (const auto & name : model->getVariableNames()) {
      scene->getCurrentStateNonConst().setVariablePosition(name, 0.0);
    }
    scene->getCurrentStateNonConst().update();
    pair.arm_names = {left.name, right.name};
    pair.group_name = "both";
  }

  GoalSpec goal(double x = 0.02, bool swap = false) const
  {
    const auto & state = scene->getCurrentState();
    const auto & arm_a = swap ? right : left;
    const auto & arm_b = swap ? left : right;
    GoalSpec goal;
    goal.reference_frame = model->getModelFrame();
    goal.relative_target = BimanualCartesianPlanner::relativePoseInReference(
      state.getGlobalLinkTransform(arm_a.tip_frame), state.getGlobalLinkTransform(arm_b.tip_frame),
      Eigen::Isometry3d::Identity());
    goal.relative_target.translation().x() += x;
    goal.relative_position_tolerance = Eigen::Vector3d::Constant(0.002);
    goal.relative_orientation_tolerance = Eigen::Vector3d::Constant(0.005);
    goal.planning_time = 5.0;
    return goal;
  }

  static Options options()
  {
    Options options;
    options.optimize_trajectory = false;
    options.velocity_scaling = 0.5;
    options.acceleration_scaling = 0.5;
    options.recovery_seeds = 2;
    options.max_subdivision_depth = 3;
    return options;
  }

  void obstacle(double x)
  {
    moveit_msgs::msg::CollisionObject object;
    object.header.frame_id = model->getModelFrame();
    object.id = "new_obstacle";
    object.operation = object.ADD;
    shape_msgs::msg::SolidPrimitive box;
    box.type = box.BOX;
    box.dimensions = {0.004, 0.02, 0.02};
    object.primitives.push_back(box);
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.orientation.w = 1.0;
    object.primitive_poses.push_back(pose);
    ASSERT_TRUE(scene->processCollisionObjectMsg(object));
  }
};

trajectory_msgs::msg::JointTrajectory singleJointTrajectory()
{
  trajectory_msgs::msg::JointTrajectory msg;
  msg.joint_names = {"joint"};
  trajectory_msgs::msg::JointTrajectoryPoint a, b;
  a.positions = {0.0}; a.velocities = {0.0}; a.accelerations = {0.0};
  b.positions = {1.0}; b.velocities = {0.0}; b.accelerations = {0.0};
  b.time_from_start.sec = 2;
  msg.points = {a, b};
  return msg;
}

void restBetweenOriginalEndpoints(PlannedTrajectory & plan)
{
  // Planning may insert intermediate poses while repairing IK or timing. This
  // regression specifically exercises one rest-to-rest quintic between the
  // unchanged planned endpoints, independently of those adaptive subdivisions.
  const auto first = plan.joint_trajectory.points.front();
  const auto last = plan.joint_trajectory.points.back();
  plan.joint_trajectory.points = {first, last};
  plan.progress_values = {0.0, 1.0};
  for (std::size_t i = 0; i < plan.joint_trajectory.points.size(); ++i) {
    auto & point = plan.joint_trajectory.points[i];
    point.velocities.assign(point.positions.size(), 0.0);
    point.accelerations.assign(point.positions.size(), 0.0);
    point.time_from_start = timing::duration(2.0 * static_cast<double>(i));
  }
  auto endpoints = std::make_shared<robot_trajectory::RobotTrajectory>(
    plan.trajectory->getRobotModel(), plan.trajectory->getGroupName());
  endpoints->setRobotTrajectoryMsg(plan.trajectory->getFirstWayPoint(), plan.joint_trajectory);
  plan.trajectory = std::move(endpoints);
}

TEST(BimanualCartesianInterpolation, QuinticBoundaryDerivativesAreActualDerivatives)
{
  const auto msg = singleJointTrajectory();
  std::string error;
  trajectory_msgs::msg::JointTrajectoryPoint sample;
  ASSERT_TRUE(BimanualCartesianPlanner::sampleTrajectory(msg, 1.0, sample, error)) << error;
  EXPECT_NEAR(sample.positions[0], 0.5, 1.0e-12);
  EXPECT_NEAR(sample.velocities[0], 0.9375, 1.0e-12);
  EXPECT_NEAR(sample.accelerations[0], 0.0, 1.0e-12);
  ASSERT_TRUE(BimanualCartesianPlanner::sampleTrajectory(msg, 0.5, sample, error));
  EXPECT_NEAR(sample.positions[0], 0.103515625, 1.0e-12);
  EXPECT_NEAR(sample.accelerations[0], 1.40625, 1.0e-12);
}

TEST(BimanualCartesianInterpolation, PositionOnlyAndCubicInputsUseTheirBoundaryPolynomials)
{
  auto msg = singleJointTrajectory();
  for (auto & point : msg.points) {point.accelerations.clear();}
  std::string error;
  trajectory_msgs::msg::JointTrajectoryPoint sample;
  ASSERT_TRUE(BimanualCartesianPlanner::sampleTrajectory(msg, 0.5, sample, error));
  EXPECT_NEAR(sample.positions[0], 0.15625, 1.0e-12);
  for (auto & point : msg.points) {point.velocities.clear();}
  ASSERT_TRUE(BimanualCartesianPlanner::sampleTrajectory(msg, 0.5, sample, error));
  EXPECT_NEAR(sample.positions[0], 0.25, 1.0e-12);
  EXPECT_NEAR(sample.velocities[0], 0.5, 1.0e-12);
}

TEST(BimanualCartesianInterpolation, RejectsMalformedTimesAndDerivativeArrays)
{
  auto msg = singleJointTrajectory();
  std::string error;
  trajectory_msgs::msg::JointTrajectoryPoint sample;
  msg.points[1].time_from_start.sec = 0;
  EXPECT_FALSE(BimanualCartesianPlanner::sampleTrajectory(msg, 0.5, sample, error));
  msg = singleJointTrajectory();
  msg.points[1].accelerations.push_back(0.0);
  EXPECT_FALSE(BimanualCartesianPlanner::sampleTrajectory(msg, 0.5, sample, error));
}



TEST(BimanualCartesianPlanner, GenericDifferentDofArmsProduceOneTimedTrajectory)
{
  SyntheticRobot robot(4, 6);
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  const auto goal = robot.goal();
  const auto options = robot.options();
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, options, plan, error)) << error;
  ASSERT_TRUE(plan.trajectory);
  ASSERT_EQ(plan.joint_trajectory.joint_names.size(), 10U);
  ASSERT_GE(plan.joint_trajectory.points.size(), 2U);
  EXPECT_EQ(plan.progress_values.size(), plan.joint_trajectory.points.size());
  double previous = -1.0;
  bool has_motion_derivative = false;
  for (const auto & point : plan.joint_trajectory.points) {
    EXPECT_EQ(point.positions.size(), 10U);
    EXPECT_EQ(point.velocities.size(), 10U);
    EXPECT_EQ(point.accelerations.size(), 10U);
    const double time = point.time_from_start.sec + point.time_from_start.nanosec * 1.0e-9;
    EXPECT_GT(time, previous);
    previous = time;
    for (const auto velocity : point.velocities) {has_motion_derivative |= std::abs(velocity) > 1.0e-8;}
  }
  EXPECT_TRUE(has_motion_derivative);
  ASSERT_TRUE(planner.validatePlan(robot.scene, robot.left, robot.right, robot.pair,
    goal, options, plan, error)) << error;
}

TEST(BimanualCartesianPlanner, EqualWeightsDistributeMotionAcrossBothArms)
{
  SyntheticRobot robot(4, 4);
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  auto goal = robot.goal();
  goal.relative_position_tolerance.setZero();
  auto options = robot.options();
  options.weights.assign(8, 1.0);
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, options, plan, error)) << error;
  const auto & final = plan.trajectory->getLastWayPoint();
  const double left = final.getVariablePosition(robot.left.joint_names.front());
  const double right = final.getVariablePosition(robot.right.joint_names.front());
  EXPECT_LT(left, -0.005);
  EXPECT_GT(right, 0.005);
  EXPECT_NEAR(left, -right, options.position_epsilon);
  EXPECT_NEAR(right - left, 0.02, options.position_epsilon);
  EXPECT_EQ(plan.joint_trajectory.points.front().positions, (std::vector<double>(8, 0.0)));
  ASSERT_TRUE(planner.validatePlan(robot.scene, robot.left, robot.right, robot.pair,
    goal, options, plan, error)) << error;
}

TEST(BimanualCartesianPlanner, LargerWeightReducesMotionFromMeasuredInitialPosition)
{
  SyntheticRobot robot(4, 4);
  auto & start = robot.scene->getCurrentStateNonConst();
  const auto & left_joint = robot.left.joint_names.front();
  const auto & right_joint = robot.right.joint_names.front();
  start.setVariablePosition(left_joint, 0.05);
  start.setVariablePosition(right_joint, -0.03);
  start.update();
  auto goal = robot.goal();
  goal.relative_position_tolerance.setZero();
  auto options = robot.options();
  options.weights.assign(8, 0.0);
  const auto & names = robot.model->getJointModelGroup(robot.pair.group_name)->getVariableNames();
  const auto index = std::distance(names.begin(), std::find(names.begin(), names.end(), left_joint));
  ASSERT_LT(static_cast<std::size_t>(index), options.weights.size());
  BimanualCartesianPlanner planner;
  PlannedTrajectory unweighted, weighted, swapped;
  std::string error;
  ASSERT_TRUE(planner.plan(robot.scene, start, robot.left, robot.right, robot.pair,
    goal, options, unweighted, error)) << error;
  options.weights[static_cast<std::size_t>(index)] = 100.0;
  ASSERT_TRUE(planner.plan(robot.scene, start, robot.left, robot.right, robot.pair,
    goal, options, weighted, error)) << error;
  const auto & baseline = unweighted.trajectory->getLastWayPoint();
  const auto & final = weighted.trajectory->getLastWayPoint();
  EXPECT_LT(std::abs(final.getVariablePosition(left_joint) - 0.05),
    0.5 * std::abs(baseline.getVariablePosition(left_joint) - 0.05));
  EXPECT_GT(final.getVariablePosition(right_joint) + 0.03,
    baseline.getVariablePosition(right_joint) + 0.03);
  // Reverse the relative target while keeping weights on the same physical joints.
  auto reverse_goal = robot.goal(-0.02, true);
  reverse_goal.relative_position_tolerance.setZero();
  ASSERT_TRUE(planner.plan(robot.scene, start, robot.right, robot.left, robot.pair,
    reverse_goal, options, swapped, error)) << error;
  const auto & reverse_final = swapped.trajectory->getLastWayPoint();
  for (const auto & name : names) {
    EXPECT_NEAR(final.getVariablePosition(name), reverse_final.getVariablePosition(name),
      options.position_epsilon);
  }
  ASSERT_TRUE(planner.validatePlan(robot.scene, robot.right, robot.left, robot.pair,
    reverse_goal, options, swapped, error)) << error;
}

TEST(BimanualCartesianPlanner, OmittedAndZeroWeightsLeaveBothArmsFree)
{
  SyntheticRobot robot(4, 4);
  BimanualCartesianPlanner planner;
  PlannedTrajectory omitted, zero;
  std::string error;
  auto options = robot.options();
  const auto goal = robot.goal();
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, options, omitted, error)) << error;
  options.weights.assign(8, 0.0);
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, options, zero, error)) << error;
  const auto & final = zero.trajectory->getLastWayPoint();
  EXPECT_LT(final.getVariablePosition(robot.left.joint_names.front()), -0.005);
  EXPECT_GT(final.getVariablePosition(robot.right.joint_names.front()), 0.005);
  for (const auto & name : zero.joint_trajectory.joint_names) {
    EXPECT_NEAR(final.getVariablePosition(name),
      omitted.trajectory->getLastWayPoint().getVariablePosition(name), 1e-9);
  }
}

TEST(BimanualCartesianPlanner, RejectsInvalidWeightsInPlanningAndRevalidation)
{
  SyntheticRobot robot(4, 4);
  BimanualCartesianPlanner planner;
  PlannedTrajectory valid, invalid;
  std::string error;
  auto options = robot.options();
  const auto goal = robot.goal();
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, options, valid, error)) << error;
  const std::vector<std::vector<double>> invalid_weights{
    std::vector<double>(7, 0.0), std::vector<double>(9, 0.0),
    std::vector<double>(8, -0.1),
    std::vector<double>(8, std::numeric_limits<double>::quiet_NaN()),
    std::vector<double>(8, std::numeric_limits<double>::infinity())};
  for (const auto & weights : invalid_weights) {
    options.weights = weights;
    EXPECT_FALSE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
      robot.right, robot.pair, goal, options, invalid, error));
    EXPECT_FALSE(invalid.trajectory);
    EXPECT_NE(error.find("planning.weights"), std::string::npos);
    EXPECT_FALSE(planner.validatePlan(robot.scene, robot.left, robot.right, robot.pair,
      goal, options, valid, error));
    EXPECT_NE(error.find("planning.weights"), std::string::npos);
  }
}

TEST(BimanualCartesianPlanner, ZeroToleranceStraightPathAcceptsRestToRestQuintic)
{
  SyntheticRobot robot;
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  auto goal = robot.goal(0.02);
  goal.cartesian_step = 0.03;
  goal.relative_position_tolerance.setZero();
  goal.relative_orientation_tolerance.setZero();
  const auto options = robot.options();
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, options, plan, error)) << error;
  ASSERT_GE(plan.joint_trajectory.points.size(), 2U);
  restBetweenOriginalEndpoints(plan);
  ASSERT_TRUE(planner.validatePlan(robot.scene, robot.left, robot.right, robot.pair,
    goal, options, plan, error)) << error;
  trajectory_msgs::msg::JointTrajectoryPoint sample;
  ASSERT_TRUE(BimanualCartesianPlanner::sampleTrajectory(plan.joint_trajectory, 0.5, sample, error));
  const auto index = static_cast<std::size_t>(std::distance(plan.joint_trajectory.joint_names.begin(),
    std::find(plan.joint_trajectory.joint_names.begin(), plan.joint_trajectory.joint_names.end(),
    robot.right.joint_names.front())));
  ASSERT_LT(index, sample.positions.size());
  // At u=.25, quintic geometric progress is .103515625, far from linear time.
  EXPECT_NEAR(sample.positions[index], 0.01 * 0.103515625, options.position_epsilon);
  EXPECT_GT(std::abs(sample.positions[index] - 0.01 * 0.25), 0.001);

  // Projection must retain lateral tolerances: introduce a y excursion with
  // unchanged endpoint positions and reject its between-waypoint curve.
  const auto y = static_cast<std::size_t>(std::distance(plan.joint_trajectory.joint_names.begin(),
    std::find(plan.joint_trajectory.joint_names.begin(), plan.joint_trajectory.joint_names.end(),
    robot.right.joint_names[1])));
  ASSERT_LT(y, sample.positions.size());
  plan.joint_trajectory.points.front().velocities[y] = 0.01;
  plan.joint_trajectory.points.back().velocities[y] = -0.01;
  EXPECT_FALSE(planner.validatePlan(robot.scene, robot.left, robot.right, robot.pair,
    goal, options, plan, error));
  EXPECT_NE(error.find("Cartesian tolerances"), std::string::npos);
}

TEST(BimanualCartesianPlanner, ZeroTolerancePureOrientationAcceptsRestToRestQuintic)
{
  SyntheticRobot robot;
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  auto goal = robot.goal(0.0);
  goal.relative_target.linear() = Eigen::AngleAxisd(0.04, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  goal.relative_position_tolerance.setZero();
  goal.relative_orientation_tolerance.setZero();
  const auto options = robot.options();
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, options, plan, error)) << error;
  ASSERT_GE(plan.joint_trajectory.points.size(), 2U);
  restBetweenOriginalEndpoints(plan);
  ASSERT_TRUE(planner.validatePlan(robot.scene, robot.left, robot.right, robot.pair,
    goal, options, plan, error)) << error;
}

TEST(BimanualCartesianPlanner, MoveItJacobiansSupportDifferentlyRotatedArmMounts)
{
  SyntheticRobot robot(4, 6, 0.6, -0.8);
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  auto goal = robot.goal(0.02);
  goal.relative_position_tolerance = Eigen::Vector3d::Constant(0.0001);
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, robot.options(), plan, error)) << error;
  ASSERT_TRUE(planner.validatePlan(robot.scene, robot.left, robot.right, robot.pair,
    goal, robot.options(), plan, error)) << error;
}

TEST(BimanualCartesianPlanner, RelativePoseUsesReferenceAxesAndSpatialRotationDifference)
{
  Eigen::Isometry3d arm_a = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d arm_b = Eigen::Isometry3d::Identity();
  Eigen::Isometry3d reference = Eigen::Isometry3d::Identity();
  arm_a.translation() = Eigen::Vector3d(0.2, -0.4, 0.1);
  arm_b.translation() = Eigen::Vector3d(0.5, 0.1, -0.2);
  arm_a.linear() = Eigen::AngleAxisd(0.8, Eigen::Vector3d::UnitY()).toRotationMatrix();
  arm_b.linear() = Eigen::AngleAxisd(-0.5, Eigen::Vector3d::UnitX()).toRotationMatrix();
  reference.translation() = Eigen::Vector3d(4.0, 2.0, 1.0);
  reference.linear() = Eigen::AngleAxisd(0.6, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Isometry3d arm_a_in_reference = reference.inverse() * arm_a;
  const Eigen::Isometry3d arm_b_in_reference = reference.inverse() * arm_b;
  const auto relative = BimanualCartesianPlanner::relativePoseInReference(arm_a, arm_b, reference);
  EXPECT_TRUE(relative.translation().isApprox(
    arm_b_in_reference.translation() - arm_a_in_reference.translation(), 1e-12));
  EXPECT_TRUE(relative.rotation().isApprox(
    arm_b_in_reference.rotation() * arm_a_in_reference.rotation().transpose(), 1e-12));
  EXPECT_GT((relative.translation() - (arm_a.inverse() * arm_b).translation()).norm(), 0.1);
  const auto swapped = BimanualCartesianPlanner::relativePoseInReference(arm_b, arm_a, reference);
  EXPECT_TRUE(swapped.translation().isApprox(-relative.translation(), 1e-12));
  EXPECT_TRUE(swapped.rotation().isApprox(relative.rotation().transpose(), 1e-12));
}

TEST(BimanualCartesianPlanner, RotatedReferenceAxesWorkForEitherArmOrder)
{
  SyntheticRobot robot(4, 6, 0.6, -0.8);
  BimanualCartesianPlanner planner;
  const auto & start = robot.scene->getCurrentState();
  const auto & reference = start.getGlobalLinkTransform("reference");
  for (bool swap : {false, true}) {
    const auto & arm_a = swap ? robot.right : robot.left;
    const auto & arm_b = swap ? robot.left : robot.right;
    auto goal = robot.goal(0.0, swap);
    goal.reference_frame = "reference";
    goal.relative_target = BimanualCartesianPlanner::relativePoseInReference(
      start.getGlobalLinkTransform(arm_a.tip_frame), start.getGlobalLinkTransform(arm_b.tip_frame), reference);
    goal.relative_target.translation().x() += 0.02;
    goal.relative_position_tolerance = Eigen::Vector3d::Constant(0.0001);
    PlannedTrajectory plan;
    std::string error;
    ASSERT_TRUE(planner.plan(robot.scene, start, arm_a, arm_b, robot.pair,
      goal, robot.options(), plan, error)) << error;
    EXPECT_EQ(plan.reference_frame, "reference");
    EXPECT_TRUE(plan.model_from_reference.matrix().isApprox(reference.matrix(), 1e-12));
    const auto & final = plan.trajectory->getLastWayPoint();
    const auto achieved = BimanualCartesianPlanner::relativePoseInReference(
      final.getGlobalLinkTransform(arm_a.tip_frame), final.getGlobalLinkTransform(arm_b.tip_frame), reference);
    EXPECT_LE((achieved.translation() - goal.relative_target.translation()).cwiseAbs().maxCoeff(), 0.000101);
    ASSERT_TRUE(planner.validatePlan(robot.scene, arm_a, arm_b, robot.pair,
      goal, robot.options(), plan, error)) << error;
    plan.model_from_reference.translation().x() += 0.01;
    EXPECT_FALSE(planner.validatePlan(robot.scene, arm_a, arm_b, robot.pair,
      goal, robot.options(), plan, error));
  }
}

TEST(BimanualCartesianPlanner, ReferenceMustBeIndependentOfCommandedArms)
{
  SyntheticRobot robot;
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  auto goal = robot.goal();
  for (const auto & frame : {robot.left.tip_frame, std::string("unknown_base")}) {
    goal.reference_frame = frame;
    EXPECT_FALSE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
      robot.right, robot.pair, goal, robot.options(), plan, error));
    EXPECT_NE(error.find("reference frame"), std::string::npos);
  }
}

TEST(SingleArmCartesianPlanner, PlansEitherArmWithVariableJointCountsAndKeepsOtherArmFixed)
{
  SyntheticRobot robot(4, 6, 0.6, -0.8);
  const auto & start = robot.scene->getCurrentState();
  for (const auto & arm : {robot.left, robot.right}) {
    GoalSpec goal;
    goal.reference_frame = "reference";
    const Eigen::Isometry3d base = start.getGlobalLinkTransform("reference");
    goal.relative_target = base.inverse() * start.getGlobalLinkTransform(arm.tip_frame);
    goal.relative_target.translation().x() += 0.01;
    goal.relative_position_tolerance = Eigen::Vector3d::Constant(0.0001);
    goal.relative_orientation_tolerance = Eigen::Vector3d::Constant(0.0001);
    PlannedTrajectory plan;
    std::string error;
    SingleArmCartesianPlanner planner;
    ASSERT_TRUE(planner.plan(robot.scene, start, arm, goal, robot.options(), plan, error)) << error;
    EXPECT_EQ(plan.joint_trajectory.joint_names.size(), arm.joint_names.size());
    const auto & other = arm.name == robot.left.name ? robot.right : robot.left;
    for (std::size_t i = 0; i < plan.trajectory->getWayPointCount(); ++i) {
      for (const auto & name : other.joint_names) {
        EXPECT_DOUBLE_EQ(plan.trajectory->getWayPoint(i).getVariablePosition(name), start.getVariablePosition(name));
      }
    }
    const Eigen::Isometry3d achieved = base.inverse() * plan.trajectory->getLastWayPoint().getGlobalLinkTransform(arm.tip_frame);
    EXPECT_LE((achieved.translation() - goal.relative_target.translation()).cwiseAbs().maxCoeff(), 0.000101);
    ASSERT_TRUE(planner.validatePlan(robot.scene, arm, goal, robot.options(), plan, error)) << error;
  }
}

TEST(SingleArmCartesianPlanner, RejectsCollisionBetweenCartesianWaypoints)
{
  SyntheticRobot robot;
  robot.obstacle(0.31);
  GoalSpec goal;
  goal.reference_frame = "world";
  goal.relative_target = robot.scene->getCurrentState().getGlobalLinkTransform(robot.right.tip_frame);
  goal.relative_target.translation().x() += 0.02;
  goal.cartesian_step = 0.03;
  goal.relative_position_tolerance = Eigen::Vector3d::Constant(0.00001);
  goal.relative_orientation_tolerance = Eigen::Vector3d::Constant(0.00001);
  auto options = robot.options();
  options.recovery_seeds = options.max_subdivision_depth = 0;
  PlannedTrajectory plan;
  std::string error;
  EXPECT_FALSE(SingleArmCartesianPlanner().plan(robot.scene, robot.scene->getCurrentState(),
    robot.right, goal, options, plan, error));
  EXPECT_FALSE(plan.trajectory);
}

TEST(BimanualCartesianPlanner, MovingArmAWithNoncommutingOrientationsUsesSpatialRelativeJacobian)
{
  SyntheticRobot robot(6, 6, 0.6, -0.8);
  auto & start = robot.scene->getCurrentStateNonConst();
  start.setVariablePosition(robot.left.joint_names[3], 0.2);
  start.setVariablePosition(robot.left.joint_names[4], 0.15);
  start.setVariablePosition(robot.left.joint_names[5], 0.1);
  start.setVariablePosition(robot.right.joint_names[3], -0.1);
  start.setVariablePosition(robot.right.joint_names[4], -0.12);
  start.setVariablePosition(robot.right.joint_names[5], 0.08);
  start.update();
  auto goal = robot.goal(0.0);
  const Eigen::Matrix3d arm_a_target = start.getGlobalLinkTransform(robot.left.tip_frame).rotation() *
    Eigen::AngleAxisd(-0.02, Eigen::Vector3d::UnitX()).toRotationMatrix();
  goal.relative_target.linear() = start.getGlobalLinkTransform(robot.right.tip_frame).rotation() *
    arm_a_target.transpose();
  goal.relative_orientation_tolerance = Eigen::Vector3d::Constant(0.0001);
  auto options = robot.options();
  for (const auto & name : robot.right.joint_names) {
    const double q = start.getVariablePosition(name);
    options.tightened_limits[name] = {q, q};
  }
  for (std::size_t i = 0; i + 1 < robot.left.joint_names.size(); ++i) {
    const auto & name = robot.left.joint_names[i];
    const double q = start.getVariablePosition(name);
    options.tightened_limits[name] = {q, q};
  }
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  ASSERT_TRUE(planner.plan(robot.scene, start, robot.left, robot.right, robot.pair,
    goal, options, plan, error)) << error;
  EXPECT_NEAR(plan.trajectory->getLastWayPoint().getVariablePosition(robot.left.joint_names[5]),
    0.08, 0.0002);
  ASSERT_TRUE(planner.validatePlan(robot.scene, robot.left, robot.right, robot.pair,
    goal, options, plan, error)) << error;
}

TEST(BimanualCartesianTiming, StationaryTrajectoryGetsHoldAndZeroDerivatives)
{
  SyntheticRobot robot;
  robot_trajectory::RobotTrajectory trajectory(robot.model, robot.pair.group_name);
  auto state = robot.scene->getCurrentState();
  state.setVariableVelocity(robot.right.joint_names.front(), 0.2);
  state.setVariableAcceleration(robot.right.joint_names.front(), 0.3);
  trajectory.addSuffixWayPoint(state, 0.0);
  trajectory.addSuffixWayPoint(state, 0.0);
  std::vector<std::string> joints = robot.left.joint_names;
  joints.insert(joints.end(), robot.right.joint_names.begin(), robot.right.joint_names.end());
  std::string error;
  ASSERT_TRUE(timing::parameterize(trajectory, joints, 0.5, 0.5, error)) << error;
  EXPECT_DOUBLE_EQ(trajectory.getWayPointDurationFromPrevious(0), 0.0);
  EXPECT_DOUBLE_EQ(trajectory.getWayPointDurationFromPrevious(1), 0.1);
  for (std::size_t i = 0; i < trajectory.getWayPointCount(); ++i) {
    for (const auto & name : joints) {
      EXPECT_DOUBLE_EQ(trajectory.getWayPoint(i).getVariableVelocity(name), 0.0);
      EXPECT_DOUBLE_EQ(trajectory.getWayPoint(i).getVariableAcceleration(name), 0.0);
    }
  }
}

TEST(BimanualCartesianPlanner, ArmAAndArmBCanBeSwapped)
{
  SyntheticRobot robot(4, 6);
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  const auto goal = robot.goal(0.01, true);
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.right,
    robot.left, robot.pair, goal, robot.options(), plan, error)) << error;
  EXPECT_EQ(plan.joint_trajectory.joint_names.size(), 10U);
  ASSERT_TRUE(planner.validatePlan(robot.scene, robot.right, robot.left, robot.pair,
    goal, robot.options(), plan, error)) << error;
}

TEST(BimanualCartesianPlanner, EitherArmCanSupplyMotionWhenTheOtherIsAtItsLimits)
{
  SyntheticRobot robot;
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  const auto goal = robot.goal(0.01);
  auto options = robot.options();
  options.tightened_limits[robot.right.joint_names[0]] = {0.0, 0.0};
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, options, plan, error)) << error;
  EXPECT_LT(plan.trajectory->getLastWayPoint().getVariablePosition(robot.left.joint_names[0]), -0.005);
  EXPECT_DOUBLE_EQ(plan.trajectory->getLastWayPoint().getVariablePosition(robot.right.joint_names[0]), 0.0);
}

TEST(BimanualCartesianPlanner, ArmACanRotateWhenArmBOrientationIsAtItsLimits)
{
  SyntheticRobot robot(6, 6);
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  auto goal = robot.goal(0.0);
  goal.relative_target.linear() = Eigen::AngleAxisd(0.02, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  auto options = robot.options();
  for (std::size_t i = 3; i < robot.right.joint_names.size(); ++i) {
    options.tightened_limits[robot.right.joint_names[i]] = {0.0, 0.0};
  }
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, options, plan, error)) << error;
  const auto & final = plan.trajectory->getLastWayPoint();
  EXPECT_LT(final.getVariablePosition(robot.left.joint_names[3]), -0.01);
  ASSERT_TRUE(planner.validatePlan(robot.scene, robot.left, robot.right, robot.pair,
    goal, options, plan, error)) << error;
}

TEST(BimanualCartesianPlanner, RejectsUnreachableTargetWithoutReturningPartialPlan)
{
  SyntheticRobot robot;
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  auto options = robot.options();
  options.tightened_limits[robot.right.joint_names[0]] = {-0.001, 0.001};
  options.tightened_limits[robot.left.joint_names[0]] = {-0.001, 0.001};
  options.recovery_seeds = 0;
  options.max_subdivision_depth = 0;
  auto goal = robot.goal(0.02);
  goal.planning_time = 0.2;
  EXPECT_FALSE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, options, plan, error));
  EXPECT_FALSE(plan.trajectory);
  EXPECT_TRUE(plan.joint_trajectory.points.empty());
  EXPECT_FALSE(error.empty());
}

TEST(BimanualCartesianPlanner, RejectsInvalidGroupsBoundsAndZeroResolution)
{
  SyntheticRobot robot;
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  auto options = robot.options();
  auto arm = robot.right;
  arm.group_name = "unknown";
  EXPECT_FALSE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    arm, robot.pair, robot.goal(), options, plan, error));
  options.tightened_limits[robot.right.joint_names[0]] = {0.1, 0.2};
  EXPECT_FALSE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, robot.goal(), options, plan, error));
  options = robot.options();
  options.validation_joint_step = 0.0;
  EXPECT_FALSE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, robot.goal(), options, plan, error));
}

TEST(BimanualCartesianPlanner, MissingAccelerationLimitsFailRatherThanUseInventedLimits)
{
  SyntheticRobot robot;
  const auto & name = robot.right.joint_names[0];
  auto * joint = robot.model->getJointModel(name);
  auto bounds = joint->getVariableBounds(name);
  bounds.acceleration_bounded_ = false;
  joint->setVariableBounds(name, bounds);
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  EXPECT_FALSE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, robot.goal(), robot.options(), plan, error));
  EXPECT_NE(error.find("velocity/acceleration"), std::string::npos);
}

TEST(BimanualCartesianPlanner, PassiveJointInsideCommandedArmIsRejected)
{
  SyntheticRobot robot;
  robot.model->getJointModel(robot.right.joint_names[0])->setPassive(true);
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  EXPECT_FALSE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, robot.goal(), robot.options(), plan, error));
  EXPECT_NE(error.find("Passive joints"), std::string::npos);
  EXPECT_FALSE(plan.trajectory);
}

TEST(BimanualCartesianPlanner, CancellationDoesNotProduceAValidPlan)
{
  SyntheticRobot robot;
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  EXPECT_FALSE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, robot.goal(), robot.options(), plan, error, [] {return true;}));
  EXPECT_NE(error.find("canceled"), std::string::npos);
  EXPECT_FALSE(plan.trajectory);
}

TEST(BimanualCartesianPlanner, RevalidationDetectsObstacleBetweenOriginalWaypoints)
{
  SyntheticRobot robot;
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  auto goal = robot.goal(0.04);
  goal.cartesian_step = 0.04;
  auto options = robot.options();
  // A large tolerance separates the interpolation/collision check from nominal
  // progress: both original endpoints are clear of the new obstacle.
  goal.relative_position_tolerance = Eigen::Vector3d::Constant(0.005);
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, options, plan, error)) << error;
  const double end_x = plan.trajectory->getLastWayPoint().getGlobalLinkTransform(robot.right.tip_frame).translation().x();
  robot.obstacle(0.5 * (0.3 + end_x));
  EXPECT_FALSE(robot.scene->isStateColliding(plan.trajectory->getFirstWayPoint()));
  EXPECT_FALSE(robot.scene->isStateColliding(plan.trajectory->getLastWayPoint()));
  EXPECT_FALSE(planner.validatePlan(robot.scene, robot.left, robot.right, robot.pair,
    goal, options, plan, error));
  EXPECT_NE(error.find("planning scene"), std::string::npos);
}

TEST(BimanualCartesianPlanner, RevalidationUsesLiveNonCommandedRobotJoints)
{
  SyntheticRobot robot;
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  const auto goal = robot.goal(0.02);
  const auto options = robot.options();
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, options, plan, error)) << error;
  robot.scene->getCurrentStateNonConst().setVariablePosition(robot.spare_joint, -1.69);
  robot.scene->getCurrentStateNonConst().update();
  EXPECT_FALSE(planner.validatePlan(robot.scene, robot.left, robot.right, robot.pair,
    goal, options, plan, error));
  EXPECT_NE(error.find("planning scene"), std::string::npos);
}

TEST(BimanualCartesianPlanner, RevalidationChecksQuinticAccelerationBetweenPoints)
{
  SyntheticRobot robot;
  BimanualCartesianPlanner planner;
  PlannedTrajectory plan;
  std::string error;
  const auto goal = robot.goal(0.02);
  const auto options = robot.options();
  ASSERT_TRUE(planner.plan(robot.scene, robot.scene->getCurrentState(), robot.left,
    robot.right, robot.pair, goal, options, plan, error)) << error;
  for (auto & point : plan.joint_trajectory.points) {
    const double old = point.time_from_start.sec + point.time_from_start.nanosec * 1.0e-9;
    const auto ns = static_cast<int64_t>(old * 0.01 * 1.0e9);
    point.time_from_start.sec = static_cast<int32_t>(ns / 1000000000LL);
    point.time_from_start.nanosec = static_cast<uint32_t>(ns % 1000000000LL);
    for (auto & value : point.velocities) {value *= 100.0;}
    for (auto & value : point.accelerations) {value *= 10000.0;}
  }
  EXPECT_FALSE(planner.validatePlan(robot.scene, robot.left, robot.right, robot.pair,
    goal, options, plan, error));
  EXPECT_NE(error.find("bounds"), std::string::npos);
}

}  // namespace
}  // namespace sura_manipulation_actions::bimanual_cartesian
