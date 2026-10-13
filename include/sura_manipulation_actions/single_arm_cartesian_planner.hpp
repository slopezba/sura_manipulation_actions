#pragma once

#include "sura_manipulation_actions/bimanual_cartesian_planner.hpp"

namespace sura_manipulation_actions::bimanual_cartesian
{

// Single-arm entry point to the shared constrained Cartesian solver. In this
// mode GoalSpec::relative_target is an absolute tip pose in reference_frame.
// Only the selected arm's joints participate.
class SingleArmCartesianPlanner
{
public:
  using CancelCheck = BimanualCartesianPlanner::CancelCheck;
  using Progress = BimanualCartesianPlanner::Progress;
  bool plan(
    const planning_scene::PlanningSceneConstPtr & scene,
    const moveit::core::RobotState & start, const Arm & arm,
    const GoalSpec & goal, const Options & options, PlannedTrajectory & output,
    std::string & message, const CancelCheck & canceled = {},
    const Progress & progress = {}) const;
  bool validatePlan(
    const planning_scene::PlanningSceneConstPtr & scene, const Arm & arm,
    const GoalSpec & goal, const Options & options, const PlannedTrajectory & plan,
    std::string & message, const CancelCheck & canceled = {}) const;
};

}  // namespace sura_manipulation_actions::bimanual_cartesian
