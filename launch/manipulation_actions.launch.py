import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_namespace = LaunchConfiguration("robot_namespace")
    moveit_namespace = LaunchConfiguration("moveit_namespace")
    robot_description_package = LaunchConfiguration("robot_description_package")
    robot_description_file = LaunchConfiguration("robot_description_file")
    robot_description_semantic_package = LaunchConfiguration("robot_description_semantic_package")
    robot_description_semantic_file = LaunchConfiguration("robot_description_semantic_file")
    xacro_arguments = LaunchConfiguration("xacro_arguments")
    use_sim_time = LaunchConfiguration("use_sim_time")
    default_planning_time = LaunchConfiguration("default_planning_time")
    default_goal_tolerance = LaunchConfiguration("default_goal_tolerance")
    controller_wait_timeout = LaunchConfiguration("controller_wait_timeout")
    execution_feedback_rate = LaunchConfiguration("execution_feedback_rate")
    execution_backend = LaunchConfiguration("execution_backend")
    left_task_action = LaunchConfiguration("left_task_action")
    right_task_action = LaunchConfiguration("right_task_action")
    current_state_timeout = LaunchConfiguration("current_state_timeout")
    planning_scene_publish_rate = LaunchConfiguration("planning_scene_publish_rate")
    predefined_poses_file = LaunchConfiguration("predefined_poses_file")
    use_rviz = LaunchConfiguration("use_rviz")

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    PathJoinSubstitution(
                        [FindPackageShare(robot_description_package), robot_description_file]
                    ),
                    " ",
                    xacro_arguments,
                ]
            ),
            value_type=str,
        )
    }

    robot_description_semantic = {
        "robot_description_semantic": ParameterValue(
            Command(
                [
                    PathJoinSubstitution([FindExecutable(name="xacro")]),
                    " ",
                    PathJoinSubstitution(
                        [
                            FindPackageShare(robot_description_semantic_package),
                            robot_description_semantic_file,
                        ]
                    ),
                    " ",
                    "robot_name:=",
                    robot_namespace,
                ]
            ),
            value_type=str,
        )
    }

    cirtesub_share = get_package_share_directory("cirtesub_description")
    manipulation_share = get_package_share_directory("sura_manipulation_actions")
    planning_pipelines = os.path.join(cirtesub_share, "moveit2", "planning_pipelines.yaml")
    ompl_planning = os.path.join(cirtesub_share, "moveit2", "ompl_planning.yaml")
    joint_limits = os.path.join(cirtesub_share, "moveit2", "joint_limits.yaml")
    kinematics = os.path.join(cirtesub_share, "moveit2", "kinematics.yaml")
    predefined_poses = os.path.join(
        manipulation_share,
        "config",
        "predefined_poses.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="cirtesub"),
            DeclareLaunchArgument("moveit_namespace", default_value="moveit"),
            DeclareLaunchArgument(
                "robot_description_package",
                default_value="cirtesub_description",
            ),
            DeclareLaunchArgument(
                "robot_description_file",
                default_value=os.path.join("urdf", "cirtesub_dual_alpha.urdf.xacro"),
            ),
            DeclareLaunchArgument(
                "robot_description_semantic_package",
                default_value="cirtesub_description",
            ),
            DeclareLaunchArgument(
                "robot_description_semantic_file",
                default_value=os.path.join("moveit2", "cirtesub_dual_alpha.srdf.xacro"),
            ),
            DeclareLaunchArgument(
                "xacro_arguments",
                default_value=(
                    "robot_name:=cirtesub environment:=sim arms:=dual use_sim:=true "
                    "alpha_desired_joint_states_topic:=/cirtesub/alpha/desired_joint_states "
                    "alpha_joint_states_topic:=/cirtesub/stonefish/alpha/joint_states "
                    "alpha_normalized_joint_states_topic:=/cirtesub/alpha/joint_states"
                ),
            ),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("default_planning_time", default_value="5.0"),
            DeclareLaunchArgument("default_goal_tolerance", default_value="0.02"),
            DeclareLaunchArgument("controller_wait_timeout", default_value="5.0"),
            DeclareLaunchArgument("execution_feedback_rate", default_value="10.0"),
            DeclareLaunchArgument("execution_backend", default_value="controller"),
            DeclareLaunchArgument(
                "left_task_action",
                default_value=(
                    "/cirtesub/controller/task_priority/tasks/"
                    "left_joint_trajectory/follow_joint_trajectory"
                ),
            ),
            DeclareLaunchArgument(
                "right_task_action",
                default_value=(
                    "/cirtesub/controller/task_priority/tasks/"
                    "right_joint_trajectory/follow_joint_trajectory"
                ),
            ),
            DeclareLaunchArgument("current_state_timeout", default_value="2.0"),
            DeclareLaunchArgument("planning_scene_publish_rate", default_value="10.0"),
            DeclareLaunchArgument("predefined_poses_file", default_value=predefined_poses),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            LifecycleNode(
                package="sura_manipulation_actions",
                executable="plan_joint_trajectory_action_server",
                name="plan_joint_trajectory_lifecycle_action_server",
                namespace="",
                output="screen",
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    planning_pipelines,
                    ompl_planning,
                    joint_limits,
                    kinematics,
                    {
                        "robot_namespace": robot_namespace,
                        "moveit_namespace": moveit_namespace,
                        "use_sim_time": use_sim_time,
                        "default_planning_time": ParameterValue(
                            default_planning_time,
                            value_type=float,
                        ),
                        "default_goal_tolerance": ParameterValue(
                            default_goal_tolerance,
                            value_type=float,
                        ),
                        "controller_wait_timeout": ParameterValue(
                            controller_wait_timeout,
                            value_type=float,
                        ),
                        "execution_feedback_rate": ParameterValue(
                            execution_feedback_rate,
                            value_type=float,
                        ),
                        "execution_backend": execution_backend,
                        "left_task_action": left_task_action,
                        "right_task_action": right_task_action,
                        "current_state_timeout": ParameterValue(
                            current_state_timeout,
                            value_type=float,
                        ),
                        "planning_scene_publish_rate": ParameterValue(
                            planning_scene_publish_rate,
                            value_type=float,
                        ),
                        "predefined_poses_file": predefined_poses_file,
                    },
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_manipulation",
                output="screen",
                arguments=[
                    "-d",
                    os.path.join(manipulation_share, "rviz", "manipulation_planning.rviz"),
                ],
                parameters=[
                    robot_description,
                    robot_description_semantic,
                    {"use_sim_time": use_sim_time},
                ],
                condition=IfCondition(use_rviz),
            ),
        ]
    )
