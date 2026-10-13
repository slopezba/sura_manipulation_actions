"""Relative two-arm planning; this launch never starts controllers or tasks."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def _render(value, robot_namespace):
    if isinstance(value, str):
        if not robot_namespace:
            value = value.replace("${robot_namespace}/", "")
        value = value.replace("${robot_namespace}", robot_namespace)
        while "//" in value:
            value = value.replace("//", "/")
        return value
    if isinstance(value, dict):
        return {key: _render(item, robot_namespace) for key, item in value.items()}
    if isinstance(value, list):
        return [_render(item, robot_namespace) for item in value]
    return value


def _setup(context):
    def argument(name):
        return LaunchConfiguration(name).perform(context)

    namespace = argument("robot_namespace").strip("/")
    with open(argument("planner_config_file"), encoding="utf-8") as stream:
        config = yaml.safe_load(stream)
    if (not isinstance(config, dict) or not isinstance(config.get("/**"), dict)
            or not isinstance(config["/**"].get("ros__parameters"), dict)):
        raise ValueError("planner_config_file must contain /**: ros__parameters:")
    config = _render(config["/**"]["ros__parameters"], namespace)
    config.update(
        robot_namespace=namespace,
        moveit_namespace=argument("moveit_namespace"),
        execution_backend=argument("execution_backend"),
        use_sim_time=argument("use_sim_time").lower() == "true",
    )
    for launch_name, parameter_name in (
        ("fixed_frame", "visualization.fixed_frame"),
        ("relative_reference_frame", "relative_reference_frame"),
        ("joint_states_topic", "joint_states_topic"),
        ("planning_scene_input_topic", "planning_scene_input_topic"),
    ):
        if argument(launch_name):
            config[parameter_name] = argument(launch_name)

    description_path = os.path.join(
        get_package_share_directory(argument("robot_description_package")),
        argument("robot_description_file"),
    )
    semantic_path = os.path.join(
        get_package_share_directory(argument("robot_description_semantic_package")),
        argument("robot_description_semantic_file"),
    )
    robot = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", description_path, " ",
                     _render(argument("xacro_arguments"), namespace)]), value_type=str),
        "robot_description_semantic": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", semantic_path, " ",
                     _render(argument("srdf_xacro_arguments"), namespace)]), value_type=str),
    }
    moveit_share = get_package_share_directory(argument("moveit_config_package"))
    parameters = [
        robot,
        os.path.join(moveit_share, argument("joint_limits_file")),
        os.path.join(moveit_share, argument("kinematics_file")),
        config,
    ]
    actions = [LifecycleNode(
        package="sura_manipulation_actions",
        executable="plan_bimanual_cartesian_trajectory_action_server",
        name="plan_bimanual_cartesian_trajectory_lifecycle_action_server",
        namespace="", output="screen", parameters=parameters,
    )]
    if argument("launch_marker_visualization").lower() == "true":
        actions.insert(0, Node(
            package="sura_manipulation_actions",
            executable="manipulation_marker_visualization.py",
            namespace=namespace, output="screen",
            parameters=[{"use_sim_time": argument("use_sim_time").lower() == "true"}],
        ))
    if argument("use_rviz").lower() == "true":
        actions.append(Node(
            package="rviz2", executable="rviz2", name="rviz2_bimanual_cartesian",
            output="screen", arguments=["-d", argument("rviz_config_file")],
            parameters=parameters,
        ))
    return actions


def generate_launch_description():
    share = get_package_share_directory("sura_manipulation_actions")
    defaults = {
        "robot_namespace": "cirtesub",
        "moveit_namespace": "moveit",
        "execution_backend": "controller",
        "planner_config_file": os.path.join(share, "config", "bimanual_cartesian_planner.yaml"),
        "robot_description_package": "cirtesub_description",
        "robot_description_file": "urdf/cirtesub_dual_alpha.urdf.xacro",
        "robot_description_semantic_package": "cirtesub_description",
        "robot_description_semantic_file": "moveit2/cirtesub_dual_alpha.srdf.xacro",
        "xacro_arguments": (
            "robot_name:=${robot_namespace} environment:=sim arms:=dual use_sim:=true "
            "alpha_desired_joint_states_topic:=/${robot_namespace}/alpha/desired_joint_states "
            "alpha_joint_states_topic:=/${robot_namespace}/stonefish/alpha/joint_states "
            "alpha_normalized_joint_states_topic:=/${robot_namespace}/alpha/joint_states"
        ),
        "srdf_xacro_arguments": "robot_name:=${robot_namespace}",
        "moveit_config_package": "cirtesub_description",
        "joint_limits_file": "moveit2/joint_limits.yaml",
        "kinematics_file": "moveit2/kinematics.yaml",
        "fixed_frame": "",
        "relative_reference_frame": "",
        "joint_states_topic": "",
        "planning_scene_input_topic": "",
        "use_sim_time": "false",
        "use_rviz": "false",
        "launch_marker_visualization": "true",
        "rviz_config_file": os.path.join(share, "rviz", "bimanual_cartesian_planning.rviz"),
    }
    return LaunchDescription([
        *[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
        OpaqueFunction(function=_setup),
    ])
