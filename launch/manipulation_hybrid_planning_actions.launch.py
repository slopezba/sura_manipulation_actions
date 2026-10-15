import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _as_float(context, launch_config):
    return float(launch_config.perform(context))


def _arm_local_solution_topic(context, arm):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    backend = LaunchConfiguration("execution_backend").perform(context)
    prefix = f"/{robot_namespace}/controller" if robot_namespace else "/controller"
    if backend == "task":
        return f"{prefix}/task_priority/tasks/{arm}_joint_trajectory/joint_trajectory"
    alpha_prefix = f"/{robot_namespace}/alpha/controller" if robot_namespace else "/alpha/controller"
    return f"{alpha_prefix}/alpha_{arm}_joint_trajectory_controller/joint_trajectory"


def _hybrid_container(context, arm, group_name, robot_description, robot_description_semantic):
    robot_namespace = LaunchConfiguration("robot_namespace").perform(context).strip("/")
    joint_states_topic = LaunchConfiguration("joint_states_topic").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time")
    local_planning_frequency = LaunchConfiguration("local_planning_frequency")
    stop_before_collision = LaunchConfiguration("stop_before_collision").perform(context).lower() == "true"

    namespace = f"hybrid_planning_{arm}"
    global_solution_topic = f"/{namespace}/global_trajectory"
    global_planning_action_name = f"/{namespace}/global_planning_action"
    local_planning_action_name = f"/{namespace}/local_planning_action"
    hybrid_planning_action_name = f"/{namespace}/run_hybrid_planning"
    planning_scene_topic = f"/{robot_namespace}/moveit/monitored_planning_scene" if robot_namespace else "/moveit/monitored_planning_scene"

    common_hybrid_planning_param = {
        "use_sim_time": use_sim_time,
        "global_solution_topic": global_solution_topic,
        "global_planning_action_name": global_planning_action_name,
        "local_planning_action_name": local_planning_action_name,
        "hybrid_planning_action_name": hybrid_planning_action_name,
    }

    global_planner_param = {
        "global_planner_name": "moveit_hybrid_planning/MoveItPlanningPipeline",
        "global_planning_action_name": global_planning_action_name,
        "planning_scene_monitor_options": {
            "name": "planning_scene_monitor",
            "robot_description": "robot_description",
            "joint_state_topic": joint_states_topic,
            "attached_collision_object_topic": "/attached_collision_object",
            "publish_planning_scene_topic": "/planning_scene",
            "monitored_planning_scene_topic": f"/{namespace}/planning_scene",
            "wait_for_initial_state_timeout": 10.0,
        },
        "planning_pipelines": {
            "pipeline_names": ["ompl"],
        },
        "plan_request_params": {
            "planning_attempts": 1,
            "planning_pipeline": "ompl",
            "planner_id": "ompl",
            "max_velocity_scaling_factor": _as_float(
                context, LaunchConfiguration("max_velocity_scaling_factor")
            ),
            "max_acceleration_scaling_factor": _as_float(
                context, LaunchConfiguration("max_acceleration_scaling_factor")
            ),
        },
    }

    local_planner_param = {
        "robot_description": "robot_description",
        "trajectory_operator_plugin_name": "moveit_hybrid_planning/SimpleSampler",
        "local_constraint_solver_plugin_name": "moveit_hybrid_planning/ForwardTrajectory",
        "local_planning_frequency": _as_float(context, local_planning_frequency),
        "global_solution_topic": global_solution_topic,
        "local_solution_topic": _arm_local_solution_topic(context, arm),
        "local_solution_topic_type": "trajectory_msgs/msg/JointTrajectory",
        "publish_joint_positions": True,
        "publish_joint_velocities": False,
        "group_name": group_name,
        "monitored_planning_scene": planning_scene_topic,
        "collision_object_topic": "/collision_object",
        "joint_states_topic": joint_states_topic,
        "stop_before_collision": stop_before_collision,
    }

    hybrid_planning_manager_param = {
        "planner_logic_plugin_name": "moveit_hybrid_planning/SinglePlanExecution",
        "global_planning_action_name": global_planning_action_name,
        "local_planning_action_name": local_planning_action_name,
        "hybrid_planning_action_name": hybrid_planning_action_name,
    }

    return ComposableNodeContainer(
        name=f"{namespace}_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::GlobalPlannerComponent",
                name="global_planner",
                namespace=namespace,
                parameters=[
                    common_hybrid_planning_param,
                    global_planner_param,
                    robot_description,
                    robot_description_semantic,
                    LaunchConfiguration("kinematics_file"),
                    LaunchConfiguration("planning_pipelines_file"),
                    LaunchConfiguration("ompl_planning_file"),
                    LaunchConfiguration("joint_limits_file"),
                ],
            ),
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::LocalPlannerComponent",
                name="local_planner",
                namespace=namespace,
                parameters=[
                    common_hybrid_planning_param,
                    local_planner_param,
                    robot_description,
                    robot_description_semantic,
                    LaunchConfiguration("kinematics_file"),
                ],
            ),
            ComposableNode(
                package="moveit_hybrid_planning",
                plugin="moveit::hybrid_planning::HybridPlanningManager",
                name="hybrid_planning_manager",
                namespace=namespace,
                parameters=[
                    common_hybrid_planning_param,
                    hybrid_planning_manager_param,
                ],
            ),
        ],
        output="screen",
    )


def _launch_setup(context, *args, **kwargs):
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
    execution_backend = LaunchConfiguration("execution_backend")
    hybrid_result_timeout = LaunchConfiguration("hybrid_result_timeout")
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
                    " floating_base:=true world_frame:=world_ned",
                ]
            ),
            value_type=str,
        )
    }

    manipulation_share = get_package_share_directory("sura_manipulation_actions")

    return [
        _hybrid_container(context, "left", "alpha_left", robot_description, robot_description_semantic),
        _hybrid_container(context, "right", "alpha_right", robot_description, robot_description_semantic),
        Node(
            package="sura_manipulation_actions",
            executable="hybrid_plan_joint_trajectory_action_server",
            name="hybrid_plan_joint_trajectory_action_server",
            namespace="",
            output="screen",
            parameters=[
                robot_description,
                robot_description_semantic,
                LaunchConfiguration("planning_pipelines_file"),
                LaunchConfiguration("ompl_planning_file"),
                LaunchConfiguration("joint_limits_file"),
                LaunchConfiguration("kinematics_file"),
                {
                    "robot_namespace": robot_namespace,
                    "moveit_namespace": moveit_namespace,
                    "use_sim_time": use_sim_time,
                    "default_planning_time": ParameterValue(
                        default_planning_time, value_type=float
                    ),
                    "default_goal_tolerance": ParameterValue(
                        default_goal_tolerance, value_type=float
                    ),
                    "execution_backend": execution_backend,
                    "hybrid_result_timeout": ParameterValue(
                        hybrid_result_timeout, value_type=float
                    ),
                    "predefined_poses_file": predefined_poses_file,
                    "left_hybrid_action": "/hybrid_planning_left/run_hybrid_planning",
                    "right_hybrid_action": "/hybrid_planning_right/run_hybrid_planning",
                    "joint_states_topic": LaunchConfiguration("joint_states_topic"),
                    "max_velocity_scaling_factor": ParameterValue(
                        LaunchConfiguration("max_velocity_scaling_factor"), value_type=float
                    ),
                    "max_acceleration_scaling_factor": ParameterValue(
                        LaunchConfiguration("max_acceleration_scaling_factor"), value_type=float
                    ),
                },
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_manipulation_hybrid",
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


def generate_launch_description():
    cirtesub_share = get_package_share_directory("cirtesub_description")
    manipulation_share = get_package_share_directory("sura_manipulation_actions")
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
            DeclareLaunchArgument("execution_backend", default_value="controller"),
            DeclareLaunchArgument("hybrid_result_timeout", default_value="15.0"),
            DeclareLaunchArgument("local_planning_frequency", default_value="10.0"),
            DeclareLaunchArgument("stop_before_collision", default_value="true"),
            DeclareLaunchArgument("max_velocity_scaling_factor", default_value="0.05"),
            DeclareLaunchArgument("max_acceleration_scaling_factor", default_value="0.05"),
            DeclareLaunchArgument(
                "joint_states_topic",
                default_value="/cirtesub/alpha/joint_states",
            ),
            DeclareLaunchArgument(
                "planning_pipelines_file",
                default_value=os.path.join(cirtesub_share, "moveit2", "planning_pipelines.yaml"),
            ),
            DeclareLaunchArgument(
                "ompl_planning_file",
                default_value=os.path.join(cirtesub_share, "moveit2", "ompl_planning.yaml"),
            ),
            DeclareLaunchArgument(
                "joint_limits_file",
                default_value=os.path.join(cirtesub_share, "moveit2", "joint_limits.yaml"),
            ),
            DeclareLaunchArgument(
                "kinematics_file",
                default_value=os.path.join(cirtesub_share, "moveit2", "kinematics.yaml"),
            ),
            DeclareLaunchArgument("predefined_poses_file", default_value=predefined_poses),
            DeclareLaunchArgument("use_rviz", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
