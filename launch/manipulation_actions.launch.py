"""Unified manipulation action servers, with manual lifecycle/backend activation."""
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def _render(value, namespace):
    if isinstance(value, str):
        if not namespace:
            value = value.replace("${robot_namespace}/", "")
        return value.replace("${robot_namespace}", namespace).replace("//", "/")
    if isinstance(value, dict):
        return {key: _render(item, namespace) for key, item in value.items()}
    if isinstance(value, list):
        return [_render(item, namespace) for item in value]
    return value


def _setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    def enabled(name):
        return arg(name).lower() == "true"
    namespace = arg("robot_namespace").strip("/")
    prefix = f"/{namespace}" if namespace else ""
    share = get_package_share_directory("sura_manipulation_actions")
    moveit_share = get_package_share_directory(arg("moveit_config_package"))
    robot = {}
    for parameter, package, filename, arguments in (
        ("robot_description", "robot_description_package", "robot_description_file", "xacro_arguments"),
        ("robot_description_semantic", "robot_description_semantic_package", "robot_description_semantic_file", "srdf_xacro_arguments"),
    ):
        path = os.path.join(get_package_share_directory(arg(package)), arg(filename))
        robot[parameter] = ParameterValue(Command([
            FindExecutable(name="xacro"), " ", path, " ", _render(arg(arguments), namespace)]), value_type=str)
    limits = os.path.join(moveit_share, arg("joint_limits_file"))
    kinematics = os.path.join(moveit_share, arg("kinematics_file"))
    common = {"robot_namespace": namespace, "moveit_namespace": arg("moveit_namespace"),
              "use_sim_time": enabled("use_sim_time"), "execution_backend": arg("execution_backend"),
              "current_state_timeout": float(arg("current_state_timeout")),
              "planning_scene_publish_rate": float(arg("planning_scene_publish_rate"))}
    left_task = arg("left_task_action") or f"{prefix}/controller/task_priority/tasks/left_joint_trajectory/follow_joint_trajectory"
    right_task = arg("right_task_action") or f"{prefix}/controller/task_priority/tasks/right_joint_trajectory/follow_joint_trajectory"
    joint = dict(common, default_planning_time=float(arg("default_planning_time")),
                 default_goal_tolerance=float(arg("default_goal_tolerance")),
                 controller_wait_timeout=float(arg("controller_wait_timeout")),
                 execution_feedback_rate=float(arg("execution_feedback_rate")),
                 left_task_action=left_task, right_task_action=right_task,
                 predefined_poses_file=arg("predefined_poses_file"))
    if arg("joint_states_topic"):
        joint["joint_states_topic"] = arg("joint_states_topic")
    pipelines = [os.path.join(moveit_share, arg(name)) for name in ("planning_pipelines_file", "ompl_planning_file")]
    actions = []
    if enabled("launch_marker_visualization"):
        actions.append(Node(package="sura_manipulation_actions",
            executable="manipulation_marker_visualization.py",
            namespace=namespace, output="screen",
            parameters=[{"use_sim_time": enabled("use_sim_time")}]))
    def server(executable, parameters):
        actions.append(LifecycleNode(package="sura_manipulation_actions", executable=executable,
            name=executable.replace("_action_server", "_lifecycle_action_server"),
            namespace="", output="screen", parameters=parameters))
    if enabled("launch_joint"):
        server("plan_joint_trajectory_action_server", [robot, *pipelines, limits, kinematics, joint])
    if enabled("launch_bimanual_joint"):
        server("plan_bimanual_joint_trajectory_action_server", [robot, *pipelines, limits, kinematics,
            dict(joint, group_name=arg("bimanual_group_name"))])
    for flag, executable, config_arg, reference_key in (
        ("launch_cartesian", "plan_cartesian_trajectory_action_server", "cartesian_planner_config_file", "reference_frame"),
        ("launch_bimanual_cartesian", "plan_bimanual_cartesian_trajectory_action_server", "planner_config_file", "relative_reference_frame"),
    ):
        if not enabled(flag):
            continue
        with open(arg(config_arg), encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        if (not isinstance(document, dict) or not isinstance(document.get("/**"), dict)
                or not isinstance(document["/**"].get("ros__parameters"), dict)):
            raise ValueError(f"{config_arg} must contain /**: ros__parameters:")
        config = _render(document["/**"]["ros__parameters"], namespace)
        config.update(common)
        config["planning.default_planning_time"] = float(arg("default_planning_time"))
        config["execution.controller_wait_timeout"] = float(arg("controller_wait_timeout"))
        config["execution.feedback_rate"] = float(arg("execution_feedback_rate"))
        for name, parameter in (("joint_states_topic", "joint_states_topic"),
            ("planning_scene_input_topic", "planning_scene_input_topic"), ("fixed_frame", "visualization.fixed_frame")):
            if arg(name):
                config[parameter] = arg(name)
        reference = arg("relative_reference_frame") if reference_key == "relative_reference_frame" else ""
        if reference or arg("reference_frame"):
            config[reference_key] = reference or arg("reference_frame")
        for arm, endpoint, explicit in (("alpha_left", left_task, arg("left_task_action")),
                                      ("alpha_right", right_task, arg("right_task_action"))):
            if explicit and f"arms.{arm}.task_action" in config:
                config[f"arms.{arm}.task_action"] = endpoint
        server(executable, [robot, limits, kinematics, config])
    if enabled("launch_hybrid"):
        arguments = {name: arg(name) for name in ("robot_namespace", "robot_description_package", "robot_description_file",
            "robot_description_semantic_package", "robot_description_semantic_file", "execution_backend", "use_sim_time")}
        arguments.update(xacro_arguments=_render(arg("xacro_arguments"), namespace), use_rviz="false",
                         joint_states_topic=arg("joint_states_topic") or f"{prefix}/alpha/joint_states")
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(share, "launch", "manipulation_hybrid_planning_actions.launch.py")), launch_arguments=arguments.items()))
    if enabled("use_rviz"):
        actions.append(Node(package="rviz2", executable="rviz2", name="rviz2_manipulation", output="screen",
            arguments=["-d", arg("rviz_config_file")], parameters=[robot, limits, kinematics, {"use_sim_time": enabled("use_sim_time")}]))
    if enabled("use_rqt"):
        actions.append(Node(package="rqt_gui", executable="rqt_gui", output="screen",
            arguments=["--standalone", "task_priority_kinematic_control_rqt/TaskPriorityPanel"],
            parameters=[{"use_sim_time": enabled("use_sim_time")}]))
    return actions


def generate_launch_description():
    share = get_package_share_directory("sura_manipulation_actions")
    defaults = {
        "robot_namespace": "cirtesub", "moveit_namespace": "moveit", "execution_backend": "controller",
        "robot_description_package": "cirtesub_description", "robot_description_file": "urdf/cirtesub_dual_alpha.urdf.xacro",
        "robot_description_semantic_package": "cirtesub_description", "robot_description_semantic_file": "moveit2/cirtesub_dual_alpha.srdf.xacro",
        "xacro_arguments": "robot_name:=${robot_namespace} environment:=sim arms:=dual use_sim:=true alpha_desired_joint_states_topic:=/${robot_namespace}/alpha/desired_joint_states alpha_joint_states_topic:=/${robot_namespace}/stonefish/alpha/joint_states alpha_normalized_joint_states_topic:=/${robot_namespace}/alpha/joint_states",
        "srdf_xacro_arguments": "robot_name:=${robot_namespace}", "moveit_config_package": "cirtesub_description",
        "planning_pipelines_file": "moveit2/planning_pipelines.yaml", "ompl_planning_file": "moveit2/ompl_planning.yaml",
        "joint_limits_file": "moveit2/joint_limits.yaml", "kinematics_file": "moveit2/kinematics.yaml",
        "predefined_poses_file": os.path.join(share, "config", "predefined_poses.yaml"),
        "planner_config_file": os.path.join(share, "config", "bimanual_cartesian_planner.yaml"),
        "cartesian_planner_config_file": os.path.join(share, "config", "single_arm_cartesian_planner.yaml"),
        "left_task_action": "", "right_task_action": "", "bimanual_group_name": "dual_alpha",
        "default_planning_time": "5.0", "default_goal_tolerance": "0.02", "controller_wait_timeout": "5.0",
        "execution_feedback_rate": "10.0", "current_state_timeout": "2.0", "planning_scene_publish_rate": "10.0",
        "reference_frame": "", "relative_reference_frame": "", "fixed_frame": "", "joint_states_topic": "", "planning_scene_input_topic": "",
        "use_sim_time": "false", "use_rviz": "false", "use_rqt": "false",
        "rviz_config_file": os.path.join(share, "rviz", "manipulation_planning.rviz"),
        "launch_joint": "true", "launch_bimanual_joint": "true", "launch_cartesian": "true", "launch_bimanual_cartesian": "true", "launch_hybrid": "false",
        "launch_marker_visualization": "true",
    }
    return LaunchDescription([*[DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()],
                              OpaqueFunction(function=_setup)])
