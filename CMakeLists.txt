cmake_minimum_required(VERSION 3.8)
project(sura_manipulation_actions)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(osqp_vendor REQUIRED)
find_package(osqp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BaseMotionLimits.msg"
  "msg/TrajectoryTrackingTolerances.msg"
  "msg/PlanningTargetState.msg"
  "srv/UpdatePlanningTarget.srv"
  "action/FollowCartesianTrajectory.action"
  "action/PlanJointTrajectory.action"
  "action/PlanBimanualJointTrajectory.action"
  "action/PlanBimanualCartesianTrajectory.action"
  "action/PlanCartesianTrajectory.action"
  DEPENDENCIES builtin_interfaces trajectory_msgs geometry_msgs nav_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Consumers using ament_target_dependencies prefer exported targets over the
# legacy include-directory variables when rosidl targets are present.
add_library(trajectory_sampling INTERFACE)
target_include_directories(trajectory_sampling INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(trajectory_sampling INTERFACE Eigen3::Eigen)
ament_target_dependencies(trajectory_sampling INTERFACE geometry_msgs trajectory_msgs)
install(TARGETS trajectory_sampling EXPORT export_trajectory_sampling)
ament_export_targets(export_trajectory_sampling)

add_library(trajectory_execution STATIC src/trajectory_execution.cpp)
target_include_directories(trajectory_execution PUBLIC include)
ament_target_dependencies(trajectory_execution control_msgs rclcpp rclcpp_action trajectory_msgs)

add_library(bimanual_cartesian_planning STATIC
  src/bimanual_cartesian_planner.cpp
  src/cartesian_trajectory_timing.cpp
  src/cartesian_moveit_context.cpp
  src/cartesian_trajectory_visualization.cpp
)
target_include_directories(bimanual_cartesian_planning PUBLIC include)
ament_target_dependencies(bimanual_cartesian_planning
  geometry_msgs moveit_core moveit_msgs moveit_ros_planning nav_msgs rclcpp sensor_msgs
  tf2 tf2_ros trajectory_msgs visualization_msgs)
target_link_libraries(bimanual_cartesian_planning Eigen3::Eigen osqp::osqp)

add_executable(plan_bimanual_cartesian_trajectory_action_server
  src/plan_bimanual_cartesian_trajectory_action_server.cpp)
target_include_directories(plan_bimanual_cartesian_trajectory_action_server PRIVATE include)
ament_target_dependencies(plan_bimanual_cartesian_trajectory_action_server
  controller_manager_msgs control_msgs geometry_msgs moveit_core moveit_msgs moveit_ros_planning
  nav_msgs rclcpp rclcpp_action rclcpp_lifecycle sensor_msgs std_srvs tf2 tf2_ros
  trajectory_msgs visualization_msgs)
target_link_libraries(plan_bimanual_cartesian_trajectory_action_server
  bimanual_cartesian_planning trajectory_execution "${cpp_typesupport_target}")

add_executable(plan_cartesian_trajectory_action_server
  src/plan_cartesian_trajectory_action_server.cpp
  src/cartesian_pose_execution.cpp)
target_include_directories(plan_cartesian_trajectory_action_server PRIVATE include)
ament_target_dependencies(plan_cartesian_trajectory_action_server
  controller_manager_msgs control_msgs geometry_msgs moveit_core moveit_msgs moveit_ros_planning
  nav_msgs rcl_interfaces rclcpp rclcpp_action rclcpp_lifecycle sensor_msgs std_srvs tf2 tf2_ros
  trajectory_msgs visualization_msgs)
target_link_libraries(plan_cartesian_trajectory_action_server
  bimanual_cartesian_planning trajectory_execution "${cpp_typesupport_target}")

add_executable(plan_joint_trajectory_action_server
  src/plan_joint_trajectory_action_server.cpp
)
target_include_directories(plan_joint_trajectory_action_server
  PRIVATE
    include
)
ament_target_dependencies(plan_joint_trajectory_action_server
  ament_index_cpp
  control_msgs
  controller_manager_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  sensor_msgs
  std_srvs
  tf2
  tf2_ros
  trajectory_msgs
  visualization_msgs
)
target_link_libraries(plan_joint_trajectory_action_server
  trajectory_execution
  "${cpp_typesupport_target}"
  yaml-cpp
)

add_executable(hybrid_plan_joint_trajectory_action_server
  src/hybrid_plan_joint_trajectory_action_server.cpp
)
target_include_directories(hybrid_plan_joint_trajectory_action_server
  PRIVATE
    include
)
ament_target_dependencies(hybrid_plan_joint_trajectory_action_server
  ament_index_cpp
  moveit_msgs
  rclcpp
  rclcpp_action
)
target_link_libraries(hybrid_plan_joint_trajectory_action_server
  "${cpp_typesupport_target}"
  yaml-cpp
)

add_executable(plan_bimanual_joint_trajectory_action_server
  src/plan_bimanual_joint_trajectory_action_server.cpp
)
target_include_directories(plan_bimanual_joint_trajectory_action_server
  PRIVATE
    include
)
ament_target_dependencies(plan_bimanual_joint_trajectory_action_server
  control_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  tf2
  tf2_ros
  trajectory_msgs
  visualization_msgs
)
target_link_libraries(plan_bimanual_joint_trajectory_action_server
  "${cpp_typesupport_target}"
)

install(
  TARGETS
    plan_joint_trajectory_action_server
    hybrid_plan_joint_trajectory_action_server
    plan_bimanual_joint_trajectory_action_server
    plan_bimanual_cartesian_trajectory_action_server
    plan_cartesian_trajectory_action_server
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)

install(PROGRAMS
  scripts/plan_bimanual_cartesian_example.py
  scripts/manipulation_marker_visualization.py
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY docs/ DESTINATION share/${PROJECT_NAME}/docs)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_marker_visualization test/test_marker_visualization.py)
  find_package(shape_msgs REQUIRED)
  find_package(srdfdom REQUIRED)
  find_package(urdf REQUIRED)
  ament_add_gtest(test_split_controller_switching
    test/test_split_controller_switching.cpp
    src/plan_joint_trajectory_action_server.cpp)
  target_compile_definitions(test_split_controller_switching PRIVATE SURA_MANIPULATION_ACTIONS_NO_MAIN)
  target_include_directories(test_split_controller_switching PRIVATE include)
  ament_target_dependencies(test_split_controller_switching
    ament_index_cpp control_msgs controller_manager_msgs moveit_core moveit_msgs
    moveit_ros_planning rclcpp rclcpp_action rclcpp_lifecycle sensor_msgs std_srvs
    tf2 tf2_ros trajectory_msgs visualization_msgs)
  target_link_libraries(test_split_controller_switching trajectory_execution "${cpp_typesupport_target}" yaml-cpp)
  ament_add_gtest(test_trajectory_execution test/test_trajectory_execution.cpp)
  target_include_directories(test_trajectory_execution PRIVATE include)
  ament_target_dependencies(test_trajectory_execution control_msgs rclcpp rclcpp_action trajectory_msgs)
  target_link_libraries(test_trajectory_execution trajectory_execution)
  ament_add_gtest(test_bimanual_cartesian_planner test/test_bimanual_cartesian_planner.cpp)
  target_include_directories(test_bimanual_cartesian_planner PRIVATE include)
  ament_target_dependencies(test_bimanual_cartesian_planner
    geometry_msgs moveit_core moveit_msgs moveit_ros_planning shape_msgs srdfdom trajectory_msgs urdf)
  target_link_libraries(test_bimanual_cartesian_planner bimanual_cartesian_planning)
  ament_add_gtest(test_base_trajectory_sampling test/test_base_trajectory_sampling.cpp)
  target_include_directories(test_base_trajectory_sampling PRIVATE include)
  ament_target_dependencies(test_base_trajectory_sampling geometry_msgs trajectory_msgs)
  target_link_libraries(test_base_trajectory_sampling Eigen3::Eigen)
  ament_add_gtest(test_cartesian_pose_path test/test_cartesian_pose_path.cpp)
  target_include_directories(test_cartesian_pose_path PRIVATE include)
  ament_target_dependencies(test_cartesian_pose_path geometry_msgs nav_msgs)
  target_link_libraries(test_cartesian_pose_path Eigen3::Eigen)
  ament_add_gtest(test_bimanual_cartesian_action_server
    test/test_bimanual_cartesian_action_server.cpp
    src/plan_bimanual_cartesian_trajectory_action_server.cpp)
  target_compile_definitions(test_bimanual_cartesian_action_server PRIVATE SURA_MANIPULATION_ACTIONS_NO_MAIN)
  target_include_directories(test_bimanual_cartesian_action_server PRIVATE include)
  ament_target_dependencies(test_bimanual_cartesian_action_server
    controller_manager_msgs control_msgs geometry_msgs lifecycle_msgs moveit_core moveit_msgs
    moveit_ros_planning nav_msgs rclcpp rclcpp_action rclcpp_lifecycle sensor_msgs std_srvs
    tf2 tf2_ros trajectory_msgs visualization_msgs)
  target_link_libraries(test_bimanual_cartesian_action_server
    bimanual_cartesian_planning trajectory_execution "${cpp_typesupport_target}")
  ament_add_gtest(test_cartesian_action_server
    test/test_cartesian_action_server.cpp src/plan_cartesian_trajectory_action_server.cpp
    src/cartesian_pose_execution.cpp)
  target_compile_definitions(test_cartesian_action_server PRIVATE SURA_MANIPULATION_ACTIONS_NO_MAIN)
  target_include_directories(test_cartesian_action_server PRIVATE include)
  ament_target_dependencies(test_cartesian_action_server
    controller_manager_msgs control_msgs geometry_msgs lifecycle_msgs moveit_core moveit_msgs
    moveit_ros_planning nav_msgs rcl_interfaces rclcpp rclcpp_action rclcpp_lifecycle sensor_msgs std_srvs
    tf2 tf2_ros trajectory_msgs visualization_msgs)
  target_link_libraries(test_cartesian_action_server
    bimanual_cartesian_planning trajectory_execution "${cpp_typesupport_target}")
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_dependencies(rosidl_default_runtime builtin_interfaces Eigen3 geometry_msgs nav_msgs trajectory_msgs)
ament_package()
