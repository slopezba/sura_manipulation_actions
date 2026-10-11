cmake_minimum_required(VERSION 3.8)
project(sura_manipulation_actions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/PlanJointTrajectory.action"
  "action/PlanBimanualJointTrajectory.action"
  DEPENDENCIES trajectory_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

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
  trajectory_msgs
  visualization_msgs
)
target_link_libraries(plan_joint_trajectory_action_server
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

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_split_controller_switching
    test/test_split_controller_switching.cpp
    src/plan_joint_trajectory_action_server.cpp)
  target_compile_definitions(test_split_controller_switching PRIVATE SURA_MANIPULATION_ACTIONS_NO_MAIN)
  target_include_directories(test_split_controller_switching PRIVATE include)
  ament_target_dependencies(test_split_controller_switching
    ament_index_cpp control_msgs controller_manager_msgs moveit_core moveit_msgs
    moveit_ros_planning rclcpp rclcpp_action rclcpp_lifecycle sensor_msgs std_srvs
    trajectory_msgs visualization_msgs)
  target_link_libraries(test_split_controller_switching "${cpp_typesupport_target}" yaml-cpp)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
