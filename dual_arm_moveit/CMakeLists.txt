cmake_minimum_required(VERSION 3.8)
project(dual_arm_moveit)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(moveit_common REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)
find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(moveit_visual_tools REQUIRED)
find_package(geometric_shapes REQUIRED)
#find_package(pcl_ros REQUIRED)
#find_package(pcl_conversions REQUIRED)
#find_package(rosbag REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(joint_space_goal_both_arms src/joint_space_goal_both_arms.cpp)
ament_target_dependencies(joint_space_goal_both_arms rclcpp moveit_ros_planning_interface)

add_executable(cartesian_eef_goal_both_arms src/cartesian_eef_goal_both_arms.cpp)
ament_target_dependencies(cartesian_eef_goal_both_arms rclcpp moveit_ros_planning_interface)

add_executable(lift_rotate src/lift_rotate.cpp)
ament_target_dependencies(lift_rotate rclcpp moveit_ros_planning_interface)

add_executable(collision_potential src/collision_potential.cpp)
ament_target_dependencies(collision_potential rclcpp moveit_ros_planning_interface)

install(TARGETS joint_space_goal_both_arms
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS cartesian_eef_goal_both_arms
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS lift_rotate
  DESTINATION lib/${PROJECT_NAME})

  install(TARGETS collision_potential
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
