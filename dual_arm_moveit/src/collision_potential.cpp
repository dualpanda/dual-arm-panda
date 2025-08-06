#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("dual_arm_motion_node");

  moveit::planning_interface::MoveGroupInterface both_arms(node, "both_arms");

  std::vector<double> before_collision = {
  // left panda joint space goal configuration
  -0.551702029847455,-0.904491595143522,1.2985342547742,-1.7110367451291,0.845094186896334,1.47244101453092,1.52139519731272,
  
  // right panda joint space goal configuration
  0.42312417410645,-1.55281633977829,-1.63589286769863,-2.45272511122762,-1.60092412678214,1.65830853520606,0.519247998138841
  };

  std::vector<double> avoid_collision = {
  // left panda joint space goal configuration
  -0.641327284414766,-1.09364122996699,1.69177877255866,-2.17825615472054,1.21217222764509,1.95911162053081,1.20459700141514,
  
  // right panda joint space goal configuration
  0.589849166229941,-1.16916687739806,-0.858750162600407,-1.86209593054847,-0.89177236264942,1.12004998328056,0.422091962861483
  };

  both_arms.setJointValueTarget(before_collision);

  // planning
  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = static_cast<bool>(both_arms.plan(plan));

  if (success)
  {
    // execute the plan in gazebo via ROS2 control
    both_arms.execute(plan);
    RCLCPP_INFO(node->get_logger(), "Both arms moved to target positions synchronously.");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning failed for the both_arms group.");
  }

  rclcpp::sleep_for(std::chrono::seconds(3));
  both_arms.setJointValueTarget(avoid_collision);

  // planning
  moveit::planning_interface::MoveGroupInterface::Plan plan_next;
  success = static_cast<bool>(both_arms.plan(plan_next));

  if (success)
  {
    // execute the plan in gazebo via ROS2 control
    both_arms.execute(plan_next);
    RCLCPP_INFO(node->get_logger(), "Both arms moved to target positions synchronously.");
  }
  else
  {
    RCLCPP_ERROR(node->get_logger(), "Planning failed for the both_arms group.");
  }

  rclcpp::shutdown();
  return 0;
}
