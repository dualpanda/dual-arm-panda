#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <vector>
#include <string>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("dual_arm_motion_node");

  // MoveGroupInterface for the group "both_arms"
  moveit::planning_interface::MoveGroupInterface both_arms(node, "both_arms");

  std::vector<double> both_targets = {
  // left panda joint space goal configuration
  0.0206780444644536,-0.787689005521735,-0.755570458607052,-2.26892338409648,-0.491352495403048,1.65786434715587,0.250407963418457,
  
  // right panda joint space goal configuration
  0.360851122837973,-0.743121251229319,-1.18566825440064,-1.91535633229137,-0.656292259712039,1.60593421018501,0.0855107678155243
  };

  both_arms.setJointValueTarget(both_targets);

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

  rclcpp::shutdown();
  return 0;
}
