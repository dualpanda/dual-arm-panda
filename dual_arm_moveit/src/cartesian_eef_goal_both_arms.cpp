#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <algorithm>  // for std::find

class DualPandaArmCartesianMotion
{
public:
  DualPandaArmCartesianMotion(const rclcpp::Node::SharedPtr& node)
  : node_(node),
    left_arm_(node_, "left_panda_arm"),
    right_arm_(node_, "right_panda_arm"),
    both_arms_(node_, "both_arms")
  {}

  void run(geometry_msgs::msg::Pose right_target_pose, geometry_msgs::msg::Pose left_target_pose)
  {

    left_arm_.setPoseTarget(left_target_pose);
    right_arm_.setPoseTarget(right_target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan left_plan, right_plan;
    bool left_success = (left_arm_.plan(left_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    bool right_success = (right_arm_.plan(right_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (left_success && right_success)
    {
      // Get final joint states
      const auto& left_trajectory = left_plan.trajectory_.joint_trajectory;
      const auto& right_trajectory = right_plan.trajectory_.joint_trajectory;
      const auto& left_final_point = left_trajectory.points.back();
      const auto& right_final_point = right_trajectory.points.back();

      // Get all joint names for both_arms group
      const std::vector<std::string>& both_arms_joint_names = both_arms_.getJointNames();
      std::vector<double> combined_joint_positions(both_arms_joint_names.size());

      // left arm joints
      for (size_t i = 0; i < left_trajectory.joint_names.size(); ++i)
      {
        const std::string& joint_name = left_trajectory.joint_names[i];
        auto it = std::find(both_arms_joint_names.begin(), both_arms_joint_names.end(), joint_name);
        if (it != both_arms_joint_names.end())
        {
          size_t index = std::distance(both_arms_joint_names.begin(), it);
          combined_joint_positions[index] = left_final_point.positions[i];
        }
      }

      // right arm joints
      for (size_t i = 0; i < right_trajectory.joint_names.size(); ++i)
      {
        const std::string& joint_name = right_trajectory.joint_names[i];
        auto it = std::find(both_arms_joint_names.begin(), both_arms_joint_names.end(), joint_name);
        if (it != both_arms_joint_names.end())
        {
          size_t index = std::distance(both_arms_joint_names.begin(), it);
          combined_joint_positions[index] = right_final_point.positions[i];
        }
      }

      // merge the joint positions
      both_arms_.setJointValueTarget(combined_joint_positions);

      moveit::planning_interface::MoveGroupInterface::Plan combined_plan;
      bool combined_success = (both_arms_.plan(combined_plan) == moveit::core::MoveItErrorCode::SUCCESS);

      if (combined_success)
      {
        both_arms_.execute(combined_plan);
        RCLCPP_INFO(node_->get_logger(), "Both arms moved synchronously to combined joint target.");
      }
      else
      {
        RCLCPP_ERROR(node_->get_logger(), "Combined joint space planning failed.");
      }
    }
    else
    {
      if (!left_success) RCLCPP_ERROR(node_->get_logger(), "Left arm planning failed.");
      if (!right_success) RCLCPP_ERROR(node_->get_logger(), "Right arm planning failed.");
    }
  }

private:
  rclcpp::Node::SharedPtr node_;
  moveit::planning_interface::MoveGroupInterface left_arm_;
  moveit::planning_interface::MoveGroupInterface right_arm_;
  moveit::planning_interface::MoveGroupInterface both_arms_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("dual_arm_cartesian_motion_node");

  DualPandaArmCartesianMotion dual_arm_motion(node);

  // Set right arm pose target
  geometry_msgs::msg::Pose right_target_pose;
  right_target_pose.position.x = 0.5;
  right_target_pose.position.y = 0.947;
  right_target_pose.position.z = 0.553;
  right_target_pose.orientation.x = 0.924;
  right_target_pose.orientation.y = -0.382;
  right_target_pose.orientation.z = 0.015;
  right_target_pose.orientation.w = -0.006;

  // Set left arm pose target
  geometry_msgs::msg::Pose left_target_pose;
  left_target_pose.position.x = 0.106;
  left_target_pose.position.y = -0.859;
  left_target_pose.position.z = 0.541;
  left_target_pose.orientation.x = 0.924;
  left_target_pose.orientation.y = -0.382;
  left_target_pose.orientation.z = 0.015;
  left_target_pose.orientation.w = -0.006;
  dual_arm_motion.run(right_target_pose, left_target_pose);

  rclcpp::shutdown();
  return 0;
}
