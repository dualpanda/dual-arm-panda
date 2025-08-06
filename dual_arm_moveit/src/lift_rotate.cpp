#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <algorithm>  // for std::find
#include<chrono>

class DualPandArmMultipleCartesianMotion
{
public:
  DualPandArmMultipleCartesianMotion(const rclcpp::Node::SharedPtr& node)
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
      // final joint states
      const auto& left_trajectory = left_plan.trajectory_.joint_trajectory;
      const auto& right_trajectory = right_plan.trajectory_.joint_trajectory;
      const auto& left_final_point = left_trajectory.points.back();
      const auto& right_final_point = right_trajectory.points.back();

      // joint names for both_arms group
      const std::vector<std::string>& both_arms_joint_names = both_arms_.getJointNames();
      std::vector<double> combined_joint_values(both_arms_joint_names.size());

      // left arm joints
      for (size_t i = 0; i < left_trajectory.joint_names.size(); ++i)
      {
        const std::string& joint_name = left_trajectory.joint_names[i];
        auto it = std::find(both_arms_joint_names.begin(), both_arms_joint_names.end(), joint_name);
        if (it != both_arms_joint_names.end())
        {
          size_t index = std::distance(both_arms_joint_names.begin(), it);
          combined_joint_values[index] = left_final_point.positions[i];
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
          combined_joint_values[index] = right_final_point.positions[i];
        }
      }
      // combine the joint positions
      both_arms_.setJointValueTarget(combined_joint_values);

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

  DualPandArmMultipleCartesianMotion dual_arm_motion(node);

  geometry_msgs::msg::Pose right_before_lifitng_pose;
  geometry_msgs::msg::Pose left_before_lifitng_pose;
  left_before_lifitng_pose.position.x = 0.045;
  left_before_lifitng_pose.position.y = -0.357;
  left_before_lifitng_pose.position.z = 0.405;
  left_before_lifitng_pose.orientation.x = -0.220;
  left_before_lifitng_pose.orientation.y = 0.720;
  left_before_lifitng_pose.orientation.z = 0.577;
  left_before_lifitng_pose.orientation.w = 0.318;

  right_before_lifitng_pose.position.x = 0.110;
  right_before_lifitng_pose.position.y = -0.016;
  right_before_lifitng_pose.position.z = 0.403;
  right_before_lifitng_pose.orientation.x = -0.293;
  right_before_lifitng_pose.orientation.y = 0.695;
  right_before_lifitng_pose.orientation.z = -0.623;
  right_before_lifitng_pose.orientation.w = -0.205;
  dual_arm_motion.run(right_before_lifitng_pose, left_before_lifitng_pose);
  rclcpp::sleep_for(std::chrono::seconds(3));
  
  geometry_msgs::msg::Pose right_target_pose_lifting;
  geometry_msgs::msg::Pose left_target_pose_lifting;

  left_target_pose_lifting.position.x = 0.045;
  left_target_pose_lifting.position.y = -0.357;
  left_target_pose_lifting.position.z = 0.405;
  left_target_pose_lifting.orientation.x = -0.220;
  left_target_pose_lifting.orientation.y = 0.720;
  left_target_pose_lifting.orientation.z = 0.576;
  left_target_pose_lifting.orientation.w = 0.319;

  right_target_pose_lifting.position.x = 0.110;
  right_target_pose_lifting.position.y = -0.016;
  right_target_pose_lifting.position.z = 0.403;
  right_target_pose_lifting.orientation.x = -0.293;
  right_target_pose_lifting.orientation.y = 0.695;
  right_target_pose_lifting.orientation.z = -0.623;
  right_target_pose_lifting.orientation.w = -0.205;
  dual_arm_motion.run(right_target_pose_lifting, left_target_pose_lifting);

  geometry_msgs::msg::Pose right_rotate;
  geometry_msgs::msg::Pose left_rotate;

  right_rotate.position.x = 0.110;
  right_rotate.position.y = -0.016;
  right_rotate.position.z = 0.403;
  right_rotate.orientation.x = 0.693;
  right_rotate.orientation.y = 0.299;
  right_rotate.orientation.z = -0.211;
  right_rotate.orientation.w = 0.622;

  left_rotate.position.x = 0.045;
  left_rotate.position.y = -0.357;
  left_rotate.position.z = 0.405;
  left_rotate.orientation.x = -0.723;
  left_rotate.orientation.y = -0.207;
  left_rotate.orientation.z = -0.308;
  left_rotate.orientation.w = 0.582;

  rclcpp::sleep_for(std::chrono::seconds(3));
  dual_arm_motion.run(right_rotate, left_rotate);
  rclcpp::shutdown();
  return 0;
}
