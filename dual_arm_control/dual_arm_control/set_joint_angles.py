#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')

        # joint position for left panda
        self.left_panda_joint_radians = {
        "left_panda_joint1": 0.263271427929107,
        "left_panda_joint2": -0.431006826665367,
        "left_panda_joint3": 0.560042807184711,
        "left_panda_joint4": -2.02842309848441,
        "left_panda_joint5": 0.224696126228442,
        "left_panda_joint6": 1.65481626768699,
        "left_panda_joint7": 1.54665491329394,
        "left_panda_finger_joint1":	-6.43E-09,
        "left_panda_finger_joint2":	7.77E-06}

        # joint position for right panda
        self.right_panda_joint_radians = {
        "right_panda_joint1": 0.299176901619843,
        "right_panda_joint2": -0.676226985257635,
        "right_panda_joint3": 0.605330360639889,
        "right_panda_joint4": -2.29661292997428,
        "right_panda_joint5": 0.344762436342283,
        "right_panda_joint6": 1.72860011965248,
        "right_panda_joint7": 1.53131259314358,
        "right_panda_finger_joint1": -1.29E-05,
        "right_panda_finger_joint2": 1.29E-05}

        # ROS2 controller publishers
        self.left_arm_pub = self.create_publisher(JointTrajectory, '/left_arm_controller/joint_trajectory', 10)
        self.right_arm_pub = self.create_publisher(JointTrajectory, '/right_arm_controller/joint_trajectory', 10)
        self.left_gripper_pub = self.create_publisher(JointTrajectory, '/left_gripper_controller/joint_trajectory',10)
        self.right_gripper_pub = self.create_publisher(JointTrajectory, '/right_gripper_controller/joint_trajectory', 10)

        self.timer = self.create_timer(2.0, self.send_joint_command)
        self.get_logger().info("JointCommandPublisher initialized and publishing...")
    
    def get_joint_msg(self, panda):
        """
        get panda arm joints Trajectory message
        """
        msg = JointTrajectory()
        msg.joint_names = [
            '{}_panda_joint1'.format(panda), '{}_panda_joint2'.format(panda), '{}_panda_joint3'.format(panda),
            '{}_panda_joint4'.format(panda), '{}_panda_joint5'.format(panda), '{}_panda_joint6'.format(panda),
            '{}_panda_joint7'.format(panda)
        ]

        positions = []
        for jnames in msg.joint_names:
            if panda == "left":
                positions.append(self.left_panda_joint_radians[jnames])
            if panda == "right":
                positions.append(self.right_panda_joint_radians[jnames])

        arm_point = JointTrajectoryPoint()
        arm_point.positions = positions
        arm_point.time_from_start.sec = 2
        msg.points.append(arm_point)

        return msg
    
    def get_gripper_msg(self, panda):
        """
        get gripper joints Trajectory message
        """
        gripper_msg = JointTrajectory()
        gripper_msg.joint_names = ['{}_panda_finger_joint1'.format(panda), '{}_panda_finger_joint2'.format(panda)]

        gripper_positions = []
        for jnames in gripper_msg.joint_names:
            if panda == "left":
                gripper_positions.append(self.left_panda_joint_radians[jnames])
            if panda == "right":
                gripper_positions.append(self.right_panda_joint_radians[jnames])

        gripper_point = JointTrajectoryPoint()
        gripper_point.positions = gripper_positions
        gripper_point.time_from_start.sec = 2
        gripper_msg.points.append(gripper_point)

        return gripper_msg

    def send_joint_command(self):
        """
        send joint commands to ROS2 controller
        """
        # left panda arm
        left_arm_msg = self.get_joint_msg(panda="left")
        self.left_arm_pub.publish(left_arm_msg)
        self.get_logger().info("Published joint trajectory to left_arm_controller.")

        # right panda arm
        right_arm_msg = self.get_joint_msg(panda="right")
        self.right_arm_pub.publish(right_arm_msg)
        self.get_logger().info("Published joint trajectory to right_arm_controller.")

        # left panda gripper
        left_gripper_msg = self.get_gripper_msg(panda="left")
        self.left_gripper_pub.publish(left_gripper_msg)
        self.get_logger().info("Published joint trajectory to left_gripper_controller.")

        # right panda arm
        right_gripper_msg = self.get_gripper_msg(panda="right")
        self.right_gripper_pub.publish(right_gripper_msg)
        self.get_logger().info("Published joint trajectory to right_gripper_controller.")

def main(args=None):
    rclpy.init(args=args)
    node = JointCommandPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
