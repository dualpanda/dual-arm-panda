#!/usr/bin/env python3

import time
import numpy as np
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from urdf_parser_py.urdf import URDF
from rcl_interfaces.srv import GetParameters

class JointLimitFromURDF(Node):
    def __init__(self):
        super().__init__('joint_limit_from_URDF')

        # Dictionary for joint limits
        self.panda_joint_limits = {}

        # Create client to get parameter from another node
        self.client_param_from_node = self.create_client(GetParameters, '/robot_state_publisher/get_parameters')
        while not self.client_param_from_node.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Waiting for /robot_state_publisher/get_parameters service...')

        # Prepare the request
        req = GetParameters.Request()
        req.names = ['robot_description']

        # Call the service
        future = self.client_param_from_node.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            urdf_string = future.result().values[0].string_value
            self.urdf_limits(urdf_string)
        else:
            self.get_logger().error('Failed to get robot_description from robot_state_publisher')

    def urdf_limits(self, urdf_string):
        """
        Set joint limits in self.panda_joint_limits 
        """
        try:
            # access URDF
            robot = URDF.from_xml_string(urdf_string)
            for joint in robot.joints:
                if joint.limit:
                    self.panda_joint_limits[joint.name] = {'lower': joint.limit.lower, 'upper': joint.limit.upper}
        except Exception as e:
            self.get_logger().error(f"Failed to parse URDF: {e}")
    
    def get_joint_limits(self):
        return self.panda_joint_limits
        

class SineMotionROScontrol(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        self.js_limit_reader = JointLimitFromURDF()

        self.joint_limits = self.js_limit_reader.get_joint_limits()

        self.left_joints = [
            'left_panda_joint1', 'left_panda_joint2', 'left_panda_joint3',
            'left_panda_joint4', 'left_panda_joint5', 'left_panda_joint6',
            'left_panda_joint7'
        ]
        self.right_joints = [
            'right_panda_joint1', 'right_panda_joint2', 'right_panda_joint3',
            'right_panda_joint4', 'right_panda_joint5', 'right_panda_joint6',
            'right_panda_joint7'
        ]

        self.left_arm_pub = self.create_publisher(JointTrajectory, '/left_arm_controller/joint_trajectory', 10)
        self.right_arm_pub = self.create_publisher(JointTrajectory, '/right_arm_controller/joint_trajectory', 10)
        self.left_gripper_pub = self.create_publisher(JointTrajectory, '/left_gripper_controller/joint_trajectory', 10)
        self.right_gripper_pub = self.create_publisher(JointTrajectory, '/right_gripper_controller/joint_trajectory', 10)

        self.start_time = time.time()
        self.timer = self.create_timer(0.1, self.send_joint_command)  # 10 Hz
        self.get_logger().info("SineMotionROScontrol initialized and publishing sine wave motions...")

    def sine_joint_position(self, joint_name, t):
        """
        create sine wave motion with the joint
        """
        if joint_name in self.joint_limits:
            lower = self.joint_limits[joint_name]['lower']
            upper = self.joint_limits[joint_name]['upper']
            mid = (upper + lower) / 2.0
            amp = (upper - lower) / 2.0
            freq = 0.5
            return mid + amp * np.sin(2 * np.pi * freq * t)
        else:
            # Default to a small sine wave
            return 0.04 * np.sin(2 * np.pi * 0.5 * t)

    def send_joint_command(self):
        """
        send joint motion commands to ROS2 controller
        """
        t = time.time() - self.start_time

        # Left Panda
        left_msg = JointTrajectory()
        left_msg.joint_names = self.left_joints
        left_point = JointTrajectoryPoint()
        left_point.positions = [self.sine_joint_position(joint, t) for joint in self.left_joints]
        left_point.time_from_start.sec = 1
        left_msg.points.append(left_point)
        self.left_arm_pub.publish(left_msg)

        # Right Panda
        right_msg = JointTrajectory()
        right_msg.joint_names = self.right_joints
        right_point = JointTrajectoryPoint()
        right_point.positions = [self.sine_joint_position(joint, t) for joint in self.right_joints]
        right_point.time_from_start.sec = 1
        right_msg.points.append(right_point)
        self.right_arm_pub.publish(right_msg)

        self.get_logger().info("Published sine wave joint positions.")


def main(args=None):
    rclpy.init(args=args)
    node = SineMotionROScontrol()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
