import os
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Your package name
    pkg_name = "dual_arm_panda_moveit_gazebo"
    pkg_share = get_package_share_directory(pkg_name)

    # Build MoveIt config
    moveit_config = (
        MoveItConfigsBuilder("dual_arm_panda", package_name=pkg_name)
        .robot_description(file_path=os.path.join(pkg_share, "config", "panda.urdf.xacro"))
        .robot_description_semantic(file_path=os.path.join(pkg_share, "config", "panda.srdf"))
        .trajectory_execution(file_path=os.path.join(pkg_share, "config", "moveit_controllers.yaml"))
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # Launch your custom C++ node that uses MoveGroupInterface
    dual_arm_motion_node = Node(
        package="dual_arm_moveit",  # Replace with your actual package name
        executable="lift_rotate",
        name="lift_rotate_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.trajectory_execution,
            {'use_sim_time': True}
        ]
    )

    return LaunchDescription([
        TimerAction(period=5.0, actions=[dual_arm_motion_node])  # Delay to ensure move_group is running
    ])
