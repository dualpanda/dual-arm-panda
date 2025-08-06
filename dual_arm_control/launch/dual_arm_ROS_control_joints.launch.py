import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from scripts import GazeboRosPaths
from launch.actions import SetEnvironmentVariable


def generate_launch_description():
    # Package paths
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

    # Gazebo: launch empty world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
        'gui': 'true'
        # 'server': 'true',
        # 'verbose': 'true'
    }.items()
    )

    # Robot state publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description, {'use_sim_time': True}],
        output="screen",
    )

    # Spawn robot in Gazebo from robot_description
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_dual_arm',
        arguments=[
            '-topic', "robot_description",
            '-entity', 'panda'
            # '-x', '0', '-y', '0', '-z', '0.1',
        ],
        output='screen'
    )

    controller_spawners = [
    Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    ),
    Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller"],
        output="screen",
    ),
    Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller"],
        output="screen",
    ),
    Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_gripper_controller"],
        output="screen",
    ),
    Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_gripper_controller"],
        output="screen",
    ),
]

    # RViz with MoveIt config
    rviz_config = os.path.join(get_package_share_directory("dual_arm_control"), "config", "dual_arm.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[{'use_sim_time': True}],
    )

    dual_arm_motion_node = Node(
        package="dual_arm_control",
        executable="set_joint_angles",
        name="set_joint_angles_node",
        output="screen",
        parameters=[{'use_sim_time': True}]
    )

    return LaunchDescription(
        [
            robot_state_publisher,
            gazebo,
            TimerAction(period=2.0, actions=[spawn_entity]),  
            TimerAction(period=2.0, actions=controller_spawners),  
            TimerAction(period=2.0, actions=[rviz]),  
            dual_arm_motion_node,  
        ]
    )
