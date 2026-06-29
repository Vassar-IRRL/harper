"""
Launch file for harper_arm.

Usage:
    ros2 launch harper_arm_description display.launch.py
    ros2 launch harper_arm_description display.launch.py namespace:=robot1
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory('harper_arm_description')

    xacro_file = os.path.join(pkg_dir, 'urdf', 'harper_arm.urdf.xacro')
    rviz_file = os.path.join(pkg_dir, 'rviz', 'display.rviz')

    ns = LaunchConfiguration('namespace')
    prefix = LaunchConfiguration('prefix')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' prefix:=', prefix]),
        value_type=str,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace', default_value='',
            description='ROS 2 namespace for topics and nodes',
        ),
        DeclareLaunchArgument(
            'prefix', default_value='',
            description='URDF link/joint name prefix (passed to xacro)',
        ),

        GroupAction([
            PushRosNamespace(ns),

            # Robot state publisher
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                parameters=[{'robot_description': robot_description}],
                output='screen',
            ),

            # Joint state publisher GUI (sliders for movable joints)
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui',
                output='screen',
            ),

            # RViz2
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_file],
                output='screen',
            ),
        ]),
    ])
