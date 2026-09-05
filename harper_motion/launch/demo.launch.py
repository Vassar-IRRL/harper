"""Fake-hardware control bringup + MoveIt move_group for planning demos."""

import os
import sys
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).parent))
from harper_moveit_config import build_moveit_params, build_rviz_params  # noqa: E402


def generate_launch_description():
    control_share = get_package_share_directory('harper_control')
    motion_share = get_package_share_directory('harper_motion')
    rviz_config = os.path.join(motion_share, 'rviz', 'moveit.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='Forwarded to harper_control bringup.',
        ),
        # Must NOT reuse the name "rviz" — bringup also declares it, and
        # launch_arguments={'rviz': 'false'} would disable this node too.
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Start RViz with the MoveIt MotionPlanning panel.',
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(control_share, 'launch', 'bringup.launch.py'),
            ),
            launch_arguments={
                'use_fake_hardware': LaunchConfiguration('use_fake_hardware'),
                'rviz': 'false',
            }.items(),
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=build_moveit_params(),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            parameters=build_rviz_params(),
        ),
    ])
