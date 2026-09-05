"""Start move_group against an already-running harper_control bringup."""

import sys
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node

sys.path.insert(0, str(Path(__file__).parent))
from harper_moveit_config import build_moveit_params  # noqa: E402


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=build_moveit_params(),
        ),
    ])
