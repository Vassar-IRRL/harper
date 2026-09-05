"""Load MoveIt parameters without moveit_configs_utils."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue


def _load_yaml(package_name, relative_path):
    path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(path, 'r', encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def _read_text(package_name, relative_path):
    path = os.path.join(get_package_share_directory(package_name), relative_path)
    with open(path, 'r', encoding='utf-8') as stream:
        return stream.read()


def build_moveit_params():
    """Return parameter dicts for move_group / RViz."""
    description_share = get_package_share_directory('harper_description')
    xacro_file = os.path.join(description_share, 'urdf', 'harper.urdf.xacro')

    robot_description = {
        'robot_description': ParameterValue(
            Command(['xacro ', xacro_file]),
            value_type=str,
        )
    }
    robot_description_semantic = {
        'robot_description_semantic': _read_text('harper_motion', 'config/harper.srdf')
    }
    robot_description_kinematics = {
        'robot_description_kinematics': _load_yaml('harper_motion', 'config/kinematics.yaml')
    }
    joint_limits = {
        'robot_description_planning': _load_yaml('harper_motion', 'config/joint_limits.yaml')
    }

    ompl_planning = _load_yaml('harper_motion', 'config/ompl_planning.yaml')
    planning_pipelines = {
        'planning_pipelines': ['ompl'],
        'default_planning_pipeline': 'ompl',
        'ompl': ompl_planning,
    }

    trajectory_execution = _load_yaml('harper_motion', 'config/moveit_controllers.yaml')

    move_group = {
        'publish_robot_description_semantic': True,
        'allow_trajectory_execution': True,
        'capabilities': '',
        'disable_capabilities': '',
        'monitor_dynamics': False,
    }

    return [
        robot_description,
        robot_description_semantic,
        robot_description_kinematics,
        joint_limits,
        planning_pipelines,
        trajectory_execution,
        move_group,
    ]


def build_rviz_params():
    """Subset of MoveIt params commonly passed to RViz."""
    return [
        {
            'robot_description': ParameterValue(
                Command([
                    'xacro ',
                    os.path.join(
                        get_package_share_directory('harper_description'),
                        'urdf',
                        'harper.urdf.xacro',
                    ),
                ]),
                value_type=str,
            )
        },
        {
            'robot_description_semantic': _read_text(
                'harper_motion', 'config/harper.srdf'
            )
        },
        {
            'robot_description_kinematics': _load_yaml(
                'harper_motion', 'config/kinematics.yaml'
            )
        },
        {
            'robot_description_planning': _load_yaml(
                'harper_motion', 'config/joint_limits.yaml'
            )
        },
        {
            'ompl': _load_yaml('harper_motion', 'config/ompl_planning.yaml'),
        },
    ]
