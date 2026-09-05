"""
This file contains the launch file for the HARPER control system.

It launches the following nodes:
- robot_state_publisher: Publishes the robot state to the ROS2 parameter server
- controller_manager: Manages the controllers for the HARPER robot
- joint_state_broadcaster: Broadcasts the joint states to the ROS2 parameter server
- left_arm_controller: Controller for the left arm
- right_arm_controller: Controller for the right arm
- rviz2: Visualization tool for the HARPER robot

It also launches the following arguments:
- use_fake_hardware: Use mock_components/GenericSystem instead of HarperDynamixelSystem
- readonly: Refuse Goal Position / torque writes on real hardware
- rviz: Start RViz with the dual-arm control view
- description_package: Package containing the HARPER xacro description
- controllers_file: Controller manager YAML file
- bus_config: Dynamixel YAML with buses.left / buses.right ports, joint IDs, models, profiles, and enabled groups
- models_config: Dynamixel model metadata YAML

Xacro receives hardware_plugin, use_fake_hardware, bus_config, models_config, and
readonly so a single harper_system can swap mock vs real backends.

Last modified: 2026-07-28
"""


import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _as_bool(value):
    return value == 'true'


def _launch_setup(context, *args, **kwargs):
    use_fake_hardware = _as_bool(LaunchConfiguration('use_fake_hardware').perform(context))
    readonly = LaunchConfiguration('readonly').perform(context)
    rviz_enabled = LaunchConfiguration('rviz')
    description_package = LaunchConfiguration('description_package').perform(context)
    controllers_file = LaunchConfiguration('controllers_file').perform(context)
    bus_config = LaunchConfiguration('bus_config').perform(context)
    models_config = LaunchConfiguration('models_config').perform(context)

    description_dir = get_package_share_directory(description_package)
    control_dir = get_package_share_directory('harper_control')
    xacro_file = os.path.join(description_dir, 'urdf', 'harper.urdf.xacro')
    rviz_file = os.path.join(control_dir, 'rviz', 'dual_arm_control.rviz')
    hardware_plugin = (
        'mock_components/GenericSystem'
        if use_fake_hardware
        else 'harper_control/HarperDynamixelSystem'
    )

    robot_description = ParameterValue(
        Command([
            'xacro ',
            xacro_file,
            ' hardware_plugin:=',
            hardware_plugin,
            ' use_fake_hardware:=',
            'true' if use_fake_hardware else 'false',
            ' bus_config:=',
            bus_config,
            ' models_config:=',
            models_config,
            ' readonly:=',
            readonly,
        ]),
        value_type=str,
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            {'robot_description': robot_description},
            controllers_file,
        ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    left_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['left_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    right_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['right_arm_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_file],
        condition=IfCondition(rviz_enabled),
        output='screen',
    )

    return [
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster,
        left_arm_controller,
        right_arm_controller,
        rviz_node,
    ]


def generate_launch_description():
    harper_description_dir = get_package_share_directory('harper_description')
    harper_control_dir = get_package_share_directory('harper_control')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='true',
            description='Use mock_components/GenericSystem instead of HarperDynamixelSystem.',
        ),
        DeclareLaunchArgument(
            'readonly',
            default_value='true',
            description='Refuse Goal Position / torque writes on real hardware.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Start RViz with the dual-arm control view.',
        ),
        DeclareLaunchArgument(
            'description_package',
            default_value='harper_description',
            description='Package containing the HARPER xacro description.',
        ),
        DeclareLaunchArgument(
            'controllers_file',
            default_value=os.path.join(
                harper_description_dir,
                'config',
                'ros2_controllers.yaml',
            ),
            description='Controller manager YAML file.',
        ),
        DeclareLaunchArgument(
            'bus_config',
            default_value=os.path.join(harper_control_dir, 'config', 'dynamixel_bus.yaml'),
            description=(
                'Dynamixel YAML with buses.left / buses.right ports, joint IDs, '
                'models, profiles, and enabled groups.'
            ),
        ),
        DeclareLaunchArgument(
            'models_config',
            default_value=os.path.join(harper_control_dir, 'config', 'dynamixel_models.yaml'),
            description='Dynamixel model metadata YAML.',
        ),
        OpaqueFunction(function=_launch_setup),
    ])
