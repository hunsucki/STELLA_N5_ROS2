#!/usr/bin/env python3

"""Launch the Xbox game-controller driver and STELLA teleoperation node."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    package_share = get_package_share_directory('teleop_joy')
    default_params = os.path.join(package_share, 'config', 'xbox_one.yaml')

    params_file = LaunchConfiguration('params_file')
    device_id = LaunchConfiguration('device_id')
    device_name = LaunchConfiguration('device_name')
    joy_topic = LaunchConfiguration('joy_topic')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    launch_joy_node = LaunchConfiguration('launch_joy_node')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Parameter YAML for the controller and teleop nodes'),
        DeclareLaunchArgument(
            'device_id',
            default_value='0',
            description='SDL game-controller device index'),
        DeclareLaunchArgument(
            'device_name',
            default_value='',
            description='SDL device name (takes precedence over device_id)'),
        DeclareLaunchArgument(
            'joy_topic',
            default_value='/joy',
            description='sensor_msgs/Joy input topic'),
        DeclareLaunchArgument(
            'cmd_vel_topic',
            default_value='/cmd_vel',
            description='geometry_msgs/Twist output topic'),
        DeclareLaunchArgument(
            'launch_joy_node',
            default_value='true',
            description='Start joy/game_controller_node'),
        Node(
            package='joy',
            executable='game_controller_node',
            name='game_controller_node',
            output='screen',
            condition=IfCondition(launch_joy_node),
            parameters=[
                params_file,
                {
                    'device_id': ParameterValue(device_id, value_type=int),
                    'device_name': ParameterValue(device_name, value_type=str),
                },
            ],
            remappings=[('joy', joy_topic)]),
        Node(
            package='teleop_joy',
            executable='teleop_joy_node',
            name='teleop_joy_node',
            output='screen',
            parameters=[params_file],
            remappings=[
                ('joy', joy_topic),
                ('cmd_vel', cmd_vel_topic),
            ]),
    ])
