#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('wheel_odometry'),
        'config',
        'wheel_odometry.yaml')
    enabled = LaunchConfiguration('enabled')

    return LaunchDescription([
        DeclareLaunchArgument(
            'enabled', default_value='true',
            description='Publish wheel/IMU odometry and odom TF'),
        Node(
            package='wheel_odometry',
            executable='wheel_odometry_node',
            name='wheel_odometry',
            output='screen',
            parameters=[config, {
                'enabled': ParameterValue(enabled, value_type=bool),
            }],
        ),
    ])
