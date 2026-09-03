#!/usr/bin/python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    
    config_dir = get_package_share_directory('stella_ahrs')
    config_file = os.path.join(config_dir, 'config', 'config.yaml')

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    sync_period_ms = LaunchConfiguration('sync_period_ms')
    publish_rate_hz = LaunchConfiguration('publish_rate_hz')
    read_rate_hz = LaunchConfiguration('read_rate_hz')
    read_success_sleep_us = LaunchConfiguration('read_success_sleep_us')
    cpu_affinity = LaunchConfiguration('cpu_affinity')
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(config_dir, 'rviz', 'imu_test.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    driver_node = LifecycleNode(package='stella_ahrs',
                                executable='stella_ahrs_node',
                                name='stella_ahrs_node',
                                output='screen',
                                emulate_tty=True,
                                prefix=['taskset -c ', cpu_affinity],
                                parameters=[config_file, {
                                    'sync_period_ms': ParameterValue(
                                        sync_period_ms, value_type=int),
                                    'read_rate_hz': ParameterValue(
                                        read_rate_hz, value_type=int),
                                    'publish_rate_hz': ParameterValue(
                                        publish_rate_hz, value_type=int),
                                    'publish_only_on_new_data': True,
                                    'read_success_sleep_us': ParameterValue(
                                        read_success_sleep_us, value_type=int),
                                    'read_idle_sleep_us': 1000,
                                    'frame_id': 'imu_link',
                                    'parent_frame_id': 'base_link',
                                    'publish_tf': False,
                                }],
                                namespace='/',
                                )

    return LaunchDescription([
      DeclareLaunchArgument(
          'sync_period_ms', default_value='5',
          description='MW-AHRS synchronous sensor data period in milliseconds'),
      DeclareLaunchArgument(
          'publish_rate_hz', default_value='100',
          description='Maximum ROS IMU topic publication rate in Hz'),
      DeclareLaunchArgument(
          'read_rate_hz', default_value='0',
          description='Serial packet read cap in Hz; 0 drains all available packets'),
      DeclareLaunchArgument(
          'read_success_sleep_us', default_value='1250',
          description='Reader yield after a decoded packet in microseconds'),
      DeclareLaunchArgument(
          'cpu_affinity', default_value='3',
          description='Linux CPU core list assigned to the AHRS driver'),
      driver_node,
    ])
