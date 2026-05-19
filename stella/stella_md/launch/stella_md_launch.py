#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    
    driver_node = LifecycleNode(package='stella_md',
                                executable='stella_md_node',
                                name='stella_md_node',
                                output='screen',
                                emulate_tty=True,
                                parameters=[{
                                    'monitoring_rate_hz': 10,
                                    'use_imu_data_orientation': False,
                                    'imu_timeout_sec': 0.0,
                                    'use_imu_yaw_filter': False,
                                    'imu_yaw_max_rate': 2.0,
                                    'imu_yaw_filter_tau_sec': 0.0,
                                    'imu_yaw_jump_warn_threshold': 0.25,
                                }],
                                namespace='/',
                                )

    
    return LaunchDescription([
       driver_node,
    ])
