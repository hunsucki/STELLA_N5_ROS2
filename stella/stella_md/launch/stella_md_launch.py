#!/usr/bin/python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
from launch_ros.actions import LifecycleNode

def generate_launch_description():
    encoder_poll_rate_hz = LaunchConfiguration('encoder_poll_rate_hz')
    enable_legacy_odom = LaunchConfiguration('enable_legacy_odom')
    cpu_affinity = LaunchConfiguration('cpu_affinity')

    driver_node = LifecycleNode(package='stella_md',
                                executable='stella_md_node',
                                name='stella_md_node',
                                output='screen',
                                emulate_tty=True,
                                prefix=['taskset -c ', cpu_affinity],
                                parameters=[{
                                    'encoder_poll_rate_hz': ParameterValue(
                                        encoder_poll_rate_hz, value_type=int),
                                    'enable_legacy_odom': ParameterValue(
                                        enable_legacy_odom, value_type=bool),
                                    'use_imu_data_orientation': False,
                                    'imu_timeout_sec': 0.0,
                                    'use_imu_yaw_filter': False,
                                    'imu_yaw_max_rate': 2.0,
                                    'imu_yaw_filter_tau_sec': 0.0,
                                    'imu_yaw_jump_warn_threshold': 0.25,
                                    'cmd_vel_timeout_sec': 0.5,
                                }],
                                namespace='/',
                                )

    
    return LaunchDescription([
       DeclareLaunchArgument(
           'encoder_poll_rate_hz', default_value='30',
           description='Paired left/right encoder polling target in Hz'),
       DeclareLaunchArgument(
           'enable_legacy_odom', default_value='false',
           description='Enable the original stella_md /odom and TF calculation'),
       DeclareLaunchArgument(
           'cpu_affinity', default_value='2',
           description='Linux CPU core list assigned to the motor driver'),
       driver_node,
    ])
