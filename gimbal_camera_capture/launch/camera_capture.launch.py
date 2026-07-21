# Copyright 2026 NTRex Co., Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch the standalone dual gimbal-camera capture node."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the standalone camera capture launch description."""
    arguments = [
        DeclareLaunchArgument(
            'trigger_topic',
            default_value='/camera/capture',
        ),
        DeclareLaunchArgument(
            'result_topic',
            default_value='/camera/capture/result',
        ),
        DeclareLaunchArgument(
            'output_directory',
            default_value='~/capcture',
        ),
        DeclareLaunchArgument(
            'camera_1_url',
            default_value='rtsp://192.168.144.25:8554/main.264',
        ),
        DeclareLaunchArgument(
            'camera_2_url',
            default_value='rtsp://192.168.144.26:8554/main.264',
        ),
        DeclareLaunchArgument('open_timeout_ms', default_value='5000'),
        DeclareLaunchArgument('read_timeout_ms', default_value='5000'),
        DeclareLaunchArgument('frame_read_attempts', default_value='5'),
        DeclareLaunchArgument('jpeg_quality', default_value='95'),
    ]

    camera_node = Node(
        package='gimbal_camera_capture',
        executable='capture_node',
        name='gimbal_camera_capture',
        output='screen',
        parameters=[{
            'trigger_topic': LaunchConfiguration('trigger_topic'),
            'result_topic': LaunchConfiguration('result_topic'),
            'output_directory': LaunchConfiguration('output_directory'),
            'camera_1_url': LaunchConfiguration('camera_1_url'),
            'camera_2_url': LaunchConfiguration('camera_2_url'),
            'open_timeout_ms': ParameterValue(
                LaunchConfiguration('open_timeout_ms'),
                value_type=int,
            ),
            'read_timeout_ms': ParameterValue(
                LaunchConfiguration('read_timeout_ms'),
                value_type=int,
            ),
            'frame_read_attempts': ParameterValue(
                LaunchConfiguration('frame_read_attempts'),
                value_type=int,
            ),
            'jpeg_quality': ParameterValue(
                LaunchConfiguration('jpeg_quality'),
                value_type=int,
            ),
        }],
    )

    return LaunchDescription(arguments + [camera_node])
