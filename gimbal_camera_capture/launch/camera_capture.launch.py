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
            'run_start_topic',
            default_value='/camera/run/start',
        ),
        DeclareLaunchArgument(
            'run_finish_topic',
            default_value='/camera/run/finish',
        ),
        DeclareLaunchArgument(
            'run_result_topic',
            default_value='/camera/run/result',
        ),
        DeclareLaunchArgument(
            'capture_run_start_service',
            default_value='/camera/capture_run/start',
        ),
        DeclareLaunchArgument(
            'capture_pair_service',
            default_value='/camera/capture_pair',
        ),
        DeclareLaunchArgument(
            'capture_run_finish_service',
            default_value='/camera/capture_run/finish',
        ),
        DeclareLaunchArgument(
            'capture_run_abort_service',
            default_value='/camera/capture_run/abort',
        ),
        DeclareLaunchArgument(
            'output_directory',
            default_value='~/capture',
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
            'run_start_topic': LaunchConfiguration('run_start_topic'),
            'run_finish_topic': LaunchConfiguration('run_finish_topic'),
            'run_result_topic': LaunchConfiguration('run_result_topic'),
            'capture_run_start_service': LaunchConfiguration(
                'capture_run_start_service'
            ),
            'capture_pair_service': LaunchConfiguration(
                'capture_pair_service'
            ),
            'capture_run_finish_service': LaunchConfiguration(
                'capture_run_finish_service'
            ),
            'capture_run_abort_service': LaunchConfiguration(
                'capture_run_abort_service'
            ),
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
