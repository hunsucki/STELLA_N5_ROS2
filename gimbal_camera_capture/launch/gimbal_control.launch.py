"""Launch topic control for both SIYI gimbals."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Build the standalone dual-gimbal control launch description."""
    arguments = [
        DeclareLaunchArgument('left_ip', default_value='192.168.144.25'),
        DeclareLaunchArgument('left_port', default_value='37260'),
        DeclareLaunchArgument(
            'left_bind_address',
            default_value='192.168.144.10',
        ),
        DeclareLaunchArgument('left_yaw_direction', default_value='1'),
        DeclareLaunchArgument('left_pitch_direction', default_value='1'),
        DeclareLaunchArgument('right_ip', default_value='192.168.144.26'),
        DeclareLaunchArgument('right_port', default_value='37260'),
        DeclareLaunchArgument(
            'right_bind_address',
            default_value='192.168.144.11',
        ),
        DeclareLaunchArgument('right_yaw_direction', default_value='1'),
        DeclareLaunchArgument('right_pitch_direction', default_value='1'),
        DeclareLaunchArgument('command_timeout_sec', default_value='0.5'),
        DeclareLaunchArgument('step_duration_sec', default_value='0.15'),
        DeclareLaunchArgument('step_speed', default_value='40'),
        DeclareLaunchArgument(
            'result_topic',
            default_value='/gimbal/control/result',
        ),
    ]

    parameters = {
        'left_ip': LaunchConfiguration('left_ip'),
        'left_port': ParameterValue(
            LaunchConfiguration('left_port'),
            value_type=int,
        ),
        'left_bind_address': LaunchConfiguration('left_bind_address'),
        'left_yaw_direction': ParameterValue(
            LaunchConfiguration('left_yaw_direction'),
            value_type=int,
        ),
        'left_pitch_direction': ParameterValue(
            LaunchConfiguration('left_pitch_direction'),
            value_type=int,
        ),
        'right_ip': LaunchConfiguration('right_ip'),
        'right_port': ParameterValue(
            LaunchConfiguration('right_port'),
            value_type=int,
        ),
        'right_bind_address': LaunchConfiguration('right_bind_address'),
        'right_yaw_direction': ParameterValue(
            LaunchConfiguration('right_yaw_direction'),
            value_type=int,
        ),
        'right_pitch_direction': ParameterValue(
            LaunchConfiguration('right_pitch_direction'),
            value_type=int,
        ),
        'command_timeout_sec': ParameterValue(
            LaunchConfiguration('command_timeout_sec'),
            value_type=float,
        ),
        'step_duration_sec': ParameterValue(
            LaunchConfiguration('step_duration_sec'),
            value_type=float,
        ),
        'step_speed': ParameterValue(
            LaunchConfiguration('step_speed'),
            value_type=int,
        ),
        'result_topic': LaunchConfiguration('result_topic'),
    }

    control_node = Node(
        package='gimbal_camera_capture',
        executable='control_node',
        name='gimbal_control',
        output='screen',
        parameters=[parameters],
    )

    return LaunchDescription(arguments + [control_node])
