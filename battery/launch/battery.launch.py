from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port',            default_value='/dev/ttyUSB4'),
        DeclareLaunchArgument('baudrate',        default_value='115200'),
        DeclareLaunchArgument('slave_id',        default_value='1'),
        DeclareLaunchArgument('voltage_set',     default_value='25.2'),
        DeclareLaunchArgument('start_current',   default_value='0.7'),
        DeclareLaunchArgument('target_current',  default_value='1.8'),
        DeclareLaunchArgument('ramp_step',       default_value='0.1'),
        DeclareLaunchArgument('ramp_interval',   default_value='5.0'),
        DeclareLaunchArgument('current_offset',  default_value='0.0'),
        DeclareLaunchArgument('status_interval', default_value='2.0'),
        DeclareLaunchArgument('poll_interval',   default_value='5.0'),

        Node(
            package='battery',
            executable='battery_status',
            name='battery_node',
            output='screen',
            parameters=[{
                'port':            LaunchConfiguration('port'),
                'baudrate':        LaunchConfiguration('baudrate'),
                'slave_id':        LaunchConfiguration('slave_id'),
                'voltage_set':     LaunchConfiguration('voltage_set'),
                'start_current':   LaunchConfiguration('start_current'),
                'target_current':  LaunchConfiguration('target_current'),
                'ramp_step':       LaunchConfiguration('ramp_step'),
                'ramp_interval':   LaunchConfiguration('ramp_interval'),
                'current_offset':  LaunchConfiguration('current_offset'),
                'status_interval': LaunchConfiguration('status_interval'),
                'poll_interval':   LaunchConfiguration('poll_interval'),
            }],
        ),
    ])
