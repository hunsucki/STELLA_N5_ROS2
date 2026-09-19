"""Launch capture and initialize both SIYI A8 Mini gimbals."""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start capture and move both gimbals to fixed startup angles."""
    package_share = FindPackageShare('gimbal_camera_capture')

    capture = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                package_share,
                'launch',
                'camera_capture.launch.py',
            ])
        )
    )

    return LaunchDescription([capture])
