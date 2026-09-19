import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share_dir = get_package_share_directory('docking')
    tag_config = os.path.join(package_share_dir, 'config', 'tags_36h11.yaml')

    # AprilTag pose estimation uses CameraInfo.P and assumes rectified pixels.
    # Keep the camera's original timestamp and calibration for synchronization.
    # The installed image_proc node crashes during image_transport teardown.
    # Use a raw ROS Image publisher with cached OpenCV rectification instead.
    rectify_node = Node(
        package='docking',
        executable='image_rectifier',
        name='rectify',
        namespace='apriltag',
        remappings=[
            ('image', '/camera/camera/color/image_raw'),
            ('image/compressed', '/camera/camera/color/image_raw/compressed'),
            ('camera_info', '/camera/camera/color/camera_info'),
            ('image_rect', '/apriltag/image_rect'),
        ],
        parameters=[{'input_transport': LaunchConfiguration('input_transport')}],
        output='screen',
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        namespace='apriltag',
        remappings=[
            ('image_rect', '/apriltag/image_rect'),
            # CameraSubscriber derives this name from the resolved image topic.
            ('/apriltag/camera_info', '/apriltag/camera_info_rect'),
        ],
        parameters=[tag_config],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('input_transport', default_value='auto'),
        rectify_node, apriltag_node,
    ])
