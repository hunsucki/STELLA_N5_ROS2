import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    package_share_dir = get_package_share_directory('docking')
    tag_config = os.path.join(package_share_dir, 'config', 'tags_36h11.yaml')

    apriltag_node = ComposableNode(
        package='apriltag_ros',
        plugin='AprilTagNode',
        name='apriltag',
        namespace='apriltag',
        remappings=[
            ('image_rect', '/camera/camera/color/image_raw'),
            ('camera_info', '/camera/camera/color/camera_info'),
        ],
        parameters=[tag_config],
        extra_arguments=[{'use_intra_process_comms': False}],
    )

    container = ComposableNodeContainer(
        name='apriltag_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[apriltag_node],
        output='screen',
    )

    return LaunchDescription([container])
