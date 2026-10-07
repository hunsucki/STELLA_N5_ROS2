"""Regression: preload must not deadlock rclpy/LTTng during initialization."""
import rclpy

rclpy.init()
node = rclpy.create_node('dds_guard_initialization_test')
print('Python ROS node initialized with DDS guard', flush=True)
node.destroy_node()
rclpy.shutdown()
