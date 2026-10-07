"""Check real DDS backlog behavior in an isolated, non-robot ROS domain."""
import time

import rclpy
from rclpy.context import Context
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped

from docking.motion import MotionController


def test_absolute_motion_state_discards_backlog_and_still_rejects_stale_headers():
    context = Context()
    rclpy.init(context=context, domain_id=217)
    node = Node('docking_latest_state_test', context=context)
    publisher = Node('docking_latest_state_source', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    try:
        MotionController.declare_parameters(node)
        node.declare_parameter('fixed_frame', 'odom')
        node.declare_parameter('base_frame', 'base_link')
        motion = MotionController(node, None)
        odom_pub = publisher.create_publisher(Odometry, '/odom', 10)
        wheel_pub = publisher.create_publisher(Vector3Stamped, '/wheel_odometry/yaw_diagnostics', 10)
        deadline = time.monotonic() + 3
        while (odom_pub.get_subscription_count() < 1 or wheel_pub.get_subscription_count() < 1):
            assert time.monotonic() < deadline
            time.sleep(.02)
        final_stamp = publisher.get_clock().now()
        for i in range(20):
            msg = Odometry()
            msg.header.frame_id, msg.child_frame_id = 'odom', 'base_link'
            msg.header.stamp = final_stamp.to_msg()
            msg.pose.pose.orientation.w = 1.
            msg.pose.pose.position.x = float(i)
            wheel = Vector3Stamped()
            wheel.header.stamp = msg.header.stamp
            wheel.vector.x = float(i)
            odom_pub.publish(msg)
            wheel_pub.publish(wheel)
        # Deliberately do not execute the receiver while the burst arrives.
        time.sleep(.1)
        deadline = time.monotonic() + 1
        while motion.last_odom is None or motion.last_wheel_yaw is None:
            assert time.monotonic() < deadline
            executor.spin_once(timeout_sec=.02)
        assert motion.last_odom.pose.pose.position.x == 19.
        assert motion.last_wheel_yaw == 19.
        assert motion.odom_is_fresh() and motion.wheel_yaw_is_fresh()
        accepted = motion.odom_sequence
        # New receipt must not revive a header that is old/repeated.
        msg.header.stamp.sec -= 2
        msg.pose.pose.position.x = 999.
        wheel.header.stamp = msg.header.stamp
        wheel.vector.x = 999.
        odom_pub.publish(msg)
        wheel_pub.publish(wheel)
        end = time.monotonic() + .1
        while time.monotonic() < end:
            executor.spin_once(timeout_sec=.01)
        assert motion.odom_sequence == accepted
        assert motion.last_odom.pose.pose.position.x == motion.last_wheel_yaw == 19.
    finally:
        executor.shutdown()
        publisher.destroy_node()
        node.destroy_node()
        context.shutdown()
