#!/usr/bin/env python3
"""Run AprilTag docking, LiDAR yaw alignment, backup, and cleanup."""

import os
import sys
import time
from typing import Any

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.action import DockRobot
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from docking.lifecycle import DockingLifecycleManager
from docking.lidar_alignment import LidarPlaneAligner
from docking.motion import MotionController
from docking.stack_manager import ManagedStack


class DockTurnBackup(Node):
    def __init__(self) -> None:
        super().__init__('dock_turn_backup')

        package_share_dir = get_package_share_directory('docking')
        default_docking_params = os.path.join(
            package_share_dir, 'config', 'docking.yaml')

        ManagedStack.declare_parameters(self, default_docking_params)
        DockingLifecycleManager.declare_parameters(self)
        MotionController.declare_parameters(self)
        LidarPlaneAligner.declare_parameters(self)
        self._declare_docking_parameters()
        self._declare_tf_parameters()

        self.motion = MotionController(self)
        self.stack = ManagedStack(self, self.motion.stop_robot)
        self.lifecycle = DockingLifecycleManager(self)
        self.lidar_aligner = LidarPlaneAligner(self, self.motion)

        self.dock_client = ActionClient(
            self, DockRobot, self.get_parameter('dock_action').value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def run(self) -> bool:
        if bool(self.get_parameter('manage_stack').value):
            if not self.stack.start():
                return False

        if bool(self.get_parameter('activate_docking_server').value):
            if not self.lifecycle.configure_and_activate():
                return False

        if not self._wait_for_dock_server():
            return False

        if not self.motion.wait_for_odom():
            return False

        if not self._wait_for_base_transform():
            return False

        self._warmup_docking_server_tf_buffer()

        if not self._dock_near_tag():
            return False

        if not self.motion.spin_180():
            return False

        if bool(self.get_parameter('use_lidar_alignment').value):
            if not self.lidar_aligner.align():
                return False

        return self.motion.backup()

    def cleanup_managed_processes(self) -> None:
        self.stack.cleanup()

    def _declare_docking_parameters(self) -> None:
        self.declare_parameter('dock_action', '/dock_robot')
        self.declare_parameter('dock_id', 'home_dock')
        self.declare_parameter('navigate_to_staging_pose', False)
        self.declare_parameter('max_staging_time', 1000.0)

    def _declare_tf_parameters(self) -> None:
        self.declare_parameter('fixed_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tf_wait_timeout_sec', 10.0)
        self.declare_parameter('docking_server_tf_warmup_sec', 2.0)

    def _wait_for_dock_server(self) -> bool:
        timeout = float(self.get_parameter('server_wait_timeout_sec').value)

        self.get_logger().info('Waiting for dock_robot action server...')
        if not self.dock_client.wait_for_server(timeout_sec=timeout):
            self.get_logger().error('dock_robot action server is not available')
            return False
        return True

    def _wait_for_base_transform(self) -> bool:
        timeout = float(self.get_parameter('tf_wait_timeout_sec').value)
        fixed_frame = str(self.get_parameter('fixed_frame').value)
        base_frame = str(self.get_parameter('base_frame').value)
        start = self.get_clock().now()
        last_log_time = time.monotonic()

        self.get_logger().info(
            f'Waiting for TF transform {fixed_frame} -> {base_frame}...')

        while rclpy.ok():
            try:
                self.tf_buffer.lookup_transform(
                    fixed_frame, base_frame, rclpy.time.Time())
                self.get_logger().info(
                    f'TF transform {fixed_frame} -> {base_frame} is available')
                return True
            except TransformException as exc:
                now = time.monotonic()
                if now - last_log_time >= 1.0:
                    self.get_logger().info(
                        f'Still waiting for TF {fixed_frame} -> {base_frame}: {exc}')
                    last_log_time = now

            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - start).nanoseconds / 1e9 > timeout:
                self.get_logger().error(
                    f'TF transform {fixed_frame} -> {base_frame} is not available')
                return False

        return False

    def _warmup_docking_server_tf_buffer(self) -> None:
        warmup_sec = float(self.get_parameter('docking_server_tf_warmup_sec').value)
        if warmup_sec <= 0.0:
            return

        self.get_logger().info(
            f'Warming up docking_server TF buffer for {warmup_sec:.1f}s...')
        deadline = time.monotonic() + warmup_sec
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _dock_near_tag(self) -> bool:
        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = str(self.get_parameter('dock_id').value)
        goal.navigate_to_staging_pose = bool(
            self.get_parameter('navigate_to_staging_pose').value)
        goal.max_staging_time = float(self.get_parameter('max_staging_time').value)

        self.get_logger().info(
            f'Docking near tag using dock_id="{goal.dock_id}" '
            f'(navigate_to_staging_pose={goal.navigate_to_staging_pose})')
        result = self._send_and_wait(self.dock_client, goal)
        if result is None:
            return False

        dock_result = result.result
        if result.status != GoalStatus.STATUS_SUCCEEDED or not dock_result.success:
            self.get_logger().error(
                'Docking step failed: '
                f'status={result.status}, error_code={dock_result.error_code}, '
                f'error_msg="{dock_result.error_msg}"')
            return False

        self.get_logger().info(
            'Docking step complete; robot is at the tag-front stop point')
        return True

    def _send_and_wait(self, client: ActionClient, goal: Any) -> Any:
        send_future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Goal was rejected')
            return None

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result()


def main(args=None) -> int:
    rclpy.init(args=args)
    node = DockTurnBackup()

    try:
        ok = node.run()
    except KeyboardInterrupt:
        node.get_logger().warn('Interrupted')
        ok = False
    finally:
        node.cleanup_managed_processes()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
