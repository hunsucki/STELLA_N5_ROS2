#!/usr/bin/env python3
"""Run AprilTag docking, LiDAR yaw alignment, backup, and cleanup."""

import os
import signal
import sys
import time
from typing import Any

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from docking.charging import ChargingVerifier
from docking.lidar_alignment import LidarPlaneAligner
from docking.lifecycle import DockingLifecycleManager
from docking.motion import MotionController
from docking.safety import (
    DockingExitCode,
    install_parent_death_signal,
    SingleInstanceLock,
)
from docking.stack_manager import ManagedStack
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import DockRobot
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


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
        ChargingVerifier.declare_parameters(self)
        self._declare_docking_parameters()
        self._declare_tf_parameters()
        self._declare_safety_parameters()

        self.stop_signal: int | None = None
        self.total_deadline: float | None = None
        self.total_timed_out = False
        self._timeout_logged = False

        self.motion = MotionController(self, self.should_stop)
        self.stack = ManagedStack(
            self, self.motion.stop_robot, self.should_stop)
        self.lifecycle = DockingLifecycleManager(self)
        self.lidar_aligner = LidarPlaneAligner(self, self.motion)
        self.charging = ChargingVerifier(self, self.should_stop)
        self.last_dock_pose: PoseStamped | None = None
        self.dock_pose_sub = self.create_subscription(
            PoseStamped,
            self.get_parameter('dock_pose_topic').value,
            self._dock_pose_callback,
            10)

        self.dock_client = ActionClient(
            self, DockRobot, self.get_parameter('dock_action').value)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def run(self) -> DockingExitCode:
        total_timeout = float(self.get_parameter('total_timeout_sec').value)
        if total_timeout <= 0.0:
            self.get_logger().error('total_timeout_sec must be greater than zero')
            return DockingExitCode.INVALID_REQUEST
        self.total_deadline = time.monotonic() + total_timeout

        development_mode = bool(
            self.get_parameter('development_test_mode').value)
        if development_mode:
            self.get_logger().warn(
                'DEVELOPMENT TEST MODE: successful LiDAR backup will return exit 0 '
                'without charger contact or charging-current confirmation')

        if self.charging.wait_for_existing_charge():
            return DockingExitCode.SUCCESS
        if self.should_stop():
            return self._failure_code(DockingExitCode.INTERNAL_ERROR)

        if bool(self.get_parameter('manage_stack').value):
            if not self.stack.start():
                return self._failure_code(
                    DockingExitCode.SENSOR_OR_BASE_UNAVAILABLE)

        if bool(self.get_parameter('activate_docking_server').value):
            if not self.lifecycle.configure_and_activate(self.should_stop):
                return self._failure_code(
                    DockingExitCode.SENSOR_OR_BASE_UNAVAILABLE)

        if not self._wait_for_dock_server():
            return self._failure_code(
                DockingExitCode.SENSOR_OR_BASE_UNAVAILABLE)

        if not self.motion.wait_for_odom():
            return self._failure_code(
                DockingExitCode.SENSOR_OR_BASE_UNAVAILABLE)

        if not self._wait_for_base_transform():
            return self._failure_code(
                DockingExitCode.SENSOR_OR_BASE_UNAVAILABLE)

        self._warmup_docking_server_tf_buffer()

        if not self._wait_for_detected_dock_pose():
            return self._failure_code(
                DockingExitCode.SENSOR_OR_BASE_UNAVAILABLE)

        if not self._dock_near_tag():
            return self._failure_code(DockingExitCode.DOCKING_FAILED)

        if not self.motion.spin_180():
            return self._failure_code(DockingExitCode.DOCKING_FAILED)

        if bool(self.get_parameter('use_lidar_alignment').value):
            if not self.lidar_aligner.align():
                return self._failure_code(DockingExitCode.DOCKING_FAILED)

        if not self.motion.backup():
            return self._failure_code(DockingExitCode.DOCKING_FAILED)

        if development_mode:
            self.get_logger().warn(
                'Development test completed at the configured backup distance; '
                'charging was not verified')
            return DockingExitCode.SUCCESS

        if not self.charging.verify():
            return self._failure_code(
                DockingExitCode.CHARGING_NOT_CONFIRMED)

        return DockingExitCode.SUCCESS

    def cleanup_managed_processes(self) -> None:
        self.charging.cancel_unconfirmed_charging()
        self.motion.stop_robot()
        self.stack.cleanup()
        self.motion.stop_robot()

    def request_stop(self, signum: int) -> None:
        if self.stop_signal is None:
            self.stop_signal = signum

    def should_stop(self) -> bool:
        if self.stop_signal is not None or not rclpy.ok():
            return True
        if (
                self.total_deadline is not None
                and time.monotonic() >= self.total_deadline):
            self.total_timed_out = True
            if not self._timeout_logged:
                self.get_logger().error('Overall docking timeout expired')
                self._timeout_logged = True
            return True
        return False

    def _declare_docking_parameters(self) -> None:
        self.declare_parameter('dock_action', '/dock_robot')
        self.declare_parameter('dock_id', 'home_dock')
        self.declare_parameter('navigate_to_staging_pose', False)
        self.declare_parameter('max_staging_time', 40.0)
        self.declare_parameter('dock_pose_topic', 'detected_dock_pose')
        self.declare_parameter('dock_pose_wait_timeout_sec', 10.0)

    def _declare_safety_parameters(self) -> None:
        self.declare_parameter('total_timeout_sec', 100.0)
        self.declare_parameter('development_test_mode', True)

    def _declare_tf_parameters(self) -> None:
        self.declare_parameter('fixed_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tf_wait_timeout_sec', 10.0)
        self.declare_parameter('docking_server_tf_warmup_sec', 2.0)

    def _wait_for_dock_server(self) -> bool:
        timeout = float(self.get_parameter('server_wait_timeout_sec').value)

        self.get_logger().info('Waiting for dock_robot action server...')
        deadline = time.monotonic() + timeout
        while not self.should_stop() and time.monotonic() < deadline:
            if self.dock_client.wait_for_server(timeout_sec=0.2):
                return True
        self.get_logger().error('dock_robot action server is not available')
        return False

    def _wait_for_base_transform(self) -> bool:
        timeout = float(self.get_parameter('tf_wait_timeout_sec').value)
        fixed_frame = str(self.get_parameter('fixed_frame').value)
        base_frame = str(self.get_parameter('base_frame').value)
        start = self.get_clock().now()
        last_log_time = time.monotonic()

        self.get_logger().info(
            f'Waiting for TF transform {fixed_frame} -> {base_frame}...')

        while rclpy.ok() and not self.should_stop():
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
        while (
                rclpy.ok()
                and not self.should_stop()
                and time.monotonic() < deadline):
            rclpy.spin_once(self, timeout_sec=0.1)

    def _wait_for_detected_dock_pose(self) -> bool:
        timeout = float(self.get_parameter('dock_pose_wait_timeout_sec').value)
        topic = str(self.get_parameter('dock_pose_topic').value)
        start = self.get_clock().now()
        last_log_time = time.monotonic()

        self.get_logger().info(f'Waiting for detected dock pose on {topic}...')

        while rclpy.ok() and not self.should_stop():
            if self.last_dock_pose is not None:
                self.get_logger().info(f'Detected dock pose is available on {topic}')
                return True

            now = time.monotonic()
            if now - last_log_time >= 1.0:
                self.get_logger().info(
                    f'Still waiting for {topic}; check AprilTag visibility and QoS')
                last_log_time = now

            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - start).nanoseconds / 1e9 > timeout:
                self.get_logger().error(
                    f'Detected dock pose is not available on {topic}. '
                    'Make sure tag36h11:0 is visible to the RealSense camera.')
                return False

        return False

    def _dock_pose_callback(self, msg: PoseStamped) -> None:
        self.last_dock_pose = msg

    def _dock_near_tag(self) -> bool:
        goal = DockRobot.Goal()
        goal.use_dock_id = True
        goal.dock_id = str(self.get_parameter('dock_id').value)
        goal.navigate_to_staging_pose = bool(
            self.get_parameter('navigate_to_staging_pose').value)
        max_staging_time = float(self.get_parameter('max_staging_time').value)
        if self.total_deadline is not None:
            max_staging_time = min(
                max_staging_time,
                max(self.total_deadline - time.monotonic(), 0.1))
        goal.max_staging_time = max_staging_time

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
        if not self._wait_for_future(send_future):
            return None
        goal_handle = send_future.result()

        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error('Goal was rejected')
            return None

        result_future = goal_handle.get_result_async()
        if not self._wait_for_future(result_future):
            cancel_future = goal_handle.cancel_goal_async()
            cancel_deadline = time.monotonic() + 1.0
            while (
                    rclpy.ok()
                    and not cancel_future.done()
                    and time.monotonic() < cancel_deadline):
                rclpy.spin_once(self, timeout_sec=0.05)
            self.get_logger().warn('Docking action was cancelled during shutdown')
            return None
        return result_future.result()

    def _wait_for_future(self, future: Any) -> bool:
        while rclpy.ok() and not self.should_stop():
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                return True
        return False

    def _failure_code(
            self, default: DockingExitCode) -> DockingExitCode:
        if self.total_timed_out:
            return DockingExitCode.TIMEOUT
        if self.stop_signal == signal.SIGINT:
            return DockingExitCode.SIGINT
        if self.stop_signal == signal.SIGTERM:
            return DockingExitCode.SIGTERM
        if self.stop_signal == signal.SIGHUP:
            return DockingExitCode.SIGHUP
        return default


def main(args=None) -> int:
    lock_path = os.environ.get(
        'DOCKING_LOCK_FILE', '/tmp/stella_dock_turn_backup.lock')
    lock = SingleInstanceLock(lock_path)
    try:
        if not lock.acquire():
            print(
                'dock_turn_backup is already running; refusing duplicate execution',
                file=sys.stderr)
            return int(DockingExitCode.INVALID_REQUEST)
    except OSError as exc:
        print(f'Could not acquire docking lock {lock_path}: {exc}', file=sys.stderr)
        return int(DockingExitCode.INVALID_REQUEST)

    node: DockTurnBackup | None = None
    exit_code = DockingExitCode.INTERNAL_ERROR
    pending_signal: int | None = None

    def handle_signal(signum, _frame) -> None:
        nonlocal pending_signal
        pending_signal = signum
        if node is not None:
            node.request_stop(signum)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)
    signal.signal(signal.SIGHUP, handle_signal)

    if not install_parent_death_signal(signal.SIGTERM):
        print(
            'Warning: could not enable parent-death signal protection',
            file=sys.stderr)

    try:
        rclpy.init(args=args)
        node = DockTurnBackup()
        if pending_signal is not None:
            node.request_stop(pending_signal)

        exit_code = node.run()
    except KeyboardInterrupt:
        exit_code = DockingExitCode.SIGINT
    except Exception as exc:  # noqa: BLE001
        if node is not None:
            node.get_logger().error(f'Unhandled docking failure: {exc!r}')
        else:
            print(f'Failed to initialize docking: {exc!r}', file=sys.stderr)
        exit_code = DockingExitCode.INTERNAL_ERROR
    finally:
        if node is not None:
            try:
                node.cleanup_managed_processes()
            except Exception as exc:  # noqa: BLE001
                node.get_logger().error(f'Docking cleanup failed: {exc!r}')
                if exit_code == DockingExitCode.SUCCESS:
                    exit_code = DockingExitCode.INTERNAL_ERROR
            finally:
                node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        lock.release()

    return int(exit_code)


if __name__ == '__main__':
    sys.exit(main())
