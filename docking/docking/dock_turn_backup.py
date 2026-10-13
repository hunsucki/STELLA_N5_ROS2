#!/usr/bin/env python3
"""Approach the tag centerline, turn 180 degrees, then reverse into the dock."""

import math
from collections import deque
import os
import signal
import sys
import time

from ament_index_python.packages import get_package_share_directory
from docking.charging import ChargingVerifier
from docking.centerline_approach import TagCenterlineApproach
from docking.docking_lidar import DockingLidar
from docking.lidar_alignment import LidarPlaneAligner
from docking.lidar_geometry import header_stamp_is_acceptable, UniqueScanStability
from docking.lifecycle import DockingLifecycleManager
from docking.motion import MotionController
from docking.safety import (
    DockingExitCode,
    install_parent_death_signal,
    SingleInstanceLock,
)
from docking.stack_manager import ManagedStack
from docking.tag_centering import AprilTagCentering
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.signals import SignalHandlerOptions
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
        DockingLidar.declare_parameters(self)
        ChargingVerifier.declare_parameters(self)
        self._declare_docking_parameters()
        self._declare_tf_parameters()
        self._declare_safety_parameters()

        self.stop_signal: int | None = None
        self.total_deadline: float | None = None
        self.total_timed_out = False
        self._timeout_logged = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.lidar = DockingLidar(self, self.tf_buffer)
        self.motion = MotionController(self, self.lidar, self.should_stop)
        self.stack = ManagedStack(
            self, self.motion.stop_robot, self.should_stop)
        self.lidar_aligner = LidarPlaneAligner(
            self, self.motion, self.lidar)
        self.motion.set_backup_lidar_heading_estimator(
            self.lidar_aligner.estimate_error)
        self.motion.set_backup_lidar_guide_estimator(
            self.lidar_aligner.estimate_guide_center)
        self.charging = ChargingVerifier(self, self.should_stop)
        self.tag_centering = AprilTagCentering(self)
        self.last_dock_pose: PoseStamped | None = None
        self.last_dock_pose_received_at = 0.0
        self.dock_pose_history = deque(maxlen=5)
        self.dock_pose_sequence = 0
        self.dock_pose_sub = self.create_subscription(
            PoseStamped,
            self.stack.detected_pose_topic,
            self._dock_pose_callback,
            10)

    def run(self) -> DockingExitCode:
        total_timeout = float(self.get_parameter('total_timeout_sec').value)
        if not math.isfinite(total_timeout) or total_timeout <= 0.0:
            self.get_logger().error('total_timeout_sec must be finite and greater than zero')
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

        if not self.motion.wait_for_odom():
            return self._failure_code(
                DockingExitCode.SENSOR_OR_BASE_UNAVAILABLE)

        if not self._wait_for_base_transform():
            return self._failure_code(
                DockingExitCode.SENSOR_OR_BASE_UNAVAILABLE)

        if (
                bool(self.get_parameter('use_lidar_alignment').value)
                or bool(self.get_parameter('use_lidar_backup').value)):
            if not self._wait_for_docking_lidar():
                return self._failure_code(
                    DockingExitCode.SENSOR_OR_BASE_UNAVAILABLE)

        if not self._wait_for_detected_dock_pose():
            return self._failure_code(
                DockingExitCode.SENSOR_OR_BASE_UNAVAILABLE)

        if not self._approach_tag_centerline():
            return self._failure_code(DockingExitCode.DOCKING_FAILED)

        if not self.motion.spin_180():
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
        first_error = None
        for cleanup in (
                self.motion.stop_robot,
                self.charging.cancel_unconfirmed_charging,
                self.stack.cleanup,
                self.motion.stop_robot):
            try:
                cleanup()
            except Exception as exc:  # noqa: BLE001
                # A failed charger command must not skip stopping the robot
                # or terminating the processes that can still publish velocity.
                if first_error is None:
                    first_error = exc
        if first_error is not None:
            raise first_error

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
        AprilTagCentering.declare_parameters(self)
        TagCenterlineApproach.declare_parameters(self)
        # Endpoint declarations also support ManagedStack's standalone tools.
        self.declare_parameter('dock_action', '/dock_robot')
        self.declare_parameter('tag_refinement_target_pose_topic', '/dock_pose')
        self.declare_parameter('dock_pose_topic', 'detected_dock_pose')
        self.declare_parameter('dock_pose_wait_timeout_sec', 10.0)
        self.declare_parameter('dock_pose_ready_samples', 3)
        self.declare_parameter('dock_pose_ready_max_gap_sec', .4)
        self.declare_parameter('tag_front_pose_max_age_sec', .30)

    def _declare_safety_parameters(self) -> None:
        self.declare_parameter('total_timeout_sec', 180.0)
        self.declare_parameter('development_test_mode', True)

    def _declare_tf_parameters(self) -> None:
        self.declare_parameter('fixed_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('tf_wait_timeout_sec', 10.0)


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


    def _wait_for_docking_lidar(self) -> bool:
        if not self.lidar.validate_parameters():
            self.get_logger().error(self.lidar.last_error)
            return False

        timeout = max(float(self.get_parameter(
            'docking_lidar_wait_timeout_sec').value), 0.0)
        topic = str(self.get_parameter('docking_lidar_topic').value)
        frame = self.lidar.expected_frame()
        deadline = time.monotonic() + timeout
        last_log_time = 0.0

        self.get_logger().info(
            f'Waiting for docking LiDAR {topic} in frame {frame}...')

        while rclpy.ok() and not self.should_stop():
            rclpy.spin_once(self, timeout_sec=0.1)
            snapshot = self.lidar.snapshot()
            transform = self.lidar.transform
            if snapshot is not None and transform is not None:
                self.get_logger().info(
                    'Docking LiDAR ready: '
                    f'{self.get_parameter("base_frame").value} <- {frame}, '
                    f'xyz=({transform.x:.4f}, {transform.y:.4f}, '
                    f'{self.lidar.transform_z:.4f})m, '
                    f'yaw={math.degrees(transform.yaw):.2f}deg')
                if (
                        bool(self.get_parameter('use_lidar_backup').value)
                        and not self.motion.validate_lidar_configuration()):
                    return False
                return True

            now = time.monotonic()
            if now - last_log_time >= 1.0:
                self.get_logger().info(
                    f'Still waiting for docking LiDAR: {self.lidar.last_error}')
                last_log_time = now
            if now >= deadline:
                self.get_logger().error(
                    f'Docking LiDAR is not ready: {self.lidar.last_error}')
                return False

        return False

    def _wait_for_detected_dock_pose(self) -> bool:
        timeout = float(self.get_parameter('dock_pose_wait_timeout_sec').value)
        topic = self.stack.detected_pose_topic
        required = int(self.get_parameter('dock_pose_ready_samples').value)
        max_gap = float(self.get_parameter('dock_pose_ready_max_gap_sec').value)
        if required < 2 or not math.isfinite(max_gap) or max_gap <= 0.0:
            self.get_logger().error('Dock pose readiness parameters are invalid')
            return False
        count = 0
        previous_stamp = None
        start = self.get_clock().now()
        last_log_time = time.monotonic()

        self.get_logger().info(f'Waiting for detected dock pose on {topic}...')

        while rclpy.ok() and not self.should_stop():
            if self._pose_is_fresh(
                    self.last_dock_pose, self.last_dock_pose_received_at,
                    float(self.get_parameter('tag_front_pose_max_age_sec').value)):
                stamp = Time.from_msg(self.last_dock_pose.header.stamp).nanoseconds
                if stamp != previous_stamp:
                    gap = ((stamp - previous_stamp) / 1e9
                           if previous_stamp is not None else math.inf)
                    count = count + 1 if 0.0 < gap <= max_gap else 1
                    previous_stamp = stamp
                if count >= required:
                    self.get_logger().info(
                        f'Detected dock pose stream is ready on {topic}: '
                        f'{count} distinct observations, gaps <= {max_gap:.2f}s')
                    return True
            else:
                count = 0
                previous_stamp = None

            now = time.monotonic()
            if now - last_log_time >= 1.0:
                self.get_logger().info(
                    f'Still waiting for {topic}: stable samples={count}/{required}; '
                    'check AprilTag visibility and camera delivery')
                last_log_time = now

            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.get_clock().now() - start).nanoseconds / 1e9 > timeout:
                self.get_logger().error(
                    f'Detected dock pose is not available on {topic}. '
                    'Make sure tag36h11:0 is visible to the RealSense camera.')
                return False

        return False

    def _dock_pose_callback(self, msg: PoseStamped) -> None:
        received_at = time.monotonic()
        if not self._pose_is_fresh(
                msg, received_at,
                float(self.get_parameter('tag_front_pose_max_age_sec').value)):
            return
        if self.last_dock_pose is not None and (
                Time.from_msg(msg.header.stamp).nanoseconds
                <= Time.from_msg(self.last_dock_pose.header.stamp).nanoseconds):
            return
        self.last_dock_pose = msg
        self.last_dock_pose_received_at = received_at
        self.dock_pose_history.append((msg, received_at))
        self.dock_pose_sequence += 1


    def _pose_is_fresh(
            self, pose: PoseStamped | None, received_at: float,
            max_age: float) -> bool:
        # A recently delivered (or republished) message can still describe an
        # old camera observation. Check both clocks without restamping it.
        return (
            pose is not None and received_at > 0.0
            and 0.0 <= time.monotonic() - received_at <= max_age
            and header_stamp_is_acceptable(
                Time.from_msg(pose.header.stamp).nanoseconds, 0,
                self.get_clock().now().nanoseconds, int(max_age * 1e9),
                50_000_000))


    def _approach_tag_centerline(self) -> bool:
        return TagCenterlineApproach(self).run()


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
        cli_args = list(sys.argv if args is None else args)
        if '--params-file' not in cli_args:
            default_params = os.path.join(
                get_package_share_directory('docking'),
                'config',
                'docking.yaml',
            )
            if '--ros-args' in cli_args:
                ros_args_index = cli_args.index('--ros-args') + 1
                cli_args[ros_args_index:ros_args_index] = [
                    '--params-file', default_params]
            else:
                cli_args.extend([
                    '--ros-args', '--params-file', default_params])
        # Keep ROS alive until zero velocity and child cleanup have finished.
        rclpy.init(args=cli_args, signal_handler_options=SignalHandlerOptions.NO)
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
