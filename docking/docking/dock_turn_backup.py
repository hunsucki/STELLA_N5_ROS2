#!/usr/bin/env python3
"""Run AprilTag docking, LiDAR yaw alignment, backup, and cleanup."""

import math
import os
import signal
import sys
import time
from typing import Any

from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from docking.charging import ChargingVerifier
from docking.docking_lidar import DockingLidar
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
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
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
        self.lifecycle = DockingLifecycleManager(self)
        self.lidar_aligner = LidarPlaneAligner(
            self, self.motion, self.lidar)
        self.motion.set_backup_lidar_heading_estimator(
            self.lidar_aligner.estimate_error)
        self.charging = ChargingVerifier(self, self.should_stop)
        self.last_dock_pose: PoseStamped | None = None
        self.last_dock_pose_received_at = 0.0
        self.dock_pose_sequence = 0
        self.dock_pose_sub = self.create_subscription(
            PoseStamped,
            self.get_parameter('dock_pose_topic').value,
            self._dock_pose_callback,
            10)
        self.last_refinement_target_pose: PoseStamped | None = None
        self.last_refinement_target_received_at = 0.0
        refinement_qos = QoSProfile(depth=1)
        refinement_qos.reliability = ReliabilityPolicy.RELIABLE
        # The ROS Jazzy opennav_docking package installed on this robot offers
        # /dock_pose with VOLATILE durability.  Requesting TRANSIENT_LOCAL here
        # is incompatible and drops every target even though the subscriber is
        # created before the docking action starts.
        refinement_qos.durability = DurabilityPolicy.VOLATILE
        self.refinement_target_sub = self.create_subscription(
            PoseStamped,
            self.get_parameter('tag_refinement_target_pose_topic').value,
            self._refinement_target_callback,
            refinement_qos)

        self.dock_client = ActionClient(
            self, DockRobot, self.get_parameter('dock_action').value)

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

        if (
                bool(self.get_parameter('use_lidar_alignment').value)
                or bool(self.get_parameter('use_lidar_backup').value)):
            if not self._wait_for_docking_lidar():
                return self._failure_code(
                    DockingExitCode.SENSOR_OR_BASE_UNAVAILABLE)

        self._warmup_docking_server_tf_buffer()

        if not self._wait_for_detected_dock_pose():
            return self._failure_code(
                DockingExitCode.SENSOR_OR_BASE_UNAVAILABLE)

        if not self._dock_near_tag():
            return self._failure_code(DockingExitCode.DOCKING_FAILED)

        if not self._refine_tag_front_pose():
            return self._failure_code(DockingExitCode.DOCKING_FAILED)

        if not self._verify_tag_front_stop_pose():
            return self._failure_code(DockingExitCode.DOCKING_FAILED)

        if not self.motion.advance_before_spin():
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
        # Optional bounded pose trim after Nav2 DockRobot succeeds.  The target
        # is Nav2's already filtered /dock_pose in odom, so disabling this flag
        # restores the previously successful motion sequence exactly.
        self.declare_parameter('use_tag_pose_refinement', True)
        self.declare_parameter('tag_refinement_target_pose_topic', '/dock_pose')
        self.declare_parameter('tag_refinement_target_wait_timeout_sec', 1.0)
        self.declare_parameter('tag_refinement_target_max_age_sec', 1.5)
        self.declare_parameter('tag_refinement_timeout_sec', 18.0)
        self.declare_parameter('tag_refinement_longitudinal_tolerance', 0.04)
        self.declare_parameter('tag_refinement_lateral_tolerance', 0.025)
        self.declare_parameter(
            'tag_refinement_yaw_tolerance', math.radians(2.0))
        self.declare_parameter('tag_refinement_stable_cycles', 5)
        self.declare_parameter('tag_refinement_linear_kp', 0.50)
        self.declare_parameter('tag_refinement_angular_k_alpha', 1.00)
        self.declare_parameter('tag_refinement_angular_k_beta', -0.30)
        self.declare_parameter('tag_refinement_final_yaw_kp', 1.00)
        self.declare_parameter('tag_refinement_max_linear_speed', 0.025)
        self.declare_parameter('tag_refinement_max_angular_speed', 0.08)
        self.declare_parameter('tag_refinement_max_initial_longitudinal', 0.18)
        self.declare_parameter('tag_refinement_max_initial_lateral', 0.10)
        self.declare_parameter(
            'tag_refinement_max_initial_yaw', math.radians(25.0))
        self.declare_parameter('tag_refinement_max_travel', 0.18)
        self.declare_parameter(
            'tag_refinement_max_yaw_excursion', math.radians(30.0))
        self.declare_parameter('tag_refinement_control_rate_hz', 20.0)
        self.declare_parameter('tag_refinement_abort_on_failure', True)
        # The docking plugin's original 0.15 m radial completion region leaves
        # enough room for the 180-degree spin.  Do not re-impose the former
        # 2.5 cm camera centering gate after the action has already stopped.
        self.declare_parameter('verify_tag_front_stop_pose', False)
        self.declare_parameter('tag_front_stop_distance', 0.80)
        self.declare_parameter('tag_front_longitudinal_tolerance', 0.04)
        self.declare_parameter('tag_front_lateral_tolerance', 0.025)
        self.declare_parameter('tag_front_pose_max_age_sec', 0.30)
        self.declare_parameter('tag_front_stable_cycles', 5)
        self.declare_parameter('tag_front_verify_timeout_sec', 3.0)

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
        self.last_dock_pose_received_at = time.monotonic()
        self.dock_pose_sequence += 1

    def _refinement_target_callback(self, msg: PoseStamped) -> None:
        self.last_refinement_target_pose = msg
        self.last_refinement_target_received_at = time.monotonic()

    def _refine_tag_front_pose(self) -> bool:
        if not bool(self.get_parameter('use_tag_pose_refinement').value):
            self.get_logger().info(
                'Optional AprilTag pose refinement is disabled')
            return True

        parameter_names = (
            'tag_refinement_target_wait_timeout_sec',
            'tag_refinement_target_max_age_sec',
            'tag_refinement_timeout_sec',
            'tag_refinement_longitudinal_tolerance',
            'tag_refinement_lateral_tolerance',
            'tag_refinement_yaw_tolerance',
            'tag_refinement_linear_kp',
            'tag_refinement_angular_k_alpha',
            'tag_refinement_angular_k_beta',
            'tag_refinement_final_yaw_kp',
            'tag_refinement_max_linear_speed',
            'tag_refinement_max_angular_speed',
            'tag_refinement_max_initial_longitudinal',
            'tag_refinement_max_initial_lateral',
            'tag_refinement_max_initial_yaw',
            'tag_refinement_max_travel',
            'tag_refinement_max_yaw_excursion',
            'tag_refinement_control_rate_hz',
        )
        values = {
            name: float(self.get_parameter(name).value)
            for name in parameter_names
        }
        stable_cycles = int(
            self.get_parameter('tag_refinement_stable_cycles').value)
        positive_names = tuple(
            name for name in parameter_names
            if name != 'tag_refinement_angular_k_beta')
        if (
                not all(math.isfinite(value) for value in values.values())
                or any(values[name] <= 0.0 for name in positive_names)
                or values['tag_refinement_angular_k_beta'] >= 0.0
                or stable_cycles < 1):
            return self._tag_refinement_failure(
                'Tag refinement parameters are outside safe bounds')

        # When DockRobot starts inside its 15 cm completion radius, the action
        # can finish in the same executor cycle that /dock_pose is published.
        # Give the queued volatile sample a bounded chance to reach our callback
        # before treating the target as unavailable.
        target_wait_deadline = (
            time.monotonic()
            + values['tag_refinement_target_wait_timeout_sec'])
        while rclpy.ok() and not self.should_stop():
            target_age = (
                time.monotonic() - self.last_refinement_target_received_at
                if self.last_refinement_target_received_at > 0.0
                else math.inf)
            if (
                    self.last_refinement_target_pose is not None
                    and target_age <= values[
                        'tag_refinement_target_max_age_sec']):
                break
            if time.monotonic() >= target_wait_deadline:
                break
            rclpy.spin_once(self, timeout_sec=0.05)

        target = self.last_refinement_target_pose
        target_age = (
            time.monotonic() - self.last_refinement_target_received_at
            if self.last_refinement_target_received_at > 0.0
            else math.inf)
        fixed_frame = str(self.get_parameter('fixed_frame').value)
        if target is None or target_age > values[
                'tag_refinement_target_max_age_sec']:
            return self._tag_refinement_failure(
                'No fresh filtered Nav2 dock pose is available for refinement')
        if target.header.frame_id != fixed_frame:
            return self._tag_refinement_failure(
                'Refinement target frame mismatch: '
                f'expected "{fixed_frame}", got "{target.header.frame_id}"')

        target_x = float(target.pose.position.x)
        target_y = float(target.pose.position.y)
        target_yaw = self._quaternion_yaw(
            float(target.pose.orientation.x),
            float(target.pose.orientation.y),
            float(target.pose.orientation.z),
            float(target.pose.orientation.w),
        )
        if not all(math.isfinite(value) for value in (
                target_x, target_y, target_yaw)):
            return self._tag_refinement_failure(
                'Refinement target contains non-finite values')
        if not self.motion.odom_is_fresh():
            return self._tag_refinement_failure(
                'Odometry is stale before tag refinement')

        current_x, current_y = self.motion.current_xy()
        current_yaw = self.motion.current_yaw()
        initial_errors = self._fixed_goal_errors_in_target(
            target_x, target_y, target_yaw,
            current_x, current_y, current_yaw)
        if (
                abs(initial_errors[0]) > values[
                    'tag_refinement_max_initial_longitudinal']
                or abs(initial_errors[1]) > values[
                    'tag_refinement_max_initial_lateral']
                or abs(initial_errors[2]) > values[
                    'tag_refinement_max_initial_yaw']):
            return self._tag_refinement_failure(
                'Refusing an unexpectedly large tag refinement: '
                f'x={initial_errors[0]:+.3f}m, '
                f'y={initial_errors[1]:+.3f}m, '
                f'yaw={math.degrees(initial_errors[2]):+.2f}deg')

        self.motion.stop_robot()
        self.get_logger().info(
            'Starting bounded tag pose refinement against the frozen Nav2 '
            'dock pose: '
            f'x={initial_errors[0]:+.3f}m, '
            f'y={initial_errors[1]:+.3f}m, '
            f'yaw={math.degrees(initial_errors[2]):+.2f}deg')

        start_yaw = current_yaw
        previous_x = current_x
        previous_y = current_y
        traveled = 0.0
        stable_count = 0
        last_log_time = 0.0
        deadline = time.monotonic() + values['tag_refinement_timeout_sec']
        period = 1.0 / values['tag_refinement_control_rate_hz']

        while rclpy.ok() and not self.should_stop():
            rclpy.spin_once(self, timeout_sec=period)
            now = time.monotonic()
            if now >= deadline:
                return self._tag_refinement_failure(
                    'Tag pose refinement timed out', traveled)
            if not self.motion.odom_is_fresh():
                return self._tag_refinement_failure(
                    'Odometry became stale during tag refinement', traveled)

            current_x, current_y = self.motion.current_xy()
            current_yaw = self.motion.current_yaw()
            traveled += math.hypot(
                current_x - previous_x, current_y - previous_y)
            previous_x, previous_y = current_x, current_y
            yaw_excursion = abs(self.motion.normalize_angle(
                current_yaw - start_yaw))
            if traveled > values['tag_refinement_max_travel']:
                return self._tag_refinement_failure(
                    'Tag refinement exceeded its travel limit', traveled)
            if yaw_excursion > values['tag_refinement_max_yaw_excursion']:
                return self._tag_refinement_failure(
                    'Tag refinement exceeded its yaw excursion limit', traveled)

            # Position tolerances belong to the frozen dock pose axes.  If they
            # were evaluated in the rotating base frame, a final in-place yaw
            # correction would appear to create a new lateral position error
            # even though the robot base has not translated.
            longitudinal, lateral, yaw_error = (
                self._fixed_goal_errors_in_target(
                    target_x, target_y, target_yaw,
                    current_x, current_y, current_yaw))
            base_longitudinal, base_lateral, _ = (
                self._fixed_goal_errors_in_base(
                    target_x, target_y, target_yaw,
                    current_x, current_y, current_yaw))
            within_position = (
                abs(longitudinal) <= values[
                    'tag_refinement_longitudinal_tolerance']
                and abs(lateral) <= values[
                    'tag_refinement_lateral_tolerance'])
            within_yaw = abs(yaw_error) <= values[
                'tag_refinement_yaw_tolerance']
            stable_count = (
                stable_count + 1 if within_position and within_yaw else 0)

            if stable_count >= stable_cycles:
                self.motion.stop_robot()
                self.get_logger().info(
                    'Tag pose refinement complete: '
                    f'x={longitudinal:+.3f}m, y={lateral:+.3f}m, '
                    f'yaw={math.degrees(yaw_error):+.2f}deg, '
                    f'travel={traveled:.3f}m')
                return True

            linear_x, angular_z = self._tag_refinement_command(
                base_longitudinal,
                base_lateral,
                yaw_error,
                within_position,
                values['tag_refinement_linear_kp'],
                values['tag_refinement_angular_k_alpha'],
                values['tag_refinement_angular_k_beta'],
                values['tag_refinement_final_yaw_kp'],
                values['tag_refinement_max_linear_speed'],
                values['tag_refinement_max_angular_speed'],
            )
            if within_position and within_yaw:
                linear_x = 0.0
                angular_z = 0.0
            self.motion.cmd_vel_pub.publish(self.motion.twist(
                linear_x=linear_x, angular_z=angular_z))

            if now - last_log_time >= 1.0:
                self.get_logger().info(
                    'Tag refinement: '
                    f'x={longitudinal:+.3f}m, y={lateral:+.3f}m, '
                    f'yaw={math.degrees(yaw_error):+.2f}deg, '
                    f'cmd=({linear_x:+.3f}m/s, '
                    f'{angular_z:+.3f}rad/s), '
                    f'stable={stable_count}/{stable_cycles}')
                last_log_time = now

        self.motion.stop_robot()
        return False

    def _tag_refinement_failure(
            self, reason: str, traveled: float = 0.0) -> bool:
        self.motion.stop_robot()
        abort = bool(self.get_parameter(
            'tag_refinement_abort_on_failure').value)
        message = f'{reason}; refinement travel={traveled:.3f}m'
        if abort:
            self.get_logger().error(message)
            return False
        self.get_logger().warning(
            message + '; continuing with the original docking sequence')
        return True

    @staticmethod
    def _quaternion_yaw(
            x: float, y: float, z: float, w: float) -> float:
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )

    @staticmethod
    def _fixed_goal_errors_in_base(
            target_x: float, target_y: float, target_yaw: float,
            current_x: float, current_y: float,
            current_yaw: float) -> tuple[float, float, float]:
        dx = target_x - current_x
        dy = target_y - current_y
        cos_yaw = math.cos(current_yaw)
        sin_yaw = math.sin(current_yaw)
        longitudinal = cos_yaw * dx + sin_yaw * dy
        lateral = -sin_yaw * dx + cos_yaw * dy
        yaw_error = math.atan2(
            math.sin(target_yaw - current_yaw),
            math.cos(target_yaw - current_yaw),
        )
        return longitudinal, lateral, yaw_error

    @staticmethod
    def _fixed_goal_errors_in_target(
            target_x: float, target_y: float, target_yaw: float,
            current_x: float, current_y: float,
            current_yaw: float) -> tuple[float, float, float]:
        dx = target_x - current_x
        dy = target_y - current_y
        cos_yaw = math.cos(target_yaw)
        sin_yaw = math.sin(target_yaw)
        longitudinal = cos_yaw * dx + sin_yaw * dy
        lateral = -sin_yaw * dx + cos_yaw * dy
        yaw_error = math.atan2(
            math.sin(target_yaw - current_yaw),
            math.cos(target_yaw - current_yaw),
        )
        return longitudinal, lateral, yaw_error

    @staticmethod
    def _tag_refinement_command(
            longitudinal: float, lateral: float, yaw_error: float,
            within_position: bool,
            linear_kp: float, angular_k_alpha: float,
            angular_k_beta: float, final_yaw_kp: float,
            max_linear_speed: float,
            max_angular_speed: float) -> tuple[float, float]:
        if within_position:
            linear_x = 0.0
            angular_z = final_yaw_kp * yaw_error
        else:
            distance = math.hypot(longitudinal, lateral)
            bearing = math.atan2(lateral, longitudinal)
            direction = 1.0
            if math.cos(bearing) < 0.0:
                direction = -1.0
                bearing = math.atan2(
                    math.sin(bearing - math.pi),
                    math.cos(bearing - math.pi),
                )
            terminal_heading = math.atan2(
                math.sin(yaw_error - bearing),
                math.cos(yaw_error - bearing),
            )
            linear_x = direction * linear_kp * distance
            angular_z = (
                angular_k_alpha * bearing
                + angular_k_beta * terminal_heading)

        linear_x = max(
            -max_linear_speed, min(max_linear_speed, linear_x))
        angular_z = max(
            -max_angular_speed, min(max_angular_speed, angular_z))
        return linear_x, angular_z

    def _verify_tag_front_stop_pose(self) -> bool:
        if not bool(self.get_parameter('verify_tag_front_stop_pose').value):
            return True

        target_distance = float(
            self.get_parameter('tag_front_stop_distance').value)
        longitudinal_tolerance = abs(float(self.get_parameter(
            'tag_front_longitudinal_tolerance').value))
        lateral_tolerance = abs(float(self.get_parameter(
            'tag_front_lateral_tolerance').value))
        max_age = float(
            self.get_parameter('tag_front_pose_max_age_sec').value)
        stable_cycles = int(
            self.get_parameter('tag_front_stable_cycles').value)
        timeout = float(
            self.get_parameter('tag_front_verify_timeout_sec').value)

        values = (
            target_distance,
            longitudinal_tolerance,
            lateral_tolerance,
            max_age,
            timeout,
        )
        if (
                not all(math.isfinite(value) for value in values)
                or target_distance <= 0.0
                or longitudinal_tolerance <= 0.0
                or lateral_tolerance <= 0.0
                or max_age <= 0.0
                or stable_cycles < 1
                or timeout <= 0.0):
            self.get_logger().error(
                'Tag-front stop verification parameters are outside safe bounds')
            return False

        self.motion.stop_robot()
        self.get_logger().info(
            'Verifying centered tag-front stop pose before the 180-degree spin...')
        deadline = time.monotonic() + timeout
        last_sequence = self.dock_pose_sequence
        stable_count = 0
        last_log_time = 0.0

        while rclpy.ok() and not self.should_stop():
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            if now >= deadline:
                break
            if self.dock_pose_sequence == last_sequence:
                continue
            last_sequence = self.dock_pose_sequence

            pose = self.last_dock_pose
            pose_is_fresh = (
                pose is not None
                and self.last_dock_pose_received_at > 0.0
                and now - self.last_dock_pose_received_at <= max_age)
            if not pose_is_fresh:
                stable_count = 0
                continue

            assert pose is not None
            longitudinal_error, lateral_error = self._tag_front_pose_errors(
                pose.pose.position.x,
                pose.pose.position.y,
                target_distance,
            )
            centered = (
                abs(longitudinal_error) <= longitudinal_tolerance
                and abs(lateral_error) <= lateral_tolerance)
            stable_count = stable_count + 1 if centered else 0
            if stable_count >= stable_cycles:
                self.get_logger().info(
                    'Tag-front stop pose verified: '
                    f'distance_error={longitudinal_error:+.3f}m, '
                    f'lateral_error={lateral_error:+.3f}m, '
                    f'unique_poses={stable_count}')
                return True

            if now - last_log_time >= 1.0:
                self.get_logger().info(
                    'Tag-front pose check: '
                    f'distance_error={longitudinal_error:+.3f}m '
                    f'(limit={longitudinal_tolerance:.3f}m), '
                    f'lateral_error={lateral_error:+.3f}m '
                    f'(limit={lateral_tolerance:.3f}m), '
                    f'stable={stable_count}/{stable_cycles}')
                last_log_time = now

        self.motion.stop_robot()
        self.get_logger().error(
            'Refusing the 180-degree spin because the robot is not stably '
            'centered at the tag-front stop pose')
        return False

    @staticmethod
    def _tag_front_pose_errors(
            tag_x: float, tag_y: float,
            target_distance: float) -> tuple[float, float]:
        if not all(math.isfinite(value) for value in (
                tag_x, tag_y, target_distance)):
            return math.inf, math.inf
        return tag_x - target_distance, tag_y

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
        rclpy.init(args=cli_args)
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
