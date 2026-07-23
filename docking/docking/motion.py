import math
import time
from typing import Any, Callable

from docking.docking_lidar import DockingLidar, ScanSnapshot
from docking.lidar_geometry import (
    effective_range_limits,
    has_consecutive_clearance_cluster,
    normalize_angle,
    rear_clearances,
    required_rear_range_at_completion,
    UniqueScanStability,
)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class MotionController:
    @staticmethod
    def declare_parameters(node: Node) -> None:
        node.declare_parameter('cmd_vel_topic', '/cmd_vel')
        node.declare_parameter('odom_topic', '/odom')
        node.declare_parameter('spin_yaw', math.pi)
        node.declare_parameter('spin_angular_speed', 0.15)
        node.declare_parameter('spin_tolerance', 0.04)
        node.declare_parameter('backup_distance', 0.50)
        node.declare_parameter('backup_speed', 0.05)
        node.declare_parameter('backup_tolerance', 0.02)
        node.declare_parameter('use_lidar_backup', True)
        node.declare_parameter('backup_lidar_sector_center', math.nan)
        node.declare_parameter('backup_lidar_sector_center_base', math.pi)
        node.declare_parameter('backup_lidar_sector_width', math.radians(20.0))
        node.declare_parameter(
            'backup_lidar_safety_sector_width', math.radians(150.0))
        node.declare_parameter('backup_rear_half_width', 0.22)
        node.declare_parameter('backup_rear_safety_margin', 0.02)
        node.declare_parameter('backup_target_rear_clearance', 0.01)
        node.declare_parameter('backup_clearance_tolerance', 0.005)
        node.declare_parameter('backup_rear_reference_x', -0.2295)
        node.declare_parameter('backup_lidar_min_range', 0.05)
        node.declare_parameter('backup_lidar_max_range', 2.0)
        node.declare_parameter('backup_lidar_success_min_points', 5)
        node.declare_parameter(
            'backup_lidar_success_min_angle_span', math.radians(3.0))
        node.declare_parameter('backup_lidar_stable_cycles', 3)
        node.declare_parameter('backup_max_travel', 0.60)
        node.declare_parameter('backup_max_yaw_drift', math.radians(5.0))
        node.declare_parameter('backup_slowdown_clearance', 0.15)
        node.declare_parameter('backup_min_speed', 0.015)
        node.declare_parameter('control_rate_hz', 20.0)
        node.declare_parameter('motion_timeout_sec', 45.0)
        node.declare_parameter('server_wait_timeout_sec', 10.0)
        node.declare_parameter('odom_max_age_sec', 0.50)

    def __init__(
            self, node: Node, lidar: DockingLidar,
            should_stop: Callable[[], bool] | None = None) -> None:
        self.node = node
        self.lidar = lidar
        self.should_stop = should_stop or (lambda: False)
        self.cmd_vel_pub = node.create_publisher(
            Twist, node.get_parameter('cmd_vel_topic').value, 10)
        self.odom_sub = node.create_subscription(
            Odometry, node.get_parameter('odom_topic').value, self._odom_callback, 10)
        self.last_odom: Odometry | None = None
        self.last_odom_received_at = 0.0

    def wait_for_odom(self) -> bool:
        timeout = float(self.node.get_parameter('server_wait_timeout_sec').value)
        start = self.node.get_clock().now()
        self.node.get_logger().info('Waiting for odom...')

        while (
                rclpy.ok()
                and not self.should_stop()
                and not self.odom_is_fresh()):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if (self.node.get_clock().now() - start).nanoseconds / 1e9 > timeout:
                self.node.get_logger().error('odom is not available')
                return False

        return self.odom_is_fresh()

    def spin_180(self) -> bool:
        target_yaw = float(self.node.get_parameter('spin_yaw').value)
        speed = abs(float(self.node.get_parameter('spin_angular_speed').value))
        tolerance = abs(float(self.node.get_parameter('spin_tolerance').value))
        direction = 1.0 if target_yaw >= 0.0 else -1.0

        self.node.get_logger().info(
            f'Spinning {target_yaw:.3f} rad at {speed:.3f} rad/s using cmd_vel')

        ok = self._run_until(
            done_cb=self._make_spin_done_cb(abs(target_yaw), tolerance),
            cmd_cb=lambda: self.twist(angular_z=direction * speed),
            timeout_sec=float(self.node.get_parameter('motion_timeout_sec').value),
        )
        self.stop_robot()

        if ok:
            self.node.get_logger().info('Spin step complete')
        else:
            self.node.get_logger().error('Spin step failed or timed out')
        return ok

    def backup(self) -> bool:
        if bool(self.node.get_parameter('use_lidar_backup').value):
            return self.backup_with_lidar()

        return self.backup_with_odom()

    def backup_with_odom(self) -> bool:
        distance = abs(float(self.node.get_parameter('backup_distance').value))
        speed = abs(float(self.node.get_parameter('backup_speed').value))
        tolerance = abs(float(self.node.get_parameter('backup_tolerance').value))

        self.node.get_logger().info(
            f'Backing up {distance:.3f} m at {speed:.3f} m/s using cmd_vel')

        ok = self._run_until(
            done_cb=self._make_backup_done_cb(distance, tolerance),
            cmd_cb=lambda: self.twist(linear_x=-speed),
            timeout_sec=float(self.node.get_parameter('motion_timeout_sec').value),
        )
        self.stop_robot()

        if ok:
            self.node.get_logger().info('Dock-turn-backup sequence complete')
        else:
            self.node.get_logger().error('Backup step failed or timed out')
        return ok

    def backup_with_lidar(self) -> bool:
        if not self.validate_lidar_configuration():
            return False

        speed = abs(float(self.node.get_parameter('backup_speed').value))
        min_speed = min(
            speed,
            abs(float(self.node.get_parameter('backup_min_speed').value)))
        slowdown_clearance = abs(float(
            self.node.get_parameter('backup_slowdown_clearance').value))
        target_clearance = abs(float(
            self.node.get_parameter('backup_target_rear_clearance').value))
        tolerance = abs(float(
            self.node.get_parameter('backup_clearance_tolerance').value))
        stable_cycles_required = int(
            self.node.get_parameter('backup_lidar_stable_cycles').value)
        success_min_points = int(
            self.node.get_parameter('backup_lidar_success_min_points').value)
        success_min_angle_span = abs(float(self.node.get_parameter(
            'backup_lidar_success_min_angle_span').value))
        max_travel = abs(float(
            self.node.get_parameter('backup_max_travel').value))
        max_yaw_drift = abs(float(
            self.node.get_parameter('backup_max_yaw_drift').value))
        timeout_sec = float(self.node.get_parameter('motion_timeout_sec').value)
        rate_hz = float(self.node.get_parameter('control_rate_hz').value)
        sleep_time = 1.0 / max(rate_hz, 1.0)
        start = self.node.get_clock().now()
        start_x, start_y = self.current_xy()
        start_yaw = self.current_yaw()
        stage_start_sequence = self.lidar.sequence
        stability = UniqueScanStability(
            stable_cycles_required, stage_start_sequence)
        current_command = Twist()
        last_log_time = 0.0

        self.node.get_logger().info(
            'Backing up using LiDAR rear clearance: '
            f'target={target_clearance:.3f}m, speed={speed:.3f}m/s')

        while rclpy.ok() and not self.should_stop():
            rclpy.spin_once(self.node, timeout_sec=0.0)

            elapsed = (self.node.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.stop_robot()
                self.node.get_logger().error('LiDAR backup step timed out')
                return False

            if not self.odom_is_fresh():
                self.stop_robot()
                self.node.get_logger().error(
                    'Odometry became stale during LiDAR backup')
                return False

            current_x, current_y = self.current_xy()
            traveled = math.hypot(current_x - start_x, current_y - start_y)
            if traveled >= max_travel:
                self.stop_robot()
                self.node.get_logger().error(
                    'LiDAR backup exceeded the travel safety limit: '
                    f'{traveled:.3f}m >= {max_travel:.3f}m')
                return False

            yaw_drift = abs(self.normalize_angle(self.current_yaw() - start_yaw))
            if yaw_drift >= max_yaw_drift:
                self.stop_robot()
                self.node.get_logger().error(
                    'LiDAR backup exceeded the yaw-drift safety limit: '
                    f'{math.degrees(yaw_drift):.2f}deg >= '
                    f'{math.degrees(max_yaw_drift):.2f}deg')
                return False

            snapshot = self.lidar.snapshot()
            if snapshot is None:
                stability.reset()
                current_command = Twist()
                self.cmd_vel_pub.publish(current_command)
                if (
                        time.monotonic() - self.lidar.last_valid_received_at
                        > self.lidar.max_scan_age):
                    self.stop_robot()
                    self.node.get_logger().error(
                        f'Docking LiDAR failed during backup: {self.lidar.last_error}')
                    return False
                now = self.node.get_clock().now().nanoseconds / 1e9
                if now - last_log_time >= 1.0:
                    self.node.get_logger().info(
                        'Waiting for a fresh docking LiDAR scan: '
                        f'{self.lidar.last_error}')
                    last_log_time = now
                rclpy.spin_once(self.node, timeout_sec=sleep_time)
                continue

            if snapshot.sequence == stability.last_sequence:
                self.cmd_vel_pub.publish(current_command)
                rclpy.spin_once(self.node, timeout_sec=sleep_time)
                continue

            indexed_clearances = self._rear_clearances(snapshot)
            if not indexed_clearances:
                stability.observe(snapshot.sequence, False)
                current_command = Twist()
                self.cmd_vel_pub.publish(current_command)
                rclpy.spin_once(self.node, timeout_sec=sleep_time)
                continue

            clearance = min(value for _, value in indexed_clearances)
            completion_threshold = target_clearance + tolerance
            completion_minimum = max(target_clearance - tolerance, 0.0)
            minimum_beam_span = math.ceil(
                success_min_angle_span / abs(snapshot.scan.angle_increment))
            completion_cluster = has_consecutive_clearance_cluster(
                indexed_clearances,
                completion_threshold,
                success_min_points,
                minimum=completion_minimum,
                minimum_beam_span=minimum_beam_span,
            )
            completion_cluster = completion_cluster and clearance >= completion_minimum
            overrun_cluster = has_consecutive_clearance_cluster(
                indexed_clearances,
                math.nextafter(completion_minimum, -math.inf),
                success_min_points,
                minimum_beam_span=minimum_beam_span,
            )
            if overrun_cluster:
                self.stop_robot()
                self.node.get_logger().error(
                    'Rear LiDAR indicates the docking target was overrun: '
                    f'clearance={clearance:.3f}m < '
                    f'{completion_minimum:.3f}m')
                return False

            safety_clearances = self._rear_safety_clearances(snapshot)
            protective_clearance = clearance
            if safety_clearances:
                protective_clearance = min(
                    protective_clearance,
                    min(value for _, value in safety_clearances))
            complete = stability.observe(
                snapshot.sequence, completion_cluster)

            if protective_clearance <= completion_threshold:
                current_command = Twist()
            else:
                command_speed = self._backup_speed_for_clearance(
                    protective_clearance,
                    target_clearance,
                    slowdown_clearance,
                    min_speed,
                    speed,
                )
                current_command = self.twist(linear_x=-command_speed)

            self.cmd_vel_pub.publish(current_command)
            if complete:
                self.stop_robot()
                self.node.get_logger().info(
                    'Dock-turn-backup sequence complete: '
                    f'rear_clearance={clearance:.3f}m, '
                    f'unique_scans={stability.count}')
                return True

            now = self.node.get_clock().now().nanoseconds / 1e9
            if now - last_log_time >= 1.0:
                self.node.get_logger().info(
                    f'LiDAR backup: rear_clearance={clearance:.3f}m, '
                    f'target={target_clearance:.3f}m, '
                    f'protective={protective_clearance:.3f}m, '
                    f'traveled={traveled:.3f}m')
                last_log_time = now

            rclpy.spin_once(self.node, timeout_sec=sleep_time)

        self.stop_robot()
        return False

    def stop_robot(self) -> None:
        if not rclpy.ok():
            return
        stop = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(stop)
            rclpy.spin_once(self.node, timeout_sec=0.02)

    def _run_until(self, done_cb: Any, cmd_cb: Any, timeout_sec: float) -> bool:
        rate_hz = float(self.node.get_parameter('control_rate_hz').value)
        sleep_time = 1.0 / max(rate_hz, 1.0)
        start = self.node.get_clock().now()

        while rclpy.ok() and not self.should_stop():
            rclpy.spin_once(self.node, timeout_sec=0.0)
            if not self.odom_is_fresh():
                self.node.get_logger().error('Odometry became stale during motion')
                return False
            if done_cb():
                return True

            elapsed = (self.node.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout_sec:
                return False

            self.cmd_vel_pub.publish(cmd_cb())
            rclpy.spin_once(self.node, timeout_sec=sleep_time)

        return False

    def _make_spin_done_cb(self, target_yaw: float, tolerance: float) -> Any:
        last_yaw = self.current_yaw()
        accumulated = 0.0

        def done() -> bool:
            nonlocal last_yaw, accumulated
            current_yaw = self.current_yaw()
            delta = self.normalize_angle(current_yaw - last_yaw)
            accumulated += abs(delta)
            last_yaw = current_yaw
            return accumulated >= max(target_yaw - tolerance, 0.0)

        return done

    def _make_backup_done_cb(self, distance: float, tolerance: float) -> Any:
        start_x, start_y = self.current_xy()

        def done() -> bool:
            current_x, current_y = self.current_xy()
            traveled = math.hypot(current_x - start_x, current_y - start_y)
            return traveled >= max(distance - tolerance, 0.0)

        return done

    def _odom_callback(self, msg: Odometry) -> None:
        self.last_odom = msg
        self.last_odom_received_at = time.monotonic()

    def rear_clearance(self) -> float | None:
        snapshot = self.lidar.snapshot()
        if snapshot is None:
            return None
        indexed_clearances = self._rear_clearances(snapshot)
        if not indexed_clearances:
            return None
        return min(value for _, value in indexed_clearances)

    def _rear_clearances(
            self, snapshot: ScanSnapshot) -> list[tuple[int, float]] | None:
        center = float(self.node.get_parameter(
            'backup_lidar_sector_center_base').value)
        width = abs(float(
            self.node.get_parameter('backup_lidar_sector_width').value))
        min_range = float(self.node.get_parameter('backup_lidar_min_range').value)
        max_range = float(self.node.get_parameter('backup_lidar_max_range').value)
        points = self.lidar.project(
            snapshot, center, width, min_range, max_range)
        if points is None:
            return None
        rear_reference_x = float(
            self.node.get_parameter('backup_rear_reference_x').value)
        return rear_clearances(points, rear_reference_x)

    def _rear_safety_clearances(
            self, snapshot: ScanSnapshot) -> list[tuple[int, float]] | None:
        center = float(self.node.get_parameter(
            'backup_lidar_sector_center_base').value)
        width = abs(float(self.node.get_parameter(
            'backup_lidar_safety_sector_width').value))
        min_range = float(self.node.get_parameter('backup_lidar_min_range').value)
        max_range = float(self.node.get_parameter('backup_lidar_max_range').value)
        points = self.lidar.project(
            snapshot, center, width, min_range, max_range)
        if points is None:
            return None

        half_width = abs(float(
            self.node.get_parameter('backup_rear_half_width').value))
        margin = abs(float(
            self.node.get_parameter('backup_rear_safety_margin').value))
        corridor_points = [
            point for point in points
            if abs(point.y) <= half_width + margin
        ]
        rear_reference_x = float(
            self.node.get_parameter('backup_rear_reference_x').value)
        return rear_clearances(corridor_points, rear_reference_x)

    def validate_lidar_configuration(self) -> bool:
        snapshot = self.lidar.snapshot()
        transform = self.lidar.transform
        if snapshot is None or transform is None:
            self.node.get_logger().error(
                f'Docking LiDAR is not ready: {self.lidar.last_error}')
            return False

        legacy_center = float(
            self.node.get_parameter('backup_lidar_sector_center').value)
        if math.isfinite(legacy_center):
            self.node.get_logger().error(
                'backup_lidar_sector_center used the old scan-frame convention. '
                'Use backup_lidar_sector_center_base with base_link angles.')
            return False

        target = float(
            self.node.get_parameter('backup_target_rear_clearance').value)
        tolerance = float(
            self.node.get_parameter('backup_clearance_tolerance').value)
        rear_x = float(self.node.get_parameter('backup_rear_reference_x').value)
        configured_min = float(
            self.node.get_parameter('backup_lidar_min_range').value)
        configured_max = float(
            self.node.get_parameter('backup_lidar_max_range').value)
        speed = float(self.node.get_parameter('backup_speed').value)
        min_speed = float(self.node.get_parameter('backup_min_speed').value)
        slowdown = float(
            self.node.get_parameter('backup_slowdown_clearance').value)
        max_travel = float(self.node.get_parameter('backup_max_travel').value)
        max_yaw_drift = float(
            self.node.get_parameter('backup_max_yaw_drift').value)
        center = float(self.node.get_parameter(
            'backup_lidar_sector_center_base').value)
        sector_width = float(
            self.node.get_parameter('backup_lidar_sector_width').value)
        safety_width = float(self.node.get_parameter(
            'backup_lidar_safety_sector_width').value)
        rear_half_width = float(
            self.node.get_parameter('backup_rear_half_width').value)
        safety_margin = float(
            self.node.get_parameter('backup_rear_safety_margin').value)
        stable_cycles = int(
            self.node.get_parameter('backup_lidar_stable_cycles').value)
        success_points = int(
            self.node.get_parameter('backup_lidar_success_min_points').value)
        success_span = float(self.node.get_parameter(
            'backup_lidar_success_min_angle_span').value)
        control_rate = float(
            self.node.get_parameter('control_rate_hz').value)
        motion_timeout = float(
            self.node.get_parameter('motion_timeout_sec').value)
        odom_max_age = float(
            self.node.get_parameter('odom_max_age_sec').value)

        finite_values = (
            target, tolerance, rear_x, configured_min, configured_max,
            speed, min_speed, slowdown, max_travel, max_yaw_drift,
            center, sector_width, safety_width, rear_half_width,
            safety_margin, success_span, control_rate, motion_timeout,
            odom_max_age,
            transform.x, transform.y, transform.yaw,
        )
        if not all(math.isfinite(value) for value in finite_values):
            self.node.get_logger().error(
                'Docking LiDAR parameters and TF must all be finite')
            return False
        if (
                target < 0.0
                or tolerance < 0.0
                or configured_min < 0.0
                or configured_max <= configured_min
                or speed <= 0.0
                or not 0.0 < min_speed <= speed
                or slowdown <= target + tolerance
                or max_travel <= 0.0
                or max_yaw_drift <= 0.0
                or not 0.0 < sector_width <= 2.0 * math.pi
                or not 0.0 < safety_width <= 2.0 * math.pi
                or safety_width < sector_width
                or rear_half_width <= 0.0
                or safety_margin < 0.0
                or stable_cycles < 1
                or success_points < 1
                or not 0.0 < success_span <= sector_width
                or control_rate <= 0.0
                or motion_timeout <= 0.0
                or odom_max_age <= 0.0):
            self.node.get_logger().error(
                'Docking LiDAR backup parameters are outside safe bounds')
            return False
        if abs(normalize_angle(math.pi - center)) > sector_width / 2.0:
            self.node.get_logger().error(
                'Docking LiDAR completion sector does not include robot rear (pi)')
            return False

        available_beams = math.floor(
            sector_width / abs(snapshot.scan.angle_increment)) + 1
        required_span_beams = math.ceil(
            success_span / abs(snapshot.scan.angle_increment))
        if success_points > available_beams or required_span_beams >= available_beams:
            self.node.get_logger().error(
                'Docking LiDAR scan resolution cannot satisfy the completion cluster')
            return False
        limits = effective_range_limits(
            snapshot.scan.range_min,
            snapshot.scan.range_max,
            configured_min,
            configured_max,
        )
        if limits is None:
            self.node.get_logger().error('Docking LiDAR range limits do not overlap')
            return False

        required_range = required_rear_range_at_completion(
            transform.x, rear_x, target, tolerance)
        if transform.x <= rear_x or required_range <= 0.0:
            self.node.get_logger().error(
                'Docking LiDAR must be forward of the rear reference: '
                f'lidar_x={transform.x:.4f}m, rear_x={rear_x:.4f}m')
            return False

        effective_min, effective_max = limits
        if not effective_min <= required_range <= effective_max:
            self.node.get_logger().error(
                'Docking completion distance is outside the usable LiDAR range: '
                f'required<={required_range:.4f}m, '
                f'usable=[{effective_min:.4f}, {effective_max:.4f}]m')
            return False

        self.node.get_logger().info(
            'Docking LiDAR geometry validated: '
            f'lidar_x={transform.x:.4f}m, rear_x={rear_x:.4f}m, '
            f'completion_range<={required_range:.4f}m, '
            f'usable_min={effective_min:.4f}m')
        return True

    def odom_is_fresh(self) -> bool:
        if self.last_odom is None:
            return False
        max_age = max(float(
            self.node.get_parameter('odom_max_age_sec').value), 0.0)
        return (
            self.last_odom_received_at > 0.0
            and time.monotonic() - self.last_odom_received_at <= max_age
        )

    @staticmethod
    def _backup_speed_for_clearance(
            clearance: float, target: float, slowdown: float,
            min_speed: float, max_speed: float) -> float:
        if slowdown <= target or clearance >= slowdown:
            return max_speed
        ratio = max(min((clearance - target) / (slowdown - target), 1.0), 0.0)
        return min_speed + ratio * (max_speed - min_speed)

    def current_yaw(self) -> float:
        assert self.last_odom is not None
        orientation = self.last_odom.pose.pose.orientation
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def current_xy(self) -> tuple[float, float]:
        assert self.last_odom is not None
        position = self.last_odom.pose.pose.position
        return position.x, position.y

    @staticmethod
    def twist(linear_x: float = 0.0, angular_z: float = 0.0) -> Twist:
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        return twist

    @staticmethod
    def normalize_angle(angle: float) -> float:
        return normalize_angle(angle)
