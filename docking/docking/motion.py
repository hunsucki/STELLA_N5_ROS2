import math
from typing import Any, Callable

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


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
        node.declare_parameter('backup_scan_topic', '/scan_2')
        node.declare_parameter('backup_lidar_sector_center', 0.0)
        node.declare_parameter('backup_lidar_sector_width', math.radians(20.0))
        node.declare_parameter('backup_target_rear_clearance', 0.01)
        node.declare_parameter('backup_clearance_tolerance', 0.005)
        node.declare_parameter('backup_lidar_to_rear_bumper_offset', 0.0635)
        node.declare_parameter('backup_lidar_min_range', 0.15)
        node.declare_parameter('backup_lidar_max_range', 2.0)
        node.declare_parameter('backup_lidar_stable_cycles', 3)
        node.declare_parameter('control_rate_hz', 20.0)
        node.declare_parameter('motion_timeout_sec', 45.0)
        node.declare_parameter('server_wait_timeout_sec', 10.0)

    def __init__(
            self, node: Node,
            should_stop: Callable[[], bool] | None = None) -> None:
        self.node = node
        self.should_stop = should_stop or (lambda: False)
        self.cmd_vel_pub = node.create_publisher(
            Twist, node.get_parameter('cmd_vel_topic').value, 10)
        self.odom_sub = node.create_subscription(
            Odometry, node.get_parameter('odom_topic').value, self._odom_callback, 10)
        self.scan_sub = node.create_subscription(
            LaserScan,
            node.get_parameter('backup_scan_topic').value,
            self._scan_callback,
            qos_profile_sensor_data)
        self.last_odom: Odometry | None = None
        self.last_scan: LaserScan | None = None

    def wait_for_odom(self) -> bool:
        timeout = float(self.node.get_parameter('server_wait_timeout_sec').value)
        start = self.node.get_clock().now()
        self.node.get_logger().info('Waiting for odom...')

        while (
                rclpy.ok()
                and not self.should_stop()
                and self.last_odom is None):
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if (self.node.get_clock().now() - start).nanoseconds / 1e9 > timeout:
                self.node.get_logger().error('odom is not available')
                return False

        return self.last_odom is not None

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
        speed = abs(float(self.node.get_parameter('backup_speed').value))
        target_clearance = abs(float(
            self.node.get_parameter('backup_target_rear_clearance').value))
        tolerance = abs(float(
            self.node.get_parameter('backup_clearance_tolerance').value))
        stable_cycles_required = int(
            self.node.get_parameter('backup_lidar_stable_cycles').value)
        timeout_sec = float(self.node.get_parameter('motion_timeout_sec').value)
        rate_hz = float(self.node.get_parameter('control_rate_hz').value)
        sleep_time = 1.0 / max(rate_hz, 1.0)
        start = self.node.get_clock().now()
        stable_cycles = 0
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

            clearance = self.rear_clearance()
            if clearance is None:
                stable_cycles = 0
                now = self.node.get_clock().now().nanoseconds / 1e9
                if now - last_log_time >= 1.0:
                    self.node.get_logger().info(
                        'Waiting for a valid rear LiDAR distance...')
                    last_log_time = now
                self.cmd_vel_pub.publish(Twist())
                rclpy.spin_once(self.node, timeout_sec=sleep_time)
                continue

            if clearance <= target_clearance + tolerance:
                stable_cycles += 1
                self.stop_robot()
                if stable_cycles >= stable_cycles_required:
                    self.node.get_logger().info(
                        'Dock-turn-backup sequence complete: '
                        f'rear_clearance={clearance:.3f}m')
                    return True
            else:
                stable_cycles = 0
                self.cmd_vel_pub.publish(self.twist(linear_x=-speed))

            now = self.node.get_clock().now().nanoseconds / 1e9
            if now - last_log_time >= 1.0:
                self.node.get_logger().info(
                    f'LiDAR backup: rear_clearance={clearance:.3f}m, '
                    f'target={target_clearance:.3f}m')
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

    def _scan_callback(self, msg: LaserScan) -> None:
        self.last_scan = msg

    def rear_clearance(self) -> float | None:
        scan = self.last_scan
        if scan is None:
            return None

        distance = self._sector_min_distance(scan)
        if distance is None:
            return None

        offset = float(
            self.node.get_parameter('backup_lidar_to_rear_bumper_offset').value)
        return distance - offset

    def _sector_min_distance(self, scan: LaserScan) -> float | None:
        center = float(self.node.get_parameter('backup_lidar_sector_center').value)
        half_width = abs(float(
            self.node.get_parameter('backup_lidar_sector_width').value)) / 2.0
        min_range = float(self.node.get_parameter('backup_lidar_min_range').value)
        max_range = float(self.node.get_parameter('backup_lidar_max_range').value)
        distances: list[float] = []

        angle = scan.angle_min
        for distance in scan.ranges:
            if (
                    math.isfinite(distance)
                    and min_range <= distance <= max_range
                    and abs(self.normalize_angle(angle - center)) <= half_width):
                distances.append(distance)
            angle += scan.angle_increment

        if not distances:
            return None
        return min(distances)

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
        return math.atan2(math.sin(angle), math.cos(angle))
