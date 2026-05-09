import math
import random

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from docking.motion import MotionController


class LidarPlaneAligner:
    @staticmethod
    def declare_parameters(node: Node) -> None:
        node.declare_parameter('use_lidar_alignment', True)
        node.declare_parameter('scan_topic', '/scan')
        node.declare_parameter('lidar_align_timeout_sec', 8.0)
        node.declare_parameter('lidar_align_sector_center', 0.0)
        node.declare_parameter('lidar_align_sector_width', math.radians(60.0))
        node.declare_parameter('lidar_align_target_normal_angle', math.pi)
        node.declare_parameter('lidar_align_min_range', 0.15)
        node.declare_parameter('lidar_align_max_range', 2.0)
        node.declare_parameter('lidar_align_min_points', 20)
        node.declare_parameter('lidar_align_min_inliers', 12)
        node.declare_parameter('lidar_align_ransac_iterations', 45)
        node.declare_parameter('lidar_align_ransac_threshold', 0.035)
        node.declare_parameter('lidar_align_min_line_length', 0.25)
        node.declare_parameter('lidar_align_tolerance', math.radians(2.0))
        node.declare_parameter('lidar_align_kp', 0.8)
        node.declare_parameter('lidar_align_angular_speed', 0.08)
        node.declare_parameter('lidar_align_min_angular_speed', 0.02)
        node.declare_parameter('lidar_align_stable_cycles', 4)

    def __init__(self, node: Node, motion: MotionController) -> None:
        self.node = node
        self.motion = motion
        self.scan_sub = node.create_subscription(
            LaserScan,
            node.get_parameter('scan_topic').value,
            self._scan_callback,
            qos_profile_sensor_data)
        self.last_scan: LaserScan | None = None

    def align(self) -> bool:
        timeout_sec = float(self.node.get_parameter('lidar_align_timeout_sec').value)
        tolerance = abs(float(self.node.get_parameter('lidar_align_tolerance').value))
        max_speed = abs(float(self.node.get_parameter('lidar_align_angular_speed').value))
        min_speed = abs(float(
            self.node.get_parameter('lidar_align_min_angular_speed').value))
        kp = float(self.node.get_parameter('lidar_align_kp').value)
        stable_cycles_required = int(
            self.node.get_parameter('lidar_align_stable_cycles').value)
        rate_hz = float(self.node.get_parameter('control_rate_hz').value)
        sleep_time = 1.0 / max(rate_hz, 1.0)
        start = self.node.get_clock().now()
        stable_cycles = 0
        last_log_time = 0.0

        self.node.get_logger().info('Fine-aligning yaw using LiDAR plane...')

        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.0)
            elapsed = (self.node.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.motion.stop_robot()
                self.node.get_logger().error('LiDAR plane alignment timed out')
                return False

            if self.last_scan is None:
                self.motion.cmd_vel_pub.publish(Twist())
                rclpy.spin_once(self.node, timeout_sec=sleep_time)
                continue

            estimate = self._estimate_error(self.last_scan)
            if estimate is None:
                stable_cycles = 0
                now = self.node.get_clock().now().nanoseconds / 1e9
                if now - last_log_time >= 1.0:
                    self.node.get_logger().info('Waiting for a valid LiDAR plane...')
                    last_log_time = now
                self.motion.cmd_vel_pub.publish(Twist())
                rclpy.spin_once(self.node, timeout_sec=sleep_time)
                continue

            error, normal_angle, inliers, total_points, line_length = estimate
            if abs(error) <= tolerance:
                stable_cycles += 1
                self.motion.stop_robot()
                if stable_cycles >= stable_cycles_required:
                    self.node.get_logger().info(
                        'LiDAR plane alignment complete: '
                        f'error={math.degrees(error):.2f}deg, '
                        f'normal={math.degrees(normal_angle):.2f}deg, '
                        f'inliers={inliers}/{total_points}, '
                        f'line_length={line_length:.2f}m')
                    return True
            else:
                stable_cycles = 0
                angular_z = self._clamp(kp * error, -max_speed, max_speed)
                if abs(angular_z) < min_speed:
                    angular_z = math.copysign(min_speed, angular_z)
                self.motion.cmd_vel_pub.publish(
                    MotionController.twist(angular_z=angular_z))

            now = self.node.get_clock().now().nanoseconds / 1e9
            if now - last_log_time >= 1.0:
                self.node.get_logger().info(
                    'LiDAR align: '
                    f'error={math.degrees(error):.2f}deg, '
                    f'cmd={kp * error:.3f}rad/s, '
                    f'inliers={inliers}/{total_points}, '
                    f'line_length={line_length:.2f}m')
                last_log_time = now

            rclpy.spin_once(self.node, timeout_sec=sleep_time)

        return False

    def _estimate_error(
            self, scan: LaserScan) -> tuple[float, float, int, int, float] | None:
        points = self._scan_points_in_sector(scan)
        min_points = int(self.node.get_parameter('lidar_align_min_points').value)
        if len(points) < min_points:
            return None

        inliers = self._ransac_line_inliers(points)
        min_inliers = int(self.node.get_parameter('lidar_align_min_inliers').value)
        if len(inliers) < min_inliers:
            return None

        line = self._fit_line_pca(inliers)
        if line is None:
            return None

        line_angle, line_length = line
        min_line_length = float(
            self.node.get_parameter('lidar_align_min_line_length').value)
        if line_length < min_line_length:
            return None

        target_angle = float(
            self.node.get_parameter('lidar_align_target_normal_angle').value)
        normal_a = MotionController.normalize_angle(line_angle + math.pi / 2.0)
        normal_b = MotionController.normalize_angle(line_angle - math.pi / 2.0)
        if abs(MotionController.normalize_angle(normal_a - target_angle)) <= abs(
                MotionController.normalize_angle(normal_b - target_angle)):
            normal_angle = normal_a
        else:
            normal_angle = normal_b

        error = MotionController.normalize_angle(normal_angle - target_angle)
        return error, normal_angle, len(inliers), len(points), line_length

    def _scan_points_in_sector(self, scan: LaserScan) -> list[tuple[float, float]]:
        center = float(self.node.get_parameter('lidar_align_sector_center').value)
        half_width = abs(float(
            self.node.get_parameter('lidar_align_sector_width').value)) / 2.0
        min_range = float(self.node.get_parameter('lidar_align_min_range').value)
        max_range = float(self.node.get_parameter('lidar_align_max_range').value)
        points: list[tuple[float, float]] = []

        angle = scan.angle_min
        for distance in scan.ranges:
            if (
                    math.isfinite(distance)
                    and min_range <= distance <= max_range
                    and abs(MotionController.normalize_angle(angle - center)) <= half_width):
                points.append((distance * math.cos(angle), distance * math.sin(angle)))
            angle += scan.angle_increment

        return points

    def _ransac_line_inliers(
            self, points: list[tuple[float, float]]) -> list[tuple[float, float]]:
        iterations = int(self.node.get_parameter('lidar_align_ransac_iterations').value)
        threshold = float(self.node.get_parameter('lidar_align_ransac_threshold').value)
        best_inliers: list[tuple[float, float]] = []

        if len(points) < 2:
            return best_inliers

        for _ in range(max(iterations, 1)):
            p1, p2 = random.sample(points, 2)
            x1, y1 = p1
            x2, y2 = p2
            dx = x2 - x1
            dy = y2 - y1
            norm = math.hypot(dx, dy)
            if norm < 1e-6:
                continue

            a = dy / norm
            b = -dx / norm
            c = -(a * x1 + b * y1)
            inliers = [
                point for point in points
                if abs(a * point[0] + b * point[1] + c) <= threshold
            ]
            if len(inliers) > len(best_inliers):
                best_inliers = inliers

        return best_inliers

    @staticmethod
    def _fit_line_pca(
            points: list[tuple[float, float]]) -> tuple[float, float] | None:
        if len(points) < 2:
            return None

        mean_x = sum(point[0] for point in points) / len(points)
        mean_y = sum(point[1] for point in points) / len(points)
        centered = [(point[0] - mean_x, point[1] - mean_y) for point in points]
        sxx = sum(point[0] * point[0] for point in centered)
        syy = sum(point[1] * point[1] for point in centered)
        sxy = sum(point[0] * point[1] for point in centered)
        if sxx + syy < 1e-9:
            return None

        line_angle = 0.5 * math.atan2(2.0 * sxy, sxx - syy)
        direction_x = math.cos(line_angle)
        direction_y = math.sin(line_angle)
        projections = [
            point[0] * direction_x + point[1] * direction_y
            for point in centered
        ]
        line_length = max(projections) - min(projections)
        return line_angle, line_length

    def _scan_callback(self, msg: LaserScan) -> None:
        self.last_scan = msg

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return min(max(value, lower), upper)
