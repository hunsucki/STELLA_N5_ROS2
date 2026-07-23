import math
import random
import time

from docking.docking_lidar import DockingLidar, ScanSnapshot
from docking.lidar_geometry import (
    line_orientation_error,
    normalize_angle,
    UniqueScanStability,
)
from docking.motion import MotionController
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class LidarPlaneAligner:
    @staticmethod
    def declare_parameters(node: Node) -> None:
        node.declare_parameter('use_lidar_alignment', True)
        node.declare_parameter('lidar_align_timeout_sec', 8.0)
        node.declare_parameter('lidar_align_sector_center', math.nan)
        node.declare_parameter('lidar_align_sector_center_base', math.pi)
        node.declare_parameter('lidar_align_sector_width', math.radians(60.0))
        node.declare_parameter('lidar_align_target_line_angle', math.pi / 2.0)
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
        node.declare_parameter(
            'lidar_align_max_rotation', math.radians(15.0))

    def __init__(
            self, node: Node, motion: MotionController,
            lidar: DockingLidar) -> None:
        self.node = node
        self.motion = motion
        self.lidar = lidar

    def align(self) -> bool:
        if not self.validate_parameters():
            return False

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
        start_yaw = self.motion.current_yaw()
        max_rotation = float(
            self.node.get_parameter('lidar_align_max_rotation').value)
        stage_start_sequence = self.lidar.sequence
        stability = UniqueScanStability(
            stable_cycles_required, stage_start_sequence)
        current_command = Twist()
        last_log_time = 0.0

        self.node.get_logger().info('Fine-aligning yaw using LiDAR plane...')

        while rclpy.ok() and not self.motion.should_stop():
            rclpy.spin_once(self.node, timeout_sec=0.0)
            elapsed = (self.node.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.motion.stop_robot()
                self.node.get_logger().error('LiDAR plane alignment timed out')
                return False

            if not self.motion.odom_is_fresh():
                self.motion.stop_robot()
                self.node.get_logger().error(
                    'Odometry became stale during LiDAR alignment')
                return False

            rotation = abs(MotionController.normalize_angle(
                self.motion.current_yaw() - start_yaw))
            if rotation >= max_rotation:
                self.motion.stop_robot()
                self.node.get_logger().error(
                    'LiDAR alignment exceeded the rotation safety limit: '
                    f'{math.degrees(rotation):.2f}deg >= '
                    f'{math.degrees(max_rotation):.2f}deg')
                return False

            snapshot = self.lidar.snapshot()
            if snapshot is None:
                stability.reset()
                current_command = Twist()
                self.motion.cmd_vel_pub.publish(current_command)
                if (
                        time.monotonic() - self.lidar.last_valid_received_at
                        > self.lidar.max_scan_age):
                    self.motion.stop_robot()
                    self.node.get_logger().error(
                        'Docking LiDAR failed during alignment: '
                        f'{self.lidar.last_error}')
                    return False
                rclpy.spin_once(self.node, timeout_sec=sleep_time)
                continue

            if snapshot.sequence == stability.last_sequence:
                self.motion.cmd_vel_pub.publish(current_command)
                rclpy.spin_once(self.node, timeout_sec=sleep_time)
                continue

            estimate = self._estimate_error(snapshot)
            if estimate is None:
                stability.observe(snapshot.sequence, False)
                current_command = Twist()
                now = self.node.get_clock().now().nanoseconds / 1e9
                if now - last_log_time >= 1.0:
                    self.node.get_logger().info('Waiting for a valid LiDAR plane...')
                    last_log_time = now
                self.motion.cmd_vel_pub.publish(current_command)
                rclpy.spin_once(self.node, timeout_sec=sleep_time)
                continue

            error, line_angle, inliers, total_points, line_length = estimate
            aligned = abs(error) <= tolerance
            complete = stability.observe(snapshot.sequence, aligned)
            if aligned:
                current_command = Twist()
            else:
                angular_z = self._clamp(kp * error, -max_speed, max_speed)
                if abs(angular_z) < min_speed:
                    angular_z = math.copysign(min_speed, angular_z)
                angular_z = self._clamp(angular_z, -max_speed, max_speed)
                current_command = MotionController.twist(angular_z=angular_z)

            self.motion.cmd_vel_pub.publish(current_command)
            if complete:
                self.motion.stop_robot()
                self.node.get_logger().info(
                    'LiDAR plane alignment complete: '
                    f'error={math.degrees(error):.2f}deg, '
                    f'line={math.degrees(line_angle):.2f}deg, '
                    f'inliers={inliers}/{total_points}, '
                    f'line_length={line_length:.2f}m, '
                    f'unique_scans={stability.count}')
                return True

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

        self.motion.stop_robot()
        return False

    def _estimate_error(
            self, snapshot: ScanSnapshot
            ) -> tuple[float, float, int, int, float] | None:
        points = self._scan_points_in_sector(snapshot)
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
            self.node.get_parameter('lidar_align_target_line_angle').value)
        error = line_orientation_error(line_angle, target_angle)
        return error, line_angle, len(inliers), len(points), line_length

    def _scan_points_in_sector(
            self, snapshot: ScanSnapshot) -> list[tuple[float, float]]:
        center = float(self.node.get_parameter(
            'lidar_align_sector_center_base').value)
        width = abs(float(
            self.node.get_parameter('lidar_align_sector_width').value))
        min_range = float(self.node.get_parameter('lidar_align_min_range').value)
        max_range = float(self.node.get_parameter('lidar_align_max_range').value)
        projected = self.lidar.project(
            snapshot, center, width, min_range, max_range)
        if projected is None:
            return []
        return [(point.x, point.y) for point in projected]

    def validate_parameters(self) -> bool:
        legacy_center = float(
            self.node.get_parameter('lidar_align_sector_center').value)
        if math.isfinite(legacy_center):
            self.node.get_logger().error(
                'lidar_align_sector_center used the old scan-frame convention. '
                'Use lidar_align_sector_center_base with base_link angles.')
            return False

        values = {
            'timeout': float(self.node.get_parameter(
                'lidar_align_timeout_sec').value),
            'center': float(self.node.get_parameter(
                'lidar_align_sector_center_base').value),
            'width': float(self.node.get_parameter(
                'lidar_align_sector_width').value),
            'target_line': float(self.node.get_parameter(
                'lidar_align_target_line_angle').value),
            'min_range': float(self.node.get_parameter(
                'lidar_align_min_range').value),
            'max_range': float(self.node.get_parameter(
                'lidar_align_max_range').value),
            'tolerance': float(self.node.get_parameter(
                'lidar_align_tolerance').value),
            'kp': float(self.node.get_parameter('lidar_align_kp').value),
            'max_speed': float(self.node.get_parameter(
                'lidar_align_angular_speed').value),
            'min_speed': float(self.node.get_parameter(
                'lidar_align_min_angular_speed').value),
            'max_rotation': float(self.node.get_parameter(
                'lidar_align_max_rotation').value),
            'control_rate': float(self.node.get_parameter(
                'control_rate_hz').value),
            'odom_max_age': float(self.node.get_parameter(
                'odom_max_age_sec').value),
        }
        if not all(math.isfinite(value) for value in values.values()):
            self.node.get_logger().error(
                'LiDAR alignment parameters must all be finite')
            return False

        min_points = int(self.node.get_parameter(
            'lidar_align_min_points').value)
        min_inliers = int(self.node.get_parameter(
            'lidar_align_min_inliers').value)
        iterations = int(self.node.get_parameter(
            'lidar_align_ransac_iterations').value)
        stable_cycles = int(self.node.get_parameter(
            'lidar_align_stable_cycles').value)
        threshold = float(self.node.get_parameter(
            'lidar_align_ransac_threshold').value)
        min_line_length = float(self.node.get_parameter(
            'lidar_align_min_line_length').value)
        if not math.isfinite(threshold) or not math.isfinite(min_line_length):
            self.node.get_logger().error(
                'LiDAR alignment fit parameters must be finite')
            return False

        if (
                values['timeout'] <= 0.0
                or not 0.0 < values['width'] <= 2.0 * math.pi
                or values['min_range'] < 0.0
                or values['max_range'] <= values['min_range']
                or values['tolerance'] < 0.0
                or values['kp'] == 0.0
                or values['max_speed'] <= 0.0
                or not 0.0 < values['min_speed'] <= values['max_speed']
                or values['max_rotation'] <= 0.0
                or values['control_rate'] <= 0.0
                or values['odom_max_age'] <= 0.0
                or min_points < 2
                or min_inliers < 2
                or min_inliers > min_points
                or iterations < 1
                or stable_cycles < 1
                or threshold <= 0.0
                or min_line_length <= 0.0):
            self.node.get_logger().error(
                'LiDAR alignment parameters are outside safe bounds')
            return False

        if abs(normalize_angle(
                math.pi - values['center'])) > values['width'] / 2.0:
            self.node.get_logger().error(
                'LiDAR alignment sector does not include robot rear (pi)')
            return False
        return True

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

    @staticmethod
    def _clamp(value: float, lower: float, upper: float) -> float:
        return min(max(value, lower), upper)
