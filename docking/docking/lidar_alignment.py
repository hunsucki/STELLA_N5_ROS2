import math
import random
import statistics
import time

from docking.docking_lidar import DockingLidar, ScanSnapshot
from docking.lidar_geometry import (
    GuideCenterEstimate,
    line_orientation_error,
    normalize_angle,
    plane_normal_alignment_error,
    UniqueScanStability,
)
from docking.motion import MotionController
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


WallEstimate = tuple[
    float,
    float,
    int,
    int,
    float,
    list[tuple[float, float]],
]


class LidarPlaneAligner:
    @staticmethod
    def declare_parameters(node: Node) -> None:
        node.declare_parameter('use_lidar_alignment', True)
        node.declare_parameter('lidar_align_timeout_sec', 12.0)
        node.declare_parameter('lidar_align_sector_center', math.nan)
        node.declare_parameter('lidar_align_sector_center_base', math.pi)
        # Fit only the dock's rear-facing panel.  A wider fan also sees the
        # station's long side rails, which can dominate RANSAC even though
        # they are not the surface used for docking alignment.
        node.declare_parameter('lidar_align_sector_width', math.radians(60.0))
        node.declare_parameter('lidar_align_target_line_angle', math.pi / 2.0)
        node.declare_parameter('lidar_align_min_range', 0.15)
        node.declare_parameter('lidar_align_max_range', 2.0)
        node.declare_parameter('lidar_align_min_points', 20)
        node.declare_parameter('lidar_align_min_inliers', 12)
        node.declare_parameter('lidar_align_ransac_iterations', 100)
        node.declare_parameter('lidar_align_ransac_threshold', 0.035)
        # The +/-30-degree sector exposes enough of the panel after a slightly
        # imperfect odom turn; keep the length threshold tolerant of occlusion.
        node.declare_parameter('lidar_align_min_line_length', 0.15)
        node.declare_parameter(
            'lidar_align_candidate_max_error', math.radians(15.0))
        node.declare_parameter('lidar_align_tolerance', math.radians(1.0))
        node.declare_parameter('lidar_align_kp', 0.8)
        node.declare_parameter('lidar_align_angular_speed', 0.06)
        node.declare_parameter('lidar_align_min_angular_speed', 0.012)
        node.declare_parameter('lidar_align_stable_cycles', 5)
        node.declare_parameter(
            'lidar_align_max_rotation', math.radians(18.0))
        # Acquire the same stationary plane across multiple unique scans
        # before commanding motion.  This rejects a motion-distorted first
        # scan immediately after the odom spin.
        node.declare_parameter('lidar_align_acquisition_stable_cycles', 3)
        node.declare_parameter(
            'lidar_align_acquisition_max_residual', math.radians(3.0))
        node.declare_parameter(
            'lidar_align_max_tracking_residual', math.radians(5.0))
        # A soft tracking jump stops and reacquires the plane.  Only a much
        # larger jump is treated as an unsafe switch to another structure.
        node.declare_parameter(
            'lidar_align_hard_tracking_residual', math.radians(12.0))
        # The two aluminum guide rails are fitted only after the rear panel
        # establishes the dock-aligned coordinate frame.
        node.declare_parameter('lidar_guide_sector_width', math.radians(150.0))
        node.declare_parameter('lidar_guide_min_range', 0.05)
        node.declare_parameter('lidar_guide_max_range', 1.20)
        node.declare_parameter('lidar_guide_min_abs_y', 0.14)
        node.declare_parameter('lidar_guide_max_abs_y', 0.40)
        node.declare_parameter('lidar_guide_min_separation', 0.44)
        node.declare_parameter('lidar_guide_max_separation', 0.60)
        node.declare_parameter('lidar_guide_wall_x_margin', 0.03)
        node.declare_parameter('lidar_guide_max_x', -0.12)
        node.declare_parameter('lidar_guide_line_threshold', 0.022)
        node.declare_parameter(
            'lidar_guide_orientation_tolerance', math.radians(12.0))
        node.declare_parameter('lidar_guide_min_inliers', 8)
        node.declare_parameter('lidar_guide_min_line_length', 0.15)

    def __init__(
            self, node: Node, motion: MotionController,
            lidar: DockingLidar) -> None:
        self.node = node
        self.motion = motion
        self.lidar = lidar
        self._wall_cache_sequence = -1
        self._wall_cache: WallEstimate | None = None
        self.last_station_alignment_error = 'no station scan evaluated'

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
        tracked_error: float | None = None
        tracked_yaw: float | None = None
        acquisition_error: float | None = None
        acquisition_yaw: float | None = None
        acquisition_count = 0
        acquisition_required = max(int(self.node.get_parameter(
            'lidar_align_acquisition_stable_cycles').value), 1)
        acquisition_max_residual = abs(float(self.node.get_parameter(
            'lidar_align_acquisition_max_residual').value))

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

            estimate = self.estimate_error(snapshot)
            if estimate is None:
                stability.observe(snapshot.sequence, False)
                acquisition_error = None
                acquisition_yaw = None
                acquisition_count = 0
                current_command = Twist()
                now = self.node.get_clock().now().nanoseconds / 1e9
                if now - last_log_time >= 1.0:
                    self.node.get_logger().info(
                        'Waiting for valid dock plane normals: '
                        f'{self.last_station_alignment_error}')
                    last_log_time = now
                self.motion.cmd_vel_pub.publish(current_command)
                rclpy.spin_once(self.node, timeout_sec=sleep_time)
                continue

            error, line_angle, inliers, total_points, line_length = estimate
            current_yaw = self.motion.current_yaw()
            stationary_yaw_rate = abs(float(self.node.get_parameter(
                'imu_stationary_yaw_rate').value))
            stationary_yaw_rate_value, stationary_source = (
                self.motion.current_stationary_yaw_rate())

            if tracked_error is None or tracked_yaw is None:
                stability.observe(snapshot.sequence, False)
                current_command = Twist()
                if abs(stationary_yaw_rate_value) > stationary_yaw_rate:
                    acquisition_error = None
                    acquisition_yaw = None
                    acquisition_count = 0
                else:
                    (
                        acquisition_error,
                        acquisition_yaw,
                        acquisition_count,
                        acquired,
                    ) = self._update_plane_acquisition(
                        acquisition_error,
                        acquisition_yaw,
                        acquisition_count,
                        error,
                        current_yaw,
                        acquisition_max_residual,
                        acquisition_required,
                    )
                    if acquired:
                        tracked_error = error
                        tracked_yaw = current_yaw
                        self.node.get_logger().info(
                            'LiDAR rear plane acquired: '
                            f'error={math.degrees(error):.2f}deg, '
                            f'consistent_scans={acquisition_count}, '
                            f'inliers={inliers}/{total_points}, '
                            f'line_length={line_length:.2f}m')
                self.motion.cmd_vel_pub.publish(current_command)
                rclpy.spin_once(self.node, timeout_sec=sleep_time)
                continue

            if tracked_error is not None and tracked_yaw is not None:
                tracking_residual = self._plane_tracking_residual(
                    tracked_error,
                    tracked_yaw,
                    error,
                    current_yaw,
                )
                max_tracking_residual = abs(float(self.node.get_parameter(
                    'lidar_align_max_tracking_residual').value))
                hard_tracking_residual = abs(float(
                    self.node.get_parameter(
                        'lidar_align_hard_tracking_residual').value))
                tracking_action = self._tracking_jump_action(
                    tracking_residual,
                    max_tracking_residual,
                    hard_tracking_residual,
                )
                if tracking_action != 'accept':
                    if tracking_action == 'abort':
                        self.motion.stop_robot()
                        self.node.get_logger().error(
                            'LiDAR plane tracking changed too far to safely '
                            'reacquire: '
                            f'error={math.degrees(error):.2f}deg, '
                            f'tracking_residual='
                            f'{math.degrees(tracking_residual):.2f}deg, '
                            f'hard_limit='
                            f'{math.degrees(hard_tracking_residual):.2f}deg')
                        return False

                    self.motion.stop_robot()
                    self.node.get_logger().warning(
                        'Rejecting one inconsistent LiDAR plane update and '
                        'reacquiring while stopped: '
                        f'error={math.degrees(error):.2f}deg, '
                        f'tracking_residual='
                        f'{math.degrees(tracking_residual):.2f}deg, '
                        f'limit={math.degrees(max_tracking_residual):.2f}deg')
                    tracked_error = None
                    tracked_yaw = None
                    acquisition_error = None
                    acquisition_yaw = None
                    acquisition_count = 0
                    stability.reset()
                    current_command = Twist()
                    self.motion.cmd_vel_pub.publish(current_command)
                    rclpy.spin_once(self.node, timeout_sec=sleep_time)
                    continue
            tracked_error = error
            tracked_yaw = current_yaw

            within_tolerance = abs(error) <= tolerance
            aligned = (
                within_tolerance
                and abs(stationary_yaw_rate_value) <= stationary_yaw_rate
            )
            complete = stability.observe(snapshot.sequence, aligned)
            if within_tolerance:
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
                    f'normal=rear:{math.degrees(error):.2f}deg, '
                    'mode=rear_only, '
                    f'line={math.degrees(line_angle):.2f}deg, '
                    f'stationary_rate='
                    f'{math.degrees(stationary_yaw_rate_value):.2f}deg/s, '
                    f'stationary_source={stationary_source}, '
                    f'inliers={inliers}/{total_points}, '
                    f'line_length={line_length:.2f}m, '
                    f'unique_scans={stability.count}')
                return True

            now = self.node.get_clock().now().nanoseconds / 1e9
            if now - last_log_time >= 1.0:
                self.node.get_logger().info(
                    'LiDAR align: '
                    f'error={math.degrees(error):.2f}deg, '
                    f'normal=rear:{math.degrees(error):.2f}deg, '
                    'mode=rear_only, '
                    f'cmd={current_command.angular.z:.3f}rad/s, '
                    f'stationary_rate='
                    f'{math.degrees(stationary_yaw_rate_value):.2f}deg/s, '
                    f'stationary_source={stationary_source}, '
                    f'inliers={inliers}/{total_points}, '
                    f'line_length={line_length:.2f}m')
                last_log_time = now

            rclpy.spin_once(self.node, timeout_sec=sleep_time)

        self.motion.stop_robot()
        return False

    def estimate_error(
            self, snapshot: ScanSnapshot
            ) -> tuple[float, float, int, int, float] | None:
        wall = self._estimate_wall(snapshot)
        if wall is None:
            self.last_station_alignment_error = 'rear panel RANSAC is invalid'
            return None
        error, line_angle, inliers, total_points, line_length, _ = wall
        return error, line_angle, inliers, total_points, line_length

    def _estimate_wall(
            self, snapshot: ScanSnapshot) -> WallEstimate | None:
        if getattr(self, '_wall_cache_sequence', -1) == snapshot.sequence:
            return self._wall_cache

        points = self._scan_points_in_sector(snapshot)
        min_points = int(self.node.get_parameter('lidar_align_min_points').value)
        if len(points) < min_points:
            self._cache_wall(snapshot.sequence, None)
            return None

        inliers = self._ransac_line_inliers(points)
        min_inliers = int(self.node.get_parameter('lidar_align_min_inliers').value)
        if len(inliers) < min_inliers:
            self._cache_wall(snapshot.sequence, None)
            return None

        line = self._fit_line_pca(inliers)
        if line is None:
            self._cache_wall(snapshot.sequence, None)
            return None

        line_angle, line_length = line
        min_line_length = float(
            self.node.get_parameter('lidar_align_min_line_length').value)
        if line_length < min_line_length:
            self._cache_wall(snapshot.sequence, None)
            return None

        target_angle = float(
            self.node.get_parameter('lidar_align_target_line_angle').value)
        error = plane_normal_alignment_error(
            line_angle, target_angle + math.pi / 2.0)
        estimate = (
            error,
            line_angle,
            len(inliers),
            len(points),
            line_length,
            inliers,
        )
        self._cache_wall(snapshot.sequence, estimate)
        return estimate

    def _cache_wall(
            self, sequence: int, estimate: WallEstimate | None) -> None:
        self._wall_cache_sequence = sequence
        self._wall_cache = estimate

    def estimate_guide_center(
            self, snapshot: ScanSnapshot) -> GuideCenterEstimate | None:
        wall = self._estimate_wall(snapshot)
        if wall is None:
            return None
        projected = self.lidar.project(
            snapshot,
            math.pi,
            abs(float(self.node.get_parameter(
                'lidar_guide_sector_width').value)),
            float(self.node.get_parameter('lidar_guide_min_range').value),
            float(self.node.get_parameter('lidar_guide_max_range').value),
        )
        if projected is None:
            return None
        wall_error, _, _, _, _, wall_inliers = wall
        return self._estimate_guide_center_from_points(
            wall_error,
            wall_inliers,
            [(point.x, point.y) for point in projected],
        )

    def _estimate_guide_center_from_points(
            self, wall_error: float,
            wall_inliers: list[tuple[float, float]],
            points: list[tuple[float, float]]) -> GuideCenterEstimate | None:
        if not wall_inliers or not points:
            return None

        cos_error = math.cos(-wall_error)
        sin_error = math.sin(-wall_error)

        def rotate(point: tuple[float, float]) -> tuple[float, float]:
            x, y = point
            return (
                cos_error * x - sin_error * y,
                sin_error * x + cos_error * y,
            )

        aligned_wall = [rotate(point) for point in wall_inliers]
        aligned_points = [rotate(point) for point in points]
        wall_x = statistics.median(point[0] for point in aligned_wall)
        wall_margin = abs(float(self.node.get_parameter(
            'lidar_guide_wall_x_margin').value))
        max_x = float(self.node.get_parameter('lidar_guide_max_x').value)
        min_abs_y = abs(float(self.node.get_parameter(
            'lidar_guide_min_abs_y').value))
        max_abs_y = abs(float(self.node.get_parameter(
            'lidar_guide_max_abs_y').value))
        corridor = [
            point for point in aligned_points
            if (
                wall_x - wall_margin <= point[0] <= max_x
                and min_abs_y <= abs(point[1]) <= max_abs_y
            )
        ]
        reference_x = float(self.node.get_parameter(
            'backup_rear_reference_x').value)
        left = self._fit_guide_line(
            [point for point in corridor if point[1] > 0.0],
            reference_x,
        )
        right = self._fit_guide_line(
            [point for point in corridor if point[1] < 0.0],
            reference_x,
        )
        if left is None or right is None:
            return None

        left_y, left_inliers, left_length, left_angle = left
        right_y, right_inliers, right_length, right_angle = right
        separation = left_y - right_y
        min_separation = float(self.node.get_parameter(
            'lidar_guide_min_separation').value)
        max_separation = float(self.node.get_parameter(
            'lidar_guide_max_separation').value)
        if (
                not min_separation <= separation <= max_separation
                or not min_abs_y <= left_y <= max_abs_y
                or not -max_abs_y <= right_y <= -min_abs_y):
            return None

        return GuideCenterEstimate(
            center_offset=0.5 * (left_y + right_y),
            separation=separation,
            left_y=left_y,
            right_y=right_y,
            left_inliers=left_inliers,
            right_inliers=right_inliers,
            left_length=left_length,
            right_length=right_length,
            left_line_angle=left_angle,
            right_line_angle=right_angle,
        )

    def _fit_guide_line(
            self, points: list[tuple[float, float]],
            reference_x: float) -> tuple[float, int, float, float] | None:
        threshold = abs(float(self.node.get_parameter(
            'lidar_guide_line_threshold').value))
        orientation_tolerance = abs(float(self.node.get_parameter(
            'lidar_guide_orientation_tolerance').value))
        min_inliers = int(self.node.get_parameter(
            'lidar_guide_min_inliers').value)
        min_length = abs(float(self.node.get_parameter(
            'lidar_guide_min_line_length').value))
        best: tuple[float, int, float, float] | None = None
        best_score = 0.0
        if len(points) < 2:
            return None

        iterations = max(int(self.node.get_parameter(
            'lidar_align_ransac_iterations').value), 1)
        for _ in range(iterations):
            first, second = random.sample(points, 2)
            dx = second[0] - first[0]
            dy = second[1] - first[1]
            norm = math.hypot(dx, dy)
            if norm < 1e-9:
                continue
            candidate_angle = math.atan2(dy, dx)
            if abs(line_orientation_error(
                    candidate_angle, 0.0)) > orientation_tolerance:
                continue
            inliers = [
                point for point in points
                if abs(
                    dy * (point[0] - first[0])
                    - dx * (point[1] - first[1])
                ) / norm <= threshold
            ]
            if len(inliers) < min_inliers:
                continue
            line = self._fit_line_pca(inliers)
            if line is None:
                continue
            line_angle, line_length = line
            if (
                    abs(line_orientation_error(line_angle, 0.0))
                    > orientation_tolerance
                    or line_length < min_length):
                continue

            mean_x = sum(point[0] for point in inliers) / len(inliers)
            mean_y = sum(point[1] for point in inliers) / len(inliers)
            y_at_reference = mean_y + math.tan(line_angle) * (
                reference_x - mean_x)
            score = len(inliers) * line_length
            if score > best_score:
                best = y_at_reference, len(inliers), line_length, line_angle
                best_score = score
        return best

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
            'candidate_max_error': float(self.node.get_parameter(
                'lidar_align_candidate_max_error').value),
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
            'acquisition_max_residual': float(self.node.get_parameter(
                'lidar_align_acquisition_max_residual').value),
            'max_tracking_residual': float(self.node.get_parameter(
                'lidar_align_max_tracking_residual').value),
            'hard_tracking_residual': float(self.node.get_parameter(
                'lidar_align_hard_tracking_residual').value),
            'guide_width': float(self.node.get_parameter(
                'lidar_guide_sector_width').value),
            'guide_min_range': float(self.node.get_parameter(
                'lidar_guide_min_range').value),
            'guide_max_range': float(self.node.get_parameter(
                'lidar_guide_max_range').value),
            'guide_min_abs_y': float(self.node.get_parameter(
                'lidar_guide_min_abs_y').value),
            'guide_max_abs_y': float(self.node.get_parameter(
                'lidar_guide_max_abs_y').value),
            'guide_min_separation': float(self.node.get_parameter(
                'lidar_guide_min_separation').value),
            'guide_max_separation': float(self.node.get_parameter(
                'lidar_guide_max_separation').value),
            'guide_wall_x_margin': float(self.node.get_parameter(
                'lidar_guide_wall_x_margin').value),
            'guide_max_x': float(self.node.get_parameter(
                'lidar_guide_max_x').value),
            'guide_line_threshold': float(self.node.get_parameter(
                'lidar_guide_line_threshold').value),
            'guide_orientation_tolerance': float(self.node.get_parameter(
                'lidar_guide_orientation_tolerance').value),
            'guide_min_line_length': float(self.node.get_parameter(
                'lidar_guide_min_line_length').value),
            'guide_reference_x': float(self.node.get_parameter(
                'backup_rear_reference_x').value),
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
        acquisition_cycles = int(self.node.get_parameter(
            'lidar_align_acquisition_stable_cycles').value)
        guide_min_inliers = int(self.node.get_parameter(
            'lidar_guide_min_inliers').value)
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
                or not 0.0 < values['candidate_max_error'] < math.pi / 2.0
                or values['tolerance'] < 0.0
                or values['kp'] == 0.0
                or values['max_speed'] <= 0.0
                or not 0.0 < values['min_speed'] <= values['max_speed']
                or values['max_rotation'] <= 0.0
                or not 0.0 < values['acquisition_max_residual'] < (
                    values['max_tracking_residual'])
                or not 0.0 < values['max_tracking_residual'] < math.pi / 2.0
                or not values['max_tracking_residual'] < (
                    values['hard_tracking_residual']) < math.pi / 2.0
                or not 0.0 < values['guide_width'] <= 2.0 * math.pi
                or values['guide_min_range'] < 0.0
                or values['guide_max_range'] <= values['guide_min_range']
                or values['guide_min_abs_y'] <= 0.0
                or values['guide_max_abs_y'] <= values['guide_min_abs_y']
                or values['guide_min_separation'] <= 0.0
                or values['guide_max_separation'] <= (
                    values['guide_min_separation'])
                or values['guide_wall_x_margin'] < 0.0
                or values['guide_line_threshold'] <= 0.0
                or not 0.0 < values['guide_orientation_tolerance'] < (
                    math.pi / 2.0)
                or values['guide_min_line_length'] <= 0.0
                or values['control_rate'] <= 0.0
                or values['odom_max_age'] <= 0.0
                or min_points < 2
                or min_inliers < 2
                or min_inliers > min_points
                or iterations < 1
                or stable_cycles < 1
                or acquisition_cycles < 1
                or guide_min_inliers < 2
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
        target_angle = float(self.node.get_parameter(
            'lidar_align_target_line_angle').value)
        candidate_max_error = abs(float(self.node.get_parameter(
            'lidar_align_candidate_max_error').value))
        best_inliers: list[tuple[float, float]] = []
        best_score = 0.0

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
            line = self._fit_line_pca(inliers)
            if line is None:
                continue
            line_angle, line_length = line
            candidate_error = line_orientation_error(
                line_angle, target_angle)
            if abs(candidate_error) > candidate_max_error:
                continue
            # A short piece of the station frame can contain many densely
            # sampled points, while a long side rail can dominate by length.
            # First reject models that cannot be the rear panel after the
            # odom-controlled 180-degree spin, then prefer physical support.
            score = len(inliers) * line_length
            if (
                    score > best_score
                    or (score == best_score
                        and len(inliers) > len(best_inliers))):
                best_inliers = inliers
                best_score = score

        return best_inliers

    @staticmethod
    def _plane_tracking_residual(
            previous_error: float, previous_yaw: float,
            current_error: float, current_yaw: float) -> float:
        """Compare a new line fit with the same static plane in odom.

        A fixed plane rotates by the negative of the robot yaw change when
        expressed in base_link.  The residual is line-orientation wrapped, so
        the PCA direction ambiguity at +/-pi does not create a false jump.
        """
        yaw_change = normalize_angle(current_yaw - previous_yaw)
        expected_error = previous_error - yaw_change
        return line_orientation_error(current_error, expected_error)

    @classmethod
    def _update_plane_acquisition(
            cls,
            previous_error: float | None,
            previous_yaw: float | None,
            count: int,
            current_error: float,
            current_yaw: float,
            max_residual: float,
            required: int,
            ) -> tuple[float, float, int, bool]:
        """Require consecutive, motion-compensated fits before tracking."""
        consistent = False
        if previous_error is not None and previous_yaw is not None:
            residual = cls._plane_tracking_residual(
                previous_error,
                previous_yaw,
                current_error,
                current_yaw,
            )
            consistent = abs(residual) <= abs(max_residual)

        new_count = count + 1 if consistent else 1
        required = max(int(required), 1)
        return current_error, current_yaw, new_count, new_count >= required

    @staticmethod
    def _tracking_jump_action(
            residual: float, soft_limit: float,
            hard_limit: float) -> str:
        magnitude = abs(residual)
        if magnitude > abs(hard_limit):
            return 'abort'
        if magnitude > abs(soft_limit):
            return 'reacquire'
        return 'accept'

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
