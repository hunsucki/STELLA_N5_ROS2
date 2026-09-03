import math
import time
from typing import Any, Callable

from docking.docking_lidar import DockingLidar, ScanSnapshot
from docking.lidar_geometry import (
    effective_range_limits,
    GuideCenterEstimate,
    has_consecutive_clearance_cluster,
    normalize_angle,
    rear_clearances,
    required_rear_range_at_completion,
    UniqueScanStability,
)
from geometry_msgs.msg import Twist, Vector3Stamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class MotionController:
    @staticmethod
    def declare_parameters(node: Node) -> None:
        node.declare_parameter('cmd_vel_topic', '/cmd_vel')
        node.declare_parameter('odom_topic', '/odom')
        node.declare_parameter('imu_topic', '/imu/data')
        node.declare_parameter('imu_max_age_sec', 0.25)
        node.declare_parameter(
            'imu_stationary_yaw_rate', math.radians(0.5))
        node.declare_parameter(
            'wheel_yaw_diagnostics_topic',
            '/wheel_odometry/yaw_diagnostics')
        node.declare_parameter('wheel_yaw_max_age_sec', 0.50)
        node.declare_parameter('spin_yaw', math.pi)
        node.declare_parameter('spin_angular_speed', 0.15)
        node.declare_parameter('spin_min_angular_speed', 0.025)
        node.declare_parameter('spin_slowdown_angle', math.radians(40.0))
        node.declare_parameter('spin_tolerance', math.radians(1.0))
        node.declare_parameter('spin_stable_cycles', 5)
        node.declare_parameter('use_pre_spin_forward', False)
        node.declare_parameter('pre_spin_forward_duration_sec', 1.0)
        node.declare_parameter('pre_spin_forward_speed', 0.03)
        node.declare_parameter('pre_spin_forward_max_distance', 0.05)
        node.declare_parameter('backup_distance', 0.50)
        node.declare_parameter('backup_speed', 0.05)
        node.declare_parameter('backup_tolerance', 0.02)
        node.declare_parameter('use_lidar_backup', True)
        node.declare_parameter('backup_lidar_sector_center', math.nan)
        node.declare_parameter('backup_lidar_sector_center_base', math.pi)
        node.declare_parameter('backup_lidar_sector_width', math.radians(20.0))
        node.declare_parameter(
            # The rear-mounted scanner's local 0 degrees points toward the
            # robot rear.  Protect only +/-30 degrees around that direction;
            # the former +/-75-degree fan included the station's side rails.
            'backup_lidar_safety_sector_width', math.radians(60.0))
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
        # The stationary LiDAR step establishes dock heading. During reverse,
        # apply only a very weak, filtered rear-plane correction.
        node.declare_parameter('backup_heading_kp', 0.20)
        node.declare_parameter('backup_heading_kd', 0.0)
        # ROS angular.z keeps the same sign while reversing.  The measured
        # response from the failed run also followed the commanded sign.
        node.declare_parameter('backup_reverse_angular_command_sign', 1.0)
        node.declare_parameter(
            'backup_heading_tolerance', math.radians(1.0))
        node.declare_parameter('backup_heading_max_angular_speed', 0.004)
        node.declare_parameter('backup_heading_max_angular_accel', 0.010)
        node.declare_parameter(
            'backup_heading_pause_error', math.radians(3.0))
        node.declare_parameter('backup_heading_resume_stable_cycles', 3)
        node.declare_parameter('use_lidar_heading_during_backup', True)
        node.declare_parameter('backup_lidar_heading_filter_coef', 0.15)
        node.declare_parameter(
            'backup_lidar_heading_max_error', math.radians(5.0))
        node.declare_parameter('backup_lidar_heading_min_inlier_ratio', 0.70)
        node.declare_parameter('backup_lidar_heading_min_line_length', 0.15)
        node.declare_parameter(
            'backup_lidar_heading_max_jump', math.radians(2.5))
        node.declare_parameter('backup_lidar_heading_stable_cycles', 3)
        node.declare_parameter(
            'backup_lidar_heading_disable_clearance', 0.10)
        node.declare_parameter(
            'backup_lidar_heading_rebase_error', math.radians(1.0))
        node.declare_parameter('use_lidar_guide_centering', False)
        node.declare_parameter('backup_guide_center_kp', 0.35)
        node.declare_parameter('backup_guide_center_tolerance', 0.015)
        node.declare_parameter(
            'backup_guide_center_max_heading', math.radians(2.0))
        node.declare_parameter('backup_guide_center_filter_coef', 0.20)
        node.declare_parameter('backup_guide_center_max_jump', 0.05)
        node.declare_parameter('backup_guide_center_stable_cycles', 3)
        node.declare_parameter(
            'backup_guide_center_disable_clearance', 0.12)
        node.declare_parameter('backup_slowdown_clearance', 0.15)
        node.declare_parameter('backup_min_speed', 0.015)
        node.declare_parameter('backup_blocked_timeout_sec', 1.0)
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
        self.imu_sub = node.create_subscription(
            Imu,
            node.get_parameter('imu_topic').value,
            self._imu_callback,
            # The STELLA AHRS publisher is RELIABLE/KEEP_LAST(10).  Matching
            # it avoids avoidable best-effort loss during CPU-heavy docking.
            10,
        )
        self.wheel_yaw_sub = node.create_subscription(
            Vector3Stamped,
            node.get_parameter('wheel_yaw_diagnostics_topic').value,
            self._wheel_yaw_callback,
            10,
        )
        self.last_odom: Odometry | None = None
        self.last_odom_received_at = 0.0
        self.last_imu_yaw_rate: float | None = None
        self.last_imu_received_at = 0.0
        self.last_imu_stamp_nanoseconds = 0
        self.integrated_imu_yaw = 0.0
        self.last_wheel_yaw: float | None = None
        self.last_wheel_yaw_received_at = 0.0
        self.backup_lidar_heading_estimator: (
            Callable[[ScanSnapshot], tuple[float, float, int, int, float] | None]
            | None
        ) = None
        self.backup_lidar_guide_estimator: (
            Callable[[ScanSnapshot], GuideCenterEstimate | None] | None
        ) = None

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
        requested_yaw = float(self.node.get_parameter('spin_yaw').value)
        max_speed = abs(float(
            self.node.get_parameter('spin_angular_speed').value))
        min_speed = min(max_speed, abs(float(
            self.node.get_parameter('spin_min_angular_speed').value)))
        slowdown_angle = abs(float(
            self.node.get_parameter('spin_slowdown_angle').value))
        tolerance = abs(float(self.node.get_parameter('spin_tolerance').value))
        stationary_yaw_rate = abs(float(
            self.node.get_parameter('imu_stationary_yaw_rate').value))
        stable_cycles = max(int(
            self.node.get_parameter('spin_stable_cycles').value), 1)
        direction = 1.0 if requested_yaw >= 0.0 else -1.0
        start_yaw = self.current_yaw()
        target_yaw = self.normalize_angle(start_yaw + requested_yaw)

        self.node.get_logger().info(
            'Spinning to an absolute odom yaw target: '
            f'start={math.degrees(start_yaw):.2f}deg, '
            f'target={math.degrees(target_yaw):.2f}deg, '
            f'delta={math.degrees(requested_yaw):.2f}deg, '
            f'max_speed={max_speed:.3f}rad/s')

        last_stationary_source = 'imu'

        def spin_command() -> Twist:
            nonlocal last_stationary_source
            _, stationary_source = self.current_stationary_yaw_rate()
            if stationary_source != last_stationary_source:
                if stationary_source == 'wheel_odom':
                    self.node.get_logger().warning(
                        'IMU delivery is temporarily stale during the spin; '
                        'continuing the absolute odom target with wheel odom '
                        'and using wheel angular velocity for settling')
                else:
                    self.node.get_logger().info(
                        'Fresh IMU delivery recovered during the spin')
                last_stationary_source = stationary_source
            return self.twist(angular_z=self._spin_angular_velocity(
                target_yaw,
                direction,
                tolerance,
                slowdown_angle,
                min_speed,
                max_speed,
            ))

        ok = self._run_until(
            done_cb=self._make_absolute_spin_done_cb(
                target_yaw,
                tolerance,
                stable_cycles,
                direction,
                stationary_yaw_rate,
            ),
            cmd_cb=spin_command,
            timeout_sec=float(self.node.get_parameter('motion_timeout_sec').value),
        )
        self.stop_robot()
        final_yaw = self.current_yaw()
        final_error = self._absolute_yaw_error(
            target_yaw, final_yaw, direction)
        final_stationary_rate, final_stationary_source = (
            self.current_stationary_yaw_rate())

        if ok:
            self.node.get_logger().info(
                'Spin step complete: '
                f'final={math.degrees(final_yaw):.2f}deg, '
                f'target_error={math.degrees(final_error):.2f}deg, '
                f'stationary_rate='
                f'{math.degrees(final_stationary_rate):.2f}deg/s, '
                f'stationary_source={final_stationary_source}')
        else:
            self.node.get_logger().error(
                'Spin step failed or timed out: '
                f'final={math.degrees(final_yaw):.2f}deg, '
                f'target_error={math.degrees(final_error):.2f}deg, '
                f'imu_age={self.imu_age_sec():.3f}s, '
                f'stationary_source={final_stationary_source}')
        return ok

    def advance_before_spin(self) -> bool:
        if not bool(self.node.get_parameter('use_pre_spin_forward').value):
            self.node.get_logger().info(
                'Optional pre-spin forward step is disabled')
            return True

        duration = float(self.node.get_parameter(
            'pre_spin_forward_duration_sec').value)
        speed = float(self.node.get_parameter(
            'pre_spin_forward_speed').value)
        max_distance = float(self.node.get_parameter(
            'pre_spin_forward_max_distance').value)
        if not all(math.isfinite(value) for value in (
                duration, speed, max_distance)) or (
                duration < 0.0 or speed <= 0.0 or max_distance <= 0.0):
            self.node.get_logger().error(
                'Pre-spin forward parameters are outside safe bounds')
            return False
        if duration == 0.0:
            return True

        start_time = time.monotonic()
        start_x, start_y = self.current_xy()

        def traveled() -> float:
            current_x, current_y = self.current_xy()
            return math.hypot(current_x - start_x, current_y - start_y)

        self.node.get_logger().info(
            'Advancing slowly before the odom spin: '
            f'duration={duration:.2f}s, speed={speed:.3f}m/s')
        ok = self._run_until(
            done_cb=lambda: time.monotonic() - start_time >= duration,
            cmd_cb=lambda: self.twist(linear_x=speed),
            timeout_sec=duration + 1.0,
            health_cb=lambda: traveled() <= max_distance,
            health_error=(
                'Pre-spin forward motion exceeded the distance safety limit'),
        )
        self.stop_robot()
        distance = traveled()
        if ok:
            self.node.get_logger().info(
                'Pre-spin forward step complete: '
                f'odom_distance={distance:.3f}m')
        else:
            self.node.get_logger().error(
                'Pre-spin forward step failed: '
                f'odom_distance={distance:.3f}m, limit={max_distance:.3f}m')
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
        heading_kp = abs(float(
            self.node.get_parameter('backup_heading_kp').value))
        heading_kd = abs(float(
            self.node.get_parameter('backup_heading_kd').value))
        reverse_angular_command_sign = float(self.node.get_parameter(
            'backup_reverse_angular_command_sign').value)
        heading_tolerance = abs(float(
            self.node.get_parameter('backup_heading_tolerance').value))
        heading_max_speed = abs(float(self.node.get_parameter(
            'backup_heading_max_angular_speed').value))
        heading_max_accel = abs(float(self.node.get_parameter(
            'backup_heading_max_angular_accel').value))
        heading_pause_error = abs(float(self.node.get_parameter(
            'backup_heading_pause_error').value))
        heading_resume_stable_cycles = max(int(self.node.get_parameter(
            'backup_heading_resume_stable_cycles').value), 1)
        stationary_yaw_rate = abs(float(self.node.get_parameter(
            'imu_stationary_yaw_rate').value))
        use_lidar_heading = bool(self.node.get_parameter(
            'use_lidar_heading_during_backup').value)
        lidar_heading_filter_coef = float(self.node.get_parameter(
            'backup_lidar_heading_filter_coef').value)
        lidar_heading_max_error = abs(float(self.node.get_parameter(
            'backup_lidar_heading_max_error').value))
        lidar_heading_min_inlier_ratio = float(self.node.get_parameter(
            'backup_lidar_heading_min_inlier_ratio').value)
        lidar_heading_min_line_length = abs(float(self.node.get_parameter(
            'backup_lidar_heading_min_line_length').value))
        lidar_heading_max_jump = abs(float(self.node.get_parameter(
            'backup_lidar_heading_max_jump').value))
        lidar_heading_stable_cycles = max(int(self.node.get_parameter(
            'backup_lidar_heading_stable_cycles').value), 1)
        lidar_heading_disable_clearance = abs(float(self.node.get_parameter(
            'backup_lidar_heading_disable_clearance').value))
        lidar_heading_rebase_error = abs(float(self.node.get_parameter(
            'backup_lidar_heading_rebase_error').value))
        use_guide_centering = bool(self.node.get_parameter(
            'use_lidar_guide_centering').value)
        guide_center_kp = abs(float(self.node.get_parameter(
            'backup_guide_center_kp').value))
        guide_center_tolerance = abs(float(self.node.get_parameter(
            'backup_guide_center_tolerance').value))
        guide_center_max_heading = abs(float(self.node.get_parameter(
            'backup_guide_center_max_heading').value))
        guide_center_filter_coef = float(self.node.get_parameter(
            'backup_guide_center_filter_coef').value)
        guide_center_max_jump = abs(float(self.node.get_parameter(
            'backup_guide_center_max_jump').value))
        guide_center_stable_cycles = max(int(self.node.get_parameter(
            'backup_guide_center_stable_cycles').value), 1)
        guide_center_disable_clearance = abs(float(self.node.get_parameter(
            'backup_guide_center_disable_clearance').value))
        blocked_timeout = float(
            self.node.get_parameter('backup_blocked_timeout_sec').value)
        completion_half_angle = math.degrees(abs(float(
            self.node.get_parameter('backup_lidar_sector_width').value))) / 2.0
        safety_half_angle = math.degrees(abs(float(self.node.get_parameter(
            'backup_lidar_safety_sector_width').value))) / 2.0
        timeout_sec = float(self.node.get_parameter('motion_timeout_sec').value)
        rate_hz = float(self.node.get_parameter('control_rate_hz').value)
        sleep_time = 1.0 / max(rate_hz, 1.0)
        start = self.node.get_clock().now()
        start_x, start_y = self.current_xy()
        start_odom_yaw = self.current_yaw()
        if not self.wheel_yaw_is_fresh():
            self.stop_robot()
            self.node.get_logger().error(
                'Wheel-only yaw diagnostics are unavailable; refusing LiDAR '
                'backup because IMU-corrected odom yaw can settle after the '
                '180-degree spin')
            return False
        start_wheel_yaw = self.current_wheel_yaw()
        start_integrated_imu_yaw = self.current_integrated_imu_yaw()
        imu_heading_reference = start_integrated_imu_yaw
        wheel_heading_reference = start_wheel_yaw
        stage_start_sequence = self.lidar.sequence
        stability = UniqueScanStability(
            stable_cycles_required, stage_start_sequence)
        current_command = Twist()
        filtered_lidar_heading_error: float | None = None
        lidar_heading_error: float | None = None
        lidar_heading_stable_count = 0
        lidar_heading_status = 'not_evaluated'
        lidar_heading_control_available = False
        guide_center_offset: float | None = None
        filtered_guide_center_offset: float | None = None
        guide_separation: float | None = None
        guide_stable_count = 0
        guide_center_heading = 0.0
        heading_source = 'imu_gyro'
        heading_correction_active = False
        heading_settled_count = 0
        last_heading_command_at = time.monotonic()
        blocked_since: float | None = None
        last_log_time = 0.0

        self.node.get_logger().info(
            'Backing up using LiDAR rear clearance: '
            f'target={target_clearance:.3f}m, speed={speed:.3f}m/s, '
            f'completion_sector=rear+/-{completion_half_angle:.1f}deg, '
            f'safety_sector=rear+/-{safety_half_angle:.1f}deg')

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
            if not self.wheel_yaw_is_fresh():
                self.stop_robot()
                self.node.get_logger().error(
                    'Wheel-only yaw diagnostics became stale during LiDAR backup')
                return False
            current_x, current_y = self.current_xy()
            traveled = math.hypot(current_x - start_x, current_y - start_y)
            if traveled >= max_travel:
                self.stop_robot()
                self.node.get_logger().error(
                    'LiDAR backup exceeded the travel safety limit: '
                    f'{traveled:.3f}m >= {max_travel:.3f}m')
                return False

            wheel_yaw_drift = self.normalize_angle(
                self.current_wheel_yaw() - start_wheel_yaw)
            odom_yaw_drift = self.normalize_angle(
                self.current_yaw() - start_odom_yaw)
            imu_yaw_from_start = (
                self.current_integrated_imu_yaw()
                - start_integrated_imu_yaw)
            imu_heading_drift = (
                self.current_integrated_imu_yaw()
                - imu_heading_reference)
            wheel_heading_drift = self.normalize_angle(
                self.current_wheel_yaw() - wheel_heading_reference)
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

            lidar_heading_error = None
            lidar_heading_control_available = False
            heading_source = (
                'imu_gyro' if self.imu_is_fresh() else 'wheel_odom')
            if (
                    use_lidar_heading
                    and protective_clearance
                    > lidar_heading_disable_clearance
                    and self.backup_lidar_heading_estimator is not None):
                estimate = self.backup_lidar_heading_estimator(snapshot)
                if estimate is not None:
                    (candidate_error, _, inliers, total_points,
                     line_length) = estimate
                    inlier_ratio = inliers / max(total_points, 1)
                    if (
                            abs(candidate_error) <= lidar_heading_max_error
                            and inlier_ratio >= lidar_heading_min_inlier_ratio
                            and line_length >= lidar_heading_min_line_length):
                        lidar_heading_error = candidate_error
                        (
                            next_filtered_error,
                            sample_accepted,
                        ) = self._filter_lidar_heading_sample(
                            filtered_lidar_heading_error,
                            candidate_error,
                            lidar_heading_filter_coef,
                            lidar_heading_max_jump,
                        )
                        if sample_accepted:
                            filtered_lidar_heading_error = next_filtered_error
                            lidar_heading_stable_count += 1
                            lidar_heading_status = 'warming_up'
                        else:
                            lidar_heading_stable_count = 0
                            lidar_heading_status = 'jump_rejected'
                        if (
                                sample_accepted
                                and lidar_heading_stable_count
                                >= lidar_heading_stable_cycles):
                            heading_source = 'lidar'
                            lidar_heading_control_available = True
                            lidar_heading_status = 'tracking'
                        if (
                                sample_accepted
                                and abs(candidate_error)
                                <= lidar_heading_rebase_error):
                            # A trustworthy near-parallel plane establishes a
                            # new gyro reference. Intentional LiDAR corrections
                            # must not accumulate as a safety-limit violation.
                            imu_heading_reference = (
                                self.current_integrated_imu_yaw())
                            wheel_heading_reference = self.current_wheel_yaw()
                            imu_heading_drift = 0.0
                            wheel_heading_drift = 0.0
                    else:
                        filtered_lidar_heading_error = None
                        lidar_heading_stable_count = 0
                        lidar_heading_status = 'quality_rejected'
                else:
                    filtered_lidar_heading_error = None
                    lidar_heading_stable_count = 0
                    lidar_heading_status = 'ransac_unavailable'
                if not lidar_heading_control_available:
                    heading_source = 'lidar_unavailable'
            elif use_lidar_heading:
                filtered_lidar_heading_error = None
                lidar_heading_stable_count = 0
                lidar_heading_status = (
                    'disabled_near_dock'
                    if protective_clearance
                    <= lidar_heading_disable_clearance
                    else 'estimator_unavailable')
                heading_source = 'lidar_disabled'

            guide_center_offset = None
            guide_separation = None
            guide_center_heading = 0.0
            if (
                    use_guide_centering
                    and self.backup_lidar_guide_estimator is not None):
                guide = self.backup_lidar_guide_estimator(snapshot)
                if guide is not None:
                    guide_center_offset = guide.center_offset
                    guide_separation = guide.separation
                    if (
                            filtered_guide_center_offset is None
                            or abs(
                                guide.center_offset
                                - filtered_guide_center_offset
                            ) > guide_center_max_jump):
                        filtered_guide_center_offset = guide.center_offset
                        guide_stable_count = 1
                    else:
                        filtered_guide_center_offset += (
                            guide_center_filter_coef
                            * (guide.center_offset
                               - filtered_guide_center_offset))
                        guide_stable_count += 1
                    if (
                            guide_stable_count >= guide_center_stable_cycles
                            and protective_clearance
                            > guide_center_disable_clearance):
                        guide_center_heading = (
                            self._backup_guide_center_heading(
                                filtered_guide_center_offset,
                                guide_center_tolerance,
                                guide_center_kp,
                                guide_center_max_heading,
                            ))
                else:
                    guide_stable_count = 0
            else:
                guide_stable_count = 0

            # A validated dock plane is the physical heading authority and may
            # legitimately request several degrees of correction. Apply the
            # dead-reckoning drift limit only when that plane is unavailable.
            fallback_heading_drift = (
                imu_heading_drift if self.imu_is_fresh()
                else wheel_heading_drift)
            if self._backup_dead_reckoning_drift_exceeded(
                    heading_source, fallback_heading_drift, max_yaw_drift):
                self.stop_robot()
                self.node.get_logger().error(
                    'LiDAR backup lost its dock plane and exceeded the '
                    f'{heading_source} yaw drift safety limit: '
                    f'{math.degrees(abs(fallback_heading_drift)):.2f}deg >= '
                    f'{math.degrees(max_yaw_drift):.2f}deg, '
                    f'lidar_heading_error='
                    f'{self._format_optional_degrees(lidar_heading_error)}')
                return False

            stationary_yaw_rate_value, stationary_source = (
                self.current_stationary_yaw_rate())
            if protective_clearance <= completion_threshold:
                current_command = Twist()
                if blocked_since is None:
                    blocked_since = time.monotonic()
            else:
                blocked_since = None
                command_speed = self._backup_speed_for_clearance(
                    protective_clearance,
                    target_clearance,
                    slowdown_clearance,
                    min_speed,
                    speed,
                )
                if lidar_heading_control_available:
                    wall_heading_error = filtered_lidar_heading_error or 0.0
                    heading_error = wall_heading_error
                elif use_lidar_heading:
                    # Fail open-loop straight: never preserve a stale angular
                    # command when RANSAC is missing, rejected, or too close.
                    heading_error = 0.0
                else:
                    # Hold the last LiDAR-confirmed physical heading. Prefer
                    # gyro integration, then fall back to encoder-only yaw if
                    # raw IMU delivery is temporarily stale.
                    heading_error = -fallback_heading_drift
                heading_error = normalize_angle(
                    heading_error + guide_center_heading)

                # Never stop translation to rotate inside the station.
                heading_correction_active = False
                heading_settled_count = 0

                requested_heading_command = (
                    self._backup_heading_angular_velocity(
                        heading_error,
                        stationary_yaw_rate_value,
                        heading_tolerance,
                        heading_kp,
                        heading_kd,
                        heading_max_speed,
                        reverse_angular_command_sign,
                    ))
                command_time = time.monotonic()
                if use_lidar_heading and not lidar_heading_control_available:
                    heading_command = 0.0
                else:
                    heading_command = self._limit_command_rate(
                        current_command.angular.z,
                        requested_heading_command,
                        heading_max_accel * max(
                            command_time - last_heading_command_at, 0.0),
                    )
                last_heading_command_at = command_time
                current_command = self.twist(
                    linear_x=(
                        0.0 if heading_correction_active
                        else -command_speed),
                    angular_z=heading_command,
                )

            self.cmd_vel_pub.publish(current_command)
            if complete:
                self.stop_robot()
                self.node.get_logger().info(
                    'Dock-turn-backup sequence complete: '
                    f'rear_clearance={clearance:.3f}m, '
                    f'unique_scans={stability.count}')
                return True

            if (
                    blocked_since is not None
                    and time.monotonic() - blocked_since >= blocked_timeout):
                self.stop_robot()
                self.node.get_logger().error(
                    'LiDAR backup remains blocked outside the completion '
                    'condition: '
                    f'center_clearance={clearance:.3f}m, '
                    f'protective_clearance={protective_clearance:.3f}m, '
                    f'safety_sector=rear+/-'
                    f'{math.degrees(float(self.node.get_parameter(
                        "backup_lidar_safety_sector_width").value)) / 2.0:.1f}deg')
                return False

            now = self.node.get_clock().now().nanoseconds / 1e9
            if now - last_log_time >= 1.0:
                heading_mode = (
                    'correcting' if heading_correction_active else 'drive')
                self.node.get_logger().info(
                    f'LiDAR backup: rear_clearance={clearance:.3f}m, '
                    f'target={target_clearance:.3f}m, '
                    f'protective={protective_clearance:.3f}m, '
                    f'traveled={traveled:.3f}m, '
                    f'wheel_yaw_drift={math.degrees(wheel_yaw_drift):.2f}deg, '
                    f'odom_yaw_drift={math.degrees(odom_yaw_drift):.2f}deg, '
                    f'imu_yaw_from_start='
                    f'{math.degrees(imu_yaw_from_start):.2f}deg, '
                    f'imu_heading_drift='
                    f'{math.degrees(imu_heading_drift):.2f}deg, '
                    f'wheel_heading_drift='
                    f'{math.degrees(wheel_heading_drift):.2f}deg, '
                    f'lidar_heading_error='
                    f'{self._format_optional_degrees(lidar_heading_error)}, '
                    f'filtered_heading_error='
                    f'{self._format_optional_degrees(
                        filtered_lidar_heading_error)}, '
                    f'lidar_heading_status={lidar_heading_status}, '
                    f'lidar_heading_stable={lidar_heading_stable_count}/'
                    f'{lidar_heading_stable_cycles}, '
                    f'guide_center_offset='
                    f'{self._format_optional_meters(guide_center_offset)}, '
                    f'filtered_guide_center='
                    f'{self._format_optional_meters(
                        filtered_guide_center_offset)}, '
                    f'guide_separation='
                    f'{self._format_optional_meters(guide_separation)}, '
                    f'guide_stable={guide_stable_count}/'
                    f'{guide_center_stable_cycles}, '
                    f'guide_heading='
                    f'{math.degrees(guide_center_heading):.2f}deg, '
                    f'heading_source={heading_source}, '
                    f'heading_mode={heading_mode}, '
                    f'stationary_rate='
                    f'{math.degrees(stationary_yaw_rate_value):.2f}deg/s, '
                    f'stationary_source={stationary_source}, '
                    f'heading_cmd={current_command.angular.z:.3f}rad/s')
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

    def _run_until(
            self, done_cb: Any, cmd_cb: Any, timeout_sec: float,
            health_cb: Callable[[], bool] | None = None,
            health_error: str = 'Motion sensor health check failed') -> bool:
        rate_hz = float(self.node.get_parameter('control_rate_hz').value)
        sleep_time = 1.0 / max(rate_hz, 1.0)
        start = self.node.get_clock().now()

        while rclpy.ok() and not self.should_stop():
            rclpy.spin_once(self.node, timeout_sec=0.0)
            if not self.odom_is_fresh():
                self.node.get_logger().error('Odometry became stale during motion')
                return False
            if health_cb is not None and not health_cb():
                self.node.get_logger().error(health_error)
                return False
            if done_cb():
                return True

            elapsed = (self.node.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout_sec:
                return False

            self.cmd_vel_pub.publish(cmd_cb())
            rclpy.spin_once(self.node, timeout_sec=sleep_time)

        return False

    def _make_absolute_spin_done_cb(
            self, target_yaw: float, tolerance: float,
            stable_cycles: int, direction: float,
            stationary_yaw_rate: float) -> Any:
        stable_count = 0

        def done() -> bool:
            nonlocal stable_count
            error = self._absolute_yaw_error(
                target_yaw, self.current_yaw(), direction)
            stationary_yaw_rate_value, _ = (
                self.current_stationary_yaw_rate())
            if (
                    abs(error) <= tolerance
                    and abs(stationary_yaw_rate_value)
                    <= stationary_yaw_rate):
                stable_count += 1
            else:
                stable_count = 0
            return stable_count >= max(stable_cycles, 1)

        return done

    def _spin_angular_velocity(
            self, target_yaw: float, direction: float, tolerance: float,
            slowdown_angle: float, min_speed: float,
            max_speed: float) -> float:
        error = self._absolute_yaw_error(
            target_yaw, self.current_yaw(), direction)
        if abs(error) <= tolerance:
            return 0.0

        if slowdown_angle <= tolerance or abs(error) >= slowdown_angle:
            speed = max_speed
        else:
            ratio = (abs(error) - tolerance) / (slowdown_angle - tolerance)
            speed = min_speed + max(min(ratio, 1.0), 0.0) * (
                max_speed - min_speed)
        return math.copysign(speed, error)

    @staticmethod
    def _absolute_yaw_error(
            target_yaw: float, current_yaw: float, direction: float) -> float:
        error = normalize_angle(target_yaw - current_yaw)
        # Exactly 180 degrees has two equally short solutions.  Preserve the
        # configured spin direction instead of letting floating-point wrap
        # choose a different direction from run to run.
        if abs(abs(error) - math.pi) <= 1e-9:
            return math.copysign(math.pi, direction)
        return error

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

    def _imu_callback(self, msg: Imu) -> None:
        if not math.isfinite(msg.angular_velocity.z):
            return
        stamp_nanoseconds = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec))
        if (
                self.last_imu_stamp_nanoseconds > 0
                and stamp_nanoseconds > self.last_imu_stamp_nanoseconds):
            dt = (
                stamp_nanoseconds - self.last_imu_stamp_nanoseconds
            ) / 1e9
            if dt <= 0.1 and self.last_imu_yaw_rate is not None:
                self.integrated_imu_yaw += 0.5 * (
                    self.last_imu_yaw_rate + msg.angular_velocity.z) * dt
        self.last_imu_yaw_rate = msg.angular_velocity.z
        self.last_imu_stamp_nanoseconds = stamp_nanoseconds
        self.last_imu_received_at = time.monotonic()

    def _wheel_yaw_callback(self, msg: Vector3Stamped) -> None:
        # wheel_odometry/yaw_diagnostics.vector.x is encoder-only yaw;
        # vector.y is IMU yaw and vector.z is the fused odom yaw.
        if not math.isfinite(msg.vector.x):
            return
        self.last_wheel_yaw = msg.vector.x
        self.last_wheel_yaw_received_at = time.monotonic()

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
        heading_kp = float(
            self.node.get_parameter('backup_heading_kp').value)
        heading_kd = float(
            self.node.get_parameter('backup_heading_kd').value)
        reverse_angular_command_sign = float(self.node.get_parameter(
            'backup_reverse_angular_command_sign').value)
        heading_tolerance = float(
            self.node.get_parameter('backup_heading_tolerance').value)
        heading_max_speed = float(self.node.get_parameter(
            'backup_heading_max_angular_speed').value)
        heading_max_accel = float(self.node.get_parameter(
            'backup_heading_max_angular_accel').value)
        heading_pause_error = float(self.node.get_parameter(
            'backup_heading_pause_error').value)
        lidar_heading_filter_coef = float(self.node.get_parameter(
            'backup_lidar_heading_filter_coef').value)
        lidar_heading_max_error = float(self.node.get_parameter(
            'backup_lidar_heading_max_error').value)
        lidar_heading_min_inlier_ratio = float(self.node.get_parameter(
            'backup_lidar_heading_min_inlier_ratio').value)
        lidar_heading_min_line_length = float(self.node.get_parameter(
            'backup_lidar_heading_min_line_length').value)
        lidar_heading_max_jump = float(self.node.get_parameter(
            'backup_lidar_heading_max_jump').value)
        lidar_heading_disable_clearance = float(self.node.get_parameter(
            'backup_lidar_heading_disable_clearance').value)
        lidar_heading_rebase_error = float(self.node.get_parameter(
            'backup_lidar_heading_rebase_error').value)
        guide_center_kp = float(self.node.get_parameter(
            'backup_guide_center_kp').value)
        guide_center_tolerance = float(self.node.get_parameter(
            'backup_guide_center_tolerance').value)
        guide_center_max_heading = float(self.node.get_parameter(
            'backup_guide_center_max_heading').value)
        guide_center_filter_coef = float(self.node.get_parameter(
            'backup_guide_center_filter_coef').value)
        guide_center_max_jump = float(self.node.get_parameter(
            'backup_guide_center_max_jump').value)
        guide_center_disable_clearance = float(self.node.get_parameter(
            'backup_guide_center_disable_clearance').value)
        wheel_yaw_max_age = float(
            self.node.get_parameter('wheel_yaw_max_age_sec').value)
        imu_max_age = float(
            self.node.get_parameter('imu_max_age_sec').value)
        imu_stationary_yaw_rate = float(
            self.node.get_parameter('imu_stationary_yaw_rate').value)
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
        heading_resume_stable_cycles = int(self.node.get_parameter(
            'backup_heading_resume_stable_cycles').value)
        guide_center_stable_cycles = int(self.node.get_parameter(
            'backup_guide_center_stable_cycles').value)
        lidar_heading_stable_cycles = int(self.node.get_parameter(
            'backup_lidar_heading_stable_cycles').value)
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
        blocked_timeout = float(
            self.node.get_parameter('backup_blocked_timeout_sec').value)

        finite_values = (
            target, tolerance,
            rear_x, configured_min, configured_max,
            speed, min_speed, slowdown, max_travel, max_yaw_drift,
            heading_kp, heading_kd, reverse_angular_command_sign,
            heading_tolerance, heading_max_speed, heading_max_accel,
            heading_pause_error,
            lidar_heading_filter_coef, lidar_heading_max_error,
            lidar_heading_min_inlier_ratio, lidar_heading_min_line_length,
            lidar_heading_rebase_error, lidar_heading_max_jump,
            lidar_heading_disable_clearance,
            guide_center_kp, guide_center_tolerance,
            guide_center_max_heading, guide_center_filter_coef,
            guide_center_max_jump, guide_center_disable_clearance,
            wheel_yaw_max_age, imu_max_age, imu_stationary_yaw_rate,
            center, sector_width, safety_width, rear_half_width,
            safety_margin, success_span, control_rate, motion_timeout,
            odom_max_age, blocked_timeout,
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
                or heading_kp <= 0.0
                or heading_kd < 0.0
                or abs(reverse_angular_command_sign) != 1.0
                or heading_tolerance < 0.0
                or heading_max_speed <= 0.0
                or heading_max_accel <= 0.0
                or heading_pause_error <= heading_tolerance
                or heading_pause_error >= lidar_heading_max_error
                or not 0.0 < lidar_heading_filter_coef <= 1.0
                or not 0.0 < lidar_heading_max_error < math.pi / 2.0
                or not 0.0 < lidar_heading_min_inlier_ratio <= 1.0
                or lidar_heading_min_line_length <= 0.0
                or not 0.0 < lidar_heading_max_jump < math.pi / 2.0
                or lidar_heading_disable_clearance <= target + tolerance
                or lidar_heading_disable_clearance > slowdown
                or not 0.0 < lidar_heading_rebase_error <= (
                    lidar_heading_max_error)
                or guide_center_kp <= 0.0
                or guide_center_tolerance < 0.0
                or guide_center_max_heading <= 0.0
                or not 0.0 < guide_center_filter_coef <= 1.0
                or guide_center_max_jump <= 0.0
                or guide_center_disable_clearance <= target + tolerance
                or guide_center_disable_clearance > slowdown
                or wheel_yaw_max_age <= 0.0
                or imu_max_age <= 0.0
                or imu_stationary_yaw_rate < 0.0
                or not 0.0 < sector_width <= 2.0 * math.pi
                or not 0.0 < safety_width <= 2.0 * math.pi
                or safety_width < sector_width
                or rear_half_width <= 0.0
                or safety_margin < 0.0
                or stable_cycles < 1
                or heading_resume_stable_cycles < 1
                or lidar_heading_stable_cycles < 1
                or guide_center_stable_cycles < 1
                or success_points < 1
                or not 0.0 < success_span <= sector_width
                or control_rate <= 0.0
                or motion_timeout <= 0.0
                or odom_max_age <= 0.0
                or blocked_timeout <= 0.0):
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

    def wheel_yaw_is_fresh(self) -> bool:
        max_age = max(float(
            self.node.get_parameter('wheel_yaw_max_age_sec').value), 0.0)
        return (
            self.last_wheel_yaw is not None
            and self.last_wheel_yaw_received_at > 0.0
            and time.monotonic() - self.last_wheel_yaw_received_at <= max_age
        )

    def imu_is_fresh(self) -> bool:
        max_age = max(float(
            self.node.get_parameter('imu_max_age_sec').value), 0.0)
        return (
            self.last_imu_yaw_rate is not None
            and self.last_imu_received_at > 0.0
            and time.monotonic() - self.last_imu_received_at <= max_age
        )

    def current_imu_yaw_rate(self) -> float:
        assert self.last_imu_yaw_rate is not None
        return self.last_imu_yaw_rate

    def current_integrated_imu_yaw(self) -> float:
        return self.integrated_imu_yaw

    def imu_age_sec(self) -> float:
        if (
                self.last_imu_yaw_rate is None
                or self.last_imu_received_at <= 0.0):
            return math.inf
        return max(time.monotonic() - self.last_imu_received_at, 0.0)

    def current_odom_yaw_rate(self) -> float:
        assert self.last_odom is not None
        yaw_rate = float(self.last_odom.twist.twist.angular.z)
        return yaw_rate if math.isfinite(yaw_rate) else math.inf

    def current_stationary_yaw_rate(self) -> tuple[float, str]:
        if self.imu_is_fresh():
            return self.current_imu_yaw_rate(), 'imu'
        # wheel_odometry publishes encoder-derived angular velocity in odom,
        # independent of its slow absolute-IMU pose correction.  It is a safe
        # settling fallback when the raw IMU subscriber has a short dropout.
        return self.current_odom_yaw_rate(), 'wheel_odom'

    def set_backup_lidar_heading_estimator(
            self,
            estimator: Callable[
                [ScanSnapshot],
                tuple[float, float, int, int, float] | None,
            ]) -> None:
        self.backup_lidar_heading_estimator = estimator

    def set_backup_lidar_guide_estimator(
            self,
            estimator: Callable[
                [ScanSnapshot], GuideCenterEstimate | None,
            ]) -> None:
        self.backup_lidar_guide_estimator = estimator

    def current_wheel_yaw(self) -> float:
        assert self.last_wheel_yaw is not None
        return self.last_wheel_yaw

    @staticmethod
    def _backup_speed_for_clearance(
            clearance: float, target: float, slowdown: float,
            min_speed: float, max_speed: float) -> float:
        if slowdown <= target or clearance >= slowdown:
            return max_speed
        ratio = max(min((clearance - target) / (slowdown - target), 1.0), 0.0)
        return min_speed + ratio * (max_speed - min_speed)

    @staticmethod
    def _backup_heading_angular_velocity(
            heading_error: float, imu_yaw_rate: float, tolerance: float,
            kp: float, kd: float, max_speed: float,
            command_sign: float) -> float:
        proportional = 0.0 if abs(heading_error) <= tolerance else (
            kp * heading_error)
        # ROS angular.z and the measured motor response retain their sign
        # while linear.x is negative. command_sign remains configurable for
        # unusual downstream velocity converters.
        correction = command_sign * (proportional - kd * imu_yaw_rate)
        return min(max(correction, -max_speed), max_speed)

    @staticmethod
    def _filter_lidar_heading_sample(
            current: float | None, candidate: float,
            filter_coefficient: float,
            max_jump: float) -> tuple[float | None, bool]:
        if current is None:
            return candidate, True
        residual = normalize_angle(candidate - current)
        if abs(residual) > abs(max_jump):
            return current, False
        coefficient = min(max(filter_coefficient, 0.0), 1.0)
        return normalize_angle(current + coefficient * residual), True

    @staticmethod
    def _backup_guide_center_heading(
            center_offset: float, tolerance: float,
            kp: float, max_heading: float) -> float:
        if abs(center_offset) <= tolerance:
            return 0.0
        requested = -kp * center_offset
        return min(max(requested, -max_heading), max_heading)

    @staticmethod
    def _backup_dead_reckoning_drift_exceeded(
            heading_source: str, heading_drift: float,
            max_yaw_drift: float) -> bool:
        return (
            heading_source != 'lidar'
            and abs(heading_drift) >= abs(max_yaw_drift)
        )

    @staticmethod
    def _limit_command_rate(
            current: float, requested: float, max_change: float) -> float:
        max_change = max(max_change, 0.0)
        return min(max(requested, current - max_change), current + max_change)

    @staticmethod
    def _update_backup_heading_correction(
            active: bool, settled_count: int, lidar_available: bool,
            heading_error: float, imu_yaw_rate: float, tolerance: float,
            pause_error: float, stationary_yaw_rate: float,
            stable_cycles: int) -> tuple[bool, int]:
        if not active:
            if lidar_available and abs(heading_error) >= pause_error:
                # Do not carve a curved path into the station. Pause
                # translation and correct slowly in place first.
                return True, 0
            return False, 0

        settled = (
            lidar_available
            and abs(heading_error) <= tolerance
            and abs(imu_yaw_rate) <= stationary_yaw_rate
        )
        settled_count = settled_count + 1 if settled else 0
        if settled_count >= max(stable_cycles, 1):
            return False, 0
        return True, settled_count

    @staticmethod
    def _format_optional_degrees(value: float | None) -> str:
        if value is None:
            return 'unavailable'
        return f'{math.degrees(value):.2f}deg'

    @staticmethod
    def _format_optional_meters(value: float | None) -> str:
        if value is None:
            return 'unavailable'
        return f'{value:+.3f}m'

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
