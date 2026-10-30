import math
import statistics

from docking.control import (
    BackupStationaryYawController, spin_for, straight_entry_heading_budget,
    remaining_straight_heading_budget,
    straight_entry_path,
)
import time
from typing import Any, Callable

from docking.docking_lidar import DockingLidar, ScanSnapshot
from docking.lidar_geometry import (
    effective_range_limits,
    GuideCenterEstimate,
    header_stamp_is_acceptable,
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
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Imu
from tf2_ros import TransformException


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
        node.declare_parameter('spin_use_lidar_heading', False)
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
        node.declare_parameter('backup_guide_known_width', 0.0)
        node.declare_parameter('backup_guide_known_width_uncertainty', .01)
        # base_scan2 is 0.0635 m ahead of the body rear reference.  These
        # defaults therefore represent a measured LiDAR-to-wall target of
        # 0.078 +/- 0.010 m at the successful fully inserted dock position.
        node.declare_parameter('backup_target_rear_clearance', 0.0145)
        node.declare_parameter('backup_clearance_tolerance', 0.010)
        node.declare_parameter('backup_rear_reference_x', -0.2295)
        node.declare_parameter('backup_lidar_min_range', 0.05)
        node.declare_parameter('backup_lidar_max_range', 2.0)
        node.declare_parameter('backup_lidar_success_min_points', 5)
        node.declare_parameter(
            'backup_lidar_success_min_angle_span', math.radians(3.0))
        node.declare_parameter('backup_lidar_stable_cycles', 3)
        node.declare_parameter('backup_max_travel', 0.60)
        node.declare_parameter('backup_max_yaw_drift', math.radians(8.0))
        # The stationary LiDAR step establishes dock heading.  The reverse
        # controller must still be strong enough to cancel drivetrain skew.
        node.declare_parameter('backup_heading_kp', 0.35)
        node.declare_parameter('backup_heading_kd', 0.0)
        # ROS angular.z keeps the same sign while reversing.  The measured
        # response from the failed run also followed the commanded sign.
        node.declare_parameter('backup_reverse_angular_command_sign', 1.0)
        node.declare_parameter(
            'backup_heading_tolerance', math.radians(1.0))
        node.declare_parameter('backup_heading_max_angular_speed', 0.015)
        node.declare_parameter('backup_heading_max_angular_accel', 0.030)
        node.declare_parameter('backup_stationary_heading_max_angular_speed', 0.06)
        node.declare_parameter('backup_stationary_heading_min_angular_speed', 0.025)
        node.declare_parameter(
            'backup_heading_pause_error', math.radians(3.0))
        node.declare_parameter('backup_heading_resume_stable_cycles', 3)
        node.declare_parameter('use_lidar_heading_during_backup', True)
        node.declare_parameter('backup_lidar_heading_filter_coef', 0.15)
        # This bounds a correctable measured panel angle, not permitted
        # reverse tilt. Errors above 3 degrees already stop translation and
        # use the bounded stationary controller before reverse can resume.
        node.declare_parameter(
            'backup_lidar_heading_max_error', math.radians(12.0))
        node.declare_parameter(
            'backup_lidar_motion_residual', math.radians(2.0))
        # A small translation-dependent RANSAC bias may disagree with both
        # yaw groups even while encoder-only yaw confirms a straight chassis.
        # After stable scans, hold wheel heading instead of steering from that
        # suspect plane. Larger disagreement remains fail-closed.
        node.declare_parameter(
            'backup_lidar_wheel_guard_residual', math.radians(4.0))
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
        # Proactive straight entry only after five verified rail/panel samples.
        node.declare_parameter('backup_guide_visibility_handoff_clearance', 0.0)
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
        node.declare_parameter('backup_plane_reacquire_timeout_sec', 2.0)
        node.declare_parameter('backup_motion_feedback_recovery_timeout_sec', 2.0)
        node.declare_parameter('backup_motion_feedback_recovery_total_sec', 4.0)
        node.declare_parameter('control_rate_hz', 20.0)
        node.declare_parameter('motion_timeout_sec', 45.0)
        node.declare_parameter('server_wait_timeout_sec', 10.0)
        node.declare_parameter('odom_max_age_sec', 0.50)
        node.declare_parameter('motion_sensor_future_tolerance_sec', 0.05)

    def __init__(
            self, node: Node, lidar: DockingLidar,
            should_stop: Callable[[], bool] | None = None) -> None:
        self.node = node
        self.lidar = lidar
        self.should_stop = should_stop or (lambda: False)
        self.cmd_vel_pub = node.create_publisher(
            Twist, node.get_parameter('cmd_vel_topic').value, 10)
        # These are absolute pose/encoder state snapshots, not increments.
        # Keeping a reliable history can replay old state after a delivery
        # delay. Prefer the latest sample, just as the docking LiDAR does;
        # unchanged header/receipt freshness checks still fail closed on loss.
        latest_state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE)
        self.odom_sub = node.create_subscription(
            Odometry, node.get_parameter('odom_topic').value,
            self._odom_callback, latest_state_qos)
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
            latest_state_qos,
        )
        self.last_odom: Odometry | None = None
        self.last_odom_received_at = 0.0
        self.last_odom_stamp_nanoseconds = 0
        self.odom_sequence = 0
        self.last_odom_error = 'no odometry received'
        self.last_imu_yaw_rate: float | None = None
        self.last_imu_received_at = 0.0
        self.last_imu_stamp_nanoseconds = 0
        self.integrated_imu_yaw = 0.0
        self.last_wheel_yaw: float | None = None
        self.last_wheel_yaw_received_at = 0.0
        self.last_wheel_yaw_stamp_nanoseconds = 0
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
                self.node.get_logger().error(
                    f'odom is not available: {self.last_odom_error}')
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
        nominal_target = target_yaw
        panel_targets = []
        panel_sequence = -1
        panel_locked = False
        use_panel = bool(self.node.get_parameter('spin_use_lidar_heading').value)
        done = self._make_absolute_spin_done_cb(
            target_yaw, tolerance, stable_cycles, direction, stationary_yaw_rate)

        self.node.get_logger().info(
            'Spinning to an absolute odom yaw target: '
            f'start={math.degrees(start_yaw):.2f}deg, '
            f'target={math.degrees(target_yaw):.2f}deg, '
            f'delta={math.degrees(requested_yaw):.2f}deg, '
            f'max_speed={max_speed:.3f}rad/s')

        last_stationary_source = 'imu'

        def spin_done() -> bool:
            nonlocal target_yaw, panel_sequence, panel_locked, done
            # Finish the same turn against the physical rear panel. Only a
            # bounded, once-only update near the nominal half-turn is allowed.
            # Ordinary spin timing, speed and stationary checks remain below.
            if (use_panel and not panel_locked
                    and abs(self._absolute_yaw_error(
                        nominal_target, self.current_yaw(), direction)) <= math.radians(12)):
                snapshot = self.lidar.snapshot()
                if snapshot is not None and snapshot.sequence != panel_sequence:
                    panel_sequence = snapshot.sequence
                    candidate = self._spin_panel_target(snapshot, nominal_target)
                    if candidate is None:
                        panel_targets.clear()
                    else:
                        panel_targets.append(self.normalize_angle(candidate-nominal_target))
                        panel_targets[:] = panel_targets[-3:]
                        if (len(panel_targets) == 3
                                and max(panel_targets)-min(panel_targets) <= math.radians(1)):
                            adjustment = statistics.median(panel_targets)
                            target_yaw = self.normalize_angle(nominal_target+adjustment)
                            panel_locked = True
                            done = self._make_absolute_spin_done_cb(
                                target_yaw, tolerance, stable_cycles, direction, stationary_yaw_rate)
                            self.node.get_logger().info(
                                'Spin target confirmed by three rear panel scans: '
                                f'adjustment={math.degrees(adjustment):+.2f}deg, '
                                f'target={math.degrees(target_yaw):+.2f}deg')
            return done()

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
            done_cb=spin_done,
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
                f'target_source={"rear_panel" if panel_locked else "odom"}, '
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

    def _spin_panel_target(self, snapshot, nominal_target):
        """Return an acquisition-time-corrected, strongly supported plane yaw."""
        if (self.backup_lidar_heading_estimator is None
                or not self.imu_is_fresh()):
            return None
        estimate = self.backup_lidar_heading_estimator(snapshot)
        if estimate is None:
            return None
        error, _, inliers, points, length = estimate
        if (not all(math.isfinite(v) for v in (error, inliers, points, length))
                or abs(error) > math.radians(15)
                or inliers < 12 or points < 12 or inliers > points
                or inliers/points < .70 or length < .25):
            return None
        try:
            transform = self.node.tf_buffer.lookup_transform(
                str(self.node.get_parameter('fixed_frame').value),
                str(self.node.get_parameter('base_frame').value),
                Time.from_msg(snapshot.scan.header.stamp)).transform
        except (TransformException, AttributeError):
            return None
        q = transform.rotation
        norm = math.hypot(q.x, q.y, q.z, q.w)
        if not math.isfinite(norm) or norm < 1e-6:
            return None
        x, y, z, w = (v/norm for v in (q.x, q.y, q.z, q.w))
        scan_yaw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
        candidate = self.normalize_angle(scan_yaw+error)
        if abs(self.normalize_angle(candidate-nominal_target)) > math.radians(8):
            return None
        return candidate

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
        lidar_motion_residual_limit = abs(float(self.node.get_parameter(
            'backup_lidar_motion_residual').value))
        lidar_wheel_guard_residual_limit = abs(float(
            self.node.get_parameter(
                'backup_lidar_wheel_guard_residual').value))
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
        guide_body_half_width = float(self.node.get_parameter('backup_rear_half_width').value)
        guide_side_margin = float(self.node.get_parameter('backup_rear_safety_margin').value)
        handoff_clearance = float(self.node.get_parameter(
            'backup_guide_visibility_handoff_clearance').value)
        known_guide_width = float(self.node.get_parameter('backup_guide_known_width').value)
        known_width_uncertainty = float(self.node.get_parameter(
            'backup_guide_known_width_uncertainty').value)
        if (not all(math.isfinite(v) for v in (known_guide_width, known_width_uncertainty))
                or (known_guide_width != 0 and not .50 <= known_guide_width <= .58)
                or not .005 <= known_width_uncertainty <= .02):
            self.stop_robot()
            self.node.get_logger().error('Invalid measured guide width profile')
            return False
        # A measured fixed station can verify a longer straight corridor with
        # the same full-body/remaining-distance budget. Do this before the
        # near-field aluminum fits drift or leave the LiDAR field of view.
        max_handoff_clearance = .50 if known_guide_width else .32
        if (not math.isfinite(handoff_clearance)
                or not 0 <= handoff_clearance <= max_handoff_clearance):
            self.stop_robot()
            self.node.get_logger().error(
                'Invalid guide visibility handoff clearance: '
                f'maximum={max_handoff_clearance:.2f}m, '
                f'known_width={known_guide_width:.3f}m')
            return False
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
        filtered_anchored_heading_error: float | None = None
        lidar_heading_error: float | None = None
        lidar_heading_anchor_error: float | None = None
        lidar_heading_anchor_imu = start_integrated_imu_yaw
        lidar_heading_anchor_odom = start_odom_yaw
        lidar_heading_anchor_wheel = start_wheel_yaw
        lidar_heading_stable_count = 0
        wheel_guard_filtered_error: float | None = None
        wheel_guard_stable_count = 0
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
        stationary_heading_controller = None
        last_heading_command_at = time.monotonic()
        blocked_since: float | None = None
        guide_missing_since: float | None = None
        guide_clearance_diagnostic: str | None = None
        guide_entry_verified = False
        straight_entry_observations = []
        guide_handoff = None
        handoff_path = None
        handoff_progress = 0.0
        handoff_lateral_spent = 0.0
        handoff_previous_xy = None
        handoff_previous_heading = 0.0
        handoff_station_yaw = 0.0
        handoff_last_clearance = 0.0
        plane_missing_since: float | None = None
        stationary_recovery_errors: list[float] = []
        stationary_recovery_wheel: float | None = None
        stationary_recovery_used = False
        reacquire_timeout = float(self.node.get_parameter(
            'backup_plane_reacquire_timeout_sec').value)
        if not math.isfinite(reacquire_timeout) or reacquire_timeout <= 0.0:
            self.stop_robot()
            self.node.get_logger().error('Invalid backup plane reacquisition timeout')
            return False
        last_log_time = 0.0
        feedback_recovery_timeout = float(self.node.get_parameter(
            'backup_motion_feedback_recovery_timeout_sec').value)
        feedback_recovery_total = float(self.node.get_parameter(
            'backup_motion_feedback_recovery_total_sec').value)
        if not (math.isfinite(feedback_recovery_timeout)
                and math.isfinite(feedback_recovery_total)
                and 0 < feedback_recovery_timeout <= 3.
                and feedback_recovery_timeout <= feedback_recovery_total <= 6.):
            self.stop_robot()
            self.node.get_logger().error('Invalid backup motion feedback recovery budgets')
            return False
        feedback_recovery_used = 0.0

        transform = getattr(self.lidar, 'transform', None)
        lidar_target_log = ''
        if transform is not None:
            rear_reference_x = float(
                self.node.get_parameter('backup_rear_reference_x').value)
            lidar_target_range = (
                transform.x - rear_reference_x + target_clearance)
            lidar_target_log = (
                f'lidar_target={lidar_target_range:.4f}'
                f'+/-{tolerance:.4f}m, ')
        self.node.get_logger().info(
            'Backing up using LiDAR rear clearance: '
            f'body_target={target_clearance:.4f}+/-{tolerance:.4f}m, '
            f'{lidar_target_log}'
            f'speed={speed:.3f}m/s, '
            f'completion_sector=rear+/-{completion_half_angle:.1f}deg, '
            f'safety_sector=rear+/-{safety_half_angle:.1f}deg')

        while rclpy.ok() and not self.should_stop():
            rclpy.spin_once(self.node, timeout_sec=0.0)

            elapsed = (self.node.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout_sec:
                self.stop_robot()
                self.node.get_logger().error('LiDAR backup step timed out')
                return False

            odom_fresh, wheel_fresh = self.odom_is_fresh(), self.wheel_yaw_is_fresh()
            if (guide_handoff is not None and (not odom_fresh or not wheel_fresh)
                    and self.imu_is_fresh()):
                # A verified straight corridor does not become invalid just
                # because encoder delivery has a brief gap. Stop immediately;
                # never continue on stale motion or relatch the heading anchor.
                recovery_budget = min(feedback_recovery_timeout,
                                      feedback_recovery_total-feedback_recovery_used,
                                      timeout_sec-elapsed)
                recovery_started = time.monotonic()
                current_command = Twist()
                self.stop_robot()
                self.node.get_logger().warning(
                    'Verified straight backup paused for fresh motion feedback: '
                    f'odom_fresh={odom_fresh}, wheel_fresh={wheel_fresh}, '
                    f'wheel_header_age={(self.node.get_clock().now().nanoseconds-self.last_wheel_yaw_stamp_nanoseconds)/1e9:.3f}s, '
                    f'recovery_budget={recovery_budget:.3f}s')
                if not self._wait_for_backup_motion_feedback(
                        recovery_started+max(0., recovery_budget)):
                    self.stop_robot()
                    self.node.get_logger().error(
                        'Verified straight backup motion feedback did not recover while stopped; '
                        'original heading/travel/time guards retained')
                    return False
                feedback_recovery_used += time.monotonic()-recovery_started
                stability.reset()
                self.node.get_logger().info(
                    'Verified straight backup motion feedback recovered while stopped: '
                    f'waited={time.monotonic()-recovery_started:.3f}s, '
                    f'total_wait={feedback_recovery_used:.3f}s; original corridor anchor retained')
                continue
            if not odom_fresh:
                self.stop_robot()
                self.node.get_logger().error(
                    'Odometry became stale during LiDAR backup')
                return False
            if not wheel_fresh:
                self.stop_robot()
                self.node.get_logger().error(
                    'Wheel-only yaw diagnostics became stale during LiDAR backup')
                return False
            current_x, current_y = self.current_xy()
            traveled = math.hypot(current_x - start_x, current_y - start_y)
            if guide_handoff is not None:
                anchor_travel, anchor_gyro, anchor_heading, heading_budget, remaining = guide_handoff
                held_heading = anchor_heading - (
                    self.current_integrated_imu_yaw() - anchor_gyro)
                step = math.hypot(current_x-handoff_previous_xy[0],
                                  current_y-handoff_previous_xy[1])
                # A signed path already includes nominal sideways travel from
                # its initial tilt. Spend only departures from that fixed ray;
                # odom and gyro independently bound the additional departure.
                tracking_yaw = handoff_station_yaw
                previous, current = handoff_previous_heading, held_heading
                if handoff_path is not None:
                    tracking_yaw -= anchor_heading
                    previous -= anchor_heading
                    current -= anchor_heading
                    handoff_progress += step
                side_step = abs(
                    -(current_x-handoff_previous_xy[0])*math.sin(tracking_yaw)
                    +(current_y-handoff_previous_xy[1])*math.cos(tracking_yaw))
                handoff_lateral_spent += max(side_step, step * max(
                    abs(math.sin(previous)), abs(math.sin(current))))
                if handoff_path is not None:
                    # Arc length overstates progress along the initial ray;
                    # charge that difference as additional position uncertainty.
                    handoff_lateral_spent += step * max(
                        0., 1-min(math.cos(previous), math.cos(current)))
                handoff_previous_xy = (current_x, current_y)
                handoff_previous_heading = held_heading
                # The latest rear scan may be slightly older than odometry.
                # Use the larger remaining distance, retaining its conservative
                # bound even between scans and across stopped sensor recovery.
                live_remaining = max(handoff_last_clearance,
                                     remaining-max(0., traveled-anchor_travel))
                path_clearance = None
                if handoff_path is not None:
                    path_clearance = handoff_path.clearance(
                        held_heading, handoff_progress, live_remaining,
                        handoff_lateral_spent)
                elif known_guide_width:
                    heading_budget = remaining_straight_heading_budget(
                        heading_budget, remaining, live_remaining,
                        handoff_lateral_spent, heading_pause_error)
                handoff_imu_fresh = self.imu_is_fresh()
                if (not handoff_imu_fresh
                        or heading_budget <= 0
                        or (path_clearance is not None
                            and path_clearance < guide_side_margin)
                        or abs(held_heading) > heading_budget
                        or traveled - anchor_travel > remaining + .02):
                    self.stop_robot()
                    self.node.get_logger().error(
                        'Guide visibility handoff lost its bounded straight heading/travel: '
                        f'heading={math.degrees(held_heading):+.3f}deg, '
                        f'budget={math.degrees(heading_budget):.3f}deg, '
                        f'anchor_heading={math.degrees(anchor_heading):+.3f}deg, '
                        f'lateral_spent={handoff_lateral_spent:.4f}m, '
                        f'path_side_clearance={self._format_optional_meters(path_clearance)}, '
                        f'remaining={live_remaining:.3f}m, '
                        f'imu_fresh={handoff_imu_fresh}, '
                        f'travel={traveled-anchor_travel:.3f}m, '
                        f'travel_limit={remaining+.02:.3f}m')
                    return False
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
                stationary_recovery_errors = []
                stationary_recovery_wheel = None
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
                spin_for(self.node, sleep_time, self.should_stop)
                continue

            if snapshot.sequence == stability.last_sequence:
                self.cmd_vel_pub.publish(current_command)
                spin_for(self.node, sleep_time, self.should_stop)
                continue

            indexed_clearances = self._rear_clearances(snapshot)
            if not indexed_clearances:
                stationary_recovery_errors = []
                stationary_recovery_wheel = None
                stability.observe(snapshot.sequence, False)
                current_command = Twist()
                self.cmd_vel_pub.publish(current_command)
                spin_for(self.node, sleep_time, self.should_stop)
                continue

            clearance = min(value for _, value in indexed_clearances)
            handoff_last_clearance = clearance
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
            # A nearest return entering the *upper* tolerance is not enough
            # to stop permanently: the supported panel can still be farther
            # away. Stop on a real cluster, or at the nominal protective gap.
            # Keep the lower tolerance as an independent overrun guard.
            completion_cluster = (
                completion_cluster and protective_clearance >= completion_minimum)
            complete = stability.observe(
                snapshot.sequence,
                completion_cluster and self.is_stationary(0.005)
                and abs(self.current_odom_yaw_rate()) <= stationary_yaw_rate)
            if complete:
                self.stop_robot()
                self.node.get_logger().info(
                    'Dock-turn-backup sequence complete: '
                    f'rear_clearance={clearance:.3f}m, '
                    f'unique_scans={stability.count}')
                return True
            if completion_cluster:
                current_command = Twist()
                self.cmd_vel_pub.publish(current_command)
                spin_for(self.node, sleep_time, self.should_stop)
                continue

            lidar_heading_error = None
            lidar_heading_control_available = False
            imu_motion_residual: float | None = None
            odom_motion_residual: float | None = None
            wheel_motion_residual: float | None = None
            wheel_guard_active = False
            heading_source = (
                'imu_gyro' if self.imu_is_fresh() else 'wheel_odom')
            if (
                    use_lidar_heading
                    and guide_handoff is None
                    and protective_clearance
                    > lidar_heading_disable_clearance
                    and self.backup_lidar_heading_estimator is not None):
                estimate = self.backup_lidar_heading_estimator(snapshot)
                if estimate is not None:
                    (candidate_error, _, inliers, total_points,
                     line_length) = estimate
                    inlier_ratio = inliers / max(total_points, 1)
                    heading_in_range = (
                        math.isfinite(candidate_error)
                        and abs(candidate_error) <= lidar_heading_max_error)
                    quality_valid = (
                            heading_in_range
                            and inlier_ratio >= lidar_heading_min_inlier_ratio
                            and line_length >= lidar_heading_min_line_length)
                    motion_consistent = True
                    if quality_valid and lidar_heading_anchor_error is not None:
                        imu_motion_residual = self._lidar_heading_motion_residual(
                            candidate_error,
                            lidar_heading_anchor_error,
                            self.current_integrated_imu_yaw(),
                            lidar_heading_anchor_imu,
                        )
                        odom_motion_residual = self._lidar_heading_motion_residual(
                            candidate_error,
                            lidar_heading_anchor_error,
                            self.current_yaw(),
                            lidar_heading_anchor_odom,
                        )
                        wheel_motion_residual = (
                            self._lidar_heading_motion_residual(
                                candidate_error,
                                lidar_heading_anchor_error,
                                self.current_wheel_yaw(),
                                lidar_heading_anchor_wheel,
                            ))
                        motion_consistent = (
                            self._lidar_heading_motion_is_consistent(
                                imu_motion_residual,
                                odom_motion_residual,
                                wheel_motion_residual,
                                lidar_motion_residual_limit,
                            )
                        )
                    # A fixed start anchor cannot recover from a changed
                    # visible panel after translation. Permit ONE new anchor
                    # only after stopping and observing five high-support,
                    # tightly agreeing scans. Never rebase while driving.
                    recovery_candidate = (
                        quality_valid and not motion_consistent
                        and not stationary_recovery_used
                        and plane_missing_since is not None
                        and current_command.linear.x == 0.0
                        and current_command.angular.z == 0.0
                        and inlier_ratio >= max(.75, lidar_heading_min_inlier_ratio)
                        and line_length >= max(.30, lidar_heading_min_line_length)
                        and abs(self.current_stationary_yaw_rate()[0])
                        <= stationary_yaw_rate
                        and abs(self.current_odom_yaw_rate()) <= stationary_yaw_rate
                        and abs(wheel_heading_drift) < max_yaw_drift)
                    if recovery_candidate:
                        if (stationary_recovery_wheel is None
                                or abs(self.normalize_angle(
                                    self.current_wheel_yaw()
                                    - stationary_recovery_wheel)) > math.radians(.5)
                                or (stationary_recovery_errors and (
                                    max(stationary_recovery_errors + [candidate_error])
                                    - min(stationary_recovery_errors + [candidate_error]))
                                    > math.radians(1.0))):
                            stationary_recovery_errors = []
                            stationary_recovery_wheel = self.current_wheel_yaw()
                        stationary_recovery_errors.append(candidate_error)
                        if (len(stationary_recovery_errors) >= 5
                                and time.monotonic() - plane_missing_since >= .4):
                            stationary_recovery_used = True
                            lidar_heading_anchor_error = candidate_error
                            lidar_heading_anchor_imu = self.current_integrated_imu_yaw()
                            lidar_heading_anchor_odom = self.current_yaw()
                            lidar_heading_anchor_wheel = self.current_wheel_yaw()
                            imu_heading_reference = self.current_integrated_imu_yaw()
                            filtered_lidar_heading_error = candidate_error
                            filtered_anchored_heading_error = candidate_error
                            lidar_heading_stable_count = 0
                            heading_correction_active = True
                            heading_settled_count = 0
                            motion_consistent = True
                            self.node.get_logger().warning(
                                'Stationary rear plane recovered; aligning in place '
                                'before resuming backup: '
                                f'error={math.degrees(candidate_error):+.2f}deg, '
                                f'inliers={inliers}/{total_points}, '
                                f'length={line_length:.2f}m, scans='
                                f'{len(stationary_recovery_errors)}')
                    else:
                        stationary_recovery_errors = []
                        stationary_recovery_wheel = None
                    if quality_valid and motion_consistent:
                        wheel_guard_filtered_error = None
                        wheel_guard_stable_count = 0
                        lidar_heading_error = candidate_error
                        previous_filtered_heading_error = filtered_lidar_heading_error
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
                            if lidar_heading_anchor_error is None:
                                filtered_anchored_heading_error = next_filtered_error
                            else:
                                anchored_error = self.normalize_angle(
                                    lidar_heading_anchor_error
                                    - (self.current_integrated_imu_yaw()
                                       - lidar_heading_anchor_imu))
                                if (filtered_anchored_heading_error is None
                                        or previous_filtered_heading_error is None):
                                    filtered_anchored_heading_error = anchored_error
                                else:
                                    filtered_anchored_heading_error += (
                                        lidar_heading_filter_coef*self.normalize_angle(
                                            anchored_error-filtered_anchored_heading_error))
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
                            if lidar_heading_anchor_error is None:
                                # Freeze one stationary wall observation.  A
                                # wall fixed in the world obeys
                                # wall_error + robot_yaw = constant.  Keeping
                                # this anchor exposes a wrong RANSAC surface;
                                # rebasing it every scan hid 5.25 degrees of
                                # inertial/plane disagreement in the September
                                # 15 run.
                                lidar_heading_anchor_error = (
                                    filtered_lidar_heading_error)
                                lidar_heading_anchor_imu = (
                                    self.current_integrated_imu_yaw())
                                lidar_heading_anchor_odom = self.current_yaw()
                                lidar_heading_anchor_wheel = (
                                    self.current_wheel_yaw())
                    elif quality_valid:
                        filtered_lidar_heading_error = None
                        lidar_heading_stable_count = 0
                        wheel_guard_eligible = (
                            self.wheel_yaw_is_fresh()
                            and not heading_correction_active
                            and abs(candidate_error) < heading_pause_error
                            and wheel_motion_residual is not None
                            and self._backup_wheel_guard_is_allowed(
                                wheel_motion_residual,
                                wheel_heading_drift,
                                lidar_wheel_guard_residual_limit,
                                max_yaw_drift,
                            ))
                        if wheel_guard_eligible:
                            (
                                next_wheel_guard_error,
                                wheel_guard_sample_accepted,
                            ) = self._filter_lidar_heading_sample(
                                wheel_guard_filtered_error,
                                candidate_error,
                                lidar_heading_filter_coef,
                                lidar_heading_max_jump,
                            )
                            if wheel_guard_sample_accepted:
                                wheel_guard_filtered_error = (
                                    next_wheel_guard_error)
                                wheel_guard_stable_count += 1
                            else:
                                wheel_guard_filtered_error = candidate_error
                                wheel_guard_stable_count = 1
                            wheel_guard_active = (
                                wheel_guard_stable_count
                                >= lidar_heading_stable_cycles)
                            lidar_heading_error = candidate_error
                            lidar_heading_status = (
                                'wheel_guard'
                                if wheel_guard_active
                                else 'wheel_guard_warming_up')
                            if wheel_guard_active:
                                self.node.get_logger().warning(
                                    'LiDAR plane has a bounded motion residual; '
                                    'continuing with encoder-only heading guard: '
                                    f'candidate={math.degrees(candidate_error):+.2f}deg, '
                                    f'wheel_residual='
                                    f'{math.degrees(wheel_motion_residual):+.2f}deg, '
                                    f'wheel_drift='
                                    f'{math.degrees(wheel_heading_drift):+.2f}deg, '
                                    f'stable={wheel_guard_stable_count}/'
                                    f'{lidar_heading_stable_cycles}',
                                    throttle_duration_sec=1.0)
                        else:
                            wheel_guard_filtered_error = None
                            wheel_guard_stable_count = 0
                            lidar_heading_status = 'motion_inconsistent'
                            self.node.get_logger().warning(
                                'Rejected backup plane inconsistent with robot motion: '
                                f'candidate={math.degrees(candidate_error):+.2f}deg, '
                                f'imu_residual='
                                f'{math.degrees(imu_motion_residual):+.2f}deg, '
                                f'odom_residual='
                                f'{math.degrees(odom_motion_residual):+.2f}deg, '
                                f'wheel_residual='
                                f'{math.degrees(wheel_motion_residual):+.2f}deg',
                                throttle_duration_sec=1.0)
                    else:
                        filtered_lidar_heading_error = None
                        lidar_heading_stable_count = 0
                        wheel_guard_filtered_error = None
                        wheel_guard_stable_count = 0
                        lidar_heading_status = (
                            'quality_rejected' if heading_in_range
                            else 'heading_out_of_range')
                        self.node.get_logger().warning(
                            'Rejected backup plane: '
                            f'error={math.degrees(candidate_error):+.2f}deg, '
                            f'angle_limit={math.degrees(lidar_heading_max_error):.2f}deg, '
                            f'inliers={inliers}/{total_points}, length={line_length:.3f}m, '
                            f'reason={lidar_heading_status}',
                            throttle_duration_sec=1.0)
                else:
                    filtered_lidar_heading_error = None
                    lidar_heading_stable_count = 0
                    wheel_guard_filtered_error = None
                    wheel_guard_stable_count = 0
                    lidar_heading_status = 'ransac_unavailable'
                    stationary_recovery_errors = []
                    stationary_recovery_wheel = None
                if not lidar_heading_control_available:
                    heading_source = (
                        'wheel_guard'
                        if wheel_guard_active
                        else 'lidar_unavailable')
            elif use_lidar_heading:
                filtered_lidar_heading_error = None
                lidar_heading_stable_count = 0
                lidar_heading_status = (
                    'verified_straight_entry' if guide_handoff is not None
                    else ('disabled_near_dock' if protective_clearance
                          <= lidar_heading_disable_clearance
                          else 'estimator_unavailable'))
                heading_source = 'lidar_disabled'

            guide_center_offset = None
            guide_separation = None
            guide_side_clearance = None
            guide_entry_offset = None
            guide_center_heading = 0.0
            wall_control_heading_error = None
            wall_control_heading_source = 'unavailable'
            heading_pause_observed_error = None
            heading_pause_source = 'unavailable'
            if (
                    use_guide_centering
                    and guide_handoff is None
                    and self.backup_lidar_guide_estimator is not None):
                guide = self.backup_lidar_guide_estimator(snapshot)
                if guide is not None:
                    guide_center_offset = guide.center_offset
                    guide_separation = guide.separation
                    rear_offset = getattr(guide, 'rear_center_offset', None)
                    guide_entry_offset = (
                        rear_offset if known_guide_width and rear_offset is not None
                        else guide_center_offset)
                    if not math.isfinite(guide_entry_offset):
                        self.stop_robot()
                        self.node.get_logger().error('Invalid guide rear entry offset')
                        return False
                    supported_guide_width = (
                        known_guide_width-known_width_uncertainty
                        if known_guide_width else guide_separation)
                    guide_side_clearance = (supported_guide_width / 2.0
                                            - abs(guide_entry_offset)
                                            - guide_body_half_width)
                    if guide_side_clearance < guide_side_margin:
                        # Unsafe warm-up scans cannot count toward a later
                        # recovery just because their centers agree closely.
                        filtered_guide_center_offset = None
                        guide_stable_count = 0
                    elif (
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
                        # Reserve a quarter degree below the unchanged wall stop
                        # threshold. Unwind steering over the last 10 cm of
                        # the correction region before straight entry.
                        guide_center_heading = (
                            self._backup_guide_center_heading(
                                filtered_guide_center_offset,
                                guide_center_tolerance * 0.5,
                                guide_center_kp,
                                min(guide_center_max_heading,
                                    max(heading_pause_error - 0.25 * heading_tolerance, 0.0))
                                * min((protective_clearance
                                       - guide_center_disable_clearance) / 0.10, 1.0),
                                max(protective_clearance - guide_center_disable_clearance, 0.05),
                            ))
                else:
                    guide_stable_count = 0
                    filtered_guide_center_offset = None
            else:
                guide_stable_count = 0

            if use_guide_centering and known_guide_width and guide_handoff is None:
                # Warm-up samples already passed plane quality/motion checks.
                # Include them in the same five-scan corridor history instead
                # of discarding them and steering before collecting five more.
                if (guide_center_offset is not None
                        and lidar_heading_status in ('warming_up', 'tracking')
                        and self.imu_is_fresh() and not heading_correction_active):
                    straight_entry_observations.append((
                        guide_center_offset, guide_separation,
                        lidar_heading_error, self.current_integrated_imu_yaw()))
                    straight_entry_observations = straight_entry_observations[-5:]
                else:
                    straight_entry_observations = []

            stationary_yaw_rate_value, stationary_source = (
                self.current_stationary_yaw_rate())
            if heading_source == 'lidar_unavailable':
                # The old loop kept reversing with zero steering while the
                # plane was rejected. Stop before drifting out of the guide.
                current_command = Twist()
                self.cmd_vel_pub.publish(current_command)
                last_heading_command_at = time.monotonic()
                # Reacquisition is stationary, not dead reckoning. A biased
                # accumulated gyro must not bypass this bounded stop window.
                # Independent wheel rotation still enforces the safety limit.
                if abs(wheel_heading_drift) >= max_yaw_drift:
                    self.stop_robot()
                    self.node.get_logger().error(
                        'Wheel yaw drift exceeded the safety limit while '
                        'reacquiring the dock plane: '
                        f'{math.degrees(abs(wheel_heading_drift)):.2f}deg >= '
                        f'{math.degrees(max_yaw_drift):.2f}deg')
                    return False
                if plane_missing_since is None:
                    plane_missing_since = time.monotonic()
                    self.node.get_logger().warning(
                        'Backup paused to reacquire the dock plane: '
                        f'{lidar_heading_status}; '
                        f'gyro_drift={math.degrees(imu_heading_drift):+.2f}deg, '
                        f'wheel_drift={math.degrees(wheel_heading_drift):+.2f}deg')
                # A rejected jump must be allowed to establish a new filtered
                # value after the robot has stopped.  The fixed motion anchor
                # still prevents an unrelated surface from being accepted.
                if lidar_heading_status == 'jump_rejected':
                    filtered_lidar_heading_error = None
                    lidar_heading_stable_count = 0
                if time.monotonic() - plane_missing_since >= reacquire_timeout:
                    self.stop_robot()
                    self.node.get_logger().error(
                        'Dock plane did not recover while stopped; '
                        f'status={lidar_heading_status}, '
                        f'gyro_drift={math.degrees(imu_heading_drift):+.2f}deg, '
                        f'wheel_drift={math.degrees(wheel_heading_drift):+.2f}deg, '
                        'imu/odom/wheel_residual='
                        f'{self._format_optional_degrees(imu_motion_residual)}/'
                        f'{self._format_optional_degrees(odom_motion_residual)}/'
                        f'{self._format_optional_degrees(wheel_motion_residual)}')
                    return False
                spin_for(self.node, sleep_time, self.should_stop)
                continue
            # A validated dock plane is the physical heading authority and may
            # legitimately request several degrees of correction. Apply the
            # dead-reckoning drift limit only when that plane is unavailable.
            fallback_heading_drift = (
                wheel_heading_drift
                if heading_source == 'wheel_guard'
                else (
                    imu_heading_drift if self.imu_is_fresh()
                    else wheel_heading_drift))
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

            plane_missing_since = None
            if use_guide_centering and guide_handoff is None:
                if protective_clearance > guide_center_disable_clearance:
                    if guide_side_clearance is not None:
                        if guide_side_clearance < guide_side_margin:
                            # A short run of reflected rail fits is not a terminal
                            # failure. Recheck unique scans while stopped, within
                            # the existing guide acquisition budget. Never reverse
                            # or retain a straight-entry proof from the bad fit.
                            current_command = Twist()
                            self.cmd_vel_pub.publish(current_command)
                            last_heading_command_at = time.monotonic()
                            straight_entry_observations = []
                            guide_entry_verified = False
                            guide_stable_count = 0
                            filtered_guide_center_offset = None
                            diagnostic = (
                                f'nearest_side={guide_side_clearance:.3f}m, '
                                f'required_margin={guide_side_margin:.3f}m, '
                                f'center_offset={guide_center_offset:+.3f}m, '
                                f'rear_entry_offset={guide_entry_offset:+.3f}m, '
                                f'guide_width={guide_separation:.3f}m, '
                                f'supported_width={supported_guide_width:.3f}m')
                            guide_clearance_diagnostic = diagnostic
                            if guide_missing_since is None:
                                guide_missing_since = time.monotonic()
                                self.node.get_logger().warning(
                                    'Backup paused to recheck guide side clearance: '
                                    + diagnostic)
                            if time.monotonic() - guide_missing_since >= reacquire_timeout:
                                self.stop_robot()
                                self.node.get_logger().error(
                                    'Insufficient guide side clearance persists after '
                                    f'{reacquire_timeout:.2f}s stopped recheck: ' + diagnostic)
                                return False
                            spin_for(self.node, sleep_time, self.should_stop)
                            continue
                    guide_ready = (
                        lidar_heading_control_available
                        and guide_stable_count >= guide_center_stable_cycles)
                    if not guide_ready:
                        if not known_guide_width:
                            straight_entry_observations = []
                        current_command = Twist()
                        self.cmd_vel_pub.publish(current_command)
                        last_heading_command_at = time.monotonic()
                        if guide_missing_since is None:
                            guide_missing_since = time.monotonic()
                            self.node.get_logger().warning(
                                'Backup paused: waiting for two stable guide rails')
                        if time.monotonic() - guide_missing_since >= reacquire_timeout:
                            self.stop_robot()
                            if guide_clearance_diagnostic is not None:
                                self.node.get_logger().error(
                                    'Guide side clearance did not recover during '
                                    f'{reacquire_timeout:.2f}s stopped recheck: '
                                    + guide_clearance_diagnostic)
                            else:
                                self.node.get_logger().error(
                                    'Guide rails unavailable or inconsistent while stopped; '
                                    'refusing uncentered entry')
                            return False
                        spin_for(self.node, sleep_time, self.should_stop)
                        continue
                    guide_missing_since = None
                    guide_clearance_diagnostic = None
                    guide_entry_verified = (
                        abs(filtered_guide_center_offset) <= guide_center_tolerance
                        and abs(guide_center_offset) <= guide_center_tolerance
                        and abs(filtered_lidar_heading_error) <= heading_tolerance
                        and abs(lidar_heading_error) <= heading_tolerance)
                    # Record safe rail observations before aluminum returns
                    # disappear behind the scanner. At a measured station,
                    # accept by full-path clearance, not the steering deadband.
                    # The independent side-clearance check above still applies.
                    if ((known_guide_width or (
                            abs(filtered_guide_center_offset) <= guide_center_tolerance
                            and abs(guide_center_offset) <= guide_center_tolerance))
                            and self.imu_is_fresh() and not heading_correction_active):
                        corridor_gyro = self.current_integrated_imu_yaw()
                        if not known_guide_width:
                            straight_entry_observations.append((
                                guide_center_offset, guide_separation,
                                lidar_heading_error, corridor_gyro))
                            straight_entry_observations = straight_entry_observations[-5:]
                        corridor_observations = [
                            (center, width, self.normalize_angle(
                                angle-(corridor_gyro-sample_gyro)))
                            for center, width, angle, sample_gyro
                            in straight_entry_observations]
                        budget = straight_entry_heading_budget(
                            corridor_observations, clearance,
                            guide_center_tolerance, guide_body_half_width,
                            # At this measured station, straight-path safety
                            # comes from the full-body side margin. The 1 deg
                            # yaw convergence target is not that safety limit.
                            # Preserve the normal pause-error ceiling (3 deg)
                            # and the old tolerance cap at unmeasured stations.
                            guide_side_margin, (
                                heading_pause_error if known_guide_width
                                else heading_tolerance),
                            float(self.node.get_parameter('backup_rear_reference_x').value),
                            known_width=known_guide_width,
                            known_width_uncertainty=known_width_uncertainty)
                        candidate_path = None
                        # Use the same signed full-body corridor throughout a
                        # measured-width handoff. Initial symmetric heading
                        # headroom cannot predict which later IMU sample will
                        # exhaust that bound, even while the body still fits.
                        # Keep the symmetric fallback if this path is invalid.
                        if known_guide_width and clearance <= handoff_clearance:
                            candidate_path = straight_entry_path(
                                corridor_observations, clearance,
                                guide_body_half_width, guide_side_margin,
                                heading_pause_error,
                                float(self.node.get_parameter('backup_rear_reference_x').value),
                                known_guide_width, known_width_uncertainty,
                                lidar_heading_error)
                            if candidate_path is not None:
                                budget = heading_pause_error
                        if (budget is not None and clearance <= handoff_clearance
                                and not heading_correction_active):
                            handoff_path = candidate_path
                            guide_handoff = (
                                traveled, self.current_integrated_imu_yaw(),
                                lidar_heading_error, budget, clearance)
                            handoff_previous_xy = (current_x, current_y)
                            handoff_previous_heading = lidar_heading_error
                            handoff_station_yaw = self.current_yaw()+lidar_heading_error
                            handoff_last_clearance = clearance
                            guide_center_heading = 0.0
                            heading_correction_active = False
                            # Switch immediately without stopping/reloading the
                            # caster. The same loop still checks rear clearance.
                            lidar_heading_control_available = False
                            heading_source = 'lidar_disabled'
                            self.node.get_logger().info(
                                'Guide visibility handoff: five verified observations; '
                                f'remaining={clearance:.3f}m, '
                                f'center={guide_center_offset:+.3f}m, '
                                f'width={guide_separation:.3f}m, '
                                f'known_width={known_guide_width:.3f}m, '
                                f'heading={math.degrees(lidar_heading_error):+.3f}deg, '
                                f'gyro_heading_limit={math.degrees(budget):.2f}deg; '
                                f'corridor_check={"signed_rectangle" if handoff_path else "symmetric_bound"}; '
                                'continuous straight entry with rear distance and IMU guards')
                    else:
                        straight_entry_observations = []
                elif not guide_entry_verified:
                    self.stop_robot()
                    self.node.get_logger().error(
                        'Guide entry is not centered before the final straight backup: '
                        f'offset={self._format_optional_meters(filtered_guide_center_offset)}, '
                        f'wall_error={self._format_optional_degrees(filtered_lidar_heading_error)}; '
                        'reposition the robot before retrying')
                    return False

            if protective_clearance <= target_clearance:
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
                if use_guide_centering and filtered_guide_center_offset is not None:
                    # Small lateral corrections need not force the minimum
                    # speed for the whole approach. Scale only the excess
                    # offset, retaining full speed once the center settles.
                    fraction = min(max(
                        (abs(filtered_guide_center_offset) - guide_center_tolerance)
                        / max(guide_side_margin, guide_center_tolerance, 1e-6),
                        0.0), 1.0)
                    command_speed += (
                        min(command_speed, min_speed) - command_speed) * fraction
                wall_heading_error = 0.0
                pause_heading_error = None
                if lidar_heading_control_available:
                    wall_heading_error = filtered_lidar_heading_error or 0.0
                    wall_control_heading_source = 'live_lidar'
                    pause_heading_error = wall_heading_error
                    heading_pause_source = 'live_lidar'
                    if (use_guide_centering and known_guide_width
                            and not heading_correction_active
                            and lidar_heading_anchor_error is not None
                            and filtered_anchored_heading_error is not None
                            and imu_motion_residual is not None
                            and abs(imu_motion_residual) <= lidar_motion_residual_limit
                            and self.imu_is_fresh()):
                        # The stationary plane fixes dock direction; fresh gyro
                        # measures actual chassis rotation. Trial88's changing
                        # visible panel added ~0.6deg without that rotation,
                        # falsely triggering a harmful in-place correction.
                        # Live plane quality/motion/guide checks remain above.
                        # Only the in-place correction trigger uses this
                        # independent rotation estimate. Normal guide steering
                        # and corridor geometry still use the live plane.
                        pause_heading_error = filtered_anchored_heading_error
                        heading_pause_source = 'stationary_lidar_imu'
                    heading_error = wall_heading_error
                elif heading_source == 'wheel_guard':
                    # The RANSAC plane is stable but its translation-dependent
                    # bias conflicts with both yaw groups. Do not steer from
                    # it; preserve the encoder-confirmed straight heading.
                    heading_error = -wheel_heading_drift
                elif use_lidar_heading:
                    # Fail open-loop straight: never preserve a stale angular
                    # command when RANSAC is missing, rejected, or too close.
                    heading_error = 0.0
                else:
                    # Hold the last LiDAR-confirmed physical heading. Prefer
                    # gyro integration, then fall back to encoder-only yaw if
                    # raw IMU delivery is temporarily stale.
                    heading_error = -fallback_heading_drift
                # A requested lateral correction is not a measured chassis
                # tilt. Stopping for their sum repeatedly reverses caster
                # loading before a continuous reverse path can establish.
                # Keep the original wall-angle stop threshold unchanged.
                correction_error = heading_error
                if lidar_heading_control_available:
                    # Reduce lateral travel while correcting a real wall
                    # error. Include the current accepted sample so the low
                    # pass filter cannot keep full speed during a growing tilt.
                    command_speed = self._backup_speed_for_heading(
                        command_speed, min_speed,
                        max(abs(wall_heading_error), abs(lidar_heading_error)),
                        heading_tolerance, heading_pause_error)

                # A large wall error produces a curved path if corrected while
                # reversing.  Stop first, realign in place, then require stable
                # stationary scans before translation resumes.
                if heading_source == 'wheel_guard':
                    heading_correction_active = False
                    heading_settled_count = 0
                was_heading_correction_active = heading_correction_active
                heading_pause_observed_error = (
                    correction_error if heading_correction_active
                    else pause_heading_error)
                heading_correction_active, heading_settled_count = (
                    self._update_backup_heading_correction(
                        heading_correction_active,
                        heading_settled_count,
                        lidar_heading_control_available,
                        (heading_pause_observed_error or 0.0)
                        if lidar_heading_control_available else 0.0,
                        max(abs(stationary_yaw_rate_value),
                            abs(self.current_odom_yaw_rate())),
                        heading_tolerance,
                        heading_pause_error,
                        stationary_yaw_rate,
                        heading_resume_stable_cycles,
                    ))
                if (
                        heading_correction_active
                        and not was_heading_correction_active):
                    self.node.get_logger().warning(
                        'Backup translation paused for heading correction: '
                        f'wall_error={math.degrees(wall_heading_error):+.2f}deg, '
                        f'guide_bias={math.degrees(guide_center_heading):+.2f}deg')
                elif (
                        was_heading_correction_active
                        and not heading_correction_active):
                    # Confirmed physical alignment defines a new straight
                    # heading. Intentional correction is not subsequent drift.
                    imu_heading_reference = self.current_integrated_imu_yaw()
                    wheel_heading_reference = self.current_wheel_yaw()
                    lidar_heading_anchor_error = filtered_lidar_heading_error
                    filtered_anchored_heading_error = filtered_lidar_heading_error
                    lidar_heading_anchor_imu = self.current_integrated_imu_yaw()
                    lidar_heading_anchor_odom = self.current_yaw()
                    lidar_heading_anchor_wheel = self.current_wheel_yaw()
                    self.node.get_logger().info(
                        'Backup heading correction settled; resuming translation '
                        'with the verified heading as the new drift reference')
                # Stationary correction always targets the fixed rear plane.
                # Guide estimates may change while rotating; they must not
                # move this target or cause a reversal during settling.
                if heading_correction_active:
                    guide_center_heading = 0.0
                    correction_error = filtered_lidar_heading_error or 0.0
                    wall_control_heading_source = 'live_lidar'
                wall_control_heading_error = correction_error
                heading_error = normalize_angle(
                    correction_error + guide_center_heading)
                # Centering has its own lateral deadband. The wall's 1-degree
                # deadband otherwise swallows small but necessary corrections.
                command_tolerance = (
                    0.0 if guide_center_heading and not heading_correction_active
                    else heading_tolerance)
                requested_heading_command = (
                    self._backup_heading_angular_velocity(
                        heading_error,
                        stationary_yaw_rate_value,
                        command_tolerance,
                        heading_kp,
                        heading_kd,
                        heading_max_speed,
                        reverse_angular_command_sign,
                    ))
                command_time = time.monotonic()
                if heading_correction_active and lidar_heading_control_available:
                    if not self.imu_is_fresh():
                        self.stop_robot()
                        self.node.get_logger().error(
                            'Stationary backup correction requires fresh independent IMU')
                        return False
                    if stationary_heading_controller is None:
                        stationary_heading_controller = BackupStationaryYawController(
                            float(self.node.get_parameter(
                                'backup_stationary_heading_max_angular_speed').value),
                            float(self.node.get_parameter(
                                'backup_stationary_heading_min_angular_speed').value))
                    # Do not rotate faster while translation is still coasting.
                    velocity = self.last_odom.twist.twist.linear
                    if math.hypot(velocity.x, velocity.y) > 0.005:
                        requested_heading_command = 0.0
                    else:
                        requested_heading_command = stationary_heading_controller.update(
                            heading_error, self.current_integrated_imu_yaw(),
                            self.current_wheel_yaw(), command_time, heading_tolerance)
                        if stationary_heading_controller.fault:
                            self.stop_robot()
                            self.node.get_logger().error(
                                'Stationary backup correction stopped: '
                                + stationary_heading_controller.fault)
                            return False
                elif not heading_correction_active:
                    stationary_heading_controller = None
                if (
                        use_lidar_heading
                        and not lidar_heading_control_available
                        and heading_source != 'wheel_guard'):
                    heading_command = 0.0
                else:
                    heading_command = self._limit_command_rate(
                        current_command.angular.z,
                        requested_heading_command,
                        (0.12 if heading_correction_active else heading_max_accel) * max(
                            command_time - last_heading_command_at, 0.0),
                    )
                last_heading_command_at = command_time
                if guide_handoff is not None:
                    heading_command = 0.0
                current_command = self.twist(
                    linear_x=(
                        0.0 if heading_correction_active
                        else -command_speed),
                    angular_z=heading_command,
                )

            self.cmd_vel_pub.publish(current_command)
            if (
                    blocked_since is not None
                    and time.monotonic() - blocked_since >= blocked_timeout):
                self.stop_robot()
                self.node.get_logger().error(
                    'LiDAR backup remains blocked outside the completion '
                    'condition: '
                    f'center_clearance={clearance:.4f}m, '
                    f'protective_clearance={protective_clearance:.4f}m, '
                    f'completion_band={completion_minimum:.4f}..{completion_threshold:.4f}m, '
                    f'panel_points_in_band={sum(completion_minimum <= v <= completion_threshold for _, v in indexed_clearances)}, '
                    f'completion_cluster={completion_cluster}, '
                    f'stable_scans={stability.count}/{stable_cycles_required}, '
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
                    f'wall_control_error='
                    f'{self._format_optional_degrees(wall_control_heading_error)}, '
                    f'wall_control_source={wall_control_heading_source}, '
                    f'heading_pause_error='
                    f'{self._format_optional_degrees(heading_pause_observed_error)}, '
                    f'heading_pause_source={heading_pause_source}, '
                    f'imu_plane_residual='
                    f'{self._format_optional_degrees(imu_motion_residual)}, '
                    f'odom_plane_residual='
                    f'{self._format_optional_degrees(odom_motion_residual)}, '
                    f'wheel_plane_residual='
                    f'{self._format_optional_degrees(wheel_motion_residual)}, '
                    f'lidar_heading_status={lidar_heading_status}, '
                    f'lidar_heading_stable={lidar_heading_stable_count}/'
                    f'{lidar_heading_stable_cycles}, '
                    f'guide_center_offset='
                    f'{self._format_optional_meters(guide_center_offset)}, '
                    f'guide_rear_entry_offset='
                    f'{self._format_optional_meters(guide_entry_offset)}, '
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
                    f'linear_cmd={current_command.linear.x:.3f}m/s, '
                    f'heading_cmd={current_command.angular.z:.3f}rad/s')
                last_log_time = now

            spin_for(self.node, sleep_time, self.should_stop)

        self.stop_robot()
        return False

    def _wait_for_backup_motion_feedback(self, deadline: float) -> bool:
        """Hold zero until new settled motion and three new LiDAR scans arrive.

        Used only after a corridor has been independently verified. Do not
        replace its IMU heading/travel anchor or extend the backup deadline.
        """
        previous = (self.last_odom_stamp_nanoseconds, self.last_imu_stamp_nanoseconds,
                    self.last_wheel_yaw_stamp_nanoseconds)
        scan_sequence = self.lidar.sequence
        new_scans, settled_samples = 0, 0
        while rclpy.ok() and not self.should_stop() and time.monotonic() < deadline:
            self.cmd_vel_pub.publish(Twist())
            spin_for(self.node, .02, self.should_stop)
            if time.monotonic() >= deadline:
                return False
            # The committed corridor depends on continuous gyro tracking.
            # Recover absolute encoder/odom delivery, never bridge a lost gyro
            # interval by pretending its unmeasured rotation was zero.
            if not self.imu_is_fresh():
                return False
            fresh = (self.odom_is_fresh() and self.imu_is_fresh()
                     and self.wheel_yaw_is_fresh())
            snapshot = self.lidar.snapshot()
            if not fresh or snapshot is None:
                settled_samples = new_scans = 0
                scan_sequence = self.lidar.sequence
                continue
            settled = (self.is_stationary(.005)
                       and abs(self.current_odom_yaw_rate()) <= math.radians(.5))
            if not settled:
                settled_samples = new_scans = 0
                scan_sequence = snapshot.sequence
                continue
            if snapshot.sequence > scan_sequence:
                scan_sequence = snapshot.sequence
                new_scans += 1
            current = (self.last_odom_stamp_nanoseconds, self.last_imu_stamp_nanoseconds,
                       self.last_wheel_yaw_stamp_nanoseconds)
            if all(new > old for new, old in zip(current, previous)):
                previous = current
                settled_samples += 1
            if settled_samples >= 5 and new_scans >= 3:
                return True
        return False

    def stop_robot(self) -> None:
        if not rclpy.ok():
            return
        stop = Twist()
        for _ in range(5):
            self.cmd_vel_pub.publish(stop)
            spin_for(self.node, 0.02)

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
                header_age = (self.node.get_clock().now().nanoseconds
                              - self.last_odom_stamp_nanoseconds) / 1e9
                receipt_age = time.monotonic() - self.last_odom_received_at
                self.node.get_logger().error(
                    'Odometry became stale during motion: '
                    f'header_age={header_age:.3f}s, receipt_age={receipt_age:.3f}s, '
                    f'last_rejection={self.last_odom_error or "none"}')
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
            spin_for(self.node, sleep_time, self.should_stop)

        return False

    def _make_absolute_spin_done_cb(
            self, target_yaw: float, tolerance: float,
            stable_cycles: int, direction: float,
            stationary_yaw_rate: float) -> Any:
        stable_count = 0
        last_sequence = self.odom_sequence

        def done() -> bool:
            nonlocal stable_count, last_sequence
            error = self._absolute_yaw_error(
                target_yaw, self.current_yaw(), direction)
            stationary_yaw_rate_value, _ = (
                self.current_stationary_yaw_rate())
            if (
                    abs(error) <= tolerance
                    and abs(stationary_yaw_rate_value)
                    <= stationary_yaw_rate):
                if self.odom_sequence != last_sequence:
                    stable_count += 1
            else:
                stable_count = 0
            last_sequence = self.odom_sequence
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
        # Near 180 degrees, tiny pose noise changes the shortest-path sign.
        # Keep the requested direction through the first five degrees instead
        # of reversing caster load on successive callbacks. Once away from
        # this ambiguous region, normal shortest-path overshoot correction
        # still applies. Preserve the actual remaining angle in either case.
        if abs(error) >= math.pi - math.radians(5):
            if error * direction < 0:
                return math.copysign(2 * math.pi - abs(error), direction)
            return math.copysign(abs(error), direction)
        return error

    def _make_backup_done_cb(self, distance: float, tolerance: float) -> Any:
        start_x, start_y = self.current_xy()

        def done() -> bool:
            current_x, current_y = self.current_xy()
            traveled = math.hypot(current_x - start_x, current_y - start_y)
            return traveled >= max(distance - tolerance, 0.0)

        return done

    def _odom_callback(self, msg: Odometry) -> None:
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        if not self._sensor_stamp_is_acceptable(
                stamp, self.last_odom_stamp_nanoseconds, 'odom_max_age_sec'):
            self.last_odom_error = 'invalid, repeated, out-of-order or stale odom stamp'
            return
        if msg.header.frame_id.strip().lstrip('/') != str(
                self.node.get_parameter('fixed_frame').value).lstrip('/'):
            self.last_odom_error = 'odom header frame does not match fixed_frame'
            return
        if not self._odom_child_matches_base(msg):
            self.last_odom_error = (
                'odom child frame must match base_frame in planar position and rotation')
            return
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
        if (
                not all(math.isfinite(value) for value in (
                    position.x, position.y, position.z, *quaternion,
                    msg.twist.twist.linear.x, msg.twist.twist.linear.y,
                    msg.twist.twist.angular.z))
                or abs(sum(value * value for value in quaternion) - 1.0) > 1e-3):
            self.last_odom_error = 'odom pose or velocity is invalid'
            return
        self.last_odom = msg
        self.last_odom_received_at = time.monotonic()
        self.last_odom_stamp_nanoseconds = stamp
        self.odom_sequence += 1
        self.last_odom_error = ''

    def _odom_child_matches_base(self, msg: Odometry) -> bool:
        child = msg.child_frame_id.strip().lstrip('/')
        base = str(self.node.get_parameter('base_frame').value).lstrip('/')
        if child == base:
            return True
        if not child:
            return False
        try:
            transform = self.node.tf_buffer.lookup_transform(
                child, base, Time.from_msg(msg.header.stamp)).transform
        except TransformException:
            return False
        # STELLA odometry uses base_footprint. Its fixed base_link joint adds
        # only 0.071 m in z, so x/y/yaw and planar twist are identical. Accept
        # that verified equivalence, but never compare camera/offset poses as
        # if they were the robot center.
        translation = transform.translation
        rotation = transform.rotation
        return (
            all(math.isfinite(value) for value in (
                translation.x, translation.y, translation.z,
                rotation.x, rotation.y, rotation.z, rotation.w))
            and max(abs(translation.x), abs(translation.y), abs(rotation.x),
                    abs(rotation.y), abs(rotation.z)) <= 1e-6
            and abs(abs(rotation.w) - 1.0) <= 1e-6)

    def _imu_callback(self, msg: Imu) -> None:
        if not math.isfinite(msg.angular_velocity.z):
            return
        stamp_nanoseconds = (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec))
        if not self._sensor_stamp_is_acceptable(
                stamp_nanoseconds, self.last_imu_stamp_nanoseconds,
                'imu_max_age_sec'):
            return
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
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        if not self._sensor_stamp_is_acceptable(
                stamp, self.last_wheel_yaw_stamp_nanoseconds,
                'wheel_yaw_max_age_sec'):
            return
        self.last_wheel_yaw = msg.vector.x
        self.last_wheel_yaw_received_at = time.monotonic()
        self.last_wheel_yaw_stamp_nanoseconds = stamp

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
        stationary_max_speed = float(self.node.get_parameter(
            'backup_stationary_heading_max_angular_speed').value)
        stationary_min_speed = float(self.node.get_parameter(
            'backup_stationary_heading_min_angular_speed').value)
        heading_pause_error = float(self.node.get_parameter(
            'backup_heading_pause_error').value)
        lidar_heading_filter_coef = float(self.node.get_parameter(
            'backup_lidar_heading_filter_coef').value)
        lidar_heading_max_error = float(self.node.get_parameter(
            'backup_lidar_heading_max_error').value)
        lidar_motion_residual = float(self.node.get_parameter(
            'backup_lidar_motion_residual').value)
        lidar_wheel_guard_residual = float(self.node.get_parameter(
            'backup_lidar_wheel_guard_residual').value)
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
            stationary_max_speed, stationary_min_speed,
            heading_pause_error,
            lidar_heading_filter_coef, lidar_heading_max_error,
            lidar_motion_residual, lidar_wheel_guard_residual,
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
                or not 0.0 < stationary_min_speed <= stationary_max_speed <= 0.06
                or heading_pause_error <= heading_tolerance
                or heading_pause_error >= lidar_heading_max_error
                or not 0.0 < lidar_heading_filter_coef <= 1.0
                or not 0.0 < lidar_heading_max_error < math.pi / 2.0
                or not 0.0 < lidar_motion_residual < lidar_heading_max_error
                or not lidar_motion_residual < lidar_wheel_guard_residual <= (
                    lidar_heading_max_error)
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
            and self._sensor_stamp_is_acceptable(
                self.last_odom_stamp_nanoseconds, 0, 'odom_max_age_sec')
        )

    def wheel_yaw_is_fresh(self) -> bool:
        max_age = max(float(
            self.node.get_parameter('wheel_yaw_max_age_sec').value), 0.0)
        return (
            self.last_wheel_yaw is not None
            and self.last_wheel_yaw_received_at > 0.0
            and time.monotonic() - self.last_wheel_yaw_received_at <= max_age
            and self._sensor_stamp_is_acceptable(
                self.last_wheel_yaw_stamp_nanoseconds, 0, 'wheel_yaw_max_age_sec')
        )

    def imu_is_fresh(self) -> bool:
        max_age = max(float(
            self.node.get_parameter('imu_max_age_sec').value), 0.0)
        return (
            self.last_imu_yaw_rate is not None
            and self.last_imu_received_at > 0.0
            and time.monotonic() - self.last_imu_received_at <= max_age
            and self._sensor_stamp_is_acceptable(
                self.last_imu_stamp_nanoseconds, 0, 'imu_max_age_sec')
        )

    def _sensor_stamp_is_acceptable(
            self, stamp: int, previous: int, age_parameter: str) -> bool:
        max_age = float(self.node.get_parameter(age_parameter).value)
        future = float(self.node.get_parameter(
            'motion_sensor_future_tolerance_sec').value)
        return (
            math.isfinite(max_age) and max_age > 0.0
            and math.isfinite(future) and future >= 0.0
            and header_stamp_is_acceptable(
                stamp, previous, self.node.get_clock().now().nanoseconds,
                int(max_age * 1e9), int(future * 1e9)))

    def is_stationary(self, linear_tolerance: float) -> bool:
        if not self.odom_is_fresh():
            return False
        velocity = self.last_odom.twist.twist.linear
        yaw_rate, _ = self.current_stationary_yaw_rate()
        yaw_tolerance = float(self.node.get_parameter(
            'imu_stationary_yaw_rate').value)
        return (
            math.hypot(velocity.x, velocity.y) <= linear_tolerance
            and abs(yaw_rate) <= yaw_tolerance)

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
    def _backup_speed_for_heading(
            speed: float, min_speed: float, error: float,
            tolerance: float, pause_error: float) -> float:
        if abs(error) <= tolerance:
            return speed
        fraction = min(
            max((abs(error) - tolerance) / (pause_error - tolerance), 0.0), 1.0)
        return speed + (min(speed, min_speed) - speed) * fraction

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
    def _lidar_heading_motion_residual(
            candidate_error: float, anchor_error: float,
            current_yaw: float, anchor_yaw: float) -> float:
        """Residual of the fixed-wall invariant: error + robot yaw."""
        return normalize_angle(
            candidate_error - anchor_error + current_yaw - anchor_yaw)

    @staticmethod
    def _lidar_heading_motion_is_consistent(
            imu_residual: float, odom_residual: float,
            wheel_residual: float,
            limit: float) -> bool:
        """Accept agreement with either inertial yaw or encoder-only yaw.

        Odom yaw is corrected from the same IMU, so IMU and odom are one
        correlated evidence group.  Encoder-only yaw is independent.  A dock
        plane is implausible only when it conflicts with both groups.  This
        still accepts real chassis slip (plane + inertial agree while wheels
        do not) and rejects an unrelated RANSAC surface when neither agrees.
        """
        threshold = abs(limit)
        inertial_conflict = (
            abs(imu_residual) > threshold
            and abs(odom_residual) > threshold
            and imu_residual * odom_residual > 0.0
        )
        wheel_conflict = abs(wheel_residual) > threshold
        return not (inertial_conflict and wheel_conflict)

    @staticmethod
    def _backup_wheel_guard_is_allowed(
            wheel_residual: float, wheel_heading_drift: float,
            residual_limit: float, max_yaw_drift: float) -> bool:
        """Bound straight encoder fallback when a stable plane is suspect."""
        return (
            math.isfinite(wheel_residual)
            and math.isfinite(wheel_heading_drift)
            and abs(wheel_residual) <= abs(residual_limit)
            and abs(wheel_heading_drift) < abs(max_yaw_drift)
        )

    @staticmethod
    def _backup_guide_center_heading(
            center_offset: float, tolerance: float,
            kp: float, max_heading: float,
            remaining_distance: float | None = None) -> float:
        if abs(center_offset) <= tolerance:
            return 0.0
        excess = math.copysign(abs(center_offset) - tolerance, center_offset)
        requested = -kp * excess
        if remaining_distance is not None:
            # Correct early, allowing a roughly triangular heading profile
            # to return parallel before the final straight segment.
            requested = -math.atan2(2.0 * kp * excess, max(remaining_distance, 0.05))
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
