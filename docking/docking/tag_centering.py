"""Bounded live front centering from AprilTag or legacy LiDAR panel normals."""

import copy
from collections import deque
import math
import statistics
import time

from geometry_msgs.msg import PoseStamped

from docking.control import BackupStationaryYawController, spin_for
from docking.docking_lidar import DockingLidar
from docking.lidar_alignment import LidarPlaneAligner
from docking.lidar_geometry import normalize_angle, plane_normal_alignment_error


class _FrontLidarParameters:
    """Share scan freshness rules while selecting the independent front sensor."""

    def __init__(self, node):
        self.node = node

    def __getattr__(self, name):
        return getattr(self.node, name)

    def get_parameter(self, name):
        aliases = {
            'docking_lidar_topic': 'tag_centering_lidar_topic',
            'docking_lidar_frame': 'tag_centering_lidar_frame',
            'lidar_align_candidate_max_error': 'tag_centering_candidate_max_error',
        }
        return self.node.get_parameter(aliases.get(name, name))


def centered_goal_in_base(goal_x, goal_y, tag_x, tag_y, panel_yaw):
    """Keep the old standoff, place the goal on the tag's panel-normal line.

    A plane supplies orientation, not an absolute lateral center. Only the
    observed tag position supplies that center; cropped panel endpoints do not.
    """
    c, s = math.cos(panel_yaw), math.sin(panel_yaw)
    longitudinal = c * goal_x + s * goal_y
    lateral = -s * tag_x + c * tag_y
    return c * longitudinal - s * lateral, s * longitudinal + c * lateral


def standoff_explains_target_shift(goal_x, goal_y, tag_x, tag_y,
                                  original_yaw, panel_yaw):
    """Validate a tag-normal target correction, not a motion budget.

    The old goal must be on the old tag-normal line within the same 15 mm
    tolerance as stationary observations. Actual travel and rotation remain
    subject to the caller's independent refinement limits.
    """
    if not all(math.isfinite(v) for v in (
            goal_x, goal_y, tag_x, tag_y, original_yaw, panel_yaw)):
        return False
    dx, dy = tag_x - goal_x, tag_y - goal_y
    c, s = math.cos(original_yaw), math.sin(original_yaw)
    standoff, residual = c * dx + s * dy, -s * dx + c * dy
    return (.5 <= standoff <= 1.5 and abs(residual) <= .015
            and abs(normalize_angle(panel_yaw - original_yaw)) <= math.radians(15))


class LidarTagCentering:
    observation_source = 'front LiDAR panel'
    pose_source = 'live_lidar_tag'
    initial_alignment_center_limit = .05
    initial_heading_lateral_limit = .08

    @staticmethod
    def declare_parameters(node):
        node.declare_parameter('use_lidar_tag_centering', False)
        node.declare_parameter('use_apriltag_front_centering', True)
        node.declare_parameter('apriltag_front_standoff', .95)
        node.declare_parameter('apriltag_front_axle_x', 0.0)
        node.declare_parameter('tag_front_body_reference_x', .2295)
        node.declare_parameter('tag_front_min_lidar_clearance', .05)
        node.declare_parameter('tag_centering_lidar_topic', '/scan')
        node.declare_parameter('tag_centering_lidar_frame', 'base_scan')
        node.declare_parameter('tag_centering_wait_sec', 1.2)
        node.declare_parameter('tag_centering_stable_pairs', 3)
        node.declare_parameter('tag_centering_max_shift', 0.10)
        node.declare_parameter('tag_centering_max_yaw_change', math.radians(15))
        node.declare_parameter('tag_centering_tag_plane_max_distance', 0.25)
        # Before refinement the robot can face the panel at up to 30 degrees.
        # The rear aligner's 15-degree prior applies only after the 180 turn.
        node.declare_parameter('tag_centering_candidate_max_error', math.radians(30))

    def __init__(self, node):
        self.node = node
        front_parameters = _FrontLidarParameters(node)
        self.lidar = DockingLidar(front_parameters, node.tf_buffer)
        self.fitter = LidarPlaneAligner(front_parameters, node.motion, self.lidar)

    def panel(self, snapshot, tag):
        # The tag selects the front station sector, keeping unrelated room
        # walls and side rails out of the normal estimate.
        bearing = math.atan2(tag.pose.position.y, tag.pose.position.x)
        self.last_panel_diagnostic = f'tag bearing={math.degrees(bearing):+.1f}deg'
        if tag.pose.position.x <= 0 or abs(bearing) > math.radians(30):
            return None
        # Prefer the panel around the tag itself. Wide sectors can mix its
        # edges with rails/background and bias the fitted normal (trial 37).
        # Expand only if the narrow view cannot establish full line support.
        for width in (20, 30, 40):
            yaw = self._panel_in_sector(snapshot, tag, bearing, width)
            if yaw is not None:
                return yaw
        return None

    def _panel_in_sector(self, snapshot, tag, bearing, width):
        projected = self.lidar.project(
            snapshot, bearing, math.radians(width), 0.15, 2.0)
        if projected is None or len(projected) < 20:
            self.last_panel_diagnostic = (
                f'front sector={width}deg, '
                f'projected points={0 if projected is None else len(projected)}/20')
            return None
        points = [(p.x, p.y) for p in projected]
        inliers = self.fitter._ransac_line_inliers(points)
        fit = self.fitter._fit_line_pca(inliers)
        self.last_panel_diagnostic = (
            f'front sector={width}deg, panel inliers={len(inliers)}/{len(points)}, '
            f'ratio={len(inliers) / len(points):.3f} (required 0.700)')
        if fit is None or len(inliers) < 20 or len(inliers) / len(points) < .70:
            return None
        angle, length = fit
        self.last_panel_diagnostic += f', length={length:.3f}m (required 0.200)'
        if length < .20:
            return None
        yaw = plane_normal_alignment_error(angle, 0.0)
        c, s = math.cos(yaw), math.sin(yaw)
        distance = statistics.median(c * x + s * y for x, y in inliers)
        tag_distance = c * tag.pose.position.x + s * tag.pose.position.y
        limit = float(self.node.get_parameter(
            'tag_centering_tag_plane_max_distance').value)
        self.last_panel_diagnostic += (
            f', plane_distance={distance:.3f}m, tag_distance={tag_distance:.3f}m, '
            f'difference={abs(tag_distance - distance):.3f}m (limit {limit:.3f})')
        if distance <= 0 or abs(tag_distance - distance) > limit:
            return None
        return yaw

    def control_observation(self, standoff):
        """Fresh station-relative errors while moving; never refresh old data."""
        node = self.node
        tag = node.last_dock_pose
        snapshot = self.lidar.snapshot()
        if (snapshot is None or not node._pose_is_fresh(
                tag, node.last_dock_pose_received_at, .30)
                or tag.header.frame_id.lstrip('/') != str(
                    node.get_parameter('base_frame').value).lstrip('/')):
            return None
        scan_stamp = (snapshot.scan.header.stamp.sec * 1_000_000_000
                      + snapshot.scan.header.stamp.nanosec)
        tag_stamp = tag.header.stamp.sec * 1_000_000_000 + tag.header.stamp.nanosec
        if abs(scan_stamp - tag_stamp) > 150_000_000:
            return None
        key = (snapshot.sequence, tag_stamp, standoff)
        if key == getattr(self, '_control_observation_key', None):
            return self._control_observation
        yaw = self.panel(snapshot, tag)
        result = None
        if yaw is not None:
            c, s = math.cos(yaw), math.sin(yaw)
            result = (c * tag.pose.position.x + s * tag.pose.position.y - standoff,
                      -s * tag.pose.position.x + c * tag.pose.position.y, yaw)
            if not all(math.isfinite(v) for v in result):
                result = None
        self._control_observation_key, self._control_observation = key, result
        if result is not None:
            previous = getattr(self, '_control_counted_pair', (-1, -1))
            if snapshot.sequence != previous[0] and tag_stamp != previous[1]:
                self.control_observation_sequence = getattr(
                    self, 'control_observation_sequence', 0) + 1
                self._control_counted_pair = (snapshot.sequence, tag_stamp)
        return result

    def _goal_and_standoff(self, original_x, original_y, tag, heading):
        gx, gy = centered_goal_in_base(
            original_x, original_y, tag.pose.position.x, tag.pose.position.y, heading)
        standoff = (math.cos(heading)*(tag.pose.position.x-original_x)
                    + math.sin(heading)*(tag.pose.position.y-original_y))
        return gx, gy, standoff

    def refine(self, target, allow_standoff_yaw_correction=False):
        self.last_verified_alignment = None
        self.last_verified_standoff = None
        node = self.node
        limits = [float(node.get_parameter(name).value) for name in (
            'tag_centering_wait_sec', 'tag_centering_max_shift',
            'tag_centering_max_yaw_change', 'tag_centering_tag_plane_max_distance')]
        if not all(math.isfinite(v) and v > 0 for v in limits):
            node.get_logger().warning('Invalid front LiDAR centering limits; using tag target')
            return target
        wait, max_shift, max_yaw, _ = limits
        required_pairs = int(node.get_parameter('tag_centering_stable_pairs').value)
        if not 3 <= required_pairs <= 10:
            node.get_logger().warning('Front centering requires 3 to 10 independent pairs')
            return target
        candidate_limit = float(node.get_parameter('tag_centering_candidate_max_error').value)
        if not math.isfinite(candidate_limit) or not 0 < candidate_limit <= math.radians(30):
            node.get_logger().warning('Invalid front panel candidate angle; using tag target')
            return target
        if not self.lidar.validate_parameters():
            node.get_logger().warning('Invalid front LiDAR input; using tag target')
            return target
        original_yaw = node._quaternion_yaw(
            target.pose.orientation.x, target.pose.orientation.y,
            target.pose.orientation.z, target.pose.orientation.w)
        if not math.isfinite(original_yaw):
            return target
        node.motion.stop_robot()
        deadline = time.monotonic() + wait
        samples = []
        reason = 'waiting for fresh stationary observations'
        previous_scan = self.lidar.sequence
        previous_tag = -1
        while not node.should_stop() and time.monotonic() < deadline:
            spin_for(node, .02, node.should_stop)
            tag = node.last_dock_pose
            snapshot = self.lidar.snapshot()
            if (snapshot is None or not node._pose_is_fresh(
                    tag, node.last_dock_pose_received_at, .30)
                    or tag.header.frame_id.lstrip('/') != str(
                        node.get_parameter('base_frame').value).lstrip('/')
                    or not node.motion.is_stationary(.005)
                    or abs(node.motion.current_odom_yaw_rate()) > math.radians(.5)):
                samples = []
                reason = ('front scan unavailable: ' + self.lidar.last_error
                          if snapshot is None else
                          'tag is stale, in the wrong frame, or robot is not stationary')
                continue
            stamp = tag.header.stamp.sec * 1_000_000_000 + tag.header.stamp.nanosec
            if snapshot.sequence == previous_scan or stamp == previous_tag:
                continue
            previous_scan, previous_tag = snapshot.sequence, stamp
            yaw = self.panel(snapshot, tag)
            if yaw is None:
                samples = []
                reason = getattr(self, 'last_panel_diagnostic',
                                 'front plane support or tag-to-plane agreement is insufficient')
                continue
            x, y = node.motion.current_xy()
            robot_yaw = node.motion.current_yaw()
            c, s = math.cos(robot_yaw), math.sin(robot_yaw)
            dx, dy = target.pose.position.x - x, target.pose.position.y - y
            original_gx, original_gy = c * dx + s * dy, -s * dx + c * dy
            gx, gy, standoff = self._goal_and_standoff(
                original_gx, original_gy, tag, yaw)
            corrected = (x + c * gx - s * gy, y + s * gx + c * gy,
                         normalize_angle(robot_yaw + yaw),
                         -math.sin(yaw) * tag.pose.position.x
                         + math.cos(yaw) * tag.pose.position.y, yaw,
                         standoff)
            shift = math.hypot(corrected[0] - target.pose.position.x,
                               corrected[1] - target.pose.position.y)
            yaw_change = abs(normalize_angle(corrected[2] - original_yaw))
            explained_shift = (allow_standoff_yaw_correction
                               and standoff_explains_target_shift(
                                   original_gx, original_gy,
                                   tag.pose.position.x, tag.pose.position.y,
                                   normalize_angle(original_yaw - robot_yaw), yaw))
            if (not all(math.isfinite(v) for v in corrected)
                    or (shift > max_shift and not explained_shift)
                    or yaw_change > max_yaw):
                samples = []
                reason = (
                    'proposed target correction exceeds its finite/shift/yaw limits: '
                    f'shift={shift:.3f}m (limit {max_shift:.3f}m), '
                    f'yaw_change={math.degrees(yaw_change):.2f}deg '
                    f'(limit {math.degrees(max_yaw):.2f}deg), '
                    f'standoff_explained={explained_shift}')
                continue
            if samples and any(
                    math.hypot(corrected[0] - p[0], corrected[1] - p[1]) > .015
                    or abs(normalize_angle(corrected[2] - p[2])) > math.radians(1)
                    for p in samples):
                samples = []
            samples.append(corrected)
            if len(samples) >= required_pairs:
                result = copy.deepcopy(target)
                result.pose.position.x = statistics.median(p[0] for p in samples)
                result.pose.position.y = statistics.median(p[1] for p in samples)
                result_yaw = normalize_angle(original_yaw + statistics.median(
                    normalize_angle(p[2] - original_yaw) for p in samples))
                result.pose.orientation.x = result.pose.orientation.y = 0.0
                result.pose.orientation.z = math.sin(result_yaw / 2)
                result.pose.orientation.w = math.cos(result_yaw / 2)
                shift = math.hypot(
                    result.pose.position.x - target.pose.position.x,
                    result.pose.position.y - target.pose.position.y)
                node.get_logger().info(
                    f'Front centering from {self.observation_source}: '
                    f'goal_shift={shift:.3f}m, '
                    f'yaw_change={math.degrees(normalize_angle(result_yaw - original_yaw)):+.2f}deg, '
                    f'distinct_pairs={len(samples)}, '
                    f'standoff={statistics.median(p[5] for p in samples):.3f}m; '
                    f'standoff_yaw_correction={shift > max_shift}; '
                    f'observed_lateral={statistics.median(p[3] for p in samples):+.3f}m, '
                    f'observed_heading={math.degrees(statistics.median(p[4] for p in samples)):+.2f}deg')
                self.last_verified_alignment = [(p[3], p[4]) for p in samples]
                self.last_verified_standoff = statistics.median(p[5] for p in samples)
                return result
        node.get_logger().warning(
            f'Front centering from {self.observation_source} not verified within the bounded wait: '
            f'{reason}, distinct_pairs={len(samples)}/{required_pairs}; '
            'keeping the original tag target and rear entry safety checks')
        return target

    def verify_before_spin(self, allow_correction=True):
        """Check actual station-relative centering after odom refinement.

        A small residual yaw may be corrected once directly from the panel;
        the original centering bounds must then pass new stationary samples.
        """
        node = self.node
        anchor = PoseStamped()
        anchor.header.frame_id = str(node.get_parameter('fixed_frame').value)
        anchor.pose.position.x, anchor.pose.position.y = node.motion.current_xy()
        yaw = node.motion.current_yaw()
        anchor.pose.orientation.z = math.sin(yaw / 2)
        anchor.pose.orientation.w = math.cos(yaw / 2)
        self.refine(anchor)
        samples = self.last_verified_alignment
        if not samples:
            node.get_logger().error(
                'Pre-spin live centering could not be verified; refusing the turn')
            return False
        lateral_limit = float(node.get_parameter('tag_refinement_lateral_tolerance').value)
        yaw_limit = float(node.get_parameter('tag_refinement_yaw_tolerance').value)
        lateral = statistics.median(p[0] for p in samples)
        heading = statistics.median(p[1] for p in samples)
        verified = all(abs(p[0]) <= lateral_limit and abs(p[1]) <= yaw_limit
                       for p in samples)
        # This is readiness for the subsequent guide-centering stage, not
        # final guide entry. A small residual may be handed over only to the
        # active, guarded LiDAR backup controller and with a tighter heading.
        guide_backup = (bool(node.get_parameter('use_lidar_backup').value)
                        and bool(node.get_parameter('use_lidar_guide_centering').value))
        handoff_limit = max(lateral_limit, min(1.5*lateral_limit, .015))
        handoff = (not verified and guide_backup and all(
            abs(p[0]) <= handoff_limit and abs(p[1]) <= min(yaw_limit, math.radians(1))
            for p in samples))
        message = (
            f'Pre-spin live station alignment: lateral={lateral:+.3f}m '
            f'(limit {lateral_limit:.3f}m), heading={math.degrees(heading):+.2f}deg '
            f'(limit {math.degrees(yaw_limit):.2f}deg), distinct_pairs={len(samples)}, '
            f'verified={verified}, rear_centering_handoff={handoff}; '
            f'measured from fresh tag position and {self.observation_source}')
        if handoff:
            node.get_logger().info(
                message + f'; pre-spin handoff limit={handoff_limit:.3f}m; '
                'final rear entry center/side-clearance gates remain unchanged')
            return True
        if verified:
            node.get_logger().info(message)
        elif (allow_correction and math.radians(1) < abs(heading) <= math.radians(4)
              and all(abs(p[0]) <= .025 and abs(p[1]) <= math.radians(4)
                      for p in samples)):
            node.get_logger().warning(message + '; trying one bounded live panel yaw correction')
            return self._correct_live_heading() and self.verify_before_spin(False)
        else:
            node.get_logger().error(message + '; refusing the turn')
        return verified

    def _correct_live_heading(self, initial=False):
        node, motion = self.node, self.node.motion
        motion.stop_robot()
        if not (motion.odom_is_fresh() and motion.imu_is_fresh()
                and motion.wheel_yaw_is_fresh() and motion.is_stationary(.005)):
            node.get_logger().error('Live front yaw correction requires stationary fresh feedback')
            return False
        gyro_start = motion.current_integrated_imu_yaw()
        wheel_start = motion.current_wheel_yaw()
        # Initial alignment has the existing approach distance still available;
        # its center is observed again before any translation. The final trim
        # retains the narrower limits used at the already refined stop pose.
        duration = 8 if initial else 6
        lateral_bound = self.initial_heading_lateral_limit if initial else .025
        # The initial stage covers the controller's existing 18-degree budget.
        # A 16-degree approach must not silently revert to translating first.
        # The controller still caps cumulative wheel AND gyro travel at 18 deg.
        angle_bound = math.radians(18 if initial else 5)
        gyro_budget = math.radians(18 if initial else 6)
        wheel_budget = math.radians(18 if initial else 8)
        controller = BackupStationaryYawController(
            max_speed=.06 if initial else .04, min_speed=.025)
        deadline = time.monotonic() + duration
        sequence, stable = -1, 0
        timing_wait_started = None
        try:
            while not node.should_stop() and time.monotonic() < deadline:
                spin_for(node, .05, node.should_stop)
                scan, tag = self.lidar.snapshot(), node.last_dock_pose
                if not (motion.odom_is_fresh() and motion.imu_is_fresh()
                        and motion.wheel_yaw_is_fresh()):
                    node.get_logger().error('Live front yaw correction lost fresh motion feedback')
                    return False
                if (tag is not None and tag.header.frame_id.lstrip('/') != str(
                        node.get_parameter('base_frame').value).lstrip('/')):
                    node.get_logger().error('Live front yaw correction received a wrong tag frame')
                    return False
                observations_fresh = (scan is not None and node._pose_is_fresh(
                    tag, node.last_dock_pose_received_at, .30))
                mismatch = math.inf
                if observations_fresh:
                    tag_stamp = tag.header.stamp.sec + tag.header.stamp.nanosec / 1e9
                    scan_stamp = scan.scan.header.stamp.sec + scan.scan.header.stamp.nanosec / 1e9
                    mismatch = abs(tag_stamp - scan_stamp)
                if not observations_fresh or mismatch > .15:
                    # Camera and LiDAR callbacks are asynchronous. Never keep
                    # turning on an unmatched pair, but allow a short stopped
                    # reacquisition inside the original motion/time budgets.
                    motion.cmd_vel_pub.publish(motion.twist())
                    stable = 0
                    now = time.monotonic()
                    if timing_wait_started is None:
                        timing_wait_started = now
                        node.get_logger().warning(
                            'Live front yaw paused for fresh synchronized tag/scan observations: '
                            f'fresh={observations_fresh}, delta={mismatch:.3f}s')
                    if now - timing_wait_started >= .5:
                        node.get_logger().error(
                            'Live front yaw tag/scan observation gap persisted for 0.5 seconds')
                        return False
                    continue
                timing_wait_started = None
                gyro = motion.current_integrated_imu_yaw()
                wheel = motion.current_wheel_yaw()
                if (abs(gyro - gyro_start) > gyro_budget
                        or abs(normalize_angle(wheel-wheel_start)) > wheel_budget):
                    node.get_logger().error('Live front yaw correction exceeded its rotation budget')
                    return False
                if scan.sequence == sequence:
                    continue
                sequence = scan.sequence
                error = self.panel(scan, tag)
                lateral = (math.inf if error is None else
                           -math.sin(error)*tag.pose.position.x
                           + math.cos(error)*tag.pose.position.y)
                if (error is None or abs(error) > angle_bound
                        or abs(lateral) > lateral_bound or tag.pose.position.x < .5):
                    node.get_logger().error('Live front yaw correction lost its bounded station geometry')
                    return False
                tolerance = math.radians(.75)
                command = controller.update(error, gyro, wheel, time.monotonic(), tolerance)
                if controller.fault:
                    node.get_logger().error('Live front yaw correction stopped: ' + controller.fault)
                    return False
                stationary = (motion.is_stationary(.005)
                              and abs(motion.current_odom_yaw_rate()) <= math.radians(.5))
                stable = stable + 1 if abs(error) <= tolerance and stationary else 0
                motion.cmd_vel_pub.publish(motion.twist(angular_z=command))
                if stable >= 3:
                    node.get_logger().info(
                        f'Live front yaw correction complete: heading={math.degrees(error):+.2f}deg, '
                        f'lateral={lateral:+.3f}m; rechecking stationary centering')
                    return True
            node.get_logger().error(
                f'Live front yaw correction exceeded {duration} seconds or was stopped')
            return False
        finally:
            motion.stop_robot()


def apriltag_normal_yaw(tag):
    """AprilTag's local z axis is its face normal, not its Euler yaw.

    The detector's tag frame uses image-plane x/y axes. The normal points
    toward the camera; use its opposite as the robot's approach heading.
    This matches the configured external detection pitch/roll conversion.
    """
    q = tag.pose.orientation
    norm = math.hypot(q.x, q.y, q.z, q.w)
    if not math.isfinite(norm) or norm < 1e-6:
        return None
    x, y, z, w = (v/norm for v in (q.x, q.y, q.z, q.w))
    nx, ny = 2*(x*z + w*y), 2*(y*z - w*x)
    if math.hypot(nx, ny) < .9:
        return None
    if nx < 0:
        nx, ny = -nx, -ny
    return math.atan2(ny, nx)


class AprilTagCentering(LidarTagCentering):
    """AprilTag supplies center and heading; LiDAR only checks front distance."""

    observation_source = 'AprilTag orientation (LiDAR distance guard only)'
    pose_source = 'live_apriltag'
    # This is the starting center error in the open front approach area, not
    # an allowed guide-entry error. Rotation/time/feedback bounds stay fixed.
    initial_alignment_center_limit = .15
    initial_heading_lateral_limit = .15

    def __init__(self, node):
        self.node = node
        self.lidar = DockingLidar(_FrontLidarParameters(node), node.tf_buffer)

    def refine(self, target, allow_standoff_yaw_correction=False):
        self._front_pose_window = deque(maxlen=5)
        self._last_filtered_tag_stamp = None
        return super().refine(target, allow_standoff_yaw_correction)

    def control_observation(self, standoff):
        raw = super().control_observation(standoff)
        motion = self.node.motion
        if raw is None or not motion.imu_is_fresh():
            self._front_pose_window = deque(maxlen=5)
            self._last_filtered_tag_stamp = None
            return None
        gyro = motion.current_integrated_imu_yaw()
        if not math.isfinite(gyro):
            return None
        tag = self.node.last_dock_pose
        stamp = tag.header.stamp.sec + tag.header.stamp.nanosec*1e-9
        window = getattr(self, '_front_pose_window', None)
        if window is None:
            window = self._front_pose_window = deque(maxlen=5)
        if stamp != getattr(self, '_last_filtered_tag_stamp', None):
            while window and stamp-window[0][0] > .5:
                window.popleft()
            window.append((stamp, gyro, raw[2], tag.pose.position.x, tag.pose.position.y))
            self._last_filtered_tag_stamp = stamp
        # Compare face normals in a common gyro reference. Averaging the raw
        # base-frame yaw would lag every real turn and itself create chatter.
        reference = window[-1][1]+window[-1][2]
        reference += statistics.median(normalize_angle(g+a-reference)
                                       for _,g,a,_,_ in window)
        positions = []
        for _,g,_,x,y in window:
            heading = normalize_angle(reference-g)
            c,s = math.cos(heading), math.sin(heading)
            positions.append((c*x+s*y-standoff, -s*x+c*y))
        return (statistics.median(p[0] for p in positions),
                statistics.median(p[1] for p in positions),
                normalize_angle(reference-gyro))

    def _goal_and_standoff(self, original_x, original_y, tag, heading):
        # One virtual pose 95 cm in front of the tag for both approach and
        # precision control; never inherit a drifting odometry standoff.
        standoff = float(self.node.get_parameter('apriltag_front_standoff').value)
        if not math.isfinite(standoff) or not .5 <= standoff <= 1.5:
            return math.nan, math.nan, math.nan
        return (tag.pose.position.x-standoff*math.cos(heading),
                tag.pose.position.y-standoff*math.sin(heading), standoff)

    def panel(self, snapshot, tag):
        # Keep the common bounded observation/controller interface. This mode
        # deliberately has no line fitter and never derives yaw from a scan.
        heading = apriltag_normal_yaw(tag)
        bearing = math.atan2(tag.pose.position.y, tag.pose.position.x)
        if (heading is None or not math.isfinite(bearing)
                or tag.pose.position.x <= 0
                or abs(bearing) > math.radians(30)
                or abs(heading) > math.radians(30)):
            self.last_panel_diagnostic = 'AprilTag face normal or forward view is invalid'
            return None
        front_x = float(self.node.get_parameter('tag_front_body_reference_x').value)
        margin = float(self.node.get_parameter('tag_front_min_lidar_clearance').value)
        if (not all(math.isfinite(v) for v in (front_x, margin))
                or front_x <= 0 or margin <= 0):
            self.last_panel_diagnostic = 'Invalid front LiDAR clearance parameters'
            return None
        points = self.lidar.project(snapshot, 0., math.radians(10), .05, 16.)
        if points is None or len(points) < 3:
            self.last_panel_diagnostic = 'Front LiDAR distance has insufficient returns'
            return None
        clearance = min(p.x for p in points) - front_x
        self.last_front_clearance = clearance
        self.last_panel_diagnostic = (
            f'AprilTag heading={math.degrees(heading):+.2f}deg; '
            f'front LiDAR clearance={clearance:.3f}m (minimum {margin:.3f}m)')
        if not math.isfinite(clearance) or clearance < margin:
            return None
        return heading
