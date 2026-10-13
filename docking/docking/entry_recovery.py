"""One bounded forward reposition before an unsafe reverse entry.

The entry margin is unchanged. Repositioning first retreats straight from the
rails, then follows a small dogleg in observed free space. It never reverses
until the original entry checks can pass again.
"""

import math
import statistics
import time

from docking.control import BackupStationaryYawController, spin_for
from docking.docking_lidar import DockingLidar
from docking.lidar_geometry import normalize_angle
from docking.tag_centering import _FrontLidarParameters


HALF_LENGTH = 0.2295
HALF_WIDTH = 0.22
MARGIN = 0.02


def rectangle_clearance(x, y, pose):
    """Signed clearance to the rectangle, using its local axis-aligned norm."""
    px, py, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = x - px, y - py
    u, v = c * dx + s * dy, -s * dx + c * dy
    return max(abs(u) - HALF_LENGTH, abs(v) - HALF_WIDTH)


def reposition_plan(offset):
    """At most 28.1 cm forward travel and 6 degrees of yaw per turn."""
    if not math.isfinite(offset) or not 0.02 <= abs(offset) <= 0.045:
        return None
    if abs(offset) > .035:
        lateral = math.copysign(min(abs(offset) - .025, .018), offset)
        yaw = math.atan2(lateral, .18)
        return [(.10, 0., 0.), (.10, 0., yaw),
                (.28, lateral, yaw), (.28, lateral, 0.)]
    lateral = math.copysign(min(abs(offset) - 0.01, 0.012), offset)
    yaw = math.atan2(lateral, 0.115)
    return [
        (0.10, 0.0, 0.0),
        (0.10, 0.0, yaw),
        (0.215, lateral, yaw),
        (0.215, lateral, 0.0),
    ]


def interpolate_path(start, goals):
    """Sample translation every centimeter and rotation every degree."""
    path = []
    previous = start
    for goal in goals:
        delta = normalize_angle(goal[2] - previous[2])
        steps = max(
            1,
            math.ceil(math.hypot(goal[0] - previous[0], goal[1] - previous[1]) / 0.01),
            math.ceil(abs(delta) / math.radians(1)),
        )
        for index in range(1, steps + 1):
            fraction = index / steps
            path.append((
                previous[0] + fraction * (goal[0] - previous[0]),
                previous[1] + fraction * (goal[1] - previous[1]),
                previous[2] + fraction * delta,
            ))
        previous = goal
    return path


def path_is_clear(points, path, escape=False):
    """During straight retreat, existing small gaps may only increase.

    Rotation/diagonal travel always requires the full original 2 cm margin.
    """
    for x, y in points:
        initial = rectangle_clearance(x, y, (0.0, 0.0, 0.0))
        if initial <= 0:
            return False
        required = min(initial, MARGIN) if escape else MARGIN
        if any(rectangle_clearance(x, y, pose) < required - 1e-6 for pose in path):
            return False
    return True


class DockEntryRecovery:
    @staticmethod
    def declare_parameters(node):
        node.declare_parameter('use_dock_entry_recovery', True)

    def __init__(self, node):
        self.node = node
        self.motion = node.motion
        centerer = getattr(node, 'tag_centering', None)
        self.front = (centerer.lidar if centerer is not None else
                      DockingLidar(_FrontLidarParameters(node), node.tf_buffer))
        self.last_path_error = ''

    def observations(self):
        pairs = []
        deadline = time.monotonic() + 2.0
        sequence = -1
        seen = 0
        rejected = 0
        spreads = None
        while not self.node.should_stop() and time.monotonic() < deadline:
            spin_for(self.node, 0.03, self.node.should_stop)
            scan = self.node.lidar.snapshot()
            if scan is None or scan.sequence == sequence:
                continue
            sequence = scan.sequence
            seen += 1
            wall = self.node.lidar_aligner.estimate_error(scan)
            guide = self.node.lidar_aligner.estimate_guide_center(scan)
            if (wall is None or guide is None
                    or abs(wall[0]) > math.radians(2)
                    or wall[2] / max(wall[3], 1) < 0.75 or wall[4] < 0.30
                    or not self.motion.is_stationary(0.005)
                    or abs(self.motion.current_odom_yaw_rate()) > math.radians(0.5)):
                pairs = []
                rejected += 1
                continue
            pairs.append(guide)
            # Sliding windows preserve four recent valid samples after one
            # noisy fit, rather than restarting disjoint batches of five.
            pairs = pairs[-5:]
            if len(pairs) >= 5:
                offsets = [guide.center_offset for guide in pairs]
                widths = [guide.separation for guide in pairs]
                spreads = (max(offsets)-min(offsets), max(widths)-min(widths))
                if spreads[0] <= 0.01 and spreads[1] <= 0.02:
                    return statistics.median(offsets), statistics.median(widths)
        self.node.get_logger().warning(
            'Entry guide verification incomplete while stopped: '
            f'unique_scans={seen}, rejected_geometry_or_motion={rejected}, '
            f'recent_valid={len(pairs)}, last_center_width_spread={spreads}')
        return None

    def _sensors(self):
        sensors = []
        for lidar in (self.front, self.node.lidar):
            snapshot = lidar.snapshot()
            if snapshot is None:
                return None
            sensors.append((lidar, snapshot))
        stamps = [snapshot.scan.header.stamp.sec + snapshot.scan.header.stamp.nanosec / 1e9
                  for _, snapshot in sensors]
        return sensors if abs(stamps[0] - stamps[1]) <= 0.15 else None

    @staticmethod
    def _visible(x, y, sensors, diagnostics=None):
        for lidar, snapshot in sensors:
            transform = lidar.transform
            dx, dy = x - transform.x, y - transform.y
            angle = normalize_angle(math.atan2(dy, dx) - transform.yaw)
            scan = snapshot.scan
            index = round((angle - scan.angle_min) / scan.angle_increment)
            distance = math.hypot(dx, dy)
            circular = abs(scan.angle_increment) * (len(scan.ranges) - 1) >= 2 * math.pi - 0.02

            def reading(beam):
                if circular:
                    beam %= len(scan.ranges)
                if not 0 <= beam < len(scan.ranges):
                    return None
                value = scan.ranges[beam]
                if math.isfinite(value) and max(scan.range_min, 0.05) <= value <= scan.range_max:
                    return value
                return None

            center = reading(index)
            detail = {
                'frame': getattr(scan.header, 'frame_id', '') if hasattr(scan, 'header') else '',
                'beam': index, 'distance': round(distance, 4), 'center': center,
            }
            if center is not None and center < distance + 0.015:
                if diagnostics is not None:
                    diagnostics.append({**detail, 'reason': 'near_return_or_occlusion'})
                continue
            brackets = []
            for direction in (-1, 1):
                for step in range(1, 13):
                    value = reading(index + direction * step)
                    if value is not None:
                        brackets.append((step, value))
                        break
            # Bridge only a narrow angular hole whose width at the footprint
            # is <15 mm, bounded by actual clear returns on BOTH sides. The
            # original 20 mm obstacle inflation remains in path_is_clear().
            gap_width = sum(step for step, _ in brackets) * abs(scan.angle_increment) * distance
            if (len(brackets) == 2 and gap_width <= 0.015
                    and all(value >= distance + 0.03 for _, value in brackets)):
                return True
            if diagnostics is not None:
                reason = ('missing_neighbor_returns' if len(brackets) != 2 else
                          'angular_gap_too_wide' if gap_width > 0.015 else
                          'neighbor_return_too_near')
                diagnostics.append({**detail, 'reason': reason,
                                    'neighbors': brackets, 'gap_width': round(gap_width, 4)})
        return False

    def clear(self, path, escape):
        sensors = self._sensors()
        if sensors is None:
            self.last_path_error = 'front/rear scans are stale or differ by more than 150 ms'
            return False
        points = []
        for index, (lidar, snapshot) in enumerate(sensors):
            # Outward-facing half scans avoid looking through the robot itself.
            projected = lidar.project(
                snapshot, 0 if index == 0 else math.pi, math.pi, 0.05, 2.0)
            if projected is None:
                self.last_path_error = 'scan projection unavailable'
                return False
            points.extend((point.x, point.y) for point in projected)
        if not path_is_clear(points, path, escape):
            self.last_path_error = 'observed obstacle intersects the swept footprint or margin'
            return False
        # Cover the boundary of each new occupied rectangle with ray checks.
        for px, py, yaw in path:
            c, s = math.cos(yaw), math.sin(yaw)
            boundary = []
            for side in (-1, 1):
                for index in range(25):
                    boundary.append((
                        -HALF_LENGTH + index * (2 * HALF_LENGTH) / 24,
                        side * (HALF_WIDTH + MARGIN),
                    ))
                    boundary.append((
                        side * (HALF_LENGTH + MARGIN),
                        -HALF_WIDTH + index * (2 * HALF_WIDTH) / 24,
                    ))
            for u, v in boundary:
                x, y = px + c * u - s * v, py + s * u + c * v
                if rectangle_clearance(x, y, (0, 0, 0)) > MARGIN + 1e-6:
                    diagnostics = []
                    if not self._visible(x, y, sensors, diagnostics):
                        self.last_path_error = (
                            f'unobserved footprint boundary at base x={x:.3f}, y={y:.3f}m; '
                            f'ray_checks={diagnostics}')
                        return False
        self.last_path_error = ''
        return True

    def _fail(self, message):
        self.node.get_logger().error('Entry recovery stopped: ' + message)
        return False

    def _side_room(self, offset, observed_width):
        """Use the same measured station width policy as reverse entry."""
        known = float(self.node.get_parameter('backup_guide_known_width').value)
        uncertainty = float(self.node.get_parameter('backup_guide_known_width_uncertainty').value)
        half_width = float(self.node.get_parameter('backup_rear_half_width').value)
        if (not all(math.isfinite(v) for v in
                    (known, uncertainty, offset, observed_width, half_width))
                or (known != 0 and not .50 <= known <= .58)
                or not .005 <= uncertainty <= .02
                or observed_width <= 0 or half_width <= 0
                or (known != 0 and not .50 <= observed_width <= .58)):
            raise ValueError('invalid measured guide width profile or guide observation')
        supported_width = known - uncertainty if known else observed_width
        return supported_width, supported_width / 2 - abs(offset) - half_width

    def run(self):
        if not all(bool(self.node.get_parameter(name).value) for name in (
                'use_dock_entry_recovery', 'use_lidar_backup', 'use_lidar_guide_centering')):
            return True
        observed = self.observations()
        if observed is None:
            self.node.get_logger().info(
                'Entry recovery: stationary guide pair not verified; original backup acquisition required')
            return True  # Original backup acquisition still must pass.
        offset, width = observed
        try:
            supported_width, room = self._side_room(offset, width)
        except ValueError as exc:
            return self._fail(str(exc))
        margin = float(self.node.get_parameter('backup_rear_safety_margin').value)
        self.node.get_logger().info(
            f'Entry recovery assessment: center={offset:+.3f}m, observed_width={width:.3f}m, '
            f'supported_width={supported_width:.3f}m, side_room={room:.3f}m, '
            f'required_margin={margin:.3f}m, reposition_required={room < margin}')
        if room >= margin:
            return True
        plan = reposition_plan(offset)
        scan = self.node.lidar.snapshot()
        clearance = self.motion._rear_clearances(scan) if scan else None
        max_travel = float(self.node.get_parameter('backup_max_travel').value)
        target = float(self.node.get_parameter('backup_target_rear_clearance').value)
        rear_clearance = min(distance for _, distance in clearance) if clearance else math.nan
        forward_distance = plan[-1][0] if plan is not None else math.nan
        required_travel = rear_clearance + forward_distance + .002 - target
        # The complete retreat must fit the configured backup distance. This
        # planner is for the verified 459 x 440 mm rectangular footprint.
        if (plan is None or room < 0.005 or not clearance
                or required_travel > max_travel - 0.01
                or abs(float(self.node.get_parameter('backup_rear_half_width').value) - HALF_WIDTH) > 1e-6
                or abs(float(self.node.get_parameter('backup_rear_reference_x').value) + HALF_LENGTH) > 1e-6
                or abs(margin - MARGIN) > 1e-6):
            return self._fail(
                'offset, clearance, footprint or remaining travel is outside bounds: '
                f'center={offset:+.3f}m, guide_width={width:.3f}m, '
                f'supported_width={supported_width:.3f}m, '
                f'side_room={room:.3f}m (min 0.005m for recovery), '
                f'offset_plan_valid={plan is not None}, rear_clearance={rear_clearance:.3f}m, '
                f'required_backup={required_travel:.3f}m, allowed_backup={max_travel - 0.01:.3f}m')
        if not self.clear(interpolate_path((0, 0, 0), plan[:1]), True):
            return self._fail('forward escape path: ' + getattr(self, 'last_path_error', 'unverified'))
        if not self.clear(interpolate_path(plan[0], plan[1:]), False):
            return self._fail('rotation/lateral path: ' + getattr(self, 'last_path_error', 'unverified'))
        self.node.get_logger().warning(
            f'Bounded entry recovery: center={offset:+.3f}m, side_room={room:.3f}m; '
            f'forward={forward_distance:.3f}m, lateral_target={plan[-1][1]:+.3f}m; reverse remains inhibited')
        start_x, start_y = self.motion.current_xy()
        start_yaw = self.motion.current_yaw()
        c, s = math.cos(start_yaw), math.sin(start_yaw)
        deadline = time.monotonic() + 25
        previous_xy = (start_x, start_y)
        travel = 0.0
        try:
            for index, goal in enumerate(plan):
                gx = start_x + c * goal[0] - s * goal[1]
                gy = start_y + s * goal[0] + c * goal[1]
                goal_yaw = normalize_angle(start_yaw + goal[2])
                rotation = index in (1, 3)
                controller = BackupStationaryYawController()
                stable = 0
                previous_odom = -1
                while not self.node.should_stop() and time.monotonic() < deadline:
                    if not (self.motion.odom_is_fresh() and self.motion.imu_is_fresh()
                            and self.motion.wheel_yaw_is_fresh()):
                        return self._fail('motion feedback became stale')
                    x, y = self.motion.current_xy()
                    travel += math.hypot(x - previous_xy[0], y - previous_xy[1])
                    previous_xy = (x, y)
                    travel_limit = forward_distance + .025
                    if travel > travel_limit:
                        return self._fail(
                            f'cumulative travel exceeded {travel_limit:.3f}m')
                    yaw = self.motion.current_yaw()
                    cy, sy = math.cos(yaw), math.sin(yaw)
                    dx, dy = gx - x, gy - y
                    bx, by = cy * dx + sy * dy, -sy * dx + cy * dy
                    error = normalize_angle(goal_yaw - yaw)
                    distance = math.hypot(dx, dy)
                    reached = abs(error) <= math.radians(0.7) if rotation else distance <= 0.005
                    if reached:
                        self.motion.stop_robot()
                        if self.motion.odom_sequence != previous_odom:
                            stationary = (self.motion.is_stationary(0.005)
                                          and abs(self.motion.current_odom_yaw_rate()) <= math.radians(0.5))
                            stable = stable + 1 if stationary else 0
                            previous_odom = self.motion.odom_sequence
                        if stable >= 3:
                            break
                    else:
                        destination = (0.0, 0.0, error) if rotation else (bx, by, error)
                        path = interpolate_path((0, 0, 0), [destination])
                        if not self.clear(path, index == 0):
                            return self._fail(getattr(self, 'last_path_error', 'path is unverified'))
                        if rotation:
                            command = controller.update(
                                error, self.motion.current_integrated_imu_yaw(),
                                self.motion.current_wheel_yaw(), time.monotonic(), math.radians(0.7))
                            if controller.fault:
                                return self._fail(controller.fault)
                            self.motion.cmd_vel_pub.publish(self.motion.twist(angular_z=command))
                        else:
                            if bx < 0 or abs(error) > math.radians(2):
                                return self._fail('translation overshot or heading exceeded 2 degrees')
                            self.motion.cmd_vel_pub.publish(self.motion.twist(
                                linear_x=min(0.035, max(0.012, 0.7 * distance)),
                                angular_z=max(-0.015, min(0.015, 1.2 * error))))
                    spin_for(self.node, 0.05, self.node.should_stop)
                else:
                    return self._fail('interrupted or exceeded the 25-second motion limit')
            observed = self.observations()
            if observed is None:
                return self._fail('guide center could not be verified after repositioning')
            after, new_width = observed
            try:
                new_supported_width, new_room = self._side_room(after, new_width)
            except ValueError as exc:
                return self._fail(str(exc))
            if (abs(after) > abs(offset) - 0.005
                    or new_room < margin):
                return self._fail(
                    'original side clearance or lateral progress was not established: '
                    f'center={after:+.3f}m, observed_width={new_width:.3f}m, '
                    f'supported_width={new_supported_width:.3f}m, side_room={new_room:.3f}m')
            self.node.get_logger().info(
                f'Entry recovery complete: center={after:+.3f}m; original backup checks resume')
            return True
        finally:
            self.motion.stop_robot()
