"""One continuous AprilTag approach to the station's virtual center pose."""

from collections import deque
import math
import statistics
import time

from docking.control import front_axle_goal_errors, spin_for
from docking.lidar_geometry import normalize_angle
from rclpy.time import Time
from tf2_ros import TransformException


class TagCenterlineApproach:
    @staticmethod
    def declare_parameters(node):
        node.declare_parameter('tag_centerline_speed', .08)
        node.declare_parameter('tag_centerline_angular_speed', .15)
        node.declare_parameter('tag_centerline_max_travel', 2.0)
        node.declare_parameter('tag_centerline_timeout_sec', 60.0)

    def __init__(self, node):
        self.node = node
        self.centerer = node.tag_centering
        self.observations = deque(maxlen=5)
        self.station = None
        self.filtered_at = None
        self.pair = (-1, -1)
        self.sequence = 0
        self.target = None
        self.reason = 'waiting for three distinct tag/scan pairs'

    def observation(self, standoff):
        """Filter the remembered station in odom at each camera acquisition time."""
        node, motion = self.node, self.node.motion
        scan = self.centerer.lidar.snapshot()
        if scan is None:
            self.reason = 'tag or front scan missing/stale, or wrong tag frame'
            self.observations.clear()
            return None
        scan_stamp = scan.scan.header.stamp.sec*1_000_000_000+scan.scan.header.stamp.nanosec
        # Scan headers precede delivery by about one scan period. A newly
        # delivered camera frame can also precede its odom TF. Pair against a
        # short history instead of discarding valid older camera observations.
        history = getattr(node, 'dock_pose_history', ()) or [
            (node.last_dock_pose, node.last_dock_pose_received_at)]
        candidates = []
        for tag, received_at in history:
            if (not node._pose_is_fresh(tag, received_at, .30)
                    or tag.header.frame_id.lstrip('/') != str(node.get_parameter('base_frame').value).lstrip('/')):
                continue
            stamp = tag.header.stamp.sec*1_000_000_000+tag.header.stamp.nanosec
            if stamp >= self.pair[1]:
                candidates.append((abs(stamp-scan_stamp), -stamp, tag))
        if not candidates:
            self.reason = 'tag missing/stale, wrong frame, or acquisition time regressed'
            self.observations.clear()
            return None
        candidates.sort(key=lambda p: p[:2])
        if candidates[0][0] > 150_000_000:
            self.reason = 'tag/front scan differ by more than 150ms'
            return None
        transform = None
        for difference, negative_stamp, tag in candidates:
            if difference > 150_000_000:
                break
            try:
                transform = node.tf_buffer.lookup_transform(
                    str(node.get_parameter('fixed_frame').value), tag.header.frame_id,
                    Time.from_msg(tag.header.stamp)).transform
                stamp = -negative_stamp
                break
            except TransformException:
                continue
        if transform is None:
            self.reason = 'odom transform unavailable at camera acquisition time'
            self.observations.clear()
            return None
        normal = self.centerer.panel(scan, tag)
        if normal is None:
            self.reason = getattr(self.centerer, 'last_panel_diagnostic', 'invalid front geometry')
            self.observations.clear()
            return None
        # Tag center and panel normal were acquired at different instants.
        # Keep each in its own acquisition-time base frame before moving
        # them to odom; a 150ms pair gap during a turn can exceed one degree.
        normal_transform = transform
        if bool(node.get_parameter('tag_front_use_lidar_panel_heading').value):
            try:
                normal_transform = node.tf_buffer.lookup_transform(
                    str(node.get_parameter('fixed_frame').value), tag.header.frame_id,
                    Time.from_msg(scan.scan.header.stamp)).transform
            except TransformException:
                self.reason = 'odom transform unavailable at front scan acquisition time'
                return None
        new_pair = scan.sequence > self.pair[0] and stamp > self.pair[1]
        if new_pair:
            p, q = transform.translation, transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            c, s = math.cos(yaw), math.sin(yaw)
            nq = normal_transform.rotation
            normal_yaw = math.atan2(2*(nq.w*nq.z+nq.x*nq.y), 1-2*(nq.y*nq.y+nq.z*nq.z))
            world = (p.x+c*tag.pose.position.x-s*tag.pose.position.y,
                     p.y+s*tag.pose.position.x+c*tag.pose.position.y, normal_yaw+normal)
            if not all(math.isfinite(v) for v in world):
                self.reason = 'nonfinite station transform'
                return None
            self.pair = (scan.sequence, stamp)
            self.observations.append((time.monotonic(), *world))
            self.sequence += 1
        while self.observations and time.monotonic()-self.observations[0][0] > .5:
            self.observations.popleft()
        if len(self.observations) < 3:
            self.reason = 'waiting for three distinct fresh tag/scan pairs'
            return None
        offset = self.centerer.lateral_offset()
        if offset is None:
            self.reason = 'invalid station lateral calibration'
            return None
        if new_pair:
            reference = self.station[2] if self.station else self.observations[0][3]
            measured = (statistics.median(p[1] for p in self.observations),
                        statistics.median(p[2] for p in self.observations),
                        reference+statistics.median(normalize_angle(p[3]-reference)
                                                    for p in self.observations))
            if self.station is None:
                if (not motion.is_stationary(.005)
                        or abs(motion.current_odom_yaw_rate()) > math.radians(.5)):
                    self.observations.clear()
                    self.reason = 'waiting for stationary tag centerline acquisition'
                    return None
                self.station = measured
                node.get_logger().info('Acquired timestamp-aligned tag centerline; tracking filtered station pose')
            else:
                # One station estimate, continuously refined as the tag grows
                # in view. A single far-field normal is never permanent.
                dt = min(.2, max(0., (stamp-self.filtered_at)/1e9))
                alpha = 1-math.exp(-dt/.7)
                px, py, yaw = self.station
                self.station = (px+alpha*(measured[0]-px), py+alpha*(measured[1]-py),
                                yaw+alpha*normalize_angle(measured[2]-yaw))
            self.filtered_at = stamp
            px, py, yaw = self.station
            c, s = math.cos(yaw), math.sin(yaw)
            self.target = (px-standoff*c+offset*s, py-standoff*s-offset*c, yaw)
        if self.target is None:
            return None
        tx, ty, yaw = self.target
        x, y = motion.current_xy()
        c, s = math.cos(yaw), math.sin(yaw)
        result = (c*(tx-x)+s*(ty-y), -s*(tx-x)+c*(ty-y),
                  normalize_angle(yaw-motion.current_yaw()))
        return result if all(math.isfinite(v) for v in result) else None

    @staticmethod
    def command(errors, axle_x, speed, angular_speed, position_tolerance, yaw_tolerance):
        x, y, heading = errors
        ax, ay = front_axle_goal_errors(x, y, heading, axle_x)
        # A final turn is about the driven axle, not base_link. Once that
        # axle is at its goal, restore heading so the base also reaches goal.
        if abs(ax) <= position_tolerance[0] and abs(ay) <= position_tolerance[1]:
            # Axle and body tolerances differ by axle_x*sin(heading). Even a
            # heading inside yaw tolerance can leave the body outside its
            # position tolerance. Finish that turn instead of publishing zero
            # forever while the completion check still rejects the body pose.
            if (abs(heading) <= yaw_tolerance
                    and abs(x) <= position_tolerance[0]
                    and abs(y) <= position_tolerance[1]):
                return 0., 0.
            return 0., math.copysign(min(angular_speed, max(.025, abs(heading))), heading)
        # Once on the centerline near the turn point, settle its heading
        # before closing the remaining forward distance. Chasing the bearing
        # of a point almost beside the axle turns away from the station normal
        # and fights the camera view guard (recorded trial 69).
        if 0. <= ax <= .20 and abs(ay) <= position_tolerance[1]:
            angular = max(-angular_speed, min(angular_speed, heading))
            if abs(heading) > yaw_tolerance:
                return 0., math.copysign(min(angular_speed, max(.025, abs(heading))), heading)
            return min(speed, .7*max(x, 0.)), angular
        if ax <= .35:
            # Near the turn point, its bearing becomes singular as forward
            # distance vanishes. Trial86 kept turning toward that side point
            # even after passing the panel normal. Follow the center LINE
            # here. Scale the lookahead with the remaining runway instead of
            # saturating at 15deg for almost every lateral error. Trial92
            # exhausted its runway with a residual axle error and then held
            # -15deg at zero speed. Use up to 25deg while there is room, and
            # smoothly return to the original 15deg limit near the turn point.
            heading_limit = math.radians(15+10*min(max(ax/.20, 0.), 1.))
            correction = max(-heading_limit, min(
                heading_limit, math.atan2(ay, max(.05, .5*ax))))
            direction = heading+correction
            angular = max(-angular_speed, min(
                angular_speed, 1.2*direction))
            # Spend only half the existing longitudinal tolerance; never
            # chase a lateral error by driving arbitrarily past the goal.
            linear = min(speed, .7*max(x+.5*position_tolerance[0], 0.))
            linear *= max(0., math.cos(heading))
            # Preserve the remaining centering runway while turning onto
            # the line. Straight, aligned motion keeps its usual speed.
            linear *= max(0., 1.-abs(direction)/math.radians(10))
            if abs(heading) > math.radians(30):
                linear = 0.
            return linear, angular
        c, s = math.cos(heading), math.sin(heading)
        bx, by = c*ax-s*ay, s*ax+c*ay
        alpha = math.atan2(by, bx)
        beta = normalize_angle(heading-alpha)
        # Pose tracking: aim at the center point while smoothly restoring the
        # station-normal heading. Only the final few centimeters slow down.
        linear = min(speed, .7*math.hypot(ax, ay))*max(0., math.cos(alpha))
        if abs(alpha) > math.radians(40):
            linear = 0.
        angular_limit = min(angular_speed, max(.035, .7*math.hypot(ax, ay)))
        angular = max(-angular_limit, min(angular_limit, 1.2*alpha-.6*beta))
        return linear, angular

    def run(self):
        node, motion = self.node, self.node.motion
        if self.centerer is None:
            motion.stop_robot()
            node.get_logger().error('Simple centerline approach requires AprilTag front observations')
            return False
        def parameter(name):
            return float(node.get_parameter(name).value)
        speed, angular_speed = parameter('tag_centerline_speed'), parameter('tag_centerline_angular_speed')
        max_travel, timeout = parameter('tag_centerline_max_travel'), parameter('tag_centerline_timeout_sec')
        standoff, axle_x = parameter('apriltag_front_standoff'), parameter('apriltag_front_axle_x')
        lateral_offset = self.centerer.lateral_offset()
        position_tolerance = (parameter('tag_refinement_longitudinal_tolerance'),
                              parameter('tag_refinement_lateral_tolerance'))
        yaw_tolerance = parameter('tag_refinement_yaw_tolerance')
        wait_budget = parameter('tag_front_observation_recovery_timeout_sec')
        values = (speed, angular_speed, max_travel, timeout, standoff, axle_x,
                  *position_tolerance, yaw_tolerance, wait_budget)
        if not (lateral_offset is not None and all(math.isfinite(v) for v in values) and 0 < speed <= .10
                and 0 < angular_speed <= .15 and 0 < max_travel <= 3
                and 0 < timeout <= 90 and .95 <= standoff <= 1.5 and 0 <= axle_x <= .20
                and all(0 < v <= .03 for v in position_tolerance)
                and 0 < yaw_tolerance <= math.radians(3) and 0 < wait_budget <= 8):
            motion.stop_robot()
            node.get_logger().error('Invalid simple centerline approach parameters')
            return False
        started = time.monotonic()
        waiting_since, waited = started, 0.
        motion_stamps = (motion.last_odom_stamp_nanoseconds, motion.last_imu_stamp_nanoseconds,
                         motion.last_wheel_yaw_stamp_nanoseconds)
        previous_xy, travel = motion.current_xy(), 0.
        previous_sequence, stable = -1, 0
        progress_at, best_distance, best_heading = started, math.inf, math.inf
        last_log = -math.inf
        node.get_logger().info(
            f'Simple tag centerline approach: standoff={standoff:.2f}m, '
            f'lateral_offset={lateral_offset:+.3f}m, speed<={speed:.2f}m/s, '
            f'heading_source={"front_panel" if node.get_parameter("tag_front_use_lidar_panel_heading").value else "tag"}')
        motion.stop_robot()
        try:
            while not node.should_stop() and time.monotonic()-started < timeout:
                spin_for(node, .05, node.should_stop)
                if node.should_stop():
                    break
                now = time.monotonic()
                xy = motion.current_xy()
                travel += math.dist(xy, previous_xy)
                previous_xy = xy
                if travel > max_travel:
                    self.reason = f'approach travel exceeded {max_travel:.2f}m'
                    break
                feedback = (motion.odom_is_fresh(), motion.imu_is_fresh(),
                            motion.wheel_yaw_is_fresh())
                fresh = all(feedback)
                if not fresh:
                    # Use the same bounded zero-command observation wait as
                    # tag/scan delivery gaps. A single stale check must not
                    # terminate an otherwise valid approach. Resumption below
                    # still requires all three fresh, advancing motion stamps,
                    # stationary chassis and three new camera/scan pairs.
                    epoch = node.get_clock().now().nanoseconds
                    status = []
                    for name, ok, receipt, stamp in zip(
                            ('odom', 'imu', 'wheel'), feedback,
                            (motion.last_odom_received_at, motion.last_imu_received_at,
                             motion.last_wheel_yaw_received_at),
                            (motion.last_odom_stamp_nanoseconds, motion.last_imu_stamp_nanoseconds,
                             motion.last_wheel_yaw_stamp_nanoseconds)):
                        status.append(f'{name}_fresh={ok}, receipt_age={now-receipt:.3f}s, '
                                      f'header_age={(epoch-stamp)/1e9:.3f}s')
                    self.reason = 'motion feedback unavailable: '+'; '.join(status)
                errors = self.observation(standoff) if fresh else None
                stationary = (motion.is_stationary(.005)
                              and abs(motion.current_odom_yaw_rate()) <= math.radians(.5))
                if errors is None or waiting_since is not None:
                    motion.cmd_vel_pub.publish(motion.twist())
                    stable = 0
                    if waiting_since is None:
                        waiting_since = now
                        self.observations.clear()
                        motion_stamps = (motion.last_odom_stamp_nanoseconds,
                                         motion.last_imu_stamp_nanoseconds, motion.last_wheel_yaw_stamp_nanoseconds)
                        node.get_logger().warning('Centerline approach holding zero: '+self.reason)
                    if waited+now-waiting_since >= wait_budget:
                        self.reason = 'stopped observation wait exhausted: '+self.reason
                        break
                    new_motion = all(new > old for new, old in zip((
                        motion.last_odom_stamp_nanoseconds, motion.last_imu_stamp_nanoseconds,
                        motion.last_wheel_yaw_stamp_nanoseconds), motion_stamps))
                    if errors is None or not stationary or not new_motion:
                        continue
                    waited += now-waiting_since
                    waiting_since = None
                    progress_at = now
                    node.get_logger().info(f'Centerline observations ready; cumulative stopped wait={waited:.2f}s')
                    continue
                x, y, heading = errors
                if x < -position_tolerance[0] or math.hypot(x, y) > max_travel+.05:
                    self.reason = f'virtual center pose outside forward approach: x={x:+.3f}, y={y:+.3f}m'
                    break
                distance = math.hypot(x, y)
                if distance < best_distance-.01:
                    best_distance, best_heading, progress_at = distance, abs(heading), now
                elif abs(heading) < best_heading-math.radians(.5):
                    best_heading, progress_at = abs(heading), now
                if now-progress_at > 6:
                    self.reason = 'tag position/heading did not converge for 6s'
                    break
                linear, angular = self.command(errors, axle_x, speed, angular_speed,
                                               position_tolerance, yaw_tolerance)
                limited = self.centerer.view_guard.limit(node.last_dock_pose, linear, angular,
                                                        axle_x, max_angular_speed=angular_speed)
                if limited is None:
                    self.reason = 'tag visibility: '+self.centerer.view_guard.last_reason
                    break
                linear, angular = limited
                # Preserve the station's established turn distance using the
                # latest observed tag center as well as the filtered goal.
                px, py = self.observations[-1][1:3]
                cx, cy = motion.current_xy()
                yaw = self.station[2]
                observed_distance = math.cos(yaw)*(px-cx)+math.sin(yaw)*(py-cy)
                settled = (abs(x) <= position_tolerance[0] and abs(y) <= position_tolerance[1]
                           and abs(heading) <= yaw_tolerance and stationary
                           and observed_distance >= standoff-position_tolerance[0])
                if self.sequence != previous_sequence:
                    stable = stable+1 if settled else 0
                    previous_sequence = self.sequence
                if stable >= 3:
                    node.get_logger().info(
                        f'Centerline approach complete: x={x:+.3f}m, y={y:+.3f}m, '
                        f'heading={math.degrees(heading):+.2f}deg, '
                        f'tag_centerline_y={y+self.centerer.lateral_offset():+.3f}m, '
                        f'travel={travel:.3f}m, elapsed={now-started:.1f}s')
                    return True
                motion.cmd_vel_pub.publish(motion.twist(linear_x=linear, angular_z=angular))
                if now-last_log >= 1:
                    node.get_logger().info(
                        f'Centerline: x={x:+.3f}m, y={y:+.3f}m, heading={math.degrees(heading):+.2f}deg, '
                        f'cmd=({linear:.3f}, {angular:+.3f}), elapsed={now-started:.1f}s')
                    last_log = now
            node.get_logger().error('Centerline approach stopped: '+(
                'cancelled or timed out' if node.should_stop() or time.monotonic()-started >= timeout else self.reason))
            return False
        finally:
            motion.stop_robot()
