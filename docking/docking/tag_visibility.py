"""Predict the entire tag's image bounds before issuing a front command."""

import math
import time

import numpy as np
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from tf2_ros import TransformException


def rotation(q):
    values = np.array([q.x, q.y, q.z, q.w], dtype=float)
    norm = np.linalg.norm(values)
    if not np.all(np.isfinite(values)) or norm < 1e-6:
        raise ValueError('invalid orientation')
    x, y, z, w = values / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def image_bounds(tag, camera_transform, info, size, linear=0., angular=0., dt=0., axle_x=0.):
    """Project tag corners after an axle-centered constant-velocity command."""
    if not all(math.isfinite(v) for v in (size, linear, angular, dt, axle_x)):
        raise ValueError('nonfinite view prediction')
    if size <= 0 or dt < 0 or not 0 <= axle_x <= .20:
        raise ValueError('invalid view prediction geometry')
    p = tag.pose.position
    center = np.array([p.x, p.y, p.z])
    h = size/2
    corners = np.array([[-h, -h, 0.], [-h, h, 0.], [h, h, 0.], [h, -h, 0.]])
    points = corners @ rotation(tag.pose.orientation).T + center
    angle = angular*dt
    c, s = math.cos(angle), math.sin(angle)
    dx = linear*dt if abs(angular) < 1e-8 else linear*math.sin(angle)/angular
    dy = 0. if abs(angular) < 1e-8 else linear*(1-math.cos(angle))/angular
    points[:, :2] -= [axle_x+dx, dy]
    points[:, :2] = points[:, :2] @ np.array([[c, -s], [s, c]])
    points[:, 0] += axle_x
    t = camera_transform.translation
    optical = (points - [t.x, t.y, t.z]) @ rotation(camera_transform.rotation)
    if not np.all(np.isfinite(optical)) or min(optical[:, 2]) <= .05:
        raise ValueError('tag is behind or too close to the camera')
    projection = np.array(info.p).reshape(3, 4)
    pixels = np.column_stack((optical, np.ones(4))) @ projection.T
    pixels = pixels[:, :2]/pixels[:, 2:3]
    return (float(min(pixels[:, 0])), float(max(pixels[:, 0])),
            float(min(pixels[:, 1])), float(max(pixels[:, 1])))


class TagVisibilityGuard:
    def __init__(self, node):
        self.node = node
        self.info = None
        self.received_at = 0.
        self.last_reason = 'no rectified CameraInfo received'
        self.last_limited = False
        self.last_log_at = -math.inf
        self.opposed_yaw_since = None
        self.opposed_yaw_seen_at = None
        self.recovering_view = False
        self.subscription = node.create_subscription(
            CameraInfo, str(node.get_parameter('apriltag_view_camera_info_topic').value),
            self._callback, 5)

    @staticmethod
    def declare_parameters(node):
        node.declare_parameter('apriltag_view_camera_info_topic', '/apriltag/camera_info_rect')
        node.declare_parameter('apriltag_view_tag_size', .154)
        node.declare_parameter('apriltag_view_margin_fraction', .10)
        node.declare_parameter('apriltag_view_prediction_sec', 1.0)

    def _callback(self, info):
        if (info.width <= 0 or info.height <= 0 or not info.header.frame_id
                or info.p[0] <= 0 or info.p[5] <= 0
                or abs(info.p[10]-1.) > 1e-6
                or not all(math.isfinite(v) for v in info.p)):
            self.info = None
            self.last_reason = 'invalid rectified CameraInfo'
            return
        self.info, self.received_at = info, time.monotonic()

    def limit(self, tag, linear, angular, axle_x, max_angular_speed=None):
        """Keep requested motion when visible; trim outward yaw before cropping.

        A tag outside the soft margin may move inward, but every predicted
        sample must retain all corners in the actual image. Missing calibration
        or an already cropped tag never authorizes movement.
        """
        self.last_limited = False
        info = self.info
        if info is None or time.monotonic()-self.received_at > .5:
            self.last_reason = 'rectified CameraInfo missing or stale'
            return None
        try:
            size = float(self.node.get_parameter('apriltag_view_tag_size').value)
            fraction = float(self.node.get_parameter('apriltag_view_margin_fraction').value)
            horizon = float(self.node.get_parameter('apriltag_view_prediction_sec').value)
            if (not all(math.isfinite(v) for v in (size, fraction, horizon))
                    or not .04 <= size <= .30 or not .05 <= fraction <= .20
                    or not .5 <= horizon <= 1.5):
                raise ValueError('invalid tag visibility parameters')
            transform = self.node.tf_buffer.lookup_transform(
                str(self.node.get_parameter('base_frame').value), info.header.frame_id, Time()).transform
            initial = image_bounds(tag, transform, info, size)

            def clearance(bounds):
                left, right, top, bottom = bounds
                # A smaller vertical reserve still covers all tag corners.
                return min(left-fraction*info.width, (1-fraction)*info.width-right,
                           top-.03*info.height, .97*info.height-bottom)

            def inside(bounds):
                left, right, top, bottom = bounds
                return left >= 2 and right <= info.width-2 and top >= 2 and bottom <= info.height-2

            if not inside(initial):
                raise ValueError(f'tag corners already cropped: bounds={initial}')
            initial_room = clearance(initial)
            if abs(linear) < 1e-10 and abs(angular) < 1e-10:
                self.opposed_yaw_since = None
                self.last_reason = 'stationary verification; tag corners visible'
                return 0., 0.

            motion = getattr(self.node, 'motion', None)
            observed = math.nan
            if motion is not None and motion.imu_is_fresh():
                observed = motion.current_imu_yaw_rate()
            outward_mismatch = False
            if (math.isfinite(observed)
                    and initial_room < fraction*info.width):
                if (abs(observed) > .025
                        and observed*angular < 0.):
                    actual_bounds = image_bounds(
                        tag, transform, info, size, linear, observed, .25, axle_x)
                    if clearance(actual_bounds) < initial_room-1:
                        outward_mismatch = True
            if outward_mismatch:
                now = time.monotonic()
                started = getattr(self, 'opposed_yaw_since', None)
                seen = getattr(self, 'opposed_yaw_seen_at', None)
                if started is None or seen is None or now-seen > .3:
                    started = now
                self.opposed_yaw_since, self.opposed_yaw_seen_at = started, now
                if now-started >= .2:
                    # Command prediction alone assumes the requested turn
                    # has already taken effect. Trial013's gyro and image
                    # kept turning outward despite an inward wheel command.
                    # Settle while still visible; add no blind recovery turn.
                    self.recovering_view = True
            else:
                self.opposed_yaw_since = None
                self.opposed_yaw_seen_at = None

            def safe(v, w):
                rooms = [initial_room]
                for index in range(1, 11):
                    bounds = image_bounds(tag, transform, info, size, v, w, horizon*index/10, axle_x)
                    if not inside(bounds):
                        return False
                    rooms.append(clearance(bounds))
                # Retain an existing soft-margin deficit only while recovering.
                return (min(rooms) >= 0 if initial_room >= 0 else
                        min(rooms) >= initial_room-1 and rooms[-1] > initial_room+1)

            def recenter_command():
                t = transform.translation
                axis = rotation(transform.rotation)[:, 2]
                bearing = math.atan2(tag.pose.position.y-t.y, tag.pose.position.x-t.x)
                camera_yaw = math.atan2(axis[1], axis[0])
                error = math.atan2(math.sin(bearing-camera_yaw), math.cos(bearing-camera_yaw))
                cap = float(self.node.get_parameter('tag_centerline_angular_speed').value)
                if max_angular_speed is not None:
                    cap = min(cap, max_angular_speed)
                if not math.isfinite(cap) or cap <= 0:
                    raise ValueError('invalid view recovery speed limit')
                return max(-cap, min(cap, .8*error))

            if getattr(self, 'recovering_view', False):
                if not math.isfinite(observed):
                    raise ValueError('fresh IMU required for tag view recovery')
                # A momentarily zero gyro does not mean the tag is safely
                # back in view. Keep translation zero until an extra soft
                # margin is restored; only turn toward the still-visible tag.
                if initial_room >= fraction*info.width:
                    self.recovering_view = False
                else:
                    still_outward = (abs(observed) > .025 and clearance(image_bounds(
                        tag, transform, info, size, 0., observed, .25, axle_x)) < initial_room-1)
                    recovery = recenter_command()
                    turn = (recovery if not still_outward and abs(recovery) > 1e-4
                            and safe(0., recovery) else 0.)
                    self.last_limited = True
                    self.last_reason = ('waiting for measured outward yaw to settle near image edge'
                                        if still_outward else 'restoring tag view with zero linear speed')
                    self.report_limited()
                    return 0., turn

            for factor in (1., .75, .5, .25, 0.):
                if factor == 0 and abs(linear) < .006 and abs(angular) > 1e-4:
                    # A blocked stationary outward turn should restore the
                    # image margin, not remain in a zero-yaw deadlock.
                    continue
                if safe(linear, angular*factor):
                    self.last_limited = factor != 1.
                    self.last_reason = f'bounds={tuple(round(v, 1) for v in initial)}, yaw_scale={factor:.2f}'
                    self.report_limited()
                    return linear, angular*factor
            # Recover visibility by facing the tag rather than chasing a
            # nearby virtual point. No translation accompanies this turn.
            recovery = recenter_command()
            if abs(recovery) > 1e-4 and safe(0., recovery):
                self.last_limited = True
                self.last_reason = f'recentering tag in view; bounds={tuple(round(v, 1) for v in initial)}'
                self.report_limited()
                return 0., recovery
            raise ValueError(f'no visible front command: bounds={initial}')
        except (ValueError, TransformException) as exc:
            self.last_reason = str(exc)
            return None

    def report_limited(self):
        now = time.monotonic()
        if self.last_limited and now-self.last_log_at >= 1.:
            self.node.get_logger().warning('Front tag visibility limited command: ' + self.last_reason)
            self.last_log_at = now
