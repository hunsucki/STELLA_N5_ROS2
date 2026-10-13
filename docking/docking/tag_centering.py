"""AprilTag station center with optional front-panel normal measurement."""

import math

from docking.docking_lidar import DockingLidar
from docking.lidar_alignment import LidarPlaneAligner
from docking.lidar_geometry import plane_normal_alignment_error
from docking.tag_visibility import TagVisibilityGuard

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

class AprilTagCentering:
    @staticmethod
    def declare_parameters(node):
        node.declare_parameter('apriltag_front_standoff', .95)
        node.declare_parameter('apriltag_front_lateral_offset', 0.0)
        node.declare_parameter('apriltag_front_axle_x', .10)
        node.declare_parameter('tag_front_body_reference_x', .2295)
        node.declare_parameter('tag_front_min_lidar_clearance', .05)
        node.declare_parameter('tag_front_use_lidar_panel_heading', False)
        node.declare_parameter('tag_centering_candidate_max_error', math.radians(30))
        node.declare_parameter('tag_centering_lidar_topic', '/scan')
        node.declare_parameter('tag_centering_lidar_frame', 'base_scan')
        node.declare_parameter('tag_refinement_longitudinal_tolerance', .02)
        node.declare_parameter('tag_refinement_lateral_tolerance', .025)
        node.declare_parameter('tag_refinement_yaw_tolerance', math.radians(2))
        node.declare_parameter('tag_front_observation_recovery_timeout_sec', 8.0)
        TagVisibilityGuard.declare_parameters(node)

    def __init__(self, node):
        self.node = node
        self.lidar = DockingLidar(_FrontLidarParameters(node), node.tf_buffer)
        self.panel_fitter = LidarPlaneAligner(_FrontLidarParameters(node), None, self.lidar)
        self.view_guard = TagVisibilityGuard(node)

    def lateral_offset(self):
        """Desired raw tag lateral coordinate at the calibrated turn point.

        Zero retains the geometric tag normal. A fixed station can calibrate
        the repeatable difference observed across front and rear stages.
        This does not change rear-guide geometry or entry acceptance.
        """
        offset = float(self.node.get_parameter('apriltag_front_lateral_offset').value)
        return offset if math.isfinite(offset) and abs(offset) <= .04 else None


    def panel(self, snapshot, tag):
        if self.lateral_offset() is None:
            self.last_panel_diagnostic = 'Invalid AprilTag front lateral offset (limit +/-0.04m)'
            return None
        heading = apriltag_normal_yaw(tag)
        bearing = math.atan2(tag.pose.position.y, tag.pose.position.x)
        if (heading is None or not math.isfinite(bearing)
                or tag.pose.position.x <= 0
                or abs(bearing) > math.radians(30)
                or abs(heading) > math.radians(30)):
            self.last_panel_diagnostic = (
                f'AprilTag face normal or forward view is invalid: heading={heading}, '
                f'bearing={math.degrees(bearing):.2f}deg, tag_x={tag.pose.position.x:.3f}m')
            return None
        front_x = float(self.node.get_parameter('tag_front_body_reference_x').value)
        margin = float(self.node.get_parameter('tag_front_min_lidar_clearance').value)
        if (not all(math.isfinite(v) for v in (front_x, margin))
                or front_x <= 0 or margin <= 0):
            self.last_panel_diagnostic = 'Invalid front LiDAR clearance parameters'
            return None
        points = self.lidar.project(snapshot, 0., math.radians(10), .05, 16.)
        if points is None or len(points) < 3:
            self.last_panel_diagnostic = (
                'Front LiDAR distance has insufficient returns: '
                f'count={0 if points is None else len(points)}/3')
            return None
        clearance = min(p.x for p in points) - front_x
        self.last_front_clearance = clearance
        self.last_panel_diagnostic = (
            f'AprilTag heading={math.degrees(heading):+.2f}deg; '
            f'front LiDAR clearance={clearance:.3f}m (minimum {margin:.3f}m)')
        if not math.isfinite(clearance) or clearance < margin:
            return None
        if bool(self.node.get_parameter('tag_front_use_lidar_panel_heading').value):
            return self._panel_heading(snapshot, tag, heading, bearing)
        return heading

    def _panel_heading(self, snapshot, tag, tag_heading, bearing):
        """Fit only the panel around the tag; return normal at SCAN time.

        The tag supplies the center and bounds the panel search. Neither
        side-rail positions nor the fitted panel midpoint become a target.
        The approach transforms this normal using the scan acquisition TF.
        """
        sequence = snapshot.sequence
        if getattr(self, '_panel_sequence', None) == sequence:
            self.last_panel_diagnostic = self._panel_diagnostic
            return self._panel_normal
        self._panel_sequence, self._panel_normal = sequence, None
        projected = self.lidar.project(snapshot, bearing, math.radians(60), .05, 16.)
        c, s = math.cos(tag_heading), math.sin(tag_heading)
        tx, ty = tag.pose.position.x, tag.pose.position.y
        points = [(p.x, p.y) for p in projected or ()
                  if abs(c*(p.x-tx)+s*(p.y-ty)) <= .20
                  and abs(-s*(p.x-tx)+c*(p.y-ty)) <= .22]
        inliers = self.panel_fitter._ransac_line_inliers(points)
        line = self.panel_fitter._fit_line_pca(inliers)
        minimum = int(self.node.get_parameter('lidar_align_min_inliers').value)
        if len(inliers) < minimum or line is None or line[1] < .25:
            self.last_panel_diagnostic = (
                f'Front panel near tag is unverified: inliers={len(inliers)}/{minimum}, '
                f'length={0. if line is None else line[1]:.3f}m (minimum 0.250m)')
            self._panel_diagnostic = self.last_panel_diagnostic
            return None
        normal = plane_normal_alignment_error(line[0], 0.)
        if abs(normal) > math.radians(30):
            self.last_panel_diagnostic = 'Front panel normal exceeds 30deg'
            self._panel_diagnostic = self.last_panel_diagnostic
            return None
        self._panel_normal = normal
        self.last_panel_diagnostic = (
            f'Tag heading={math.degrees(tag_heading):+.2f}deg; '
            f'front panel heading={math.degrees(normal):+.2f}deg; '
            f'inliers={len(inliers)}, length={line[1]:.3f}m')
        self._panel_diagnostic = self.last_panel_diagnostic
        return normal
