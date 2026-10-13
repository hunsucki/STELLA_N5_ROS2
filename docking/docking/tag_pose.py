"""Recover a tag pose from its measured planar projection, without PnP branches."""
import math

import cv2
import numpy as np

from docking.tag_visibility import rotation
from geometry_msgs.msg import Pose


def homography_pose(detection, info, size, camera_transform):
    """Rectified pixels -> optical tag pose -> robot base, with tag y up/z out.

    AprilTag's homography uses unit coordinates with y down. Change those
    local tag axes before decomposition; never rotate the camera axes.
    """
    h = np.asarray(detection.homography, dtype=float).reshape(3, 3)
    k = np.asarray(info.p, dtype=float).reshape(3, 4)[:, :3]
    if (not math.isfinite(size) or not .04 <= size <= .30
            or not np.all(np.isfinite(h)) or not np.all(np.isfinite(k))
            or k[0, 0] <= 0 or k[1, 1] <= 0 or abs(k[2, 2]-1) > 1e-6
            or abs(np.linalg.det(h)) < 1e-8 or abs(np.linalg.det(k)) < 1e-8):
        raise ValueError('invalid tag homography or rectified calibration')
    # The same corner order as apriltag_ros PnP: bottom left/right, top right/left.
    normalized = np.linalg.solve(k, h @ np.diag([2/size, -2/size, 1.]))
    scale = 2 / (np.linalg.norm(normalized[:, 0])+np.linalg.norm(normalized[:, 1]))
    if normalized[2, 2] < 0:
        scale = -scale
    x, y = normalized[:, 0]*scale, normalized[:, 1]*scale
    u, _, vt = np.linalg.svd(np.column_stack((x, y, np.cross(x, y))))
    optical_rotation = u @ np.diag([1., 1., np.linalg.det(u @ vt)]) @ vt
    optical_position = normalized[:, 2]*scale
    if optical_position[2] <= .05:
        raise ValueError('tag is behind or too close to camera')
    half = size/2
    corners = np.array([[-half, -half, 0], [half, -half, 0],
                        [half, half, 0], [-half, half, 0]])
    points = corners @ optical_rotation.T + optical_position
    if min(points[:, 2]) <= .01:
        raise ValueError('tag corners cross the camera plane')
    projected = points @ k.T
    projected = projected[:, :2]/projected[:, 2:3]
    measured = np.array([[p.x, p.y] for p in detection.corners])
    rmse = float(np.sqrt(np.mean(np.sum((projected-measured)**2, axis=1))))
    if not math.isfinite(rmse) or rmse > 1.:
        raise ValueError(f'tag projection residual {rmse:.3f}px exceeds 1px')
    base_rotation = rotation(camera_transform.rotation)
    p = camera_transform.translation
    position = base_rotation @ optical_position + [p.x, p.y, p.z]
    orientation = base_rotation @ optical_rotation
    rv = cv2.Rodrigues(orientation)[0].ravel()
    angle = np.linalg.norm(rv)
    q = rv * (math.sin(angle/2)/angle if angle > 1e-12 else .5)
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = map(float, position)
    pose.orientation.x, pose.orientation.y, pose.orientation.z = map(float, q)
    pose.orientation.w = math.cos(angle/2)
    return pose
