"""Known geometry and stationary robot data for planar tag pose recovery."""
import json
from copy import deepcopy
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from apriltag_msgs.msg import AprilTagDetection, AprilTagDetectionArray
import cv2
from docking.apriltag_bridge import AprilTagToPoseBridge
from docking.tag_centering import apriltag_normal_yaw
from docking.tag_pose import homography_pose
from docking.tag_visibility import rotation
from geometry_msgs.msg import PoseStamped, Transform
import numpy as np
import pytest
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo


def recorded():
    data = json.loads((Path(__file__).parent/'fixtures/tag_pose_trial71.json').read_text())
    info = CameraInfo()
    info.p = data['camera_info']['p']
    info.width, info.height = 640, 480
    info.header.frame_id = 'camera_color_optical_frame'
    camera = Transform()
    camera.translation.x, camera.translation.y, camera.translation.z = data['camera_transform']['xyz']
    q = camera.rotation
    q.x, q.y, q.z, q.w = data['camera_transform']['quaternion_xyzw']
    return data, info, camera


def detection(values):
    d = AprilTagDetection()
    d.family, d.id = 'tag36h11', 0
    d.homography = values['homography']
    for p, pair in zip(d.corners, values['corners']):
        p.x, p.y = pair
    return d


def heading(pose):
    return apriltag_normal_yaw(SimpleNamespace(pose=pose))


def test_recorded_stationary_frames_do_not_switch_between_pnp_branches():
    data, info, camera = recorded()
    angles, old_angles = [], []
    k = np.array(info.p).reshape(3, 4)[:, :3]
    objects = np.array([[-.077, -.077, 0], [.077, -.077, 0],
                        [.077, .077, 0], [-.077, .077, 0]])
    for row in data['observations']:
        d = detection(row['detection'])
        pose = homography_pose(d, info, .154, camera)
        angles.append(math.degrees(heading(pose)))
        assert np.linalg.det(rotation(pose.orientation)) == pytest.approx(1.)
        ok, rv, _ = cv2.solvePnP(objects, np.array(row['detection']['corners']), k, None)
        assert ok
        n = rotation(camera.rotation) @ cv2.Rodrigues(rv)[0][:, 2]
        if n[0] < 0:
            n *= -1
        old_angles.append(math.degrees(math.atan2(n[1], n[0])))
    assert max(old_angles)-min(old_angles) > 7.
    assert max(angles)-min(angles) < 3.1
    assert np.median(angles) == pytest.approx(4.335, abs=.01)


@pytest.mark.parametrize('yaw', [-25., -8., 0., 8., 25.])
@pytest.mark.parametrize('distance', [1., 2.4])
@pytest.mark.parametrize('scale', [-2., 1., 3.])
def test_exact_geometry_recovers_center_normal_and_camera_axes(yaw, distance, scale):
    _, info, camera = recorded()
    h = math.radians(yaw)
    r = np.array([[math.sin(h), 0, -math.cos(h)],
                  [-math.cos(h), 0, -math.sin(h)], [0, 1, 0]])
    position = np.array([distance, .11, .06])
    rc = rotation(camera.rotation)
    pc = np.array([camera.translation.x, camera.translation.y, camera.translation.z])
    optical_r, optical_t = rc.T @ r, rc.T @ (position-pc)
    k = np.array(info.p).reshape(3, 4)[:, :3]
    # AprilTag's homography unit coordinates use y down, unlike tag pose axes.
    hh = k @ np.column_stack((optical_r[:, 0]*.077, -optical_r[:, 1]*.077, optical_t))
    hh *= scale/hh[2, 2]
    d = AprilTagDetection()
    d.homography = hh.ravel().tolist()
    units = np.array([[-1, 1, 1], [1, 1, 1], [1, -1, 1], [-1, -1, 1]])
    pixels = units @ hh.T
    for p, uv in zip(d.corners, pixels[:, :2]/pixels[:, 2:3]):
        p.x, p.y = map(float, uv)
    pose = homography_pose(d, info, .154, camera)
    assert [pose.position.x, pose.position.y, pose.position.z] == pytest.approx(position)
    assert rotation(pose.orientation) == pytest.approx(r)
    assert heading(pose) == pytest.approx(h)


@pytest.mark.parametrize('fault', ['nan', 'singular', 'wrong_corners', 'bad_calibration', 'bad_size'])
def test_invalid_geometry_does_not_produce_a_pose(fault):
    data, info, camera = recorded()
    d = detection(data['observations'][0]['detection'])
    size = .154
    if fault == 'nan':
        d.homography[0] = math.nan
    elif fault == 'singular':
        d.homography = [0.]*9
    elif fault == 'wrong_corners':
        d.corners[0].x += 8
    elif fault == 'bad_calibration':
        info.p[0] = 0.
    else:
        size = 0.
    with pytest.raises(ValueError):
        homography_pose(d, info, size, camera)


@pytest.mark.parametrize('fault', [None, 'duplicate', 'stale', 'other_tag', 'mismatched_header'])
def test_detection_bridge_preserves_observation_time_and_rejects_invalid_pairs(fault):
    data, info, camera = recorded()
    stamp_ns = 9_900_000_000
    info.header.stamp = Time(nanoseconds=stamp_ns).to_msg()
    array = AprilTagDetectionArray()
    array.header = info.header
    array.detections = [detection(data['observations'][0]['detection'])]
    if fault == 'mismatched_header':
        info = deepcopy(info)
        info.header.frame_id = 'wrong_camera'
    if fault == 'other_tag':
        array.detections[0].id = 1
    node = SimpleNamespace(last_published_stamp_nanoseconds=stamp_ns if fault == 'duplicate' else 0,
        transform_max_age_sec=.5, transform_future_tolerance_sec=.05, target_frame='tag36h11:0',
        source_frame='base_link', tag_size=.154, publisher_=Mock(),
        get_clock=lambda: SimpleNamespace(now=lambda: Time(nanoseconds=11_000_000_000
            if fault == 'stale' else 10_000_000_000)),
        tf_buffer=SimpleNamespace(lookup_transform=lambda *args: SimpleNamespace(transform=camera)))
    AprilTagToPoseBridge.on_detection(node, array, info)
    if fault:
        node.publisher_.publish.assert_not_called()
    else:
        msg = node.publisher_.publish.call_args.args[0]
        assert msg.header.frame_id == 'base_link'
        assert Time.from_msg(msg.header.stamp).nanoseconds == stamp_ns
        assert msg.pose.position.x > 2.4
        assert node.last_published_stamp_nanoseconds == stamp_ns
