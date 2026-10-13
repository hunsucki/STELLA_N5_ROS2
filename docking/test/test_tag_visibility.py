"""Full-tag visibility with measured camera intrinsics and axle pivot."""
import math
import time
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
from geometry_msgs.msg import Transform
from sensor_msgs.msg import CameraInfo
from docking.tag_visibility import TagVisibilityGuard, image_bounds
from test_apriltag_front_centering import tag_pose


def make_guard(node=None):
    parameters = {'base_frame': 'base_link', 'tag_centerline_angular_speed': .08}
    if node is None:
        node = NS(declare_parameter=lambda n, v: parameters.setdefault(n, v),
                  get_parameter=lambda n: NS(value=parameters[n]), get_logger=lambda: Mock())
    else:
        node.declare_parameter = lambda n, v: None
    transform = Transform()
    transform.translation.x, transform.translation.y, transform.translation.z = .2190114, .01471558, .1419925
    q = transform.rotation
    q.x, q.y, q.z, q.w = .49739035, -.50066961, .50237384, -.49955311
    node.tf_buffer = NS(lookup_transform=lambda *args: NS(transform=transform))
    TagVisibilityGuard.declare_parameters(node)
    info = CameraInfo()
    info.header.frame_id = 'camera_color_optical_frame'
    info.width, info.height = 640, 480
    info.p = [605.7516, 0., 307.4033, 0., 0., 605.8572, 237.0302, 0., 0., 0., 1., 0.]
    guard = TagVisibilityGuard.__new__(TagVisibilityGuard)
    guard.node, guard.info = node, info
    guard.received_at, guard.last_log_at = time.monotonic(), -math.inf
    guard.last_limited, guard.last_reason = False, ''
    return guard, transform


def observed_tag(bearing):
    tag = tag_pose(bearing)
    tag.pose.position.x = 1.1*math.cos(bearing)
    tag.pose.position.y = 1.1*math.sin(bearing)
    tag.pose.position.z = .142
    return tag


def test_centered_motion_is_unchanged_and_all_corners_remain_visible():
    guard, transform = make_guard()
    tag = observed_tag(0.)
    assert guard.limit(tag, .04, .01, .1) == (.04, .01)
    left, right, top, bottom = image_bounds(tag, transform, guard.info, .154)
    assert 64 < left < right < 576
    assert 0 < top < bottom < 480


@pytest.mark.parametrize('side', [-1., 1.])
def test_outward_turn_is_limited_before_tag_corners_leave_the_image(side):
    guard, transform = make_guard()
    tag = observed_tag(side*math.radians(16))
    requested = (.04, -side*.08)
    result = guard.limit(tag, *requested, .1)
    assert result is not None and result != requested and guard.last_limited
    for dt in [.1, .5, 1.]:
        left, right, top, bottom = image_bounds(tag, transform, guard.info, .154, *result, dt, .1)
        assert 2 <= left < right <= 638
        assert 2 <= top < bottom <= 478


def test_stationary_confirmation_does_not_turn_even_outside_the_soft_margin():
    guard, _ = make_guard()
    assert guard.limit(observed_tag(math.radians(16)), 0., 0., .1) == (0., 0.)


@pytest.mark.parametrize('side', [-1., 1.])
def test_blocked_stationary_outward_yaw_recenters_instead_of_returning_zero_forever(side):
    guard, _ = make_guard()
    tag = observed_tag(side*math.radians(15))
    result = guard.limit(tag, 0., -side*.08, .1)
    assert result is not None and abs(result[1]) > 1e-4
    if result[1]*side < 0:
        assert abs(result[1]) < .08  # A smaller outward turn still fits the margin.


def test_recorded_trial46_zero_yaw_deadlock_recovers_image_margin():
    import json
    from pathlib import Path
    from geometry_msgs.msg import PoseStamped
    record = json.loads((Path(__file__).parent/'fixtures'/'front_visibility_trial46.json').read_text())
    guard, transform = make_guard()
    camera = record['camera_info']
    guard.info.width, guard.info.height = camera['width'], camera['height']
    guard.info.p = camera['p']
    tf = record['camera_transform']
    for axis in ('x', 'y', 'z'):
        setattr(transform.translation, axis, tf[axis])
    for axis, value in zip(('x', 'y', 'z', 'w'), tf['q']):
        setattr(transform.rotation, axis, value)
    tag = PoseStamped()
    for axis in ('x', 'y', 'z'):
        setattr(tag.pose.position, axis, record['tag'][axis])
    for axis, value in zip(('x', 'y', 'z', 'w'), record['tag']['q']):
        setattr(tag.pose.orientation, axis, value)
    initial = image_bounds(tag, transform, guard.info, .154)
    result = guard.limit(tag, 0., .08, .1)
    assert result is not None and result[0] == 0. and result[1] < 0.
    assert guard.last_limited
    final = image_bounds(tag, transform, guard.info, .154, *result, 1., .1)
    assert final[1] < initial[1]


@pytest.mark.parametrize('fault', ['missing_info', 'stale_info', 'cropped', 'bad_profile'])
def test_missing_calibration_or_cropped_tag_never_authorizes_motion(fault):
    guard, _ = make_guard()
    tag = observed_tag(0.)
    if fault == 'missing_info':
        guard.info = None
    elif fault == 'stale_info':
        guard.received_at -= .6
    elif fault == 'cropped':
        tag = observed_tag(math.radians(28))
    else:
        old = guard.node.get_parameter
        guard.node.get_parameter = lambda name: NS(value=math.nan) if name == 'apriltag_view_tag_size' else old(name)
    assert guard.limit(tag, .04, .08, .1) is None


def test_failed_run_last_filtered_geometry_requests_an_inward_turn():
    # Reconstructed from filtered log errors, not a recorded raw tag pose.
    guard, _ = make_guard()
    heading = math.radians(20.42)
    tag = tag_pose(heading)
    tag.pose.position.x = math.cos(heading)*1.029-math.sin(heading)*(-.078)
    tag.pose.position.y = math.sin(heading)*1.029+math.cos(heading)*(-.078)
    tag.pose.position.z = .142
    result = guard.limit(tag, .011, -.08, .1)
    assert result is not None and result[0] == 0 and result[1] > 0
    result = guard.limit(tag, 0., -.04, .1, max_angular_speed=.04)
    assert result is not None and abs(result[1]) <= .04


def test_configured_visibility_size_matches_the_apriltag_detector():
    from pathlib import Path
    import yaml
    root = Path(__file__).parents[1]/'config'
    detector = yaml.safe_load((root/'tags_36h11.yaml').read_text())
    config = yaml.safe_load((root/'docking.yaml').read_text())
    assert config['dock_turn_backup']['ros__parameters']['apriltag_view_tag_size'] == detector['/**']['ros__parameters']['size']
