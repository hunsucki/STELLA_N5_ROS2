"""Camera pose defines the center; varying LiDAR surfaces must not steer it."""

import json
import math
from pathlib import Path
from types import SimpleNamespace

from docking.tag_centering import AprilTagCentering, LidarTagCentering, apriltag_normal_yaw
from geometry_msgs.msg import PoseStamped
import pytest


def tag_pose(heading):
    tag = PoseStamped()
    tag.pose.position.x = 1.2
    # A visible upright tag has its local z normal facing the camera.
    s, c = math.sin(heading/2), math.cos(heading/2)
    tag.pose.orientation.x = s/math.sqrt(2)
    tag.pose.orientation.y = -c/math.sqrt(2)
    tag.pose.orientation.z = s/math.sqrt(2)
    tag.pose.orientation.w = c/math.sqrt(2)
    return tag


def make_centerer(points):
    parameters = {}
    node = SimpleNamespace(
        declare_parameter=lambda n, v: parameters.setdefault(n, v),
        get_parameter=lambda n: SimpleNamespace(value=parameters[n]))
    LidarTagCentering.declare_parameters(node)
    centerer = AprilTagCentering.__new__(AprilTagCentering)
    centerer.node = node
    centerer.lidar = SimpleNamespace(project=lambda *args: points)
    return centerer, parameters


@pytest.mark.parametrize('angle', [-25., -8., 0., 8., 25.])
@pytest.mark.parametrize('scale', [-2., 1., 3.])
def test_heading_uses_tag_face_normal_with_correct_camera_axis_conversion(angle, scale):
    tag = tag_pose(math.radians(angle))
    for axis in ('x', 'y', 'z', 'w'):
        setattr(tag.pose.orientation, axis, scale*getattr(tag.pose.orientation, axis))
    assert apriltag_normal_yaw(tag) == pytest.approx(math.radians(angle))


@pytest.mark.parametrize('fault', ['zero', 'nan', 'vertical'])
def test_invalid_tag_orientation_is_rejected(fault):
    tag = PoseStamped()
    if fault == 'nan':
        tag.pose.orientation.w = math.nan
    elif fault == 'vertical':
        tag.pose.orientation.w = 1.
    assert apriltag_normal_yaw(tag) is None


def test_changing_lidar_surface_cannot_change_camera_heading_or_center():
    tag = tag_pose(math.radians(-8.))
    tag.pose.position.y = -.11
    centerer, _ = make_centerer([])
    results = []
    for slope in [-.5, 0., .5]:
        # A sloping / curved scan cannot supply the station's normal here.
        centerer.lidar.project = lambda *args: [
            SimpleNamespace(x=1.1+slope*y, y=y) for y in [-.08,0.,.08]]
        heading = centerer.panel(None, tag)
        assert heading is not None
        results.append((-math.sin(heading)*tag.pose.position.x
                        + math.cos(heading)*tag.pose.position.y, heading))
    assert results[0] == results[1] == results[2]
    assert results[0][0] > .05  # Same sign as the camera, unlike the old narrow fit.
    assert not hasattr(centerer, 'fitter')


def test_virtual_goal_keeps_the_configured_tag_standoff_despite_old_odom_goal():
    tag = tag_pose(math.radians(-8.))
    tag.pose.position.y = -.11
    centerer, _ = make_centerer([])
    heading = apriltag_normal_yaw(tag)
    for old_goal in [(0.,0.),(.2,-.3),(.4,.1)]:
        x,y,standoff = centerer._goal_and_standoff(*old_goal,tag,heading)
        assert standoff == .95
        assert math.hypot(tag.pose.position.x-x, tag.pose.position.y-y) == pytest.approx(.95)
        assert -math.sin(heading)*x+math.cos(heading)*y == pytest.approx(
            -math.sin(heading)*tag.pose.position.x+math.cos(heading)*tag.pose.position.y)


@pytest.mark.parametrize('with_outlier', [False,True])
def test_filtered_pose_compensates_real_rotation_without_lag_or_yaw_outliers(
        monkeypatch, with_outlier):
    from test_tag_centering import make_live_centerer
    legacy,node,snapshot = make_live_centerer()
    state = SimpleNamespace(now=10.,gyro=0.,fresh=True)
    monkeypatch.setattr('time.monotonic',lambda:state.now)
    helper = AprilTagCentering.__new__(AprilTagCentering)
    helper.node,helper.lidar = node,legacy.lidar
    helper.lidar.project = lambda *args: [SimpleNamespace(x=1.,y=y) for y in [-.04,0.,.04]]
    node.motion = SimpleNamespace(
        imu_is_fresh=lambda:state.fresh,current_integrated_imu_yaw=lambda:state.gyro)
    for index,heading in enumerate([.12,.09,.06,.03,0.]):
        state.now = 10.+index*.1
        state.gyro = -heading
        node.now_ns = int(state.now*1e9)
        tag = tag_pose(heading+(.15 if with_outlier and index==2 else 0.))
        tag.header.frame_id = 'base_link'
        tag.header.stamp = node.get_clock().now().to_msg()
        tag.pose.position.x = math.cos(heading)*1.1-math.sin(heading)*.01
        tag.pose.position.y = math.sin(heading)*1.1+math.cos(heading)*.01
        node.last_dock_pose,node.last_dock_pose_received_at = tag,state.now
        snapshot.sequence = index+1
        snapshot.scan.header.stamp = tag.header.stamp
        longitudinal,lateral,filtered_heading = helper.control_observation(.95)
        assert longitudinal == pytest.approx(.15)
        assert lateral == pytest.approx(.01)
        assert filtered_heading == pytest.approx(heading)
    for _ in range(20):
        helper.control_observation(.95)
    assert len(helper._front_pose_window) == 5
    # Cached camera samples cannot survive either an old stamp or lost gyro.
    state.fresh = False
    assert helper.control_observation(.95) is None
    state.fresh = True
    assert helper.control_observation(.95) is not None
    state.now += .31
    node.now_ns = int(state.now*1e9)
    assert helper.control_observation(.95) is None


@pytest.mark.parametrize('points', [None, [], [.8,.8], [.25,.8,.8]])
def test_missing_or_close_front_range_prevents_refinement(points):
    projected = None if points is None else [SimpleNamespace(x=x, y=0.) for x in points]
    centerer, _ = make_centerer(projected)
    assert centerer.panel(None, tag_pose(0.)) is None


def test_recorded_trial39_camera_center_is_independent_of_bad_narrow_panel_normal():
    rows = json.loads((Path(__file__).parent/'fixtures'/'front_apriltag_trial39.json').read_text())
    laterals = []
    for row in rows:
        record = row['tag']
        tag = PoseStamped()
        tag.pose.position.x, tag.pose.position.y = record['x'], record['y']
        q = tag.pose.orientation
        q.x, q.y, q.z, q.w = record['q']
        centerer, _ = make_centerer([SimpleNamespace(x=x,y=y) for x,y in row['front_points']])
        heading = centerer.panel(None, tag)
        assert heading is not None, centerer.last_panel_diagnostic
        laterals.append(-math.sin(heading)*record['x'] + math.cos(heading)*record['y'])
    assert .05 < laterals[0] < .07  # Narrow LiDAR previously said the opposite side.
    assert abs(laterals[-1]) < .02  # Camera does not reproduce the false ~14 cm error.
