"""Camera pose defines the center; varying LiDAR surfaces must not steer it."""

import json
import math
from pathlib import Path
from types import SimpleNamespace

from docking.tag_centering import AprilTagCentering, apriltag_normal_yaw
from docking.tag_centering import _FrontLidarParameters
from docking.lidar_alignment import LidarPlaneAligner
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
    AprilTagCentering.declare_parameters(node)
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


def fused_centerer(points):
    centerer, parameters = make_centerer(points)
    LidarPlaneAligner.declare_parameters(centerer.node)
    parameters['tag_front_use_lidar_panel_heading'] = True
    centerer.panel_fitter = LidarPlaneAligner(
        _FrontLidarParameters(centerer.node), None, centerer.lidar)
    return centerer


@pytest.mark.parametrize('angle', [-12., -4., 0., 4., 12.])
def test_tag_position_is_retained_while_panel_supplies_normal(angle):
    normal = math.radians(angle)
    tag = tag_pose(normal+math.radians(4.5))
    tag.pose.position.y = .04
    original_position = (tag.pose.position.x, tag.pose.position.y)
    points = [SimpleNamespace(x=1.2-math.sin(normal)*u,
                              y=.04+math.cos(normal)*u)
              for u in [i*.006 for i in range(-32,33)]]
    # Background and side rails must not become the normal or tag center.
    points += [SimpleNamespace(x=1.7,y=i*.01) for i in range(-30,31)]
    points += [SimpleNamespace(x=.5+i*.02,y=y)
               for i in range(30) for y in [-.27,.27]]
    centerer = fused_centerer(points)
    assert centerer.panel(SimpleNamespace(sequence=1), tag) == pytest.approx(normal)
    assert (tag.pose.position.x,tag.pose.position.y) == original_position


@pytest.mark.parametrize('fault', ['few', 'short', 'background', 'side_rails'])
def test_missing_or_wrong_front_panel_cannot_silently_use_tag_heading(fault):
    points = [SimpleNamespace(x=1.2, y=i*.006) for i in range(-32,33)]
    if fault == 'few':points = points[:3]
    elif fault == 'short':points = points[25:40]
    elif fault == 'background':
        points = [SimpleNamespace(x=1.7,y=p.y) for p in points]
    else:
        points = [SimpleNamespace(x=1.05+i*.003,y=.04) for i in range(60)]
    centerer = fused_centerer(points)
    assert centerer.panel(SimpleNamespace(sequence=1),tag_pose(0.)) is None
    assert 'unverified' in centerer.last_panel_diagnostic
    assert centerer.panel(SimpleNamespace(sequence=1),tag_pose(0.)) is None
    assert 'unverified' in centerer.last_panel_diagnostic


@pytest.mark.parametrize('use_panel', [False, True])
@pytest.mark.parametrize('side', [-1, 1])
def test_tag_angle_limit_uses_the_selected_heading_source(use_panel, side):
    panel_angle = side*math.radians(29)
    tag = tag_pose(side*math.radians(32))
    points = [SimpleNamespace(x=1.2-math.sin(panel_angle)*u,
                              y=math.cos(panel_angle)*u)
              for u in [i*.006 for i in range(-32, 33)]]
    if use_panel:
        centerer = fused_centerer(points)
        assert centerer.panel(SimpleNamespace(sequence=1), tag) == pytest.approx(panel_angle)
    else:
        centerer, _ = make_centerer(points)
        assert centerer.panel(None, tag) is None


@pytest.mark.parametrize('fault', ['panel_angle', 'bearing', 'missing_points'])
def test_panel_mode_still_rejects_unverified_or_out_of_view_geometry(fault):
    angle = math.radians(32 if fault == 'panel_angle' else 29)
    tag = tag_pose(math.radians(32))
    if fault == 'bearing':
        tag.pose.position.y = tag.pose.position.x*math.tan(math.radians(31))
    points = [SimpleNamespace(x=tag.pose.position.x-math.sin(angle)*u,
                              y=tag.pose.position.y+math.cos(angle)*u)
              for u in [i*.006 for i in range(-32, 33)]]
    if fault == 'missing_points':
        points = []
    centerer = fused_centerer(points)
    assert centerer.panel(SimpleNamespace(sequence=1), tag) is None
