"""Tag center + panel normal must be independent, fresh and bounded."""

import math
import random
from types import SimpleNamespace
from unittest.mock import Mock

from docking.dock_turn_backup import DockTurnBackup
from docking.lidar_alignment import LidarPlaneAligner
from docking.tag_centering import (
    centered_goal_in_base, LidarTagCentering, standoff_explains_target_shift)
from geometry_msgs.msg import PoseStamped
import pytest


def make_live_centerer():
    """Keep actual pose freshness logic while isolating plane fitting here."""
    from test_observation_timing import FakeNode
    node = FakeNode()
    tag = PoseStamped()
    tag.header.frame_id = 'base_link'
    tag.header.stamp.sec = 10
    tag.pose.position.x = 1.2
    tag.pose.position.y = .1
    node.last_dock_pose = tag
    node.last_dock_pose_received_at = 10.
    node._pose_is_fresh = lambda *args: DockTurnBackup._pose_is_fresh(node, *args)
    snapshot = SimpleNamespace(sequence=1, scan=SimpleNamespace(
        header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0))))
    centerer = LidarTagCentering.__new__(LidarTagCentering)
    centerer.node = node
    centerer.lidar = SimpleNamespace(snapshot=lambda: snapshot)
    centerer.panel = Mock(return_value=math.radians(5))
    return centerer, node, snapshot


def test_live_errors_use_tag_center_and_independent_station_axes(monkeypatch):
    monkeypatch.setattr('time.monotonic', lambda: 10.)
    centerer, node, _ = make_live_centerer()
    longitudinal, lateral, yaw = centerer.control_observation(.95)
    assert longitudinal == pytest.approx(math.cos(yaw)*1.2 + math.sin(yaw)*.1 - .95)
    assert lateral == pytest.approx(-math.sin(yaw)*1.2 + math.cos(yaw)*.1)
    assert yaw == math.radians(5)
    # Reading the same scan/tag repeatedly cannot prove a stable center.
    for _ in range(20):
        assert centerer.control_observation(.95) == (longitudinal, lateral, yaw)
        assert centerer.control_observation_sequence == 1
    centerer.panel.assert_called_once()


def test_live_stability_requires_both_new_scan_and_new_tag(monkeypatch):
    monkeypatch.setattr('time.monotonic', lambda: 10.)
    centerer, node, snapshot = make_live_centerer()
    assert centerer.control_observation(.95) is not None
    snapshot.sequence += 1
    snapshot.scan.header.stamp.nanosec = 10_000_000
    assert centerer.control_observation(.95) is not None
    assert centerer.control_observation_sequence == 1
    node.last_dock_pose.header.stamp.nanosec = 10_000_000
    assert centerer.control_observation(.95) is not None
    assert centerer.control_observation_sequence == 2


@pytest.mark.parametrize('invalid', ['stale_tag', 'scan_gap', 'wrong_frame', 'unsynchronized'])
def test_live_cache_does_not_keep_invalid_observations_alive(monkeypatch, invalid):
    now = [10.]
    monkeypatch.setattr('time.monotonic', lambda: now[0])
    centerer, node, snapshot = make_live_centerer()
    assert centerer.control_observation(.95) is not None
    if invalid == 'stale_tag':
        now[0] = 10.31
        node.now_ns = 10_310_000_000
    elif invalid == 'scan_gap':
        centerer.lidar.snapshot = lambda: None
    elif invalid == 'wrong_frame':
        node.last_dock_pose.header.frame_id = 'camera'
    else:
        snapshot.scan.header.stamp.nanosec = 151_000_000
    assert centerer.control_observation(.95) is None
    assert centerer.control_observation_sequence == 1


@pytest.mark.parametrize('angle', [-.2, 0.0, .2])
def test_panel_normal_corrects_lateral_target_without_changing_standoff(angle):
    c, s = math.cos(angle), math.sin(angle)
    goal_x, goal_y = c * .12 - s * .06, s * .12 + c * .06
    tag_x, tag_y = c * 1.1, s * 1.1
    x, y = centered_goal_in_base(goal_x, goal_y, tag_x, tag_y, angle)
    assert c * x + s * y == pytest.approx(.12)
    assert -s * x + c * y == pytest.approx(0.0, abs=1e-10)


@pytest.mark.parametrize('seed', [0, 42, 137])
def test_recorded_trial37_central_panel_excludes_unrelated_wide_sector_points(seed):
    import json
    from pathlib import Path
    from docking.tag_centering import _FrontLidarParameters
    values = {}
    node = SimpleNamespace(
        declare_parameter=lambda n, v: values.setdefault(n, v),
        get_parameter=lambda n: SimpleNamespace(value=values[n]))
    LidarPlaneAligner.declare_parameters(node)
    LidarTagCentering.declare_parameters(node)
    helper = LidarTagCentering.__new__(LidarTagCentering)
    helper.node = node
    helper.fitter = LidarPlaneAligner(_FrontLidarParameters(node), None, None)
    rows = json.loads((Path(__file__).parent/'fixtures'/'front_panel_trial37.json').read_text())
    state = random.getstate()
    random.seed(seed)
    angles = []
    try:
        for row in rows:
            helper.lidar = SimpleNamespace(project=lambda _, bearing, width, *args: [
                SimpleNamespace(x=x, y=y)
                for x,y in row['sectors'][str(round(math.degrees(width)))]])
            tag = PoseStamped()
            tag.pose.position.x, tag.pose.position.y, tag.pose.position.z = row['tag']
            angle = helper.panel(None, tag)
            assert angle is not None, helper.last_panel_diagnostic
            assert 'sector=20deg' in helper.last_panel_diagnostic
            assert 9 < math.degrees(angle) < 12
            angles.append(angle)
        assert max(angles)-min(angles) < math.radians(1)
    finally:
        random.setstate(state)


@pytest.mark.parametrize('invalid', ['too_short', 'too_few', 'wrong_plane'])
def test_narrow_sector_does_not_bypass_panel_geometry_contract(invalid):
    values = {}
    node = SimpleNamespace(
        declare_parameter=lambda n, v: values.setdefault(n, v),
        get_parameter=lambda n: SimpleNamespace(value=values[n]))
    LidarPlaneAligner.declare_parameters(node)
    LidarTagCentering.declare_parameters(node)
    helper = LidarTagCentering.__new__(LidarTagCentering)
    helper.node = node
    helper.fitter = LidarPlaneAligner(node, None, None)
    points = [SimpleNamespace(x=1.,y=-.15+i*.01) for i in range(31)]
    if invalid == 'too_short':
        points = [SimpleNamespace(x=1.,y=i*.005) for i in range(31)]
    elif invalid == 'too_few':
        points = points[:19]
    helper.lidar = SimpleNamespace(project=lambda *args: points)
    tag = PoseStamped()
    tag.pose.position.x = 1.6 if invalid == 'wrong_plane' else 1.
    assert helper.panel(None, tag) is None


@pytest.mark.parametrize('first_supported_width', [30, 40])
def test_narrow_view_expands_only_when_it_lacks_panel_support(first_supported_width):
    values = {}
    node = SimpleNamespace(
        declare_parameter=lambda n, v: values.setdefault(n, v),
        get_parameter=lambda n: SimpleNamespace(value=values[n]))
    LidarPlaneAligner.declare_parameters(node)
    LidarTagCentering.declare_parameters(node)
    helper = LidarTagCentering.__new__(LidarTagCentering)
    helper.node = node
    helper.fitter = LidarPlaneAligner(node, None, None)
    tried = []

    def project(_, bearing, width, *args):
        degrees = round(math.degrees(width))
        tried.append(degrees)
        count = 19 if degrees < first_supported_width else 31
        return [SimpleNamespace(x=1., y=-.15+i*.01) for i in range(count)]

    helper.lidar = SimpleNamespace(project=project)
    tag = PoseStamped()
    tag.pose.position.x = 1.
    assert helper.panel(None, tag) == pytest.approx(0.)
    assert tried == ([20,30] if first_supported_width == 30 else [20,30,40])


@pytest.mark.parametrize('tag_x,valid', [(1.0, True), (1.6, False)])
def test_front_plane_must_be_near_the_tag_surface(tag_x, valid):
    parameters = {}
    node = SimpleNamespace(
        declare_parameter=lambda n, v: parameters.setdefault(n, v),
        get_parameter=lambda n: SimpleNamespace(value=parameters[n]))
    LidarPlaneAligner.declare_parameters(node)
    LidarTagCentering.declare_parameters(node)
    centerer = LidarTagCentering.__new__(LidarTagCentering)
    centerer.node = node
    centerer.fitter = LidarPlaneAligner(node, None, None)
    centerer.lidar = SimpleNamespace(project=lambda *args: [
        SimpleNamespace(x=1.0, y=-.25 + i * .01) for i in range(51)])
    tag = PoseStamped()
    tag.pose.position.x = tag_x
    assert (centerer.panel(None, tag) is not None) is valid


@pytest.mark.parametrize('scenario', [
    'stable', 'duplicate_tag', 'duplicate_scan', 'moving', 'stale',
    'jumping_center', 'too_large', 'wrong_frame', 'no_panel',
    'standoff_bias', 'standoff_disabled', 'standoff_residual'])
@pytest.mark.parametrize('required_pairs', [3, 5])
def test_target_correction_requires_distinct_stationary_consistent_pairs(
        monkeypatch, scenario, required_pairs):
    parameters = {}
    node = SimpleNamespace(
        declare_parameter=lambda n, v: parameters.setdefault(n, v),
        get_parameter=lambda n: SimpleNamespace(value=parameters[n]),
        get_logger=Mock(return_value=Mock()), should_stop=lambda: False,
        _quaternion_yaw=DockTurnBackup._quaternion_yaw)
    LidarTagCentering.declare_parameters(node)
    parameters['tag_centering_stable_pairs'] = required_pairs
    parameters['base_frame'] = 'base_link'
    now, tick = [10.0], [0]
    monkeypatch.setattr('docking.tag_centering.time.monotonic', lambda: now[0])
    tag = PoseStamped()
    tag.header.frame_id = 'camera' if scenario == 'wrong_frame' else 'base_link'
    tag.header.stamp.sec = 10
    tag.pose.position.x = 1.0
    node.last_dock_pose = tag
    node.last_dock_pose_received_at = now[0]
    node._pose_is_fresh = lambda *args: scenario != 'stale'
    node.motion = SimpleNamespace(
        stop_robot=Mock(), is_stationary=lambda tol: scenario != 'moving',
        current_xy=lambda: (0.0, 0.0), current_yaw=lambda: 0.0,
        current_odom_yaw_rate=lambda: 0.0)
    centerer = LidarTagCentering.__new__(LidarTagCentering)
    centerer.node = node
    centerer.lidar = SimpleNamespace(
        sequence=0, validate_parameters=lambda: True,
        snapshot=lambda: SimpleNamespace(sequence=0 if scenario == 'duplicate_scan' else tick[0]))
    centerer.panel = lambda *args: None if scenario == 'no_panel' else 0.0

    def advance(*args):
        now[0] += .1
        tick[0] += 1
        if scenario != 'duplicate_tag':
            tag.header.stamp.nanosec = tick[0] * 1_000_000
        if scenario == 'jumping_center':
            tag.pose.position.y = .05 if tick[0] % 2 else -.02

    monkeypatch.setattr('docking.tag_centering.spin_for', advance)
    target = PoseStamped()
    target.pose.position.x = .12
    target.pose.position.y = .20 if scenario == 'too_large' else .06
    target.pose.orientation.z = math.sin(math.radians(10) / 2)
    target.pose.orientation.w = math.cos(math.radians(10) / 2)
    if scenario.startswith('standoff'):
        target.pose.position.x = 1.0 - .95 * math.cos(math.radians(10))
        target.pose.position.y = -.95 * math.sin(math.radians(10))
        if scenario == 'standoff_residual':
            target.pose.position.y -= .04
    result = centerer.refine(
        target, allow_standoff_yaw_correction=scenario in (
            'standoff_bias', 'standoff_residual'))
    if scenario in ('stable', 'standoff_bias'):
        assert result is not target
        assert result.pose.position.x == pytest.approx(target.pose.position.x)
        assert result.pose.position.y == pytest.approx(0.0)
        assert result.pose.orientation.w == pytest.approx(1.0)
        assert tick[0] == required_pairs
        assert abs(target.pose.position.y) > .05  # Original remains unchanged.
        assert len(centerer.last_verified_alignment) == required_pairs
    else:
        assert result is target
        assert now[0] < 11.5
        assert centerer.last_verified_alignment is None


@pytest.mark.parametrize('direction', [-1, 1])
@pytest.mark.parametrize('standoff,residual,angle,expected', [
    (.95, .0, 8.4, True), (.95, .014, 8.4, True),
    (.95, .016, 8.4, False), (.49, 0., 8.4, False),
    (1.51, 0., 8.4, False), (.95, 0., 15.1, False),
    (.95, math.nan, 8.4, False),
])
def test_standoff_explanation_requires_consistent_old_tag_normal(
        direction, standoff, residual, angle, expected):
    yaw = direction * math.radians(angle)
    tag_x, tag_y = 1.031, -.226
    gx = tag_x - standoff * math.cos(yaw) + residual * math.sin(yaw)
    gy = tag_y - standoff * math.sin(yaw) - residual * math.cos(yaw)
    assert standoff_explains_target_shift(gx, gy, tag_x, tag_y, yaw, 0.) is expected


@pytest.mark.parametrize('offset,yaw,expected', [
    (0.005, 0.0, True), (-0.005, 0.0, True),
    (.07, 0.0, False), (-.07, 0.0, False),
    (0.0, math.radians(3), False),
    (.012, math.radians(.5), True), (-.012, math.radians(-.5), True),
    (.016, 0.0, False), (.012, math.radians(1.1), False),
])
@pytest.mark.parametrize('lidar_backup,guide_centering', [(True, True), (False, True), (True, False)])
def test_live_pre_spin_verification_rejects_slip_hidden_by_zero_odom_error(
        monkeypatch, offset, yaw, expected, lidar_backup, guide_centering):
    values = {}
    logger = Mock()
    node = SimpleNamespace(
        declare_parameter=lambda n, v: values.setdefault(n, v),
        get_parameter=lambda n: SimpleNamespace(value=values[n]),
        get_logger=lambda: logger, should_stop=lambda: False,
        _quaternion_yaw=DockTurnBackup._quaternion_yaw,
        _pose_is_fresh=lambda *args: True,
        motion=SimpleNamespace(
            current_xy=lambda: (0.0, 0.0), current_yaw=lambda: 0.0,
            is_stationary=lambda _: True, current_odom_yaw_rate=lambda: 0.0,
            stop_robot=Mock()))
    LidarTagCentering.declare_parameters(node)
    values.update(fixed_frame='odom', base_frame='base_link',
                  tag_refinement_lateral_tolerance=.01,
                  tag_refinement_yaw_tolerance=math.radians(2),
                  use_lidar_backup=lidar_backup, use_lidar_guide_centering=guide_centering)
    tag = PoseStamped()
    tag.header.frame_id = 'base_link'
    tag.pose.position.x = math.cos(yaw) - math.sin(yaw) * offset
    tag.pose.position.y = math.sin(yaw) + math.cos(yaw) * offset
    node.last_dock_pose, node.last_dock_pose_received_at = tag, 10.0
    tick = [0]
    helper = LidarTagCentering.__new__(LidarTagCentering)
    helper.node = node
    helper.panel = lambda *args: yaw
    helper.lidar = SimpleNamespace(
        sequence=0, validate_parameters=lambda: True,
        snapshot=lambda: SimpleNamespace(sequence=tick[0]))
    monkeypatch.setattr('docking.tag_centering.time.monotonic', lambda: 10 + tick[0] * .1)

    def advance(*args):
        tick[0] += 1
        tag.header.stamp.nanosec = tick[0] * 100_000_000

    monkeypatch.setattr('docking.tag_centering.spin_for', advance)
    expected = expected and (abs(offset) <= .01 or (lidar_backup and guide_centering))
    assert helper.verify_before_spin(allow_correction=False) is expected
    assert tick[0] == values['tag_centering_stable_pairs']
    assert helper.last_verified_alignment[0][0] == pytest.approx(offset)
    node.motion.stop_robot.assert_called_once()


def test_recorded_stationary_rear_panel_and_center_match_physical_inspection():
    import json
    from pathlib import Path
    from docking.lidar_geometry import plane_normal_alignment_error

    fixture = json.loads((Path(__file__).parent / 'fixtures' /
                          'rear_panel_and_guides_20261002.json').read_text())
    parameters = {'backup_rear_reference_x': -.2295}
    node = SimpleNamespace(
        declare_parameter=lambda n, v: parameters.setdefault(n, v),
        get_parameter=lambda n: SimpleNamespace(value=parameters[n]))
    LidarPlaneAligner.declare_parameters(node)
    parameters['lidar_align_ransac_threshold'] = .010
    aligner = LidarPlaneAligner(node, None, None)
    rng = random.getstate()
    random.seed(42)
    try:
        for scan in fixture['clouds']:
            inliers = aligner._ransac_line_inliers(scan['wall_points'])
            angle, length = aligner._fit_line_pca(inliers)
            error = plane_normal_alignment_error(angle, math.pi)
            assert abs(error) < math.radians(.35)
            assert len(inliers) / len(scan['wall_points']) > .85
            guide = aligner._estimate_guide_center_from_points(
                error, inliers, scan['guide_points'])
            assert guide is not None
            # User confirmed ~11 cm left clearance and right-side intrusion.
            assert .045 < guide.center_offset < .065
            assert .51 < guide.separation < .55
            assert guide.separation / 2 - guide.center_offset - .22 < .005
    finally:
        random.setstate(rng)


@pytest.mark.parametrize('degrees', [-20, 20])
def test_front_plane_can_exceed_rear_prior_without_changing_rear_limit(degrees):
    from docking.tag_centering import _FrontLidarParameters
    values = {}
    node = SimpleNamespace(
        declare_parameter=lambda n, v: values.setdefault(n, v),
        get_parameter=lambda n: SimpleNamespace(value=values[n]))
    LidarPlaneAligner.declare_parameters(node)
    LidarTagCentering.declare_parameters(node)
    helper = LidarTagCentering.__new__(LidarTagCentering)
    helper.node = node
    helper.fitter = LidarPlaneAligner(_FrontLidarParameters(node), None, None)
    yaw = math.radians(degrees)
    c, s = math.cos(yaw), math.sin(yaw)
    helper.lidar = SimpleNamespace(project=lambda *args: [
        SimpleNamespace(x=c-s*y, y=s+c*y) for y in [i*.01-.25 for i in range(51)]])
    tag = PoseStamped()
    tag.pose.position.x, tag.pose.position.y = c, s
    assert helper.panel(None, tag) == pytest.approx(yaw)
    assert values['lidar_align_candidate_max_error'] == math.radians(15)


def test_recorded_front_panel_is_supported_after_separating_candidate_prior():
    import json
    from pathlib import Path
    from docking.tag_centering import _FrontLidarParameters
    values = {}
    node = SimpleNamespace(
        declare_parameter=lambda n, v: values.setdefault(n, v),
        get_parameter=lambda n: SimpleNamespace(value=values[n]))
    LidarPlaneAligner.declare_parameters(node)
    LidarTagCentering.declare_parameters(node)
    helper = LidarTagCentering.__new__(LidarTagCentering)
    helper.node = node
    helper.fitter = LidarPlaneAligner(_FrontLidarParameters(node), None, None)
    fixture = json.loads((Path(__file__).parent/'fixtures'/'front_panel_20261002.json').read_text())
    state = random.getstate()
    random.seed(42)
    try:
        angles = []
        for row in fixture:
            helper.lidar = SimpleNamespace(project=lambda *args: [
                SimpleNamespace(x=x, y=y) for x,y in row['points']])
            tag = PoseStamped()
            tag.pose.position.x, tag.pose.position.y, tag.pose.position.z = row['tag']
            yaw = helper.panel(None, tag)
            assert yaw is not None, helper.last_panel_diagnostic
            assert -18 < math.degrees(yaw) < -16
            angles.append(yaw)
        assert max(angles)-min(angles) < math.radians(1)
    finally:
        random.setstate(state)
