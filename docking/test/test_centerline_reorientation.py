"""Recovery uses recent memory only to regain live observations, never to dock."""
import math
from types import SimpleNamespace as S

import pytest
from rclpy.time import Time
from docking.lidar_geometry import yaw_from_quaternion
from test_backup_entry_completion import simulation  # noqa: F401
from test_centerline_approach import prepare


def remembered_approach(simulation, monkeypatch, side=1, lateral=None):
    motion, state = simulation
    approach, station = prepare(simulation, monkeypatch, lateral=side*.30 if lateral is None else lateral,
        runway=1.1, yaw=side*math.radians(31.5), lateral_offset=0., visibility=True)
    node = motion.node
    node.parameters.update(tag_front_use_lidar_panel_heading=True,
        tag_refinement_lateral_tolerance=.012,
        tag_refinement_yaw_tolerance=math.radians(1))
    # A station already observed before the turn crossed the fit's limit.
    approach.station = (*station, 0.)
    approach.target = (station[0]-.95, station[1], 0.)
    approach.filtered_at = node.now_ns
    approach.last_verified_at = state.now
    def panel(scan, tag):
        q = node.tf_buffer.lookup_transform('odom', 'base_link',
            Time.from_msg(scan.scan.header.stamp)).transform.rotation
        heading = -yaw_from_quaternion(q.x, q.y, q.z, q.w)
        approach.centerer.last_front_clearance = tag.pose.position.x-.2295
        approach.centerer.panel_recovery_ready = abs(heading) > math.radians(30)
        if approach.centerer.panel_recovery_ready:
            approach.centerer.last_panel_diagnostic = 'front panel exceeds 30deg'
            return None
        return heading
    approach.centerer.panel = panel
    return approach, station


@pytest.mark.parametrize('side', [-1, 1])
def test_recently_seen_panel_is_reacquired_before_translation(simulation, monkeypatch, side):
    motion, state = simulation
    # At 31.5deg panel heading, this places the still-visible tag about
    # 18deg off-axis, like the recorded failed approach.
    approach, _ = remembered_approach(simulation, monkeypatch, side, lateral=side*.50)
    motion.node.should_stop = lambda: approach.sequence >= 3 and motion.is_stationary(.005)
    assert not approach.run()  # Deliberate cancellation after re-observation.
    assert approach.reorientation_used
    assert approach.sequence >= 3
    assert abs(state.yaw) < math.radians(30)
    assert all(v == 0. for _, _, _, v, _ in state.sent)
    assert any(abs(w) > .01 for _, _, _, _, w in state.sent)
    assert state.linear == state.angular == 0.


@pytest.mark.parametrize('fault', ['old_memory', 'tag_missing', 'imu_missing',
                                   'front_obstacle', 'view_rejects_turn'])
def test_bad_inputs_do_not_authorize_memory_motion(simulation, monkeypatch, fault):
    motion, state = simulation
    approach, _ = remembered_approach(simulation, monkeypatch)
    if fault == 'old_memory':
        approach.last_verified_at = state.now-4.
    elif fault == 'tag_missing':
        motion.node._pose_is_fresh = lambda *a: False
    elif fault == 'imu_missing':
        motion.imu_is_fresh = lambda: False
    elif fault == 'front_obstacle':
        old_panel = approach.centerer.panel
        def panel(scan, tag):
            old_panel(scan, tag)
            approach.centerer.last_front_clearance = .01
            approach.centerer.panel_recovery_ready = False
            return None
        approach.centerer.panel = panel
    else:
        approach.centerer.view_guard = S(limit=lambda *a, **k: None)
    assert not approach.run()
    assert all(v == w == 0. for _, _, _, v, w in state.sent)


@pytest.mark.parametrize('slip', [False, True])
def test_recovery_is_bounded_and_cannot_finish_without_new_panel(simulation, monkeypatch, slip):
    motion, state = simulation
    approach, _ = remembered_approach(simulation, monkeypatch)
    original_yaw = state.yaw
    def missing_panel(scan, tag):
        approach.centerer.last_front_clearance = 1.5
        approach.centerer.panel_recovery_ready = True
        return None
    approach.centerer.panel = missing_panel
    if slip:
        motion.current_integrated_imu_yaw = lambda: original_yaw
    assert not approach.run()
    moving = [t for t, _, _, _, w in state.sent if abs(w) > .0001]
    assert moving and max(moving)-min(moving) <= 3.05
    assert all(v == 0. for _, _, _, v, _ in state.sent)
    assert abs(state.yaw-original_yaw) <= math.radians(3.5 if slip else 15.5)
    assert all(abs(w) <= .10 for _, _, _, _, w in state.sent)
    assert state.linear == state.angular == 0.
    assert not any('approach complete' in c.args[0].lower()
                   for c in motion.node.get_logger().info.call_args_list)


@pytest.mark.parametrize('lateral', [-.30, .30, .40])
def test_preserved_controller_still_completes_known_centering_cases(simulation, monkeypatch, lateral):
    motion, state = simulation
    approach, station = prepare(simulation, monkeypatch, lateral=lateral,
        runway=1.1, yaw=-math.copysign(math.radians(4), lateral), lateral_offset=0.,
        fault='motion_camera_delay', visibility=True)
    motion.node.parameters.update(tag_front_use_lidar_panel_heading=True,
        tag_refinement_lateral_tolerance=.012,
        tag_refinement_yaw_tolerance=math.radians(1))
    observed = []
    def panel(scan, tag):
        q = motion.node.tf_buffer.lookup_transform('odom', 'base_link',
            Time.from_msg(scan.scan.header.stamp)).transform.rotation
        h = -yaw_from_quaternion(q.x, q.y, q.z, q.w)
        observed.append(abs(h))
        approach.centerer.panel_recovery_ready = abs(h) > math.radians(30)
        approach.centerer.last_front_clearance = tag.pose.position.x-.2295
        return None if approach.centerer.panel_recovery_ready else h
    approach.centerer.panel = panel
    assert approach.run(), (motion.node.get_logger().error.call_args, state.x, state.y, state.yaw, approach.reorientation_used, approach.centerer.view_guard.last_reason, [c.args[0] for c in motion.node.get_logger().info.call_args_list[-5:]])
    assert max(observed) < math.radians(30)
    assert abs(state.x-(station[0]-.95)) <= .02
    assert abs(state.y-station[1]) <= .012
    assert abs(state.yaw) <= math.radians(1)


def test_reorientation_can_resume_and_finish_with_fresh_observations(simulation, monkeypatch):
    motion, state = simulation
    approach, station = remembered_approach(simulation, monkeypatch)
    assert approach.run(), motion.node.get_logger().error.call_args
    assert approach.reorientation_used
    assert abs(state.x-(station[0]-.95)) <= .02
    assert abs(state.y-station[1]) <= .012
    assert abs(state.yaw) <= math.radians(1)
    assert state.linear == state.angular == 0.
