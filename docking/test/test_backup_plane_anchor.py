"""Keep a changing visible panel from inventing chassis rotation (trial88)."""

import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
import yaml

from test_backup_entry_completion import simulation  # noqa: F401
from test_backup_initial_heading import sync_initial_pose


def configure(simulation, monkeypatch, side, offset=.030, bias=.6, kick=1.2):
    motion, state = simulation
    profile = yaml.safe_load((Path(__file__).parents[1]/'config/docking.yaml').read_text())
    motion.node.parameters.update(profile['dock_turn_backup']['ros__parameters'])
    state.gap, state.offset = .479, -side*offset
    state.yaw = side*math.radians(1.43)
    sync_initial_pose(motion, state)
    # Model weak steering before the caster has loaded, rather than assume
    # motor rotation immediately produces full chassis rotation.
    state.yaw_response = lambda distance: .2 if distance < .12 else 1.

    def heading(snapshot):
        if state.gap+state.x < .11:
            return None
        plane_bias = side*math.radians(bias)*min(abs(state.x)/.08, 1.)
        return -state.yaw-plane_bias, 0., 90, 100, .6

    def guide(snapshot):
        if state.gap+state.x < .11:
            return None
        center = state.offset-state.y
        return SimpleNamespace(center_offset=center, separation=.55,
            rear_center_offset=center+.2295*math.sin(state.yaw))

    motion.backup_lidar_heading_estimator = heading
    motion.backup_lidar_guide_estimator = guide
    original_spin = rclpy.spin_once
    state.caster_started = None

    def spin(node, timeout_sec):
        if state.linear < 0 and state.caster_started is None:
            state.caster_started = state.now
        requested = state.angular
        if (state.caster_started is not None
                and .2 <= state.now-state.caster_started < .8):
            state.angular += side*math.radians(kick)/(
                .6*state.yaw_response(abs(state.x)))
        original_spin(node, timeout_sec)
        state.angular = requested

    monkeypatch.setattr(rclpy, 'spin_once', spin)
    return motion, state


@pytest.mark.parametrize('side', [-1, 1])
@pytest.mark.parametrize('bias', [0., .6])
def test_measured_plane_bias_does_not_trigger_in_place_rotation(
        simulation, monkeypatch, side, bias):
    motion, state = configure(simulation, monkeypatch, side, bias=bias)
    # Deliberately bound this regression to the initial caster/guide phase.
    # A later successful full docking is not assumed from this partial run.
    motion.should_stop = lambda: abs(state.x) >= .15
    assert not motion.backup_with_lidar()
    assert abs(state.x) >= .15
    warnings = [c.args[0] for c in motion.node.get_logger().warning.call_args_list]
    assert not any('paused for heading correction' in row for row in warnings)
    logs = [c.args[0] for c in motion.node.get_logger().info.call_args_list]
    assert any('heading_pause_source=stationary_lidar_imu' in row for row in logs)
    assert state.linear == state.angular == 0


@pytest.mark.parametrize('side', [-1, 1])
def test_centered_biased_panel_entry_completes_and_holds_straight_after_loss(
        simulation, monkeypatch, side):
    motion, state = configure(simulation, monkeypatch, side, offset=.025)
    assert motion.backup_with_lidar(), motion.node.get_logger().error.call_args
    assert .022 <= state.gap+state.x <= .034
    assert all(w == 0 for _, x, _, v, w in state.sent
               if v < 0 and state.gap+x < .11)
    assert state.linear == state.angular == 0


@pytest.mark.parametrize('side', [-1, 1])
def test_inconsistent_gyro_anchor_retains_live_plane_authority(
        simulation, monkeypatch, side):
    motion, state = configure(simulation, monkeypatch, side, offset=.025, bias=0.)
    # A gyro integral discontinuity disagrees with the independently observed
    # plane and wheels. It cannot acquire authority just because it is fresh.
    motion.current_integrated_imu_yaw = lambda: state.yaw+(
        side*math.radians(5) if abs(state.x) > .03 else 0.)
    assert motion.backup_with_lidar(), motion.node.get_logger().error.call_args
    warnings = [c.args[0] for c in motion.node.get_logger().warning.call_args_list]
    assert not any('paused for heading correction' in row for row in warnings)
    logs = [c.args[0] for c in motion.node.get_logger().info.call_args_list]
    assert any('heading_pause_source=live_lidar' in row for row in logs)
    assert state.linear == state.angular == 0


@pytest.mark.parametrize('side', [-1, 1])
@pytest.mark.parametrize('fault', ['actual_rotation', 'plane_jump', 'no_guides', 'unsafe_side'])
def test_anchor_does_not_bypass_real_rotation_or_geometry_guards(
        simulation, monkeypatch, side, fault):
    motion, state = configure(simulation, monkeypatch, side,
        offset=.05 if fault == 'unsafe_side' else .030,
        bias=2.5 if fault == 'plane_jump' else .6,
        kick=2. if fault == 'actual_rotation' else 1.2)
    if fault == 'no_guides':
        motion.backup_lidar_guide_estimator = lambda snapshot: None
    assert not motion.backup_with_lidar()
    assert abs(state.x) < .15
    assert state.linear == state.angular == 0
    if fault == 'actual_rotation':
        warnings = [c.args[0] for c in motion.node.get_logger().warning.call_args_list]
        assert any('paused for heading correction' in row for row in warnings)
