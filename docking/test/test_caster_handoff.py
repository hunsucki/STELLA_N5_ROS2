"""Keep guides through the measured initial reverse caster disturbance."""

import math
from pathlib import Path
from types import SimpleNamespace

import pytest
import rclpy
import yaml

from test_backup_entry_completion import simulation
from test_backup_initial_heading import sync_initial_pose


@pytest.mark.parametrize('handoff_clearance,expected', [(.50, False), (.32, True)])
def test_trial76_caster_turn_does_not_abort_before_guided_reverse(
        simulation, monkeypatch, handoff_clearance, expected):
    motion, state = simulation
    config = yaml.safe_load((Path(__file__).parents[1] / 'config/docking.yaml').read_text())
    motion.node.parameters.update(config['dock_turn_backup']['ros__parameters'])
    motion.node.parameters['backup_guide_visibility_handoff_clearance'] = handoff_clearance
    state.gap = .485
    state.yaw = math.radians(.171)
    motion.last_odom.pose.pose.orientation.z = math.sin(state.yaw / 2)
    motion.last_odom.pose.pose.orientation.w = math.cos(state.yaw / 2)
    motion.last_wheel_yaw = state.yaw
    # Trial76's 0.741deg corridor budget implies the early five-scan
    # history contained a ~9.8mm center estimate, despite the last one being
    # only 3mm. Model that warm-up spread and later actual guide disappearance.
    def guide(snapshot):
        if state.gap + state.x < .30:
            return None
        return SimpleNamespace(center_offset=(.0098 if snapshot.sequence < 3 else .003)-state.y,
                               separation=.542)

    motion.backup_lidar_guide_estimator = guide
    original_spin = rclpy.spin_once
    state.caster_started = None

    def spin(node, timeout_sec):
        if state.linear < 0 and state.caster_started is None:
            state.caster_started = state.now
        requested = state.angular
        # Trial76's handoff happened ~0.2s into reverse; the next 0.6s
        # changed gyro heading by .586deg. Exercise a somewhat larger .8deg
        # physical disturbance; the plane and wheel see the turn as well.
        if (state.caster_started is not None
                and .2 <= state.now-state.caster_started < .8):
            state.angular += math.radians(.8)/.6
        original_spin(node, timeout_sec)
        state.angular = requested

    monkeypatch.setattr(rclpy, 'spin_once', spin)
    assert motion.backup_with_lidar() is expected, motion.node.get_logger().error.call_args
    assert state.linear == state.angular == 0
    logs = [c.args[0] for c in motion.node.get_logger().info.call_args_list]
    handoff = [line for line in logs if 'Guide visibility handoff:' in line]
    assert len(handoff) == 1
    if expected:
        assert state.gap + state.x <= .034
        # Stay guided until >16cm of reverse has loaded the rear caster.
        assert 'remaining=0.3' in handoff[0]
        assert all(w == 0 for _, x, _, v, w in state.sent
                   if v < 0 and state.gap+x < .30)
    else:
        assert abs(state.x) < .03
        assert 'bounded straight heading/travel' in motion.node.get_logger().error.call_args.args[0]


@pytest.mark.parametrize('side', [-1, 1])
@pytest.mark.parametrize('disturbance', [.55, .8, 1.0])
@pytest.mark.parametrize('legacy_limit', [False, True])
def test_trial87_guide_target_leaves_room_for_caster_loading(
        simulation, monkeypatch, side, disturbance, legacy_limit):
    motion, state = simulation
    config = yaml.safe_load((Path(__file__).parents[1]/'config/docking.yaml').read_text())
    motion.node.parameters.update(config['dock_turn_backup']['ros__parameters'])
    if legacy_limit:
        motion.node.parameters['backup_guide_center_max_heading'] = math.radians(4)
    state.gap = .482
    state.offset = -side*.030
    state.yaw = side*math.radians(2.27)
    sync_initial_pose(motion, state)

    def guide(snapshot):
        if state.gap+state.x < .11:
            return None
        center = state.offset-state.y
        return SimpleNamespace(center_offset=center, separation=.56,
            rear_center_offset=center+.2295*math.sin(state.yaw))

    motion.backup_lidar_guide_estimator = guide
    original_heading = motion.backup_lidar_heading_estimator
    motion.backup_lidar_heading_estimator = lambda snapshot: (
        None if state.gap+state.x < .11 else original_heading(snapshot))
    original_spin = rclpy.spin_once
    state.caster_started = None

    def spin(node, timeout_sec):
        if state.linear < 0 and state.caster_started is None:
            state.caster_started = state.now
        requested = state.angular
        if (state.caster_started is not None
                and .2 <= state.now-state.caster_started < .8):
            state.angular += side*math.radians(disturbance)/.6
        original_spin(node, timeout_sec)
        state.angular = requested

    monkeypatch.setattr(rclpy, 'spin_once', spin)
    expected = disturbance < 1. and (not legacy_limit or disturbance < .8)
    assert motion.backup_with_lidar() is expected, motion.node.get_logger().error.call_args
    assert state.linear == state.angular == 0
    warnings = [c.args[0] for c in motion.node.get_logger().warning.call_args_list]
    if expected:
        assert .022 <= state.gap+state.x <= .034
        assert not any('paused for heading correction' in row for row in warnings)
        assert all(w == 0 for _, x, _, v, w in state.sent
                   if v < 0 and state.gap+x < .11)
        assert state.now-10 < 35
    else:
        # Larger disturbances still activate the original 3deg correction
        # and side-margin guards; the change cannot force uncentered entry.
        assert abs(state.x) < .04
        assert any('paused for heading correction' in row for row in warnings)
        assert 'Insufficient guide side clearance' in motion.node.get_logger().error.call_args.args[0]
