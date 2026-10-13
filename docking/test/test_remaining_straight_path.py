"""Account for traveled side displacement instead of retaining a full-path angle."""
import math
from pathlib import Path

import pytest
import rclpy
import yaml

from docking.control import remaining_straight_heading_budget as budget
from test_backup_entry_completion import simulation


def test_remaining_path_keeps_original_side_room_and_whole_body():
    initial = math.radians(1.208)
    room = math.sin(initial)*(.320+.459)
    assert budget(initial, .320, .320, 0., math.radians(3)) == pytest.approx(initial)
    for remaining, spent in [(.20, .001), (.076, .003), (.025, .005)]:
        angle = budget(initial, .320, remaining, spent, math.radians(3))
        assert spent + (remaining+.459)*math.sin(angle) <= room+1e-12
    assert budget(initial, .320, .076, .003, math.radians(3)) > math.radians(1.210)
    assert budget(initial, .320, .076, .010, math.radians(3)) < initial
    assert budget(initial, .320, .076, room, math.radians(3)) == 0.


@pytest.mark.parametrize('field,value', [(0, math.nan), (1, -1.), (2, -.01),
                                         (3, -.01), (3, math.inf), (4, math.nan)])
def test_invalid_remaining_path_cannot_authorize_motion(field, value):
    args = [math.radians(1.2), .32, .10, .002, math.radians(3)]
    args[field] = value
    assert budget(*args) == 0.


@pytest.mark.parametrize('end_angle,expected', [(1.1, True), (3.5, False)])
def test_late_physical_yaw_uses_remaining_corridor(simulation, monkeypatch, end_angle, expected):
    motion, state = simulation
    config = yaml.safe_load((Path(__file__).parents[1]/'config/docking.yaml').read_text())
    motion.node.parameters.update(config['dock_turn_backup']['ros__parameters'])
    state.gap = .32
    state.offset = .0056
    original_spin = rclpy.spin_once
    def spin(node, timeout_sec):
        requested = state.angular
        # Physical gradual yaw shared by IMU, odom, wheel and side displacement;
        # guides disappear after the verified commitment, as at the station.
        if state.linear < 0:
            state.angular += math.radians(end_angle) * abs(state.linear)/.245
        original_spin(node, timeout_sec)
        state.angular = requested
    motion.backup_lidar_guide_estimator = lambda _: (
        None if state.gap+state.x < .30 else type('Guide', (), {
            'center_offset': state.offset-state.y, 'separation': .519})())
    monkeypatch.setattr(rclpy, 'spin_once', spin)
    assert motion.backup_with_lidar() is expected, motion.node.get_logger().error.call_args
    assert state.linear == state.angular == 0
    if expected:
        assert state.gap+state.x <= .034
    else:
        assert 'bounded straight heading/travel' in motion.node.get_logger().error.call_args.args[0]
