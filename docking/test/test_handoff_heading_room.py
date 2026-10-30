"""Reproduce the 2026-10-30 near-bound handoff without claiming physics replay."""

import math
from pathlib import Path
from types import SimpleNamespace as S

import pytest
import rclpy
import yaml

from docking.control import (
    remaining_straight_heading_budget,
    straight_entry_heading_budget,
    straight_entry_path,
)
from test_backup_entry_completion import simulation  # noqa: F401
from test_backup_initial_heading import sync_initial_pose


@pytest.mark.parametrize('sign', [-1, 1])
def test_small_heading_change_can_fit_after_symmetric_budget_is_spent(sign):
    # The terminal log does not contain all five raw observations. These
    # synthetic values reproduce its ~1.04deg budget and 6mm final center.
    heading = -sign * math.radians(.917)
    rows = [(sign*c, .528, heading) for c in (.00785, .007, .0065, .006, .006)]
    budget = straight_entry_heading_budget(
        rows, .318, .015, .22, .02, math.radians(3), -.2295, known_width=.54)
    assert budget is not None
    changed_heading = heading - sign*math.radians(.16)
    assert abs(changed_heading) > remaining_straight_heading_budget(
        budget, .318, .307, .0002, math.radians(3))
    path = straight_entry_path(
        rows, .318, .22, .02, math.radians(3), -.2295, .54, .01, heading)
    assert path is not None
    assert path.clearance(changed_heading, .011, .307, .0002) >= .02
    assert path.clearance(heading-sign*math.radians(2), .011, .307, .0002) < .02


@pytest.mark.parametrize('sign', [-1, 1])
@pytest.mark.parametrize('legacy,disturbance,expected', [
    (True, .16, False), (False, .16, True),
    (False, .8, False), (False, 2.5, False),
])
def test_near_bound_handoff_continues_only_while_body_fits(
        simulation, monkeypatch, sign, legacy, disturbance, expected):
    motion, state = simulation
    motion.node.parameters.update(yaml.safe_load(
        (Path(__file__).parents[1]/'config/docking.yaml').read_text()
    )['dock_turn_backup']['ros__parameters'])
    state.gap, state.offset, state.yaw = .32, sign*.006, sign*math.radians(.917)
    sync_initial_pose(motion, state)
    motion.backup_lidar_guide_estimator = lambda snapshot: (
        None if state.gap+state.x < .30 else S(
            center_offset=(sign*.00785 if snapshot.sequence < 3 else state.offset)-state.y,
            separation=.528,
            rear_center_offset=state.offset-state.y+.2295*math.sin(state.yaw)))
    if legacy:
        # Recreate the former symmetric-only choice when that budget exists.
        monkeypatch.setattr('docking.motion.straight_entry_path', lambda *args: None)
    original_spin = rclpy.spin_once

    def spin(node, timeout_sec):
        requested = state.angular
        # A brief physical yaw change shared by IMU, odom, wheel and XY.
        if state.linear < 0 and .011 <= -state.x < .022:
            state.angular += sign*math.radians(disturbance)/(.011/.035)
        original_spin(node, timeout_sec)
        state.angular = requested

    monkeypatch.setattr(rclpy, 'spin_once', spin)
    assert motion.backup_with_lidar() is expected, motion.node.get_logger().error.call_args
    assert state.linear == state.angular == 0
    if expected:
        assert .012 <= state.gap+state.x <= .0245
        assert any('corridor_check=signed_rectangle' in str(c)
                   for c in motion.node.get_logger().info.call_args_list)
        assert all(w == 0 for _, x, _, v, w in state.sent
                   if v < 0 and state.gap+x < .30)
    else:
        assert 'bounded straight heading/travel' in (
            motion.node.get_logger().error.call_args.args[0])


@pytest.mark.parametrize('known_width', [0., .54])
def test_handoff_uses_body_path_for_measured_station_only(simulation, known_width):
    motion, state = simulation
    motion.node.parameters.update(yaml.safe_load(
        (Path(__file__).parents[1]/'config/docking.yaml').read_text()
    )['dock_turn_backup']['ros__parameters'])
    motion.node.parameters['backup_guide_known_width'] = known_width
    state.gap, state.offset = .32, .006
    assert motion.backup_with_lidar()
    assert state.linear == state.angular == 0
    mode = 'signed_rectangle' if known_width else 'symmetric_bound'
    assert any(f'corridor_check={mode}' in str(c)
               for c in motion.node.get_logger().info.call_args_list)


@pytest.mark.parametrize('sign', [-1, 1])
def test_left20_log_heading_change_fits_signed_corridor(sign):
    # Synthetic five-sample history: the supplied terminal log reports only
    # the final center, heading and budget, not the individual raw samples.
    heading = sign*math.radians(1.139)
    rows = [(sign*c, .526, heading)
            for c in (.0019, .0015, .001, .0012, .001)]
    budget = straight_entry_heading_budget(
        rows, .318, .015, .22, .02, math.radians(3), -.2295, known_width=.54)
    assert budget is not None
    # This case escaped the former quarter-degree selection gate.
    assert math.degrees(budget-abs(heading)) > .25
    changed_heading = sign*math.radians(1.507)
    assert abs(changed_heading) > remaining_straight_heading_budget(
        budget, .318, .217, .0025, math.radians(3))
    path = straight_entry_path(
        rows, .318, .22, .02, math.radians(3), -.2295, .54, .01, heading)
    assert path is not None
    assert path.clearance(changed_heading, .105, .217, .0025) >= .02


@pytest.mark.parametrize('sign', [-1, 1])
def test_left20_small_late_yaw_change_continues_with_body_margin(
        simulation, monkeypatch, sign):
    # Exercise selection and continuous reverse, not an exact sensor replay.
    motion, state = simulation
    motion.node.parameters.update(yaml.safe_load(
        (Path(__file__).parents[1]/'config/docking.yaml').read_text()
    )['dock_turn_backup']['ros__parameters'])
    state.gap, state.offset, state.yaw = .32, sign*.001, sign*math.radians(1.139)
    sync_initial_pose(motion, state)
    motion.backup_lidar_guide_estimator = lambda snapshot: (
        None if state.gap+state.x < .30 else S(
            center_offset=(sign*.0019 if snapshot.sequence < 3 else state.offset)-state.y,
            separation=.526,
            rear_center_offset=state.offset-state.y+.2295*math.sin(state.yaw)))
    original_spin = rclpy.spin_once

    def spin(node, timeout_sec):
        requested = state.angular
        if state.linear < 0 and .10 <= -state.x < .111:
            state.angular += sign*math.radians(.368)/(.011/.035)
        original_spin(node, timeout_sec)
        state.angular = requested

    monkeypatch.setattr(rclpy, 'spin_once', spin)
    assert motion.backup_with_lidar(), motion.node.get_logger().error.call_args
    assert state.linear == state.angular == 0
    assert .012 <= state.gap+state.x <= .0245
    assert any('corridor_check=signed_rectangle' in str(c)
               for c in motion.node.get_logger().info.call_args_list)
    assert all(w == 0 for _, x, _, v, w in state.sent
               if v < 0 and state.gap+x < .30)
