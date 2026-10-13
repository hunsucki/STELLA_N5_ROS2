"""The installed spin calibration retains direction, limits and settling."""

import math
from pathlib import Path

import pytest
import yaml

from test_backup_entry_completion import simulation  # noqa: F401
from test_backup_initial_heading import sync_initial_pose


@pytest.mark.parametrize('start_degrees', [-105., 172., -179.])
@pytest.mark.parametrize('direction', [-1, 1])
def test_configured_spin_target_handles_wrap_and_stops(
        simulation, start_degrees, direction):
    motion, state = simulation
    profile = yaml.safe_load((Path(__file__).parents[1]/'config/docking.yaml').read_text())
    motion.node.parameters.update(profile['dock_turn_backup']['ros__parameters'])
    requested = direction*motion.node.parameters['spin_yaw']
    assert abs(requested) == pytest.approx(math.radians(176))
    motion.node.parameters['spin_yaw'] = requested
    start = math.radians(start_degrees)
    state.yaw = start
    sync_initial_pose(motion, state)
    assert motion.spin_180(), motion.node.get_logger().error.call_args
    assert abs(state.yaw-start-requested) <= math.radians(1)
    nonzero = [w for _, _, _, _, w in state.sent if w]
    assert nonzero and nonzero[0]*direction > 0
    assert all(v == 0 and abs(w) <= .15 for _, _, _, v, w in state.sent)
    assert state.linear == state.angular == state.actual_angular == 0
