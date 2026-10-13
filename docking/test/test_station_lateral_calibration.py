"""Check the installed station correction's sign with a biased tag center."""
import math
from pathlib import Path

import pytest
import yaml

from test_backup_entry_completion import simulation  # noqa: F401
from test_centerline_approach import prepare


@pytest.mark.parametrize('perceived_right_bias', [.01,.03,.05])
@pytest.mark.parametrize('fault', [None,'motion_camera_delay'])
def test_station_calibration_moves_front_goal_left_before_half_turn(
        simulation,monkeypatch,perceived_right_bias,fault):
    profile = yaml.safe_load((Path(__file__).parents[1]/'config/docking.yaml').read_text())['dock_turn_backup']['ros__parameters']
    offset = profile['apriltag_front_lateral_offset']
    actual_center_y = .12
    # Model a tag-derived center to the right of the actual corridor. This
    # does not claim the camera hardware has this particular error.
    motion,state = simulation
    approach,perceived_station = prepare(simulation,monkeypatch,
        lateral=actual_center_y-perceived_right_bias-offset,
        runway=.8,yaw=math.radians(5),lateral_offset=offset,
        fault=fault,visibility=True)
    for name in ('tag_refinement_lateral_tolerance','tag_refinement_yaw_tolerance',
                 'apriltag_view_margin_fraction'):
        motion.node.parameters[name] = profile[name]
    assert perceived_station[1] == pytest.approx(actual_center_y-perceived_right_bias)
    assert approach.run(),motion.node.get_logger().error.call_args
    expected_y = perceived_station[1]-offset
    assert offset < 0 and expected_y > perceived_station[1]
    assert abs(state.y-expected_y) <= profile['tag_refinement_lateral_tolerance']
    assert abs(state.yaw) <= profile['tag_refinement_yaw_tolerance']
    # A half turn about the front axle moves base; check that it cannot turn
    # the intended +3cm correction into an opposite +3cm systematic error.
    post_turn_y = state.y+2*.10*math.sin(state.yaw)
    assert abs(post_turn_y-actual_center_y) < .035
    assert state.linear == state.angular == 0
    assert all(0 <= v <= .08 and abs(w) <= .15 for _,_,_,v,w in state.sent)
