"""Recorded boundary recurrence and delayed-drive centerline regressions."""
import json
import math
from pathlib import Path

import pytest
from rclpy.time import Time

from docking.lidar_geometry import yaw_from_quaternion
from test_backup_entry_completion import simulation  # noqa: F401
from test_centerline_approach import prepare


@pytest.mark.parametrize('mirror', [-1, 1])
def test_recorded_reacquisition_does_not_turn_back_out_of_panel_view(
        simulation, monkeypatch, mirror):
    motion, state = simulation
    approach, _ = prepare(simulation, monkeypatch)
    motion.node.parameters['tag_front_use_lidar_panel_heading'] = True
    # Replay acquisition-time normals against the recorded CURRENT odom yaw,
    # rather than the unrelated initial odom message from prepare().
    motion.current_yaw = lambda: state.yaw
    rows = json.loads((Path(__file__).parent/'fixtures/front_boundary_trial23.json').read_text())['samples']
    for row in rows:
        x, y, h = row['errors']
        errors = x, mirror*y, mirror*h
        state.yaw = mirror*row['current_yaw']
        approach.observations.extend((state.now, 0., 0., mirror*v)
                                     for v in row['world_panel_normals'])
        command = approach.command(errors, .10, .08, .15, (.02, .012), math.radians(1))
        guarded = approach.limit_panel_turn(errors, *command, .10)
        assert guarded[0] == command[0]  # No blanket forward slowdown.
        assert abs(guarded[1]) <= abs(command[1])
        assert guarded[1]*command[1] >= 0.  # Never invent a contrary turn.
        if row is rows[0]:
            assert guarded == command
        if row is rows[-1]:
            assert command[1]*mirror < 0.
            assert guarded[1] == 0.
            assert guarded[0] > .07
            # A turn back toward the panel remains fully available.
            assert approach.limit_panel_turn(errors, .08, mirror*.10, .10) == (.08, mirror*.10)
            # Close-in final centering and tag-only mode stay unchanged.
            assert approach.limit_panel_turn((.20, errors[1], errors[2]), *command, .10) == command
            motion.node.parameters['tag_front_use_lidar_panel_heading'] = False
            assert approach.limit_panel_turn(errors, *command, .10) == command


@pytest.mark.parametrize('lateral,yaw_deg', [(-.20, -10), (-.20, 10), (.20, -10), (.20, 10), (.40, -4)])
def test_delayed_drive_preserves_supported_staging_and_view(
        simulation, monkeypatch, lateral, yaw_deg):
    motion, state = simulation
    approach, station = prepare(simulation, monkeypatch, lateral=lateral,
        runway=1.1, yaw=math.radians(yaw_deg), lateral_offset=0.,
        fault='motion_camera_delay', visibility=True)
    motion.node.parameters.update(tag_front_use_lidar_panel_heading=True,
        tag_refinement_lateral_tolerance=.012,
        tag_refinement_yaw_tolerance=math.radians(1))
    observed = []
    def panel(scan, tag):
        q = motion.node.tf_buffer.lookup_transform('odom', 'base_link',
            Time.from_msg(scan.scan.header.stamp)).transform.rotation
        heading = -yaw_from_quaternion(q.x, q.y, q.z, q.w)
        observed.append(abs(heading))
        approach.centerer.panel_recovery_ready = abs(heading) > math.radians(30)
        approach.centerer.last_front_clearance = tag.pose.position.x-.2295
        return None if approach.centerer.panel_recovery_ready else heading
    approach.centerer.panel = panel
    assert approach.run(), motion.node.get_logger().error.call_args
    assert max(observed) < math.radians(28)
    assert not approach.reorientation_used
    assert abs(state.x-(station[0]-.95)) <= .02
    assert abs(state.y-station[1]) <= .012
    assert abs(state.yaw) <= math.radians(1)
    assert state.linear == state.angular == 0.
