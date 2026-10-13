"""Check signed rectangle geometry, recorded guide loss, and live guard paths."""
import json
import math
from pathlib import Path
import random
from types import SimpleNamespace as S

import pytest
import yaml

from docking.control import straight_entry_path, straight_entry_heading_budget
from docking.lidar_alignment import LidarPlaneAligner
from docking.lidar_geometry import (
    PlanarTransform, project_scan_to_base, rear_clearances, yaw_from_quaternion,
)
from test_backup_entry_completion import simulation  # noqa: F401
from test_backup_initial_heading import sync_initial_pose


def path(rows, distance=.16, anchor=None):
    return straight_entry_path(rows, distance, .22, .02, math.radians(3),
                               -.2295, .54, .01,
                               rows[-1][2] if anchor is None else anchor)


@pytest.mark.parametrize('sign', [-1, 1])
def test_reverse_toward_center_preserves_both_sides_of_whole_body(sign):
    heading = sign*math.radians(2)
    rows = [(sign*.010, .53, heading)]*5
    assert straight_entry_heading_budget(rows,.16,.015,.22,.02,math.radians(3),
                                         -.2295,known_width=.54) is None
    corridor = path(rows)
    assert corridor is not None
    for progress in (0., .04, .08, .16):
        assert corridor.clearance(heading, progress, .16-progress) >= .02
        for future in (0., (.16-progress)/2, .16-progress):
            for x in (-.2295, 0., .2295):
                for y in (-.22, .22):
                    for delta in (-math.radians(.1), 0., math.radians(.1)):
                        angle = heading+delta
                        corner = ((progress+future-x)*math.sin(angle)
                                  + y*math.cos(angle)-rows[0][0])
                        assert .265-abs(corner)-.003 >= .02
    # Reversing with the opposite tilt leads away from the center.
    assert path([(sign*.010,.53,-heading)]*5) is None


@pytest.mark.parametrize('fault', ['few','side','heading','width','jitter','nan','unmeasured'])
def test_signed_path_requires_five_safe_stable_observations(fault):
    rows = [(-.010,.53,math.radians(-2))]*5
    if fault == 'few':rows.pop()
    if fault == 'side':rows = [(-.04,.53,math.radians(-2))]*5
    if fault == 'heading':rows = [(-.010,.53,math.radians(-3.1))]*5
    if fault == 'width':rows[-1] = (-.010,.49,math.radians(-2))
    if fault == 'jitter':rows[-1] = (.010,.53,math.radians(-2))
    if fault == 'nan':rows[-1] = (math.nan,.53,math.radians(-2))
    if fault == 'unmeasured':
        assert straight_entry_path(rows,.16,.22,.02,math.radians(3),-.2295,0.,.01,
                                   math.radians(-2)) is None
    else:
        assert path(rows) is None


@pytest.mark.parametrize('seed', [7,42,123])
def test_trial91_can_verify_before_guide_visibility_loss(seed):
    data = json.loads((Path(__file__).parent/'fixtures/straight_entry_trial91.json').read_text())
    values = {}
    node = S(declare_parameter=lambda k,v:values.setdefault(k,v),
             get_parameter=lambda k:S(value=values[k]))
    LidarPlaneAligner.declare_parameters(node)
    values.update(yaml.safe_load((Path(__file__).parents[1]/'config/docking.yaml').read_text())['dock_turn_backup']['ros__parameters'])
    values['backup_rear_reference_x'] = -.2295
    t = data['transform']
    transform = PlanarTransform(t['x'],t['y'],yaw_from_quaternion(*t['q']))
    observations, accepted, old = [], [], []
    saved = random.getstate()
    try:
        random.seed(seed)
        for row in data['scans']:
            def project(snapshot,center,width,minimum,maximum):
                return project_scan_to_base(
                    [v if v is not None else math.inf for v in row['ranges']],
                    row['angle_min'],row['angle_increment'],row['range_min'],row['range_max'],
                    transform,center,width,minimum,maximum)
            aligner = LidarPlaneAligner(node,None,S(project=project))
            snapshot = S(sequence=1)
            wall, guide = aligner.estimate_error(snapshot), aligner.estimate_guide_center(snapshot)
            if wall is None or guide is None:
                observations = []
                continue
            gyro = row['gyro']
            observations.append((guide.center_offset,guide.separation,wall[0],gyro))
            observations = observations[-5:]
            rows = [(c,w,a-(gyro-g)) for c,w,a,g in observations]
            distance = min(v for _,v in rear_clearances(
                project(snapshot,math.pi,math.radians(20),.05,1.2),-.2295))
            corridor = path(rows,distance,wall[0])
            if corridor is not None:accepted.append(distance)
            if straight_entry_heading_budget(rows,distance,.015,.22,.02,math.radians(3),
                                               -.2295,known_width=.54) is not None:old.append(distance)
        assert not old
        assert accepted and .15 < accepted[0] < .21
    finally:
        random.setstate(saved)


@pytest.mark.parametrize('sign', [-1,1])
@pytest.mark.parametrize('fault', [None,'imu_loss','yaw_drift','side_motion','overrun'])
def test_signed_handoff_keeps_zero_steering_and_independent_guards(simulation,sign,fault):
    motion,state = simulation
    motion.node.parameters.update(yaml.safe_load(
        (Path(__file__).parents[1]/'config/docking.yaml').read_text())['dock_turn_backup']['ros__parameters'])
    state.gap,state.yaw = .16, sign*math.radians(2)
    sync_initial_pose(motion,state)
    # World station normal is zero; guide center is 10mm opposite the robot
    # yaw. Raw fit center is at the base origin in the guide frame.
    motion.backup_lidar_guide_estimator = lambda _: (
        None if state.gap+state.x < .15 else S(
            center_offset=-sign*.010-state.y,separation=.53,
            rear_center_offset=-sign*.010-state.y+.2295*math.sin(state.yaw)))
    if fault == 'imu_loss':motion.imu_is_fresh = lambda:state.gap+state.x >= .12
    if fault == 'yaw_drift':motion.current_integrated_imu_yaw = lambda:(
        state.yaw+math.radians(4) if state.gap+state.x < .12 else state.yaw)
    if fault == 'side_motion':
        original = motion.current_xy
        motion.current_xy = lambda:(original()[0],original()[1]+(
            .04 if state.gap+state.x < .12 else 0.))
    if fault == 'overrun':motion._rear_clearances = lambda _: [
        (i,.002 if state.gap+state.x < .12 else state.gap+state.x) for i in range(12)]
    assert motion.backup_with_lidar() is (fault is None),motion.node.get_logger().error.call_args
    assert state.linear == state.angular == 0
    assert any('corridor_check=signed_rectangle' in str(c)
               for c in motion.node.get_logger().info.call_args_list)
    if fault is None:
        assert state.gap+state.x <= .034
        assert all(w == 0 for _,x,_,v,w in state.sent if v < 0 and state.gap+x < .15)
