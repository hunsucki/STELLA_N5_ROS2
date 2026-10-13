"""Measured corridor acceptance, without claiming replay simulates physics."""
import json
import math
from pathlib import Path
import random
from types import SimpleNamespace

import pytest

from docking.control import straight_entry_heading_budget
from docking.lidar_alignment import LidarPlaneAligner
from docking.lidar_geometry import (
    PlanarTransform, project_scan_to_base, rear_clearances, yaw_from_quaternion,
)


def budget(rows, distance=.32, center_limit=.01, known_width=0., heading_limit_deg=1.):
    return straight_entry_heading_budget(rows, distance, center_limit, .22, .02,
                                         math.radians(heading_limit_deg), -.2295,
                                         known_width=known_width)


@pytest.mark.parametrize('fault', ['few', 'width', 'center', 'heading', 'unstable', 'nan'])
def test_unverified_corridors_cannot_commit(fault):
    rows = [(.001, .504, 0.)] * 5
    if fault == 'few':
        rows.pop()
    elif fault == 'width':
        rows[-1] = (.001, .482, 0.)
    elif fault == 'center':
        rows = [(.009, .504, 0.)] * 5
    elif fault == 'heading':
        rows = [(.001, .504, math.radians(1))] * 5
    elif fault == 'unstable':
        rows[-1] = (-.008, .504, 0.)
    else:
        rows[-1] = (math.nan, .504, 0.)
    assert budget(rows) is None


def test_heading_budget_preserves_center_and_body_side_margin():
    rows = [(.002, .504, math.radians(.2))] * 5
    limit = budget(rows)
    assert limit is not None
    center_bound = .002 + .003 + .32 * math.sin(limit)
    side_bound = center_bound + .459 * math.sin(limit)
    assert center_bound <= .01
    assert .504/2 - .22 - side_bound >= .02 - 1e-12


def test_measured_station_profile_preserves_full_body_side_margin_with_width_jitter():
    rows = [(c,w,math.radians(.2)) for c,w in zip(
        [.012,.007,.009,.004,.006], [.501,.514,.512,.519,.517])]
    limit = budget(rows,.16,.015,known_width=.54)
    assert limit is not None
    lateral_bound = .012+.003+(.16+.459)*math.sin(limit)
    assert (.54-.01)/2-.22-lateral_bound >= .02-1e-12
    assert budget(rows,.16,.015) is None


@pytest.mark.parametrize('fault', ['missing','center','width','heading'])
def test_known_width_cannot_replace_center_observations_or_heading_confirmation(fault):
    rows = [(.003,.514,0.)]*5
    if fault == 'missing':rows=[]
    elif fault == 'center':rows=[(.026,.514,0.)]*5
    elif fault == 'width':rows=[(.003,.49,0.)]*5
    else:rows=[(.003,.514,math.radians(2))]*5
    assert budget(rows,.16,.015,known_width=.54) is None


@pytest.mark.parametrize('seed', [7, 42, 123])
@pytest.mark.parametrize('fixture,center_limit,minimum_distance,max_distance,heading_limit_deg', [
    ('straight_entry_trial22.json', .01, .28, .32, 1.),
    ('straight_entry_trial42.json', .015, .14, .32, 1.),
    ('straight_entry_trial54.json', .015, .43, .50, 1.),
    ('straight_entry_trial55.json', .015, .37, .50, 3.),
    ('straight_entry_trial68.json', .015, .20, .50, 3.),
])
def test_recorded_corridor_can_commit_before_near_guide_loss(
        seed, fixture, center_limit, minimum_distance, max_distance, heading_limit_deg):
    data = json.loads((Path(__file__).parent / 'fixtures' /
                       fixture).read_text())
    values = {}
    node = SimpleNamespace(
        declare_parameter=lambda n, v: values.setdefault(n, v),
        get_parameter=lambda n: SimpleNamespace(value=values[n]))
    LidarPlaneAligner.declare_parameters(node)
    values.update(backup_rear_reference_x=-.2295,
                  lidar_guide_min_separation=.50, lidar_guide_max_separation=.58)
    m = data['transform']
    transform = PlanarTransform(m['x'], m['y'], yaw_from_quaternion(*m['q']))
    rng = random.getstate()
    observations = []
    accepted = []
    try:
        random.seed(seed)
        for row in data['scans']:
            def project(snapshot, center, width, minimum, maximum):
                return project_scan_to_base(
                    [v if v is not None else math.inf for v in row['ranges']],
                    row['angle_min'], row['angle_increment'], row['range_min'], row['range_max'],
                    transform, center, width, minimum, maximum)

            aligner = LidarPlaneAligner(node, None, SimpleNamespace(project=project))
            snapshot = SimpleNamespace(sequence=1)
            wall = aligner.estimate_error(snapshot)
            guide = aligner.estimate_guide_center(snapshot)
            if wall is None or guide is None:
                observations = []
                continue
            gyro = row.get('gyro', 0.)
            observations.append((guide.center_offset, guide.separation, wall[0], gyro))
            observations = observations[-5:]
            current_pose_rows = [(center,width,angle-(gyro-old_gyro))
                                 for center,width,angle,old_gyro in observations]
            distance = min(v for _, v in rear_clearances(
                project(snapshot, math.pi, math.radians(20), .05, 1.2), -.2295))
            limit = budget(current_pose_rows, distance, center_limit,
                           known_width=.54 if fixture != 'straight_entry_trial22.json' else 0.,
                           heading_limit_deg=heading_limit_deg)
            if limit is not None and distance <= max_distance:
                accepted.append((distance, limit))
        assert accepted, 'Recorded stable corridor must support an automatic handoff'
        assert minimum_distance < accepted[0][0] <= max_distance
    finally:
        random.setstate(rng)


def test_small_heading_drift_uses_physical_clearance_without_losing_side_margin():
    rows = [(c, .525, math.radians(-.87)) for c in [-.003, .004, -.0045, .003, .002]]
    old = budget(rows, .403, .015, known_width=.54)
    physical = budget(rows, .403, .015, known_width=.54, heading_limit_deg=3.)
    held_heading = math.radians(-.87-.15)
    assert old < abs(held_heading) < physical < math.radians(3)
    lateral_bound = .0045+.003+(.403+.459)*math.sin(physical)
    assert (.54-.01)/2-.22-lateral_bound >= .02-1e-12
    assert budget([(.015,.525,math.radians(-.87))]*5, .403, .015,
                  known_width=.54, heading_limit_deg=3.) is None


@pytest.mark.parametrize('sign', [-1, 1])
def test_measured_corridor_accepts_residual_above_steering_deadband(sign):
    rows = [(sign*.017, .53, math.radians(.1))]*5
    limit = budget(rows, .38, .015, known_width=.54, heading_limit_deg=3.)
    assert limit is not None
    worst_side = (.54-.01)/2-.22-(.017+.003+(.38+.459)*math.sin(limit))
    assert worst_side >= .02-1e-12
    assert budget(rows, .38, .015, known_width=0.) is None


@pytest.mark.parametrize('center,angle', [(.023, 0.), (.020, .5), (.017, 2.)])
def test_measured_corridor_still_rejects_insufficient_remaining_room(center, angle):
    assert budget([(center,.53,math.radians(angle))]*5,.38,.015,
                  known_width=.54,heading_limit_deg=3.) is None


@pytest.mark.parametrize('center', [-.017, 0., .017])
def test_measured_heading_limit_keeps_each_body_corner_inside_corridor(center):
    limit = budget([(center,.54,0.)]*5,.38,.015,
                   known_width=.54,heading_limit_deg=3.)
    assert limit is not None
    # Directly rotate and translate the rectangular body in the guide frame,
    # including 3 mm of lateral measurement error and either yaw direction.
    for angle in [-limit, 0., limit]:
        for travel in [0., .19, .38]:
            for x in [-.2295, .2295]:
                for y in [-.22, .22]:
                    for error in [-.003,.003]:
                        corner_y = (-(x-travel)*math.sin(angle)
                                    + y*math.cos(angle)-center+error)
                        assert (.54-.01)/2-abs(corner_y) >= .02-1e-12
