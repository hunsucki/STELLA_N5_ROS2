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


def budget(rows, distance=.32, center_limit=.01, known_width=0.):
    return straight_entry_heading_budget(rows, distance, center_limit, .22, .02,
                                         math.radians(1), -.2295, known_width=known_width)


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
    elif fault == 'center':rows=[(.016,.514,0.)]*5
    elif fault == 'width':rows=[(.003,.49,0.)]*5
    else:rows=[(.003,.514,math.radians(2))]*5
    assert budget(rows,.16,.015,known_width=.54) is None


@pytest.mark.parametrize('seed', [7, 42, 123])
@pytest.mark.parametrize('fixture,center_limit,minimum_distance', [
    ('straight_entry_trial22.json', .01, .28),
    ('straight_entry_trial42.json', .015, .14),
])
def test_recorded_corridor_can_commit_before_near_guide_loss(
        seed, fixture, center_limit, minimum_distance):
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
                           known_width=.54 if fixture == 'straight_entry_trial42.json' else 0.)
            if limit is not None and distance <= .32:
                accepted.append((distance, limit))
        assert accepted, 'Recorded stable corridor must support an automatic handoff'
        assert minimum_distance < accepted[0][0] <= .32
    finally:
        random.setstate(rng)
