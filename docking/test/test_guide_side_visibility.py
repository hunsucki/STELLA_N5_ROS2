"""Replay the physical stop where the rear fan discarded visible rail tips."""
import json
import math
from pathlib import Path
import random
from types import SimpleNamespace

import pytest

from docking.lidar_alignment import LidarPlaneAligner
from docking.lidar_geometry import (
    PlanarTransform, project_scan_to_base, yaw_from_quaternion,
)


@pytest.mark.parametrize('seed', [7, 42, 123])
@pytest.mark.parametrize('scan_index', [0, 1, 2])
def test_recorded_side_returns_restore_guide_detection(seed, scan_index):
    data = json.loads((Path(__file__).parent / 'fixtures' /
                       'guide_side_visibility_trial13.json').read_text())
    values = {}
    node = SimpleNamespace(
        declare_parameter=lambda n, v: values.setdefault(n, v),
        get_parameter=lambda n: SimpleNamespace(value=values[n]))
    LidarPlaneAligner.declare_parameters(node)
    values.update(backup_rear_reference_x=-.2295,
                  lidar_guide_min_separation=.50, lidar_guide_max_separation=.58)
    m = data['transform']
    transform = PlanarTransform(m['x'], m['y'], yaw_from_quaternion(*m['q']))
    row = data['scans'][scan_index]

    def project(snapshot, center, width, minimum, maximum):
        return project_scan_to_base(
            [v if v is not None else math.inf for v in row['ranges']],
            row['angle_min'], row['angle_increment'], row['range_min'], row['range_max'],
            transform, center, width, minimum, maximum)

    aligner = LidarPlaneAligner(node, None, SimpleNamespace(project=project))
    snapshot = SimpleNamespace(sequence=1)
    rng = random.getstate()
    try:
        random.seed(seed)
        estimate = aligner.estimate_guide_center(snapshot)
        assert estimate is not None
        assert abs(estimate.center_offset) < .01
        assert .50 <= estimate.separation <= .58
        assert min(estimate.left_length, estimate.right_length) >= .15
        assert min(estimate.left_inliers, estimate.right_inliers) >= 8
        # The original fan fails on the very same scan and wall estimate.
        values['lidar_guide_sector_width'] = math.radians(150)
        random.seed(seed)
        assert aligner.estimate_guide_center(snapshot) is None
    finally:
        random.setstate(rng)
