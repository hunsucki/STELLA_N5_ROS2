import math

from docking.lidar_geometry import (
    effective_range_limits,
    has_consecutive_clearance_cluster,
    header_stamp_is_acceptable,
    line_orientation_error,
    plane_normal_alignment_error,
    PlanarTransform,
    project_scan_to_base,
    rear_clearances,
    required_rear_range_at_completion,
    scan_is_fresh,
    UniqueScanStability,
)
import pytest


SENSOR_X = -0.166
REAR_REFERENCE_X = -0.2295
TARGET_CLEARANCE = 0.01
TOLERANCE = 0.005
MOUNT = PlanarTransform(SENSOR_X, 0.0, math.pi)


def make_scan(default=math.inf):
    return [default] * 361, -math.pi, math.radians(1.0)


def project(ranges, angle_min, angle_increment):
    return project_scan_to_base(
        ranges,
        angle_min,
        angle_increment,
        0.05,
        12.0,
        MOUNT,
        math.pi,
        math.radians(20.0),
        0.05,
        2.0,
    )


def test_new_mount_rear_clearance_at_target():
    ranges, angle_min, angle_increment = make_scan()
    ranges[180] = 0.0735

    points = project(ranges, angle_min, angle_increment)

    assert points is not None
    assert len(points) == 1
    assert points[0].x == pytest.approx(-0.2395)
    assert points[0].y == pytest.approx(0.0, abs=1e-9)
    assert rear_clearances(points, REAR_REFERENCE_X)[0][1] == pytest.approx(
        TARGET_CLEARANCE)


def test_tf_projection_corrects_off_axis_wall_range():
    ranges, angle_min, angle_increment = make_scan()
    wall_x = REAR_REFERENCE_X - 0.10
    local_angle = math.radians(8.0)
    beam_index = 188
    ranges[beam_index] = (SENSOR_X - wall_x) / math.cos(local_angle)

    points = project(ranges, angle_min, angle_increment)

    assert points is not None
    assert rear_clearances(points, REAR_REFERENCE_X)[0][1] == pytest.approx(0.10)


def test_rear_60_degree_safety_sector_excludes_old_side_rail_return():
    # Field scan that stopped backup with the former +/-75-degree safety fan.
    old_edge_angle = math.radians(74.85)
    points = project_scan_to_base(
        [0.2412],
        old_edge_angle,
        math.radians(0.5),
        0.05,
        12.0,
        MOUNT,
        math.pi,
        math.radians(60.0),
        0.05,
        2.0,
    )

    assert points == []


def test_completion_range_is_inside_driver_range():
    completion_range = required_rear_range_at_completion(
        SENSOR_X,
        REAR_REFERENCE_X,
        TARGET_CLEARANCE,
        TOLERANCE,
    )
    limits = effective_range_limits(0.05, 12.0, 0.05, 2.0)

    assert completion_range == pytest.approx(0.0785)
    assert limits is not None
    assert limits[0] <= completion_range <= limits[1]


def test_range_limits_reject_old_unreachable_minimum():
    completion_range = required_rear_range_at_completion(
        SENSOR_X,
        REAR_REFERENCE_X,
        TARGET_CLEARANCE,
        TOLERANCE,
    )
    old_limits = effective_range_limits(0.05, 12.0, 0.15, 2.0)

    assert old_limits is not None
    assert old_limits[0] > completion_range


def test_completion_requires_neighboring_beams():
    isolated = [(100, 0.01), (105, 0.01), (110, 0.01)]
    cluster = [(100, 0.01), (101, 0.012), (102, 0.014)]

    assert not has_consecutive_clearance_cluster(isolated, 0.015, 3)
    assert has_consecutive_clearance_cluster(cluster, 0.015, 3)


def test_completion_rejects_negative_clearance_and_insufficient_span():
    self_returns = [(100, -0.0135), (101, -0.0135), (102, -0.0135)]
    narrow = [(100, 0.01), (101, 0.01), (102, 0.01), (103, 0.01)]

    assert not has_consecutive_clearance_cluster(
        self_returns, 0.015, 3, minimum=0.005)
    assert not has_consecutive_clearance_cluster(
        narrow, 0.015, 3, minimum=0.005, minimum_beam_span=4)


def test_flat_rear_wall_meets_completion_cluster_contract():
    ranges, angle_min, angle_increment = make_scan()
    wall_x = REAR_REFERENCE_X - TARGET_CLEARANCE
    for beam_index in range(170, 191):
        local_angle = angle_min + beam_index * angle_increment
        ranges[beam_index] = (SENSOR_X - wall_x) / math.cos(local_angle)

    points = project(ranges, angle_min, angle_increment)
    indexed = rear_clearances(points, REAR_REFERENCE_X)

    assert has_consecutive_clearance_cluster(
        indexed,
        TARGET_CLEARANCE + TOLERANCE,
        required_points=5,
        minimum=TARGET_CLEARANCE - TOLERANCE,
        minimum_beam_span=3,
    )


def test_stability_counts_unique_scans_only():
    stability = UniqueScanStability(required=3, initial_sequence=10)

    assert not stability.observe(11, True)
    assert not stability.observe(11, True)
    assert stability.count == 1
    assert not stability.observe(12, True)
    assert stability.observe(13, True)


def test_unstable_new_scan_resets_stability():
    stability = UniqueScanStability(required=2)

    assert not stability.observe(1, True)
    assert not stability.observe(2, False)
    assert stability.count == 0
    assert not stability.observe(3, True)
    assert stability.observe(4, True)


def test_scan_freshness_is_fail_closed():
    assert scan_is_fresh(10.0, 10.25, 0.30)
    assert not scan_is_fresh(10.0, 10.31, 0.30)
    assert not scan_is_fresh(0.0, 0.1, 0.30)
    assert not scan_is_fresh(10.0, 9.0, 0.30)


def test_header_stamp_must_be_new_and_fresh():
    assert header_stamp_is_acceptable(
        9_900_000_000, 9_800_000_000, 10_000_000_000,
        500_000_000, 50_000_000)
    assert not header_stamp_is_acceptable(
        9_900_000_000, 9_900_000_000, 10_000_000_000,
        500_000_000, 50_000_000)
    assert not header_stamp_is_acceptable(
        9_000_000_000, 8_900_000_000, 10_000_000_000,
        500_000_000, 50_000_000)
    assert not header_stamp_is_acceptable(
        10_100_000_000, 9_900_000_000, 10_000_000_000,
        500_000_000, 50_000_000)


def test_alignment_line_error_has_correct_sign():
    target = math.pi / 2.0

    assert math.degrees(line_orientation_error(
        target - math.radians(5.0), target)) == pytest.approx(-5.0)
    assert math.degrees(line_orientation_error(
        target + math.radians(5.0), target)) == pytest.approx(5.0)
    assert line_orientation_error(target + math.pi, target) == pytest.approx(0.0)


def test_plane_normal_error_matches_rear_and_side_station_surfaces():
    yaw_error = math.radians(4.0)

    assert plane_normal_alignment_error(
        math.pi / 2.0 + yaw_error, 0.0) == pytest.approx(yaw_error)
    assert plane_normal_alignment_error(
        yaw_error, math.pi / 2.0) == pytest.approx(yaw_error)
    assert plane_normal_alignment_error(
        math.pi + yaw_error, math.pi / 2.0) == pytest.approx(yaw_error)


@pytest.mark.parametrize(
    'ranges,angle_min,angle_increment,scan_min,scan_max',
    [
        ([], -math.pi, 0.01, 0.05, 12.0),
        ([1.0], math.nan, 0.01, 0.05, 12.0),
        ([1.0], -math.pi, 0.0, 0.05, 12.0),
        ([1.0], -math.pi, 0.01, 0.5, 0.1),
    ],
)
def test_malformed_scan_is_rejected(
        ranges, angle_min, angle_increment, scan_min, scan_max):
    assert project_scan_to_base(
        ranges,
        angle_min,
        angle_increment,
        scan_min,
        scan_max,
        MOUNT,
        math.pi,
        math.radians(20.0),
        0.05,
        2.0,
    ) is None


def test_nonfinite_configured_range_is_rejected():
    assert effective_range_limits(0.05, 12.0, math.nan, 2.0) is None
    assert effective_range_limits(0.05, 12.0, 0.05, math.inf) is None
