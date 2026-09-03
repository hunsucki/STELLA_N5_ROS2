import math
import random

from docking.lidar_alignment import LidarPlaneAligner
import pytest


class _Parameter:
    def __init__(self, value):
        self.value = value


class _Node:
    def __init__(self, values):
        self.values = values

    def get_parameter(self, name):
        return _Parameter(self.values[name])


def test_ransac_prefers_long_plane_over_denser_short_frame_edge():
    aligner = LidarPlaneAligner.__new__(LidarPlaneAligner)
    aligner.node = _Node({
        'lidar_align_ransac_iterations': 500,
        'lidar_align_ransac_threshold': 0.001,
        'lidar_align_target_line_angle': math.pi / 2.0,
        'lidar_align_candidate_max_error': math.radians(15.0),
    })
    long_plane = [(-1.0, -0.5 + 0.05 * index) for index in range(21)]
    short_edge = [(-1.3 + 0.02 * index, 0.8) for index in range(31)]

    random_state = random.getstate()
    random.seed(7)
    try:
        inliers = aligner._ransac_line_inliers(long_plane + short_edge)
    finally:
        random.setstate(random_state)

    line = aligner._fit_line_pca(inliers)
    assert line is not None
    line_angle, line_length = line
    assert abs(abs(line_angle) - math.pi / 2.0) < math.radians(0.1)
    assert line_length > 1.0


def test_plane_tracking_accepts_same_static_plane_during_robot_rotation():
    residual = LidarPlaneAligner._plane_tracking_residual(
        previous_error=math.radians(-5.0),
        previous_yaw=math.radians(10.0),
        current_error=math.radians(-2.2),
        current_yaw=math.radians(7.0),
    )

    assert residual == pytest.approx(math.radians(-0.2))


def test_plane_tracking_rejects_failed_run_surface_jump():
    residual = LidarPlaneAligner._plane_tracking_residual(
        previous_error=math.radians(-5.03),
        previous_yaw=0.0,
        current_error=math.radians(-11.14),
        current_yaw=math.radians(-3.6),
    )

    assert abs(residual) > math.radians(5.0)


def test_plane_acquisition_discards_first_outlier_then_requires_consensus():
    update = LidarPlaneAligner._update_plane_acquisition
    error = yaw = None
    count = 0
    required = 3
    tolerance = math.radians(3.0)

    error, yaw, count, acquired = update(
        error, yaw, count, math.radians(13.54), 0.0,
        tolerance, required)
    assert count == 1
    assert not acquired

    # This is the real failed-run transition.  It must reset acquisition,
    # rather than steering from the first 13.54-degree fit.
    error, yaw, count, acquired = update(
        error, yaw, count, math.radians(6.22), math.radians(2.18),
        tolerance, required)
    assert count == 1
    assert not acquired

    for sample in (6.0, 5.8):
        error, yaw, count, acquired = update(
            error, yaw, count, math.radians(sample), math.radians(2.18),
            tolerance, required)

    assert count == 3
    assert acquired


def test_plane_acquisition_compensates_for_robot_rotation():
    update = LidarPlaneAligner._update_plane_acquisition
    error, yaw, count, acquired = update(
        None, None, 0, math.radians(8.0), math.radians(2.0),
        math.radians(1.0), 2)
    assert not acquired

    error, yaw, count, acquired = update(
        error, yaw, count, math.radians(6.0), math.radians(4.0),
        math.radians(1.0), 2)
    assert count == 2
    assert acquired


def test_failed_run_tracking_jump_reacquires_instead_of_aborting():
    decide = LidarPlaneAligner._tracking_jump_action

    assert decide(
        math.radians(-5.14),
        math.radians(5.0),
        math.radians(12.0),
    ) == 'reacquire'
    assert decide(
        math.radians(4.9),
        math.radians(5.0),
        math.radians(12.0),
    ) == 'accept'
    assert decide(
        math.radians(12.1),
        math.radians(5.0),
        math.radians(12.0),
    ) == 'abort'


def _rotate_points(points, angle):
    cosine = math.cos(angle)
    sine = math.sin(angle)
    return [
        (cosine * x - sine * y, sine * x + cosine * y)
        for x, y in points
    ]


def test_guide_center_uses_both_parallel_rails_in_wall_aligned_frame():
    aligner = LidarPlaneAligner.__new__(LidarPlaneAligner)
    aligner.node = _Node({
        'lidar_guide_wall_x_margin': 0.03,
        'lidar_guide_max_x': -0.12,
        'lidar_guide_min_abs_y': 0.14,
        'lidar_guide_max_abs_y': 0.40,
        'lidar_guide_min_separation': 0.44,
        'lidar_guide_max_separation': 0.60,
        'lidar_guide_line_threshold': 0.022,
        'lidar_guide_orientation_tolerance': math.radians(12.0),
        'lidar_guide_min_inliers': 8,
        'lidar_guide_min_line_length': 0.15,
        'lidar_align_ransac_iterations': 200,
        'backup_rear_reference_x': -0.2295,
    })
    wall = [(-0.58, -0.30 + 0.02 * index) for index in range(31)]
    left = [(-0.56 + 0.02 * index, 0.28) for index in range(22)]
    right = [(-0.56 + 0.02 * index, -0.22) for index in range(22)]
    distractor = [(-0.40, -0.38 + 0.02 * index) for index in range(39)]
    wall_error = math.radians(7.0)
    observed_wall = _rotate_points(wall, wall_error)
    observed_points = _rotate_points(
        wall + left + right + distractor, wall_error)

    estimate = aligner._estimate_guide_center_from_points(
        wall_error, observed_wall, observed_points)

    assert estimate is not None
    assert estimate.center_offset == pytest.approx(0.03, abs=0.002)
    assert estimate.separation == pytest.approx(0.50, abs=0.002)
    assert estimate.left_inliers >= 20
    assert estimate.right_inliers >= 20
    assert estimate.left_line_angle == pytest.approx(0.0, abs=0.04)
    assert estimate.right_line_angle == pytest.approx(0.0, abs=0.04)


def test_guide_center_rejects_a_single_visible_rail():
    aligner = LidarPlaneAligner.__new__(LidarPlaneAligner)
    aligner.node = _Node({
        'lidar_guide_wall_x_margin': 0.03,
        'lidar_guide_max_x': -0.12,
        'lidar_guide_min_abs_y': 0.14,
        'lidar_guide_max_abs_y': 0.40,
        'lidar_guide_min_separation': 0.44,
        'lidar_guide_max_separation': 0.60,
        'lidar_guide_line_threshold': 0.022,
        'lidar_guide_orientation_tolerance': math.radians(12.0),
        'lidar_guide_min_inliers': 8,
        'lidar_guide_min_line_length': 0.15,
        'lidar_align_ransac_iterations': 200,
        'backup_rear_reference_x': -0.2295,
    })
    wall = [(-0.58, -0.30 + 0.02 * index) for index in range(31)]
    left = [(-0.56 + 0.02 * index, 0.28) for index in range(22)]

    assert aligner._estimate_guide_center_from_points(
        0.0, wall, wall + left) is None
