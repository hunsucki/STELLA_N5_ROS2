import math

from docking.motion import MotionController
import pytest
from sensor_msgs.msg import Imu


def test_absolute_odom_spin_handles_yaw_wraparound_and_stability():
    controller = MotionController.__new__(MotionController)
    controller.odom_sequence = 0
    target = math.radians(-91.0)
    samples = iter([
        math.radians(170.0),
        math.radians(-93.5),
        math.radians(-92.0),
        math.radians(-91.5),
        math.radians(-91.2),
    ])
    controller.current_yaw = lambda: next(samples)
    controller.current_imu_yaw_rate = lambda: 0.0
    controller.imu_is_fresh = lambda: True

    done = controller._make_absolute_spin_done_cb(
        target,
        math.radians(2.0),
        stable_cycles=3,
        direction=1.0,
        stationary_yaw_rate=math.radians(0.5),
    )

    for _ in range(4):
        controller.odom_sequence += 1
        assert not done()
    controller.odom_sequence += 1
    assert done()


def test_absolute_odom_spin_waits_until_imu_reports_stationary():
    controller = MotionController.__new__(MotionController)
    controller.odom_sequence = 0
    controller.current_yaw = lambda: 0.0
    yaw_rates = iter([math.radians(2.0), math.radians(0.2)])
    controller.current_imu_yaw_rate = lambda: next(yaw_rates)
    controller.imu_is_fresh = lambda: True
    done = controller._make_absolute_spin_done_cb(
        0.0,
        math.radians(1.0),
        stable_cycles=1,
        direction=1.0,
        stationary_yaw_rate=math.radians(0.5),
    )

    controller.odom_sequence += 1
    assert not done()
    controller.odom_sequence += 1
    assert done()


def test_absolute_odom_spin_uses_wheel_rate_during_imu_dropout():
    controller = MotionController.__new__(MotionController)
    controller.odom_sequence = 0
    controller.current_yaw = lambda: 0.0
    controller.imu_is_fresh = lambda: False
    wheel_rates = iter([math.radians(2.0), math.radians(0.2)])
    controller.current_odom_yaw_rate = lambda: next(wheel_rates)
    done = controller._make_absolute_spin_done_cb(
        0.0,
        math.radians(1.0),
        stable_cycles=1,
        direction=1.0,
        stationary_yaw_rate=math.radians(0.5),
    )

    controller.odom_sequence += 1
    assert not done()
    controller.odom_sequence += 1
    assert done()


def test_absolute_odom_spin_uses_requested_direction_at_180_degree_tie():
    assert MotionController._absolute_yaw_error(
        -math.pi / 2.0, math.pi / 2.0, 1.0) == pytest.approx(math.pi)
    assert MotionController._absolute_yaw_error(
        -math.pi / 2.0, math.pi / 2.0, -1.0) == pytest.approx(-math.pi)


def test_absolute_odom_spin_slows_down_and_corrects_small_overshoot():
    controller = MotionController.__new__(MotionController)
    target = 0.0
    current = math.radians(-15.0)
    controller.current_yaw = lambda: current

    slowed = controller._spin_angular_velocity(
        target, 1.0, math.radians(2.0), math.radians(30.0), 0.04, 0.15)
    assert 0.04 < slowed < 0.15

    current = math.radians(3.0)
    correction = controller._spin_angular_velocity(
        target, 1.0, math.radians(2.0), math.radians(30.0), 0.04, 0.15)
    assert correction < 0.0


def test_backup_speed_slows_near_rear_target():
    assert MotionController._backup_speed_for_clearance(
        0.20, 0.01, 0.15, 0.015, 0.05) == 0.05
    speed = MotionController._backup_speed_for_clearance(
        0.08, 0.01, 0.15, 0.015, 0.05)
    assert 0.015 < speed < 0.05


def test_backup_heading_controller_uses_heading_error_and_imu_damping():
    tolerance = math.radians(1.0)

    assert MotionController._backup_heading_angular_velocity(
        math.radians(0.4), 0.0, tolerance, 0.35, 0.10, 0.015, 1.0) == 0.0
    assert MotionController._backup_heading_angular_velocity(
        math.radians(2.0), 0.0, tolerance, 0.35, 0.10, 0.015, 1.0) > 0.0
    assert MotionController._backup_heading_angular_velocity(
        math.radians(-2.0), 0.0, tolerance, 0.35, 0.10, 0.015, 1.0) < 0.0
    assert MotionController._backup_heading_angular_velocity(
        math.radians(20.0), 0.0, tolerance, 0.35, 0.10, 0.015, 1.0) == 0.015
    assert MotionController._backup_heading_angular_velocity(
        0.0, math.radians(5.0), tolerance, 0.35, 0.10, 0.015, 1.0) < 0.0


def test_backup_heading_command_rate_is_limited_in_both_directions():
    assert MotionController._limit_command_rate(0.0, 0.015, 0.003) == 0.003
    assert MotionController._limit_command_rate(0.0, -0.015, 0.003) == -0.003
    assert MotionController._limit_command_rate(0.01, -0.01, 0.004) == 0.006


def test_lidar_heading_filter_smooths_small_changes_and_rejects_jumps():
    update = MotionController._filter_lidar_heading_sample
    current, accepted = update(
        None, math.radians(2.0), 0.15, math.radians(2.5))
    assert accepted
    assert current == pytest.approx(math.radians(2.0))

    current, accepted = update(
        current, math.radians(1.0), 0.15, math.radians(2.5))
    assert accepted
    assert current == pytest.approx(math.radians(1.85))

    unchanged, accepted = update(
        current, math.radians(-2.0), 0.15, math.radians(2.5))
    assert not accepted
    assert unchanged == pytest.approx(current)


def test_fixed_wall_motion_residual_rejects_plane_that_ignores_robot_yaw():
    residual = MotionController._lidar_heading_motion_residual
    anchor_error = math.radians(0.3)
    anchor_yaw = math.radians(10.0)

    # A fixed rear wall appears to rotate by the opposite amount.
    assert residual(
        math.radians(-4.7), anchor_error,
        math.radians(15.0), anchor_yaw) == pytest.approx(0.0)

    # This was the failed-run pattern: robot yaw changed, but the fitted
    # surface stayed almost parallel and was therefore the wrong authority.
    assert residual(
        math.radians(-0.7), anchor_error,
        math.radians(15.0), anchor_yaw) == pytest.approx(math.radians(4.0))


def test_motion_consistency_accepts_either_inertial_or_wheel_agreement():
    consistent = MotionController._lidar_heading_motion_is_consistent
    limit = math.radians(2.0)

    # All three disagree with the fitted plane: reject it.
    assert not consistent(
        math.radians(4.3), math.radians(3.2), math.radians(3.0), limit)
    # IMU/odom are correlated and may drift together. LiDAR plus encoders win.
    assert consistent(
        math.radians(4.3), math.radians(3.2), math.radians(0.6), limit)
    # Physical wheel slip is visible to LiDAR and inertial yaw, but not wheels.
    assert consistent(
        math.radians(0.4), math.radians(1.0), math.radians(-5.0), limit)


def test_september_15_backup_trace_accepts_reacquired_slip_plane():
    residual = MotionController._lidar_heading_motion_residual
    consistent = MotionController._lidar_heading_motion_is_consistent
    anchor = math.radians(0.28)
    limit = math.radians(2.0)

    stale_imu = residual(
        math.radians(-0.68), anchor, math.radians(5.25), 0.0)
    stale_odom = residual(
        math.radians(-0.68), anchor, math.radians(4.14), 0.0)
    stale_wheel = residual(
        math.radians(-0.68), anchor, math.radians(0.01), 0.0)
    # This sample is ambiguous: LiDAR and wheels agree, while the correlated
    # IMU/odom group disagrees. It remains valid until a real plane jump makes
    # the heading filter stop and reacquire.
    assert consistent(stale_imu, stale_odom, stale_wheel, limit)

    reacquired_imu = residual(
        math.radians(-5.57), anchor, math.radians(5.25), 0.0)
    reacquired_odom = residual(
        math.radians(-5.57), anchor, math.radians(4.14), 0.0)
    reacquired_wheel = residual(
        math.radians(-5.57), anchor, math.radians(0.01), 0.0)
    assert consistent(
        reacquired_imu, reacquired_odom, reacquired_wheel, limit)


def test_september_17_straight_backup_ignores_correlated_imu_bias():
    residual = MotionController._lidar_heading_motion_residual
    consistent = MotionController._lidar_heading_motion_is_consistent
    anchor = math.radians(0.67)
    limit = math.radians(2.0)

    candidate = math.radians(1.32)
    imu = residual(candidate, anchor, math.radians(1.52), 0.0)
    odom = residual(candidate, anchor, math.radians(1.44), 0.0)
    wheel = residual(candidate, anchor, math.radians(-0.05), 0.0)

    assert math.degrees(imu) == pytest.approx(2.17)
    assert math.degrees(odom) == pytest.approx(2.09)
    assert math.degrees(wheel) == pytest.approx(0.60)
    assert consistent(imu, odom, wheel, limit)


def test_wheel_heading_guard_is_bounded_around_latest_failure():
    allowed = MotionController._backup_wheel_guard_is_allowed

    assert allowed(
        math.radians(2.99), math.radians(0.13),
        math.radians(4.0), math.radians(8.0))
    assert not allowed(
        math.radians(4.01), math.radians(0.13),
        math.radians(4.0), math.radians(8.0))
    assert not allowed(
        math.radians(2.99), math.radians(8.0),
        math.radians(4.0), math.radians(8.0))


def test_failed_run_heading_sample_now_commands_the_correct_direction():
    command = MotionController._backup_heading_angular_velocity(
        math.radians(-2.24),
        math.radians(4.0),
        math.radians(1.0),
        0.35,
        0.10,
        0.015,
        1.0,
    )
    assert command == -0.015


def test_guide_center_heading_steers_reverse_path_toward_opening_center():
    heading = MotionController._backup_guide_center_heading(
        center_offset=0.10,
        tolerance=0.015,
        kp=0.35,
        max_heading=math.radians(2.0),
    )
    assert heading == pytest.approx(-math.radians(2.0))
    assert MotionController._backup_guide_center_heading(
        -0.05, 0.015, 0.35, math.radians(2.0)) > 0.0
    assert MotionController._backup_guide_center_heading(
        0.01, 0.015, 0.35, math.radians(2.0)) == 0.0


def test_valid_lidar_plane_overrides_dead_reckoning_drift_abort():
    exceeded = MotionController._backup_dead_reckoning_drift_exceeded
    limit = math.radians(5.0)

    assert not exceeded('lidar', math.radians(8.0), limit)
    assert exceeded('imu_gyro', math.radians(5.1), limit)
    assert exceeded('wheel_odom', math.radians(-5.1), limit)


def test_backup_pauses_for_large_lidar_error_and_resumes_after_stability():
    update = MotionController._update_backup_heading_correction
    tolerance = math.radians(1.0)
    pause_error = math.radians(2.0)
    stationary = math.radians(0.5)

    active, count = update(
        False, 0, True, math.radians(-2.1), 0.0,
        tolerance, pause_error, stationary, 3)
    assert active and count == 0

    for expected_count in (1, 2):
        active, count = update(
            active, count, True, math.radians(0.5), math.radians(0.1),
            tolerance, pause_error, stationary, 3)
        assert active and count == expected_count

    active, count = update(
        active, count, True, math.radians(0.5), math.radians(0.1),
        tolerance, pause_error, stationary, 3)
    assert not active and count == 0


def test_backup_does_not_resume_without_lidar_or_while_still_rotating():
    update = MotionController._update_backup_heading_correction
    common = (
        math.radians(1.0), math.radians(2.0), math.radians(0.5), 3)

    assert update(
        True, 2, False, 0.0, 0.0, *common) == (True, 0)
    assert update(
        True, 2, True, 0.0, math.radians(1.0), *common) == (True, 0)


def test_imu_yaw_rate_is_integrated_with_sensor_timestamps():
    from types import SimpleNamespace

    controller = MotionController.__new__(MotionController)
    controller.node = SimpleNamespace(
        get_parameter=lambda name: SimpleNamespace(value={
            'imu_max_age_sec': 0.25,
            'motion_sensor_future_tolerance_sec': 0.05,
        }[name]),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=10_020_000_000)),
    )
    controller.last_imu_yaw_rate = None
    controller.last_imu_received_at = 0.0
    controller.last_imu_stamp_nanoseconds = 0
    controller.integrated_imu_yaw = 0.0

    first = Imu()
    first.header.stamp.sec = 10
    first.angular_velocity.z = 1.0
    second = Imu()
    second.header.stamp.sec = 10
    second.header.stamp.nanosec = 20_000_000
    second.angular_velocity.z = 1.0

    controller._imu_callback(first)
    controller._imu_callback(second)

    assert controller.current_integrated_imu_yaw() == pytest.approx(0.02)


def test_logged_large_tag_refinement_converges_within_safety_envelope():
    """Regression for x=0.126m, y=0.041m, yaw=-17.63deg failure."""
    from docking.dock_turn_backup import DockTurnBackup

    current_x = 0.0
    current_y = 0.0
    current_yaw = 0.0
    target_x = 0.126
    target_y = 0.041
    target_yaw = math.radians(-17.63)
    dt = 0.05
    traveled = 0.0
    max_yaw_excursion = 0.0
    converged = False

    for _ in range(int(18.0 / dt)):
        longitudinal, lateral, yaw_error = (
            DockTurnBackup._fixed_goal_errors_in_target(
                target_x, target_y, target_yaw,
                current_x, current_y, current_yaw))
        base_longitudinal, base_lateral, _ = (
            DockTurnBackup._fixed_goal_errors_in_base(
                target_x, target_y, target_yaw,
                current_x, current_y, current_yaw))
        within_position = (
            abs(longitudinal) <= 0.04 and abs(lateral) <= 0.025)
        if within_position and abs(yaw_error) <= math.radians(2.0):
            converged = True
            break

        linear, angular = DockTurnBackup._tag_refinement_command(
            base_longitudinal,
            base_lateral,
            yaw_error,
            within_position,
            0.5,
            1.0,
            -0.3,
            1.0,
            0.025,
            0.08,
        )
        current_x += linear * math.cos(current_yaw) * dt
        current_y += linear * math.sin(current_yaw) * dt
        current_yaw = math.atan2(
            math.sin(current_yaw + angular * dt),
            math.cos(current_yaw + angular * dt),
        )
        traveled += abs(linear) * dt
        max_yaw_excursion = max(max_yaw_excursion, abs(current_yaw))

    assert converged
    assert traveled <= 0.18
    assert max_yaw_excursion <= math.radians(30.0)
