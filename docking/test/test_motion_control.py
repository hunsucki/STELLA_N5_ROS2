import math

from docking.motion import MotionController
import pytest
from sensor_msgs.msg import Imu


def test_absolute_odom_spin_handles_yaw_wraparound_and_stability():
    controller = MotionController.__new__(MotionController)
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

    assert not done()
    assert not done()
    assert not done()
    assert not done()
    assert done()


def test_absolute_odom_spin_waits_until_imu_reports_stationary():
    controller = MotionController.__new__(MotionController)
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

    assert not done()
    assert done()


def test_absolute_odom_spin_uses_wheel_rate_during_imu_dropout():
    controller = MotionController.__new__(MotionController)
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

    assert not done()
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
    controller = MotionController.__new__(MotionController)
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
